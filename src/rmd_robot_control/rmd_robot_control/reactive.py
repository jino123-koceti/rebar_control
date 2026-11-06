#!/usr/bin/env python3
"""반응형 순회 — "다음에 어디로 갈까" 를 **센서만 보고** 정한다. ROS 의존 없음.

## 왜 지도 기반을 버렸나

`coverage_map.plan_next()` 는 격자 지도 위에서 탐욕 순회를 한다. 잘 도는데,
**지도가 주행의 입력**이라는 게 문제였다. 아직 본 적 없는 자리를
`explore_ring=3` 이 **지어내서**(피치 3칸 ≈ 600mm) 프런티어로 쓰고, 로봇은
그 지어낸 곳으로 간다. 데크가 거기서 끝나도 지도는 모른다.

반응형은 순서를 뒤집는다 — **지도는 주행의 산출물**이다.

    ① 탐색   센서만 본다. 검출 피치로 스텝, 막히면 횡, 또 막히면 끝
    ② 기록   정차마다 지도에 적는다 (`coverage_map.observe`)
    ③ 수거   ①이 끝나면 지도는 **실측된 지도**다. 그때 탐욕으로 남은 것만 순회

③에서 탐욕을 다시 쓴다 — 입력이 "지어낸 프런티어" 에서 "실측 셀" 로 바뀔 뿐이고,
그건 원래 탐욕이 잘하는 문제(순회 비용 최소화)다.

## 스텝은 왜 피치의 정수배인가

결속영역 U 폭이 520mm, 배근 피치가 실측 ~225mm 다. 520 을 그냥 쓰면 철근 행이
영역 경계에 **걸터앉아** 한 행을 통째로 잃는다. 피치 1칸(225mm)씩 가면 안전하지만
정차가 2배가 된다. `floor(520/225) × 225 = 450mm` 가 답이고, 여유 70mm 가 남는다.

## 횡이동은 "막혔을 때만"

횡 1회전이 7.2초, 같은 거리 주행이 0.7초로 **10배**다. 그래서 전진이 막힐
때까지 쓰지 않는다. 막히면 한 레인 폭만큼 옮기고 **반대 방향으로 되돌아온다**
(지그재그). 이 규칙 자체가 비용 비대칭에서 바로 나온다 — 가중치를 따로 둘
필요가 없다.

⚠ 횡 **방향**은 이 모듈이 정하지 않는다. 좌우 어느 쪽이 데크인지는 측면
  카메라가 안다. 호출부가 `side_open` 으로 알려줘야 한다 — 추측하면 떨어진다.
"""

import math

# 왜 `kind` 를 문자열로 두나: 미션 문장으로 그대로 번역되고, 로그에서 바로 읽힌다.
DRIVE, LATERAL, DONE, BLOCKED = 'drive', 'lateral', 'done', 'blocked'

MIN_PITCH_MM = 80.0          # 이보다 작으면 피치 추정이 깨진 것으로 본다
MAX_PITCH_MM = 600.0
DEFAULT_PITCH_MM = 225.0     # 2026-11-05 실측 (교차점 간격 X 232 / Y 218)


def step_for(pitch_mm, reach_u_mm):
    """결속영역에 들어가는 **피치의 정수배** 중 가장 큰 것."""
    p = float(pitch_mm or DEFAULT_PITCH_MM)
    if not (MIN_PITCH_MM <= p <= MAX_PITCH_MM):
        p = DEFAULT_PITCH_MM
    k = max(1, int(reach_u_mm // p))
    return k * p


class Reactive:
    """지그재그 순회의 상태. **지도를 보지 않는다.**

    u_dir : +1 전진 / -1 후진. 횡이동할 때마다 뒤집는다 (지그재그)
    v_dir : +1 / -1. 그쪽이 막히면 뒤집고, 양쪽 다 막히면 끝이다
    """

    def __init__(self, reach_u_mm, reach_v_mm, lat_mm_per_turn=50.0,
                 pitch_mm=None, u_dir=1, v_dir=1, lane_overlap=0.15,
                 lane_margin_mm=40.0):
        self.reach_u = float(reach_u_mm)
        self.reach_v = float(reach_v_mm)
        self.lat_mm = float(lat_mm_per_turn)
        self.pitch = pitch_mm
        self.u_dir = 1 if u_dir >= 0 else -1
        self.v_dir = 1 if v_dir >= 0 else -1
        # 레인을 **겹쳐서** 옮긴다. 딱 맞게 옮기면 주행 오차(2.2% 실측)와
        # 검출 누락이 겹쳐 레인 사이에 묶이지 않은 줄이 남는다.
        self.lane_overlap = float(lane_overlap)
        # 영역 경계에서 이만큼 **안쪽**에 들어와야 "확실히 덮였다" 로 본다.
        # 주행 오차(2.2% 실측 ≈ 10mm)·뷰별 ±12mm·횡 양자화 ±25mm 를 합친 예산.
        self.lane_margin = float(lane_margin_mm)
        # ⚠ **횡 방향은 한 번 정하면 끝까지 유지한다.** 막힐 때마다 반대로
        #   틀면 위아래로 **무한 왕복**한다 — 반응형은 설계상 지도를 안 보므로
        #   "이미 덮은 레인" 을 기억하지 못한다 (시뮬에서 레인 28개가 나왔다).
        #   한 방향으로 쭉 가다 막히면 거기가 끝이고, 시작점 반대쪽에 남은 띠는
        #   ③ 수거 단계가 **실측된 지도**를 보고 처리한다. 그게 단계를 나눈 이유다.
        self.v_locked = False
        self.v_tried = set()            # 아직 한 번도 못 쓴 횡 방향
        self.stops = 0
        self.lanes = 0

    # ---- 입력 ----------------------------------------------------------
    def set_pitch(self, pitch_mm):
        """검출에서 추정한 피치. 범위 밖이면 무시한다 (추정이 깨질 때가 있다)."""
        if pitch_mm and MIN_PITCH_MM <= float(pitch_mm) <= MAX_PITCH_MM:
            self.pitch = float(pitch_mm)
            return True
        return False

    def step_mm(self):
        return step_for(self.pitch, self.reach_u)

    def lane_turns(self, rows_v=None, rv=0.0):
        """다음 레인까지의 횡 회전수. **실측 줄 위치로 매번 다시 잰다.**

        ## 왜 고정량을 버렸나 (2026-11-06 측정)

        고정 300mm 는 **모든 피치에서 한 줄을 빠뜨렸다.** 겹침이 모자란 게
        아니라 **위상이 밀리는** 것이 원인이다 — 피치 215 에 300mm 를 옮기면
        레인마다 85mm 밀리고, 몇 레인 뒤 한 줄이 영역 경계에 올라탄다. 그러면
        그 줄을 빠뜨리거나, 이미 묶은 자리를 또 묶는다.

        "피치의 정수배" 도 같은 병을 앓는다 — 횡은 50mm 양자화라 피치 180 의
        정수배 360mm 를 못 가고 350mm 로 가서, 레인마다 10mm 가 **누적**된다.

        그래서 피치를 **아예 쓰지 않는다.** 검출된 줄 위치를 직접 보고 매
        레인 다시 계산하면 양자화 오차가 누적되지 않는다.

            다음 레인 중심 = (아직 안 묶은 첫 줄) + 진행방향 × (영역폭/2 - 여유)
            회전수        = floor(그 거리 / 50mm)

        ⚠ **내림이다.** 목표를 넘치면 그 줄이 영역 **밖으로** 빠져 다시 잃는다.
          내림이어도 모자랄 일은 없다 — 그 줄이 안 덮였다는 것이
          `|v-rv| > half` 이므로 이동량이 항상 `2*half`(300mm)를 넘고,
          따라서 최소 6회전이다 (찔끔거리지 않는다).

        측정 (데크 V 1400, 영역 380, 여유 40, 모든 위상에 대한 최악):
            고정 300mm        1줄 빠뜨림 · 24회전
            피치 정수배        1줄 빠뜨림(피치 180) · 25~28회전
            **실측 줄 기반**   **0줄** · 27~28회전   ← +4회전(29초)에 미결속 0

        rows_v 가 비면(아직 그쪽을 못 봤다) 고정 폭으로 **탐색**한다. 반응형은
        지도를 산출물로 보므로 "안 보이는 곳 = 없는 곳" 이 아니다.

        ⚠ 미해결 — 데크 끝의 **마지막 줄**은 이동량을 데크 끝으로 clamp 해야
          덮이는데, 측면 305 는 `rebar_frac` 불린만 주고 **남은 mm 를 모른다**.
          측면에 `corridor` 를 붙이면 완결된다 (전후방이 쓰는 그 코드다).
        """
        half = self.reach_v / 2.0 - self.lane_margin
        d = self.v_dir
        ahead = [float(v) for v in (rows_v or ())
                 if (float(v) - rv) * d > half]
        if ahead:
            v1 = min(ahead, key=lambda v: (v - rv) * d)
            want = v1 + d * half                 # 그 줄이 **가까운 쪽** 경계에
            t = int(math.floor(abs(want - rv) / self.lat_mm))
            if t >= 1:
                return t
        return self._lane_turns_fixed()

    def _lane_turns_fixed(self):
        """줄 위치를 모를 때의 탐색 폭. 내림 — 올리면 겹침이 의도보다 얇아진다."""
        need = self.reach_v * (1.0 - self.lane_overlap)
        return max(1, int(math.floor(need / self.lat_mm)))

    # ---- 결정 ----------------------------------------------------------
    def decide(self, go_mm, blocked_dirs, side_open, max_lanes=None,
               rows_v=None, rv=0.0):
        """다음 행동. 센서 값만 받는다.

        go_mm        전진 가능 거리 (deck_edge). None = 이 판정으로는 제한 없음
        blocked_dirs 차단된 방향 집합 — 'forward' / 'backward'
        side_open    {'+1': bool, '-1': bool} 그쪽 횡으로 갈 수 있나 (측면 카메라)
        max_lanes    레인 상한 (시험용). None 이면 무제한
        rows_v       **아직 안 묶은** 줄의 V 좌표 (작업 좌표). 레인 간격의 근거
        rv           지금 횡 위치 (작업 좌표)

        반환 {'kind': ..., 'mm'|'turns': ..., 'why': ...}
        """
        want = self.step_mm()
        fwd_key = 'forward' if self.u_dir > 0 else 'backward'
        blocked = fwd_key in (blocked_dirs or ())
        short = (go_mm is not None) and (go_mm < want)

        if not blocked and not short:
            return {'kind': DRIVE, 'mm': self.u_dir * want,
                    'why': f'피치 {self.pitch or DEFAULT_PITCH_MM:.0f}mm × '
                           f'{int(want / (self.pitch or DEFAULT_PITCH_MM))}'}

        # ── 전진이 막혔다 → 레인 전환
        if max_lanes is not None and self.lanes + 1 >= max_lanes:
            return {'kind': DONE, 'why': f'레인 상한 {max_lanes}'}
        why = '전방 차단' if blocked else f'남은 {go_mm:.0f}mm < 스텝 {want:.0f}mm'
        # 방향이 고정된 뒤에는 그쪽만 본다. 막히면 그게 끝이다.
        cands = (self.v_dir,) if self.v_locked else (self.v_dir, -self.v_dir)
        for d in cands:
            if d in self.v_tried:
                continue
            if not (side_open or {}).get(d, True):
                self.v_tried.add(d)
                continue
            self.v_dir = d
            t = self.lane_turns(rows_v, rv)
            return {'kind': LATERAL, 'turns': d * t, 'why': why}
        if self.v_locked:
            return {'kind': DONE, 'why': f'{why} · 횡 {self.v_dir:+d} 쪽도 끝'}
        return {'kind': DONE, 'why': '전진·좌우 모두 막혔다'}

    # ---- 결과 반영 ------------------------------------------------------
    def moved(self, kind, amount, ok=True):
        """실제로 움직인 뒤 호출. 지그재그 상태를 여기서만 바꾼다."""
        if kind == DRIVE:
            if ok:
                self.stops += 1
                self.v_tried.clear()   # 전진했으면 좌우를 다시 시도할 수 있다
        elif kind == LATERAL:
            if ok:
                self.lanes += 1
                self.u_dir = -self.u_dir      # ⚠ 지그재그는 **여기서만** 뒤집는다
                self.v_locked = True          # 한 번 간 방향으로만 계속 간다
                self.v_tried.clear()
            else:
                self.v_tried.add(1 if amount > 0 else -1)

    def snapshot(self):
        return {'u_dir': self.u_dir, 'v_dir': self.v_dir, 'v_locked': self.v_locked,
                'pitch_mm': self.pitch,
                'step_mm': round(self.step_mm()), 'lane_turns': self.lane_turns(),
                'stops': self.stops, 'lanes': self.lanes,
                'v_tried': sorted(self.v_tried)}


def pitch_from(points, axis=0, tol=0.25):
    """검출 좌표에서 피치를 추정한다. 인접 간격의 **중앙값**.

    ⚠ 평균이 아니라 중앙값이다. 교차점이 하나 빠지면 그 자리가 2배 간격이
      되는데, 평균은 거기 끌려가고 중앙값은 안 끌려간다.
    ⚠ 전체 좌표를 정렬해 인접차를 보면 **같은 줄의 점들**이 0 근처 간격을
      무더기로 만든다. 그래서 `tol` 보다 가까운 것은 같은 줄로 보고 묶는다.
    """
    v = sorted(float(p[axis]) for p in points or ())
    if len(v) < 3:
        return None
    rows, cur = [], [v[0]]
    span = max(1.0, (v[-1] - v[0]))
    for x in v[1:]:
        if x - cur[-1] <= tol * span / max(1, len(v) - 1) * 4.0:
            cur.append(x)
        else:
            rows.append(sum(cur) / len(cur))
            cur = [x]
    rows.append(sum(cur) / len(cur))
    if len(rows) < 2:
        return None
    d = sorted(b - a for a, b in zip(rows, rows[1:]))
    med = d[len(d) // 2]
    return med if MIN_PITCH_MM <= med <= MAX_PITCH_MM else None

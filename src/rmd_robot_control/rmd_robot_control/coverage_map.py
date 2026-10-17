#!/usr/bin/env python3
"""L6 — 자율작업 커버리지 지도. **ROS 무관 순수 모듈.**

    관측(교차점) ─▶ 격자에 붙이기 ─▶ 셀 상태 ─▶ 다음 정차지

설계 근거는 `docs/design/AUTONOMY_COVERAGE_DESIGN.md` 다. 여기서는 그 중
"왜 이렇게 짰나" 만 적는다.

## 왜 ROS 를 안 쓰나

2차년도에서 효과가 증명된 선택이다. `coverage_planner.py` 가 순수함수라
장비 없이 자가테스트가 됐고, 그 덕에 "마지막 열 + 마진" 규칙의 버그를
책상에서 잡았다. 지도·플래너는 자율작업에서 가장 틀리기 쉬운 부분이라
**장비 없이 돌려볼 수 있어야 한다** (`tools/test/coverage_sim.py`).

## 좌표 규약 — 정준 좌표 (U, V)

**1번 자세의 스테이지 좌표**를 정준 좌표로 쓴다. U = 스테이지 X(주행방향),
V = 스테이지 Y(횡방향), 단위 mm.

왜 하필 1번 자세인가: 스테이지 좌표는 **자세마다 다르다** (같은 교차점이
1번에서 X 278mm 인데 3번에서 398mm — 오프셋 최대 122mm). 그래서 "스테이지
좌표" 자체는 로봇 고정 좌표계가 아니다. 하지만 **자세 하나를 정해 두면**
그 자세의 스테이지 좌표는 로봇에 고정된 미터법 평면 좌표가 된다. 1번을
고른 것은 검출 자세(`park_pose`)라서 그렇다 — 임의 선택이고, 어느 자세든
된다. 실제로 어느 자세로 결속할지는 `Reach` 가 따로 판정한다.

⚠ 카메라 좌표를 그대로 쓰지 않는 이유: 카메라가 철근 평면에 **29° 기울어**
  있어서 Δcam 이 평면상 거리가 아니다. 정준 좌표는 모델이 이미 그 보정을
  하고 나온 값이라 평면상 mm 다.

## 작업 좌표와 격자

    작업 좌표  (wu, wv) = 로봇 포즈 (ru, rv) + 정준 좌표 (du, dv)
    격자 지수  (i, j)   = round((wu - origin_u) / pitch_u), 같은 식으로 j

로봇 포즈는 누적이다 — 주행은 실제로 간 mm, 횡이동은 회전수 × 50mm.
위치추정(SLAM)은 하지 않는다. 누적 오차는 §observe 의 격자 정렬이 잡는다.
"""
from __future__ import annotations

import math
import time

from .coverage_reach import Reach          # noqa: F401  (재수출 — 쓰는 쪽 편의)
from .coverage_space import FreeSpace     # noqa: F401

# 횡이동 1회전 = 50mm (실측 2026-10-17). 플래너에서 이 값은 **횡방향 양자화
# 단위**다 — 그 이하 간격으로는 설 수 없다. `lateral_axes.MM_PER_TURN` 과
# 같은 값이어야 하지만, 이 모듈은 ROS·CAN 을 import 하지 않으려고 따로 둔다.
LAT_MM_PER_TURN = 50.0

UNKNOWN, PENDING, TIED, FAILED, BLOCKED, EMPTY = (
    'unknown', 'pending', 'tied', 'failed', 'blocked', 'empty')

# 못 본 자리(frontier)를 "결속 대상일 것" 으로 칠 때의 할인. 격자가 있다고
# 말하지만 실제로 없을 수도 있다(배근이 끊겼거나 모서리거나).
# ⚠ 1.0 으로 두면 본 것과 못 본 것이 같아져 **헛걸음**이 늘고, 0 으로 두면
#   앞으로 나아갈 후보가 사라져 **비싼 횡이동**을 고른다 (시뮬에서 실제로
#   61.6초짜리 횡이동을 전진 대신 골랐다).
FRONTIER_WEIGHT = 0.7


# ──────────────────────────────────────────────────────────────────────
# 피치 추정 — 2차년도 `coverage_planner.py` 에서 이식 (검증된 순수함수)
# ──────────────────────────────────────────────────────────────────────

def cluster_1d(vals, tol):
    """1D 값들을 tol(mm) 이내로 그룹핑 → [(중심, 멤버수), ...] 오름차순."""
    s = sorted(float(v) for v in vals)
    if not s:
        return []
    groups = [[s[0]]]
    for v in s[1:]:
        if v - groups[-1][-1] <= tol:
            groups[-1].append(v)
        else:
            groups.append([v])
    return [(sum(g) / len(g), len(g)) for g in groups]


def estimate_pitch(centers, fallback=None):
    """정렬된 클러스터 중심 → (피치, 출처).

    결측 열/행 때문에 2×·3× 간격이 섞여도 **최소간격의 정수배로 정규화**해
    1× 로 복원한다. 이게 이 함수의 전부이자 핵심이다 (2차년도 검증).
    """
    if len(centers) < 2:
        return (float(fallback), 'fallback') if fallback else (None, 'none')
    diffs = [b - a for a, b in zip(sorted(centers), sorted(centers)[1:])]
    base = min(diffs)
    if base < 1e-3:
        return (float(fallback), 'fallback') if fallback else (None, 'none')
    norm = sorted(d / max(1, round(d / base)) for d in diffs)
    n = len(norm)
    med = norm[n // 2] if n % 2 else 0.5 * (norm[n // 2 - 1] + norm[n // 2])
    return float(med), 'measured'


# ──────────────────────────────────────────────────────────────────────
# 비용
# ──────────────────────────────────────────────────────────────────────

class Cost:
    """이동·작업 비용(초). ⚠ 전부 **실측 전 임시값**이다.

    상수를 재기 전에 플래너를 복잡하게 만들지 말 것 — 결속이 이동보다 훨씬
    비싸면 "한 번 서서 최대한 많이 묶기" 가 정답이 되어 이동 최적화의 실익이
    거의 없다. 어느 쪽인지는 재 봐야 안다.
    """

    def __init__(self, drive_mm_s=70.0, drive_fix=2.0,
                 lat_turn_s=7.2, lat_fix=3.0, stop_fix=20.0, tie_s=0.0):
        self.drive_mm_s = drive_mm_s
        self.drive_fix = drive_fix
        self.lat_turn_s = lat_turn_s
        self.lat_fix = lat_fix
        # 정차 1회의 **고정 비용** — 검출 + 결속 시퀀스 진입·park.
        # ⚠ 1초로 두면 정차가 너무 싸서 플래너가 조금씩 자주 선다. park 하나가
        #   22.5초라는 실측이 있으니 20초는 오히려 보수적이다. [실측 전 임시값]
        self.stop_fix = stop_fix
        self.tie_s = tie_s

    def move(self, d_drive_mm, n_turns):
        c = self.stop_fix
        if abs(d_drive_mm) > 1e-6:
            c += self.drive_fix + abs(d_drive_mm) / self.drive_mm_s
        if n_turns:
            c += self.lat_fix + abs(n_turns) * self.lat_turn_s
        return c


# ──────────────────────────────────────────────────────────────────────
# 셀과 지도
# ──────────────────────────────────────────────────────────────────────

class Cell:
    __slots__ = ('i', 'j', 'u', 'v', 'n', 'state', 'why', 't', 'fails')

    def __init__(self, i, j, u, v):
        self.i, self.j = i, j
        self.u, self.v = u, v      # 작업 좌표 (누적평균으로 갱신)
        self.n = 1                 # 관측 횟수
        self.state = PENDING
        self.why = ''
        self.t = time.time()
        self.fails = 0

    def key(self):
        return (self.i, self.j)

    def __repr__(self):
        return f'<셀 ({self.i},{self.j}) {self.state} u={self.u:.0f} v={self.v:.0f}>'


class CoverageMap:
    """격자 지도 + 다음 정차지 선택."""

    def __init__(self, reach, cost=None, pitch=(None, None),
                 cluster_tol=50.0, merge_frac=0.35, max_fails=2,
                 lat_mm_per_turn=LAT_MM_PER_TURN, fov=None, explore_ring=3,
                 lane_stick=0.4, space=None):
        self.reach = reach
        self.cost = cost or Cost()
        self.pitch_u, self.pitch_v = pitch
        self.cluster_tol = cluster_tol
        # 관측을 기존 셀에 붙이는 반경 = 피치의 이 비율. ⚠ 0.5 보다 작아야
        # 옆 칸을 삼키지 않는다. 2차년도는 **고정 mm** 를 써서 한 줄이 둘로
        # 쪼개졌다 (1202 로 고정 → 1308 이 106mm 떨어져 신규로 샘).
        self.merge_frac = merge_frac
        self.max_fails = max_fails
        self.lat_mm = lat_mm_per_turn

        self.ru = 0.0              # 로봇 포즈 (작업 좌표)
        self.rv = 0.0
        self.origin = None         # 격자 원점 (작업 좌표)
        self.cells = {}
        self.log = []

        bb = reach.bbox()
        self.reach_bbox = bb
        self.reach_mid = (0.5 * (bb[0] + bb[1]), 0.5 * (bb[2] + bb[3]))
        # 검출 시야. **결속영역보다 넓다** — 이게 없으면 지도는 처음 선 자리에서
        # 보이는 것만 알고 끝난다 (시뮬에서 실제로 그렇게 됐다: 레인 A 만 묶고
        # "완료" 했다). 탐색 계획의 전제다.
        self.fov = fov if fov is not None else reach.image_fov()
        self.fov_mid = (0.5 * (self.fov[0] + self.fov[1]),
                        0.5 * (self.fov[2] + self.fov[3]))
        # 격자를 몇 칸까지 외삽해 "있을 법한 자리" 로 볼지.
        # ⚠ 1 이면 **한 열 앞만** 보이므로 플래너가 한 피치씩만 간다. 결속영역은
        #   3열이 닿는데 2점만 묶고 320mm 를 겹쳐 서게 된다(사용자 지적).
        #   배근은 주기적이고 데크는 이어지므로 3칸 외삽은 안전하다 — 끝은
        #   `set_edge` 와 `empty` 가 닫는다.
        self.explore_ring = explore_ring
        self.edge = {}             # (축, 부호) → 그 너머는 데크 밖이다
        self.miss = {}             # 격자키 → 시야 안인데 안 보인 횟수
        # 피치를 아직 못 잰 축에서 쓰는 **탐침 보폭**. 시야 폭의 60% —
        # 겹침을 남겨 두어야 경계에 걸친 교차점을 놓치지 않는다.
        self.probe_u = 0.6 * (self.fov[1] - self.fov[0])
        self.probe_v = 0.6 * (self.fov[3] - self.fov[2])
        # 시야 안인데 이만큼 연속으로 안 보이면 '없다'(empty)로 닫는다.
        # ⚠ 1 로 두면 검출 한 번 놓친 자리가 영구히 '없음'이 된다 = 누락.
        #   2 면 쓸고 지나갈 때 시야가 겹치므로 자연히 두 번 본다.
        self.empty_confirm = 2
        # **레인 유지 선호 (0~1).** 1 = 같은 레인에서 벌 게 있으면 무조건
        # 유지, 0 = 순수 탐욕(점수만 본다).
        #
        # 왜 필요한가: 탐욕 점수는 근시안이라 V 방향으로 들락거린다.
        # 시뮬 7x5(피치 200mm) 실측 —
        #     순수 탐욕   횡이동 32회 · 이동 340초
        #     레인 유지   횡이동 16회 · 이동 258초   ← 이상적 전환 횟수와 같다
        # 횡이동 1회전이 주행 350mm 와 맞먹으니 이득이 크다.
        self.lane_stick = lane_stick
        # 한 셀에 대해 볼 횡이동 격자 행의 최대 수 (후보 폭발 방지)
        self.max_rows = 9
        # **자유공간 층** — "여기 있을 수 있나" 는 이쪽이 답한다.
        # ⚠ 결속영역(건이 닿는 범위)과 몸체영역(로봇이 차지하는 범위)은
        #   다르다. 처음엔 결속영역을 점유로 썼는데, 그러면 장애물이 작업
        #   범위에 들어오기만 해도 못 서는 것이 되어 지나치게 엄했다.
        #   두 영역의 **중심축은 일치한다** (사용자 확인 2026-10-17) —
        #   그래서 몸체 중심에 결속영역 중심을 넘긴다.
        self.space = space if space is not None else FreeSpace(
            center=(0.5 * (bb[0] + bb[1]), 0.5 * (bb[2] + bb[3])))

    # ---- 포즈 ----
    def advance(self, drive_mm=0.0, lateral_turns=0):
        """로봇이 실제로 움직인 만큼 포즈를 누적한다.

        ⚠ **명령값이 아니라 실측값**을 넣을 것. `drive_node` 는 바퀴 각도
          폐루프라 실제 간 거리를 보고한다 (`/drive/status`).
        """
        self.ru += float(drive_mm)
        self.rv += float(lateral_turns) * self.lat_mm

    # ---- 관측 ----
    def observe(self, dets, tied_flags=None):
        """검출 결과를 지도에 반영한다.

        dets : [(du, dv), ...] 정준 좌표. 로봇 기준이다.
        tied_flags : 같은 길이. True 면 '이미 결속됨' 으로 본 것.

        세 가지를 한다 — ① 피치 갱신 ② 격자 정렬(포즈 보정) ③ 셀 갱신.
        """
        if not dets:
            return {'n': 0, 'why': '검출 0'}
        wu = [self.ru + d[0] for d in dets]
        wv = [self.rv + d[1] for d in dets]

        # ① 피치
        cu = [c for c, _ in cluster_1d(wu, self.cluster_tol)]
        cv = [c for c, _ in cluster_1d(wv, self.cluster_tol)]
        pu, su = estimate_pitch(cu, self.pitch_u)
        pv, sv = estimate_pitch(cv, self.pitch_v)
        old = (self.step_u(), self.step_v())
        if su == 'measured':
            self.pitch_u = pu if self.pitch_u is None else 0.7 * self.pitch_u + 0.3 * pu
        if sv == 'measured':
            self.pitch_v = pv if self.pitch_v is None else 0.7 * self.pitch_v + 0.3 * pv
        self._reindex(old)

        # ② 격자 정렬 — 누적 오차를 1피치 안으로 묶는다
        if self.origin is None:
            self.origin = (min(wu), min(wv))
            shift = (0.0, 0.0)
        else:
            shift = (self._align(wu, self.origin[0], self.pitch_u),
                     self._align(wv, self.origin[1], self.pitch_v))
            # 포즈를 보정한다 (원점이 아니라) — 틀린 쪽은 누적 포즈다
            self.ru -= shift[0]
            self.rv -= shift[1]
            wu = [x - shift[0] for x in wu]
            wv = [x - shift[1] for x in wv]

        # ③ 셀
        flags = tied_flags or [None] * len(dets)
        n_new = n_hit = 0
        seen_keys = set()
        for u, v, tf in zip(wu, wv, flags):
            c = self._attach(u, v)
            seen_keys.add(c.key())
            if c.n == 1:
                n_new += 1
            else:
                n_hit += 1
            if tf and c.state == PENDING:
                c.state = TIED
                c.why = '검출이 이미 결속됨으로 분류'
        n_empty = self._close_empty(seen_keys)
        return {'n': len(dets), 'new': n_new, 'hit': n_hit, 'empty': n_empty,
                'pitch': (self.pitch_u, self.pitch_v), 'shift': shift,
                'pitch_src': (su, sv)}

    def _close_empty(self, seen_keys):
        """**시야 안인데 안 보인 자리**를 '없다'로 닫는다.

        이게 없으면 탐색이 끝나지 않는다 — 못 본 자리는 영원히 못 본 자리로
        남아 플래너가 계속 보러 가자고 한다. 시뮬에서 바로 드러났다.

        ⚠ 검출을 한 번 놓친 것과 진짜로 없는 것을 구별하려고 `empty_confirm`
          번 연속으로 안 보여야 닫는다. 나중에 보이면 `_attach` 가 되살린다.
        """
        n = 0
        for k, (u, v) in self.frontier() + [(c.key(), (c.u, c.v))
                                            for c in self.cells.values()]:
            if k in seen_keys:
                self.miss.pop(k, None)
                continue
            du, dv = u - self.ru, v - self.rv
            if not (self.fov[0] <= du <= self.fov[1]
                    and self.fov[2] <= dv <= self.fov[3]):
                continue                      # 시야 밖 — 안 보인 게 당연하다
            c = self.cells.get(k)
            if c is not None and c.state != PENDING:
                continue                      # 이미 묶었거나 막힌 자리
            self.miss[k] = self.miss.get(k, 0) + 1
            if self.miss[k] < self.empty_confirm:
                continue
            if c is None:
                c = Cell(k[0], k[1], u, v)
                self.cells[k] = c
            c.state, c.why = EMPTY, f'시야 안에서 {self.miss[k]}회 안 보임'
            n += 1
        return n

    def _reindex(self, old_step):
        """보폭이 크게 바뀌었으면 셀 키를 다시 매긴다.

        **위치(u, v)가 진실이고 격자 지수는 파생값이다.** 처음에는 피치를
        몰라 탐침 보폭으로 매기는데, 진짜 피치가 나오면 그 키들이 전부
        틀린 값이 된다. 안 고치면 같은 자리가 두 셀로 갈라지거나 서로 다른
        자리가 한 셀로 합쳐진다.

        합쳐질 때는 관측이 많은 쪽을 남기고 상태는 **더 진행된 쪽**을 쓴다
        (묶은 것을 안 묶은 것으로 되돌리면 이중결속이 난다).
        """
        new = (self.step_u(), self.step_v())
        if all(abs(a - b) < 0.05 * max(a, b, 1.0) for a, b in zip(old_step, new)):
            return 0
        rank = {TIED: 4, FAILED: 3, BLOCKED: 3, PENDING: 2, EMPTY: 1, UNKNOWN: 0}
        out = {}
        for c in self.cells.values():
            c.i = self._idx(c.u, self.origin[0], new[0])
            c.j = self._idx(c.v, self.origin[1], new[1])
            k = (c.i, c.j)
            prev = out.get(k)
            if prev is None:
                out[k] = c
                continue
            keep, drop = (prev, c) if prev.n >= c.n else (c, prev)
            if rank.get(drop.state, 0) > rank.get(keep.state, 0):
                keep.state, keep.why = drop.state, drop.why
            keep.n += drop.n
            out[k] = keep
        n = len(self.cells) - len(out)
        self.cells = out
        self.miss = {}
        return n

    def _align(self, vals, origin, pitch):
        """관측이 격자에서 평균 얼마나 밀렸나 (원형 평균, [-p/2, p/2))."""
        if not pitch or pitch <= 0:
            return 0.0
        ang = [2 * math.pi * ((x - origin) / pitch % 1.0) for x in vals]
        s = sum(math.sin(a) for a in ang) / len(ang)
        c = sum(math.cos(a) for a in ang) / len(ang)
        if abs(s) < 1e-12 and abs(c) < 1e-12:
            return 0.0
        return math.atan2(s, c) / (2 * math.pi) * pitch

    def _attach(self, u, v):
        """관측을 셀에 붙인다. 없으면 새로 만든다."""
        i = self._idx(u, self.origin[0], self.step_u())
        j = self._idx(v, self.origin[1], self.step_v())
        c = self.cells.get((i, j))
        if c is None:
            c = Cell(i, j, u, v)
            self.cells[(i, j)] = c
            return c
        if c.state == EMPTY:
            # 없다고 닫았는데 보였다 — 검출을 놓쳤던 것이다. 되살린다.
            c.state, c.why = PENDING, '없음으로 닫았다가 재검출'
            self.miss.pop((i, j), None)
        # ⚠ 누적평균으로 갱신한다. 첫 관측만 붙들면 나중 관측이 병합범위
        #   밖으로 밀려나 **없는 셀이 새로 생긴다** (2차년도 실측).
        c.n += 1
        c.u += (u - c.u) / c.n
        c.v += (v - c.v) / c.n
        return c

    def _idx(self, x, origin, step):
        """작업 좌표 → 격자 지수.

        ⚠ 피치를 아직 못 쟀을 때 0 을 돌려주면 **그 축의 모든 점이 한 셀로
          뭉개진다** — 시뮬에서 550mm 떨어진 두 교차점이 같은 셀이 되어, 앞
          열을 "이미 묶었다" 로 보고 건너뛰었다. 그래서 탐침 보폭으로라도
          나눈다. 나중에 진짜 피치가 나오면 `_reindex` 가 다시 매긴다.
        """
        if not step or step <= 0:
            return 0
        return int(round((x - origin) / step))

    # ---- 상태 변경 ----
    def see_obstacle(self, du, dv, size=None, why='장애물 감지'):
        """**가 보지 않고** 센서로 본 장애물 (로봇 기준 좌표, 크기 mm).

        **`size` 가 핵심이다.** 둘은 다른 것이라 섞으면 안 된다:

          size 있음 (기둥·자재더미) → 자유공간 + 격자 둘 다.
              주행을 막는다. "지나갈 수 있나" 가 여기서 풀린다
          size 없음 (결속 불가 지점) → 격자만.
              그 자리를 못 묶을 뿐, 로봇은 지나갈 수 있다

        ⚠ 크기 있는 장애물을 격자에만 찍으면 **기둥의 두께를 표현할 수 없고**,
          자유공간에만 넣으면 플래너가 그 위의 교차점을 계속 묶으러 간다.
        """
        if self.origin is None:
            return None
        u, v = self.ru + du, self.rv + dv
        su, sv = (size if isinstance(size, (tuple, list)) else (size, size)) \
            if size else (0.0, 0.0)
        if su > 0.0 or sv > 0.0:
            self.space.add(u - su / 2, u + su / 2, v - sv / 2, v + sv / 2,
                           why, time.time())
        hit = []
        for c in self.cells.values():
            if (abs(c.u - u) <= su / 2 and abs(c.v - v) <= sv / 2
                    and c.state in (PENDING, UNKNOWN, EMPTY)):
                c.state, c.why, c.t = BLOCKED, why, time.time()
                hit.append(c)
        if not hit and su <= 1.0:          # 점 장애물 — 가장 가까운 셀 하나
            c = self._attach(u, v)
            c.state, c.why, c.t = BLOCKED, why, time.time()
            hit = [c]
        return hit

    def mark(self, key, state, why=''):
        c = self.cells.get(tuple(key))
        if c is None:
            return None
        if state == FAILED:
            c.fails += 1
            # 재시도가 남아 있으면 pending 으로 되돌린다 — 다음에 들를 때 다시.
            state = FAILED if c.fails >= self.max_fails else PENDING
        c.state, c.why, c.t = state, why, time.time()
        self.log.append((time.time(), tuple(key), state, why))
        return c

    def set_edge(self, axis, sign, value, why='데크 끝'):
        """그 축의 그 방향 **너머는 데크 밖**이라고 못박는다.

        탐색이 거기로 나가지 않게 한다. `block_beyond` 가 이미 본 셀을
        막는다면, 이쪽은 **아직 안 본 곳**을 막는다 — 둘 다 필요하다.
        안 그러면 플래너가 데크 밖으로 계속 보러 나가자고 한다.
        """
        self.edge[(axis, int(sign))] = (float(value), why)
        return self.block_beyond(axis, value, sign, why)

    def block_beyond(self, axis, value, sign, why='데크 끝'):
        """그 방향으로 더 간 셀을 전부 blocked 로. axis 'u'|'v'."""
        n = 0
        for c in self.cells.values():
            if c.state not in (PENDING, UNKNOWN):
                continue
            x = c.u if axis == 'u' else c.v
            if (x - value) * sign > 0:
                c.state, c.why = BLOCKED, why
                n += 1
        return n

    # ---- 질의 ----
    def occupiable(self, ru, rv):
        """그 자리에 설 수 있나 — 자유공간 층에 묻는다."""
        return self.space.occupiable(ru, rv)

    def path_to(self, ru1, rv1):
        """지금 자리에서 그 자리까지의 **경로와 비용**. 못 가면 None.

        ⚠ 가능 여부만 받아 오고 실행은 ㄱ자로 하면 **기둥을 관통한다.**
          경로를 그대로 들고 다녀야 한다 (실행·비용·그림 전부).
        """
        return self.space.path_to((self.ru, self.rv), (ru1, rv1), self.cost)

    def move_ok(self, ru1, rv1):
        """갈 수 있나 (호환용 — 새 코드는 `path_to` 를 쓴다)."""
        return self.path_to(ru1, rv1) is not None

    def pending(self):
        return [c for c in self.cells.values() if c.state == PENDING]

    def counts(self):
        out = {}
        for c in self.cells.values():
            out[c.state] = out.get(c.state, 0) + 1
        return out

    def step_u(self):
        """U 축 외삽 보폭. 피치를 아직 못 쟀으면 **탐침 보폭**을 쓴다.

        ⚠ 이 폴백이 없으면 한 축의 피치를 못 잰 동안 앞으로 갈 후보가 아예
          안 생긴다 — 시뮬에서 그 탓에 전진(10초) 대신 횡이동(61.6초)을 골랐다.
        """
        return self.pitch_u or self.probe_u

    def step_v(self):
        return self.pitch_v or self.probe_v

    def uv_of(self, i, j):
        """격자 지수 → 작업 좌표 (피치 외삽)."""
        return (self.origin[0] + i * self.step_u(),
                self.origin[1] + j * self.step_v())

    def frontier(self):
        """**아직 못 본** 격자 자리 중 본 자리에 붙어 있는 것들.

        "안 보이는 것은 없는 것" 이 아니다 — 시야 밖일 수도, 건이 가렸을
        수도 있다(2026-10-16 에 건이 교차점 4개를 가렸다). 피치로 외삽해
        "있을 법한 자리" 를 만들고, 보러 갈 가치가 있는 곳으로 삼는다.

        ⚠ 무한히 뻗지 않게 두 가지로 막는다 — ① 본 셀에 **붙은** 자리만
          (ring 1) ② `set_edge` 로 못박은 데크 경계 너머는 제외.
        """
        if self.origin is None:
            return []
        known = set(self.cells)
        dead = {k for k, c in self.cells.items() if c.state == BLOCKED}
        r = self.explore_ring
        out = {}
        for (i, j), c in self.cells.items():
            if c.state == BLOCKED:
                continue
            for di in range(-r, r + 1):
                for dj in range(-r, r + 1):
                    k = (i + di, j + dj)
                    if k in known or k in dead or k in out:
                        continue
                    u, v = self.uv_of(*k)
                    if self._past_edge(u, v):
                        continue
                    out[k] = (u, v)
        return [(k, uv) for k, uv in out.items()]

    def _past_edge(self, u, v):
        for (axis, sign), (val, _why) in self.edge.items():
            x = u if axis == 'u' else v
            if (x - val) * sign > 0:
                return True
        return False

    # ---- 계획 ----
    def candidates(self, fr=None):
        """후보 정차지. **셀에서 역으로 만든다** — 자유격자를 훑지 않는다.

        각 셀이 결속영역 한가운데 오는 포즈를 후보로 내고, 횡방향은 횡이동
        1회전 단위로 양자화한다 (그 이하로는 설 수 없다).

        미결속 셀뿐 아니라 **못 본 자리(frontier)** 도 생성원으로 쓴다.
        안 그러면 앞으로 나아가는 후보가 안 만들어진다.
        """
        fr = self.frontier() if fr is None else fr
        src = [(c.u, c.v) for c in self.pending()] + [uv for _k, uv in fr]
        # ⚠ 셀마다 포즈 **하나**(가운데)만 내면, 그 자리가 막혔을 때 셀을 통째로
        #   버린다. 흔들기 몇 개로도 부족했다 — 횡은 1회전 격자 위에서만 설 수
        #   있는데, 결속영역에 그 셀을 담는 격자 행이 여럿이고 그중 일부만
        #   막혀 있는 경우가 흔하다. 그래서 **V 는 가능한 격자 행을 전부**,
        #   U 는 몇 군데를 낸다 (U 는 연속이라 촘촘히 볼 실익이 적다).
        b = self.reach_bbox
        inset = 0.12 * (b[1] - b[0])
        seen, out = set(), []
        for cu, cv in src:
            t_lo = math.ceil((cv - b[3] - self.rv) / self.lat_mm)
            t_hi = math.floor((cv - b[2] - self.rv) / self.lat_mm)
            if t_hi < t_lo:
                continue
            if t_hi - t_lo > self.max_rows:          # 너무 많으면 가운데만
                mid_t = (t_lo + t_hi) // 2
                rows = range(mid_t - self.max_rows // 2, mid_t + self.max_rows // 2 + 1)
            else:
                rows = range(t_lo, t_hi + 1)
            for turns in rows:
                rv = self.rv + turns * self.lat_mm
                for ru in (cu - self.reach_mid[0],
                           cu - b[0] - inset, cu - b[1] + inset):
                    k = (round(ru / 25.0), turns)
                    if k in seen:
                        continue
                    seen.add(k)
                    out.append((ru, rv, turns))
        return out

    def covered(self, ru, rv):
        """그 포즈에서 결속 가능한 pending 셀들.

        ⚠ 사각 선필터를 먼저 건다. `pose_for` 는 자세 네 개를 다 계산하므로
          비싼데, 결속영역 외접 사각 밖은 어차피 None 이다. 352점짜리
          테스트베드에서 이 한 줄이 후보 평가를 수십 배 줄인다.
        """
        b = self.reach_bbox
        got = []
        for c in self.pending():
            du, dv = c.u - ru, c.v - rv
            if not (b[0] <= du <= b[1] and b[2] <= dv <= b[3]):
                continue
            p = self.reach.pose_for(du, dv)
            if p is not None:
                got.append((c, p))
        return got

    def covered_frontier(self, ru, rv, fr=None):
        """그 포즈에서 **결속영역 안에 들어오는 못 본 자리** 수."""
        fr = self.frontier() if fr is None else fr
        b = self.reach_bbox
        n = 0
        for _k, (u, v) in fr:
            du, dv = u - ru, v - rv
            if not (b[0] <= du <= b[1] and b[2] <= dv <= b[3]):
                continue
            if self.reach.pose_for(du, dv) is not None:
                n += 1
        return n

    def explore_candidates(self, fr=None):
        """못 본 자리를 **시야에** 넣는 포즈들. 결속영역이 아니라 시야 기준이다.

        1단계(결속영역 안에 뭔가 들어오는 포즈)가 하나도 없을 때만 쓴다.
        """
        fr = self.frontier() if fr is None else fr
        seen, out = set(), []
        for _k, (u, v) in fr:
            ru = u - self.fov_mid[0]
            turns = round((v - self.fov_mid[1] - self.rv) / self.lat_mm)
            rv = self.rv + turns * self.lat_mm
            kk = (round(ru, 1), turns)
            if kk in seen:
                continue
            seen.add(kk)
            n = sum(1 for _, (fu, fv) in fr
                    if self.fov[0] <= fu - ru <= self.fov[1]
                    and self.fov[2] <= fv - rv <= self.fov[3])
            out.append((ru, rv, turns, n))
        return out

    def plan_next(self):
        """다음 정차지. 없으면 None (= 완료).

        **두 단계다. 섞지 않는다.**
          ① 결속할 수 있는 곳이 있으면 간다 — 점수 = 새로 묶을 셀 수 / 이동비용
          ② 하나도 없으면 **보러 간다** — 못 본 자리를 시야에 넣는 가장 싼 포즈

        섞지 않는 이유: 두 이득의 단위가 다르다("묶는다" vs "알게 된다").
        가중치를 만들어 합치면 그 가중치가 곧 숨은 손잡이가 된다. 단계를
        나누면 행동이 설명 가능해진다 — **닿는 건 다 묶고, 없으면 보러 간다.**

        지그재그는 보상 설계 없이 여기서 저절로 나온다 — 횡이동 1회전이
        7.2초인데 같은 거리 주행은 1~2초라, 비용이 알아서 가른다.
        """
        fr = self.frontier()
        best = stay = None
        for ru, rv, turns in self.candidates(fr):
            got = self.covered(ru, rv)
            nf = self.covered_frontier(ru, rv, fr)
            yld = len(got) + FRONTIER_WEIGHT * nf
            if yld <= 0:
                continue
            if abs(ru - self.ru) < 1.0 and turns == 0:
                # 제자리 — 닿는 것은 이미 다 묶었으므로 아무것도 안 바뀐다.
                # 금지하지 않으면 **같은 자리를 계속 고른다** (시뮬에서 2회 발생).
                continue
            pr = self.path_to(ru, rv)
            if pr is None:
                continue
            cst, path = pr
            score = yld / max(cst, 1e-6)
            # 정렬 키: 점수 ↑, 횡이동 적은 쪽, 주행 짧은 쪽
            k = (-score, abs(turns), abs(ru - self.ru))
            item = (k, {'kind': 'tie', 'ru': ru, 'rv': rv, 'turns': turns,
                        'drive_mm': ru - self.ru, 'cells': [c.key() for c, _ in got],
                        'poses': {c.key(): p for c, p in got},
                        'frontier': nf, 'cost_s': cst, 'score': score,
                        'path': path})
            if best is None or k < best[0]:
                best = item
            if turns == 0 and (stay is None or k < stay[0]):
                stay = item
        if best:
            # 레인 유지 — 같은 레인에서 웬만큼 벌 수 있으면 비싼 횡이동을 미룬다
            if stay and stay[1]['score'] >= (1.0 - self.lane_stick) * best[1]['score']:
                return self._centre(stay[1])
            return self._centre(best[1])

        for ru, rv, turns, n in self.explore_candidates(fr):
            if abs(ru - self.ru) < 1.0 and turns == 0:
                continue
            pr = self.path_to(ru, rv)
            if pr is None:
                continue
            cst, path = pr
            # 탐색은 **싼 것부터**. 많이 보는 것보다 가까운 것이 낫다 —
            # 어차피 다 보러 가야 하고, 멀리 질러갔다 오면 그만큼 되돌아온다.
            k = (round(cst, 3), -n, abs(turns))
            if best is None or k < best[0]:
                best = (k, {'kind': 'explore', 'ru': ru, 'rv': rv, 'turns': turns,
                            'drive_mm': ru - self.ru, 'cells': [], 'poses': {},
                            'reveal': n, 'cost_s': cst, 'score': 0.0,
                            'path': path})
        return best[1] if best else None

    def _centre(self, plan):
        """**덮을 것을 고른 뒤, 그 묶음 가운데로 자세를 옮긴다.**

        점수는 "무엇을 덮나" 만 정한다. 같은 것을 덮는 자세가 여럿이면 묶음이
        결속영역 한가운데 오는 쪽이 낫다 — 검출·주행 오차에 대한 여유가
        사방으로 고르게 생기고, 그림도 "한 정차에 3열 × 2행" 으로 읽힌다.

        ⚠ 옮긴 자세가 **같은 셀을 그대로 덮고**, 설 수 있고, 갈 수 있어야만
          받아들인다. 하나라도 어긋나면 원래 자세를 쓴다.
        """
        cells = plan.get('cells') or []
        if len(cells) < 2:
            return plan
        pts = [self.cells[k] for k in cells if k in self.cells]
        if not pts:
            return plan
        cu = sum(c.u for c in pts) / len(pts)
        cv = sum(c.v for c in pts) / len(pts)
        ru = cu - self.reach_mid[0]
        turns = round((cv - self.reach_mid[1] - self.rv) / self.lat_mm)
        rv = self.rv + turns * self.lat_mm
        if abs(ru - plan['ru']) < 1.0 and abs(rv - plan['rv']) < 1.0:
            return plan
        got = {c.key() for c, _p in self.covered(ru, rv)}
        if not set(cells).issubset(got):
            return plan
        pr = self.path_to(ru, rv)
        if pr is None:
            return plan
        cst, path = pr
        plan = dict(plan, ru=ru, rv=rv, turns=turns, cost_s=cst, path=path,
                    drive_mm=ru - self.ru,
                    cells=[c.key() for c, _p in self.covered(ru, rv)])
        return plan

    def snapshot(self):
        return {
            'pose': [round(self.ru, 1), round(self.rv, 1)],
            'origin': None if self.origin is None else [round(x, 1) for x in self.origin],
            'pitch': [None if self.pitch_u is None else round(self.pitch_u, 1),
                      None if self.pitch_v is None else round(self.pitch_v, 1)],
            'counts': self.counts(),
            'cells': [{'ij': list(c.key()), 'uv': [round(c.u, 1), round(c.v, 1)],
                       'state': c.state, 'n': c.n, 'why': c.why}
                      for c in sorted(self.cells.values(), key=lambda x: x.key())],
        }

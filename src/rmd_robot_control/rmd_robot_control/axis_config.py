#!/usr/bin/env python3
"""`axes.yaml` 읽기 — 축 정의를 한 곳에서 꺼낸다.

`homing_node` 와 `stage_node` 가 같은 파일을 각자 읽고 있어서 분리했다. 설정을
어떻게 읽는지는 "원점을 어떤 순서로 찾나"(호밍) 나 "목표까지 어떻게 보내나"(스테이지)
와 다른 관심사다.

⚠ **모터 ID 를 코드에 직접 쓰지 않는다** — 예시로도 쓰지 않는다. 2차년도는 ID 가
  10개 파일에 흩어져 있어 주석과 실제가 어긋났다 (YEAR3_ARCHITECTURE.md §6 R1).
  ID 는 오직 `axes.yaml` 에만 있고 여기서 읽어 문자열로 만든다.
"""

import os

AXIS_NAMES = ('x', 'y', 'z', 'yaw')

HOMING_AXES = {
    # dir 은 2026-09-30 실장비 실측값이다 (`tools/test/axis_direction_check.py`,
    # `tools/test/enc_read.py`). 기본값이 전부 틀려 있었다 — x·y 는 부호가 반대였다.
    # 2026-09-30: 탐색·후퇴 100dps, 정밀 30dps 로 상향 (2차년도 값과 맞춤).
    #   실측 환산으로 보면 탐색 100dps = X 29.1mm/s, Y 22.3mm/s (기존 30dps 는 8.7/6.7).
    #   호밍 시간이 3.3배 줄어든다. 정밀은 30dps 라 리미트 재확정 정밀도는 유지된다.
    'x':   dict(joint=3, home_limit='x_min', far_limit='x_max', dir=+1,
                seek=100.0, back_off=100.0, fine=30.0),
    'y':   dict(joint=4, home_limit='y_min', far_limit='y_max', dir=+1,
                seek=100.0, back_off=100.0, fine=30.0),
    # ⚠ z_min 센서는 **위쪽**에 있다. 이름과 물리 방향이 반대다 (axes.yaml 참조).
    #   음수 명령 → 엔코더 증가 → 상승 → z_min. 중력 반대라 호밍 방향으로 안전하다.
    'z':   dict(joint=5, home_limit='z_min', far_limit='z_max', dir=-1),
    # yaw 는 홈이 **구동범위 한가운데**이고 끝 리미트가 없다. 그래서 `dir` 은 고정값이
    # 의미 없고 **자세 판별로 매번 정한다** (load_pose_id). 방향을 틀리면 에지를 못
    # 만나고 기계 끝단으로 달리므로, 거리 상한(search_limit_gun_deg)으로 막는다.
    # ~~sweep: 스톨 나면 방향 뒤집어 재탐색~~ → 2026-10-03 제거. 그건 **이미 박은 뒤**의
    #   동작이다 (2026-09-30 에 그렇게 10.7A/86°C 까지 갔다).
    # ~~breakaway: 160dps 로 밀어붙이기~~ → 2026-10-03 제거. "저속에서 정지마찰을 못
    #   이긴다" 는 관측은 맞지만 원인 진단이 틀렸다. 속도 제어기는 **명령 속도가
    #   낮아도 오차를 적분해 전류를 올린다** — 12dps 로도 5.4A 까지 올라가 2.4초 뒤
    #   풀리고, 풀린 뒤는 1.2A 다. 필요한 것은 더 센 명령이 아니라 **기다림**
    #   (stall_sec > 돌파 시간)과 **브레이크 해제 확인**이다.
    # ⚠ **fine 을 30 아래로 내리지 말 것.** 느린 단계가 '정밀' 하지 않아 보이지만,
    #   BACK_OFF(30dps) 뒤 FINE 이 **방향을 뒤집어** 출발하므로 정지마찰 + 백래시를
    #   한 번 더 깨야 한다. 12dps 로 내렸다가 2번 자세 호밍이 FINE 스톨로 실패했다
    #   (2026-10-03). 9/30 에도 15dps 로 같은 실패를 봤다.
    #   정밀도는 fine 속도가 아니라 **에지값을 호밍 실측으로 잡는 것**에서 나온다
    #   (건 1.27° → 0.21°). 그 에지값은 fine 속도에 묶이므로, 속도를 바꾸면
    #   `edge_single_*` 을 다시 재야 한다.
    'yaw': dict(joint=6, home_limit='yaw_home', far_limit=None, dir=+1,
                fine=30.0, back_off=30.0),
}



def _stage():
    import yaml
    from ament_index_python.packages import get_package_share_directory
    p = os.path.join(get_package_share_directory('rebar_base_control'),
                     'config', 'axes.yaml')
    return (yaml.safe_load(open(p, encoding='utf-8')) or {}).get('stage', {}) or {}


def load_axis_motor_ids(names=AXIS_NAMES):
    """축 이름 → 위치 토픽에 쓰는 CAN ID 문자열 (소문자 16진, 세 자리).

    못 읽으면 빈 dict. 호출부는 위치 토픽을 못 구독할 뿐 호밍 자체는 리미트로
    동작하므로, 실패해도 죽이지 않는다.
    """
    try:
        stage = _stage()
        out = {}
        for name in names:
            cid = (stage.get(name) or {}).get('can_id')
            if cid is not None:
                out[name] = f"0x{int(cid):03x}"
        return out
    except Exception:
        return {}


def _edges(name):
    """감지 경계 단회전값 → (아래 경계, 위 경계). 없으면 (None, None)."""
    enc = ((_stage().get(name) or {}).get('encoder') or {})
    return (enc.get('edge_single_from_low'), enc.get('edge_single_from_high'))


def load_home_offsets(names=AXIS_NAMES):
    """축별 "에지 도달 후 **탐색 방향별로** 더 갈 각도" — {축: {dir: 모터축 도}}.

    스위치는 **기준점**이지 작업 위치가 아니다. yaw 는 감지 구간이 12시보다 아래라
    에지에서 멈추면 12시가 아니고, 그대로 Y 를 호밍하면 상부 프레임을 친다.

    ⚠⚠ **오프셋은 접근 방향마다 다르다.** 감지판에 폭이 있어서 켜지는 지점이
    다르다 — 건 각도 **증가** 방향으로 접근하면 아래 경계에서, **감소** 방향이면
    위 경계에서 켜진다. 2026-10-03 실장비 실행에서 1번 자세(증가 방향)로 접근했는데
    한 값만 써서 그만큼 비껴갔다.

    손으로 계산한 상수를 두지 않고 **경계 단회전값에서 유도한다** (상수를 두면
    감지판이 움직일 때 또 어긋난다 — 실제로 그렇게 어긋났다):

        offset(모터축) = wrap(noon_single − edge_single) / (cpr/360)

    명령 부호 규약: 명령 + → counts 증가 → 건 각도 증가. wrap 이 양수면 counts 를
    키워야 하므로 오프셋도 양수다.
    """
    try:
        stage = _stage()
        out = {}
        for name in names:
            enc = ((stage.get(name) or {}).get('encoder') or {})
            noon = enc.get('noon_single')
            if noon is None:
                continue
            cpr = int(enc.get('cpr', 262144))
            cpd = cpr / 360.0
            lo, hi = enc.get('edge_single_from_low'), enc.get('edge_single_from_high')
            per = {}
            for d, edge in ((1, lo), (-1, hi)):
                if edge is None:
                    continue
                per[d] = ((int(noon) - int(edge) + cpr // 2) % cpr - cpr // 2) / cpd
            if per:
                out[name] = per
        return out
    except Exception:
        return {}


def load_seek_dirs(names=AXIS_NAMES):
    """축별 탐색 방향(명령 부호). `axes.yaml` 에 있으면 코드 기본값을 덮는다.

    yaw 는 "사용자가 호밍 전에 12시 ±3° 안에 놓는다" 는 전제 덕에 방향이 확정된다
    (12시가 감지 구간보다 위라 항상 counts 감소 방향으로 찾는다).
    """
    try:
        stage = _stage()
        out = {}
        for name in names:
            v = (((stage.get(name) or {}).get('encoder') or {})).get('seek_dir')
            if v is not None:
                out[name] = int(v)
        return out
    except Exception:
        return {}


def load_precheck(names=AXIS_NAMES):
    """호밍 전 전제 검사 정보 — 축별 (목표 단회전값, 허용오차 counts).

    **왜 필요한가:** 사용자가 "호밍 전에 yaw 를 12시 근처에 놓는다" 고 약속했지만,
    약속은 시간이 지나면 어긋난다. 단회전값은 전원과 무관하게 유효하므로 그 약속을
    **기계가 검사**할 수 있다. 전제가 깨진 채로 호밍하면 yaw 탐색 방향이 틀려
    기계 끝으로 밀거나, 12시가 아닌 상태로 Y 가 프레임을 친다.
    """
    CPR = 262144
    try:
        stage = _stage()
        out = {}
        for name in names:
            enc = ((stage.get(name) or {}).get('encoder') or {})
            target = enc.get('noon_single')
            if target is None:
                continue
            tol_gun = float(enc.get('home_tolerance_gun_deg', 3.0))
            gear = float(stage.get('gear', 12.5))
            out[name] = (int(target), tol_gun * (CPR / 360.0) * gear, CPR)
        return out
    except Exception:
        return {}


def precheck_violation(precheck, single, gear=12.5):
    """전제 검사 — 깨져 있으면 사유 문자열, 괜찮으면 None.

    `precheck` 는 `load_precheck()` 결과, `single` 은 축 → 현재 단회전값.
    """
    for name, (target, tol, cpr) in precheck.items():
        cur = single.get(name)
        if cur is None:
            return (f"{name} 단회전값을 못 받고 있다 — "
                    f"/motor_*/encoder_single 이 발행되는지 확인하세요")
        d = cur - target
        if d > cpr / 2:
            d -= cpr
        if d < -cpr / 2:
            d += cpr
        if abs(d) > tol:
            gun = d / (cpr / 360.0) / gear
            return (f"{name} 가 기준 자세에서 건 {gun:+.2f}° 벗어나 있다 "
                    f"(허용 ±{tol/(cpr/360.0)/gear:.1f}°). 손으로 맞춘 뒤 다시 시작하세요")
    return None


def load_max_torque(names=AXIS_NAMES, default=100):
    """축별 `0xA2`/`0xA4` DATA[1] maxTorque. {CAN ID: 값}.

    **정격 전류의 백분율**이다 (1 LSB = 1%). 0 이거나 스톨 전류보다 크면 힘 제어가
    비활성되고 모터 자체 한계만 남는다 — 횡이동이 255 를 쓰는 이유다.

    상부축 기본은 100 인데, yaw 는 2·3번 자세 사이에서 그 상한에 걸려 못 지난다
    (7.6~7.8A 에 막혀 건 0.1°). 그래서 축별로 올릴 수 있게 한다.
    ⚠ **상부축에는 전류 보호(Protection)가 없다** — 이 값이 유일한 상한이므로
    255 로 풀지 말 것.
    """
    try:
        stage = _stage()
        out = {}
        for name in names:
            c = stage.get(name) or {}
            cid = c.get('can_id')
            if cid is None:
                continue
            v = c.get('max_torque', default)
            out[int(cid)] = max(0, min(255, int(v)))
        return out
    except Exception:
        return {}


def load_pose_id(name='yaw'):
    """자세 판별 정보. 없으면 None.

    **왜 자세를 알아야 하나:** yaw 는 리미트 센서가 하나뿐이고 가동범위가 1.23바퀴다.
    게다가 에지가 자세 범위 **안쪽**(12시 -4.62°)에 있어서, 시작 자세에 따라 탐색
    방향이 ± 두 가지다. 한 방향으로만 탐색하면 반대쪽 자세에서 출발했을 때 에지를
    못 만나고 **기계 끝단으로 달린다**(+ 로만 하면 3·4번, - 로만 하면 1·2번).

    멀티턴은 전원에 날아가지만 단회전은 복원되고, 12시와 1~4번의 단회전값이 최소
    건 4.54° 떨어져 있어 유일하게 갈린다. 그래서 **전원을 올린 직후에도 자세를
    알 수 있고**, 사용자가 호밍 전에 yaw 를 손으로 12시에 맞출 필요가 없다.

    돌려주는 각 항목: gun(12시 기준 건 각도), single(기대 단회전값),
    dir(탐색 명령 부호 — 에지가 위에 있으면 +1).
    """
    try:
        stage = _stage()
        cfg = stage.get(name) or {}
        enc = cfg.get('encoder') or {}
        noon = enc.get('noon_single')
        poses = enc.get('pose_offset_from_noon_gun_deg')
        if noon is None or not poses:
            return None
        cpr = int(enc.get('cpr', 262144))
        gear = float(stage.get('gear', 12.5))
        cpg = (cpr / 360.0) * gear               # counts / 건 1도
        # 에지가 12시 기준 건 몇 도인가 — **두 경계의 중앙**을 쓴다. 자세가 에지보다
        # 위/아래인지만 보면 되므로 판 폭(건 0.45°)은 판정에 영향이 없다.
        lo, hi = enc.get('edge_single_from_low'), enc.get('edge_single_from_high')
        es = [int(x) for x in (lo, hi) if x is not None]
        if not es:
            return None
        edge_gun = sum(((e - int(noon) + cpr // 2) % cpr - cpr // 2)
                       for e in es) / len(es) / cpg
        tol_gun = float(enc.get('pose_id_tolerance_gun_deg', 2.0))
        cand = {0: 0.0}                          # 0 = 12시 (호밍 직후 자세)
        cand.update({int(k): float(v) for k, v in poses.items()})
        out = {}
        for n, g in cand.items():
            out[n] = dict(gun=g,
                          single=int(round((noon + g * cpg) % cpr)),
                          dir=1 if g < edge_gun else -1)
        # 후보를 걸러낼 구동범위 — 자세 범위 양쪽으로 여유를 둔다.
        # ⚠ 기계 끝단은 미측정이다. **크게 잡으면 거부가 늘어 안전**하고, 작게
        #   잡으면 진짜 후보를 버려 틀린 쪽을 받아들인다.
        pad = float(enc.get('gun_range_pad_deg', 3.0))
        gs = [p['gun'] for p in out.values()]
        return dict(poses=out, cpr=cpr, cpg=cpg, noon=int(noon), gear=gear,
                    edge_gun=edge_gun, tol_gun=tol_gun, tol=tol_gun * cpg,
                    gun_lo=min(gs) - pad, gun_hi=max(gs) + pad, pad=pad,
                    period=360.0 / gear,          # 단회전이 접히는 주기 (건 28.8°)
                    limit_gun=float(enc.get('search_limit_gun_deg', 26.0)))
    except Exception:
        return None


def load_ready_yaw_pose():
    """준비자세의 yaw 자세 번호 — 호밍이 끝나면 yaw 가 서는 자세. 없으면 None."""
    try:
        v = (_stage().get('ready') or {}).get('yaw_pose')
        return None if v is None else int(v)
    except Exception:
        return None


def gun_from_anchor(info, anchor, multi):
    """기준점 `(멀티턴, 건각)` + 지금 멀티턴 → 지금 건 각도. 못 구하면 None.

    ⚠⚠ **단회전만으로는 자세를 못 가린다.** 단회전은 모터 1회전(건 28.8°)마다
    접히고 yaw 구동범위는 1.23바퀴라, **별칭이 엉뚱한 자세 옆에 떨어지는 각도가
    실제로 존재한다.** 2026-10-03 에 건 +10.80° 를 1번 자세(−16.74°)로 읽었다 —
    별칭 −18.00° 가 1번에서 1.26° 떨어져 허용오차 1.5° 안에 들었다. 그 상태로
    회전을 시키면 27.6° 를 지나쳐 밀고, yaw 는 끝 리미트가 없다.
    허용오차를 0.4° 까지 조여도 그런 구간이 남는다 — **원리적 한계다.**

    멀티턴은 전원 세션 안에서 접히지 않는다. 기준점 하나만 있으면 유일하다.
    기준점은 **각도를 아는 순간**에 잡는다: 호밍 완료(준비자세) 또는 자세 회전 성공.
    (전원이 꺼지면 멀티턴이 날아가므로 기준점도 무효다 → 재호밍.)
    """
    if not info or not anchor or multi is None:
        return None
    m0, g0 = anchor
    return g0 + (multi - m0) / info['cpg']


def pose_from_gun(info, gun):
    """건 각도 → (자세번호, 상세) 또는 (None, 사유). **별칭이 없다.**"""
    if not info or gun is None:
        return None, '건 각도를 모른다 (멀티턴 기준점이 없다)'
    best, bd = None, None
    for n, p in info['poses'].items():
        d = gun - p['gun']
        if bd is None or abs(d) < abs(bd):
            best, bd = n, d
    if abs(bd) > info['tol_gun']:
        return None, (f"자세 사이다 — 건 {gun:+.2f}°, 가장 가까운 {pose_label(best)} "
                      f"에서 {bd:+.2f}° (허용 ±{info['tol_gun']:.2f}°)")
    out = dict(info['poses'][best])
    out['err_gun'], out['gun_now'] = bd, gun
    return best, out


def pose_label(n):
    return '12시' if n == 0 else f'{n}번 자세'


def identify_pose(info, single):
    """단회전값 → (자세번호, 상세) 또는 (None, 거부 사유).

    ⚠⚠ **단회전은 모터 1회전(건 28.8°)마다 접히고 yaw 전행정은 그보다 넓다.**
    그래서 한 단회전값에 **구동범위 안의 후보가 둘** 생기는 각도가 있다:
    1번(−16.74°)↔+12.06° · 4번(+16.69°)↔−12.11°. 2026-10-03 에 건 +10.80° 를
    1번으로, 건 −11.71° 를 4번으로 읽었다. 그대로 돌리면 20~28° 를 지나쳐 밀고
    yaw 는 끝 리미트가 없다. **허용오차를 조여도 그 구간은 남는다.**

    그래서 후보를 구동범위로 걸러내고 **둘 남으면 거부한다.** 12시·2번·3번은
    별칭이 범위 밖이라 유일하게 갈리고 1번·4번만 거부된다. 확실히 알아야 하면
    멀티턴 기준점을 쓴다 (`gun_from_anchor`).

    자세 사이에 있어도 거부한다 — 모르는 채로 탐색하면 끝단에 박는다.
    """
    if not info:
        return None, "자세 판별 정보가 없다 (axes.yaml 의 noon_single / pose_offset 확인)"
    if single is None:
        return None, ("단회전값을 못 받고 있다 — "
                      "/motor_*/encoder_single 이 발행되는지 확인하세요")
    # 이 단회전값이 가리킬 수 있는 건 각도 후보들 (28.8° 주기)
    cpr, per = info['cpr'], info['period']
    base = ((single - info['noon'] + cpr // 2) % cpr - cpr // 2) / info['cpg']
    cands = [base + k * per for k in (-2, -1, 0, 1, 2)]
    cands = [g for g in cands if info['gun_lo'] <= g <= info['gun_hi']]
    if not cands:
        return None, (f"단회전 {single} 이 구동범위 밖을 가리킨다 "
                      f"(건 {base:+.2f}°, 범위 {info['gun_lo']:+.1f}~"
                      f"{info['gun_hi']:+.1f}°) — 기구나 기준값을 확인하세요")
    if len(cands) > 1:
        return None, ("단회전만으로는 자세를 가릴 수 없다 — 후보가 "
                      + " / ".join(f"건 {g:+.2f}°" for g in sorted(cands))
                      + f" 둘 다 구동범위 안이다 (별칭 주기 {per:.1f}°). "
                      "12시·2번·3번 자세로 옮긴 뒤 다시 하거나, "
                      "/stage/yaw_declare 로 지금 자세를 알려주세요")
    gun = cands[0]
    best, bd = None, None
    for n, p in info['poses'].items():
        d = gun - p['gun']
        if bd is None or abs(d) < abs(bd):
            best, bd = n, d
    if abs(bd) > info['tol_gun']:
        return None, (f"어느 자세에도 맞지 않는다 — 건 {gun:+.2f}°, 가장 가까운 "
                      f"{pose_label(best)} 에서 {bd:+.2f}° "
                      f"(허용 ±{info['tol_gun']:.2f}°). "
                      f"1~4번 자세나 12시로 옮긴 뒤 다시 시작하세요")
    p = dict(info['poses'][best])
    p['err_gun'] = bd
    p['gun_now'] = gun
    p['to_edge_gun'] = info['edge_gun'] - p['gun']
    return best, p


def load_search_limit(name='yaw'):
    """탐색 거리 상한 (모터축 도). 없으면 None.

    yaw 는 리미트가 하나뿐이라 **잘못된 방향으로 달리면 기계 끝단에 박는다.**
    어느 자세에서든 에지까지 최대 건 23.42° 이므로, 그보다 넉넉한 값을 넘기면
    방향이 틀렸거나 기구 이상이다. **그때는 방향을 뒤집지 말고 멈춘다** —
    역방향 재탐색은 이미 박은 뒤의 동작이다 (2026-09-30: 10.7A/86°C).
    """
    try:
        stage = _stage()
        enc = ((stage.get(name) or {}).get('encoder') or {})
        v = enc.get('search_limit_gun_deg')
        if v is None:
            return None
        return float(v) * float(stage.get('gear', 12.5))
    except Exception:
        return None


def load_ready_pose():
    """준비자세 — 축별 **원점 레퍼런스로부터의 토픽 각도 차이**와 이동 순서.

    `/motor_*_position` 토픽은 counts 부호와 반대다(토픽 = -counts/728). 그래서
    **명령 부호가 양수면 토픽은 감소한다.** 아래 부호는 그 규약에 맞춰 계산했다.

    · Y: mm 는 원점(y_min) 기준이고 `mm_per_deg` 가 부호를 흡수한다 →
         `ref + mm / mm_per_deg`. stage_node 의 `deg_of()` 와 같은 식이다.
    · yaw: 레퍼런스는 **에지**에 기록되고, 에지→12시 오프셋은 **접근 방향마다 다르다.**
         그래서 여기서 숫자를 못 낸다 — 자세의 건 각도만 돌려주고, 호밍 노드가
         **실제 적용한 오프셋**과 합친다:
             토픽차 = −(적용 오프셋) − (자세 건각도 × gear)

    돌려주는 것: (순서 리스트, {선형축: 토픽각도차}, yaw 자세 **모터축** 각도, 속도dps)
    """
    try:
        stage = _stage()
        r = stage.get('ready') or {}
        gear = float(stage.get('gear', 12.5))
        out = {}
        order = [str(x) for x in (r.get('order') or [])]
        if 'y' in order:
            mpd = (stage.get('y') or {}).get('mm_per_deg')
            if not mpd:
                order.remove('y')        # 환산을 모르면 Y 는 건너뛴다
            else:
                out['y'] = float(r['y_mm']) / float(mpd)
        yaw_gun = None
        if 'yaw' in order:
            enc = ((stage.get('yaw') or {}).get('encoder') or {})
            poses = enc.get('pose_offset_from_noon_gun_deg') or {}
            key = r.get('yaw_pose')
            g = poses.get(key, poses.get(str(key)))
            if g is None:
                order.remove('yaw')
            else:
                yaw_gun = float(g) * gear          # 모터축으로 환산해 넘긴다
        return order, out, yaw_gun, float(r.get('speed_dps', 60))
    except Exception:
        return [], {}, None, 60.0


def ready_target(axis, ref, lin_off, yaw_motor, off_used):
    """준비자세 목표 (위치 토픽 각도). 못 구하면 None.

    선형축은 `ref + 토픽각도차` 로 끝난다. yaw 는 레퍼런스가 **에지**에 기록되고
    에지→12시 오프셋이 **접근 방향마다 다르므로**, 실제 적용한 오프셋에서
    거꾸로 계산한다:

        목표 = ref − (적용 오프셋) − (자세 모터축 각도)

    명령 + 는 토픽을 줄이므로 둘 다 부호를 뒤집어 더한다.
    """
    if ref is None:
        return None
    if axis in lin_off:
        return ref + lin_off[axis]
    if axis == 'yaw' and yaw_motor is not None and off_used is not None:
        return ref - off_used - yaw_motor
    return None


def load_envelope():
    """자세별 X·Y 가동 범위. {'poses': {n: {축: (lo, hi)}}, 'any': {축: (lo, hi)}}.

    **yaw 자세에 따라 X·Y 가동 범위가 다르다** — 결속건이 회전하며 간섭 방향이
    바뀐다. 이 표가 없으면 검출 지점으로 보낼 때 프레임을 친다.

    `margin_mm` 을 양쪽에서 깎아 돌려준다. `any` 는 자세와 **표본**을 모두 겹친
    교집합이고, 자세를 못 가릴 때 쓰는 보수적 범위다 — 어느 각도에서든 안전하다.

    `samples` 는 자세 **사이** 각도의 실측이다. 결속에 쓰는 자세가 아니므로
    `poses` 와 섞지 않고(자세별 강제·자세 선택에는 안 들어간다) 회전 안전창과
    `any` 에만 쓴다. ⚠ **가동범위는 자세 사이에서 단조롭지 않다** — 양 끝만
    재서 추정하면 틀린다 (12시 X 22mm, 건 +10.8° Y 65mm 차이가 실측됐다).
    표본은 **잰 축만** 제약한다.
    """
    try:
        env = (_stage().get('envelope') or {})
        poses = env.get('poses') or {}
        if not poses:
            return None
        m = float(env.get('margin_mm', 0.0))
        # ⚠ **여유는 간섭 경계에만 적용한다.** 기계 끝단(원점·스트로크 끝)은
        #   리미트 스위치가 지키는 하드 스톱이지 실측한 간섭선이 아니다. 거기서
        #   10mm 를 깎으면 안전에 기여하지 않고 정상 동작만 막는다 —
        #   2026-10-03 에 호밍 직후 X=-3mm(원점)에서 자세 변경이 거부됐다.
        #   끝단 쪽은 **제약 없음(±inf)** 으로 둔다.
        stage = _stage()
        stroke = {a: (stage.get(a) or {}).get('stroke_mm') for a in ('x', 'y', 'z')}
        INF = float('inf')

        def bound(a, lo, hi):
            lo2 = -INF if lo <= 0.0 else lo + m
            st = stroke.get(a)
            hi2 = INF if (st and hi >= float(st) - 0.5) else hi - m
            return (lo2, hi2)

        out = {}
        for n, axes in poses.items():
            out[int(n)] = {a: bound(a, float(v[0]), float(v[1]))
                           for a, v in axes.items()}
        samples = []
        for sm in (env.get('samples') or []):
            rng = {a: bound(a, float(v[0]), float(v[1]))
                   for a, v in sm.items() if a in ('x', 'y')}
            if rng:
                samples.append({'gun': float(sm['gun']), 'range': rng,
                                'note': str(sm.get('note', ''))})
        # 교집합은 자세 + 표본 전부를 겹친다 — 자세를 모르면 그 사이일 수도 있다
        pools = list(out.values()) + [sm['range'] for sm in samples]
        any_ = {}
        for a in set().union(*(set(v) for v in pools)):
            any_[a] = (max(v[a][0] for v in pools if a in v),
                       min(v[a][1] for v in pools if a in v))
        return {'poses': out, 'any': any_, 'margin': m, 'samples': samples}
    except Exception:
        return None


def envelope_for(env, pose):
    """그 자세에서 적용할 X·Y 범위와 설명. (범위, 설명).

    표에 없는 자세(판별 실패, 또는 12시처럼 안 잰 자세)는 **교집합**으로 본다 —
    모르면 좁게 잡는다. ⚠ 강제와 표시가 **반드시 같은 값**을 써야 한다. 따로
    계산했다가 12시에서 표시만 빈 값이 나온 일이 있다 (2026-10-03).
    """
    if not env:
        return None, '범위 표 없음'
    lim = env['poses'].get(pose) if pose is not None else None
    if lim is not None:
        return lim, pose_label(pose)
    why = ('자세 미확인' if pose is None
           else f'{pose_label(pose)} 는 범위 미측정')
    return env['any'], f'{why} → 교집합'


def envelope_violation(env, pose, want):
    """목표가 가동 범위를 벗어나는가. 벗어나면 사유 문자열, 괜찮으면 None."""
    if not env:
        return None
    lim, where = envelope_for(env, pose)
    for a, v in want.items():
        rng = lim.get(a)
        if rng is None:
            continue
        if not (rng[0] <= v <= rng[1]):
            return (f"{a}={v:.1f}mm 가 가동 범위 밖 ({where}: "
                    f"{rng[0]:.1f}~{rng[1]:.1f}mm, 여유 {env['margin']:.0f}mm 포함)")
    return None


def transit_window(env, info, a, b):
    """자세 a → b 로 **회전하는 동안** XY 가 있어야 하는 범위. (범위, 통과 자세들).

    회전은 중간 자세를 **지나간다** — 1번에서 3번으로 가면 12시와 2번을 지난다.
    그래서 지나가는 자세 전부의 **교집합** 안에 있어야 한다. 양 끝 자세만 보면
    사이에 있는 최악값을 놓친다 — 2026-10-03 에 12시(X 361.0mm)를 안 재고 3번
    (383.2)을 최악으로 써서 X 상한이 22mm 과했다.

    범위를 못 구하면 네 자세 교집합(`any`)으로 떨어진다 — 모르면 좁게 잡는다.
    """
    if not env or not info:
        return None, []
    gun = {n: p['gun'] for n, p in info['poses'].items()}
    if a not in gun or b not in gun:
        return env['any'], []
    lo, hi = sorted((gun[a], gun[b]))
    pools, labels = [], []
    for n, g in sorted(gun.items(), key=lambda kv: kv[1]):
        if lo <= g <= hi and n in env['poses']:
            pools.append(env['poses'][n])
            labels.append(pose_label(n))
    # 자세 사이 표본도 지나간다 — 이게 빠지면 양 끝만 보고 추정하는 셈이다
    for sm in env.get('samples') or []:
        if lo <= sm['gun'] <= hi:
            pools.append(sm['range'])
            labels.append(f"건 {sm['gun']:+.1f}°")
    if not pools:
        return env['any'], []
    win = {}
    for ax in ('x', 'y'):
        vals = [p[ax] for p in pools if ax in p]
        win[ax] = (max(v[0] for v in vals), min(v[1] for v in vals))
    return win, labels


def load_pose_select():
    """결속 자세 선택 규칙. 없으면 None.

    {'x_mid','y_mid','dead','quadrant'} — quadrant 는 ('x_max'|'x_min',
    'y_max'|'y_min') → 자세번호.
    """
    try:
        cfg = (_stage().get('pose_select') or {})
        q = cfg.get('quadrant') or {}
        if not q or cfg.get('x_mid_mm') is None or cfg.get('y_mid_mm') is None:
            return None
        quad = {}
        for k, v in q.items():
            xs, ys = k.split('_y_')
            quad[(xs, 'y_' + ys)] = int(v)
        return {'x_mid': float(cfg['x_mid_mm']), 'y_mid': float(cfg['y_mid_mm']),
                'dead': float(cfg.get('deadband_mm', 0.0)), 'quadrant': quad}
    except Exception:
        return None


def select_pose(sel, x_mm, y_mm, current=None):
    """결속 지점 (x,y)mm → (자세번호, 사유). 바꿀 필요가 없으면 자세 = current.

    사분면으로 고른다 — X 가 중앙보다 xmax 쪽이면 1·4번, xmin 쪽이면 2·3번,
    Y 가 ymin 쪽이면 1·2번, ymax 쪽이면 3·4번. 둘을 겹치면 하나로 떨어진다.

    **중앙 ±deadband 안은 "절반지점"** 으로 보고 자세를 바꾸지 않는다 (사용자 지정:
    "정확히 절반지점에서는 그냥 당시 자세로"). 한 축만 절반이면 그 축만 현재
    자세를 따르고 나머지 축으로는 못 고르므로 역시 current 를 쓴다.
    """
    if not sel:
        return current, "선택 규칙이 없다 (axes.yaml 의 stage.pose_select)"
    if x_mm is None or y_mm is None:
        return current, "목표 X·Y 가 둘 다 있어야 자세를 고를 수 있다"

    def side(v, mid, lo, hi):
        if abs(v - mid) <= sel['dead']:
            return None
        return hi if v > mid else lo

    xs = side(x_mm, sel['x_mid'], 'x_min', 'x_max')
    ys = side(y_mm, sel['y_mid'], 'y_min', 'y_max')
    if xs is None or ys is None:
        half = ' · '.join(n for n, v in (('X', xs), ('Y', ys)) if v is None)
        return current, (f"{half} 가 절반지점(±{sel['dead']:.0f}mm) — "
                         f"현재 자세 유지")
    want = sel['quadrant'].get((xs, ys))
    if want is None:
        return current, f"사분면 {xs}/{ys} 에 배정된 자세가 없다"
    return want, (f"X {x_mm:.1f}mm {'>' if xs == 'x_max' else '<'} "
                  f"{sel['x_mid']:.1f} · Y {y_mm:.1f}mm "
                  f"{'>' if ys == 'y_max' else '<'} {sel['y_mid']:.1f} "
                  f"→ {pose_label(want)}")


def load_stage_axes(names=('x', 'y', 'z')):
    """스테이지 이동에 필요한 축 정보 (관절 번호·모터·mm 환산·브레이크)."""
    stage = _stage()
    out = {}
    for name in names:
        cfg = stage.get(name) or {}
        joint = str(cfg.get('joint', ''))
        cid = cfg.get('can_id')
        out[name] = dict(
            joint=int(joint.split('_')[-1]) if '_' in joint else None,
            motor=f"0x{int(cid):03x}" if cid is not None else None,
            mm_per_deg=cfg.get('mm_per_deg'),
            home_dir=int(cfg.get('home_dir', 1)),
            brake=bool(cfg.get('brake', False)),
            gravity=bool(cfg.get('gravity_load', False)),
        )
    return out

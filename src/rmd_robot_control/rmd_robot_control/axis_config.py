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
    # yaw 는 홈이 **구동범위 한가운데**라 고정 부호가 의미 없다. 끝 리미트도 없다.
    # 한 방향으로 쓸다가 스톨(기계 끝)이 나면 반대로 뒤집는다 — 전 구간이 건 35°뿐이라
    # 최악이어도 금방 찾는다.
    # ⚠ yaw 는 **저속에서 정지마찰을 못 이긴다.** 2026-09-30 실측:
    #   15dps → 3% 밖에 안 움직이고, 30dps → 정상. 탐색(30)은 되는데 후퇴·정밀(15)에서
    #   스톨로 실패했다. 그래서 이 축만 느린 단계도 30dps 로 올린다.
    'yaw': dict(joint=6, home_limit='yaw_home', far_limit=None, dir=+1, sweep=True,
    #   또 하나: **정지 상태에서 출발할 때만** 세게 밀어야 한다. 속도 제어기가 명령
    #   속도에 비례해서만 전류를 올려서 (30dps→2.2A, 50→3.1A, 80→3.7A), 낮은 속도로는
    #   정지마찰을 못 깬다. 한 번 움직이면 30dps 로도 유지된다 → breakaway 로 처리한다.
    #   ⚠ 이탈 속도는 **홈 근처 기준**으로 정해야 한다. 홈에서 먼 곳은 80dps 에 풀리지만
    #     홈 근처(건 2° 이내)는 80·120 에서 전류가 1.98A 에 머물며 꿈쩍도 안 하고,
    #     160dps 에서 4.46A 로 올라가며 풀린다 (2026-09-30 실측, 정격 6.1A 이내).
                fine=30.0, back_off=30.0, fine_stall_ok=True, breakaway=160.0),
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


def load_home_offsets(names=AXIS_NAMES):
    """축별 "에지 도달 후 탐색 방향으로 더 갈 각도"(모터축 도).

    스위치는 **기준점**이지 작업 위치가 아니다. yaw 는 감지판이 2번 자세 쪽으로
    옮겨져 12시에서 센서가 안 켜진다 — 에지에서 멈추면 12시가 아니다. 그대로 Y 를
    호밍하면 상부 프레임을 친다 (Y 는 yaw 12시에서만 리미트에 닿는다).
    """
    try:
        stage = _stage()
        out = {}
        for name in names:
            v = (((stage.get(name) or {}).get('encoder') or {})).get('home_offset_deg')
            if v:
                out[name] = float(v)
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

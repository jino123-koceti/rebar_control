#!/usr/bin/env python3
"""주행모터 좌우 짝 점검 — 좌우 설정이 벌어졌는지 한 줄로 확인한다.

## 왜 (2026-09-09)
0x141(X4-10)이 스톨·과열로 죽어 **X4-36**으로 교체했다. 좌우 모델이 달라지면
정상속도가 같아도 아래 셋이 어긋나서 직진이 틀어진다:

  ① 가감속(0x43 index 2/3)  — 램프 구간에서 좌우가 벌어진다
  ② 토크 상한(0xA2 DATA[1]) — **정격전류의 %** 라 정격이 다르면 절대토크가 다르다
  ③ 최고속                   — 느린 쪽이 먼저 포화하면 그때부터 돈다

## ★ 2026-09-14 — **양쪽 다 X4-36으로 통일**
혼합 구성은 저속 25초에 X4-10이 96°C까지 타서 폐기했다. 이제 좌우가 같은 모델이므로
**모든 설정이 좌우 동일**해야 한다. 그런데 혼합 시절의 *비대칭 보정값*이 남아 있으면
(가감속 0x141=1200 / 0x142=500, 토크 69% / 180%) 오히려 그게 직진을 틀어버린다.
그래서 이 도구는 이제 두 가지를 본다:

  · 좌우가 **서로** 같은가
  · 그 값이 **목표값**(TARGET_ACCEL)과 같은가 ← 둘 다 낡은 값이면 위만으론 못 잡는다

⚠ **정지 상태에서** 실행할 것. `--fix` 없이는 읽기만 하고 아무것도 쓰지 않는다.

## 사용
    python3 tools/motor/check_drive_pair.py
    python3 tools/motor/check_drive_pair.py --fix              # 양쪽을 목표값으로 정렬
    python3 tools/motor/check_drive_pair.py --fix --accel 900  # 목표값을 바꿔서 정렬
"""
import argparse
import struct
import subprocess
import sys
import time

import can
import yaml

CFG = '/home/koceti/ros2_ws/src/rebar_base_control/config/can_devices.yaml'
IDX = {0x02: '속도계획 가속', 0x03: '속도계획 감속'}

# ★ 목표 가감속 (dps/s). **`scripts/service/robot_control_service.sh` 와 같은 값**이어야
#   한다 — 실제로 매 기동마다 모터에 써넣는 주체는 그 스크립트다. X4-36은 0x43이
#   ROM에 안 남아(공장기본 5000으로 원복) 재적용이 필수이고, 여기서 --fix로 쓴 값도
#   재부팅하면 날아간다. **튜닝 값을 확정했으면 서비스 스크립트도 같이 고칠 것.**
# ★★ [2026-09-14 저녁] **0x43은 출력축이 아니라 모터축 기준이다.** 실측으로 확정:
#   지령을 3초간 고정했을 때 실제 가속도가 35.7 출력축 dps/s 였는데
#   1200 ÷ 36(감속비) = 33.3 과 일치한다. 다른 후보는 전부 어긋난다
#   (출력축 해석이면 1200 → 34배, 슬루 0.25m/s² 면 500 → 14배).
#   방증: 혼합 시절 현장에서 실측으로 맞춘 비대칭 1200(X4-36)/500(X4-10) = 2.4배 ≈
#         기어비 36/12.5 = 2.88배. 그 보정의 정체가 기어비였다.
#   ⟹ **같은 설정값이라도 모델이 다르면 실효 램프가 다르다.** 목표값을 정할 땐
#      "출력축에서 몇 dps/s 를 원하는가"를 먼저 정하고 감속비를 곱한다.
RATIO = SPEC_RATIO = 36.0        # 현재 장착 모델(X4-36)의 감속비

# 가속: X4-10 시절 0x43=1200 이 출력축 96 dps/s 였고 그 값으로 전류 피크가 검증됐다
#       ([[robot_freeze_safety]]). 같은 **출력축 램프**를 복원한다 → 96 × 36.
TARGET_ACCEL = 3456
# 감속: 회생이라 전류 피크 제약이 없다. 정지거리를 직접 줄이는 값이다.
#       12000 → 출력축 333 dps/s → 0.078 m/s 에서 정지거리 약 18mm.
#       (1200이면 180mm. 이게 "관성으로 한참 간다"의 정체였다.)
TARGET_DECEL = 12000

# ★ 현재 장착된 모터 모델. **교체하면 여기를 고친다** — 토크 %를 절대 N·m로
#   환산하는 데만 쓴다(같은 %라도 정격이 다르면 절대토크가 다르기 때문).
MODEL = {'좌': 'X4-36', '우': 'X4-36'}

# 데이터시트 (can_devices.yaml 주석에서)
SPEC = {'X4-10': dict(ratio=12.5, rated_nm=4.0, max_nm=10.0, rated_rpm=238),
        'X4-36': dict(ratio=36.0, rated_nm=10.5, max_nm=34.0, rated_rpm=83)}


def txrx(bus, mid, data, want_idx, timeout=0.5, retries=6):
    """0x42 응답만 골라 받는다 — can_sender가 같은 버스를 폴링해 프레임이 섞인다."""
    for _ in range(retries):
        bus.send(can.Message(arbitration_id=mid, data=data, is_extended_id=False))
        t0 = time.time()
        while time.time() - t0 < timeout:
            m = bus.recv(timeout=0.1)
            if (m and m.arbitration_id == mid + 0x100
                    and m.data[0] == 0x42 and m.data[1] == want_idx):
                return struct.unpack('<i', bytes(m.data[4:8]))[0]
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--channel', default='can2')
    ap.add_argument('--accel', type=int, default=TARGET_ACCEL,
                    help=f'목표 가속 dps/s (기본 {TARGET_ACCEL})')
    ap.add_argument('--decel', type=int, default=TARGET_DECEL,
                    help=f'목표 감속 dps/s (기본 {TARGET_DECEL})')
    ap.add_argument('--fix', action='store_true',
                    help='좌우 **양쪽**을 목표값으로 맞춘다 (X4-36은 재부팅 시 날아감)')
    a = ap.parse_args()

    cfg = yaml.safe_load(open(CFG))['can_sender']['ros__parameters']
    lid, rid = cfg['left_motor_id'], cfg['right_motor_id']
    mt, lt, rt = (cfg['max_torque_pct'],
                  cfg.get('left_max_torque_pct', -1),
                  cfg.get('right_max_torque_pct', -1))
    lt = mt if lt < 0 else lt
    rt = mt if rt < 0 else rt
    rpm = cfg.get('wheel_max_speed_rpm', 238.0)
    circ = 2 * 3.14159265 * cfg['wheel_radius']

    bus = can.interface.Bus(channel=a.channel, interface='socketcan')
    acc = {mid: {i: txrx(bus, mid, [0x42, i, 0, 0, 0, 0, 0, 0], i) for i in IDX}
           for mid in (lid, rid)}

    print(f'\n{"":22}{"0x%03X (좌)" % lid:>16}{"0x%03X (우)" % rid:>16}'
          f'   [목표 가속 {a.accel} / 감속 {a.decel}]')
    print(f'{"모델":22}{MODEL["좌"]:>16}{MODEL["우"]:>16}')
    print('-' * 68)

    # 인덱스마다 목표가 다르다 — 가속(2)과 감속(3)은 성격이 반대다.
    tgt_of = {0x02: a.accel, 0x03: a.decel}

    mismatched, off_target = [], []
    for i, name in IDX.items():
        tgt = tgt_of[i]
        l, r = acc[lid][i], acc[rid][i]
        ls = '응답없음' if l is None else f'{l} dps/s'
        rs = '응답없음' if r is None else f'{r} dps/s'
        same = (l is not None and l == r)
        on_tgt = (l == tgt and r == tgt)
        mark = ('✅' if (same and on_tgt)
                else (f'⚠ 목표 {tgt} 와 다름' if same else '❌ 좌우 불일치'))
        out = '' if l is None else f'  (출력축 {l/RATIO:.0f} dps/s)'
        print(f'{name:22}{ls:>16}{rs:>16}   {mark}{out}')
        if not same:
            mismatched.append((name, l, r))
        elif not on_tgt:
            off_target.append((name, l, tgt))

    tq_same = (lt == rt)
    print(f'{"토크 상한(%)":22}{lt:>16}{rt:>16}   '
          f'{"✅" if tq_same else "❌ 좌우 불일치"}')
    print(f'{"속도 상한":22}{rpm*6:>13.0f} dps{rpm*6:>13.0f} dps')
    print(f'{"":22}{rpm/60*circ:>13.3f} m/s{rpm/60*circ:>13.3f} m/s')
    print('-' * 68)

    # 토크 상한은 **정격전류의 %** 라, 모델을 알아야 절대 N·m 가 나온다.
    print('\n[토크 절대값]  같은 %라도 모델이 다르면 절대토크가 다르다')
    nm = {}
    for side, pct in (('좌', lt), ('우', rt)):
        nm[side] = SPEC[MODEL[side]]['rated_nm'] * pct / 100
        print(f'   {side} {MODEL[side]} {pct:4d}%  →  {nm[side]:.1f} N·m')
    if abs(nm['좌'] - nm['우']) > 0.05:
        print(f'   ❌ 좌우 절대토크 불일치 — can_devices.yaml 의 '
              f'left/right_max_torque_pct 를 맞출 것')
    else:
        print('   ✅ 좌우 절대토크 일치')

    if mismatched:
        print('\n❌ 가감속 좌우 불일치 — 램프마다 차체가 틀어진다:')
        for name, l, r in mismatched:
            print(f'   {name}: 좌 {l} vs 우 {r}')
    if off_target:
        print('\n⚠ 좌우는 같지만 목표와 다름:')
        for name, v, tgt in off_target:
            print(f'   {name}: {v} (목표 {tgt})')

    if mismatched or off_target:
        if a.fix:
            print(f'\n▶ 좌우 **양쪽**을 가속 {a.accel} / 감속 {a.decel} dps/s 로 맞춘다')
            for mid in (lid, rid):
                for i in IDX:
                    subprocess.run([sys.executable,
                                    '/home/koceti/ros2_ws/tools/motor/rmd_accel.py',
                                    '--ids', hex(mid), '--set', str(tgt_of[i]),
                                    '--index', str(i)])
            print('\n다시 읽어 확인하세요: python3 tools/motor/check_drive_pair.py')
            print('⚠ X4-36은 ROM에 안 남는다 — 값을 확정했으면 '
                  'scripts/service/robot_control_service.sh 도 같이 고칠 것')
        else:
            print('   → 맞추려면  python3 tools/motor/check_drive_pair.py --fix')
    elif tq_same:
        print('\n✅ 가감속·토크 좌우 일치, 목표값과도 일치')

    print('\n다음: python3 tools/motor/straight_drive_test.py --watch  '
          '(리모콘 주행하며 정지거리·좌우편차 측정)')


main()

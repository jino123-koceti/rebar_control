#!/usr/bin/env python3
"""주행모터(0x141/0x142) **명령속도 vs 실제속도** 추종성 분석.

## 왜
"리모콘으로 얼마의 속도를 명령하고, 실제로 그 속도에 도달하는가"를 확인한다.
ROS 토픽을 거치지 않고 **CAN 프레임을 직접** 읽으므로 변환·스케일 오차가 끼지 않는다.

## 프레임 (RMD V4.3)
  송신 0x141/0x142: [0xA2, maxTorque, 0, 0, spd_b0..b3]  int32, **0.01 dps/LSB**
  응답 0x241/0x242: [0xA2, temp, iq_lo, iq_hi, spd_lo, spd_hi, ang_lo, ang_hi]
                    speed int16, **1 dps/LSB**  /  iq int16, 0.01 A/LSB

⚠ 단위가 다르다(명령 0.01 dps, 응답 1 dps). 이걸 맞추지 않으면 100배 어긋난다.

사용:
    python3 tools/diag/drive_speed_probe.py            # 20초 요약
    python3 tools/diag/drive_speed_probe.py --sec 60 --live
"""
import argparse
import math
import struct
import time
from collections import defaultdict

WHEEL_RADIUS = 0.02865   # m (config can_devices.yaml)


def dps_to_mps(dps):
    return dps / 360.0 * (2 * math.pi * WHEEL_RADIUS)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--channel', default='can2')
    ap.add_argument('--sec', type=float, default=20.0)
    ap.add_argument('--live', action='store_true', help='1초마다 현재값 출력')
    a = ap.parse_args()

    import can
    bus = can.interface.Bus(channel=a.channel, interface='socketcan')

    cmd = {0x141: 0.0, 0x142: 0.0}       # 최근 명령 (dps)
    act = {0x141: 0.0, 0x142: 0.0}       # 최근 실측 (dps)
    # 주행 중(명령 != 0)일 때만 추종성 통계를 낸다
    stat = defaultdict(lambda: {'n': 0, 'sum_cmd': 0.0, 'sum_act': 0.0,
                                'max_cmd': 0.0, 'max_act': 0.0,
                                'sum_ratio': 0.0, 'iq_max': 0.0})
    t0 = time.time()
    last_print = 0.0
    print('주행하세요 — %.0f초 관측 (명령 vs 실제)' % a.sec, flush=True)
    try:
        while time.time() - t0 < a.sec:
            m = bus.recv(timeout=0.2)
            if m is None:
                continue
            d = m.data
            if len(d) < 8 or d[0] != 0xA2:
                continue

            if m.arbitration_id in (0x141, 0x142):          # 송신 = 명령
                cmd[m.arbitration_id] = struct.unpack('<i', bytes(d[4:8]))[0] * 0.01

            elif m.arbitration_id in (0x241, 0x242):        # 응답 = 실측
                mid = m.arbitration_id - 0x100
                act[mid] = float(struct.unpack('<h', bytes(d[4:6]))[0])
                iq = abs(struct.unpack('<h', bytes(d[2:4]))[0] * 0.01)
                c, v = abs(cmd[mid]), abs(act[mid])
                if c > 5.0:                                  # 정지 구간 제외
                    s = stat[mid]
                    s['n'] += 1
                    s['sum_cmd'] += c
                    s['sum_act'] += v
                    s['sum_ratio'] += (v / c)
                    s['max_cmd'] = max(s['max_cmd'], c)
                    s['max_act'] = max(s['max_act'], v)
                    s['iq_max'] = max(s['iq_max'], iq)

            if a.live and time.time() - last_print > 1.0:
                last_print = time.time()
                print('  0x141 명령 %7.1f → 실제 %7.1f dps   |   '
                      '0x142 명령 %7.1f → 실제 %7.1f dps'
                      % (cmd[0x141], act[0x141], cmd[0x142], act[0x142]), flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        bus.shutdown()

    print('\n' + '=' * 66)
    if not stat:
        print('주행 구간이 없었다 (명령 5dps 이하만 관측됨).')
        return
    for mid in sorted(stat):
        s = stat[mid]
        n = s['n']
        print('0x%03X  샘플 %d' % (mid, n))
        print('  평균 명령 %7.1f dps (%.3f m/s)' % (s['sum_cmd'] / n,
                                                  dps_to_mps(s['sum_cmd'] / n)))
        print('  평균 실제 %7.1f dps (%.3f m/s)' % (s['sum_act'] / n,
                                                  dps_to_mps(s['sum_act'] / n)))
        print('  **추종률 평균 %.1f%%**' % (100.0 * s['sum_ratio'] / n))
        print('  최대 명령 %7.1f dps  /  최대 실제 %7.1f dps' % (s['max_cmd'], s['max_act']))
        print('  구간 최대 전류 %.1f A' % s['iq_max'])


if __name__ == '__main__':
    main()

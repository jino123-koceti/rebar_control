#!/usr/bin/env python3
"""모터 상태 프로브 (RMD 0x9A) — **버스 전압 + 에러 플래그**를 fsync로 남긴다.

## 왜 (2026-08-14) — 이게 지금 가장 중요한 계측이다
프리즈 순간 **모터 LED가 점멸(알람)** 하는 것이 육안으로 확인됐다. 그런데 지금까지
`/motor_feedback`의 `error_code`는 **can_parser가 전류·온도로 자체 판정한 값**이지
모터가 보고한 알람이 아니었다 → **모터가 실제로 무슨 알람을 띄웠는지 못 보고 있었다.**

그리고 결정적으로, `0x9A` 응답에는 **모터 버스 전압**이 들어 있다.
`volt_monitor`가 보는 INA3221은 **Jetson 모듈 입력단**뿐이라 상류(배터리) 새그를
못 봤는데(프리즈 직전까지 18.4V 평탄), 이 명령이 바로 그 상류를 보여준다.

판독:
  errorState에 **0x0004(저전압)** → 전원/배선 문제 확정. Jetson도 같은 이유로 즉사.
  0x0010(과전류)/0x0002(스톨)     → 기구 부하 문제
  전압 평탄 + 알람 없음            → 모터는 무죄. EMI/그라운드로 넘어감

## 프로토콜 (CAN BUS Motor Motion Protocol V4.3, 2.14)
  요청: [0x9A,0,0,0,0,0,0,0] → 응답 [0x9A, temp, MOStemp, brake, volt_lo, volt_hi,
                                     err_lo, err_hi]
  전압 0.1V/LSB, errorState는 비트 중첩(예: 0x0016 = 스톨+저전압+과전류)

## 기록 방식
`hz`(기본 10Hz)로 기록하되, 그 사이 폴링은 더 빠르게 돌려 **구간 최저전압**과
**OR 누적 에러비트**를 남긴다 — 짧은 새그/순간 알람을 놓치지 않기 위함.
**매 레코드 fsync** (프리즈는 하드 리셋이라 없으면 마지막 구간이 통째로 날아간다).

사용:
    python3 tools/diag/motor_status_probe.py
    python3 tools/diag/motor_status_probe.py --analyze /var/log/robot_control/motor_status.csv
"""
import argparse
import os
import struct
import time

OUT = '/var/log/robot_control/motor_status.csv'
COLS = ['time', 'up', 'motor', 'volt_v', 'volt_min_v', 'err', 'err_name',
        'temp', 'mos_temp']

ERR_BITS = [
    (0x0002, '스톨'), (0x0004, '저전압'), (0x0008, '과전압'),
    (0x0010, '과전류'), (0x0040, '파워초과'), (0x0080, '캘리브레이션쓰기오류'),
    (0x0100, '과속'), (0x0800, '부품과열'), (0x1000, '모터과열'),
    (0x2000, '엔코더캘리오류'), (0x4000, '엔코더데이터오류'),
]


def err_names(e):
    if not e:
        return ''
    n = [name for bit, name in ERR_BITS if e & bit]
    return '|'.join(n) if n else f'미상0x{e:04X}'


def analyze(path, tail=80):
    rows = [l.rstrip('\n').split(',') for l in open(path) if l.strip()]
    if len(rows) < 2:
        print('데이터 없음'); return
    hdr, data = rows[0], [r for r in rows[1:] if len(r) == len(rows[0])]
    cuts = [i for i in range(len(data)) if data[i][2] == 'BOOT']
    if not cuts:
        print('리셋 흔적 없음. 마지막 구간만 표시.'); cuts = [len(data)]
    iv, ie = hdr.index('volt_min_v'), hdr.index('err')
    for c in cuts[-2:]:
        seg = [r for r in data[max(0, c - tail):c] if r[2].startswith('0x')]
        if not seg:
            continue
        print(f"\n{'='*84}\n리셋 직전 (마지막 {seg[-1][0]})\n{'='*84}")
        print(' | '.join(f'{h:>10}' for h in hdr))
        for r in seg:
            print(' | '.join(f'{v:>10}' for v in r))
        vs = [float(r[iv]) for r in seg if r[iv]]
        es = [int(r[ie]) for r in seg if r[ie].isdigit()]
        if vs:
            print(f"\n  구간 **최저 전압 {min(vs):.1f} V**  (최고 {max(vs):.1f} V)")
        bad = [e for e in es if e]
        if bad:
            allbits = 0
            for e in bad:
                allbits |= e
            print(f"  ⚠ 에러 발생: {err_names(allbits)}  (0x{allbits:04X})")
        else:
            print("  에러 플래그 없음")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--channel', default='can2')
    ap.add_argument('--ids', nargs='+', default=['0x141', '0x142'])
    ap.add_argument('--hz', type=float, default=10.0, help='모터별 기록 주기')
    ap.add_argument('--analyze', metavar='CSV')
    ap.add_argument('--out', default=OUT)
    a = ap.parse_args()
    if a.analyze:
        analyze(a.analyze); return

    import can
    ids = [int(x, 0) for x in a.ids]
    os.makedirs(os.path.dirname(a.out) or '.', exist_ok=True)
    new = not os.path.exists(a.out) or os.path.getsize(a.out) == 0
    f = open(a.out, 'a')
    if new:
        f.write(','.join(COLS) + '\n')
    up0 = float(open('/proc/uptime').read().split()[0])
    f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')},{up0:.0f},"
            f"{'BOOT' if up0 < 60 else 'RESTART'},,,,,,\n")
    f.flush(); os.fsync(f.fileno())

    bus = can.interface.Bus(channel=a.channel, interface='socketcan')
    period = 1.0 / a.hz
    # 모터별 [최저전압, OR누적에러, 마지막응답, 마지막기록시각]
    acc = {i: [None, 0, None, 0.0] for i in ids}
    print(f'motor_status_probe → {a.out}  (0x9A, 모터별 {a.hz:.0f}Hz, 매 레코드 fsync)',
          flush=True)
    try:
        while True:
            for mid in ids:
                bus.send(can.Message(arbitration_id=mid,
                                     data=[0x9A, 0, 0, 0, 0, 0, 0, 0],
                                     is_extended_id=False))
            t0 = time.time()
            while time.time() - t0 < 0.02:      # 응답 수집
                m = bus.recv(timeout=0.02)
                if m is None:
                    break
                mid = m.arbitration_id - 0x100
                if mid not in acc or m.data[0] != 0x9A:
                    continue
                volt = struct.unpack('<H', bytes(m.data[4:6]))[0] / 10.0
                err = struct.unpack('<H', bytes(m.data[6:8]))[0]
                st = acc[mid]
                st[0] = volt if st[0] is None else min(st[0], volt)
                st[1] |= err
                st[2] = (volt, err, m.data[1], m.data[2])
            now = time.time()
            for mid, st in acc.items():
                if st[2] is None or now - st[3] < period:
                    continue
                volt, err, temp, mos = st[2]
                up = float(open('/proc/uptime').read().split()[0])
                f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')},{up:.0f},"
                        f"0x{mid:03X},{volt:.1f},{st[0]:.1f},{st[1]},"
                        f"{err_names(st[1])},{temp},{mos}\n")
                f.flush()
                os.fsync(f.fileno())     # ★ 하드 리셋 생존
                st[0] = None; st[1] = 0; st[3] = now
    except KeyboardInterrupt:
        pass
    finally:
        bus.shutdown()


if __name__ == '__main__':
    main()

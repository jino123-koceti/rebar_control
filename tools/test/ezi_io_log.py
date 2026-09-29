#!/usr/bin/env python3
"""EZIO 입력 변화를 시각과 함께 기록한다 — 배경 실행·매핑 작업용. 읽기 전용.

화면을 갱신하는 ezi_io_check.py 와 달리 **변화만 한 줄씩** 찍으므로
nohup/백그라운드로 돌려 로그를 남기기에 적합하다.

사용:
    python3 ezi_io_log.py --upper --sec 240 > upper.log
    python3 ezi_io_log.py --lower --sec 60
    python3 ezi_io_log.py --ip 192.168.0.6 --bid 0

2026-09-29 기준 보드: 하부 0.5 = Ethernet-IN16(입력전용), 상부 0.6 = I8O8.
연결은 Plus-E UDP(FAS_Connect) — Modbus 502 는 이 보드가 거부한다.
"""

import argparse
import os
import sys
import time
from datetime import datetime

sys.path.append(os.environ.get('FASTECH_LIBRARY_PATH',
                               os.path.expanduser('~/python/PE/Library')))
try:
    from FAS_EziMOTIONPlusE import (FAS_Connect, FAS_Close, FAS_GetInput,
                                    FAS_GetSlaveInfo)
    from ReturnCodes_Define import FMM_OK
except ImportError as e:
    print(f"FASTECH 라이브러리를 찾을 수 없습니다: {e}")
    sys.exit(1)

BOARD_TYPES = {150: ("Ethernet-IN16", 16), 151: ("Ethernet-IN32", 32),
               155: ("Ethernet-I8O8", 8), 156: ("Ethernet-I16O16", 16),
               160: ("Ethernet-OUT16", 0), 161: ("Ethernet-OUT32", 0)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ip', default=None)
    ap.add_argument('--bid', type=int, default=None)
    ap.add_argument('--upper', action='store_true')
    ap.add_argument('--lower', action='store_true')
    ap.add_argument('--sec', type=float, default=240.0, help='기록 시간(초)')
    ap.add_argument('--hz', type=float, default=50.0, help='폴링 주기')
    a = ap.parse_args()

    if a.ip:
        ip, bid = a.ip, (a.bid if a.bid is not None else (0 if a.ip.endswith('.6') else 1))
    elif a.upper:
        ip, bid = "192.168.0.6", 0
    else:
        ip, bid = "192.168.0.5", 1

    octets = [int(x) for x in ip.split('.')]
    if not FAS_Connect(*octets, bid):
        print(f"{ip} (board_id={bid}) 연결 실패")
        return 1
    r, dtype, desc = FAS_GetSlaveInfo(bid)
    name, n_in = BOARD_TYPES.get(dtype, (f"알수없음({dtype})", 16))
    print(f"# {ip} board_id={bid}  {name} ({desc})  입력 {n_in}점")
    print(f"# {a.sec:.0f}초간 변화만 기록합니다. 센서를 하나씩 건드리세요.")
    print(f"# {'시각':<12} {'입력(hex)':>10}  변화")
    sys.stdout.flush()

    prev = None
    t_end = time.time() + a.sec
    period = 1.0 / a.hz
    try:
        while time.time() < t_end:
            r, inp, latch = FAS_GetInput(bid)
            if r != FMM_OK:
                print(f"{datetime.now():%H:%M:%S.%f}"[:-3] + "  읽기 실패 "
                      f"(코드 {r})")
                sys.stdout.flush()
                time.sleep(0.5)
                continue
            inp &= (1 << n_in) - 1
            if prev is None:
                print(f"{datetime.now():%H:%M:%S.%f}"[:-3]
                      + f"  0x{inp:0{(n_in+3)//4}X}  시작 상태"
                      + f"  ON: {[i for i in range(n_in) if inp >> i & 1] or '없음'}")
                sys.stdout.flush()
            elif inp != prev:
                ch = [(i, 'ON ' if inp >> i & 1 else 'OFF')
                      for i in range(n_in) if (inp ^ prev) >> i & 1]
                desc_ch = ", ".join(f"IN{i:02d} {st}" for i, st in ch)
                print(f"{datetime.now():%H:%M:%S.%f}"[:-3]
                      + f"  0x{inp:0{(n_in+3)//4}X}  {desc_ch}")
                sys.stdout.flush()
            prev = inp
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        FAS_Close(bid)
        print(f"# 종료. 마지막 입력 0x{(prev or 0):0{(n_in+3)//4}X}")
    return 0


if __name__ == '__main__':
    sys.exit(main())

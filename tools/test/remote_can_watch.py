#!/usr/bin/env python3
"""
리모콘 수신기(can3, 250 kbps) 실시간 확인 — 매핑 작업용. 읽기 전용.

can3 프레임 (2026-09-29 실측):
  0x1E4  8바이트, 약 60ms  — 아날로그. DATA[0..3] 가 움직이고 DATA[4..7] 은 0x80 고정.
  0x2E4  8바이트, 약 60ms  — 스위치/상태. DATA[0]=0x62, DATA[3]=0x10, DATA[5]=0x05,
                             DATA[6]=0xF0, DATA[7]=0x99 는 고정. 변하는 건 DATA[1], [2], [4].
  0x764  1바이트 0x00, 약 300ms — 하트비트로 추정.

주의: rebar_base_control/can_parser.py 의 스위치 매핑(DATA[0] bit6/7 = 비상정지,
DATA[3] bit4/5 = S19/S20)은 2차년도 리모콘 기준이며 이 수신기에서는 맞지 않는다.
그 두 바이트는 고정값이다. 조이스틱 매핑(DATA[0..3])은 맞고, 중립값만 127 → 128 로 고쳐야 한다.

사용:
    python3 remote_can_watch.py             # can3
    python3 remote_can_watch.py --if can2
"""

import argparse
import collections
import select
import socket
import struct
import sys
import time
from datetime import datetime

NEUTRAL = 0x80
GREEN, YELLOW, RED, CYAN, DIM = '\033[92m', '\033[93m', '\033[91m', '\033[96m', '\033[2m'
BOLD, RESET = '\033[1m', '\033[0m'
CLEAR = '\033[H\033[2J'

# 0x2E4 에서 고정으로 관측된 바이트 — 변하면 눈에 띄게 표시한다
# (DATA[3] 은 2026-09-29 조작 중 0x10/0x02/0x00 으로 변하는 것이 확인돼 고정 목록에서 뺐다)
KNOWN_FIXED = {0: 0x62, 5: 0x05, 6: 0xF0, 7: 0x99}

# DATA[4] 는 하위 4비트 롤링 카운터(0x40~0x4F)다. 로그에 찍으면 다른 변화가 다 묻힌다.
COUNTER = (0x2E4, 4)
# 스위치가 들어 있는 바이트 — 이 조합의 변화를 한 줄로 묶어 기록한다
SWITCH_BYTES = (1, 2, 3)

AXIS_NAMES = ["AN1", "AN2", "AN3", "AN4"]   # 매핑 확인되면 이름을 바꿀 것


def bar(v, width=21):
    """중립 0x80 기준 좌우 막대."""
    mid = width // 2
    pos = int(round(v / 255 * (width - 1)))
    cells = []
    for i in range(width):
        if i == pos:
            cells.append(f"{YELLOW}{BOLD}█{RESET}")
        elif i == mid:
            cells.append(f"{DIM}|{RESET}")
        else:
            cells.append(f"{DIM}·{RESET}")
    return "".join(cells)


def bits(b):
    return " ".join(f"{CYAN}1{RESET}" if b >> (7 - i) & 1 else f"{DIM}0{RESET}" for i in range(8))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--if', dest='iface', default='can3')
    ap.add_argument('--rate', type=float, default=10.0, help='화면 갱신 Hz')
    a = ap.parse_args()

    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        s.bind((a.iface,))
    except OSError as e:
        print(f"{a.iface} 바인드 실패: {e}\n  ip link show {a.iface} 로 상태를 확인하세요")
        return 1
    s.settimeout(0.05)

    last = {}                      # can_id -> 최근 payload
    seen_vals = collections.defaultdict(set)   # (can_id, idx) -> 관측된 값
    counts = collections.Counter()
    last_seen = {}
    log = []
    states = {}                    # "DATA1 DATA2 DATA3" -> (첫 관측 시각, 순번)
    started = time.time()
    next_draw = 0.0

    try:
        while True:
            try:
                frame = s.recv(16)
                cid, dlc = struct.unpack("=IB3x", frame[:8])
                cid &= socket.CAN_EFF_MASK
                data = frame[8:8 + dlc]
                counts[cid] += 1
                last_seen[cid] = time.time()
                prev = last.get(cid)
                for i, b in enumerate(data):
                    seen_vals[(cid, i)].add(b)
                if prev is not None and prev != data:
                    ts = f"{datetime.now():%H:%M:%S.%f}"[:-3]
                    # 스위치 바이트가 하나라도 바뀌면 세 바이트 조합을 한 줄로
                    if cid == 0x2E4 and any(prev[i] != data[i] for i in SWITCH_BYTES
                                            if i < len(prev) and i < len(data)):
                        before = " ".join(f"{prev[i]:02X}" for i in SWITCH_BYTES)
                        after = " ".join(f"{data[i]:02X}" for i in SWITCH_BYTES)
                        which = ",".join(f"DATA[{i}]" for i in SWITCH_BYTES
                                         if prev[i] != data[i])
                        log.append(f"{ts}  {BOLD}스위치{RESET} [1][2][3] {before} → "
                                   f"{YELLOW}{after}{RESET}   ({which})")
                        states.setdefault(after, (ts, len(states)))
                    for i in range(min(len(prev), len(data))):
                        if prev[i] == data[i]:
                            continue
                        if (cid, i) == COUNTER or (cid == 0x2E4 and i in SWITCH_BYTES):
                            continue      # 카운터·스위치는 위에서 처리
                        note = ""
                        if cid == 0x2E4 and i in KNOWN_FIXED:
                            note = f"  {RED}← 고정으로 알려진 바이트가 변했다{RESET}"
                        diff = data[i] ^ prev[i]
                        bitlist = [7 - k for k in range(8) if diff >> (7 - k) & 1]
                        log.append(f"{ts}  0x{cid:03X} DATA[{i}] {prev[i]:#04x} → {data[i]:#04x}"
                                   + f"  (bit {','.join(map(str, bitlist))}){note}")
                last[cid] = data
            except socket.timeout:
                pass

            # 조작 중 이름을 입력하고 Enter → 로그에 표식을 남긴다
            if select.select([sys.stdin], [], [], 0)[0]:
                mark = sys.stdin.readline().strip()
                if mark:
                    log.append(f"{datetime.now():%H:%M:%S.%f}"[:-3]
                               + f"  {CYAN}{BOLD}━━ {mark} ━━{RESET}")

            if time.time() < next_draw:
                continue
            next_draw = time.time() + 1.0 / a.rate

            out = [CLEAR + f"{BOLD}리모콘 can3 확인{RESET}  {datetime.now():%H:%M:%S}"
                   f"  경과 {time.time()-started:.0f}s   Ctrl+C 종료", ""]

            ana = last.get(0x1E4)
            out.append(f"{CYAN}{BOLD}0x1E4{RESET} 아날로그   ({counts[0x1E4]}건)")
            if ana:
                for i in range(min(8, len(ana))):
                    v = ana[i]
                    off = v - NEUTRAL
                    name = AXIS_NAMES[i] if i < len(AXIS_NAMES) else f"ch{i}"
                    fixed = f"  {DIM}(중립 고정){RESET}" if len(seen_vals[(0x1E4, i)]) == 1 else ""
                    out.append(f"   DATA[{i}] {name:<4} {v:3d} {off:+4d}  {bar(v)}{fixed}")
            else:
                out.append(f"   {RED}프레임 없음{RESET}")

            sw = last.get(0x2E4)
            out.append("")
            out.append(f"{CYAN}{BOLD}0x2E4{RESET} 스위치/상태   ({counts[0x2E4]}건)")
            if sw:
                for i in range(min(8, len(sw))):
                    n = len(seen_vals[(0x2E4, i)])
                    tag = f"{DIM}고정{RESET}" if n == 1 else f"{YELLOW}변화 {n}종{RESET}"
                    exp = KNOWN_FIXED.get(i)
                    warn = f"  {RED}← 예상 {exp:#04x}{RESET}" if exp is not None and sw[i] != exp else ""
                    out.append(f"   DATA[{i}] {sw[i]:#04x} {bits(sw[i])}  {tag}{warn}")
            else:
                out.append(f"   {RED}프레임 없음{RESET}")

            hb = last_seen.get(0x764)
            age = f"{time.time()-hb:.1f}s 전" if hb else "없음"
            stale = hb is None or time.time() - hb > 1.0
            out.append("")
            out.append(f"{CYAN}{BOLD}0x764{RESET} 하트비트  {counts[0x764]}건, 마지막 {age}"
                       + (f"   {RED}{BOLD}← 끊김{RESET}" if stale else f"   {GREEN}정상{RESET}"))

            if states:
                out.append("")
                out.append(f"{BOLD}관측된 스위치 상태{RESET} [1][2][3] — 나타난 순서")
                for st, (ts, order) in sorted(states.items(), key=lambda kv: kv[1][1]):
                    out.append(f"   {order+1:2d}. {st}   {DIM}{ts}{RESET}")

            out.append("")
            out.append(f"{BOLD}변화 기록{RESET} (최근 14건)  {DIM}— 조작명 입력 후 Enter 로 표식{RESET}")
            out += log[-14:] or [f"{DIM}  아직 없음 — 스틱/스위치를 하나씩 움직여 보세요{RESET}"]
            sys.stdout.write("\n".join(out) + "\n")
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        s.close()
        print(f"\n프레임 합계: " + ", ".join(f"0x{c:03X}={n}" for c, n in sorted(counts.items())))
        print(f"변화 {len(log)}건")
        if states:
            print("\n관측된 스위치 상태 [DATA1 DATA2 DATA3] — 나타난 순서:")
            for st, (ts, order) in sorted(states.items(), key=lambda kv: kv[1][1]):
                print(f"  {order+1:2d}. {st}   ({ts})")
        print("\n전체 기록 (카운터 제외):")
        for line in log:
            print("  " + line)
    return 0


if __name__ == '__main__':
    sys.exit(main())

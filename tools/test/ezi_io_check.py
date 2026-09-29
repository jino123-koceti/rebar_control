#!/usr/bin/env python3
"""
EZI-IO 입출력 실시간 확인 (FASTECH Plus-E 라이브러리 사용)

보드 종류를 자동 판별해 입력(IN)은 항상, 출력(OUT)은 출력이 있는 보드에서만 보여준다.
기본은 읽기 전용이다. --write 를 줘야 출력 토글 키가 활성화된다.

사용법:
    python3 ezi_io_check.py                 # 하부 보드 (192.168.0.5, board_id=1)
    python3 ezi_io_check.py --upper         # 상부 보드 (192.168.0.6, board_id=0)
    python3 ezi_io_check.py --all           # 두 보드 동시
    python3 ezi_io_check.py --ip 192.168.0.6 --bid 0 --write   # 출력 토글 허용

키:
    Ctrl+C / q     종료
    (--write 일 때) 0-9 a-f   OUT00~OUT15 토글 (첫 번째 출력 보드 대상)
                    x         전체 출력 OFF
    종료 시 --write 로 바꾼 출력은 시작 시점 상태로 되돌린다.

2026-09-22 확인 사항:
  - 하부 보드(0.5)는 장치타입 150 = Ezi-IO Ethernet-IN16 (입력 전용). 출력 관련
    함수는 FMP_FRAMETYPEERROR(128)을 돌려준다 — 고장이 아니다.
  - ezi_io.yaml 의 port 502(Modbus)는 이 보드에서 거부된다. 라이브러리(FAS_Connect, UDP)로 붙는다.
  - 하부 범퍼는 실측 매핑: 전방 IN07(NO), 후방 IN05(NO), 좌측 IN13(NC), 우측 IN10(NC).
    극성이 섞여 있어 '범퍼:' 줄에서 눌림으로 환산해 보여준다.
  - IN00 = 후방 START, IN01 = STOP (평상시 OFF → NO 로 판단).
  - IN02 는 평상시 ON, 용도 미확인. 근접센서(IN03/IN04)와 상부 라벨은 미검증.
"""

import argparse
import os
import select
import sys
import termios
import time
import tty
from datetime import datetime

sys.path.append(os.environ.get('FASTECH_LIBRARY_PATH',
                               os.path.expanduser('~/python/PE/Library')))
try:
    from FAS_EziMOTIONPlusE import (FAS_Connect, FAS_Close, FAS_GetInput,
                                    FAS_GetOutput, FAS_SetOutput, FAS_GetSlaveInfo)
    from MOTION_DEFINE import *  # noqa: F401,F403  (DEVTYPE_*)
    from ReturnCodes_Define import FMM_OK
except ImportError as e:
    print(f"FASTECH 라이브러리를 찾을 수 없습니다: {e}")
    print("  FASTECH_LIBRARY_PATH 를 설정하거나 ~/python/PE/Library 에 설치하세요.")
    sys.exit(1)

# 장치타입 → (이름, 입력 수, 출력 수). MOTION_DEFINE 의 DEVTYPE_EZI_IO_* 값.
BOARD_TYPES = {
    150: ("Ethernet-IN16", 16, 0),
    151: ("Ethernet-IN32", 32, 0),
    155: ("Ethernet-I8O8", 8, 8),
    156: ("Ethernet-I16O16", 16, 16),
    160: ("Ethernet-OUT16", 0, 16),
    161: ("Ethernet-OUT32", 0, 32),
}

# 채널 이름 — 하부 범퍼는 2026-09-22 실측, 나머지는 ezi_io.yaml 기준(미검증, '?' 표시)
LABELS = {
    'lower': {0: "후방START", 1: "STOP", 3: "후방근접?", 4: "전방근접?", 5: "후방범퍼", 7: "전방범퍼",
              10: "우측범퍼", 13: "좌측범퍼"},
    'upper': {0: "Y최대", 1: "Y원점", 2: "X원점", 3: "X최대", 4: "Yaw원점",
              5: "Z원점", 6: "Z최대"},
}

# 범퍼: 이름 → (채널, NC 여부). 극성이 섞여 있어 원시 ON/OFF 로는 충돌 판정이 안 된다.
#   전방/후방 = NO (눌림 = ON), 좌측/우측 = NC (평상시 ON, 눌림 = OFF)
BUMPERS = {
    'lower': [("전방", 7, False), ("후방", 5, False), ("좌측", 13, True), ("우측", 10, True)],
    'upper': [],
}

# 조작 스위치: (이름, 채널, NC 여부). 평상시 OFF 였으므로 NO 로 둔다.
SWITCHES = {
    'lower': [("후방START", 0, False), ("STOP", 1, False)],
    'upper': [],
}

GREEN, YELLOW, RED, CYAN, DIM = '\033[92m', '\033[93m', '\033[91m', '\033[96m', '\033[2m'
BOLD, RESET = '\033[1m', '\033[0m'
CLEAR = '\033[H\033[2J'


class Board:
    def __init__(self, ip, bid, name, labels, bumpers=()):
        self.ip, self.bid, self.name, self.labels = ip, bid, name, labels
        self.bumpers = bumpers
        self.key = 'upper' if ip.endswith('.6') else 'lower'
        self.connected = False
        self.type_name, self.n_in, self.n_out = "?", 16, 0
        self.inp = self.latch = self.out = None
        self.out_initial = None
        self.flash = {}          # ('IN'|'OUT', ch) -> 남은 하이라이트 프레임
        self.err = ""
        self.last_try = 0.0

    def connect(self):
        self.last_try = time.time()
        ip = [int(x) for x in self.ip.split('.')]
        # UDP 로 붙는다. TCP(FAS_ConnectTCP)는 꺼진 보드에서 3초씩 막혀
        # --all 일 때 살아있는 보드 표시까지 같이 멈춘다. UDP 는 0.2초 만에 실패한다.
        if not FAS_Connect(*ip, self.bid):
            self.err = "연결 실패 (전원/케이블/IP 확인)"
            return False
        self.connected = True
        self.err = ""
        r, dtype, desc = FAS_GetSlaveInfo(self.bid)
        if r == FMM_OK:
            self.type_name, self.n_in, self.n_out = BOARD_TYPES.get(
                dtype, (f"알 수 없음({dtype}) {desc}", 16, 16))
        return True

    def close(self):
        if self.connected:
            FAS_Close(self.bid)
            self.connected = False

    def poll(self, log):
        if not self.connected:
            if time.time() - self.last_try > 3.0:
                self.connect()
            return
        if self.n_in:
            r, inp, latch = FAS_GetInput(self.bid)
            if r != FMM_OK:
                self.err = f"입력 읽기 실패 (코드 {r}) — 재연결 시도"
                self.close()
                return
            self._diff('IN', self.inp, inp, self.n_in, log)
            self.inp, self.latch = inp, latch
        if self.n_out:
            r, out, _ = FAS_GetOutput(self.bid)
            if r == FMM_OK:
                self._diff('OUT', self.out, out, self.n_out, log)
                self.out = out
                if self.out_initial is None:
                    self.out_initial = out
        self.err = ""

    def _diff(self, kind, prev, cur, n, log):
        if prev is None:
            return
        changed = (prev ^ cur) & ((1 << n) - 1)
        for ch in range(n):
            if changed >> ch & 1:
                self.flash[(kind, ch)] = 15
                on = "ON " if cur >> ch & 1 else "OFF"
                label = self.labels.get(ch, "") if kind == 'IN' else ""
                event = ""
                if kind == 'IN':
                    for bname, bch, nc in self.bumpers:
                        if bch == ch:
                            pressed = (not (cur >> ch & 1)) if nc else bool(cur >> ch & 1)
                            event = f"   ⇒ {bname}범퍼 {'눌림' if pressed else '해제'}"
                    for sname, sch, nc in SWITCHES.get(self.key, []):
                        if sch == ch:
                            pressed = (not (cur >> ch & 1)) if nc else bool(cur >> ch & 1)
                            event = f"   ⇒ {sname} {'눌림' if pressed else '해제'}"
                log.append(f"{datetime.now():%H:%M:%S.%f}"[:-3]
                           + f"  [{self.name}] {kind}{ch:02d}{'(' + label + ')' if label else ''} → {on}{event}")

    def cell(self, kind, ch, on):
        f = self.flash.get((kind, ch), 0)
        if f:
            self.flash[(kind, ch)] = f - 1
            color = YELLOW + BOLD
        else:
            color = (GREEN if kind == 'IN' else RED) + BOLD if on else DIM
        return f"{color}{kind}{ch:02d} {'●' if on else '·'}{RESET}"

    def render(self):
        lines = [f"{CYAN}{BOLD}[{self.name}]{RESET} {self.ip}  board_id={self.bid}  {self.type_name}"]
        if not self.connected or self.err:
            lines.append(f"  {RED}{self.err or '연결 안 됨'}{RESET}")
            return lines
        if self.n_in and self.inp is not None:
            lines.append(f"  입력 0x{self.inp & ((1 << self.n_in) - 1):0{(self.n_in + 3) // 4}X}"
                         f"   래치 0x{self.latch & ((1 << self.n_in) - 1):0{(self.n_in + 3) // 4}X}")
            for base in range(0, self.n_in, 8):
                lines.append("   " + "  ".join(self.cell('IN', c, self.inp >> c & 1)
                                                for c in range(base, min(base + 8, self.n_in))))
            active = [f"IN{c:02d}({self.labels[c]})" if c in self.labels else f"IN{c:02d}"
                      for c in range(self.n_in) if self.inp >> c & 1]
            lines.append(f"  ON: {', '.join(active) if active else '없음'}")
            if self.bumpers:
                parts = []
                for bname, ch, nc in self.bumpers:
                    raw = self.inp >> ch & 1
                    pressed = (not raw) if nc else bool(raw)
                    parts.append(f"{RED + BOLD}{bname} 눌림{RESET}" if pressed
                                 else f"{GREEN}{bname} ·{RESET}")
                lines.append("  범퍼: " + "   ".join(parts)
                             + f"   {DIM}(전후=NO, 좌우=NC){RESET}")
            sws = SWITCHES.get(self.key, [])
            if sws:
                parts = []
                for sname, ch, nc in sws:
                    raw = self.inp >> ch & 1
                    pressed = (not raw) if nc else bool(raw)
                    parts.append(f"{YELLOW + BOLD}{sname} 눌림{RESET}" if pressed
                                 else f"{GREEN}{sname} ·{RESET}")
                lines.append("  스위치: " + "   ".join(parts))
        if self.n_out:
            if self.out is None:
                lines.append(f"  {RED}출력 읽기 실패{RESET}")
            else:
                lines.append(f"  출력 0x{self.out:0{(self.n_out + 3) // 4}X}")
                for base in range(0, self.n_out, 8):
                    lines.append("   " + "  ".join(self.cell('OUT', c, self.out >> c & 1)
                                                    for c in range(base, min(base + 8, self.n_out))))
        else:
            lines.append(f"  {DIM}출력 없음 (입력 전용 보드){RESET}")
        return lines


def read_keys(fd):
    """os.read 로 커널 버퍼를 통째로 읽는다 (sys.stdin.read(1)+select 조합은 키를 잃는다)."""
    if select.select([fd], [], [], 0)[0]:
        return os.read(fd, 1024).decode('utf-8', 'ignore')
    return ''


def main():
    ap = argparse.ArgumentParser(description="EZI-IO 입출력 실시간 확인")
    ap.add_argument('--ip', default=None)
    ap.add_argument('--bid', type=int, default=None)
    ap.add_argument('--upper', action='store_true', help='상부 보드 (192.168.0.6, bid 0)')
    ap.add_argument('--all', action='store_true', help='하부+상부 동시')
    ap.add_argument('--rate', type=float, default=20.0, help='폴링 Hz (기본 20)')
    ap.add_argument('--write', action='store_true', help='출력 토글 키 활성화')
    a = ap.parse_args()

    if a.all:
        boards = [Board("192.168.0.5", 1, "하부", LABELS['lower'], BUMPERS['lower']),
                  Board("192.168.0.6", 0, "상부", LABELS['upper'])]
    elif a.ip:
        is_upper = a.ip.endswith('.6')
        boards = [Board(a.ip, a.bid if a.bid is not None else (0 if is_upper else 1),
                        "상부" if is_upper else "하부" if a.ip.endswith('.5') else a.ip,
                        LABELS['upper'] if is_upper else LABELS['lower'],
                        () if is_upper else BUMPERS['lower'])]
    elif a.upper:
        boards = [Board("192.168.0.6", 0, "상부", LABELS['upper'])]
    else:
        boards = [Board("192.168.0.5", 1, "하부", LABELS['lower'], BUMPERS['lower'])]

    for b in boards:
        b.connect()

    fd = sys.stdin.fileno()
    tty_ok = sys.stdin.isatty()
    old = termios.tcgetattr(fd) if tty_ok else None
    log = []
    period = 1.0 / a.rate
    try:
        if tty_ok:
            tty.setcbreak(fd)
        while True:
            t0 = time.time()
            for b in boards:
                b.poll(log)

            keys = read_keys(fd) if tty_ok else ''
            if 'q' in keys or '\x03' in keys:
                break
            target = next((b for b in boards if b.connected and b.n_out and b.out is not None), None)
            if a.write and target and keys:
                for k in keys:
                    if k == 'x':
                        FAS_SetOutput(target.bid, 0, (1 << target.n_out) - 1)
                    elif k in '0123456789abcdef':
                        ch = int(k, 16)
                        if ch < target.n_out:
                            m = 1 << ch
                            if target.out >> ch & 1:
                                FAS_SetOutput(target.bid, 0, m)
                            else:
                                FAS_SetOutput(target.bid, m, 0)

            out = [CLEAR + f"{BOLD}EZI-IO 입출력 확인{RESET}  {datetime.now():%H:%M:%S}"
                   f"   {a.rate:.0f}Hz   q/Ctrl+C 종료", ""]
            for b in boards:
                out += b.render() + [""]
            if a.write:
                out.append(f"{YELLOW}[쓰기 모드]{RESET} 0-9 a-f: OUT 토글   x: 전체 OFF"
                           + (f"   대상: {target.name}" if target else "   (출력 보드 없음)"))
            else:
                out.append(f"{DIM}읽기 전용 — 출력 토글은 --write{RESET}")
            out.append("")
            out.append(f"{BOLD}변화 기록{RESET} (최근 12건)")
            out += log[-12:] or [f"{DIM}  아직 없음 — 센서를 건드려 보세요{RESET}"]
            sys.stdout.write("\n".join(out) + "\n")
            sys.stdout.flush()

            time.sleep(max(0.0, period - (time.time() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        if old:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        for b in boards:
            # --write 로 바꾼 출력은 시작 시점 상태로 복구
            if a.write and b.connected and b.n_out and b.out_initial is not None \
                    and b.out != b.out_initial:
                mask = (1 << b.n_out) - 1
                FAS_SetOutput(b.bid, b.out_initial & mask, ~b.out_initial & mask)
                print(f"[{b.name}] 출력 복구 → 0x{b.out_initial:X}")
            b.close()
        print(f"\n종료. 변화 {len(log)}건")
        for line in log[-30:]:
            print("  " + line)


if __name__ == '__main__':
    main()

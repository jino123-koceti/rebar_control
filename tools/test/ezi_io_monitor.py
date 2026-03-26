#!/usr/bin/env python3
"""
EZI-IO DIO 보드 실시간 모니터링

터미널에서 전체 입력 포트(IN00~IN15) 상태를 실시간 갱신하고,
변화가 발생하면 하이라이트 + 로그를 남깁니다.

사용법:
    python3 ezi_io_monitor.py                          # 기본 (192.168.0.5, board_id=1)
    python3 ezi_io_monitor.py --ip 192.168.0.6 --bid 0 # 상부체 보드
    python3 ezi_io_monitor.py --rate 50                 # 50Hz 폴링
    python3 ezi_io_monitor.py --all                     # 두 보드 동시 모니터링

키 조작:
    Ctrl+C: 종료 (변화 이력 요약 출력)
"""

import sys
import os
import time
import argparse
from datetime import datetime

sys.path.append(os.environ.get('FASTECH_LIBRARY_PATH', '/home/koceti/python/PE/Library'))

try:
    from FAS_EziMOTIONPlusE import *
    from MOTION_DEFINE import *
    from ReturnCodes_Define import *
except ImportError as e:
    print("FASTECH library not found: {}".format(e))
    sys.exit(1)


# ANSI 색상
GREEN = '\033[92m'
RED = '\033[91m'
YELLOW = '\033[93m'
CYAN = '\033[96m'
BOLD = '\033[1m'
DIM = '\033[2m'
RESET = '\033[0m'
CLEAR_SCREEN = '\033[2J\033[H'


class EziIoMonitor:
    def __init__(self, ip_str, board_id, rate=20.0, label=""):
        self.ip = [int(x) for x in ip_str.split('.')]
        self.ip_str = ip_str
        self.board_id = board_id
        self.rate = rate
        self.label = label or ip_str
        self.connected = False

        self.prev_status = None
        self.change_log = []       # (timestamp, port, from, to)
        self.change_flash = {}     # port -> flash 남은 횟수 (변화 하이라이트)

    def connect(self):
        result = FAS_ConnectTCP(self.ip[0], self.ip[1], self.ip[2], self.ip[3], self.board_id)
        if result == 0:
            return False
        self.connected = True
        return True

    def close(self):
        if self.connected:
            FAS_Close(self.board_id)
            self.connected = False

    def read(self):
        """입력 상태 읽기. 실패시 None 반환"""
        if not self.connected:
            return None
        try:
            status_result, input_status, latch_status = FAS_GetInput(self.board_id)
            if status_result != FMM_OK:
                return None
            return input_status
        except Exception:
            self.connected = False
            return None

    def check_changes(self, current):
        """이전 상태와 비교, 변화 기록"""
        if self.prev_status is None:
            self.prev_status = current
            return

        changed = current ^ self.prev_status
        if changed:
            now = datetime.now().strftime('%H:%M:%S.%f')[:-3]
            for i in range(16):
                if changed & (1 << i):
                    was = "ON" if (self.prev_status & (1 << i)) else "OFF"
                    now_state = "ON" if (current & (1 << i)) else "OFF"
                    self.change_log.append((now, i, was, now_state))
                    self.change_flash[i] = 20  # 20 프레임 동안 하이라이트

        self.prev_status = current

    def format_port(self, port, state_on, width=10):
        """포트 하나의 표시 문자열 생성"""
        flash = self.change_flash.get(port, 0)
        if flash > 0:
            self.change_flash[port] = flash - 1
            color = YELLOW + BOLD
        elif state_on:
            color = GREEN
        else:
            color = DIM

        state_str = "ON" if state_on else "OFF"
        return "{}IN{:02d}:{:>3s}{}".format(color, port, state_str, RESET)

    def format_status(self, input_status):
        """전체 상태를 포맷팅"""
        lines = []
        lines.append("{}[{}]{} (board_id={}, 0x{:04X})".format(
            CYAN + BOLD, self.label, RESET, self.board_id, input_status))

        # 2행으로 8개씩 표시
        row1 = "  ".join([self.format_port(i, bool(input_status & (1 << i))) for i in range(8)])
        row2 = "  ".join([self.format_port(i, bool(input_status & (1 << i))) for i in range(8, 16)])
        lines.append("  " + row1)
        lines.append("  " + row2)

        return "\n".join(lines)


def run_single(ip, board_id, rate, label):
    """단일 보드 모니터링"""
    mon = EziIoMonitor(ip, board_id, rate, label)

    if not mon.connect():
        print("ERROR: Cannot connect to {} (board_id={})".format(ip, board_id))
        return

    print("Connected to {} (board_id={}). Polling at {}Hz. Ctrl+C to stop.\n".format(
        ip, board_id, rate))

    try:
        while True:
            status = mon.read()
            if status is None:
                print("\rConnection lost, reconnecting...", end='')
                mon.connect()
                time.sleep(1)
                continue

            mon.check_changes(status)

            # 화면 갱신
            output = CLEAR_SCREEN
            output += "{}EZI-IO Monitor{} | {}\n".format(
                BOLD, RESET, datetime.now().strftime('%H:%M:%S'))
            output += "=" * 60 + "\n\n"
            output += mon.format_status(status)
            output += "\n\n"

            # 최근 변화 이력 (최대 10개)
            if mon.change_log:
                output += "{}--- Recent Changes ---{}\n".format(YELLOW, RESET)
                for ts, port, was, now in mon.change_log[-10:]:
                    arrow_color = GREEN if now == "ON" else RED
                    output += "  {} IN{:02d}: {} {}->{}  {}{}\n".format(
                        ts, port, was, arrow_color, now, RESET, "")
            else:
                output += "{}No changes yet{}\n".format(DIM, RESET)

            print(output, end='', flush=True)
            time.sleep(1.0 / rate)

    except KeyboardInterrupt:
        pass
    finally:
        mon.close()
        print("\n\n{}=== Change Summary ==={}\n".format(BOLD, RESET))
        if mon.change_log:
            for ts, port, was, now in mon.change_log:
                print("  {} IN{:02d}: {} -> {}".format(ts, port, was, now))
            print("\nTotal changes: {}".format(len(mon.change_log)))
        else:
            print("  No changes detected.")


def run_dual(rate):
    """두 보드 동시 모니터링"""
    boards = [
        EziIoMonitor("192.168.0.6", 0, rate, "Upper (0.6)"),
        EziIoMonitor("192.168.0.5", 1, rate, "Lower (0.5)"),
    ]

    for mon in boards:
        if not mon.connect():
            print("WARNING: Cannot connect to {} (board_id={})".format(
                mon.ip_str, mon.board_id))

    connected = [m for m in boards if m.connected]
    if not connected:
        print("ERROR: No boards connected")
        return

    print("Connected {} board(s). Polling at {}Hz. Ctrl+C to stop.\n".format(
        len(connected), rate))

    try:
        while True:
            output = CLEAR_SCREEN
            output += "{}EZI-IO Monitor (Dual){} | {}\n".format(
                BOLD, RESET, datetime.now().strftime('%H:%M:%S'))
            output += "=" * 60 + "\n"

            for mon in boards:
                status = mon.read()
                if status is None:
                    output += "\n{}[{}]{} DISCONNECTED\n".format(
                        RED, mon.label, RESET)
                    mon.connect()
                    continue

                mon.check_changes(status)
                output += "\n" + mon.format_status(status) + "\n"

            # 모든 보드의 최근 변화
            all_changes = []
            for mon in boards:
                for ts, port, was, now in mon.change_log[-5:]:
                    all_changes.append((ts, mon.label, port, was, now))
            all_changes.sort(key=lambda x: x[0])

            output += "\n{}--- Recent Changes ---{}\n".format(YELLOW, RESET)
            if all_changes:
                for ts, label, port, was, now in all_changes[-10:]:
                    arrow_color = GREEN if now == "ON" else RED
                    output += "  {} [{}] IN{:02d}: {} {}->{}  {}\n".format(
                        ts, label, port, was, arrow_color, now, RESET)
            else:
                output += "  {}No changes yet{}\n".format(DIM, RESET)

            print(output, end='', flush=True)
            time.sleep(1.0 / rate)

    except KeyboardInterrupt:
        pass
    finally:
        for mon in boards:
            mon.close()
        print("\n\n{}=== Change Summary ==={}\n".format(BOLD, RESET))
        for mon in boards:
            if mon.change_log:
                print("[{}]".format(mon.label))
                for ts, port, was, now in mon.change_log:
                    print("  {} IN{:02d}: {} -> {}".format(ts, port, was, now))
                print()
        total = sum(len(m.change_log) for m in boards)
        print("Total changes: {}".format(total))


def main():
    parser = argparse.ArgumentParser(description='EZI-IO DIO 실시간 모니터링')
    parser.add_argument('--ip', type=str, default='192.168.0.5',
                        help='DIO 보드 IP (default: 192.168.0.5)')
    parser.add_argument('--bid', type=int, default=1,
                        help='Board ID (default: 1)')
    parser.add_argument('--rate', type=float, default=20.0,
                        help='폴링 주기 Hz (default: 20)')
    parser.add_argument('--label', type=str, default='',
                        help='보드 라벨명')
    parser.add_argument('--all', action='store_true',
                        help='두 보드 동시 모니터링 (0.5 + 0.6)')

    args = parser.parse_args()

    if args.all:
        run_dual(args.rate)
    else:
        run_single(args.ip, args.bid, args.rate, args.label)


if __name__ == '__main__':
    main()

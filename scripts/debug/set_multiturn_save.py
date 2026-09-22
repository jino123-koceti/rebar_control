#!/usr/bin/env python3
"""
RMD-X4 Function Control 0x20 idx=0x04
"多圈值掉电时保存使能" — Multi-turn value save on power off

매뉴얼 V4.3 §2.35.4:
  Index 0x04 — The multi-turn value is saved when the power is off
    "1" = enable: 전원 OFF 직전 멀티턴 값을 ROM에 저장 → 부팅 후 복원
    "0" = disable: single lap (0~360°) 모드로 폴백
    Takes effect after restart.

⚠️  매뉴얼상 0x20 명령에 read variant가 없음.
    현재 idx 0x04 상태는 CAN으로 직접 조회 불가 → 셋업 SW로만 확인 가능.
    이 스크립트는 set 명령 송신 + (선택) 시스템 리셋만 수행.
    적용 확인은 후속 검증 스크립트(test_rmd_multiturn.py 시나리오 2)로 함.

응답 동작:
  §2.35.3 — "the frame data is the same as the command sent by the host"
  즉 echo 응답만 옴 (성공/실패 구분 없음).

사용법:
  python3 set_multiturn_save.py --motor 0x147 --enable
  python3 set_multiturn_save.py --motor 0x147 --disable
  python3 set_multiturn_save.py --motor 0x147 --enable --reset
  python3 set_multiturn_save.py --all --enable --reset   # 0x141~0x147 일괄
"""
import argparse
import struct
import sys
import time

import can


CAN_IFACE = 'can2'
BITRATE = 1_000_000
ALL_MOTORS = [0x141, 0x142, 0x143, 0x144, 0x145, 0x146, 0x147]


def send_and_wait(bus, motor_id, payload, expected_cmd, timeout=1.0):
    msg = can.Message(
        arbitration_id=motor_id,
        data=payload + bytes(8 - len(payload)),
        is_extended_id=False,
    )
    bus.send(msg)
    reply_id = motor_id + 0x100  # 0x141 → 0x241
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None
        rx = bus.recv(timeout=remaining)
        if rx is None:
            continue
        if rx.arbitration_id == reply_id and rx.data[0] == expected_cmd:
            return bytes(rx.data)


def set_multiturn_save(bus, motor_id, value):
    """0x20 idx=0x04 value=value (1=enable, 0=disable)."""
    payload = bytes([0x20, 0x04, 0x00, 0x00, value & 0xFF, 0x00, 0x00, 0x00])
    reply = send_and_wait(bus, motor_id, payload, expected_cmd=0x20, timeout=0.5)
    label = 'enable' if value else 'disable'
    if reply is None:
        print(f"  [0x{motor_id:03X}] 0x20 idx=0x04 {label}: <no reply>")
        return False
    echo = ' '.join(f'{b:02X}' for b in reply)
    print(f"  [0x{motor_id:03X}] 0x20 idx=0x04 {label}: echo = {echo}")
    return True


def system_reset(bus, motor_id):
    """0x76 system reset. 응답 불확실 (리셋 중이므로) — 송신만 하고 짧게 대기."""
    payload = bytes([0x76, 0, 0, 0, 0, 0, 0, 0])
    msg = can.Message(
        arbitration_id=motor_id,
        data=payload,
        is_extended_id=False,
    )
    bus.send(msg)
    print(f"  [0x{motor_id:03X}] 0x76 system reset 송신 (응답 미확인, 부팅 대기)")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--iface', default=CAN_IFACE, help=f'CAN interface (default {CAN_IFACE})')

    group_target = parser.add_mutually_exclusive_group(required=True)
    group_target.add_argument('--motor', type=lambda x: int(x, 0),
                              help='Single motor CAN ID (e.g. 0x147)')
    group_target.add_argument('--all', action='store_true',
                              help='Apply to all motors 0x141~0x147')

    group_action = parser.add_mutually_exclusive_group(required=True)
    group_action.add_argument('--enable', action='store_true',
                              help='Set idx 0x04 = 1 (save on power off)')
    group_action.add_argument('--disable', action='store_true',
                              help='Set idx 0x04 = 0 (single lap mode)')

    parser.add_argument('--reset', action='store_true',
                        help='Also send 0x76 system reset after set (to apply immediately)')
    args = parser.parse_args()

    targets = ALL_MOTORS if args.all else [args.motor]
    value = 1 if args.enable else 0
    action_label = 'ENABLE (1)' if args.enable else 'DISABLE (0)'

    print("=" * 70)
    print("Function Control 0x20 idx=0x04 — multi-turn save on power off")
    print("=" * 70)
    print(f"CAN: {args.iface} @ {BITRATE} bps")
    print(f"Targets: {', '.join(f'0x{m:03X}' for m in targets)}")
    print(f"Action:  {action_label}")
    if args.reset:
        print("Then:    0x76 System Reset")
    print()
    print("⚠️  robot-control 서비스가 떠 있으면 충돌 가능: sudo systemctl stop robot-control")
    print()

    try:
        input("계속하려면 Enter, 취소하려면 Ctrl+C…")
    except (EOFError, KeyboardInterrupt):
        print()
        sys.exit(0)

    bus = can.interface.Bus(channel=args.iface, interface='socketcan', bitrate=BITRATE)

    try:
        print()
        print("--- 0x20 Function Control 송신 ---")
        for motor_id in targets:
            set_multiturn_save(bus, motor_id, value)
            time.sleep(0.05)

        if args.reset:
            print()
            print("--- 0x76 System Reset 송신 ---")
            for motor_id in targets:
                system_reset(bus, motor_id)
                time.sleep(0.05)
            print()
            print("⏳ 5초 대기 (모터 부팅)…")
            time.sleep(5.0)
            print("✅ 리셋 적용 완료 (추정)")
        else:
            print()
            print("⚠️  변경사항은 모터 재시작 후 적용됨.")
            print("    → 전원 사이클 또는 'python3 set_multiturn_save.py ... --reset' 실행 필요")

        print()
        print("다음 단계:")
        print("  python3 scripts/debug/test_rmd_multiturn.py --scenario 2")
        print("    → 시나리오 2(전원 OFF/그대로/ON)로 idx 0x04 효과 검증")

    finally:
        bus.shutdown()


if __name__ == '__main__':
    main()

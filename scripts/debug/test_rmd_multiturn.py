#!/usr/bin/env python3
"""
RMD-X4 Yaw (0x147) Multi-turn Absolute Encoder 검증 테스트

V4.3/V4.4 프로토콜 + Function Control 0x20 idx 0x04
(多圈值掉电时保存使能 = multi-turn value save on power off)

테스트 시나리오
  1) 전원 OFF + 손으로 회전 → 전원 ON → 변경된 위치 인식하는가?
  2) 전원 OFF (그대로) → 전원 ON → 같은 위치 유지하는가?

읽는 CAN 명령
  - 0x60 Read Multi-Turn Encoder Position       (int32, raw-offset, 인코더 카운트)
  - 0x61 Read Multi-Turn Encoder Original Pos   (int32, raw)
  - 0x62 Read Multi-Turn Encoder Zero Offset    (int32)
  - 0x92 Read Multi-Turn Angle                  (int32, 0.01°/LSB) ← 핵심
  - 0x94 Read Single-Turn Angle                 (uint16, 0.01°/LSB)
  - 0x20 idx 0x04: 멀티턴 저장 기능 enable 명령 (선택)

CAN bus: can2 @ 1Mbps
모터 ID: 0x147 (yaw), 응답 ID: 0x247

⚠️  실행 전 robot-control 서비스 중지:
    sudo systemctl stop robot-control
"""
import argparse
import struct
import sys
import time

import can


DEFAULT_MOTOR = 0x144  # X-axis stage (Yaw=0x147 기구한계로 X로 변경)
CAN_IFACE = 'can2'
BITRATE = 1_000_000


def send_cmd(bus, motor_id, payload):
    msg = can.Message(
        arbitration_id=motor_id,
        data=payload + bytes(8 - len(payload)),
        is_extended_id=False,
    )
    bus.send(msg)


def read_reply(bus, motor_id, expected_cmd, timeout=1.0):
    response_id = motor_id + 0x100
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None
        msg = bus.recv(timeout=remaining)
        if msg is None:
            continue
        if msg.arbitration_id == response_id and msg.data[0] == expected_cmd:
            return bytes(msg.data)


def parse_int32(data):
    return struct.unpack('<i', data[4:8])[0]


def parse_uint16_tail(data):
    return struct.unpack('<H', data[6:8])[0]


def read_one(bus, motor_id, cmd, label, parser, scale=1.0, unit=''):
    send_cmd(bus, motor_id, bytes([cmd]))
    time.sleep(0.02)
    reply = read_reply(bus, motor_id, cmd, timeout=1.0)
    if reply is None:
        print(f"  {label:40s}: <no reply>")
        return None
    raw = parser(reply)
    value = raw * scale
    if scale == 1.0:
        print(f"  {label:40s}: {raw}")
    else:
        print(f"  {label:40s}: {value:.3f}{unit}  (raw={raw})")
    return value


def read_0x90(bus, motor_id):
    """0x90 Single-Turn Encoder — can_parser와 동일 해석:
    data[2:4]=position(raw-offset), data[4:6]=raw, data[6:8]=offset (uint16).
    position이 homing의 yaw_encoder_90 (config yaw_*_encoder_90 비교 대상)."""
    send_cmd(bus, motor_id, bytes([0x90]))
    time.sleep(0.02)
    reply = read_reply(bus, motor_id, 0x90, timeout=1.0)
    label = '0x90 single-turn enc (pos/raw/off)'
    if reply is None:
        print(f"  {label:40s}: <no reply>")
        return None
    pos = struct.unpack('<H', reply[2:4])[0]
    raw = struct.unpack('<H', reply[4:6])[0]
    off = struct.unpack('<H', reply[6:8])[0]
    print(f"  {label:40s}: pos={pos} raw={raw} off={off}  (pos→{pos * 360.0 / 65536:.2f} deg)")
    return pos


def snapshot(bus, motor_id, header):
    print()
    print(f"=== {header} ===")
    results = {}
    results['0x60'] = read_one(bus, motor_id, 0x60, '0x60 multi-turn pos (raw-offset)', parse_int32)
    results['0x61'] = read_one(bus, motor_id, 0x61, '0x61 multi-turn raw position', parse_int32)
    results['0x62'] = read_one(bus, motor_id, 0x62, '0x62 multi-turn zero offset', parse_int32)
    results['0x90'] = read_0x90(bus, motor_id)
    results['0x92'] = read_one(bus, motor_id, 0x92, '0x92 multi-turn angle', parse_int32, 0.01, ' deg')
    results['0x94'] = read_one(bus, motor_id, 0x94, '0x94 single-turn angle', parse_uint16_tail, 0.01, ' deg')
    return results


def enable_multiturn_save(bus, motor_id):
    """0x20 Function Control idx=0x04, value=1 → 멀티턴 전원오프 저장 활성화."""
    print()
    print("→ 0x20 Function Control: idx=0x04 (multi-turn save on power off), value=1 송신")
    payload = bytes([0x20, 0x04, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00])
    send_cmd(bus, motor_id, payload)
    time.sleep(0.05)
    reply = read_reply(bus, motor_id, 0x20, timeout=1.0)
    if reply is None:
        print("  ⚠️  응답 없음 — 모터가 0x20 명령을 지원하지 않거나 묵음 응답일 수 있음")
    else:
        print(f"  응답: {' '.join(f'{b:02X}' for b in reply)}")
    print("  ⚠️  변경 사항은 모터 재시작(전원 사이클 또는 0x76) 후 적용됨")


def diff_report(label1, label2, snap1, snap2):
    print()
    print(f"--- DIFF: {label2} vs {label1} ---")
    for key in ('0x60', '0x61', '0x62', '0x90', '0x92', '0x94'):
        v1 = snap1.get(key)
        v2 = snap2.get(key)
        if v1 is None or v2 is None:
            print(f"  {key}: ?")
            continue
        delta = v2 - v1
        print(f"  {key}: {v1}  →  {v2}   (Δ = {delta:+.3f})")


def wait_user(prompt):
    print()
    print(prompt)
    try:
        input("   준비됐으면 Enter…")
    except (EOFError, KeyboardInterrupt):
        print()
        sys.exit(0)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--iface', default=CAN_IFACE, help=f'CAN interface (default {CAN_IFACE})')
    parser.add_argument('--motor', type=lambda x: int(x, 0), default=DEFAULT_MOTOR,
                        help=f'Motor CAN ID (default 0x{DEFAULT_MOTOR:03X}). '
                             'Stage X=0x144, Y=0x145, Z=0x146, Yaw=0x147')
    parser.add_argument('--enable-save', action='store_true',
                        help='Send 0x20 idx=0x04 value=1 (enable multi-turn save) at start')
    parser.add_argument('--scenario', choices=['1', '2', 'both'], default='both',
                        help='Which scenario to run')
    args = parser.parse_args()

    motor_id = args.motor
    motor_label = {0x141: 'Left wheel', 0x142: 'Right wheel', 0x143: 'Lateral',
                   0x144: 'X-axis stage', 0x145: 'Y-axis stage',
                   0x146: 'Z-axis stage', 0x147: 'Yaw'}.get(motor_id, '?')

    print("=" * 70)
    print("RMD-X4 Multi-turn Absolute Encoder 검증 테스트")
    print("=" * 70)
    print(f"CAN: {args.iface} @ {BITRATE} bps   Motor: 0x{motor_id:03X} ({motor_label})")
    print()
    print("⚠️  robot-control 서비스가 떠 있으면 CAN 충돌 가능.")
    print("    필요 시: sudo systemctl stop robot-control")
    wait_user("준비됐으면 진행합니다.")

    bus = can.interface.Bus(channel=args.iface, interface='socketcan', bitrate=BITRATE)

    try:
        if args.enable_save:
            enable_multiturn_save(bus, motor_id)
            print()
            print("⚠️  지금 모터 전원을 한 번 OFF→ON 해서 설정을 적용하세요.")
            wait_user("모터 재시작 끝났으면 진행합니다.")

        if args.scenario in ('1', 'both'):
            print()
            print("#" * 70)
            print("# 시나리오 1: 전원 OFF + 수동 회전 → 부팅 후 변경 위치 인식 검증")
            print("#" * 70)

            snap1a = snapshot(bus, motor_id, "Phase 1.A  초기 위치 (전원 ON 상태)")

            wait_user(
                f"👉 (1) 0x{motor_id:03X} 모터 전원 OFF\n"
                "👉 (2) 손으로 축을 임의 각도(가능하면 1바퀴 이상)만큼 회전\n"
                "👉 (3) 전원 ON, 부팅 안정화 대기"
            )

            time.sleep(1.5)  # 모터 부팅 안정화
            snap1b = snapshot(bus, motor_id, "Phase 1.B  재인가 후 위치")
            diff_report("1.A 초기", "1.B 재인가", snap1a, snap1b)

            print()
            print("✅ 시나리오 1 해석:")
            print("   - 0x92 Δ ≈ 회전 각도 → 멀티턴 absolute 정상 동작")
            print("   - 0x92 = 0 또는 1.A와 동일 → multi-turn lap count 손실")
            print("   - 0x94 만 변하고 0x92 안 변함 → single-turn 만 절대값")

        if args.scenario in ('2', 'both'):
            print()
            print("#" * 70)
            print("# 시나리오 2: 전원 OFF (회전 없음) → 부팅 후 위치 유지 검증")
            print("#" * 70)

            snap2a = snapshot(bus, motor_id, "Phase 2.A  현재 위치")

            wait_user(
                f"👉 (1) 0x{motor_id:03X} 모터 전원 OFF\n"
                "👉 (2) ⚠️  손대지 마세요. 모터 회전 금지.\n"
                "👉 (3) 약 3초 후 전원 ON, 부팅 안정화 대기"
            )

            time.sleep(1.5)
            snap2b = snapshot(bus, motor_id, "Phase 2.B  재인가 후 위치")
            diff_report("2.A 직전", "2.B 재인가", snap2a, snap2b)

            print()
            print("✅ 시나리오 2 해석:")
            print("   - 0x92 Δ ≈ 0 → power-cycle 안정성 OK (idx 0x04 활성화 또는 항상 절대)")
            print("   - 0x92 = 0 으로 리셋 → idx 0x04 비활성화 / 멀티턴 미보존")
            print("   - 0x94 동일 + 0x92 다름 → single-turn absolute 만 보장")

    finally:
        bus.shutdown()
        print()
        print("CAN bus 해제, 테스트 완료.")


if __name__ == '__main__':
    main()

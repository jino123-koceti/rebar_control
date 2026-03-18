#!/usr/bin/env python3
"""
0x20 명령으로 RMD 모터 ID 변경
Index 0x05: Set CANID
"""

import socket
import struct
import time

def change_motor_id(can_interface='can2', current_id=0x141, new_comm_id=2):
    """
    0x20 명령으로 모터 ID 변경

    Args:
        can_interface: CAN 인터페이스
        current_id: 현재 모터 CAN ID
        new_comm_id: 새로운 통신 ID (1-32)
    """

    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((can_interface,))
    sock.settimeout(1.0)

    print("=" * 70)
    print("RMD Motor ID Change (0x20 Command)")
    print("=" * 70)
    print(f"Current Motor ID: 0x{current_id:03X} (Comm ID: {current_id - 0x140})")
    print(f"New Comm ID: {new_comm_id}")
    print(f"New CAN ID: 0x{0x140 + new_comm_id:03X}")
    print("=" * 70)

    # 0x20 명령 구성
    # DATA[0] = 0x20 (Function Control Command)
    # DATA[1] = 0x05 (Set CANID index)
    # DATA[2-3] = 0x00
    # DATA[4-7] = Value (32-bit little-endian CANID)

    cmd_data = struct.pack('<BB2xI', 0x20, 0x05, new_comm_id)

    print(f"\nSending 0x20 command...")
    print(f"  Command: 0x20 (Function Control)")
    print(f"  Index: 0x05 (Set CANID)")
    print(f"  Value: {new_comm_id}")
    print(f"  Data: {cmd_data.hex().upper()}")

    # 명령 전송
    frame = struct.pack("IB3x8s", current_id, 8, cmd_data)
    sock.send(frame)

    time.sleep(0.2)

    # 응답 확인
    try:
        resp_frame = sock.recv(16)
        resp_id = struct.unpack("I", resp_frame[:4])[0] & 0x1FFFFFFF
        resp_data = resp_frame[8:16]

        print(f"\n✓ Response received!")
        print(f"  Response ID: 0x{resp_id:03X}")
        print(f"  Data: {resp_data.hex().upper()}")

        # 응답 확인
        if resp_data[0] == 0x20 and resp_data[1] == 0x05:
            print(f"\n✓✓ ID 변경 명령 성공!")
            print(f"\n⚠️  IMPORTANT: 모터 전원을 재부팅해야 적용됩니다!")
            print(f"  1. 모터 전원 OFF")
            print(f"  2. 3초 대기")
            print(f"  3. 모터 전원 ON")
            print(f"  4. 새 ID (0x{0x140 + new_comm_id:03X})로 통신 가능")
            return True
        else:
            print(f"\n⚠️  예상과 다른 응답")
            return False

    except socket.timeout:
        print(f"\n✗ No response")
        return False
    finally:
        sock.close()

def main():
    print("╔" + "═"*68 + "╗")
    print("║" + " "*15 + "RMD Motor ID Changer (0x20)" + " "*25 + "║")
    print("╚" + "═"*68 + "╝\n")

    # 현재 모터 ID 확인
    print("Step 1: 현재 모터 ID 확인...")

    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind(('can2',))
    sock.settimeout(0.5)

    current_id = None
    # ID 1~9까지 스캔
    for test_id in range(0x141, 0x14A):  # 0x141 ~ 0x149
        # 버퍼 비우기
        try:
            while sock.recv(16):
                pass
        except:
            pass

        # Read Status 명령
        cmd = bytes([0x9A, 0, 0, 0, 0, 0, 0, 0])
        frame = struct.pack("IB3x8s", test_id, 8, cmd)
        sock.send(frame)

        time.sleep(0.1)

        try:
            resp_frame = sock.recv(16)
            resp_id = struct.unpack("I", resp_frame[:4])[0] & 0x1FFFFFFF

            if resp_id:
                current_id = test_id
                print(f"✓ Motor found at 0x{test_id:03X} (Comm ID: {test_id - 0x140})\n")
                break
        except:
            pass

    sock.close()

    if not current_id:
        print("✗ No motor detected!")
        return 1

    # 새 ID 입력
    print("Step 2: 새 통신 ID 입력")
    print("  Available IDs: 1 ~ 9")
    print("  1 = 0x141")
    print("  2 = 0x142")
    print("  3 = 0x143")
    print("  ...")
    print("  9 = 0x149")

    try:
        new_id = int(input("\nNew Comm ID (1-9): "))
        if new_id < 1 or new_id > 9:
            print("Invalid ID! Please enter 1-9.")
            return 1
    except KeyboardInterrupt:
        print("\nCancelled by user.")
        return 1
    except:
        print("Invalid input!")
        return 1

    # 확인
    print(f"\n통신 ID를 {current_id - 0x140} → {new_id}로 변경하시겠습니까?")
    confirm = input("Continue? (yes/no): ")

    if confirm.lower() != 'yes':
        print("Cancelled.")
        return 0

    # ID 변경
    success = change_motor_id('can2', current_id, new_id)

    if success:
        print("\n" + "=" * 70)
        print("✓ 명령 전송 완료!")
        print("=" * 70)
        print("\n다음 단계:")
        print("  1. 모터 전원 OFF")
        print("  2. 모터 전원 ON")
        print("  3. python3 /home/test/scan_all_motor_ids.py 실행하여 확인")
        print("=" * 70)

    return 0 if success else 1

if __name__ == "__main__":
    import sys
    sys.exit(main())

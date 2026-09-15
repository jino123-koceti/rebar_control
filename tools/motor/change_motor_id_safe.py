#!/usr/bin/env python3
"""
RMD 모터 CAN ID 안전 변경 도구

기존 change_motor_id_0x20.py 와 달리, 대상 ID 에 응답하는 모터가
정확히 1대일 때만 쓰기를 수행한다.

버스에 같은 ID 의 모터가 여러 대 있으면 변경 명령을 모두가 수신해서
전부 같은 새 ID 로 바뀌어 버리므로, 반드시 1대씩 연결해서 작업해야 한다.

사용법:
  # 1) 탐색만 (아무것도 쓰지 않음)
  python3 change_motor_id_safe.py --probe

  # 2) 실제 변경: 0x141 에 있는 모터 1대를 통신 ID 2 (=0x142) 로
  python3 change_motor_id_safe.py --from 0x141 --to 2
"""

import argparse
import socket
import struct
import sys
import time

CAN_FRAME_FMT = "IB3x8s"
CMD_READ_STATUS = 0x9A
CMD_FUNCTION_CONTROL = 0x20
IDX_SET_CANID = 0x05


def open_socket(interface, timeout=0.4):
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((interface,))
    sock.settimeout(timeout)
    return sock


def drain(sock):
    """수신 버퍼 비우기"""
    sock.settimeout(0.05)
    try:
        while True:
            sock.recv(16)
    except (socket.timeout, BlockingIOError):
        pass


def send(sock, can_id, data8):
    sock.send(struct.pack(CAN_FRAME_FMT, can_id, 8, data8))


def collect(sock, window=0.5):
    """window 초 동안 수신되는 모든 프레임 수집"""
    frames = []
    deadline = time.time() + window
    while time.time() < deadline:
        sock.settimeout(max(0.01, deadline - time.time()))
        try:
            raw = sock.recv(16)
        except (socket.timeout, BlockingIOError):
            break
        cid = struct.unpack("I", raw[:4])[0] & 0x1FFFFFFF
        frames.append((cid, raw[8:16]))
    return frames


def decode_status(data):
    """0x9A 응답 해석: 온도, 전압, 에러"""
    temp = data[1]
    voltage = struct.unpack("<H", data[4:6])[0] / 10.0
    error = struct.unpack("<H", data[6:8])[0]
    return temp, voltage, error


def probe_one(sock, can_id, window=0.5):
    """해당 ID 에 상태조회를 보내고 (응답ID, 데이터) 목록을 돌려준다.

    ID 변경 후 전원 재투입 전에는 '명령은 옛 ID 로 받고 응답은 새 ID 로'
    보내는 과도 상태가 되므로, 응답 ID 를 질의ID+0x100 으로 한정하지 않고
    0x9A 응답인 모든 프레임을 받는다.
    """
    drain(sock)
    send(sock, can_id, bytes([CMD_READ_STATUS, 0, 0, 0, 0, 0, 0, 0]))
    return [(cid, d) for cid, d in collect(sock, window)
            if cid != can_id and d and d[0] == CMD_READ_STATUS]


def cmd_probe(args):
    sock = open_socket(args.interface)
    print("=" * 66)
    print(f"모터 탐색: {args.interface}  (0x141 ~ 0x160)")
    print("=" * 66)
    found = 0
    for comm_id in range(1, 33):
        can_id = 0x140 + comm_id
        resps = probe_one(sock, can_id, window=0.25)
        if not resps:
            continue
        found += 1
        flag = "  ⚠️  ID 중복!" if len(resps) > 1 else ""
        print(f"\n0x{can_id:03X} (통신 ID {comm_id}) — 응답 {len(resps)}개{flag}")
        for i, (cid, d) in enumerate(resps, 1):
            temp, volt, err = decode_status(d)
            expected = can_id + 0x100
            note = "" if cid == expected else f"  ← 응답ID 불일치 (전원 재투입 대기 중일 수 있음)"
            print(f"    [{i}] 응답 0x{cid:03X}  온도 {temp}°C  전압 {volt:.1f}V  "
                  f"에러 0x{err:04X}{'' if err == 0 else ' ← 에러'}{note}")
    sock.close()
    print()
    if found == 0:
        print("응답하는 모터가 없습니다. 배선/전원/종단저항을 확인하세요.")
        return 1
    print("=" * 66)
    return 0


def cmd_change(args):
    src = args.src
    new_comm_id = args.dst
    dst = 0x140 + new_comm_id

    if not 1 <= new_comm_id <= 32:
        print(f"새 통신 ID 는 1~32 여야 합니다 (입력: {new_comm_id})")
        return 1

    sock = open_socket(args.interface)

    print("=" * 66)
    print("RMD 모터 CAN ID 변경")
    print("=" * 66)
    print(f"인터페이스 : {args.interface}")
    print(f"현재 ID    : 0x{src:03X} (통신 ID {src - 0x140})")
    print(f"새 ID      : 0x{dst:03X} (통신 ID {new_comm_id})")
    print("=" * 66)

    # --- 1단계: 대상 ID 응답자 수 확인 -------------------------------------
    print(f"\n[1/3] 0x{src:03X} 응답자 확인 중...")
    resps = probe_one(sock, src, window=0.8)

    if len(resps) == 0:
        print(f"  ✗ 0x{src:03X} 에서 응답이 없습니다. 연결/전원을 확인하세요.")
        sock.close()
        return 1

    for i, (cid, d) in enumerate(resps, 1):
        temp, volt, err = decode_status(d)
        note = "" if cid == src + 0x100 else "  ← 응답ID 불일치"
        print(f"  [{i}] 응답 0x{cid:03X}  온도 {temp}°C  전압 {volt:.1f}V  "
              f"에러 0x{err:04X}{note}")

    if len(resps) > 1:
        print(f"\n  ✗ 중단: 0x{src:03X} 에 {len(resps)}대가 응답합니다.")
        print("    같은 ID 의 모터가 여러 대 붙어 있으면 변경 명령을 전부가 받아서")
        print("    모두 같은 새 ID 로 바뀝니다. 문제가 그대로 반복됩니다.")
        print("\n    → 모터를 1대만 남기고 나머지는 CAN 선을 분리한 뒤 다시 실행하세요.")
        sock.close()
        return 1

    print("  ✓ 응답자 1대 확인")

    # --- 2단계: 새 ID 가 이미 사용 중인지 확인 -----------------------------
    if dst != src:
        print(f"\n[2/3] 새 ID 0x{dst:03X} 중복 확인 중...")
        if probe_one(sock, dst, window=0.5):
            print(f"  ✗ 중단: 0x{dst:03X} 를 이미 다른 모터가 쓰고 있습니다.")
            sock.close()
            return 1
        print("  ✓ 사용 가능")
    else:
        print("\n[2/3] 현재 ID 와 동일 — 건너뜀")

    # --- 3단계: 변경 ------------------------------------------------------
    if not args.yes:
        print(f"\n0x{src:03X} → 0x{dst:03X} 로 변경합니다.")
        if input("계속할까요? (yes/no): ").strip().lower() != "yes":
            print("취소했습니다.")
            sock.close()
            return 0

    print(f"\n[3/3] 변경 명령 전송 (0x20 / index 0x{IDX_SET_CANID:02X})...")
    cmd = struct.pack("<BB2xI", CMD_FUNCTION_CONTROL, IDX_SET_CANID, new_comm_id)
    print(f"  데이터: {cmd.hex().upper()}")
    drain(sock)
    send(sock, src, cmd)

    acked = False
    for cid, d in collect(sock, window=0.6):
        print(f"  응답 0x{cid:03X}: {d.hex().upper()}")
        if d[0] == CMD_FUNCTION_CONTROL and d[1] == IDX_SET_CANID:
            acked = True
    sock.close()

    if not acked:
        print("\n  ⚠️  기대한 형태의 응답을 받지 못했습니다.")
        print("     전원 재투입 후 --probe 로 실제 반영 여부를 확인하세요.")
        return 1

    print("\n  ✓ 명령 수락됨")
    print("\n" + "=" * 66)
    print("⚠️  모터 전원을 껐다 켜야 적용됩니다.")
    print("    1. 모터 전원 OFF → 3초 대기 → ON")
    print(f"    2. python3 {sys.argv[0]} --probe 로 0x{dst:03X} 확인")
    print("=" * 66)
    return 0


def main():
    p = argparse.ArgumentParser(description="RMD 모터 CAN ID 안전 변경")
    p.add_argument("--interface", default="can2", help="CAN 인터페이스 (기본 can2)")
    p.add_argument("--probe", action="store_true", help="탐색만 수행 (쓰기 없음)")
    p.add_argument("--from", dest="src", type=lambda x: int(x, 0),
                   default=0x141, help="현재 CAN ID (기본 0x141)")
    p.add_argument("--to", dest="dst", type=int, help="새 통신 ID (1~32)")
    p.add_argument("--yes", action="store_true", help="확인 프롬프트 생략")
    args = p.parse_args()

    if args.probe or args.dst is None:
        return cmd_probe(args)
    return cmd_change(args)


if __name__ == "__main__":
    sys.exit(main())

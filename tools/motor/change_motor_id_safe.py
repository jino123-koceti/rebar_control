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
CMD_READ_STATUS = 0x9A      # Motor Status 1 (온도·전압·에러)
CMD_READ_STATUS2 = 0x9C     # Motor Status 2 (온도·전류·속도·엔코더)
CMD_FUNCTION_CONTROL = 0x20
IDX_SET_CANID = 0x05

# 탐색에 쓸 명령 후보. 둘 다 동작하므로 응답이 오는 첫 명령을 쓴다.
# (2026-10-03: 한동안 "X4-36 은 0x9A 를 지원하지 않는다" 고 오진했는데,
#  실제로는 warm() 이 설명하는 첫 프레임 유실이었다. 0x9A 는 정상 동작하고
#  전압까지 돌려준다 — 구동륜 38.1V. 명령 지원 여부로 의심하기 전에
#  프레임을 두 번 보내볼 것.)
PROBE_CMDS = (CMD_READ_STATUS, CMD_READ_STATUS2)

# 전용 CANID 명령. 0x20/index0x05 는 문서상 "saved to ROM" 이라는데
# **실제로는 전원 재투입에 날아갔다** (2026-10-03 주행축 실측: 응답 ID 가
# 0x242 로 바뀌어 적용된 것처럼 보였지만 재투입 후 0x141 로 복귀). 그래서
# 전용 명령 0x79 를 기본으로 쓴다. 이쪽은 **읽기가 있어서 전원을 내리지
# 않고도 반영 여부를 확인할 수 있다** — 이게 결정적이다.
#
# 자리배치: DATA[2] = 0 쓰기 / 1 읽기, DATA[7] = 통신ID(1~32).
# 문서 본문은 "Data[7]=1 이면 CANID 2" 라고 쓰여 있으나 **예제 프레임은
# DATA[7]=0x02**, 규격 줄도 (1~32) 라서 1-기반이 맞다 (본문이 OCR 오류).
# 거꾸로 읽으면 0x143 으로 가서 횡이동축과 충돌하므로 주의.
# 읽기 응답: DATA[6](하위)·DATA[7](상위) 리틀엔디안으로 **응답 ID**(0x240+n).
CMD_CANID = 0x79
CMD_SYSTEM_RESET = 0x76   # 설정을 적용하는 정규 재시작


def open_socket(interface, timeout=0.4):
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((interface,))
    sock.settimeout(timeout)
    return sock


def drain(sock, limit=0.15):
    """수신 버퍼 비우기.

    제어 노드가 돌고 있으면 프레임이 50ms 보다 촘촘히 들어와서 타임아웃이
    영원히 오지 않는다. 그래서 '조용해질 때까지' 가 아니라 '정해진 시간만'
    비운다. 벽시계로 끊지 않으면 여기서 멈춰 버린다.
    """
    deadline = time.time() + limit
    while True:
        left = deadline - time.time()
        if left <= 0:
            return
        sock.settimeout(min(0.02, left))
        try:
            sock.recv(16)
        except (socket.timeout, BlockingIOError):
            return


def send(sock, can_id, data8):
    sock.send(struct.pack(CAN_FRAME_FMT, can_id, 8, data8))


def warm(sock, can_id):
    """버스를 깨운다.

    **0.5초쯤 조용했다가 보내는 첫 프레임은 모터가 무시한다** (2026-10-03
    실측: 0x9C 를 15회 보내 14회 응답, 실패한 것은 첫 번째 하나뿐). 운영
    중에는 제어 노드가 35Hz 로 쉬지 않고 폴링해서 드러나지 않는 함정이다.

    읽기 명령을 버리는 셈으로 한 번 보내 두면, 뒤따르는 **쓰기 명령이
    조용히 유실되는 일**을 막을 수 있다. ID 변경이 안 먹었는데 먹은 줄
    알고 넘어가는 것이 제일 위험하다.
    """
    drain(sock)
    send(sock, can_id, bytes([CMD_READ_STATUS2, 0, 0, 0, 0, 0, 0, 0]))
    collect(sock, 0.15)


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


def describe(cmd, data):
    """응답을 사람이 읽을 문장으로. 명령마다 자리배치가 다르다."""
    temp = data[1]
    if cmd == CMD_READ_STATUS:          # 0x9A: 온도·전압·에러
        volt = struct.unpack("<H", data[4:6])[0] / 10.0
        err = struct.unpack("<H", data[6:8])[0]
        tail = f"전압 {volt:.1f}V  에러 0x{err:04X}" + ("" if err == 0 else " ← 에러")
    else:                               # 0x9C: 온도·전류·속도·엔코더
        cur = struct.unpack("<h", data[2:4])[0] * 0.01
        spd = struct.unpack("<h", data[4:6])[0]
        enc = struct.unpack("<H", data[6:8])[0]
        tail = f"전류 {cur:.2f}A  속도 {spd}dps  엔코더 {enc}"
    return f"온도 {temp}°C  {tail}"


def probe_one(sock, can_id, window=0.5):
    """해당 ID 에 상태조회를 보내고 (응답ID, 데이터) 목록을 돌려준다.

    ID 변경 후 전원 재투입 전에는 '명령은 옛 ID 로 받고 응답은 새 ID 로'
    보내는 과도 상태가 되므로, 응답 ID 를 질의ID+0x100 으로 한정하지 않고
    0x9A 응답인 모든 프레임을 받는다.
    """
    # 응답은 1.5ms 안에 온다. 창을 넉넉히 잡으면 **재시도 간격이 유실
    # 임계값(~0.5s)을 넘겨** 매번 다시 "조용한 뒤 첫 프레임" 이 되고,
    # 모터가 있는데도 없다고 판정한다 (2026-10-03 에 이걸로 두 번 막혔다).
    # 그러니 창은 짧게, 재시도는 촘촘히 한다.
    win = min(window, 0.2)
    for cmd in PROBE_CMDS:
        for _ in range(3):
            drain(sock, 0.05)
            send(sock, can_id, bytes([cmd, 0, 0, 0, 0, 0, 0, 0]))
            resps = [(cid, d) for cid, d in collect(sock, win)
                     if cid != can_id and d and d[0] == cmd]
            if resps:
                return [(cid, cmd, d) for cid, d in resps]
    return []


def canid_read(sock, can_id):
    """0x79 읽기. 성공하면 통신ID, 실패하면 None."""
    for _ in range(3):
        drain(sock, 0.05)
        send(sock, can_id, bytes([CMD_CANID, 0, 1, 0, 0, 0, 0, 0]))
        for cid, d in collect(sock, 0.2):
            if d and d[0] == CMD_CANID and d[2] == 1:
                reply_id = struct.unpack("<H", d[6:8])[0]
                return reply_id - 0x240
    return None


def canid_write(sock, can_id, comm_id):
    """0x79 쓰기. 모터는 보낸 명령을 그대로 되돌려준다."""
    warm(sock, can_id)
    drain(sock, 0.05)
    frame = bytes([CMD_CANID, 0, 0, 0, 0, 0, 0, comm_id])
    send(sock, can_id, frame)
    for cid, d in collect(sock, 0.3):
        if d and d[0] == CMD_CANID and d[2] == 0:
            return cid, d
    return None, None


def cmd_read(args):
    sock = open_socket(args.interface)
    print(f"0x{args.src:03X} 의 CANID 를 읽습니다 (0x79)...")
    comm = canid_read(sock, args.src)
    sock.close()
    if comm is None:
        print("  ✗ 응답 없음")
        return 1
    print(f"  통신 ID {comm}  →  명령 0x{0x140 + comm:03X} / 응답 0x{0x240 + comm:03X}")
    return 0


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
        for i, (cid, cmd, d) in enumerate(resps, 1):
            expected = can_id + 0x100
            note = "" if cid == expected else "  ← 응답ID 불일치 (전원 재투입 대기 중일 수 있음)"
            print(f"    [{i}] 응답 0x{cid:03X}  [0x{cmd:02X}]  {describe(cmd, d)}{note}")
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

    for i, (cid, cmd, d) in enumerate(resps, 1):
        note = "" if cid == src + 0x100 else "  ← 응답ID 불일치"
        print(f"  [{i}] 응답 0x{cid:03X}  [0x{cmd:02X}]  {describe(cmd, d)}{note}")

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

    # --- 3단계: 쓰기 전 현재값 읽기 --------------------------------------
    print("\n[3/5] 현재 CANID 읽기 (0x79)...")
    before = canid_read(sock, src)
    if before is None:
        print("  ⚠ 0x79 읽기에 응답하지 않습니다 — 이 펌웨어는 0x79 미지원일 수 있습니다.")
        if args.method == "0x79":
            print("    → --method 0x20 으로 다시 시도해 보세요.")
            sock.close()
            return 1
    else:
        print(f"  통신 ID {before} (명령 0x{0x140 + before:03X})")
        if before != src - 0x140:
            print(f"  ⚠ 질의한 ID(0x{src:03X})와 읽은 값이 다릅니다. 중단합니다.")
            sock.close()
            return 1

    # --- 4단계: 쓰기 ------------------------------------------------------
    if args.method == "0x79":
        print(f"\n[4/5] CANID 설정 (0x79, DATA[7]={new_comm_id})...")
        cid, d = canid_write(sock, src, new_comm_id)
        if d is None:
            print("  ✗ 응답 없음 — 쓰기가 전달되지 않았습니다.")
            sock.close()
            return 1
        print(f"  응답 0x{cid:03X}: {d.hex().upper()}")
        if d[7] != new_comm_id:
            print(f"  ✗ 되돌아온 값이 {d[7]} 로 요청({new_comm_id})과 다릅니다. 중단합니다.")
            sock.close()
            return 1
    else:
        print(f"\n[4/5] 변경 명령 전송 (0x20 / index 0x{IDX_SET_CANID:02X})...")
        cmd = struct.pack("<BB2xI", CMD_FUNCTION_CONTROL, IDX_SET_CANID, new_comm_id)
        print(f"  데이터: {cmd.hex().upper()}")
        warm(sock, src)         # 첫 프레임 유실로 쓰기가 사라지는 것을 막는다
        drain(sock, 0.05)
        send(sock, src, cmd)
        acked = False
        for cid, d in collect(sock, 0.3):
            print(f"  응답 0x{cid:03X}: {d.hex().upper()}")
            if d[0] == CMD_FUNCTION_CONTROL and d[1] == IDX_SET_CANID:
                acked = True
        if not acked:
            print("\n  ✗ 기대한 응답을 받지 못했습니다.")
            sock.close()
            return 1
    print("  ✓ 쓰기 수락됨")

    # --- 5단계: 0x76 으로 재시작시켜 적용 ---------------------------------
    # **쓰기만 하고 전원을 끊으면 날아간다** (2026-10-03 주행축 실측: ACK 가
    # 새 ID 로 와서 적용된 줄 알았는데 재투입 후 옛 ID 로 복귀). 0x76 을
    # 보내 재시작시킨 뒤에야 새 ID 로 붙었다. 옛 ID 와 새 ID 양쪽으로
    # 보내는데, 이 시점 모터는 명령을 옛 ID 로 받고 응답만 새 ID 로 한다.
    print("\n[5/5] 시스템 리셋(0x76) 후 확인...")
    for rid in (src, dst):
        send(sock, rid, bytes([CMD_SYSTEM_RESET, 0, 0, 0, 0, 0, 0, 0]))
        time.sleep(0.2)
    time.sleep(2.5)                      # 재부팅 대기

    new_resp = probe_one(sock, dst, window=0.2)
    old_resp = probe_one(sock, src, window=0.2)
    sock.close()

    print(f"  0x{dst:03X} 응답 {len(new_resp)}개 / 0x{src:03X} 응답 {len(old_resp)}개")
    if len(new_resp) != 1 or old_resp:
        print("\n  ✗ 기대와 다릅니다. 반영되지 않았을 수 있습니다.")
        print("     전원을 끊지 말고 --probe 로 현재 상태를 먼저 확인하세요.")
        return 1

    print(f"  ✓ 0x{dst:03X} 로 이동 확인 (옛 ID 응답 없음)")
    print("\n" + "=" * 66)
    print("⚠️  마지막으로 **모터 전원을 껐다 켜서 유지되는지** 확인하세요.")
    print("    0x76 재시작은 통과해도 실제 전원 차단에서 날아간 전례가 있습니다.")
    print(f"    python3 {sys.argv[0]} --probe")
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
    p.add_argument("--read", action="store_true", help="--from 의 CANID 만 읽는다")
    p.add_argument("--method", choices=("0x79", "0x20"), default="0x79",
                   help="쓰기 방식 (기본 0x79 전용 명령)")
    args = p.parse_args()

    if args.read:
        return cmd_read(args)
    if args.probe or args.dst is None:
        return cmd_probe(args)
    return cmd_change(args)


if __name__ == "__main__":
    sys.exit(main())

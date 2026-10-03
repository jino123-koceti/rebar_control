#!/usr/bin/env python3
"""RMD 모터들의 ROM 파라미터를 읽어 나란히 비교한다.

**모터를 교체하면 공장 기본값으로 온다.** 노드 설정은 그대로니 동작하는 것처럼
보이지만 가감속·PID 가 다른 채로 굴러간다. 교체 후에는 반드시 멀쩡한 쪽과
비교할 것. 특히 가감속은 ROM 값이고 **노드가 아니라 모터가 들고 있다**.

사용법:
  python3 compare_motor_params.py 0x141 0x142        # 두 대 비교
  python3 compare_motor_params.py 0x143 0x144
  python3 compare_motor_params.py --all              # 8축 전부 나열

주의: 제어 노드(rebar-teleop.service)를 내린 뒤 실행할 것. 돌고 있으면
트래픽에 섞여 응답을 놓친다.
"""

import argparse
import socket
import struct
import sys
import time

CAN_FMT = "IB3x8s"

# (명령, 인덱스, 이름, 해석) — 해석: 'i32' 는 int32, 'f32' 는 float
PARAMS = [
    (0x42, 0x00, "위치계획 가속",   "i32", "dps/s"),
    (0x42, 0x01, "위치계획 감속",   "i32", "dps/s"),
    (0x42, 0x02, "속도계획 가속",   "i32", "dps/s"),
    (0x42, 0x03, "속도계획 감속",   "i32", "dps/s"),
    (0x30, 0x01, "전류루프 KP",     "f32", ""),
    (0x30, 0x02, "전류루프 KI",     "f32", ""),
    (0x30, 0x04, "속도루프 KP",     "f32", ""),
    (0x30, 0x05, "속도루프 KI",     "f32", ""),
    (0x30, 0x07, "위치루프 KP",     "f32", ""),
    (0x30, 0x08, "위치루프 KI",     "f32", ""),
    (0x30, 0x09, "위치루프 KD",     "f32", ""),
]


def drain(sock, limit=0.05):
    end = time.time() + limit
    while time.time() < end:
        sock.settimeout(max(0.005, end - time.time()))
        try:
            sock.recv(16)
        except (socket.timeout, BlockingIOError):
            return


def ask(sock, can_id, cmd, index, tries=3):
    """cmd/index 를 묻고 DATA[4:8] 4바이트를 돌려준다. 실패하면 None.

    **조용한 버스의 첫 프레임은 유실된다**(0.5초쯤 유휴 후). 그래서 재시도는
    0.2초 간격으로 촘촘히 한다 — 넉넉히 기다리면 매번 다시 첫 프레임이 된다.
    """
    want = can_id + 0x100          # 0x141 로 보내면 0x241 로 온다
    for _ in range(tries):
        drain(sock)
        sock.send(struct.pack(CAN_FMT, can_id, 8,
                              bytes([cmd, index, 0, 0, 0, 0, 0, 0])))
        end = time.time() + 0.2
        while time.time() < end:
            sock.settimeout(max(0.005, end - time.time()))
            try:
                raw = sock.recv(16)
            except (socket.timeout, BlockingIOError):
                break
            cid = struct.unpack("I", raw[:4])[0] & 0x1FFFFFFF
            d = raw[8:16]
            if cid == want and d[0] == cmd and d[1] == index:
                return d[4:8]
    return None


def fmt(raw, kind, unit):
    if raw is None:
        return "응답없음"
    if kind == "i32":
        v = struct.unpack("<i", raw)[0]
        return f"{v}{' ' + unit if unit else ''}"
    v = struct.unpack("<f", raw)[0]
    return f"{v:g}"


def main():
    p = argparse.ArgumentParser(description="RMD 모터 ROM 파라미터 비교")
    p.add_argument("ids", nargs="*", help="비교할 CAN ID (예: 0x141 0x142)")
    p.add_argument("--all", action="store_true", help="0x141~0x148 전부")
    p.add_argument("--interface", default="can2")
    args = p.parse_args()

    ids = list(range(0x141, 0x149)) if args.all else [int(x, 0) for x in args.ids]
    if len(ids) < 1:
        p.error("비교할 ID 를 2개 이상 주거나 --all 을 쓰세요")

    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((args.interface,))

    for i in ids:                       # 버스 깨우기 (첫 프레임 유실 대비)
        ask(sock, i, 0x42, 0x00, tries=1)

    rows = []
    for cmd, idx, name, kind, unit in PARAMS:
        vals = [ask(sock, i, cmd, idx) for i in ids]
        rows.append((f"0x{cmd:02X}/{idx:02X}", name,
                     [fmt(v, kind, unit) for v in vals],
                     len({v for v in vals if v is not None}) > 1))
    sock.close()

    w = max(14, max(len(r[1]) for r in rows) + 1)
    head = f"{'명령':<9} {'파라미터':<{w}}" + "".join(f"{f'0x{i:03X}':>14}" for i in ids)
    print(head)
    print("-" * len(head))
    diffs = 0
    for code, name, vals, differs in rows:
        mark = "  ← 다름" if differs else ""
        if differs:
            diffs += 1
        print(f"{code:<9} {name:<{w}}" + "".join(f"{v:>14}" for v in vals) + mark)
    print("-" * len(head))
    print(f"다른 항목: {diffs} 개")
    return 1 if diffs else 0


if __name__ == "__main__":
    sys.exit(main())

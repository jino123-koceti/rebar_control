#!/usr/bin/env python3
"""호밍 전제 검사 — yaw 가 12시 ±3° 안에 있는가.

왜 필요한가: `axes.yaml` 의 yaw 호밍은 **"사용자가 호밍 전에 yaw 를 12시 ±3° 안에
놓아둔다"** 를 전제로 한다. 12시는 감지 구간보다 **위**라서 탐색 방향이 `counts
감소` 로 확정되는데, 전제가 깨지면(예: 4번 자세에서 시작) **방향이 뒤집혀 기계
끝으로 민다.** `homing_node` 도 거부하지만, 거부 사유만 보면 "얼마나 틀어졌는지"
를 모르므로 이 도구로 각도를 먼저 재서 사용자에게 알려준다.

기준값은 `noon_single`(단회전 절대). **전원과 무관하게 유효**한 유일한 값이다 —
멀티턴은 전원에 날아가고 `0x92`/`0x94` 는 전원 인가 시점 기준이다.

덤으로 **기구 무결성 검사**도 된다. 사용자가 기구적으로 12시에 맞춰 둔 상태에서
이 값이 크게 다르면 커플링이 미끄러졌거나 감지판이 움직인 것이다.

사용
  python3 yaw_noon_check.py
"""

import socket
import struct
import sys
import time

import yaml

FMT = "IB3x8s"
CMD_RAW_MULTI = 0x61        # 원시 멀티턴. 단회전 = 값 % CPR
CID = 0x148
AXES_YAML = ('/home/koceti/ros2_ws/src/rebar_control/'
             'src/rebar_base_control/config/axes.yaml')


def load_ref():
    """`axes.yaml` 에서 noon_single·cpr·gear 를 읽는다.

    손으로 적은 상수를 두면 기구를 만질 때마다 어긋나므로 설정에서 읽는다.
    """
    d = yaml.safe_load(open(AXES_YAML, encoding='utf-8')) or {}
    st = (d.get('stage') or {})
    enc = ((st.get('yaw') or {}).get('encoder') or {})
    noon = enc.get('noon_single')
    if noon is None:
        raise SystemExit("axes.yaml 에 stage.yaw.encoder.noon_single 이 없습니다.")
    return int(noon), int(enc.get('cpr', 262144)), float(st.get('gear', 12.5))


def read_raw(sock, tries=6):
    for _ in range(tries):
        sock.settimeout(0.02)
        while True:
            try:
                sock.recv(16)
            except OSError:
                break
        sock.send(struct.pack(FMT, CID, 8,
                              bytes([CMD_RAW_MULTI, 0, 0, 0, 0, 0, 0, 0])))
        end = time.time() + 0.15
        while time.time() < end:
            sock.settimeout(max(0.005, end - time.time()))
            try:
                raw = sock.recv(16)
            except OSError:
                break
            rid = struct.unpack("I", raw[:4])[0] & 0x1FFFFFFF
            d = raw[8:16]
            if rid == CID + 0x100 and d[0] == CMD_RAW_MULTI:
                return struct.unpack("<i", d[4:8])[0]
    return None


def main():
    noon, cpr, gear = load_ref()
    cpg = (cpr / 360.0) * gear               # counts / 건 1도
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind(("can2",))
    # ⚠ 여러 번 읽는다. 1Hz 로 묵은 값을 그대로 쓰면 오진이 영구화된다
    #   (2026-10-06 에 반복 확인으로 걸렀다).
    vals = []
    for _ in range(5):
        v = read_raw(sock)
        if v is not None:
            vals.append(v % cpr)
        time.sleep(0.4)
    sock.close()
    if not vals:
        print("✗ 0x148 이 0x61 에 응답하지 않습니다.")
        return 1
    if len(set(vals)) > 1:
        print(f"⚠ 값이 흔들립니다: {vals} — 축이 움직이고 있을 수 있습니다")
    cur = vals[-1]
    diff = (cur - noon + cpr // 2) % cpr - cpr // 2      # 최단 방향
    gun = diff / cpg
    print(f"yaw 단회전 {cur}  /  12시 기준 {noon}  →  "
          f"{diff:+d} counts = 건 {gun:+.2f}°")
    if abs(gun) <= 3.0:
        print("✓ 12시 ±3° 안 — 호밍 전제 통과")
        return 0
    print(f"✗ ±3° 벗어남 — 호밍이 거부됩니다. 건을 {-gun:+.2f}° 움직여야 합니다")
    return 2


if __name__ == '__main__':
    sys.exit(main())

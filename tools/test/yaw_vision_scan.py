#!/usr/bin/env python3
"""yaw 를 손으로 돌리는 동안 **각도–영상 쌍**을 모은다.

왜 (2026-10-07 사용자 제안): yaw 1번(건 −16.77°)·4번(+15.18°)은 **단회전만으로는
구분이 안 된다** (별칭 주기 건 28.8°). 전원 재투입 후 하필 그 자세에 있으면
시스템이 자세를 못 가려 호밍을 못 한다 — 사람이 12시로 옮기거나 선언해야 한다.
작업영역 카메라(Gemini 2L)는 **yaw 와 같이 돌지 않으므로**(캘리브레이션이 자세별인
이유가 이것이다 — 1번↔4번 `b` 가 Y 로 122mm 차이) 건 끝단이 자세마다 다른 자리에
보인다. 그걸로 후보 둘 중 하나를 고르자는 것이다.

⚠ 비전을 **단독 판정**으로 쓰지 않는 설계를 전제로 한다. 단회전이 후보를 **둘로**
좁혀 주므로 비전은 **32° 떨어진 둘 중 고르기만** 하면 된다. 거친 판정으로 충분하고,
틀려도 엔코더 후보 밖으로는 못 간다.

**각도 라벨을 어떻게 믿나:** 단회전은 별칭이 있지만 **멀티턴은 전원 세션 안에서
연속**이다. 그래서 시작 자세를 하나 알면(`--anchor-gun`) 이후 모든 프레임의 각도를
멀티턴 차이로 정확히 계산할 수 있다. 전원을 끊으면 이 기준은 무효다.

사용 (리모콘으로 천천히 전 구간을 돌리면 된다. Ctrl-C 로 종료)
  python3 yaw_vision_scan.py --out-dir /tmp/yawscan --anchor-gun -16.77
"""

import argparse
import csv
import os
import socket
import struct
import sys
import time

FMT = "IB3x8s"
CMD_RAW_MULTI = 0x61
CID = 0x148
CPR = 262144
GEAR = 12.5
CPG = (CPR / 360.0) * GEAR          # counts / 건 1도


def read_raw(sock):
    for _ in range(3):
        sock.settimeout(0.01)
        while True:
            try:
                sock.recv(16)
            except OSError:
                break
        sock.send(struct.pack(FMT, CID, 8,
                              bytes([CMD_RAW_MULTI, 0, 0, 0, 0, 0, 0, 0])))
        end = time.time() + 0.05
        while time.time() < end:
            sock.settimeout(max(0.003, end - time.time()))
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
    p = argparse.ArgumentParser()
    p.add_argument('--out-dir', required=True)
    p.add_argument('--anchor-gun', type=float, required=True,
                   help='시작 시점의 건 각도(도). 지금 자세를 알아야 라벨이 선다')
    p.add_argument('--step-gun', type=float, default=0.5,
                   help='이 각도만큼 움직일 때마다 한 장 저장')
    p.add_argument('--topic', default='/camera/color/image_raw')
    a = p.parse_args()

    import cv2
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    os.makedirs(a.out_dir, exist_ok=True)
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind(("can2",))
    raw0 = read_raw(sock)
    if raw0 is None:
        print("✗ yaw(0x148) 가 0x61 에 응답하지 않습니다.")
        return 1

    rclpy.init()
    node = Node('yaw_vision_scan')
    bridge = CvBridge()
    last = {'img': None}
    node.create_subscription(
        Image, a.topic,
        lambda m: last.__setitem__('img', bridge.imgmsg_to_cv2(m, 'bgr8')), 1)

    idx_path = os.path.join(a.out_dir, 'index.csv')
    new = not os.path.exists(idx_path)
    f = open(idx_path, 'a', newline='')
    w = csv.writer(f)
    if new:
        w.writerow(['file', 'gun_deg', 'raw_multi', 'single_turn',
                    'alias_gun_a', 'alias_gun_b'])
        f.flush()

    print(f"기준점: 건 {a.anchor_gun:+.2f}° (멀티턴 {raw0})")
    print(f"저장 간격 건 {a.step_gun}°  →  {a.out_dir}")
    print("리모콘으로 yaw 를 천천히 전 구간 돌려 주세요. Ctrl-C 로 종료합니다.\n")

    saved, last_gun = 0, None
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.02)
            raw = read_raw(sock)
            if raw is None or last['img'] is None:
                continue
            gun = a.anchor_gun + (raw - raw0) / CPG
            if last_gun is not None and abs(gun - last_gun) < a.step_gun:
                continue
            last_gun = gun
            s1 = raw % CPR
            # 단회전만 보면 어떤 후보 둘이 나오는지도 같이 적어 둔다
            base = (s1 / CPG) % 28.8
            cand = sorted({round(base - 28.8, 2), round(base, 2)})
            name = f"gun{gun:+07.2f}.png".replace('+', 'p').replace('-', 'm')
            cv2.imwrite(os.path.join(a.out_dir, name), last['img'])
            w.writerow([name, f"{gun:.2f}", raw, s1, cand[0], cand[1]])
            f.flush()
            saved += 1
            print(f"  건 {gun:+7.2f}°  단회전 {s1:>6d}  → {name}", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        f.close()
        sock.close()
        rclpy.shutdown()
    print(f"\n{saved}장 저장 → {a.out_dir}")
    return 0


if __name__ == '__main__':
    sys.exit(main())

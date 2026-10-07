#!/usr/bin/env python3
"""작업영역 카메라에서 한 장 받아 파일로 저장한다.

용도 (2026-10-07): yaw 자세 **별칭 문제**를 비전으로 가리는 방안을 검토한다.
1번(건 −16.77°)과 4번(+15.18°)은 단회전만으로는 구분이 안 되는데(별칭 주기
건 28.8°), 카메라가 yaw 와 **같이 돌지 않으므로**(캘리브레이션이 자세별인 이유가
이것이다 — 1번↔4번 오프셋이 Y 로 122mm 차이) 건 끝단이 영상에서 다르게 보인다.

**먼저 확인할 것은 "건 끝단이 화면에 보이는가" 다.** 안 보이면 방안이 성립하지
않는다. 그래서 자세별로 한 장씩 받아 눈으로 비교한다.

사용
  python3 grab_frame.py --out /tmp/pose1.png
"""

import argparse
import sys
import time


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--out', required=True)
    p.add_argument('--topic', default='/camera/color/image_raw')
    p.add_argument('--timeout', type=float, default=15.0)
    a = p.parse_args()

    import cv2
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    rclpy.init()
    node = Node('grab_frame')
    bridge = CvBridge()
    got = {'img': None}

    def on_img(msg):
        if got['img'] is None:
            got['img'] = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    # ⚠ 첫 프레임은 버리지 않는다 — 10fps 라 기다리면 느리고, 정지 상태에서
    #   재므로 한 장으로 충분하다.
    node.create_subscription(Image, a.topic, on_img, 1)
    end = time.time() + a.timeout
    while got['img'] is None and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    rclpy.shutdown()

    if got['img'] is None:
        print(f"✗ {a.topic} 에서 프레임을 못 받았습니다 (카메라 확인)")
        return 1
    h, w = got['img'].shape[:2]
    cv2.imwrite(a.out, got['img'])
    print(f"저장 {a.out}  ({w}x{h})")
    return 0


if __name__ == '__main__':
    sys.exit(main())

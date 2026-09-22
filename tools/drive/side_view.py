#!/usr/bin/env python3
"""주행 전·후방 카메라(zedxmini1/2) 실시간 뷰어.
데크끝/바닥영역 인식 데이터 확인·수집용. 전·후방 RGB를 나란히 표시.

[2026-08-06] zedxmini1/2를 좌·우 측면 → 전방/후방으로 물리 이동. 측면 이동은
  아직 미구현이라 좌우 뷰어에서 전후 뷰어로 전환했다.

사전: zedxmini2(전방 SN54946194)/zedxmini1(후방 SN56755054) 노드 실행 중이어야 함.
  (full_system.launch.py에 포함되거나, 수동:
   ros2 launch zed_wrapper zed_camera.launch.py camera_name:=zedxmini2 \\
       camera_model:=zedxm serial_number:=54946194 grab_resolution:=HD720)

  python3 tools/drive/side_view.py

  s  전후 스냅샷 저장(/tmp/drive_front.png, /tmp/drive_back.png)   q/ESC 종료
"""
import argparse
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

TOPIC_L = '/zedxmini2/zed_node/rgb/color/rect/image'   # 전방
TOPIC_R = '/zedxmini1/zed_node/rgb/color/rect/image'   # 후방


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--height', type=int, default=540, help='표시 높이(px)')
    ap.add_argument('--left', default=TOPIC_L)
    ap.add_argument('--right', default=TOPIC_R)
    args = ap.parse_args()

    rclpy.init()
    node = Node('side_view')
    br = CvBridge()
    last = {'L': None, 'R': None}
    cnt = {'L': 0, 'R': 0}
    t0 = {'L': time.time(), 'R': time.time()}
    fps = {'L': 0.0, 'R': 0.0}

    def make_cb(key):
        def cb(msg):
            last[key] = br.imgmsg_to_cv2(msg, 'bgr8')
            cnt[key] += 1
            now = time.time()
            if now - t0[key] >= 1.0:
                fps[key] = cnt[key] / (now - t0[key])
                cnt[key] = 0
                t0[key] = now
        return cb

    node.create_subscription(Image, args.left, make_cb('L'), qos_profile_sensor_data)
    node.create_subscription(Image, args.right, make_cb('R'), qos_profile_sensor_data)

    win = 'drive view  L=zedxmini2(front)  R=zedxmini1(back)  (s=snap q=quit)'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 1600, 540)

    def panel(img, label, fps_v, h):
        """이미지를 높이 h로 맞추고 라벨/FPS 오버레이. 없으면 검은 패널."""
        if img is None:
            p = np.zeros((h, int(h * 16 / 9), 3), np.uint8)
            cv2.putText(p, f'{label}: NO SIGNAL', (20, h // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            return p
        s = h / img.shape[0]
        p = cv2.resize(img, (int(img.shape[1] * s), h))
        cv2.putText(p, f'{label}  {fps_v:.1f}fps', (14, 34),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        return p

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)
            h = args.height
            l = panel(last['L'], 'FRONT (zedxmini2)', fps['L'], h)
            r = panel(last['R'], 'BACK (zedxmini1)', fps['R'], h)
            sep = np.full((h, 6, 3), (0, 255, 0), np.uint8)
            cv2.imshow(win, np.hstack([l, sep, r]))
            k = cv2.waitKey(1) & 0xFF
            if k in (ord('q'), 27):
                break
            if k == ord('s'):
                if last['L'] is not None:
                    cv2.imwrite('/tmp/drive_front.png', last['L'])
                if last['R'] is not None:
                    cv2.imwrite('/tmp/drive_back.png', last['R'])
                print('  저장: /tmp/drive_front.png /tmp/drive_back.png')
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

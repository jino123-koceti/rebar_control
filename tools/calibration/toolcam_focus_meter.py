#!/usr/bin/env python3
"""zedxone 초점 맞추기 도우미 — 실시간 선명도(Laplacian 분산) 출력.
렌즈를 돌리며(또는 카메라 거리 조정하며) 숫자가 커지게 → 선명.
목표: var > 100 (선명). 8~50 = 흐림.

사용: python3 toolcam_focus_meter.py
"""
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

rclpy.init()
n = Node('focus')
br = CvBridge()
buf = []
n.create_subscription(Image, '/zedxone/zed_node/rgb/rect/image',
    lambda m: buf.append(br.imgmsg_to_cv2(m, 'bgr8')), qos_profile_sensor_data)
print('zedxone 초점 미터 — 렌즈 돌리며 var 최대화 (목표 >100). Ctrl-C 종료')
best = 0
try:
    while True:
        buf.clear()
        t = time.time()
        while not buf and time.time() - t < 2:
            rclpy.spin_once(n, timeout_sec=0.05)
        if not buf:
            continue
        g = cv2.cvtColor(buf[-1], cv2.COLOR_BGR2GRAY)
        v = cv2.Laplacian(g, cv2.CV_64F).var()
        best = max(best, v)
        bar = '#' * min(int(v / 5), 60)
        tag = ' ← 선명!' if v > 100 else (' (흐림)' if v < 50 else '')
        print(f'\r선명도 {v:6.0f}  최고 {best:6.0f}  {bar}{tag}          ', end='', flush=True)
        time.sleep(0.15)
except KeyboardInterrupt:
    print(f'\n최고 선명도: {best:.0f}')
finally:
    n.destroy_node(); rclpy.shutdown()

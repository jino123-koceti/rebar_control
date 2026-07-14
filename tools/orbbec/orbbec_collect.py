#!/usr/bin/env python3
"""Orbbec 교차점 YOLO 학습 데이터 수집기 (ROS 토픽).
카메라 실행 후 로봇/작업영역 옮겨가며 프레임 저장 → 교차점 검출 파인튜닝용.

  ros2 launch orbbec_camera gemini2L.launch.py    # 먼저 카메라 실행
  python3 orbbec_collect.py

  SPACE/s  현재 프레임 저장 (모션블러 체크)
  a        자동수집 토글 (1.5초 간격, 변화 프레임만)
  f        강제 저장
  q/ESC    종료

저장: data/orbbec_crossing_dataset/images/ocross_NNNN.png
"""
import os, time, argparse
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

OUT_DIR = '/home/koceti/ros2_ws/data/orbbec_crossing_dataset/images'


class Collector(Node):
    def __init__(self, topic):
        super().__init__('orbbec_collect')
        self.br = CvBridge()
        self.latest = None
        self.last_saved = None
        self.create_subscription(Image, topic, self._cb, qos_profile_sensor_data)
        os.makedirs(OUT_DIR, exist_ok=True)
        existing = [f for f in os.listdir(OUT_DIR) if f.startswith('ocross_')]
        self.n = max([int(f[7:11]) for f in existing], default=0)
        self.get_logger().info(f'topic={topic}  기존 {len(existing)}장, 다음번호 {self.n+1}')

    def _cb(self, m):
        self.latest = self.br.imgmsg_to_cv2(m, 'bgr8')

    def sharpness(self, img):
        return cv2.Laplacian(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var()

    def changed(self, img, thresh=10.0):
        if self.last_saved is None:
            return True
        a = cv2.resize(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (160, 100)).astype(int)
        b = cv2.resize(cv2.cvtColor(self.last_saved, cv2.COLOR_BGR2GRAY), (160, 100)).astype(int)
        return np.mean(np.abs(a - b)) > thresh

    def save(self, force=False):
        if self.latest is None:
            print('  영상 없음'); return False
        img = self.latest
        sh = self.sharpness(img)
        if not force and sh < 30:     # 컬러 데크/철근이면 정상 수십~수백
            print(f'  ⚠️ 흐림 의심(sharp {sh:.0f}) — 정지 후 (강제: f)'); return False
        self.n += 1
        p = os.path.join(OUT_DIR, f'ocross_{self.n:04d}.png')
        cv2.imwrite(p, img)
        self.last_saved = img.copy()
        print(f'  ✅ ocross_{self.n:04d}.png (sharp {sh:.0f})')
        return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/camera/color/image_raw')
    args = ap.parse_args()
    rclpy.init()
    node = Collector(args.topic)
    print('=' * 60)
    print(' Orbbec 교차점 수집 — SPACE=저장 a=자동 f=강제 q=종료')
    print('=' * 60)
    win = 'orbbec collect (SPACE=save a=auto f=force q=quit)'
    try:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL); cv2.resizeWindow(win, 1280, 800)
    except Exception as e:
        print(f'⚠️ 디스플레이 없음({e})'); node.destroy_node(); rclpy.shutdown(); return

    auto, last_auto = False, time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            img = node.latest
            if img is not None:
                sh = node.sharpness(img)
                disp = img.copy() if img.shape[1] <= 1280 else cv2.resize(img, (1280, 800))
                col = (0, 255, 0) if sh >= 30 else (0, 0, 255)
                cv2.putText(disp, f'saved:{node.n}  sharp:{sh:.0f}  AUTO:{"ON" if auto else "off"}',
                            (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)
                cv2.putText(disp, 'SPACE=save  a=auto  f=force  q=quit',
                            (10, disp.shape[0]-12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                cv2.imshow(win, disp)
            if auto and img is not None and time.time()-last_auto > 1.5:
                if node.changed(img) and node.sharpness(img) >= 30:
                    node.save()
                last_auto = time.time()
            k = cv2.waitKey(20) & 0xFF
            if k in (ord('q'), 27):
                break
            elif k in (ord(' '), ord('s')):
                node.save()
            elif k == ord('a'):
                auto = not auto; print(f'  자동수집 {"ON" if auto else "OFF"}')
            elif k == ord('f'):
                node.save(force=True)
    finally:
        cv2.destroyAllWindows()
        cnt = len([f for f in os.listdir(OUT_DIR) if f.startswith('ocross_')])
        print(f'\n총 {cnt}장 → {OUT_DIR}')
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()

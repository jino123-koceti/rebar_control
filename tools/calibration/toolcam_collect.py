#!/usr/bin/env python3
"""툴캠(zedxone) 교차점 학습 데이터 수집기.
로봇을 여러 결속포인트로 옮겨가며 프레임 저장 → YOLO 파인튜닝용.

대화형:  python3 toolcam_collect.py
  Enter   : 현재 프레임 1장 저장 (선명도 체크 — 흐리면 경고)
  a       : 자동수집 토글 (1.5초 간격, 변화있는 프레임만)
  s       : 현재 수집 현황
  q       : 종료

저장: data/toolcam_dataset/images/tcam_NNNN.png
"""
import os
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

OUT_DIR = '/home/koceti/ros2_ws/data/toolcam_dataset/images'
TOPIC = '/zedxone/zed_node/rgb/rect/image'


class Collector(Node):
    def __init__(self):
        super().__init__('toolcam_collect')
        self.br = CvBridge()
        self.latest = None
        self.create_subscription(Image, TOPIC, self._cb, qos_profile_sensor_data)
        os.makedirs(OUT_DIR, exist_ok=True)
        # 기존 번호 이어가기
        existing = [f for f in os.listdir(OUT_DIR) if f.startswith('tcam_')]
        self.n = max([int(f[5:9]) for f in existing], default=0)
        self.last_saved = None
        self.get_logger().info(f'기존 {len(existing)}장, 다음번호 {self.n+1}')

    def _cb(self, m):
        self.latest = self.br.imgmsg_to_cv2(m, 'bgr8')

    def spin(self, sec):
        t = time.time()
        while time.time() - t < sec:
            rclpy.spin_once(self, timeout_sec=0.05)

    def sharpness(self, img):
        g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return cv2.Laplacian(g, cv2.CV_64F).var()

    def changed(self, img, thresh=12.0):
        if self.last_saved is None:
            return True
        a = cv2.resize(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (160, 100))
        b = cv2.resize(cv2.cvtColor(self.last_saved, cv2.COLOR_BGR2GRAY), (160, 100))
        return np.mean(np.abs(a.astype(int) - b.astype(int))) > thresh

    def save(self, force=False):
        if self.latest is None:
            print('  영상 없음'); return False
        img = self.latest
        sh = self.sharpness(img)
        if not force and sh < 4.0:   # zedxone 정상=~7-11, <4면 심한 모션블러
            print(f'  ⚠️ 모션블러 의심(sharpness {sh:.0f}) — 정지 후 다시 (강제: f)')
            return False
        self.n += 1
        p = os.path.join(OUT_DIR, f'tcam_{self.n:04d}.png')
        cv2.imwrite(p, img)
        self.last_saved = img.copy()
        print(f'  ✅ 저장 tcam_{self.n:04d}.png (sharpness {sh:.0f})')
        return True


TARGET = (477, 886)   # 결속 목표픽셀 (전체뷰 1920x1200)


def main():
    rclpy.init()
    node = Collector()
    print('=' * 60)
    print(' 툴캠 라이브 수집 — 창에서 보고 캡처')
    print('   [SPACE/s]=저장  [a]=자동토글  [f]=강제저장  [q/ESC]=종료')
    print('=' * 60)
    win = 'toolcam (SPACE=save, a=auto, q=quit)'
    try:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win, 960, 600)
    except Exception as e:
        print(f'⚠️ 디스플레이 없음({e}) — DISPLAY 환경변수 필요. 헤드리스면 알려주세요.')
        node.destroy_node(); rclpy.shutdown(); return

    auto = False
    last_auto = time.time()
    scale = 0.5
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.02)
            img = node.latest
            if img is not None:
                sh = node.sharpness(img)
                disp = cv2.resize(img, None, fx=scale, fy=scale)
                # 목표픽셀
                tx, ty = int(TARGET[0]*scale), int(TARGET[1]*scale)
                cv2.drawMarker(disp, (tx, ty), (255, 0, 255),
                               cv2.MARKER_TILTED_CROSS, 26, 2)
                cv2.putText(disp, 'TARGET', (tx+14, ty), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (255, 0, 255), 1)
                cnt = node.n
                col = (0, 255, 0) if sh >= 4 else (0, 0, 255)
                cv2.putText(disp, f'saved:{cnt}  sharp:{sh:.0f}'
                            f'  AUTO:{"ON" if auto else "off"}',
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
                cv2.putText(disp, 'SPACE=save  a=auto  f=force  q=quit',
                            (10, disp.shape[0]-12), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 255), 1)
                cv2.imshow(win, disp)

            if auto and img is not None and time.time()-last_auto > 1.5:
                if node.changed(img) and node.sharpness(img) >= 4:
                    node.save()
                last_auto = time.time()

            key = cv2.waitKey(30) & 0xFF
            if key in (ord('q'), 27):
                break
            elif key in (ord(' '), ord('s')):
                node.save()
            elif key == ord('a'):
                auto = not auto
                print(f'  자동수집 {"ON" if auto else "OFF"}')
            elif key == ord('f'):
                node.save(force=True)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        cnt = len([f for f in os.listdir(OUT_DIR) if f.startswith('tcam_')])
        print(f'\n총 {cnt}장 수집 → {OUT_DIR}')
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()

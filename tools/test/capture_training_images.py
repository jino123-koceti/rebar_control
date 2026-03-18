#!/usr/bin/env python3
"""
YOLO 학습용 이미지 수집 스크립트

사용법:
  python3 capture_training_images.py
  python3 capture_training_images.py --output_dir captured_training
  python3 capture_training_images.py --fps 3

동작:
  1. 실행 후 Enter 누르면 캡처 시작 (2fps)
  2. 리모콘으로 장비 주행하면서 다양한 위치에서 이미지 수집
  3. 'q' 또는 Ctrl+C로 종료

출력:
  {output_dir}/left/left_000001.jpg
  {output_dir}/right/right_000001.jpg
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import sys
import time
import threading
import select

# 카메라 토픽 (rebar_detection_node.py와 동일)
LEFT_TOPIC = '/zedxmini1/zed_node/left/image_rect_color'
RIGHT_TOPIC = '/zedxmini2/zed_node/left/image_rect_color'


class TrainingImageCapture(Node):
    def __init__(self, output_dir='captured_training', fps=2.0):
        super().__init__('training_image_capture')
        self.bridge = CvBridge()
        self.output_dir = output_dir
        self.capture_interval = 1.0 / fps
        self.fps = fps

        # 최신 이미지 버퍼
        self.left_image = None
        self.right_image = None
        self.left_lock = threading.Lock()
        self.right_lock = threading.Lock()

        # 상태
        self.capturing = False
        self.capture_count = 0
        self.start_time = None

        # 출력 디렉토리 생성
        self.left_dir = os.path.join(output_dir, 'left_cam')
        self.right_dir = os.path.join(output_dir, 'right_cam')
        os.makedirs(self.left_dir, exist_ok=True)
        os.makedirs(self.right_dir, exist_ok=True)

        # 기존 이미지 번호 확인 (이어서 저장)
        existing = []
        for d in [self.left_dir, self.right_dir]:
            for f in os.listdir(d):
                if f.endswith('.jpg'):
                    try:
                        num = int(f.split('_')[-1].replace('.jpg', ''))
                        existing.append(num)
                    except ValueError:
                        pass
        self.capture_count = max(existing) if existing else 0

        # 구독자
        self.left_sub = self.create_subscription(
            Image, LEFT_TOPIC, self._left_cb, 1)
        self.right_sub = self.create_subscription(
            Image, RIGHT_TOPIC, self._right_cb, 1)

        self.get_logger().info(f'Left topic: {LEFT_TOPIC}')
        self.get_logger().info(f'Right topic: {RIGHT_TOPIC}')
        self.get_logger().info(f'Output: {os.path.abspath(output_dir)}/')
        self.get_logger().info(f'FPS: {fps}, 기존 이미지: {self.capture_count}장')

    def _left_cb(self, msg):
        with self.left_lock:
            self.left_image = msg

    def _right_cb(self, msg):
        with self.right_lock:
            self.right_image = msg

    def save_pair(self):
        """좌우 카메라 이미지 한 쌍 저장"""
        with self.left_lock:
            left_msg = self.left_image
        with self.right_lock:
            right_msg = self.right_image

        if left_msg is None and right_msg is None:
            return False

        self.capture_count += 1
        saved = []

        if left_msg is not None:
            try:
                img = self.bridge.imgmsg_to_cv2(left_msg, 'bgr8')
                path = os.path.join(self.left_dir, f'left_{self.capture_count:06d}.jpg')
                cv2.imwrite(path, img, [cv2.IMWRITE_JPEG_QUALITY, 95])
                saved.append('L')
            except Exception as e:
                self.get_logger().warn(f'Left save error: {e}')

        if right_msg is not None:
            try:
                img = self.bridge.imgmsg_to_cv2(right_msg, 'bgr8')
                path = os.path.join(self.right_dir, f'right_{self.capture_count:06d}.jpg')
                cv2.imwrite(path, img, [cv2.IMWRITE_JPEG_QUALITY, 95])
                saved.append('R')
            except Exception as e:
                self.get_logger().warn(f'Right save error: {e}')

        if saved:
            elapsed = time.monotonic() - self.start_time
            print(f'  [{self.capture_count:5d}] {"+".join(saved)} saved  '
                  f'({elapsed:.1f}s elapsed)', end='\r')
            return True
        return False


def main():
    import argparse
    parser = argparse.ArgumentParser(description='YOLO 학습용 이미지 수집')
    parser.add_argument('--output_dir', default='captured_training',
                        help='저장 디렉토리 (default: captured_training)')
    parser.add_argument('--fps', type=float, default=2.0,
                        help='초당 캡처 수 (default: 2.0)')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = TrainingImageCapture(output_dir=args.output_dir, fps=args.fps)

    # ROS spin 쓰레드
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print('=' * 60)
    print(' YOLO 학습용 이미지 수집')
    print('=' * 60)
    print(f'  저장 위치: {os.path.abspath(args.output_dir)}/')
    print(f'  캡처 속도: {args.fps} fps')
    print(f'  기존 이미지: {node.capture_count}장')
    print()

    # 카메라 연결 대기
    print('카메라 연결 대기 중...')
    timeout = time.monotonic() + 10.0
    while time.monotonic() < timeout:
        if node.left_image is not None or node.right_image is not None:
            cams = []
            if node.left_image is not None:
                cams.append('Left')
            if node.right_image is not None:
                cams.append('Right')
            print(f'  카메라 연결됨: {", ".join(cams)}')
            break
        time.sleep(0.5)
    else:
        print('[ERROR] 카메라 이미지 수신 안됨. 토픽 확인 필요.')
        node.destroy_node()
        rclpy.shutdown()
        return

    print()
    print('-' * 60)
    print('  [Enter] = 캡처 시작/일시정지')
    print('  [q]     = 종료')
    print('-' * 60)

    try:
        while True:
            cmd = input('\n[Enter]=시작, [q]=종료: ').strip().lower()
            if cmd == 'q':
                break

            # 캡처 시작
            node.capturing = True
            node.start_time = time.monotonic()
            start_count = node.capture_count
            print(f'\n>>> 캡처 시작 ({args.fps} fps) - [Enter]로 일시정지, [q]로 종료')
            print()

            last_save = 0
            while True:
                # 비차단 키 입력 체크
                if select.select([sys.stdin], [], [], node.capture_interval)[0]:
                    key = sys.stdin.readline().strip().lower()
                    if key == 'q':
                        node.capturing = False
                        session_count = node.capture_count - start_count
                        print(f'\n\n종료. 이번 세션: {session_count}장 저장')
                        raise KeyboardInterrupt
                    elif key == '':
                        # Enter = 일시정지
                        node.capturing = False
                        session_count = node.capture_count - start_count
                        print(f'\n\n>>> 일시정지. 이번 세션: {session_count}장, 총: {node.capture_count}장')
                        break

                now = time.monotonic()
                if now - last_save >= node.capture_interval:
                    node.save_pair()
                    last_save = now

    except KeyboardInterrupt:
        print('\n\n종료 (Ctrl+C)')
    finally:
        total = node.capture_count
        print(f'\n총 저장: {total}장')
        print(f'  Left:  {node.left_dir}/')
        print(f'  Right: {node.right_dir}/')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

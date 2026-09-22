#!/usr/bin/env python3
"""
철근 데이터셋 캡처 스크립트

전후진 카메라(front=zedxmini2, back=zedxmini1) 영상을 2fps로 저장.
리모콘 매뉴얼 주행 중 실행하여 YOLO 학습 데이터 획득.

사용법:
  python3 capture_rebar_dataset.py                # 양쪽 카메라 (기본)
  python3 capture_rebar_dataset.py --camera front  # 전진 카메라만
  python3 capture_rebar_dataset.py --camera back   # 후진 카메라만
  python3 capture_rebar_dataset.py --fps 1         # 1fps로 저장

저장 경로: ros2_ws/data/images/rebar_dataset/{front,back}/YYYYMMDD_HHMMSS_###.jpg
"""

import argparse
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


SAVE_DIR = '/home/koceti/ros2_ws/data/images/rebar_dataset'

CAMERA_TOPICS = {
    'front': '/zedxmini2/zed_node/left/image_rect_color',
    'back': '/zedxmini1/zed_node/left/image_rect_color',
}


class RebarDatasetCapture(Node):
    def __init__(self, cameras, fps):
        super().__init__('rebar_dataset_capture')
        self.bridge = CvBridge()
        self.fps = fps
        self.interval = 1.0 / fps
        self.cameras = cameras
        self.counters = {}
        self.last_save_time = {}
        self.latest_image = {}

        session = datetime.now().strftime('%Y%m%d_%H%M%S')

        for cam in cameras:
            save_dir = os.path.join(SAVE_DIR, cam, session)
            os.makedirs(save_dir, exist_ok=True)
            self.counters[cam] = 0
            self.last_save_time[cam] = 0.0
            self.latest_image[cam] = None

            topic = CAMERA_TOPICS[cam]
            self.create_subscription(
                Image, topic,
                lambda msg, c=cam: self._image_cb(msg, c),
                qos_profile_sensor_data)
            self.get_logger().info(f'[{cam}] 구독: {topic} → {save_dir}')

        self.save_dirs = {
            cam: os.path.join(SAVE_DIR, cam, session) for cam in cameras
        }

        self.timer = self.create_timer(0.05, self._timer_cb)
        self.get_logger().info(
            f'캡처 시작: {fps}fps, 카메라: {", ".join(cameras)}')

    def _image_cb(self, msg, cam):
        self.latest_image[cam] = msg

    def _timer_cb(self):
        now = time.monotonic()
        for cam in self.cameras:
            if self.latest_image[cam] is None:
                continue
            if now - self.last_save_time[cam] < self.interval:
                continue

            try:
                img = self.bridge.imgmsg_to_cv2(
                    self.latest_image[cam], 'bgr8')
                self.counters[cam] += 1
                fname = f'{self.counters[cam]:06d}.jpg'
                fpath = os.path.join(self.save_dirs[cam], fname)
                cv2.imwrite(fpath, img, [cv2.IMWRITE_JPEG_QUALITY, 95])
                self.last_save_time[cam] = now

                if self.counters[cam] % 10 == 0:
                    total = sum(self.counters.values())
                    status = '  '.join(
                        f'{c}:{self.counters[c]}' for c in self.cameras)
                    self.get_logger().info(f'저장: {status}  (총 {total})')

            except Exception as e:
                self.get_logger().error(f'[{cam}] 저장 실패: {e}')


def main():
    parser = argparse.ArgumentParser(description='철근 데이터셋 캡처')
    parser.add_argument('--camera', choices=['front', 'back', 'both'],
                        default='both', help='카메라 선택 (기본: both)')
    parser.add_argument('--fps', type=float, default=2.0,
                        help='초당 프레임 수 (기본: 2)')
    args, ros_args = parser.parse_known_args()

    cameras = ['front', 'back'] if args.camera == 'both' else [args.camera]

    rclpy.init(args=ros_args)
    node = RebarDatasetCapture(cameras, args.fps)

    print('=' * 50)
    print(f' 철근 데이터셋 캡처 ({args.fps}fps)')
    print(f' 카메라: {", ".join(cameras)}')
    print(f' 저장: {SAVE_DIR}/')
    print(f' Ctrl+C로 종료')
    print('=' * 50)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        total = sum(node.counters.values())
        status = '  '.join(
            f'{c}:{node.counters[c]}' for c in cameras)
        print(f'\n캡처 종료: {status}  (총 {total}장)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

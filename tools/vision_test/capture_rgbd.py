#!/usr/bin/env python3
"""
Phase 1: ZED 카메라에서 RGB + Depth 프레임 캡처

사용법:
    python3 capture_rgbd.py                    # 양쪽 카메라 1프레임 캡처
    python3 capture_rgbd.py --camera left      # 좌측만
    python3 capture_rgbd.py --camera right     # 우측만
    python3 capture_rgbd.py --frames 5         # 5프레임 캡처 (temporal median용)

저장 위치: data/vision_test/YYYYMMDD_HHMMSS/
    left_rgb.png, left_depth.npy, left_depth_vis.png
    right_rgb.png, right_depth.npy, right_depth_vis.png
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2
import os
import argparse
import time
from datetime import datetime


class RGBDCapture(Node):
    def __init__(self, cameras, num_frames):
        super().__init__('rgbd_capture')
        self.bridge = CvBridge()
        self.cameras = cameras
        self.num_frames = num_frames

        # 수집 버퍼
        self.data = {}
        for cam in cameras:
            self.data[cam] = {
                'rgb': None,
                'depth_frames': [],
                'camera_info': None,
                'done': False,
            }

        # 토픽 매핑
        topic_map = {
            'left': {
                'rgb': '/zedxmini1/zed_node/left/image_rect_color',
                'depth': '/zedxmini1/zed_node/depth/depth_registered',
                'info': '/zedxmini1/zed_node/left/camera_info',
            },
            'right': {
                'rgb': '/zedxmini2/zed_node/left/image_rect_color',
                'depth': '/zedxmini2/zed_node/depth/depth_registered',
                'info': '/zedxmini2/zed_node/left/camera_info',
            },
        }

        # 구독 생성
        for cam in cameras:
            topics = topic_map[cam]
            self.create_subscription(
                Image, topics['rgb'],
                lambda msg, c=cam: self._rgb_cb(msg, c), 1)
            self.create_subscription(
                Image, topics['depth'],
                lambda msg, c=cam: self._depth_cb(msg, c), 1)
            self.create_subscription(
                CameraInfo, topics['info'],
                lambda msg, c=cam: self._info_cb(msg, c), 1)

        self.get_logger().info(
            f'캡처 시작: 카메라={cameras}, 프레임수={num_frames}')

    def _rgb_cb(self, msg, cam):
        if self.data[cam]['done']:
            return
        self.data[cam]['rgb'] = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _depth_cb(self, msg, cam):
        if self.data[cam]['done']:
            return
        depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')  # float32, meters
        buf = self.data[cam]['depth_frames']
        if len(buf) < self.num_frames:
            buf.append(depth.copy())
            self.get_logger().info(
                f'[{cam}] depth 프레임 {len(buf)}/{self.num_frames}')

    def _info_cb(self, msg, cam):
        if self.data[cam]['camera_info'] is None:
            self.data[cam]['camera_info'] = {
                'fx': msg.k[0], 'fy': msg.k[4],
                'cx': msg.k[2], 'cy': msg.k[5],
                'width': msg.width, 'height': msg.height,
            }

    def is_done(self):
        for cam in self.cameras:
            d = self.data[cam]
            if d['rgb'] is None:
                return False
            if len(d['depth_frames']) < self.num_frames:
                return False
        return True

    def save(self, output_dir):
        os.makedirs(output_dir, exist_ok=True)

        for cam in self.cameras:
            d = self.data[cam]
            rgb = d['rgb']
            depth_stack = np.stack(d['depth_frames'], axis=0)  # (N, H, W)

            # Temporal median depth
            depth_median = np.nanmedian(depth_stack, axis=0)  # (H, W) meters

            # 저장: RGB
            cv2.imwrite(os.path.join(output_dir, f'{cam}_rgb.png'), rgb)

            # 저장: Depth raw (float32 npy, meters)
            np.save(os.path.join(output_dir, f'{cam}_depth.npy'), depth_median)

            # 저장: Depth 16bit PNG (mm 단위)
            depth_mm = np.nan_to_num(depth_median * 1000.0, nan=0.0)
            depth_u16 = np.clip(depth_mm, 0, 65535).astype(np.uint16)
            cv2.imwrite(os.path.join(output_dir, f'{cam}_depth_u16.png'), depth_u16)

            # 저장: Depth 시각화 (TURBO colormap, 0~2m 범위)
            depth_vis = np.nan_to_num(depth_median, nan=0.0)
            depth_norm = np.clip(depth_vis / 2.0, 0.0, 1.0)  # 0~2m → 0~1
            depth_color = cv2.applyColorMap(
                (depth_norm * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
            # 무효 영역 검정
            invalid = (depth_vis <= 0.01) | ~np.isfinite(depth_median)
            depth_color[invalid] = [0, 0, 0]
            cv2.imwrite(os.path.join(output_dir, f'{cam}_depth_vis.png'), depth_color)

            # 저장: camera info
            if d['camera_info']:
                import json
                with open(os.path.join(output_dir, f'{cam}_camera_info.json'), 'w') as f:
                    json.dump(d['camera_info'], f, indent=2)

            self.get_logger().info(
                f'[{cam}] 저장 완료: RGB={rgb.shape}, '
                f'Depth median={np.nanmin(depth_median):.3f}~{np.nanmax(depth_median):.3f}m')

        self.get_logger().info(f'저장 경로: {output_dir}')


def main():
    parser = argparse.ArgumentParser(description='ZED RGB+Depth 캡처')
    parser.add_argument('--camera', choices=['left', 'right', 'both'],
                        default='both', help='캡처할 카메라')
    parser.add_argument('--frames', type=int, default=5,
                        help='temporal median용 depth 프레임 수')
    parser.add_argument('--output', type=str, default=None,
                        help='저장 경로 (기본: data/vision_test/타임스탬프/)')
    args = parser.parse_args()

    cameras = ['left', 'right'] if args.camera == 'both' else [args.camera]

    if args.output is None:
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        args.output = f'/home/koceti/ros2_ws/data/vision_test/{ts}'

    rclpy.init()
    node = RGBDCapture(cameras, args.frames)

    timeout = 15.0
    start = time.monotonic()
    while not node.is_done():
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.monotonic() - start > timeout:
            node.get_logger().warn(f'{timeout}초 타임아웃 - 수신된 데이터만 저장')
            break

    node.save(args.output)
    node.destroy_node()
    rclpy.shutdown()
    print(f'\n캡처 완료: {args.output}')


if __name__ == '__main__':
    main()

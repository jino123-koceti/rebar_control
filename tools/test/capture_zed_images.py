#!/usr/bin/env python3
"""ZED 듀얼 카메라 이미지 뷰어 + 캡처 도구

키 입력:
  s: 양쪽 이미지 저장
  1: 좌측(zedxmini1)만 저장
  2: 우측(zedxmini2)만 저장
  q/ESC: 종료
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from datetime import datetime


class ZedImageViewer(Node):
    def __init__(self):
        super().__init__('zed_image_viewer')

        self.bridge = CvBridge()
        self.left_image = None   # zedxmini1
        self.right_image = None  # zedxmini2

        # 저장 경로
        self.save_dir = os.path.expanduser('~/ros2_ws/captured_images')
        os.makedirs(f'{self.save_dir}/left', exist_ok=True)
        os.makedirs(f'{self.save_dir}/right', exist_ok=True)
        self.capture_count = 0

        # 구독
        self.create_subscription(
            Image,
            '/zedxmini1/zed_node/left/image_rect_color',
            self._left_cb, 10
        )
        self.create_subscription(
            Image,
            '/zedxmini2/zed_node/left/image_rect_color',
            self._right_cb, 10
        )

        # 30Hz 디스플레이 타이머
        self.create_timer(1.0 / 30.0, self._display)

        self.get_logger().info(f'저장 경로: {self.save_dir}')
        self.get_logger().info('키: s=양쪽저장, 1=좌측저장, 2=우측저장, q=종료')

    def _left_cb(self, msg):
        self.left_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _right_cb(self, msg):
        self.right_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _display(self):
        # 플레이스홀더 (토픽 미수신 시)
        h, w = 720, 1280
        left = self.left_image if self.left_image is not None else np.zeros((h, w, 3), dtype=np.uint8)
        right = self.right_image if self.right_image is not None else np.zeros((h, w, 3), dtype=np.uint8)

        # 라벨 표시
        left_disp = left.copy()
        right_disp = right.copy()
        cv2.putText(left_disp, 'LEFT (zedxmini1)', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        cv2.putText(right_disp, 'RIGHT (zedxmini2)', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

        # 리사이즈해서 나란히 표시
        scale = 0.5
        left_small = cv2.resize(left_disp, None, fx=scale, fy=scale)
        right_small = cv2.resize(right_disp, None, fx=scale, fy=scale)
        combined = np.hstack([left_small, right_small])

        cv2.imshow('ZED Cameras (s:save, q:quit)', combined)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('s'):
            self._save_both()
        elif key == ord('1'):
            self._save_one('left', self.left_image)
        elif key == ord('2'):
            self._save_one('right', self.right_image)
        elif key in (ord('q'), 27):  # q or ESC
            self.get_logger().info('종료')
            cv2.destroyAllWindows()
            raise SystemExit

    def _save_both(self):
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        saved = []
        if self.left_image is not None:
            path = f'{self.save_dir}/left/{ts}_{self.capture_count:04d}.png'
            cv2.imwrite(path, self.left_image)
            saved.append(f'L:{path}')
        if self.right_image is not None:
            path = f'{self.save_dir}/right/{ts}_{self.capture_count:04d}.png'
            cv2.imwrite(path, self.right_image)
            saved.append(f'R:{path}')
        self.capture_count += 1
        self.get_logger().info(f'[{self.capture_count}] 저장: {", ".join(saved)}')

    def _save_one(self, side, image):
        if image is None:
            self.get_logger().warn(f'{side} 이미지 없음')
            return
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = f'{self.save_dir}/{side}/{ts}_{self.capture_count:04d}.png'
        cv2.imwrite(path, image)
        self.capture_count += 1
        self.get_logger().info(f'[{self.capture_count}] 저장: {path}')


def main():
    rclpy.init()
    node = ZedImageViewer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

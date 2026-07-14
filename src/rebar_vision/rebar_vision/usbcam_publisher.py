#!/usr/bin/env python3
"""
USB 캠(UVC) 퍼블리셔 — 결속 미세보정(top-view) proof용.

GMSL/Argus와 완전 별개(USB)라 ZED 카메라들과 대역폭/공존 충돌 없음.
어안 렌즈라 정밀 위치측정 시 결속영역(노랑) 근방 로컬 캘리브로 사용.

발행: /usbcam/image_raw (sensor_msgs/Image, bgr8, BEST_EFFORT)
"""
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class UsbCamPublisher(Node):
    def __init__(self):
        super().__init__('usbcam_publisher')
        self.declare_parameter('device', '/dev/video9')
        self.declare_parameter('width', 1920)
        self.declare_parameter('height', 1080)
        self.declare_parameter('fps', 15)
        self.declare_parameter('frame_id', 'usbcam')

        dev = self.get_parameter('device').value
        w = int(self.get_parameter('width').value)
        h = int(self.get_parameter('height').value)
        self.fps = int(self.get_parameter('fps').value)
        self.frame_id = self.get_parameter('frame_id').value

        self.bridge = CvBridge()
        self.pub = self.create_publisher(
            Image, '/usbcam/image_raw', qos_profile_sensor_data)

        self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error(f'USB 캠 열기 실패: {dev}')
            raise RuntimeError(f'cannot open {dev}')
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        aw = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        ah = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f'✅ USB 캠 열림 {dev} {aw}x{ah}@{self.fps}')

        self.timer = self.create_timer(1.0 / float(self.fps), self._tick)
        self._n = 0

    def _tick(self):
        r, f = self.cap.read()
        if not r or f is None:
            return
        msg = self.bridge.cv2_to_imgmsg(f, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)
        self._n += 1
        if self._n % 150 == 0:
            self.get_logger().info(f'USB 캠 발행 {self._n} 프레임')

    def destroy_node(self):
        try:
            self.cap.release()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = UsbCamPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

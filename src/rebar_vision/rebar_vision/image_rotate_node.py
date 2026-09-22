#!/usr/bin/env python3
"""이미지 180°(또는 90/270) 회전 재발행 노드.

Orbbec Gemini 305는 드라이버(color_rotation/color_flip/mirror)를 지원 안 함(silently skip).
좌측 305가 케이블 배선상 거꾸로(180°) 장착돼서, 여기서 회전 보정 후 재발행한다.
소비자(횡이동 seg·뷰어)는 회전된 토픽만 구독하면 정방향 프레임을 받는다.

파라미터:
  input_topic  : 원본 이미지 토픽 (기본 /camera_left/color/image_raw)
  output_topic : 회전 재발행 토픽 (기본 /camera_left/color/image_rotated)
  rotation     : 90 | 180 | 270 (기본 180)

  ros2 run rebar_vision image_rotate --ros-args \
    -p input_topic:=/camera_left/color/image_raw \
    -p output_topic:=/camera_left/color/image_rotated -p rotation:=180
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

_ROT = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180, 270: cv2.ROTATE_90_COUNTERCLOCKWISE}


class ImageRotate(Node):
    def __init__(self):
        super().__init__('image_rotate')
        self.declare_parameter('input_topic', '/camera_left/color/image_raw')
        self.declare_parameter('output_topic', '/camera_left/color/image_rotated')
        self.declare_parameter('rotation', 180)

        self.in_topic = self.get_parameter('input_topic').value
        self.out_topic = self.get_parameter('output_topic').value
        deg = int(self.get_parameter('rotation').value)
        self.rot_code = _ROT.get(deg)
        if self.rot_code is None:
            self.get_logger().error(f'rotation은 90/180/270만 지원 (받음: {deg}) → 180으로 대체')
            self.rot_code = cv2.ROTATE_180
            deg = 180

        self.br = CvBridge()
        # QoS는 센서 기본(sensor_data)에 맞춰 depth=10
        self.pub = self.create_publisher(Image, self.out_topic, 10)
        self.sub = self.create_subscription(Image, self.in_topic, self.cb, 10)
        self.get_logger().info(
            f'이미지 회전 {deg}°: {self.in_topic} → {self.out_topic}')

    def cb(self, msg):
        try:
            # passthrough로 인코딩 보존(컬러 bgr8/rgb8, depth 16UC1 모두 대응)
            img = self.br.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            out = cv2.rotate(img, self.rot_code)
            out_msg = self.br.cv2_to_imgmsg(out, encoding=msg.encoding)
            out_msg.header = msg.header          # stamp/frame_id 보존
            self.pub.publish(out_msg)
        except Exception as e:
            self.get_logger().error(f'회전 실패: {e}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = ImageRotate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

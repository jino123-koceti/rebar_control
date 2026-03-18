#!/usr/bin/env python3
"""
테스트용 Odometry 발행 노드
odom_to_pose 노드의 변환 기능을 테스트하기 위한 간단한 발행자
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Pose
import math


class TestOdomPublisher(Node):
    def __init__(self):
        super().__init__('test_odom_publisher')

        self.publisher = self.create_publisher(Odometry, '/test/odom', 10)
        self.timer = self.create_timer(0.1, self.publish_odom)  # 10Hz
        self.counter = 0

        self.get_logger().info('Test Odometry Publisher 시작')
        self.get_logger().info('토픽: /test/odom')

    def publish_odom(self):
        msg = Odometry()

        # 헤더
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'

        # 위치 (원 궤적)
        t = self.counter * 0.1
        msg.pose.pose.position = Point(
            x=math.cos(t),
            y=math.sin(t),
            z=0.0
        )

        # 자세 (yaw 회전)
        msg.pose.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(t/2),
            w=math.cos(t/2)
        )

        self.publisher.publish(msg)
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = TestOdomPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

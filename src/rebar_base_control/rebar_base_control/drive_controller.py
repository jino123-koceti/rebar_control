#!/usr/bin/env python3
"""
Drive Controller Node
Twist (cmd_vel) → DriveControl 변환

Differential drive kinematics를 사용하여
선속도/각속도를 좌우 모터 속도로 변환
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rebar_base_interfaces.msg import DriveControl


class DriveController(Node):
    """cmd_vel을 DriveControl로 변환하는 노드"""

    def __init__(self):
        super().__init__('drive_controller')

        # 파라미터 선언
        self.declare_parameter('wheel_base', 0.5)  # m, 바퀴 간 거리
        self.declare_parameter('max_linear_vel', 10.0)  # m/s
        self.declare_parameter('max_angular_vel', 2.0)  # rad/s

        # 파라미터 가져오기
        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_linear = self.get_parameter('max_linear_vel').value
        self.max_angular = self.get_parameter('max_angular_vel').value

        # ROS2 Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # ROS2 Publisher
        self.drive_control_pub = self.create_publisher(
            DriveControl,
            '/drive_control',
            10
        )

        self.get_logger().info("Drive Controller 노드 초기화 완료")
        self.get_logger().info(f"  - Wheel base: {self.wheel_base} m")
        self.get_logger().info(f"  - Max linear: {self.max_linear} m/s")
        self.get_logger().info(f"  - Max angular: {self.max_angular} rad/s")

    def cmd_vel_callback(self, msg):
        """
        Twist 메시지를 DriveControl로 변환

        Differential Drive Kinematics:
        - v_left = v_linear - (omega * wheel_base / 2)
        - v_right = v_linear + (omega * wheel_base / 2)

        단, 오른쪽 모터는 하드웨어적으로 반대 방향이므로
        can_sender에서 반전 처리
        """
        try:
            # 입력 제한
            linear = max(-self.max_linear, min(self.max_linear, msg.linear.x))
            angular = max(-self.max_angular, min(self.max_angular, msg.angular.z))

            # Differential drive 변환
            # v_left = linear - angular * wheel_base / 2
            # v_right = linear + angular * wheel_base / 2
            left_speed = linear - (angular * self.wheel_base / 2.0)
            right_speed = linear + (angular * self.wheel_base / 2.0)

            # DriveControl 메시지 생성
            drive_msg = DriveControl()
            drive_msg.left_speed = left_speed
            drive_msg.right_speed = right_speed

            # 발행
            self.drive_control_pub.publish(drive_msg)

        except Exception as e:
            self.get_logger().error(f"cmd_vel 변환 오류: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DriveController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

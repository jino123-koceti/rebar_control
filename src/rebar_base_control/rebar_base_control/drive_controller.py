#!/usr/bin/env python3
"""
Drive Controller Node
리모콘(RemoteControl) 또는 cmd_vel → DriveControl 변환

Tire Roller 방식:
- Manual 모드: 리모콘 조이스틱 값 직접 변환
- Auto 모드: cmd_vel (Twist) 사용
- Differential drive kinematics 적용
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rebar_base_interfaces.msg import DriveControl, RemoteControl
from std_msgs.msg import String


class DriveController(Node):
    """리모콘과 cmd_vel을 DriveControl로 변환하는 통합 노드 (Tire Roller 방식)"""

    def __init__(self):
        super().__init__('drive_controller')

        # 파라미터 선언
        self.declare_parameter('wheel_base', 0.5)  # m, 바퀴 간 거리
        self.declare_parameter('max_linear_vel', 10.0)  # m/s
        self.declare_parameter('max_angular_vel', 2.0)  # rad/s
        self.declare_parameter('joystick_deadzone', 20)  # 조이스틱 데드존
        self.declare_parameter('joystick_center', 127)  # 조이스틱 중립값
        self.declare_parameter('publish_frequency', 20)  # Hz

        # 파라미터 가져오기
        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_linear = self.get_parameter('max_linear_vel').value
        self.max_angular = self.get_parameter('max_angular_vel').value
        self.joystick_deadzone = self.get_parameter('joystick_deadzone').value
        self.joystick_center = self.get_parameter('joystick_center').value
        self.publish_frequency = self.get_parameter('publish_frequency').value

        # 상태 변수
        self.control_mode = 'idle'  # 'idle', 'manual', 'auto', 'emergency_stop'
        self.cmd_vel_msg = Twist()
        self.remote_control_msg = RemoteControl()
        self.last_drive_msg = DriveControl()

        # ROS2 Subscribers
        # Auto 모드용: cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Manual 모드용: 리모콘
        self.remote_control_sub = self.create_subscription(
            RemoteControl,
            '/remote_control',
            self.remote_control_callback,
            10
        )

        # 제어 모드 구독
        self.control_mode_sub = self.create_subscription(
            String,
            '/control_mode',
            self.control_mode_callback,
            10
        )

        # ROS2 Publisher
        self.drive_control_pub = self.create_publisher(
            DriveControl,
            '/drive_control',
            10
        )

        # 주기적 발행 타이머 (Tire Roller 방식)
        self.timer = self.create_timer(
            1.0 / self.publish_frequency,
            self.publish_drive_control
        )

        self.get_logger().info("Drive Controller 노드 초기화 완료 (Tire Roller 방식)")
        self.get_logger().info(f"  - Wheel base: {self.wheel_base} m")
        self.get_logger().info(f"  - Max linear: {self.max_linear} m/s")
        self.get_logger().info(f"  - Max angular: {self.max_angular} rad/s")
        self.get_logger().info(f"  - Publish frequency: {self.publish_frequency} Hz")

    def control_mode_callback(self, msg):
        """제어 모드 업데이트"""
        old_mode = self.control_mode
        self.control_mode = msg.data

        if old_mode != self.control_mode:
            self.get_logger().info(f"제어 모드 변경: {old_mode} → {self.control_mode}")

    def cmd_vel_callback(self, msg):
        """cmd_vel 저장 (Auto 모드에서 사용)"""
        self.cmd_vel_msg = msg

    def remote_control_callback(self, msg):
        """리모콘 신호 저장 (Manual 모드에서 사용)"""
        self.remote_control_msg = msg

    def publish_drive_control(self):
        """
        타이머 콜백: 현재 모드에 따라 DriveControl 발행

        - manual: 리모콘 조이스틱 → DriveControl
        - auto: cmd_vel → DriveControl
        - 기타: 정지
        """
        drive_msg = DriveControl()

        if self.control_mode == 'manual':
            # Manual 모드: 리모콘 조이스틱 변환
            drive_msg = self._convert_remote_to_drive()

        elif self.control_mode == 'auto' or self.control_mode == 'navigating':
            # Auto 모드: cmd_vel 변환
            drive_msg = self._convert_cmd_vel_to_drive()

        else:
            # idle, emergency_stop 등: 정지
            drive_msg.left_speed = 0.0
            drive_msg.right_speed = 0.0

        # 발행
        self.drive_control_pub.publish(drive_msg)
        self.last_drive_msg = drive_msg

    def _convert_remote_to_drive(self):
        """
        리모콘 조이스틱 → DriveControl 변환

        조이스틱 매핑 (iron_md_teleop_node 참조):
        - joysticks[2] (AN3): 전후진 (중립=0.0, 전진=-1.0~0.0, 후진=0.0~1.0)
        - joysticks[3] (AN4): 좌우회전 (중립=0.0, CCW=0.0~1.0, CW=-1.0~0.0)
        """
        msg = self.remote_control_msg
        drive_msg = DriveControl()

        # 조이스틱 값 가져오기 (-1.0 ~ 1.0 범위)
        if len(msg.joysticks) >= 4:
            joy_linear = msg.joysticks[2]   # AN3: 전후진
            joy_angular = msg.joysticks[3]  # AN4: 좌우회전
        else:
            joy_linear = 0.0
            joy_angular = 0.0

        # 데드존 적용 (정규화된 값 기준: -0.157 ~ 0.157 정도)
        deadzone_normalized = self.joystick_deadzone / 127.0
        if abs(joy_linear) < deadzone_normalized:
            joy_linear = 0.0
        if abs(joy_angular) < deadzone_normalized:
            joy_angular = 0.0

        # 조이스틱 값 → 선속도/각속도 변환
        # AN3-: 전진, AN3+: 후진
        linear_velocity = -joy_linear * self.max_linear  # 부호 반전 (AN3- = 전진)

        # AN4+: CCW (왼쪽), AN4-: CW (오른쪽)
        angular_velocity = joy_angular * self.max_angular

        # Differential drive kinematics
        left_speed = linear_velocity - (angular_velocity * self.wheel_base / 2.0)
        right_speed = linear_velocity + (angular_velocity * self.wheel_base / 2.0)

        drive_msg.left_speed = left_speed
        drive_msg.right_speed = right_speed

        return drive_msg

    def _convert_cmd_vel_to_drive(self):
        """
        cmd_vel (Twist) → DriveControl 변환

        Differential Drive Kinematics:
        - v_left = v_linear - (omega * wheel_base / 2)
        - v_right = v_linear + (omega * wheel_base / 2)
        """
        msg = self.cmd_vel_msg
        drive_msg = DriveControl()

        try:
            # 입력 제한
            linear = max(-self.max_linear, min(self.max_linear, msg.linear.x))
            angular = max(-self.max_angular, min(self.max_angular, msg.angular.z))

            # Differential drive 변환
            left_speed = linear - (angular * self.wheel_base / 2.0)
            right_speed = linear + (angular * self.wheel_base / 2.0)

            drive_msg.left_speed = left_speed
            drive_msg.right_speed = right_speed

        except Exception as e:
            self.get_logger().error(f"cmd_vel 변환 오류: {e}")
            drive_msg.left_speed = 0.0
            drive_msg.right_speed = 0.0

        return drive_msg


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

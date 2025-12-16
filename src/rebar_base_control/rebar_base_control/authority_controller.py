#!/usr/bin/env python3
"""
Authority Controller Node
제어권한 관리 (Manual/Auto 전환)

S10 (S19): Manual 모드 - 리모콘 제어 허용
S20: Auto 모드 - UI/Navigator 제어만 허용, 리모콘 무시
"""

import rclpy
from rclpy.node import Node
from rebar_base_interfaces.msg import RemoteControl
from std_msgs.msg import String, Bool


class AuthorityController(Node):
    """제어권한을 관리하는 노드"""

    def __init__(self):
        super().__init__('authority_controller')

        # 현재 모드
        self.current_mode = 'idle'  # 'idle', 'manual', 'auto'
        self.emergency_stopped = False

        # ROS2 Subscribers
        self.remote_control_sub = self.create_subscription(
            RemoteControl,
            '/remote_control',
            self.remote_control_callback,
            10
        )

        # 상위 제어 계층에서의 모드 요청
        self.control_mode_request_sub = self.create_subscription(
            String,
            '/control_mode_request',
            self.control_mode_request_callback,
            10
        )

        # ROS2 Publishers
        self.authority_status_pub = self.create_publisher(
            String,
            '/authority_status',
            10
        )

        self.emergency_stop_pub = self.create_publisher(
            Bool,
            '/emergency_stop',
            10
        )

        # 상태 발행 타이머 (5Hz)
        self.status_timer = self.create_timer(0.2, self.publish_status)

        self.get_logger().info("Authority Controller 노드 초기화 완료")
        self.get_logger().info(f"초기 모드: {self.current_mode}")

    def remote_control_callback(self, msg):
        """
        리모콘 신호 처리
        - S10 (S19): Manual 모드
        - S20: Auto 모드
        - Emergency Stop
        """
        # 비상정지 처리
        if msg.emergency_stop:
            if not self.emergency_stopped:
                self.emergency_stopped = True
                self.current_mode = 'emergency_stop'
                self.get_logger().warn("🚨 비상정지 활성화")

                # 비상정지 메시지 발행
                estop_msg = Bool()
                estop_msg.data = True
                self.emergency_stop_pub.publish(estop_msg)
        else:
            if self.emergency_stopped:
                self.emergency_stopped = False
                self.current_mode = 'idle'
                self.get_logger().info("✅ 비상정지 해제")

                # 비상정지 해제 메시지 발행
                estop_msg = Bool()
                estop_msg.data = False
                self.emergency_stop_pub.publish(estop_msg)

        # 비상정지 중에는 모드 전환 불가
        if self.emergency_stopped:
            return

        # 모드 전환 처리
        new_mode = None
        if msg.switch_s10:
            # S10 (S19) = Manual 모드
            new_mode = 'manual'
        elif msg.switch_s20:
            # S20 = Auto 모드
            new_mode = 'auto'
        else:
            # 중립 = Idle
            new_mode = 'idle'

        # 모드 변경 감지
        if new_mode != self.current_mode:
            old_mode = self.current_mode
            self.current_mode = new_mode
            self.get_logger().info(f"모드 전환: {old_mode} → {new_mode}")

    def control_mode_request_callback(self, msg):
        """
        상위 제어 계층에서의 모드 요청 처리
        (Navigator 등에서 요청)
        """
        requested_mode = msg.data.lower()

        # Auto 모드에서만 상위 제어 허용
        if self.current_mode != 'auto':
            self.get_logger().warn(f"모드 요청 거부: 현재 {self.current_mode} 모드 (Auto 모드 필요)")
            return

        # 요청된 모드 처리
        if requested_mode in ['navigating', 'tying', 'homing']:
            self.get_logger().info(f"상위 제어 모드 요청: {requested_mode}")
            # navigator_base로 전달 (여기서는 로깅만)

    def publish_status(self):
        """현재 권한 상태 발행"""
        status_msg = String()
        status_msg.data = self.current_mode
        self.authority_status_pub.publish(status_msg)

    def is_manual_mode(self):
        """Manual 모드 여부"""
        return self.current_mode == 'manual'

    def is_auto_mode(self):
        """Auto 모드 여부"""
        return self.current_mode == 'auto'

    def is_emergency_stopped(self):
        """비상정지 상태 여부"""
        return self.emergency_stopped


def main(args=None):
    rclpy.init(args=args)
    node = AuthorityController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

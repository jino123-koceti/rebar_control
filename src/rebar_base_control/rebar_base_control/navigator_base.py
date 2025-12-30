#!/usr/bin/env python3
"""
Navigator Base Node
State Machine 기반 상태 관리

상태:
- idle: 초기 상태
- manual: S10 (S19) Manual 모드
- auto: S20 Auto 모드
- navigating: 정밀 주행 중
- tying: 결속 작업 중
- emergency_stop: 비상정지

전이:
- idle → manual (S10 활성화)
- idle → auto (S20 활성화)
- manual ↔ auto (스위치 전환)
- auto → navigating (주행 명령 수신)
- navigating → tying (결속 명령 수신)
- * → emergency_stop (비상정지)
- emergency_stop → idle (비상정지 해제)
"""

import rclpy
from rclpy.node import Node
from rebar_base_interfaces.msg import RemoteControl
from std_msgs.msg import String, Bool
from statemachine import StateMachine, State


class RebarStateMachine(StateMachine):
    """Rebar 시스템 State Machine"""

    # 상태 정의
    idle = State(initial=True)
    manual = State()
    auto = State()
    navigating = State()
    tying = State()
    emergency_stop = State()

    # 상태 전이 정의
    to_manual = idle.to(manual) | auto.to(manual)
    to_auto = idle.to(auto) | manual.to(auto)
    to_idle = manual.to(idle) | auto.to(idle)

    start_navigation = auto.to(navigating)
    finish_navigation = navigating.to(auto)

    start_tying = auto.to(tying) | navigating.to(tying)
    finish_tying = tying.to(auto)

    trigger_estop = (
        idle.to(emergency_stop) |
        manual.to(emergency_stop) |
        auto.to(emergency_stop) |
        navigating.to(emergency_stop) |
        tying.to(emergency_stop)
    )
    release_estop = emergency_stop.to(idle)

    def __init__(self, navigator_node):
        self.navigator = navigator_node
        super().__init__()

    def on_enter_idle(self):
        """Idle 상태 진입"""
        self.navigator.get_logger().info("🟢 [STATE] Idle")

    def on_enter_manual(self):
        """Manual 상태 진입"""
        self.navigator.get_logger().info("🔵 [STATE] Manual (S10)")

    def on_enter_auto(self):
        """Auto 상태 진입"""
        self.navigator.get_logger().info("🟢 [STATE] Auto (S20)")

    def on_enter_navigating(self):
        """Navigating 상태 진입"""
        self.navigator.get_logger().info("🚀 [STATE] Navigating")

    def on_enter_tying(self):
        """Tying 상태 진입"""
        self.navigator.get_logger().info("🔨 [STATE] Tying")

    def on_enter_emergency_stop(self):
        """Emergency Stop 상태 진입"""
        self.navigator.get_logger().warn("🚨 [STATE] Emergency Stop")

    def on_exit_emergency_stop(self):
        """Emergency Stop 상태 퇴출"""
        self.navigator.get_logger().info("✅ [STATE] Emergency Stop Released")


class NavigatorBase(Node):
    """State Machine 기반 상태 관리 노드"""

    def __init__(self):
        super().__init__('navigator_base')

        # State Machine 초기화
        self.sm = RebarStateMachine(self)

        # ROS2 Subscribers
        self.remote_control_sub = self.create_subscription(
            RemoteControl,
            '/remote_control',
            self.remote_control_callback,
            10
        )

        self.authority_status_sub = self.create_subscription(
            String,
            '/authority_status',
            self.authority_status_callback,
            10
        )

        self.emergency_stop_sub = self.create_subscription(
            Bool,
            '/emergency_stop',
            self.emergency_stop_callback,
            10
        )

        # 상위 제어 계층으로부터의 명령
        self.motion_cmd_sub = self.create_subscription(
            String,
            '/rebar_motion_cmd',
            self.motion_cmd_callback,
            10
        )

        # ROS2 Publishers
        self.control_mode_pub = self.create_publisher(
            String,
            '/control_mode',
            10
        )

        self.state_status_pub = self.create_publisher(
            String,
            '/state_machine_status',
            10
        )

        # 상태 발행 타이머 (5Hz)
        self.status_timer = self.create_timer(0.2, self.publish_status)

        # 상태 관리 변수
        self.current_authority = 'idle'

        self.get_logger().info("Navigator Base 노드 초기화 완료")
        self.get_logger().info(f"초기 상태: {self.sm.current_state.id}")

    def remote_control_callback(self, msg):
        """리모콘 신호 처리"""
        # 비상정지는 emergency_stop_callback에서 처리
        pass

    def authority_status_callback(self, msg):
        """Authority Controller로부터 권한 상태 수신"""
        self.current_authority = msg.data

        # 권한 상태에 따라 State Machine 전이
        try:
            if self.current_authority == 'manual':
                if self.sm.current_state == self.sm.idle or self.sm.current_state == self.sm.auto:
                    self.sm.to_manual()
            elif self.current_authority == 'auto':
                if self.sm.current_state == self.sm.idle or self.sm.current_state == self.sm.manual:
                    self.sm.to_auto()
            elif self.current_authority == 'idle':
                if self.sm.current_state == self.sm.manual or self.sm.current_state == self.sm.auto:
                    self.sm.to_idle()
        except Exception as e:
            self.get_logger().warn(f"상태 전이 실패: {e}")

    def emergency_stop_callback(self, msg):
        """비상정지 신호 처리"""
        if msg.data:
            # 비상정지 활성화
            try:
                self.sm.trigger_estop()
            except Exception as e:
                self.get_logger().warn(f"비상정지 전이 실패: {e}")
        else:
            # 비상정지 해제
            try:
                if self.sm.current_state == self.sm.emergency_stop:
                    self.sm.release_estop()
            except Exception as e:
                self.get_logger().warn(f"비상정지 해제 실패: {e}")

    def motion_cmd_callback(self, msg):
        """상위 제어 계층으로부터의 모션 명령 처리"""
        command = msg.data

        # Auto 모드에서만 명령 수락
        if self.current_authority != 'auto':
            self.get_logger().warn(f"모션 명령 거부: 현재 {self.current_authority} 모드")
            return

        try:
            if command.startswith('MOVE_'):
                # 주행 명령
                if self.sm.current_state == self.sm.auto:
                    self.sm.start_navigation()
                elif self.sm.current_state == self.sm.navigating:
                    self.get_logger().info("이미 주행 중")
            elif command == 'START_TYING':
                # 결속 명령
                if self.sm.current_state == self.sm.auto or self.sm.current_state == self.sm.navigating:
                    self.sm.start_tying()
            elif command == 'NAVIGATION_COMPLETE':
                # 주행 완료
                if self.sm.current_state == self.sm.navigating:
                    self.sm.finish_navigation()
            elif command == 'TYING_COMPLETE':
                # 결속 완료
                if self.sm.current_state == self.sm.tying:
                    self.sm.finish_tying()
        except Exception as e:
            self.get_logger().error(f"모션 명령 처리 오류: {e}")

    def publish_status(self):
        """현재 상태 발행"""
        # Control mode 발행
        mode_msg = String()
        mode_msg.data = self.sm.current_state.id
        self.control_mode_pub.publish(mode_msg)

        # State machine status 발행
        status_msg = String()
        status_msg.data = f"{self.sm.current_state.id}|{self.current_authority}"
        self.state_status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = NavigatorBase()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Authority Controller Node
제어권한 관리 (Manual/Auto 전환)

S10 (S19): Manual 모드 - 리모콘 제어 허용 (횡이동/yaw/시퀀스 등)
S20: Auto 모드 - UI/Navigator 제어 + 리모콘 S17/S18로 UI 미션 시작/중단
  · S17 → START_MISSION (자율작업 시작)
  · S18 → CANCEL (중단)
"""

import rclpy
from rclpy.node import Node
from rebar_base_interfaces.msg import RemoteControl
from std_msgs.msg import String, Bool
import json


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

        # Auto 모드에서 리모콘 S17/S18 → UI와 동일한 미션 명령 발행
        self.mission_cmd_pub = self.create_publisher(
            String,
            '/mission/command',
            10
        )
        # 호밍 명령 발행 (Auto 모드 S21 → homing_controller /homing_cmd)
        self.homing_cmd_pub = self.create_publisher(
            String,
            '/homing_cmd',
            10
        )
        self.prev_buttons = []  # rising-edge 감지용

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

        # 비상정지 중에는 모드 전환/미션 불가 (prev_buttons만 갱신)
        if self.emergency_stopped:
            self.prev_buttons = list(msg.buttons)
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

        # Auto 모드: 리모콘 S17/S18 → UI 미션 명령 (rising edge)
        self._handle_auto_mission_buttons(msg)
        self.prev_buttons = list(msg.buttons)

    def _handle_auto_mission_buttons(self, msg):
        """Auto 모드에서 리모콘 버튼 → 미션/호밍 명령 발행 (rising edge)

        buttons = [s13, s14, s17, s18, s21, s22, s23, s24]  (can_parser 기준)
          · S17 (buttons[2]) → START_MISSION repeat=ON (반복 자율결속, 시연용)
          · S18 (buttons[3]) → CANCEL (반복 중지 + 정지)
          · S21 (buttons[4]) → 호밍 시작 (/homing_cmd "START")
        manual 모드에선 무시 (횡이동/Z 등 기존 기능은 joint_controller가 처리).
        """
        if self.current_mode != 'auto':
            return
        if len(msg.buttons) < 4:
            return

        def b(i):
            return msg.buttons[i] if len(msg.buttons) > i else 0

        def pb(i):
            return self.prev_buttons[i] if len(self.prev_buttons) > i else 0

        s17, s18, s21 = b(2), b(3), b(4)
        s22 = b(5)             # S22=경로/모드 선택 (ON유지=1회 실결속, OFF=무한전시)
        s23, s24 = b(6), b(7)  # S23=일시중지, S24=재개

        if pb(2) == 0 and s17 == 1:
            if s22 == 1:
                # S22 ON + S17: 별도 경로(path2) 1회만 결속 (실결속, 끝나면 정지 → 결속점 끊을 시간)
                self._publish_mission('START_MISSION', repeat='OFF',
                                      path='default_path_one_cam2.json')
                self.get_logger().info(
                    "🟢 [리모콘 AUTO] S22+S17 → 1회 결속 (path2=default_path_one_cam2, repeat OFF)")
            else:
                # 기본 S17: default 경로(path1) 무한반복 (상시 전시/시연)
                self._publish_mission('START_MISSION', repeat='ON',
                                      path='default_path_one_cam.json')
                self.get_logger().info(
                    "🟢 [리모콘 AUTO] S17 → 무한반복 전시 (path1=default_path_one_cam, repeat ON)")
        elif pb(3) == 0 and s18 == 1:
            self._publish_mission('CANCEL')
            self.get_logger().info("🛑 [리모콘 AUTO] S18 → CANCEL (반복 중지)")
        elif pb(4) == 0 and s21 == 1:
            self._publish_homing()
            self.get_logger().info("🏠 [리모콘 AUTO] S21 → 호밍 시작")
        elif pb(6) == 0 and s23 == 1:
            self._publish_mission('PAUSE')
            self.get_logger().info("⏸️ [리모콘 AUTO] S23 → PAUSE (일시중지)")
        elif pb(7) == 0 and s24 == 1:
            self._publish_mission('RESUME')
            self.get_logger().info("▶️ [리모콘 AUTO] S24 → RESUME (재개)")

    def _publish_mission(self, command, repeat=None, path=None):
        """UI와 동일한 /mission/command(JSON) 발행. repeat='ON'/'OFF', path=경로파일명 선택."""
        payload = {"command": command}
        if repeat is not None:
            payload["repeat"] = repeat
        if path is not None:
            payload["path"] = path   # navigator가 그 경로 로드 (기존 waypoints 덮어씀)
        out = String()
        out.data = json.dumps(payload)
        self.mission_cmd_pub.publish(out)

    def _publish_homing(self):
        """homing_controller에 호밍 시작 명령 발행 (/homing_cmd 'START')."""
        out = String()
        out.data = 'START'
        self.homing_cmd_pub.publish(out)

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

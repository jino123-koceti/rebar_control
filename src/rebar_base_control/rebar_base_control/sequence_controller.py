#!/usr/bin/env python3
"""
Sequence Controller Node
S21/S22 작업 시퀀스 관리

S21 시퀀스 (하강→닫힘):
1. 그리퍼 홈 위치
2. 2초 대기
3. Z축 -1100° 하강
4. 6초 대기
5. 그리퍼 닫기

S22 시퀀스 (열림→상승):
1. 트리거 동작
2. 3초 대기
3. 그리퍼 열기
4. 2초 대기
5. Z축 +3600° 상승 (리밋까지)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from rebar_base_interfaces.msg import RemoteControl, JointControl, GripperControl
from std_msgs.msg import String, Bool, Float32
import time
from enum import Enum, auto


class SequenceState(Enum):
    """시퀀스 상태"""
    IDLE = auto()
    # S21 시퀀스
    S21_GRIPPER_HOME = auto()
    S21_WAIT_HOME = auto()
    S21_Z_DOWN = auto()
    S21_WAIT_Z = auto()
    S21_GRIPPER_CLOSE = auto()
    S21_COMPLETE = auto()
    # S22 시퀀스
    S22_TRIGGER = auto()
    S22_WAIT_TRIGGER = auto()
    S22_TRIGGER_RETURN = auto()
    S22_WAIT_TRIGGER_RETURN = auto()
    S22_GRIPPER_OPEN = auto()
    S22_WAIT_OPEN = auto()
    S22_Z_UP = auto()
    S22_WAIT_Z_UP = auto()  # Z축 상승 대기 (리밋 감지까지)
    S22_COMPLETE = auto()


class SequenceController(Node):
    """S21/S22 작업 시퀀스 관리 노드"""

    def __init__(self):
        super().__init__('sequence_controller')

        # 파라미터 선언
        self.declare_parameter('z_down_deg', 1100.0)      # S21 Z축 하강 각도
        self.declare_parameter('z_up_deg', 3600.0)        # S22 Z축 상승 각도 (리밋까지)
        self.declare_parameter('z_speed', 200.0)          # Z축 속도 (dps)
        self.declare_parameter('gripper_speed', 50)       # 그리퍼 속도 (0-100%)
        self.declare_parameter('gripper_force', 50)       # 그리퍼 힘 (0-100%)
        self.declare_parameter('wait_home', 2.0)          # 그리퍼 홈 대기 시간 (초)
        self.declare_parameter('wait_z_down', 6.0)        # Z축 하강 후 대기 시간 (초)
        self.declare_parameter('wait_trigger', 3.0)       # 트리거 후 대기 시간 (초)
        self.declare_parameter('wait_open', 2.0)          # 그리퍼 열기 후 대기 시간 (초)
        self.declare_parameter('trigger_speed', 1.0)      # 트리거 속도 (-1.0 ~ 1.0)

        # 파라미터 가져오기
        self.z_down_deg = self.get_parameter('z_down_deg').value
        self.z_up_deg = self.get_parameter('z_up_deg').value
        self.z_speed = self.get_parameter('z_speed').value
        self.gripper_speed = self.get_parameter('gripper_speed').value
        self.gripper_force = self.get_parameter('gripper_force').value
        self.wait_home = self.get_parameter('wait_home').value
        self.wait_z_down = self.get_parameter('wait_z_down').value
        self.wait_trigger = self.get_parameter('wait_trigger').value
        self.wait_open = self.get_parameter('wait_open').value
        self.trigger_speed = self.get_parameter('trigger_speed').value

        # 상태 변수
        self.state = SequenceState.IDLE
        self.state_start_time = None
        self.control_mode = 'idle'
        self.prev_buttons = [0] * 8
        self.remote_msg = RemoteControl()

        # 리밋 센서 상태
        self.z_upper_limit = False  # Z축 상한 리밋 (z_max)
        self.z_lower_limit = False  # Z축 하한 리밋 (z_min) - 상승 시 사용

        # ROS2 Subscribers
        self.remote_sub = self.create_subscription(
            RemoteControl,
            '/remote_control',
            self.remote_callback,
            qos_profile_sensor_data
        )

        self.control_mode_sub = self.create_subscription(
            String,
            '/control_mode',
            self.control_mode_callback,
            qos_profile_system_default
        )

        # Z축 리밋 센서 구독
        self.z_max_limit_sub = self.create_subscription(
            Bool,
            '/limit_sensors/z_max',
            self.z_max_limit_callback,
            10
        )
        self.z_min_limit_sub = self.create_subscription(
            Bool,
            '/limit_sensors/z_min',
            self.z_min_limit_callback,
            10
        )

        # ROS2 Publishers
        self.joint_pub = self.create_publisher(
            JointControl,
            '/joint_control',
            qos_profile_system_default
        )

        self.gripper_pub = self.create_publisher(
            GripperControl,
            '/gripper_control',
            10
        )

        self.trigger_pub = self.create_publisher(
            Bool,
            '/trigger_control',
            10
        )

        # Pololu 트리거 모터 퍼블리셔 (Float32: -1.0 ~ 1.0)
        self.pololu_trigger_pub = self.create_publisher(
            Float32,
            '/motor_0/vel',
            10
        )

        self.sequence_status_pub = self.create_publisher(
            String,
            '/sequence_status',
            10
        )

        # 자동 시퀀스 명령 구독 (tying_orchestrator에서 트리거)
        self.sequence_cmd_sub = self.create_subscription(
            String,
            '/sequence_cmd',
            self.sequence_cmd_callback,
            10
        )

        # 상태 머신 타이머 (20Hz)
        self.timer = self.create_timer(0.05, self.state_machine_loop)

        self.get_logger().info("Sequence Controller 노드 초기화 완료")
        self.get_logger().info(f"  - S21: Z 하강 {self.z_down_deg}°")
        self.get_logger().info(f"  - S22: Z 상승 {self.z_up_deg}° (리밋까지)")
        self.get_logger().info(f"  - Z 속도: {self.z_speed} dps")

    def sequence_cmd_callback(self, msg: String):
        """자동 모드 시퀀스 명령 수신 (tying_orchestrator에서 호출)"""
        if self.state != SequenceState.IDLE:
            self.get_logger().warn(
                f"시퀀스 명령 거부: 현재 {self.state.name} 상태"
            )
            return

        command = msg.data

        if command == "AUTO_S21":
            self.get_logger().info("=" * 50)
            self.get_logger().info("[AUTO] S21 시퀀스 시작: 하강 -> 닫힘")
            self.get_logger().info("=" * 50)
            self._start_s21_sequence()
        elif command == "AUTO_S22":
            self.get_logger().info("=" * 50)
            self.get_logger().info("[AUTO] S22 시퀀스 시작: 열림 -> 상승")
            self.get_logger().info("=" * 50)
            self._start_s22_sequence()
        else:
            self.get_logger().warn(f"알 수 없는 시퀀스 명령: {command}")

    def remote_callback(self, msg: RemoteControl):
        """리모콘 입력 처리"""
        self.remote_msg = msg

    def control_mode_callback(self, msg: String):
        """제어 모드 업데이트"""
        self.control_mode = msg.data

    def z_max_limit_callback(self, msg: Bool):
        """Z축 상한 리밋 센서 상태 (z_max)"""
        self.z_upper_limit = msg.data

    def z_min_limit_callback(self, msg: Bool):
        """Z축 하한 리밋 센서 상태 (z_min) - 상승 시 트리거"""
        prev_state = self.z_lower_limit
        self.z_lower_limit = msg.data
        # S22 상승 대기 중 z_min 리밋 도달 시 정지
        if msg.data and not prev_state and self.state == SequenceState.S22_WAIT_Z_UP:
            self._send_z_stop()
            self._reset_z_position()  # Z축 위치 기준점 리셋
            self.get_logger().info("🛑 Z축 상승 리밋 도달 - 정지 및 위치 리셋")
            self._transition_to(SequenceState.S22_COMPLETE)

    def state_machine_loop(self):
        """상태 머신 메인 루프"""
        # Manual 모드에서만 S21/S22 처리
        if self.control_mode == 'manual' and self.state == SequenceState.IDLE:
            self._check_button_trigger()

        # 상태 머신 실행
        if self.state != SequenceState.IDLE:
            self._execute_state_machine()

        # 이전 버튼 상태 저장
        if self.remote_msg.buttons:
            self.prev_buttons = list(self.remote_msg.buttons)

    def _check_button_trigger(self):
        """S21/S22 버튼 트리거 확인"""
        if not self.remote_msg.buttons or len(self.remote_msg.buttons) < 6:
            return

        # buttons 배열: [s13, s14, s17, s18, s21, s22, s23, s24]
        s21 = self.remote_msg.buttons[4]
        s22 = self.remote_msg.buttons[5]
        prev_s21 = self.prev_buttons[4] if len(self.prev_buttons) > 4 else 0
        prev_s22 = self.prev_buttons[5] if len(self.prev_buttons) > 5 else 0

        # 엣지 감지 (0→1)
        if prev_s21 == 0 and s21 == 1:
            self.get_logger().info("=" * 50)
            self.get_logger().info("🔽 S21 시퀀스 시작: 하강→닫힘")
            self.get_logger().info("=" * 50)
            self._start_s21_sequence()
        elif prev_s22 == 0 and s22 == 1:
            self.get_logger().info("=" * 50)
            self.get_logger().info("🔼 S22 시퀀스 시작: 열림→상승")
            self.get_logger().info("=" * 50)
            self._start_s22_sequence()

    def _start_s21_sequence(self):
        """S21 시퀀스 시작"""
        self.state = SequenceState.S21_GRIPPER_HOME
        self.state_start_time = time.monotonic()
        self._publish_status("S21_STARTED")

    def _start_s22_sequence(self):
        """S22 시퀀스 시작"""
        self.state = SequenceState.S22_TRIGGER
        self.state_start_time = time.monotonic()
        self._publish_status("S22_STARTED")

    def _execute_state_machine(self):
        """상태 머신 실행"""
        elapsed = time.monotonic() - self.state_start_time if self.state_start_time else 0

        # S21 시퀀스
        if self.state == SequenceState.S21_GRIPPER_HOME:
            self._send_gripper_command(GripperControl.COMMAND_OPEN)  # 홈 = 열기
            self.get_logger().info("  [1/5] 그리퍼 홈 위치")
            self._transition_to(SequenceState.S21_WAIT_HOME)

        elif self.state == SequenceState.S21_WAIT_HOME:
            if elapsed >= self.wait_home:
                self.get_logger().info(f"  [2/5] 대기 완료 ({self.wait_home}초)")
                self._transition_to(SequenceState.S21_Z_DOWN)

        elif self.state == SequenceState.S21_Z_DOWN:
            # Z축: + 방향이 하강 (모터 실제 방향)
            self._send_z_command(self.z_down_deg)
            self.get_logger().info(f"  [3/5] Z축 하강 +{self.z_down_deg}°")
            self._transition_to(SequenceState.S21_WAIT_Z)

        elif self.state == SequenceState.S21_WAIT_Z:
            if elapsed >= self.wait_z_down:
                self.get_logger().info(f"  [4/5] 대기 완료 ({self.wait_z_down}초)")
                self._transition_to(SequenceState.S21_GRIPPER_CLOSE)

        elif self.state == SequenceState.S21_GRIPPER_CLOSE:
            self._send_gripper_command(GripperControl.COMMAND_CLOSE)
            self.get_logger().info("  [5/5] 그리퍼 닫기")
            self._transition_to(SequenceState.S21_COMPLETE)

        elif self.state == SequenceState.S21_COMPLETE:
            self.get_logger().info("=" * 50)
            self.get_logger().info("✅ S21 시퀀스 완료!")
            self.get_logger().info("=" * 50)
            self._publish_status("S21_COMPLETE")
            self._transition_to(SequenceState.IDLE)

        # S22 시퀀스
        elif self.state == SequenceState.S22_TRIGGER:
            self._send_trigger_command(True)
            self.get_logger().info("  [1/5] 트리거 동작")
            self._transition_to(SequenceState.S22_WAIT_TRIGGER)

        elif self.state == SequenceState.S22_WAIT_TRIGGER:
            if elapsed >= 1.0:  # 1초 후 역방향 원복
                self._send_trigger_command(False)  # 트리거 정지
                self.get_logger().info("  [2/7] 트리거 1초 대기 완료")
                self._transition_to(SequenceState.S22_TRIGGER_RETURN)

        elif self.state == SequenceState.S22_TRIGGER_RETURN:
            self._send_trigger_return_command()  # 역방향 원복
            self.get_logger().info("  [3/7] 트리거 원복 시작")
            self._transition_to(SequenceState.S22_WAIT_TRIGGER_RETURN)

        elif self.state == SequenceState.S22_WAIT_TRIGGER_RETURN:
            if elapsed >= 1.0:  # 1초 후 정지
                self._send_trigger_command(False)  # 트리거 정지
                self.get_logger().info("  [4/7] 트리거 원복 완료")
                self._transition_to(SequenceState.S22_GRIPPER_OPEN)

        elif self.state == SequenceState.S22_GRIPPER_OPEN:
            self._send_gripper_command(GripperControl.COMMAND_OPEN)
            self.get_logger().info("  [5/7] 그리퍼 열기")
            self._transition_to(SequenceState.S22_WAIT_OPEN)

        elif self.state == SequenceState.S22_WAIT_OPEN:
            if elapsed >= self.wait_open:
                self.get_logger().info(f"  [6/7] 대기 완료 ({self.wait_open}초)")
                self._transition_to(SequenceState.S22_Z_UP)

        elif self.state == SequenceState.S22_Z_UP:
            # Z축: - 방향이 상승 (모터 실제 방향)
            self._send_z_command(-self.z_up_deg)
            self.get_logger().info(f"  [5/5] Z축 상승 -{self.z_up_deg}° (리밋까지)")
            self._transition_to(SequenceState.S22_WAIT_Z_UP)

        elif self.state == SequenceState.S22_WAIT_Z_UP:
            # 리밋 감지 시 z_min_limit_callback에서 S22_COMPLETE로 전환
            # 타임아웃 (20초) 시 강제 완료
            if elapsed >= 20.0:
                self.get_logger().warn("⚠️ Z축 상승 타임아웃 - 강제 완료")
                self._send_z_stop()
                self._reset_z_position()  # Z축 위치 기준점 리셋
                self._transition_to(SequenceState.S22_COMPLETE)

        elif self.state == SequenceState.S22_COMPLETE:
            self.get_logger().info("=" * 50)
            self.get_logger().info("✅ S22 시퀀스 완료!")
            self.get_logger().info("=" * 50)
            self._publish_status("S22_COMPLETE")
            self._transition_to(SequenceState.IDLE)

    def _transition_to(self, new_state: SequenceState):
        """상태 전환"""
        self.state = new_state
        self.state_start_time = time.monotonic()

    def _send_z_command(self, delta_deg: float):
        """Z축 위치 명령 전송 (0x146)"""
        msg = JointControl()
        msg.joint_id = 0x146
        msg.position = delta_deg
        msg.velocity = self.z_speed
        msg.control_mode = JointControl.MODE_RELATIVE
        self.joint_pub.publish(msg)

    def _send_z_stop(self):
        """Z축 정지 명령 전송 (0x146) - 속도 0으로 설정"""
        msg = JointControl()
        msg.joint_id = 0x146
        msg.position = 0.0
        msg.velocity = 0.0
        msg.control_mode = JointControl.MODE_SPEED  # 속도 0 = 정지
        self.joint_pub.publish(msg)

    def _reset_z_position(self):
        """Z축 위치 기준점 리셋 (리밋 도달 후 호출)

        can_sender의 joint_positions[0x146]을 0으로 리셋하기 위해
        절대 위치 0으로 명령 전송 (실제 이동 없이 기준점만 설정)
        """
        msg = JointControl()
        msg.joint_id = 0x146
        msg.position = 0.0
        msg.velocity = 0.0
        msg.control_mode = JointControl.MODE_ABSOLUTE  # 절대 위치 모드
        self.joint_pub.publish(msg)
        self.get_logger().info("🔄 Z축 위치 기준점 리셋 (0°)")

    def _send_gripper_command(self, command: int):
        """그리퍼 명령 전송"""
        msg = GripperControl()
        msg.command = command
        msg.speed = self.gripper_speed
        msg.force = self.gripper_force
        self.gripper_pub.publish(msg)

    def _send_trigger_command(self, activate: bool):
        """트리거 명령 전송 (Pololu 모터 제어)"""
        # Bool 토픽 (하위 호환성)
        bool_msg = Bool()
        bool_msg.data = activate
        self.trigger_pub.publish(bool_msg)

        # Pololu 모터 속도 명령 (-1.0 ~ 1.0)
        vel_msg = Float32()
        vel_msg.data = self.trigger_speed if activate else 0.0
        self.pololu_trigger_pub.publish(vel_msg)
        self.get_logger().info(f"🔧 트리거: {'ON' if activate else 'OFF'} (vel={vel_msg.data})")

    def _send_trigger_return_command(self):
        """트리거 역방향 원복 명령 전송"""
        vel_msg = Float32()
        vel_msg.data = -self.trigger_speed  # 역방향
        self.pololu_trigger_pub.publish(vel_msg)
        self.get_logger().info(f"🔧 트리거 원복: vel={vel_msg.data}")

    def _publish_status(self, status: str):
        """시퀀스 상태 발행"""
        msg = String()
        msg.data = status
        self.sequence_status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SequenceController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

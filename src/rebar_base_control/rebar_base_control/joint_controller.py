#!/usr/bin/env python3
"""
Joint Controller Node
리모콘 스위치 입력을 관절 모터 위치 제어 명령으로 변환

제어 모터:
- 0x143: 횡이동 (Lateral) - S17/S18, 360도 단위
- 0x147: Yaw 회전 - S23/S24, 5도 단위

제어 방식:
- 출력축 각도(0x92) 기반 절대 위치 제어
- S17/S18: 12시 위치 복귀 → 1회전 → 12시 복귀
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from rebar_base_interfaces.msg import RemoteControl, JointControl, MotorFeedback
from std_msgs.msg import String
import time


class JointController(Node):

    def __init__(self):
        super().__init__('joint_controller')
        self.nodeName = self.get_name()
        self.get_logger().info(f'{self.nodeName} started')

        # Parameters
        self.declare_parameter('lateral_step_deg', 360.0)
        self.declare_parameter('lateral_max_speed', 200.0)
        self.declare_parameter('yaw_step_deg', 5.0)
        self.declare_parameter('yaw_max_speed', 134.0)
        self.declare_parameter('command_interval', 6.0)
        # 0x90 단회전(16bit) → 0~65535 counts
        self.declare_parameter('lateral_encoder_cpr', 65536.0)
        # 허용 오차: 각도(deg) 기준
        self.declare_parameter('position_tolerance', 5.0)

        self.lateral_step = self.get_parameter('lateral_step_deg').value
        self.lateral_speed = self.get_parameter('lateral_max_speed').value
        self.yaw_step = self.get_parameter('yaw_step_deg').value
        self.yaw_speed = self.get_parameter('yaw_max_speed').value
        self.command_interval = self.get_parameter('command_interval').value
        self.lateral_cpr = self.get_parameter('lateral_encoder_cpr').value
        self.position_tolerance = self.get_parameter('position_tolerance').value

        # 0x90 명령 요청 publisher
        self.encoder_request_pub = self.create_publisher(
            JointControl, '/encoder_request', 10)

        # State
        self.control_mode = 'idle'
        self.remote_msg = RemoteControl()
        self.prev_buttons = [0] * 8
        self.last_command_time = None

        # Home position (12시 위치)
        self.home_encoder = None  # 초기화 필요
        self.current_encoder = None  # 최근 엔코더 값
        self.current_encoder_time = 0.0  # monotonic timestamp of last encoder update
        self.home_angle_deg = None  # 0x92 출력축 각도 기반 홈
        self.current_angle_deg = None  # 최근 출력축 각도 (deg)
        self.current_angle_time = 0.0

        # Motion state
        self.motion_state = 'idle'  # idle, moving_to_home, rotating, returning_home

        # Subscribers
        self.remote_subscriber = self.create_subscription(
            RemoteControl, '/remote_control', self.recv_remote, qos_profile_sensor_data)
        self.status_subscriber = self.create_subscription(
            String, '/control_mode', self.recv_status, qos_profile_system_default)
        self.motor_feedback_subscriber = self.create_subscription(
            MotorFeedback, '/motor_feedback', self.recv_motor_feedback, 10)

        # Publisher
        self.joint_publisher = self.create_publisher(
            JointControl, '/joint_control', qos_profile_system_default)

        # Timer (20Hz - tire_roller style)
        self.control_frequency = 20
        self.timer = self.create_timer(1/self.control_frequency, self.process_joint_control)

        self.get_logger().info(f'Lateral: {self.lateral_step}° @ {self.lateral_speed} dps (0x143)')
        self.get_logger().info(f'Yaw: {self.yaw_step}° @ {self.yaw_speed} dps (0x147)')
        self.get_logger().info(f'Encoder CPR: {self.lateral_cpr}, Tolerance: {self.position_tolerance} deg')

        # Home position 초기화 (5초 후)
        self.create_timer(5.0, self._calibrate_home_position_once)

    def _calibrate_home_position_once(self):
        """초기 Home position 설정 (1회만 실행)"""
        if self.home_angle_deg is not None:
            return  # 이미 설정됨

        # 0x92 출력축 각도 캐시가 있으면 바로 사용
        if self.current_angle_deg is not None:
            self.home_angle_deg = self.current_angle_deg
            self.get_logger().info(f"🏠 Home angle set: {self.home_angle_deg:.2f}° (최근 0x92 피드백)")
            return

        # (백업) 0x90 엔코더 캐시가 있으면 사용
        if self.current_encoder is not None:
            self.home_encoder = self.current_encoder & 0xFFFF
            self.get_logger().info(f"🏠 Home position set (encoder counts): {self.home_encoder} counts")
            return

        # 없으면 0x92 요청 후 다음 타이머에 재시도
        self._request_output_angle(0x143)
        self.get_logger().warn("⚠️ Home angle calibration pending (0x92), retry in 5s")
        self.create_timer(5.0, self._calibrate_home_position_once)

    def recv_status(self, msg: String):
        self.control_mode = msg.data

    def recv_remote(self, msg: RemoteControl):
        self.remote_msg = msg

    def recv_motor_feedback(self, msg: MotorFeedback):
        """모터 피드백 수신 (엔코더 값 업데이트)"""
        # 관측용 로깅 (0x143 필터)
        if msg.motor_id == 0x43:
            self.get_logger().info(
                f"[FEEDBACK] id:0x{msg.motor_id:02X} status:0x{int(msg.status):02X} pos:{msg.current_position}",
                throttle_duration_sec=0.2
            )

        if msg.motor_id == 0x43:  # 0x143
            # 0x92: Multi-turn angle (출력축 기준, 0.01deg/LSB)
            if msg.status == 0x92:
                angle_deg = float(msg.current_position)
                self.current_angle_deg = angle_deg
                self.current_angle_time = time.monotonic()

                if self.home_angle_deg is None:
                    self.home_angle_deg = angle_deg
                    self.get_logger().info(
                        f"🏠 Home angle set via 0x92: {self.home_angle_deg:.2f}°",
                        throttle_duration_sec=0.2
                    )

                self.get_logger().info(
                    f"[0x92] Updated output angle: {angle_deg:.2f}°",
                    throttle_duration_sec=0.2
                )
                return

            # 0x90 응답만 사용 (싱글턴 엔코더)
            if msg.status == 0x90:
                encoder_value = int(msg.current_position) & 0xFFFF
                self.current_encoder = encoder_value
                self.current_encoder_time = time.monotonic()
                # 홈 미설정 시 최초 값으로 홈 설정
                if self.home_encoder is None:
                    self.home_encoder = encoder_value
                    self.get_logger().info(
                        f"🏠 Home position set via feedback: {self.home_encoder} counts",
                        throttle_duration_sec=0.2
                    )
                self.get_logger().info(
                    f"[0x90] Updated encoder: {encoder_value}",
                    throttle_duration_sec=0.2
                )

    def process_joint_control(self):
        """주기적으로 리모콘 입력 처리 (tire_roller pattern)"""
        if self.control_mode != 'manual':
            return

        # 명령 간격 제한
        if self.last_command_time:
            elapsed = (self.get_clock().now() - self.last_command_time).nanoseconds / 1e9
            if elapsed < self.command_interval:
                return

        msg = self.remote_msg
        if not msg.buttons or len(msg.buttons) < 8:
            return

        # S17/S18: 횡이동 (0x143) - 12시 복귀 로직
        self._handle_lateral_with_home(msg)

        # S23/S24: Yaw 회전 (0x147)
        self._handle_yaw(msg)

        # 이전 버튼 상태 저장
        self.prev_buttons = list(msg.buttons)

    def _handle_lateral_with_home(self, msg):
        """S17/S18 → 12시 복귀 + 1회전 + 12시 복귀"""
        s17 = msg.buttons[2]
        s18 = msg.buttons[3]
        prev_s17 = self.prev_buttons[2] if len(self.prev_buttons) > 2 else 0
        prev_s18 = self.prev_buttons[3] if len(self.prev_buttons) > 3 else 0

        # 엣지 감지 (0→1)
        if prev_s17 == 0 and s17 == 1:
            self._execute_home_rotation(direction='+')
        elif prev_s18 == 0 and s18 == 1:
            self._execute_home_rotation(direction='-')

    def _execute_home_rotation(self, direction):
        """12시 복귀 → 1회전 → 12시 복귀 시퀀스"""
        if self.home_angle_deg is None:
            self.get_logger().warn("⚠️ Home angle not calibrated! Set home via 0x92 first.")
            return

        # 1. 현재 출력축 각도 읽기
        current = self._read_output_angle_deg(0x143)
        if current is None:
            self.get_logger().error("❌ Failed to read current output angle (0x92)")
            return

        self.current_angle_deg = current
        self.get_logger().info(f"📍 Current angle: {current:.2f}°, Home: {self.home_angle_deg:.2f}°")

        # 2. 12시 위치 확인
        delta_home = self._calculate_delta_deg(current, self.home_angle_deg)
        if abs(delta_home) > self.position_tolerance:
            # 12시가 아니면 먼저 12시로 이동 (절대 위치 명령)
            self.get_logger().info(f"🔄 Not at home! Moving to home first (delta: {delta_home:.1f}°)")
            self._send_joint_command_abs(0x143, self.home_angle_deg, self.lateral_speed, 'Move to Home')
            time.sleep(self.command_interval)  # 이동 완료 대기

            # 이동 후 위치 확인
            current = self._read_output_angle_deg(0x143)
            if current is None:
                return
            self.current_angle_deg = current
            self.get_logger().info(f"✅ Arrived near home: {current:.2f}°")

        # 3. 1회전 실행 (출력축 기준)
        rotation_angle = self.lateral_step if direction == '+' else -self.lateral_step
        self.get_logger().info(f"🔄 Rotating {rotation_angle:.0f}° from home (output shaft)...")
        target_angle = self.home_angle_deg + rotation_angle
        self._send_joint_command_abs(0x143, target_angle, self.lateral_speed, f'Rotate {direction}360°')
        time.sleep(self.command_interval)  # 회전 완료 대기

        # 4. 12시로 복귀 확인
        final = self._read_output_angle_deg(0x143)
        if final is not None:
            self.current_angle_deg = final
            delta_final = self._calculate_delta_deg(final, self.home_angle_deg)
            if abs(delta_final) <= self.position_tolerance:
                self.get_logger().info(f"✅ Successfully returned to home! angle: {final:.2f}°")
            else:
                self.get_logger().warn(f"⚠️ Not at home! Final: {final:.2f}°, Delta: {delta_final:.1f}°")
                # 보정 이동 (절대 위치)
                self.get_logger().info("🔧 Correcting position...")
                self._send_joint_command_abs(0x143, self.home_angle_deg, self.lateral_speed, 'Correction')

    def _calculate_delta_deg(self, current_deg, target_deg):
        """현재 각도 → 목표 각도까지의 최단 경로(±180°) 계산"""
        delta = (target_deg - current_deg + 180.0) % 360.0 - 180.0
        return delta

    def _read_single_turn_encoder(self, motor_id):
        """
        motor_feedback 토픽에서 최근 엔코더 값 읽기

        Returns:
        - encoder position (uint16) or None
        """
        # 0x90 명령 요청 (can_sender가 처리)
        self.get_logger().info(f"[REQ 0x90] motor:0x{motor_id:03X} 단회전 엔코더 요청")
        # 캐시가 있으면 즉시 반환
        if self.current_encoder is not None:
            return self.current_encoder

        # 없으면 0x90 요청만 보내고 None 반환 (콜백 수신 대기)
        self._request_single_turn_encoder(motor_id)
        return None

    def _read_output_angle_deg(self, motor_id):
        """motor_feedback 토픽에서 최근 출력축 각도(0x92) 읽기"""
        self.get_logger().info(f"[REQ 0x92] motor:0x{motor_id:03X} 출력축 각도 요청")

        if self.current_angle_deg is not None:
            return self.current_angle_deg

        self._request_output_angle(motor_id)
        return None

    def _request_single_turn_encoder(self, motor_id):
        """0x90 엔코더 읽기 요청 발행 (비동기)"""
        self.get_logger().info(f"[REQ 0x90] motor:0x{motor_id:03X} 단회전 엔코더 요청")
        request_msg = JointControl()
        request_msg.joint_id = motor_id
        request_msg.position = 0.0
        request_msg.velocity = 0.0
        request_msg.control_mode = 0x90
        self.encoder_request_pub.publish(request_msg)

    def _request_output_angle(self, motor_id):
        """0x92 출력축 각도 읽기 요청 발행 (멀티턴, 비동기)"""
        self.get_logger().info(f"[REQ 0x92] motor:0x{motor_id:03X} 출력축 각도 요청")
        request_msg = JointControl()
        request_msg.joint_id = motor_id
        request_msg.position = 0.0
        request_msg.velocity = 0.0
        request_msg.control_mode = 0x92
        self.encoder_request_pub.publish(request_msg)

    def _handle_yaw(self, msg):
        """S23/S24 → Yaw ±step"""
        if not msg.buttons or len(msg.buttons) < 8:
            return

        s23 = msg.buttons[6]
        s24 = msg.buttons[7]
        prev_s23 = self.prev_buttons[6] if len(self.prev_buttons) > 6 else 0
        prev_s24 = self.prev_buttons[7] if len(self.prev_buttons) > 7 else 0

        # 엣지 감지 (0→1)
        if prev_s23 == 0 and s23 == 1:
            self._send_joint_command_rel(0x147, self.yaw_step, self.yaw_speed, 'Yaw +step')
        elif prev_s24 == 0 and s24 == 1:
            self._send_joint_command_rel(0x147, -self.yaw_step, self.yaw_speed, 'Yaw -step')
        """S23/S24 → Yaw ±5도"""
        s23 = msg.buttons[6]
        s24 = msg.buttons[7]
        prev_s23 = self.prev_buttons[6] if len(self.prev_buttons) > 6 else 0
        prev_s24 = self.prev_buttons[7] if len(self.prev_buttons) > 7 else 0

        # 엣지 감지 (0→1)
        if prev_s23 == 0 and s23 == 1:
            self._send_joint_command_rel(0x147, self.yaw_step, self.yaw_speed, 'Yaw +5°')
        elif prev_s24 == 0 and s24 == 1:
            self._send_joint_command_rel(0x147, -self.yaw_step, self.yaw_speed, 'Yaw -5°')

    def _send_joint_command_abs(self, joint_id, position_abs_deg, velocity, name='Joint ABS'):
        """JointControl 메시지 발행 (절대 위치)"""
        joint_msg = JointControl()
        joint_msg.joint_id = joint_id
        joint_msg.position = position_abs_deg
        joint_msg.velocity = velocity
        joint_msg.control_mode = JointControl.MODE_ABSOLUTE

        self.joint_publisher.publish(joint_msg)
        self.last_command_time = self.get_clock().now()

        self.get_logger().info(
            f'{name}: 0x{joint_id:03X} -> {position_abs_deg:.1f}° @ {velocity:.0f} dps (ABS)'
        )

    def _send_joint_command_rel(self, joint_id, delta_deg, velocity, name='Joint REL'):
        """JointControl 메시지 발행 (상대 위치)"""
        joint_msg = JointControl()
        joint_msg.joint_id = joint_id
        joint_msg.position = delta_deg
        joint_msg.velocity = velocity
        joint_msg.control_mode = JointControl.MODE_RELATIVE

        self.joint_publisher.publish(joint_msg)
        self.last_command_time = self.get_clock().now()

        direction = '+' if delta_deg > 0 else ''
        self.get_logger().info(
            f'{name}: 0x{joint_id:03X} {direction}{delta_deg:.1f}° @ {velocity:.0f} dps (REL)'
        )

    def destroy_node(self):
        """노드 종료 시 정리"""
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JointController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

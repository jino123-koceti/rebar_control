#!/usr/bin/env python3
"""
Joint Controller Node
리모콘 스위치 입력을 관절 모터 위치 제어 명령으로 변환

제어 모터:
- 0x143: 횡이동 (Lateral) - S17/S18, 360도 단위
- 0x147: Yaw 회전 - S23/S24, 5도 단위

제어 방식:
- 0x143 (횡이동): 0x90 싱글턴 엔코더 기반 상대 위치 제어 (S17/S18: ±360°)
- 0x147 (Yaw): 절대/상대 위치 제어 (S23/S24: ±5°)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from rebar_base_interfaces.msg import RemoteControl, JointControl, MotorFeedback
from std_msgs.msg import String, Bool
import time
from enum import Enum


class HomingState(Enum):
    IDLE = 0
    Z_UP = 1            # Z축 상승 (z_min)
    COARSE_HOME = 2     # X+Y+Yaw 동시 리미트 이동 (거시적 호밍)
    BACK_OFF = 3        # 리미트에서 후퇴
    FINE_HOME = 4       # 느린 속도로 다시 리미트 접근 (미시적 호밍)
    MOVE_TO_READY = 5   # 준비 위치로 이동
    COMPLETE = 6


class JointController(Node):

    def __init__(self):
        super().__init__('joint_controller')
        self.nodeName = self.get_name()
        self.get_logger().info(f'{self.nodeName} started')

        # Parameters - 횡이동 (0x143)
        self.declare_parameter('lateral_step_deg', 360.0)
        self.declare_parameter('lateral_max_speed', 200.0)
        # Parameters - Yaw (0x147)
        self.declare_parameter('yaw_step_deg', 10.0)
        self.declare_parameter('yaw_max_speed', 134.0)
        # Parameters - 공통
        self.declare_parameter('command_interval', 6.0)
        # 0x90 단회전(16bit) → 0~65535 counts
        self.declare_parameter('lateral_encoder_cpr', 65536.0)
        # 허용 오차: 각도(deg) 기준
        self.declare_parameter('position_tolerance', 1.0)
        # 12시 방향 각도 (0x94 기준, return_12.py 캘리브레이션 값)
        self.declare_parameter('home_angle_94', 33.50)
        # 홈 위치 0x90 엔코더 값 (33.50° 위치의 0x90 값, 2025-12-26 하드웨어 재조립 후 캘리브레이션)
        self.declare_parameter('home_encoder_90', 24399)

        # Parameters - 3축 스테이지 (0x144, 0x145, 0x146)
        self.declare_parameter('stage_x_step_deg', 36.0)      # 1mm = 36° (리드스크류 10mm/rev)
        self.declare_parameter('stage_x_max_speed', 200.0)    # dps
        self.declare_parameter('stage_y_step_deg', 36.0)
        self.declare_parameter('stage_y_max_speed', 200.0)
        self.declare_parameter('stage_z_step_deg', 36.0)
        self.declare_parameter('stage_z_max_speed', 200.0)
        self.declare_parameter('joystick_deadzone', 0.15)     # 15% 데드존
        self.declare_parameter('joystick_scale', 1.0)         # 조이스틱 감도

        # 리밋 센서 사용 여부
        self.declare_parameter('use_limit_sensors', True)

        # Homing parameters
        self.declare_parameter('homing_speed', 100.0)
        self.declare_parameter('homing_fine_speed', 30.0)
        self.declare_parameter('homing_backoff_time', 0.5)
        self.declare_parameter('homing_timeout', 30.0)
        self.declare_parameter('yaw_home_encoder_90', 16328)
        self.declare_parameter('yaw_max_encoder_90', 44710)
        self.declare_parameter('yaw_full_stroke_deg', 400.0)
        self.declare_parameter('ready_x_mm', 0.0)
        self.declare_parameter('ready_y_mm', 0.0)
        self.declare_parameter('ready_z_mm', 0.0)
        self.declare_parameter('ready_yaw_deg', 0.0)

        self.lateral_step = self.get_parameter('lateral_step_deg').value
        self.lateral_speed = self.get_parameter('lateral_max_speed').value
        self.yaw_step = self.get_parameter('yaw_step_deg').value
        self.yaw_speed = self.get_parameter('yaw_max_speed').value
        self.command_interval = self.get_parameter('command_interval').value
        self.lateral_cpr = self.get_parameter('lateral_encoder_cpr').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.home_angle_94_target = self.get_parameter('home_angle_94').value  # 12시 = 33.50° (0x94)
        self.home_encoder_fixed = int(self.get_parameter('home_encoder_90').value)  # 홈 엔코더 고정값

        # 3축 스테이지 파라미터
        self.stage_x_step = self.get_parameter('stage_x_step_deg').value
        self.stage_x_speed = self.get_parameter('stage_x_max_speed').value
        self.stage_y_step = self.get_parameter('stage_y_step_deg').value
        self.stage_y_speed = self.get_parameter('stage_y_max_speed').value
        self.stage_z_step = self.get_parameter('stage_z_step_deg').value
        self.stage_z_speed = self.get_parameter('stage_z_max_speed').value
        self.joystick_deadzone = self.get_parameter('joystick_deadzone').value
        self.joystick_scale = self.get_parameter('joystick_scale').value
        self.use_limit_sensors = self.get_parameter('use_limit_sensors').value

        # Homing parameter values
        self.homing_speed = self.get_parameter('homing_speed').value
        self.homing_fine_speed = self.get_parameter('homing_fine_speed').value
        self.homing_backoff_time = self.get_parameter('homing_backoff_time').value
        self.homing_timeout = self.get_parameter('homing_timeout').value
        self.yaw_home_enc_90 = int(self.get_parameter('yaw_home_encoder_90').value)
        self.yaw_max_enc_90 = int(self.get_parameter('yaw_max_encoder_90').value)
        self.yaw_full_stroke = self.get_parameter('yaw_full_stroke_deg').value
        self.ready_x_mm = self.get_parameter('ready_x_mm').value
        self.ready_y_mm = self.get_parameter('ready_y_mm').value
        self.ready_z_mm = self.get_parameter('ready_z_mm').value
        self.ready_yaw_deg = self.get_parameter('ready_yaw_deg').value

        # 리밋 센서 상태 (True = 리밋 도달, False = 정상)
        self.limit_sensors = {
            'x_min': False,
            'x_max': False,
            'y_min': False,
            'y_max': False,
            'z_min': False,
            'z_max': False,
            'yaw_home': False,
        }

        # 0x90 명령 요청 publisher
        self.encoder_request_pub = self.create_publisher(
            JointControl, '/encoder_request', 10)

        # State
        self.control_mode = 'idle'
        self.remote_msg = RemoteControl()
        self.prev_buttons = [0] * 8
        self.last_command_time = None

        # Home position (12시 위치) - 고정값 사용
        self.home_encoder = self.home_encoder_fixed  # 47282 counts로 고정
        self.current_encoder = None  # 최근 엔코더 값
        self.current_encoder_time = 0.0  # monotonic timestamp of last encoder update
        self.home_angle_deg = None  # 0x92 출력축 각도 기반 홈
        self.current_angle_deg = None  # 최근 출력축 각도 (0x92, deg)
        self.current_angle_time = 0.0
        self.current_angle_94 = None  # 최근 단일 회전 각도 (0x94, deg)
        self.current_angle_94_time = 0.0

        self.get_logger().info(f"🏠 Home encoder initialized: {self.home_encoder} counts (fixed value)")

        # Motion state
        self.motion_state = 'idle'  # idle, moving_to_home, rotating, returning_home

        # Angle request timing (주기적 요청용)
        self.last_angle_request_time = 0.0

        # Subscribers
        self.remote_subscriber = self.create_subscription(
            RemoteControl, '/remote_control', self.recv_remote, qos_profile_sensor_data)
        self.status_subscriber = self.create_subscription(
            String, '/control_mode', self.recv_status, qos_profile_system_default)
        self.motor_feedback_subscriber = self.create_subscription(
            MotorFeedback, '/motor_feedback', self.recv_motor_feedback, 10)

        # Auto 모드용 joint_control 구독 (외부 제어 명령)
        self.joint_control_subscriber = self.create_subscription(
            JointControl, '/joint_control_cmd', self.recv_joint_control_cmd, 10)

        # 리밋 센서 구독 (ezi_io_controller에서 발행)
        if self.use_limit_sensors:
            self._setup_limit_sensor_subscribers()

        # Publisher
        self.joint_publisher = self.create_publisher(
            JointControl, '/joint_control', qos_profile_system_default)

        # 횡이동 완료 신호 publisher
        self.lateral_complete_pub = self.create_publisher(
            String, '/lateral_motion_complete', 10)

        # Homing topics
        self.homing_cmd_sub = self.create_subscription(
            String, '/homing_cmd', self._recv_homing_cmd, 10)
        self.homing_status_pub = self.create_publisher(
            String, '/homing_status', 10)

        # Auto 모드 횡이동 추적 상태
        self.auto_lateral_target_encoder = None  # 목표 엔코더 값
        self.auto_lateral_in_progress = False  # 횡이동 진행 중 플래그
        self.auto_lateral_tolerance_counts = int(self.lateral_cpr * self.position_tolerance / 360.0)
        # 12시 정렬 후 회전 대기 상태
        self.auto_lateral_pending_rotation = None  # 대기 중인 회전 각도
        self.auto_lateral_pending_velocity = None  # 대기 중인 속도
        self.auto_lateral_aligning_to_home = False  # 12시 정렬 중 플래그
        # 360° 회전 명령 전송 후 settling time (모터가 움직이기 시작할 때까지 대기)
        self.auto_lateral_command_time = None  # 명령 전송 시간 (monotonic)
        self.auto_lateral_settling_time = 0.5  # 명령 전송 후 0.5초 동안 완료 체크 안함

        # Manual 모드: 누적 추적 없음 (싱글턴 엔코더 기반)

        # 3축 스테이지 상태 추적 (0x144, 0x145, 0x146)
        self.stage_x_angle = None       # 0x144 현재 각도 (0x92 멀티턴)
        self.stage_x_angle_time = 0.0
        self.stage_y_angle = None       # 0x145 현재 각도
        self.stage_y_angle_time = 0.0
        self.stage_z_angle = None       # 0x146 현재 각도
        self.stage_z_angle_time = 0.0
        self.yaw_angle = None           # 0x147 현재 각도
        self.yaw_angle_time = 0.0

        # Homing state
        self.homing_state = HomingState.IDLE
        self.homing_cmd_sent = False
        self.homing_start_time = 0.0
        self.homing_state_start_time = 0.0
        self.homing_references = {}  # {motor_id: reference_angle_0x92}
        self.homing_axes_done = set()  # COARSE/FINE 완료 축 추적: {'x', 'y', 'yaw'}
        self.is_homed = False
        self.yaw_encoder_90 = None  # Yaw 0x90 value (16-bit)
        self.yaw_encoder_90_time = 0.0

        # 조이스틱 연속 제어 상태
        self.stage_x_moving = False     # X축 이동 중 플래그
        self.stage_y_moving = False     # Y축 이동 중 플래그
        self.stage_x_stop_count = 0    # X축 정지 명령 잔여 횟수
        self.stage_y_stop_count = 0    # Y축 정지 명령 잔여 횟수
        self.prev_joysticks = [0.0, 0.0, 0.0, 0.0]  # 이전 조이스틱 값

        # Timer (20Hz - tire_roller style)
        self.control_frequency = 20
        self.timer = self.create_timer(1/self.control_frequency, self.process_joint_control)

        self.get_logger().info(f'Lateral: {self.lateral_step}° @ {self.lateral_speed} dps (0x143)')
        self.get_logger().info(f'Yaw: {self.yaw_step}° @ {self.yaw_speed} dps (0x147)')
        self.get_logger().info(f'Stage X: {self.stage_x_step}° @ {self.stage_x_speed} dps (0x144)')
        self.get_logger().info(f'Stage Y: {self.stage_y_step}° @ {self.stage_y_speed} dps (0x145)')
        self.get_logger().info(f'Stage Z: {self.stage_z_step}° @ {self.stage_z_speed} dps (0x146)')
        self.get_logger().info(f'Encoder CPR: {self.lateral_cpr}, Tolerance: {self.position_tolerance} deg')

        # Home position 초기화 (5초 후)
        self.create_timer(5.0, self._calibrate_home_position_once)

    def _calibrate_home_position_once(self):
        """초기 Home position 설정 (1회만 실행)

        0x94가 33.50°일 때의 0x90 엔코더 값을 홈으로 설정
        - 정확히 33.50°일 때만 홈 설정
        - 그 외 위치에서는 홈 설정하지 않음 (S17/S18 입력 시 자동 이동)
        """
        # 0x90 홈 엔코더가 이미 설정되어 있으면 완료
        if self.home_encoder is not None:
            return  # 이미 설정됨

        # 0x94와 0x90 모두 있으면 홈 엔코더 설정
        if self.current_angle_94 is not None and self.current_encoder is not None:
            # 0x94가 33.50° 근처일 때만: 현재 0x90 값을 홈으로 설정
            delta_94 = abs(self.current_angle_94 - self.home_angle_94_target)
            if delta_94 < self.position_tolerance:
                self.home_encoder = self.current_encoder
                self.get_logger().info(
                    f"🏠 Home encoder set: {self.home_encoder} counts "
                    f"(0x94={self.current_angle_94:.2f}° ≈ {self.home_angle_94_target}°)"
                )
                self.get_logger().info(f"   ✅ Home position calibrated at 12 o'clock (33.50°)!")
                return
            else:
                # 0x94가 33.50°가 아니면 홈 설정하지 않음 (DEBUG 레벨로 변경 - 반복 로그 방지)
                self.get_logger().debug(
                    f"📍 Currently at 0x94={self.current_angle_94:.1f}° "
                    f"(target: {self.home_angle_94_target}°), "
                    f"waiting for 12 o'clock to set home encoder..."
                )

        # 0x94 또는 0x90이 없으면 요청
        if self.current_angle_94 is None:
            self._request_single_circle_angle(0x143)
        if self.current_encoder is None:
            self._request_single_turn_encoder(0x143)

        self.get_logger().debug(
            "Home calibration pending (waiting for 0x94 ≈ 33.50° and 0x90 encoder), retry in 5s"
        )
        self.create_timer(5.0, self._calibrate_home_position_once)

    def _setup_limit_sensor_subscribers(self):
        """리밋 센서 토픽 구독 설정"""
        limit_topics = [
            ('x_min', '/limit_sensors/x_min'),
            ('x_max', '/limit_sensors/x_max'),
            ('y_min', '/limit_sensors/y_min'),
            ('y_max', '/limit_sensors/y_max'),
            ('z_min', '/limit_sensors/z_min'),
            ('z_max', '/limit_sensors/z_max'),
            ('yaw_home', '/limit_sensors/yaw_home'),
        ]

        for name, topic in limit_topics:
            self.create_subscription(
                Bool,
                topic,
                lambda msg, n=name: self._recv_limit_sensor(n, msg),
                10
            )

        self.get_logger().info("✅ 리밋 센서 구독 활성화")

    def _recv_limit_sensor(self, name: str, msg: Bool):
        """리밋 센서 상태 수신"""
        prev_state = self.limit_sensors.get(name, False)
        self.limit_sensors[name] = msg.data

        # 상태 변경 시 로그
        if prev_state != msg.data:
            if msg.data:
                self.get_logger().warn(f"⚠️ 리밋 도달: {name}")
            else:
                self.get_logger().info(f"✅ 리밋 해제: {name}")

        # Homing: rising edge (False→True) 감지
        if self.homing_state != HomingState.IDLE and not prev_state and msg.data:
            self._homing_on_limit_triggered(name)

    def _check_limit_safe(self, axis: str, direction: float) -> bool:
        """리밋 센서 체크하여 이동 가능 여부 반환

        Args:
            axis: 'x', 'y', 'z', 'yaw' 중 하나
            direction: 이동 방향 (+면 max 방향, -면 min 방향)

        Returns:
            True: 이동 가능
            False: 리밋에 도달하여 이동 불가
        """
        if not self.use_limit_sensors:
            return True  # 리밋 센서 비활성화 시 항상 허용

        if axis == 'x':
            # x_min은 홈센서: ON일 때 + 방향 막음 (- 방향은 허용)
            # x_max는 끝단 리밋: ON일 때 - 방향 막음
            if direction > 0 and self.limit_sensors['x_min']:
                return False
            if direction < 0 and self.limit_sensors['x_max']:
                return False
        elif axis == 'y':
            if direction > 0 and self.limit_sensors['y_max']:
                return False
            if direction < 0 and self.limit_sensors['y_min']:
                return False
        elif axis == 'z':
            if direction > 0 and self.limit_sensors['z_max']:
                return False
            if direction < 0 and self.limit_sensors['z_min']:
                return False
        elif axis == 'yaw':
            # Yaw는 원점 센서만 있음 (홈 위치 감지용, 이동 제한 아님)
            pass

        return True

    def recv_status(self, msg: String):
        self.control_mode = msg.data

    def recv_remote(self, msg: RemoteControl):
        self.remote_msg = msg

    def recv_joint_control_cmd(self, msg: JointControl):
        """
        Auto 모드에서 외부 제어 명령 수신
        (navigator → rebar_controller 또는 상위 제어 노드에서)

        0x143 횡이동: 홈 엔코더 기준 상대 위치 제어로 변환
        기타 모터: 그대로 전달
        """
        if self.control_mode != 'auto':
            self.get_logger().debug(
                f"Joint control command ignored (mode={self.control_mode}, expected 'auto')"
            )
            return

        # Auto 모드에서만 외부 명령 처리
        self.get_logger().info(
            f"[Auto] Joint control command received: "
            f"id=0x{msg.joint_id:02X}, pos={msg.position:.1f}°, "
            f"vel={msg.velocity:.1f} dps, mode={msg.control_mode}"
        )

        # 0x143 횡이동: 홈 엔코더 기준 상대 위치 제어로 변환
        if msg.joint_id == 0x143 and msg.control_mode == JointControl.MODE_RELATIVE:
            self._handle_auto_lateral_motion(msg)
        else:
            # 리밋 센서 안전 체크 (0x144~0x147)
            axis_map = {0x144: 'x', 0x145: 'y', 0x146: 'z', 0x147: 'yaw'}
            axis = axis_map.get(msg.joint_id)
            if axis and not self._check_limit_safe(axis, msg.position):
                self.get_logger().warn(
                    f"[Auto] 리밋 센서 차단: 0x{msg.joint_id:03X} ({axis}) "
                    f"방향={msg.position:.1f}° - 이동 거부"
                )
                return

            self.joint_publisher.publish(msg)
            self.last_command_time = self.get_clock().now()

    def recv_motor_feedback(self, msg: MotorFeedback):
        """모터 피드백 수신 (엔코더 값 업데이트)"""
        # 관측용 로깅 (DEBUG 레벨로 변경 - 반복 로그 방지)
        if msg.motor_id == 0x43:
            self.get_logger().debug(
                f"[FEEDBACK] id:0x{msg.motor_id:02X} status:0x{int(msg.status):02X} pos:{msg.current_position}"
            )

        if msg.motor_id == 0x43:  # 0x143
            # 0x94: Single-turn angle (0~360°, already converted by can_parser)
            if msg.status == 0x94:
                # current_position은 can_parser에서 이미 degree로 변환됨
                angle_94 = float(msg.current_position)
                self.current_angle_94 = angle_94
                self.current_angle_94_time = time.monotonic()

                self.get_logger().debug(
                    f"[0x94] Updated single-turn angle: {angle_94:.2f}°"
                )
                return

            # 0x92: Multi-turn angle (출력축 기준, already converted by can_parser)
            if msg.status == 0x92:
                # current_position은 can_parser에서 이미 degree로 변환됨 (multi-turn 누적)
                angle_deg = float(msg.current_position)
                self.current_angle_deg = angle_deg
                self.current_angle_time = time.monotonic()

                # 0x92 멀티턴 각도 자동 설정 비활성화 (0x90 엔코더만 사용, 47282 counts 고정)
                # if self.home_angle_deg is None and self.current_angle_94 is not None:
                #     if abs(self.current_angle_94 - self.home_angle_94_target) < self.position_tolerance:
                #         self.home_angle_deg = angle_deg
                #         self.get_logger().info(
                #             f"🏠 Home angle auto-set: 0x92={self.home_angle_deg:.2f}° "
                #             f"(0x94={self.current_angle_94:.2f}° ≈ {self.home_angle_94_target}°)"
                #         )

                self.get_logger().debug(
                    f"[0x92] Updated multi-turn angle: {angle_deg:.2f}°"
                )
                return

            # 0x90 응답만 사용 (싱글턴 엔코더)
            if msg.status == 0x90:
                encoder_value = int(msg.current_position) & 0xFFFF
                self.current_encoder = encoder_value
                self.current_encoder_time = time.monotonic()

                # 홈 엔코더 자동 설정 (0x94와 0x90 모두 있을 때)
                # _calibrate_home_position_once에서 처리하므로 여기서는 로깅만

                self.get_logger().debug(
                    f"[0x90] Updated encoder: {encoder_value}"
                )

                # Auto 모드 횡이동 완료 감지
                self._check_lateral_completion()

        # 3축 스테이지 피드백 처리 (0x44=X, 0x45=Y, 0x46=Z, 0x47=Yaw)
        elif msg.motor_id == 0x44:  # 0x144 X축
            if msg.status == 0x92:
                self.stage_x_angle = float(msg.current_position)
                self.stage_x_angle_time = time.monotonic()
                self.get_logger().debug(f"[0x144 X] angle: {self.stage_x_angle:.2f}°")

        elif msg.motor_id == 0x45:  # 0x145 Y축
            if msg.status == 0x92:
                self.stage_y_angle = float(msg.current_position)
                self.stage_y_angle_time = time.monotonic()
                self.get_logger().debug(f"[0x145 Y] angle: {self.stage_y_angle:.2f}°")

        elif msg.motor_id == 0x46:  # 0x146 Z축
            if msg.status == 0x92:
                self.stage_z_angle = float(msg.current_position)
                self.stage_z_angle_time = time.monotonic()
                self.get_logger().debug(f"[0x146 Z] angle: {self.stage_z_angle:.2f}°")

        elif msg.motor_id == 0x47:  # 0x147 Yaw
            if msg.status == 0x92:
                self.yaw_angle = float(msg.current_position)
                self.yaw_angle_time = time.monotonic()
                self.get_logger().debug(f"[0x147 Yaw] angle: {self.yaw_angle:.2f}°")
            elif msg.status == 0x90:
                enc_16 = int(msg.current_position) & 0xFFFF
                self.yaw_encoder_90 = enc_16
                self.yaw_encoder_90_time = time.monotonic()
                self.get_logger().debug(f"[0x147 Yaw] 0x90 encoder: {enc_16}")

    def process_joint_control(self):
        """주기적으로 리모콘 입력 처리 및 Auto 모드 횡이동 완료 감지"""
        # 주기적으로 0x90/0x92/0x94 요청 (0.2초마다) - Manual/Auto 모두
        # 0x90: 싱글턴 엔코더 (횡이동 완료 감지용)
        # 0x92: 멀티턴 각도 (횡이동 제어용)
        # 0x94: 단일 회전 각도 (12시 정렬 확인용)
        current_time = time.monotonic()
        if current_time - self.last_angle_request_time > 0.2:
            # 횡이동 (0x143)
            self._request_single_turn_encoder(0x143)  # 0x90 요청
            self._request_single_circle_angle(0x143)  # 0x94 요청
            self._request_output_angle(0x143)  # 0x92 요청
            # 3축 스테이지 (0x144~0x146) + Yaw (0x147)
            self._request_output_angle(0x144)  # X축 0x92 요청
            self._request_output_angle(0x145)  # Y축 0x92 요청
            self._request_output_angle(0x146)  # Z축 0x92 요청
            self._request_output_angle(0x147)  # Yaw 0x92 요청
            # Homing 중: Yaw 0x90 추가 요청
            if self.homing_state != HomingState.IDLE:
                self._request_single_turn_encoder(0x147)
            self.last_angle_request_time = current_time

        # Homing 중: 호밍 상태머신만 실행 (일반 제어 비활성화)
        if self.homing_state != HomingState.IDLE:
            self._homing_loop()
            return

        # Auto 모드: 횡이동 완료 감지만 수행 (리모콘 입력 무시)
        if self.control_mode == 'auto':
            self._check_lateral_completion()
            return

        # Manual 모드가 아니면 리모콘 입력 처리 안함
        if self.control_mode != 'manual':
            return

        msg = self.remote_msg
        if not msg.buttons or len(msg.buttons) < 8:
            return

        # AN1/AN2: X/Y축 스테이지 (0x144, 0x145) - 조이스틱 연속 제어
        # command_interval 무관하게 항상 처리 (정지 명령 누락 방지)
        self._handle_stage_xy(msg)

        # 명령 간격 제한 (토글 스위치 중복 실행 방지용)
        if self.last_command_time:
            elapsed = (self.get_clock().now() - self.last_command_time).nanoseconds / 1e9
            if elapsed < self.command_interval:
                return

        # S17/S18: 횡이동 (0x143) - 0x90 싱글턴 엔코더 기반 상대 위치 1회전
        self._handle_lateral_with_home(msg)

        # S23/S24: Yaw 회전 (0x147)
        self._handle_yaw(msg)

        # S21/S22: Z축 시퀀스는 sequence_controller.py에서 처리

        # 이전 버튼 상태 저장
        self.prev_buttons = list(msg.buttons)
        if hasattr(msg, 'joysticks') and msg.joysticks:
            self.prev_joysticks = list(msg.joysticks)

    def _handle_lateral_with_home(self, msg):
        """S17/S18 → 0x90 싱글턴 엔코더 기반 상대 위치 1회전"""
        s17 = msg.buttons[2]
        s18 = msg.buttons[3]
        prev_s17 = self.prev_buttons[2] if len(self.prev_buttons) > 2 else 0
        prev_s18 = self.prev_buttons[3] if len(self.prev_buttons) > 3 else 0

        # 엣지 감지 (0→1)
        if prev_s17 == 0 and s17 == 1:
            self._execute_single_turn_rotation(direction='+')  # 시계 방향
        elif prev_s18 == 0 and s18 == 1:
            self._execute_single_turn_rotation(direction='-')  # 반시계 방향

    def _execute_single_turn_rotation(self, direction):
        """Manual 모드 1회전 (0x92 멀티턴 엔코더 기반, 최단 경로 12시 정렬)

        S17: 시계 방향 +360° 회전
        S18: 반시계 방향 -360° 회전

        로직:
        1. 현재 0x92 멀티턴 각도 읽기 (예: 1000°)
        2. 가장 가까운 12시(33.50 + n*360) 찾기 (최단 경로, 360° 이내)
        3. 가장 가까운 12시로 이동 (절대 위치)
        4. 거기서 ±360° 회전 (절대 위치)

        예시:
        - 현재 1000° → 가장 가까운 12시 = 874.94° (차이: -125.06°) → 874.94°로 이동
        - S17: 874.94 + 360 = 1234.94°로 이동

        프로그램 재시작 후에도 항상 360° 이내 회전으로 12시 정렬!
        """
        # 명령 간격 제한 확인
        if self.last_command_time:
            elapsed = (self.get_clock().now() - self.last_command_time).nanoseconds / 1e9
            if elapsed < self.command_interval:
                self.get_logger().warn(
                    f"⚠️ 명령 간격 부족: {elapsed:.1f}s < {self.command_interval}s, 대기 중..."
                )
                return

        # 현재 0x92 멀티턴 각도 읽기
        if self.current_angle_deg is None:
            self._request_output_angle(0x143)
            self.get_logger().info("📡 0x92 멀티턴 각도 요청 중... (다음 입력에서 재시도)")
            return

        # 현재 0x90 엔코더 값 확인 (로깅용)
        if self.current_encoder is None:
            self._request_single_turn_encoder(0x143)

        current_angle = self.current_angle_deg  # 멀티턴 (예: 1000°)

        # 가장 가까운 12시(33.50 + n*360) 찾기
        # 현재 각도를 360으로 나눈 나머지와 33.50 비교
        remainder = current_angle % 360.0  # 예: 1000 % 360 = 280°
        delta_in_circle = self.home_angle_94_target - remainder  # 33.50 - 280 = -246.50°

        # 최단 경로 (-180 ~ +180)
        if delta_in_circle > 180.0:
            delta_in_circle -= 360.0
        elif delta_in_circle < -180.0:
            delta_in_circle += 360.0

        nearest_home = current_angle + delta_in_circle  # 1000 + (-125.06) = 874.94°

        # 1단계: 12시로 정렬 (tolerance 이내가 아니면)
        if abs(delta_in_circle) > self.position_tolerance:
            self.get_logger().info(
                f"📍 12시 정렬: {current_angle:.1f}° → {nearest_home:.1f}° "
                f"(delta={delta_in_circle:.1f}°, 360° 이내)"
            )
            self._send_joint_command_abs(
                0x143,
                nearest_home,
                self.lateral_speed,
                '12시 정렬 (멀티턴)'
            )
            self.last_command_time = self.get_clock().now()
            return

        # 2단계: 12시에서 ±360° 회전 (절대 위치)
        rotation_deg = 360.0 if direction == '+' else -360.0
        target_angle = nearest_home + rotation_deg

        self.get_logger().info(
            f"🔄 12시{direction}360° 회전: "
            f"0x92={current_angle:.1f}° → {target_angle:.1f}°, 0x90={self.current_encoder}"
        )

        self._send_joint_command_abs(
            0x143,
            target_angle,
            self.lateral_speed,
            f'12시{direction}360° 회전 (멀티턴)'
        )

        self.last_command_time = self.get_clock().now()

    def _move_to_home_angle(self):
        """33.50°로 이동 (홈 엔코더 설정 전 필수)"""
        if self.current_angle_94 is None:
            self._request_single_circle_angle(0x143)
            return
        
        # 현재 0x94와 목표(33.50°)의 차이 계산
        delta_94 = self.current_angle_94 - self.home_angle_94_target
        
        # 최단 경로 계산 (-180 ~ +180)
        while delta_94 > 180:
            delta_94 -= 360
        while delta_94 < -180:
            delta_94 += 360
        
        # 오차가 tolerance 이내면 홈 설정만 수행
        if abs(delta_94) < self.position_tolerance:
            if self.current_encoder is not None:
                self.home_encoder = self.current_encoder
                self.get_logger().info(
                    f"🏠 Home encoder set: {self.home_encoder} counts "
                    f"(0x94={self.current_angle_94:.2f}° ≈ {self.home_angle_94_target}°)"
                )
            return
        
        # 33.50°로 이동 (상대 위치 제어)
        self.get_logger().info(
            f"🔄 Moving to 12 o'clock (33.50°): "
            f"current={self.current_angle_94:.1f}°, "
            f"delta={delta_94:.1f}°"
        )
        self._send_joint_command_rel(0x143, -delta_94, self.lateral_speed, 'Move to 12 o\'clock (33.50°)')
        self.last_command_time = self.get_clock().now()

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
        self.get_logger().debug(f"[REQ 0x90] motor:0x{motor_id:03X} 단회전 엔코더 요청")
        # 캐시가 있으면 즉시 반환
        if self.current_encoder is not None:
            return self.current_encoder

        # 없으면 0x90 요청만 보내고 None 반환 (콜백 수신 대기)
        self._request_single_turn_encoder(motor_id)
        return None

    def _read_output_angle_deg(self, motor_id):
        """motor_feedback 토픽에서 최근 출력축 각도(0x92) 읽기"""
        self.get_logger().debug(f"[REQ 0x92] motor:0x{motor_id:03X} 출력축 각도 요청")

        if self.current_angle_deg is not None:
            return self.current_angle_deg

        self._request_output_angle(motor_id)
        return None

    def _request_single_turn_encoder(self, motor_id):
        """0x90 엔코더 읽기 요청 발행 (비동기)"""
        self.get_logger().debug(f"[REQ 0x90] motor:0x{motor_id:03X} 단회전 엔코더 요청")
        request_msg = JointControl()
        request_msg.joint_id = motor_id
        request_msg.position = 0.0
        request_msg.velocity = 0.0
        request_msg.control_mode = 0x90
        self.encoder_request_pub.publish(request_msg)

    def _request_output_angle(self, motor_id):
        """0x92 출력축 각도 읽기 요청 발행 (멀티턴, 비동기)"""
        self.get_logger().debug(f"[REQ 0x92] motor:0x{motor_id:03X} 출력축 각도 요청")
        request_msg = JointControl()
        request_msg.joint_id = motor_id
        request_msg.position = 0.0
        request_msg.velocity = 0.0
        request_msg.control_mode = 0x92
        self.encoder_request_pub.publish(request_msg)

    def _request_single_circle_angle(self, motor_id):
        """0x94 단일 회전 각도 읽기 요청 발행 (비동기)"""
        self.get_logger().debug(f"[REQ 0x94] motor:0x{motor_id:03X} 단일 회전 각도 요청")
        request_msg = JointControl()
        request_msg.joint_id = motor_id
        request_msg.position = 0.0
        request_msg.velocity = 0.0
        request_msg.control_mode = 0x94
        self.encoder_request_pub.publish(request_msg)

    def _handle_yaw(self, msg):
        """S23/S24 → Yaw ±step (0x147) - 실제 모터 피드백 기반 절대 위치 제어"""
        if not msg.buttons or len(msg.buttons) < 8:
            return

        # buttons 배열: [s13, s14, s17, s18, s21, s22, s23, s24]
        # S23 = buttons[6], S24 = buttons[7]
        s23 = msg.buttons[6]
        s24 = msg.buttons[7]
        prev_s23 = self.prev_buttons[6] if len(self.prev_buttons) > 6 else 0
        prev_s24 = self.prev_buttons[7] if len(self.prev_buttons) > 7 else 0

        # 엣지 감지 (0→1)
        if prev_s23 == 0 and s23 == 1:
            # 현재 0x92 멀티턴 각도 기반 절대 위치 제어
            if self.yaw_angle is None:
                self._request_output_angle(0x147)
                self.get_logger().warn("📡 0x147 각도 요청 중... (다음 입력에서 재시도)")
                return
            target_angle = self.yaw_angle + self.yaw_step
            self._send_joint_command_abs(0x147, target_angle, self.yaw_speed, 'Yaw +step')
        elif prev_s24 == 0 and s24 == 1:
            # 현재 0x92 멀티턴 각도 기반 절대 위치 제어
            if self.yaw_angle is None:
                self._request_output_angle(0x147)
                self.get_logger().warn("0x147 각도 요청 중... (다음 입력에서 재시도)")
                return
            target_angle = self.yaw_angle - self.yaw_step
            self._send_joint_command_abs(0x147, target_angle, self.yaw_speed, 'Yaw -step')

    def _handle_stage_xy(self, msg):
        """AN1/AN2 조이스틱 → X/Y축 스테이지 속도 제어 (0x144, 0x145)

        조이스틱 기반 속도 제어 (0xA2):
        - AN1 (joysticks[0]) > deadzone: X축 + 속도
        - AN1 (joysticks[0]) < -deadzone: X축 - 속도
        - AN2 (joysticks[1]) > deadzone: Y축 + 속도
        - AN2 (joysticks[1]) < -deadzone: Y축 - 속도
        - 조이스틱이 데드존으로 돌아오면 속도=0 전송 (즉시 정지)
        """
        if not hasattr(msg, 'joysticks') or not msg.joysticks or len(msg.joysticks) < 2:
            return

        joy_x = msg.joysticks[0]  # AN1 → X축
        joy_y = msg.joysticks[1]  # AN2 → Y축

        # X축 (0x144) 속도 제어
        if abs(joy_x) > self.joystick_deadzone:
            # 데드존 보정: 데드존 이후부터 0~1 범위로 정규화
            sign = 1.0 if joy_x > 0 else -1.0
            normalized = (abs(joy_x) - self.joystick_deadzone) / (1.0 - self.joystick_deadzone)
            speed_dps = sign * normalized * self.stage_x_speed * self.joystick_scale

            # 리밋 체크: 이동 방향의 리밋에 도달하면 정지
            if self._check_limit_safe('x', speed_dps):
                self._send_speed_command(0x144, speed_dps)
                self.stage_x_moving = True
                self.stage_x_stop_count = 0
            else:
                # 리밋 도달 → 정지
                if self.stage_x_moving:
                    self.get_logger().warn("⛔ X축 리밋 도달 - 정지")
                    self.stage_x_moving = False
                    self.stage_x_stop_count = 5
        else:
            if self.stage_x_moving:
                self.get_logger().debug("X축 정지 (속도=0)")
                self.stage_x_moving = False
                self.stage_x_stop_count = 5
        # 정지 명령 반복 전송 (CAN 메시지 누락 방지)
        if self.stage_x_stop_count > 0:
            self._send_speed_command(0x144, 0.0)
            self.stage_x_stop_count -= 1

        # Y축 (0x145) 속도 제어 (방향 반전)
        if abs(joy_y) > self.joystick_deadzone:
            sign = 1.0 if joy_y > 0 else -1.0
            normalized = (abs(joy_y) - self.joystick_deadzone) / (1.0 - self.joystick_deadzone)
            speed_dps = -sign * normalized * self.stage_y_speed * self.joystick_scale  # 방향 반전

            # 리밋 체크: 이동 방향의 리밋에 도달하면 정지
            if self._check_limit_safe('y', speed_dps):
                self._send_speed_command(0x145, speed_dps)
                self.stage_y_moving = True
                self.stage_y_stop_count = 0
            else:
                # 리밋 도달 → 정지
                if self.stage_y_moving:
                    self.get_logger().warn("⛔ Y축 리밋 도달 - 정지")
                    self.stage_y_moving = False
                    self.stage_y_stop_count = 5
        else:
            if self.stage_y_moving:
                self.get_logger().debug("Y축 정지 (속도=0)")
                self.stage_y_moving = False
                self.stage_y_stop_count = 5
        # 정지 명령 반복 전송 (CAN 메시지 누락 방지)
        if self.stage_y_stop_count > 0:
            self._send_speed_command(0x145, 0.0)
            self.stage_y_stop_count -= 1

    def _handle_stage_z(self, msg):
        """S21/S22 → Z축 스테이지 스텝 제어 (0x146)

        버튼 기반 스텝 이동:
        - S21 (buttons[4]): Z축 하강 (-step)
        - S22 (buttons[5]): Z축 상승 (+step)
        """
        if not msg.buttons or len(msg.buttons) < 6:
            return

        s21 = msg.buttons[4]
        s22 = msg.buttons[5]
        prev_s21 = self.prev_buttons[4] if len(self.prev_buttons) > 4 else 0
        prev_s22 = self.prev_buttons[5] if len(self.prev_buttons) > 5 else 0

        # 엣지 감지 (0→1)
        if prev_s21 == 0 and s21 == 1:
            # 리밋 체크: 하강 방향 (-) 리밋에 도달하면 무시
            if self._check_limit_safe('z', -1.0):
                self._send_joint_command_rel(
                    0x146,
                    -self.stage_z_step,  # 하강 (-)
                    self.stage_z_speed,
                    'Stage Z 하강'
                )
            else:
                self.get_logger().warn("⛔ Z축 하한 리밋 도달 - 하강 명령 무시")

        elif prev_s22 == 0 and s22 == 1:
            # 리밋 체크: 상승 방향 (+) 리밋에 도달하면 무시
            if self._check_limit_safe('z', 1.0):
                self._send_joint_command_rel(
                    0x146,
                    self.stage_z_step,   # 상승 (+)
                    self.stage_z_speed,
                    'Stage Z 상승'
                )
            else:
                self.get_logger().warn("⛔ Z축 상한 리밋 도달 - 상승 명령 무시")

    def _send_homing_speeds(self, commands):
        """호밍용: 다수 모터에 속도 명령 동시 전송 (CAN 직접)

        Args:
            commands: [(motor_id, speed_dps), ...] 리스트
        """
        for motor_id, speed_dps in commands:
            self._send_speed_command(motor_id, speed_dps)

    def _send_speed_command(self, joint_id, speed_dps):
        """JointControl 메시지 발행 (속도 제어) - 조이스틱 연속 제어용

        Args:
            joint_id: 모터 CAN ID (0x144, 0x145 등)
            speed_dps: 속도 (dps, degree per second)
        """
        joint_msg = JointControl()
        joint_msg.joint_id = joint_id
        joint_msg.position = speed_dps  # MODE_SPEED에서는 position 필드에 속도 지정
        joint_msg.velocity = 0.0  # 사용 안함
        joint_msg.control_mode = JointControl.MODE_SPEED

        self.joint_publisher.publish(joint_msg)
        # 조이스틱 연속 제어는 command_interval 적용 안함

    def _send_joint_command_rel_quiet(self, joint_id, delta_deg, velocity, name='Joint REL'):
        """JointControl 메시지 발행 (상대 위치) - 로그 없이 (조이스틱 연속 제어용)"""
        joint_msg = JointControl()
        joint_msg.joint_id = joint_id
        joint_msg.position = delta_deg
        joint_msg.velocity = velocity
        joint_msg.control_mode = JointControl.MODE_RELATIVE

        self.joint_publisher.publish(joint_msg)
        # 조이스틱 연속 제어는 command_interval 적용 안함
        # self.last_command_time = self.get_clock().now()

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

    def _handle_auto_lateral_motion(self, msg: JointControl):
        """
        Auto 모드에서 횡이동 명령 처리 (S17/S18과 동일한 2단계 방식)

        S17/S18 Manual 모드와 동일하게:
        1단계: 12시(33.50°)로 정렬 (360° 이내 최단 경로)
        2단계: 12시에서 ±360° 회전

        Args:
            msg: JointControl 메시지 (position은 ±360° 회전 방향)
        """
        # 현재 0x92 멀티턴 각도 필요
        if self.current_angle_deg is None:
            self._request_output_angle(0x143)
            self.get_logger().warn("📡 0x92 멀티턴 각도 요청 중... (다음 입력에서 재시도)")
            return

        # 현재 0x90 엔코더 값 확인 (로깅용 + 완료 감지용)
        if self.current_encoder is None:
            self._request_single_turn_encoder(0x143)

        current_angle = self.current_angle_deg  # 멀티턴 (예: 1000°)
        rotation_deg = msg.position  # ±360°

        # 가장 가까운 12시(33.50 + n*360) 찾기
        remainder = current_angle % 360.0
        delta_in_circle = self.home_angle_94_target - remainder

        # 최단 경로 (-180 ~ +180)
        if delta_in_circle > 180.0:
            delta_in_circle -= 360.0
        elif delta_in_circle < -180.0:
            delta_in_circle += 360.0

        nearest_home = current_angle + delta_in_circle

        # 1단계: 12시로 정렬 (tolerance 이내가 아니면)
        if abs(delta_in_circle) > self.position_tolerance:
            self.get_logger().info(
                f"📍 [Auto] 12시 정렬: {current_angle:.1f}° → {nearest_home:.1f}° "
                f"(delta={delta_in_circle:.1f}°, 360° 이내)"
            )
            self._send_joint_command_abs(
                0x143,
                nearest_home,
                msg.velocity,
                '[Auto] 12시 정렬 (멀티턴)'
            )
            # 12시 정렬 후 다시 호출되도록 상태 저장
            self.auto_lateral_pending_rotation = rotation_deg
            self.auto_lateral_pending_velocity = msg.velocity
            self.last_command_time = self.get_clock().now()
            return

        # 2단계: 12시에서 ±360° 회전 (절대 위치)
        target_angle = nearest_home + rotation_deg

        # 목표 엔코더 값 계산 (완료 감지용)
        # 홈에서 rotation_deg만큼 회전 후 다시 홈으로 돌아옴 (360° = 1회전)
        self.auto_lateral_target_encoder = self.home_encoder
        self.auto_lateral_in_progress = True
        # 명령 전송 시간 기록 (settling time용 - 모터가 움직이기 시작할 때까지 대기)
        self.auto_lateral_command_time = time.monotonic()

        self.get_logger().info(
            f"🔄 [Auto] 12시{'+' if rotation_deg > 0 else ''}{rotation_deg:.0f}° 회전: "
            f"0x92={current_angle:.1f}° → {target_angle:.1f}°, 0x90={self.current_encoder}"
        )

        self._send_joint_command_abs(
            0x143,
            target_angle,
            msg.velocity,
            f'[Auto] 12시{rotation_deg:+.0f}° 회전 (멀티턴)'
        )

        self.last_command_time = self.get_clock().now()

    def _check_lateral_completion(self):
        """
        Auto 모드 횡이동 완료 감지

        1. 12시 정렬 완료 체크 → 대기 중인 회전 시작
        2. 360° 회전 완료 체크 → 완료 신호 발행
        """
        # 1. 12시 정렬 완료 체크 (대기 중인 회전이 있을 때)
        if self.auto_lateral_pending_rotation is not None:
            self._check_home_alignment_completion()
            return

        # 2. 360° 회전 완료 체크
        if not self.auto_lateral_in_progress:
            return

        if self.auto_lateral_target_encoder is None or self.current_encoder is None:
            return

        # Settling time 체크: 명령 전송 후 일정 시간 동안 완료 체크 안함
        # (모터가 움직이기 시작할 때까지 대기, 12시 위치에서 바로 완료 판정 방지)
        if self.auto_lateral_command_time is not None:
            elapsed = time.monotonic() - self.auto_lateral_command_time
            if elapsed < self.auto_lateral_settling_time:
                self.get_logger().debug(
                    f"[Auto] Settling time: {elapsed:.2f}s < {self.auto_lateral_settling_time}s, waiting..."
                )
                return

        # 엔코더 차이 계산 (wrap-around 처리)
        delta = self.current_encoder - self.auto_lateral_target_encoder

        if delta > self.lateral_cpr / 2:
            delta -= self.lateral_cpr
        elif delta < -self.lateral_cpr / 2:
            delta += self.lateral_cpr

        # 목표 도달 체크
        if abs(delta) <= self.auto_lateral_tolerance_counts:
            self.get_logger().info(
                f"✅ [Auto] 횡이동 완료! "
                f"(현재={self.current_encoder}, 목표={self.auto_lateral_target_encoder}, "
                f"오차={delta} counts ≈ {delta/self.lateral_cpr*360:.1f}°)"
            )

            # 완료 신호 발행
            complete_msg = String()
            complete_msg.data = "COMPLETE"
            self.lateral_complete_pub.publish(complete_msg)

            # 상태 초기화
            self.auto_lateral_in_progress = False
            self.auto_lateral_target_encoder = None
            self.auto_lateral_command_time = None

    def _check_home_alignment_completion(self):
        """
        12시 정렬 완료 체크

        0x94 (단일 회전 각도)가 33.50° 근처이면 정렬 완료 → 360° 회전 시작
        """
        if self.current_angle_94 is None:
            self._request_single_circle_angle(0x143)
            return

        # 12시(33.50°)까지 오차 확인
        delta_94 = abs(self.current_angle_94 - self.home_angle_94_target)

        # 360° wrap-around 고려
        if delta_94 > 180.0:
            delta_94 = 360.0 - delta_94

        if delta_94 <= self.position_tolerance:
            # 12시 정렬 완료! → 대기 중인 회전 시작
            rotation_deg = self.auto_lateral_pending_rotation
            velocity = self.auto_lateral_pending_velocity

            self.get_logger().info(
                f"✅ [Auto] 12시 정렬 완료 (0x94={self.current_angle_94:.1f}° ≈ {self.home_angle_94_target}°)"
            )

            # 대기 상태 초기화
            self.auto_lateral_pending_rotation = None
            self.auto_lateral_pending_velocity = None

            # 2단계: 360° 회전 시작 (현재 0x92 기준)
            if self.current_angle_deg is not None:
                current_angle = self.current_angle_deg
                target_angle = current_angle + rotation_deg

                # 목표 엔코더 값 계산 (완료 감지용 - 360° 회전 후 다시 홈)
                self.auto_lateral_target_encoder = self.home_encoder
                self.auto_lateral_in_progress = True
                # 명령 전송 시간 기록 (settling time용)
                self.auto_lateral_command_time = time.monotonic()

                self.get_logger().info(
                    f"🔄 [Auto] 12시{'+' if rotation_deg > 0 else ''}{rotation_deg:.0f}° 회전: "
                    f"0x92={current_angle:.1f}° → {target_angle:.1f}°, 0x90={self.current_encoder}"
                )

                self._send_joint_command_abs(
                    0x143,
                    target_angle,
                    velocity,
                    f'[Auto] 12시{rotation_deg:+.0f}° 회전 (멀티턴)'
                )

                self.last_command_time = self.get_clock().now()
            else:
                self._request_output_angle(0x143)
                self.get_logger().warn("📡 0x92 각도 요청 중... (다음 사이클에서 재시도)")

    # ==================== Homing Methods ====================

    def _recv_homing_cmd(self, msg: String):
        """호밍 명령 수신: START, STOP, SET_READY"""
        cmd = msg.data.strip().upper()

        if cmd == 'START':
            if self.homing_state != HomingState.IDLE:
                self.get_logger().warn("Homing already in progress")
                return
            self.get_logger().info("=" * 60)
            self.get_logger().info("  HOMING SEQUENCE START")
            self.get_logger().info("=" * 60)
            self.homing_start_time = time.monotonic()
            self.homing_references = {}
            self.homing_axes_done = set()
            self.is_homed = False
            self._homing_enter_state(HomingState.Z_UP)

        elif cmd == 'STOP':
            if self.homing_state != HomingState.IDLE:
                self.get_logger().warn("Homing STOPPED by user")
                for motor_id in [0x144, 0x145, 0x146, 0x147]:
                    self._send_speed_command(motor_id, 0.0)
                self.homing_state = HomingState.IDLE
                self._publish_homing_status("STOPPED")

        elif cmd == 'SET_READY':
            if not self.is_homed:
                self.get_logger().warn("Cannot SET_READY: not homed yet")
                return
            self._set_ready_position()

    def _homing_enter_state(self, new_state: HomingState):
        """호밍 상태 전환"""
        self.get_logger().info(f"Homing: {self.homing_state.name} -> {new_state.name}")
        self.homing_state = new_state
        self.homing_cmd_sent = False
        self.homing_state_start_time = time.monotonic()
        self._publish_homing_status(f"HOMING_{new_state.name}")

    def _publish_homing_status(self, status: str):
        """호밍 상태 발행"""
        status_msg = String()
        status_msg.data = status
        self.homing_status_pub.publish(status_msg)


    def _homing_loop(self):
        """호밍 상태머신 - process_joint_control에서 호출 (20Hz)
        Z↑ → COARSE(X+Y+Yaw 동시) → BACK_OFF → FINE(X+Y+Yaw 동시) → READY
        """
        now = time.monotonic()

        # 타임아웃 체크
        if now - self.homing_state_start_time > self.homing_timeout:
            self.get_logger().error(
                f"Homing TIMEOUT in state {self.homing_state.name} "
                f"({now - self.homing_state_start_time:.1f}s)"
            )
            self._homing_fail("TIMEOUT")
            return

        # === Z_UP: Z축 상승 (z_min 리미트까지) ===
        if self.homing_state == HomingState.Z_UP:
            if not self.homing_cmd_sent:
                if self.limit_sensors.get('z_min', False):
                    self.get_logger().info("Homing: Z already at z_min, skipping")
                    if self.stage_z_angle is not None:
                        self.homing_references[0x146] = self.stage_z_angle
                    self._homing_enter_state(HomingState.COARSE_HOME)
                    return
                self._send_joint_command_rel(
                    0x146, -9999.0, self.homing_speed, 'Homing: Z UP'
                )
                self.homing_cmd_sent = True
                self.get_logger().info("Homing: Z moving UP to z_min")
            # Transition: z_min limit sensor → _homing_on_limit_triggered

        # === COARSE_HOME: X+Y+Yaw 동시에 리미트로 이동 (거시적 호밍) ===
        elif self.homing_state == HomingState.COARSE_HOME:
            if not self.homing_cmd_sent:
                self.homing_axes_done = set()
                # 이미 리미트에 있는 축은 건너뛰기
                if self.limit_sensors.get('x_min', False):
                    self.homing_axes_done.add('x')
                    self.get_logger().info("Homing COARSE: X already at x_min")
                if self.limit_sensors.get('y_min', False):
                    self.homing_axes_done.add('y')
                    self.get_logger().info("Homing COARSE: Y already at y_min")
                if self.limit_sensors.get('yaw_home', False):
                    self.homing_axes_done.add('yaw')
                    self.get_logger().info("Homing COARSE: Yaw already at home")
                if len(self.homing_axes_done) == 3:
                    self.get_logger().info("Homing COARSE: All axes at limits, skipping")
                    self._homing_enter_state(HomingState.BACK_OFF)
                    return
                self.homing_cmd_sent = True
                yaw_spd = self.homing_speed * 0.5
                self.get_logger().info(
                    f"Homing COARSE: X+Y @ {self.homing_speed} dps, Yaw @ {yaw_spd} dps"
                )
            # 아직 리미트 도달하지 않은 축만 속도 명령 동시 전송
            cmds = []
            if 'x' not in self.homing_axes_done:
                cmds.append((0x144, self.homing_speed))
            if 'y' not in self.homing_axes_done:
                cmds.append((0x145, -self.homing_speed))
            if 'yaw' not in self.homing_axes_done:
                cmds.append((0x147, -self.homing_speed * 0.5))
            if cmds:
                self._send_homing_speeds(cmds)
            # Transition: 모든 축 리미트 → _homing_on_limit_triggered

        # === BACK_OFF: 리미트에서 후퇴 (센서 해제될 때까지) ===
        elif self.homing_state == HomingState.BACK_OFF:
            if not self.homing_cmd_sent:
                self.homing_cmd_sent = True
                self.homing_axes_done = set()  # 센서 해제된 축 추적
                self.get_logger().info("Homing BACK_OFF: Retreating until sensors release")
            # 아직 센서 ON인 축만 후퇴, 해제된 축은 정지
            cmds = []
            if self.limit_sensors.get('x_min', False):
                cmds.append((0x144, -self.homing_speed))
            elif 'x' not in self.homing_axes_done:
                cmds.append((0x144, 0.0))
                self.homing_axes_done.add('x')
                self.get_logger().info("Homing BACK_OFF: X sensor released")
            if self.limit_sensors.get('y_min', False):
                cmds.append((0x145, self.homing_speed))
            elif 'y' not in self.homing_axes_done:
                cmds.append((0x145, 0.0))
                self.homing_axes_done.add('y')
                self.get_logger().info("Homing BACK_OFF: Y sensor released")
            if self.limit_sensors.get('yaw_home', False):
                cmds.append((0x147, self.homing_speed * 0.5))
            elif 'yaw' not in self.homing_axes_done:
                cmds.append((0x147, 0.0))
                self.homing_axes_done.add('yaw')
                self.get_logger().info("Homing BACK_OFF: Yaw sensor released")
            if cmds:
                self._send_homing_speeds(cmds)
            # 3축 모두 센서 해제 → FINE_HOME
            if len(self.homing_axes_done) >= 3:
                self._send_homing_speeds([(0x144, 0.0), (0x145, 0.0), (0x147, 0.0)])
                self.get_logger().info("Homing BACK_OFF: All sensors released")
                self._homing_enter_state(HomingState.FINE_HOME)

        # === FINE_HOME: 느린 속도로 다시 리미트 접근 (미시적 호밍) ===
        elif self.homing_state == HomingState.FINE_HOME:
            if not self.homing_cmd_sent:
                self.homing_axes_done = set()
                # BACK_OFF 후에도 여전히 리미트에 있는 축 건너뛰기
                if self.limit_sensors.get('x_min', False):
                    self.homing_axes_done.add('x')
                    if self.stage_x_angle is not None:
                        self.homing_references[0x144] = self.stage_x_angle
                    self.get_logger().info("Homing FINE: X still at x_min, skipping")
                if self.limit_sensors.get('y_min', False):
                    self.homing_axes_done.add('y')
                    if self.stage_y_angle is not None:
                        self.homing_references[0x145] = self.stage_y_angle
                    self.get_logger().info("Homing FINE: Y still at y_min, skipping")
                if self.limit_sensors.get('yaw_home', False):
                    self.homing_axes_done.add('yaw')
                    if self.yaw_angle is not None:
                        self.homing_references[0x147] = self.yaw_angle
                    self.get_logger().info("Homing FINE: Yaw still at home, skipping")
                if len(self.homing_axes_done) >= 3:
                    self.get_logger().info("Homing FINE: All at limits, skipping to READY")
                    self._homing_enter_state(HomingState.MOVE_TO_READY)
                    return
                self.homing_cmd_sent = True
                moving = 3 - len(self.homing_axes_done)
                yaw_fine = self.homing_fine_speed * 0.5
                self.get_logger().info(
                    f"Homing FINE: {moving} axes, X+Y @ {self.homing_fine_speed} dps, Yaw @ {yaw_fine} dps"
                )
            # 아직 리미트 도달하지 않은 축만 속도 명령 동시 전송
            cmds = []
            if 'x' not in self.homing_axes_done:
                cmds.append((0x144, self.homing_fine_speed))
            if 'y' not in self.homing_axes_done:
                cmds.append((0x145, -self.homing_fine_speed))
            if 'yaw' not in self.homing_axes_done:
                cmds.append((0x147, -self.homing_fine_speed * 0.5))
            if cmds:
                self._send_homing_speeds(cmds)
            # Transition: 모든 축 리미트 → _homing_on_limit_triggered

        # === MOVE_TO_READY ===
        elif self.homing_state == HomingState.MOVE_TO_READY:
            if not self.homing_cmd_sent:
                self._move_axes_to_ready()
                self.homing_cmd_sent = True
                self.homing_state_start_time = time.monotonic()  # 타임아웃 리셋
            else:
                # 2초 안정화 후 완료
                if now - self.homing_state_start_time > 2.0:
                    self._homing_enter_state(HomingState.COMPLETE)

        elif self.homing_state == HomingState.COMPLETE:
            self.is_homed = True
            # Reference angles를 JSON으로 포함하여 발행
            import json
            ref_data = {
                'x': self.homing_references.get(0x144, 0.0),
                'y': self.homing_references.get(0x145, 0.0),
                'z': self.homing_references.get(0x146, 0.0),
                'yaw': self.homing_references.get(0x147, 0.0),
            }
            self._publish_homing_status(f"COMPLETE:{json.dumps(ref_data)}")
            elapsed = now - self.homing_start_time
            self.get_logger().info("=" * 60)
            self.get_logger().info(f"  HOMING COMPLETE ({elapsed:.1f}s)")
            self.get_logger().info(f"  References: {self.homing_references}")
            self.get_logger().info("=" * 60)
            self.homing_state = HomingState.IDLE

    def _homing_on_limit_triggered(self, sensor_name: str):
        """호밍 중 리밋 센서 트리거 처리 (rising edge)"""

        if self.homing_state == HomingState.Z_UP and sensor_name == 'z_min':
            self._send_speed_command(0x146, 0.0)
            if self.stage_z_angle is not None:
                self.homing_references[0x146] = self.stage_z_angle
                # joint_positions 동기화 (상대이동 -9999로 오염된 값 복구)
                self._send_joint_command_abs(
                    0x146, self.stage_z_angle, 0.0, 'Homing: Z sync'
                )
            self.get_logger().info(f"Homing: Z reached z_min (ref={self.stage_z_angle})")
            self._homing_enter_state(HomingState.COARSE_HOME)

        elif self.homing_state in (HomingState.COARSE_HOME, HomingState.FINE_HOME):
            phase = "COARSE" if self.homing_state == HomingState.COARSE_HOME else "FINE"
            next_state = HomingState.BACK_OFF if self.homing_state == HomingState.COARSE_HOME else HomingState.MOVE_TO_READY

            if sensor_name == 'x_min' and 'x' not in self.homing_axes_done:
                self._send_homing_speeds([(0x144, 0.0)])
                self.homing_axes_done.add('x')
                if self.homing_state == HomingState.FINE_HOME and self.stage_x_angle is not None:
                    self.homing_references[0x144] = self.stage_x_angle
                self.get_logger().info(f"Homing {phase}: X reached x_min (ref={self.stage_x_angle})")

            elif sensor_name == 'y_min' and 'y' not in self.homing_axes_done:
                self._send_homing_speeds([(0x145, 0.0)])
                self.homing_axes_done.add('y')
                if self.homing_state == HomingState.FINE_HOME and self.stage_y_angle is not None:
                    self.homing_references[0x145] = self.stage_y_angle
                self.get_logger().info(f"Homing {phase}: Y reached y_min (ref={self.stage_y_angle})")

            elif sensor_name == 'yaw_home' and 'yaw' not in self.homing_axes_done:
                self._send_homing_speeds([(0x147, 0.0)])
                self.homing_axes_done.add('yaw')
                if self.homing_state == HomingState.FINE_HOME and self.yaw_angle is not None:
                    self.homing_references[0x147] = self.yaw_angle
                self.get_logger().info(f"Homing {phase}: Yaw reached home (ref={self.yaw_angle})")

            # 3축 모두 완료 시 다음 상태로 전환
            if len(self.homing_axes_done) >= 3:
                self.get_logger().info(f"Homing {phase}: All axes done -> {next_state.name}")
                self._homing_enter_state(next_state)


    def _homing_fail(self, reason: str):
        """호밍 실패 처리"""
        self._send_homing_speeds([
            (0x144, 0.0), (0x145, 0.0), (0x146, 0.0), (0x147, 0.0)
        ])
        self.homing_state = HomingState.IDLE
        self._publish_homing_status(f"FAILED:{reason}")
        self.get_logger().error(f"Homing FAILED: {reason}")

    def _move_axes_to_ready(self):
        """호밍 완료 후 준비 위치로 이동 + can_sender joint_positions 동기화"""
        self.get_logger().info("Homing: Moving to ready positions...")

        # X: reference + offset (mm -> deg) - 항상 절대 위치 전송 (joint_positions 동기화)
        if 0x144 in self.homing_references:
            x_offset_deg = self.ready_x_mm * self.stage_x_step
            x_target = self.homing_references[0x144] + x_offset_deg
            self._send_joint_command_abs(0x144, x_target, self.stage_x_speed, 'Ready: X')

        # Y: reference + offset (mm -> deg)
        if 0x145 in self.homing_references:
            y_offset_deg = self.ready_y_mm * self.stage_y_step
            y_target = self.homing_references[0x145] + y_offset_deg
            self._send_joint_command_abs(0x145, y_target, self.stage_y_speed, 'Ready: Y')

        # Z: reference + offset (mm -> deg)
        if 0x146 in self.homing_references:
            z_offset_deg = self.ready_z_mm * self.stage_z_step
            z_target = self.homing_references[0x146] + z_offset_deg
            self._send_joint_command_abs(0x146, z_target, self.stage_z_speed, 'Ready: Z')

        # Yaw: reference + offset (deg directly)
        if 0x147 in self.homing_references:
            yaw_target = self.homing_references[0x147] + self.ready_yaw_deg
            self._send_joint_command_abs(0x147, yaw_target, self.yaw_speed, 'Ready: Yaw')

    def _set_ready_position(self):
        """현재 위치를 준비 위치로 기록 (YAML 업데이트 값 출력)"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("  SET READY POSITION")
        self.get_logger().info("=" * 60)

        if 0x144 in self.homing_references and self.stage_x_angle is not None:
            offset_deg = self.stage_x_angle - self.homing_references[0x144]
            offset_mm = offset_deg / self.stage_x_step if self.stage_x_step != 0 else 0
            self.get_logger().info(
                f"  X: ref={self.homing_references[0x144]:.1f} -> "
                f"cur={self.stage_x_angle:.1f} = {offset_mm:.1f}mm"
            )

        if 0x145 in self.homing_references and self.stage_y_angle is not None:
            offset_deg = self.stage_y_angle - self.homing_references[0x145]
            offset_mm = offset_deg / self.stage_y_step if self.stage_y_step != 0 else 0
            self.get_logger().info(
                f"  Y: ref={self.homing_references[0x145]:.1f} -> "
                f"cur={self.stage_y_angle:.1f} = {offset_mm:.1f}mm"
            )

        if 0x146 in self.homing_references and self.stage_z_angle is not None:
            offset_deg = self.stage_z_angle - self.homing_references[0x146]
            offset_mm = offset_deg / self.stage_z_step if self.stage_z_step != 0 else 0
            self.get_logger().info(
                f"  Z: ref={self.homing_references[0x146]:.1f} -> "
                f"cur={self.stage_z_angle:.1f} = {offset_mm:.1f}mm"
            )

        if 0x147 in self.homing_references and self.yaw_angle is not None:
            offset_deg = self.yaw_angle - self.homing_references[0x147]
            self.get_logger().info(
                f"  Yaw: ref={self.homing_references[0x147]:.1f} -> "
                f"cur={self.yaw_angle:.1f} = {offset_deg:.1f}deg"
            )

        self.get_logger().info("=" * 60)
        self.get_logger().info("  Copy above values to can_devices.yaml ready_* parameters")
        self.get_logger().info("=" * 60)

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

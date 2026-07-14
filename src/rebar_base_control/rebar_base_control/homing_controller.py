#!/usr/bin/env python3
"""
Homing Controller Node
호밍 시퀀스를 전담하는 독립 노드 (joint_controller에서 분리)

호밍 시퀀스:
1. Z_SAFE     — Z_min 리미트까지 상승
2. X_SAFE     — X_min 리미트까지 이동
3. YAW_CHECK  — 0x90 엔코더로 Left/Right 자세 판단, Left이면 L→R 복귀
4. YAW_SAFE   — Yaw home 리미트까지 이동
5. Y_SAFE     — Y_min 리미트까지 이동
6. BACK_OFF   — X+Y+Yaw 리미트에서 살짝 후퇴 (센서 해제)
7. FINE_HOME  — 느린 속도로 리미트 재접근 (정밀 호밍, 레퍼런스 기록)
8. READY      — 준비 위치로 이동 (X 3단계 분할)
9. COMPLETE   — 레퍼런스 발행, IDLE 복귀
"""

import rclpy
from rclpy.node import Node
from rebar_base_interfaces.msg import JointControl, MotorFeedback
from std_msgs.msg import String, Bool
import time
import json
from datetime import datetime
from enum import Enum


class HomingState(Enum):
    IDLE = 0
    Z_SAFE = 1
    X_SAFE = 2
    YAW_CHECK = 3       # 0x90으로 Left/Right 판단
    YAW_L2R = 4         # L→R 자세복귀 시퀀스 실행 중
    YAW_SAFE = 5        # Yaw home 리미트까지 이동
    Y_SAFE = 6
    BACK_OFF = 7        # X+Y+Yaw 리미트에서 후퇴
    FINE_HOME = 8       # 느린 속도로 리미트 재접근
    READY = 9           # 준비 위치 이동
    COMPLETE = 10
    Z_ONLY_UP = 11      # Z축 단독: z_min까지 상승
    Z_ONLY_READY = 12   # Z축 단독: ready 위치로 하강


class L2RPhase(Enum):
    """L→R 자세복귀 서브 단계

    Left(~393°)에서는 Y 이동 없이 173°까지 회전 안전.
    1) Yaw → -220° 상대 → ~173° (mid)
    2) Y → -150mm 상대 (y_min 방향, 간섭 방지)
    3) Yaw → -168° 상대 → ~5° (홈 근처)
    4) Y → y_min 리미트까지 이동
    """
    YAW_MID = 1        # Yaw → 173° (중간자세)
    Y_CLEAR = 2        # Y → -200mm 상대이동
    YAW_HOME = 3       # Yaw → ~5° (홈 근처)
    Y_MIN = 4          # Y → y_min 리미트까지
    DONE = 5


class HomingController(Node):

    def __init__(self):
        super().__init__('homing_controller')
        self.get_logger().info('homing_controller started')

        # 파일 로거
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._log_path = f'/tmp/homing_controller_{ts}.log'
        self._log_file = open(self._log_path, 'a')
        self.flog('=== Homing Controller 시작 ===')

        # === Parameters ===
        # 호밍 속도
        self.declare_parameter('homing_speed', 100.0)
        self.declare_parameter('homing_fine_speed', 30.0)
        self.declare_parameter('homing_backoff_time', 0.5)
        self.declare_parameter('homing_timeout', 30.0)

        # Yaw 0x90 엔코더
        self.declare_parameter('yaw_home_encoder_90', 61857)
        self.declare_parameter('yaw_max_encoder_90', 25896)
        self.declare_parameter('yaw_full_stroke_deg', 399.0)
        self.declare_parameter('yaw_left_threshold_90', 43000)

        # 스테이지 deg_per_mm
        self.declare_parameter('stage_x_step_deg', 4.497)
        self.declare_parameter('stage_x_max_speed', 200.0)
        self.declare_parameter('stage_y_step_deg', 4.462)
        self.declare_parameter('stage_y_max_speed', 200.0)
        self.declare_parameter('stage_z_step_deg', 13.45)
        self.declare_parameter('stage_z_max_speed', 200.0)
        self.declare_parameter('yaw_max_speed', 134.0)

        # 준비 위치
        self.declare_parameter('ready_x_mm', -300.0)
        self.declare_parameter('ready_y_mm', 0.0)
        self.declare_parameter('ready_z_mm', 0.0)
        self.declare_parameter('ready_yaw_deg', 0.0)

        # L→R 자세복귀 파라미터 (tying_orchestrator 값 사용)
        self.declare_parameter('pose_change_y_far_mm', 250.0)
        self.declare_parameter('pose_change_y_near_mm', 100.0)
        self.declare_parameter('yaw_approach_deg', 390.0)
        self.declare_parameter('yaw_mid_deg', 173.0)
        # YAW_SAFE에서 yaw_home 리미트 접근 속도 (dps) - 센서 과주행/충돌 방지 (정밀은 FINE_HOME 담당)
        self.declare_parameter('yaw_home_approach_speed', 25.0)

        # === Load parameter values ===
        self.homing_speed = self.get_parameter('homing_speed').value
        self.homing_fine_speed = self.get_parameter('homing_fine_speed').value
        self.homing_backoff_time = self.get_parameter('homing_backoff_time').value
        self.homing_timeout = self.get_parameter('homing_timeout').value

        self.yaw_home_enc_90 = int(self.get_parameter('yaw_home_encoder_90').value)
        self.yaw_max_enc_90 = int(self.get_parameter('yaw_max_encoder_90').value)
        self.yaw_full_stroke = self.get_parameter('yaw_full_stroke_deg').value
        self.yaw_left_threshold_90 = int(self.get_parameter('yaw_left_threshold_90').value)

        self.stage_x_step = self.get_parameter('stage_x_step_deg').value
        self.stage_x_speed = self.get_parameter('stage_x_max_speed').value
        self.stage_y_step = self.get_parameter('stage_y_step_deg').value
        self.stage_y_speed = self.get_parameter('stage_y_max_speed').value
        self.stage_z_step = self.get_parameter('stage_z_step_deg').value
        self.stage_z_speed = self.get_parameter('stage_z_max_speed').value
        self.yaw_speed = self.get_parameter('yaw_max_speed').value

        self.ready_x_mm = self.get_parameter('ready_x_mm').value
        self.ready_y_mm = self.get_parameter('ready_y_mm').value
        self.ready_z_mm = self.get_parameter('ready_z_mm').value
        self.ready_yaw_deg = self.get_parameter('ready_yaw_deg').value

        self.pose_y_far_mm = self.get_parameter('pose_change_y_far_mm').value
        self.pose_y_near_mm = self.get_parameter('pose_change_y_near_mm').value
        self.yaw_approach_deg = self.get_parameter('yaw_approach_deg').value
        self.yaw_mid_deg = self.get_parameter('yaw_mid_deg').value
        self.yaw_home_approach_speed = self.get_parameter('yaw_home_approach_speed').value

        # === State ===
        self.homing_state = HomingState.IDLE
        self.homing_cmd_sent = False
        self.homing_start_time = 0.0
        self.homing_state_start_time = 0.0
        self.homing_references = {}  # {motor_id: reference_angle_0x92}
        self.homing_axes_done = set()
        self.is_homed = False

        # L→R 자세복귀 서브 단계
        self.l2r_phase = L2RPhase.DONE
        self.l2r_cmd_sent = False
        self.l2r_phase_start_time = 0.0

        # 리미트 센서 상태
        self.limit_sensors = {
            'x_min': False, 'x_max': False,
            'y_min': False, 'y_max': False,
            'z_min': False, 'z_max': False,
            'yaw_home': False,
        }

        # 모터 피드백 (0x92 멀티턴 각도)
        self.stage_x_angle = None
        self.stage_y_angle = None
        self.stage_z_angle = None
        self.yaw_angle = None
        self.yaw_encoder_90 = None  # 0x90 값
        self.yaw_encoder_90_time = 0.0

        # === Publishers ===
        self.joint_pub = self.create_publisher(
            JointControl, '/joint_control', 10)
        self.homing_status_pub = self.create_publisher(
            String, '/homing_status', 10)
        self.encoder_request_pub = self.create_publisher(
            JointControl, '/encoder_request', 10)

        # === Subscribers ===
        self.create_subscription(
            String, '/homing_cmd', self._recv_homing_cmd, 10)
        self.create_subscription(
            MotorFeedback, '/motor_feedback', self._recv_motor_feedback, 10)
        self.create_subscription(
            Bool, '/emergency_stop', self._emergency_stop_callback, 10)
        self.create_subscription(
            String, '/mission/command', self._mission_command_callback, 10)

        # 리미트 센서 구독
        for name, topic in [
            ('x_min', '/limit_sensors/x_min'),
            ('x_max', '/limit_sensors/x_max'),
            ('y_min', '/limit_sensors/y_min'),
            ('y_max', '/limit_sensors/y_max'),
            ('z_min', '/limit_sensors/z_min'),
            ('z_max', '/limit_sensors/z_max'),
            ('yaw_home', '/limit_sensors/yaw_home'),
        ]:
            self.create_subscription(
                Bool, topic,
                lambda msg, n=name: self._recv_limit_sensor(n, msg), 10)

        # === Timer (20Hz) ===
        self.last_encoder_request_time = 0.0
        self.timer = self.create_timer(1/20, self._control_loop)

        self.get_logger().info(
            f'Homing params: speed={self.homing_speed}, fine={self.homing_fine_speed}, '
            f'timeout={self.homing_timeout}s')
        self.get_logger().info(
            f'Yaw 0x90: home={self.yaw_home_enc_90}, max={self.yaw_max_enc_90}, '
            f'threshold={self.yaw_left_threshold_90}')

    # ==================== Logging ====================

    def flog(self, msg):
        ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        self._log_file.write(f'{ts} {msg}\n')
        self._log_file.flush()

    # ==================== Motor commands ====================

    def _send_speed_command(self, joint_id, speed_dps):
        msg = JointControl()
        msg.joint_id = joint_id
        msg.position = speed_dps
        msg.velocity = 0.0
        msg.control_mode = JointControl.MODE_SPEED
        self.joint_pub.publish(msg)

    def _send_abs_command(self, joint_id, position_deg, velocity, label=''):
        msg = JointControl()
        msg.joint_id = joint_id
        msg.position = position_deg
        msg.velocity = velocity
        msg.control_mode = JointControl.MODE_ABSOLUTE
        self.joint_pub.publish(msg)
        self.get_logger().info(
            f'{label}: 0x{joint_id:03X} -> {position_deg:.1f}deg @ {velocity:.0f}dps (ABS)')

    def _send_rel_command(self, joint_id, delta_deg, velocity, label=''):
        msg = JointControl()
        msg.joint_id = joint_id
        msg.position = delta_deg
        msg.velocity = velocity
        msg.control_mode = JointControl.MODE_RELATIVE
        self.joint_pub.publish(msg)
        self.get_logger().info(
            f'{label}: 0x{joint_id:03X} {delta_deg:+.1f}deg @ {velocity:.0f}dps (REL)')

    def _send_homing_speeds(self, commands):
        for motor_id, speed in commands:
            self._send_speed_command(motor_id, speed)

    def _request_encoder(self, motor_id, cmd_code):
        """엔코더 읽기 요청 (cmd_code: 0x90=싱글턴, 0x92=멀티턴)"""
        msg = JointControl()
        msg.joint_id = motor_id
        msg.position = 0.0
        msg.velocity = 0.0
        msg.control_mode = cmd_code
        self.encoder_request_pub.publish(msg)

    # ==================== Callbacks ====================

    def _recv_motor_feedback(self, msg: MotorFeedback):
        if msg.motor_id == 0x44 and msg.status == 0x92:
            self.stage_x_angle = float(msg.current_position)
        elif msg.motor_id == 0x45 and msg.status == 0x92:
            self.stage_y_angle = float(msg.current_position)
        elif msg.motor_id == 0x46 and msg.status == 0x92:
            self.stage_z_angle = float(msg.current_position)
        elif msg.motor_id == 0x47:
            if msg.status == 0x92:
                self.yaw_angle = float(msg.current_position)
            elif msg.status == 0x90:
                self.yaw_encoder_90 = int(msg.current_position) & 0xFFFF
                self.yaw_encoder_90_time = time.monotonic()

    def _recv_limit_sensor(self, name: str, msg: Bool):
        prev = self.limit_sensors.get(name, False)
        self.limit_sensors[name] = msg.data

        # Rising edge → 호밍 중 리미트 트리거
        if self.homing_state != HomingState.IDLE and not prev and msg.data:
            self._on_limit_triggered(name)

    def _recv_homing_cmd(self, msg: String):
        cmd = msg.data.strip().upper()

        if cmd == 'START':
            if self.homing_state != HomingState.IDLE:
                self.get_logger().warn("Homing already in progress")
                return
            self.get_logger().info("=" * 60)
            self.get_logger().info("  HOMING SEQUENCE START (new)")
            self.get_logger().info("=" * 60)
            self.flog("========== HOMING START ==========")
            self.homing_start_time = time.monotonic()
            self.homing_references = {}
            self.homing_axes_done = set()
            self.is_homed = False
            self._enter_state(HomingState.Z_SAFE)

        elif cmd == 'STOP':
            if self.homing_state != HomingState.IDLE:
                self.get_logger().warn("Homing STOPPED by user")
                self._stop_all_motors()
                self.homing_state = HomingState.IDLE
                self._publish_status("STOPPED")

        elif cmd == 'Z_HOME':
            if self.homing_state != HomingState.IDLE:
                self.get_logger().warn("Z_HOME: homing already in progress")
                return
            self.get_logger().info("=" * 60)
            self.get_logger().info("  Z ONLY HOMING START")
            self.get_logger().info("=" * 60)
            self.homing_start_time = time.monotonic()
            self._enter_state(HomingState.Z_ONLY_UP)

        elif cmd == 'SET_READY':
            if not self.is_homed:
                self.get_logger().warn("Cannot SET_READY: not homed yet")
                return
            self._set_ready_position()

    def _emergency_stop_callback(self, msg: Bool):
        if msg.data and self.homing_state != HomingState.IDLE:
            self.get_logger().error("ESTOP -> homing stopped")
            self._stop_all_motors()
            self.homing_state = HomingState.IDLE
            self._publish_status("EMERGENCY_STOPPED")

    def _mission_command_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            cmd = data.get('command', '')
        except (json.JSONDecodeError, AttributeError):
            cmd = msg.data.strip().upper()

        if cmd in ('EMERGENCY_STOP', 'CANCEL', 'E-STOP', 'STOP'):
            if self.homing_state != HomingState.IDLE:
                self.get_logger().error(f"Mission {cmd} -> homing stopped")
                self._stop_all_motors()
                self.homing_state = HomingState.IDLE
                self._publish_status("EMERGENCY_STOPPED")

    # ==================== State machine helpers ====================

    def _enter_state(self, new_state: HomingState):
        self.get_logger().info(f"Homing: {self.homing_state.name} -> {new_state.name}")
        self.flog(f"State: {self.homing_state.name} -> {new_state.name}")
        self.homing_state = new_state
        self.homing_cmd_sent = False
        self.homing_state_start_time = time.monotonic()
        self._publish_status(f"HOMING_{new_state.name}")

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.homing_status_pub.publish(msg)

    def _stop_all_motors(self):
        for mid in [0x144, 0x145, 0x146, 0x147]:
            self._send_speed_command(mid, 0.0)

    def _homing_fail(self, reason: str):
        self._stop_all_motors()
        self.homing_state = HomingState.IDLE
        self._publish_status(f"FAILED:{reason}")
        self.get_logger().error(f"Homing FAILED: {reason}")
        self.flog(f"FAILED: {reason}")

    # ==================== Main control loop (20Hz) ====================

    def _control_loop(self):
        if self.homing_state == HomingState.IDLE:
            return

        now = time.monotonic()

        # 주기적 엔코더 요청 (0.2초)
        if now - self.last_encoder_request_time > 0.2:
            self._request_encoder(0x144, 0x92)
            self._request_encoder(0x145, 0x92)
            self._request_encoder(0x146, 0x92)
            self._request_encoder(0x147, 0x92)
            self._request_encoder(0x147, 0x90)  # Yaw 0x90
            self.last_encoder_request_time = now

        # 타임아웃 체크 (YAW_L2R은 서브 단계별 타임아웃 별도 관리)
        if self.homing_state != HomingState.YAW_L2R:
            if now - self.homing_state_start_time > self.homing_timeout:
                self.get_logger().error(
                    f"TIMEOUT in {self.homing_state.name} "
                    f"({now - self.homing_state_start_time:.1f}s)")
                self._homing_fail("TIMEOUT")
                return

        self._homing_loop()

    # ==================== Homing state machine ====================

    def _homing_loop(self):
        now = time.monotonic()

        # === Z_SAFE: Z축 z_min까지 상승 ===
        if self.homing_state == HomingState.Z_SAFE:
            if not self.homing_cmd_sent:
                # 센서 안정화 대기 (0.5초)
                if now - self.homing_state_start_time < 0.5:
                    if self.limit_sensors.get('z_min', False):
                        self.get_logger().info("Z_SAFE: already at z_min, skip")
                        self._record_z_ref()
                        self._enter_state(HomingState.X_SAFE)
                    return
                if self.limit_sensors.get('z_min', False):
                    self.get_logger().info("Z_SAFE: already at z_min, skip")
                    self._record_z_ref()
                    self._enter_state(HomingState.X_SAFE)
                    return
                self._send_rel_command(0x146, -9999.0, self.homing_speed, 'Z_SAFE: UP')
                self.homing_cmd_sent = True
            # Transition via _on_limit_triggered('z_min')

        # === X_SAFE: X축 x_min까지 이동 ===
        elif self.homing_state == HomingState.X_SAFE:
            if not self.homing_cmd_sent:
                if self.limit_sensors.get('x_min', False):
                    self.get_logger().info("X_SAFE: already at x_min, skip")
                    self._enter_state(HomingState.YAW_CHECK)
                    return
                # X축: 모터 양수 = x_min 방향
                self._send_speed_command(0x144, self.homing_speed)
                self.homing_cmd_sent = True
                self.get_logger().info("X_SAFE: moving to x_min")
            else:
                # 속도 명령 유지 (20Hz 반복)
                if not self.limit_sensors.get('x_min', False):
                    self._send_speed_command(0x144, self.homing_speed)
            # Transition via _on_limit_triggered('x_min')

        # === YAW_CHECK: 0x90으로 Left/Right 판단 ===
        elif self.homing_state == HomingState.YAW_CHECK:
            if self.yaw_encoder_90 is None:
                # 아직 0x90 값이 없으면 요청 후 대기
                if not self.homing_cmd_sent:
                    self._request_encoder(0x147, 0x90)
                    self.homing_cmd_sent = True
                    self.get_logger().info("YAW_CHECK: requesting 0x90 encoder...")
                return

            enc = self.yaw_encoder_90
            # 0x90은 16bit wrapping (0~65535)
            # Home→Max 경로: 61857 → 65535 → 0 → ... → 25896
            # wrap-around 고려한 거리 계산
            is_left = self._is_yaw_left(enc)
            self.get_logger().info(
                f"YAW_CHECK: 0x90={enc}, threshold={self.yaw_left_threshold_90}, "
                f"{'LEFT' if is_left else 'RIGHT'}")
            self.flog(f"YAW_CHECK: 0x90={enc} -> {'LEFT' if is_left else 'RIGHT'}")

            if is_left:
                self.get_logger().info("YAW_CHECK: Left pose detected -> L2R sequence")
                self._enter_state(HomingState.YAW_L2R)
                self.l2r_phase = L2RPhase.YAW_MID
                self.l2r_cmd_sent = False
                self.l2r_phase_start_time = time.monotonic()
            else:
                self.get_logger().info("YAW_CHECK: Right pose, skip L2R")
                self._enter_state(HomingState.YAW_SAFE)

        # === YAW_L2R: Left→Right 자세복귀 시퀀스 ===
        elif self.homing_state == HomingState.YAW_L2R:
            self._l2r_loop()

        # === YAW_SAFE: Yaw home 리미트까지 이동 ===
        elif self.homing_state == HomingState.YAW_SAFE:
            if not self.homing_cmd_sent:
                if self.limit_sensors.get('yaw_home', False):
                    self.get_logger().info("YAW_SAFE: already at yaw_home, skip")
                    self._enter_state(HomingState.Y_SAFE)
                    return
                # Yaw: 모터 음수 = home 방향 (센서 과주행 방지용 저속 접근)
                self._send_speed_command(0x147, -self.yaw_home_approach_speed)
                self.homing_cmd_sent = True
                self.get_logger().info(
                    f"YAW_SAFE: moving to yaw_home @ {self.yaw_home_approach_speed}dps")
            else:
                if not self.limit_sensors.get('yaw_home', False):
                    self._send_speed_command(0x147, -self.yaw_home_approach_speed)
            # Transition via _on_limit_triggered('yaw_home')

        # === Y_SAFE: Y축 y_min까지 이동 ===
        elif self.homing_state == HomingState.Y_SAFE:
            if not self.homing_cmd_sent:
                if self.limit_sensors.get('y_min', False):
                    self.get_logger().info("Y_SAFE: already at y_min, skip")
                    self._enter_state(HomingState.BACK_OFF)
                    return
                # Y축: 모터 음수 = y_min 방향
                self._send_speed_command(0x145, -self.homing_speed)
                self.homing_cmd_sent = True
                self.get_logger().info("Y_SAFE: moving to y_min")
            else:
                if not self.limit_sensors.get('y_min', False):
                    self._send_speed_command(0x145, -self.homing_speed)
            # Transition via _on_limit_triggered('y_min')

        # === BACK_OFF: X+Y+Yaw 리미트에서 후퇴 ===
        elif self.homing_state == HomingState.BACK_OFF:
            if not self.homing_cmd_sent:
                self.homing_cmd_sent = True
                self.homing_axes_done = set()
                self.get_logger().info("BACK_OFF: retreating until sensors release")
            cmds = []
            if self.limit_sensors.get('x_min', False):
                cmds.append((0x144, -self.homing_speed))
            elif 'x' not in self.homing_axes_done:
                cmds.append((0x144, 0.0))
                self.homing_axes_done.add('x')
                self.get_logger().info("BACK_OFF: X released")
            if self.limit_sensors.get('y_min', False):
                cmds.append((0x145, self.homing_speed))
            elif 'y' not in self.homing_axes_done:
                cmds.append((0x145, 0.0))
                self.homing_axes_done.add('y')
                self.get_logger().info("BACK_OFF: Y released")
            if self.limit_sensors.get('yaw_home', False):
                cmds.append((0x147, self.homing_speed * 0.5))
            elif 'yaw' not in self.homing_axes_done:
                cmds.append((0x147, 0.0))
                self.homing_axes_done.add('yaw')
                self.get_logger().info("BACK_OFF: Yaw released")
            if cmds:
                self._send_homing_speeds(cmds)
            if len(self.homing_axes_done) >= 3:
                self._send_homing_speeds([(0x144, 0.0), (0x145, 0.0), (0x147, 0.0)])
                self.get_logger().info("BACK_OFF: all released")
                self._enter_state(HomingState.FINE_HOME)

        # === FINE_HOME: 느린 속도로 리미트 재접근 ===
        elif self.homing_state == HomingState.FINE_HOME:
            if not self.homing_cmd_sent:
                self.homing_axes_done = set()
                if self.limit_sensors.get('x_min', False):
                    self.homing_axes_done.add('x')
                    if self.stage_x_angle is not None:
                        self.homing_references[0x144] = self.stage_x_angle
                if self.limit_sensors.get('y_min', False):
                    self.homing_axes_done.add('y')
                    if self.stage_y_angle is not None:
                        self.homing_references[0x145] = self.stage_y_angle
                if self.limit_sensors.get('yaw_home', False):
                    self.homing_axes_done.add('yaw')
                    if self.yaw_angle is not None:
                        self.homing_references[0x147] = self.yaw_angle
                if len(self.homing_axes_done) >= 3:
                    self.get_logger().info("FINE_HOME: all at limits, skip to READY")
                    self._enter_state(HomingState.READY)
                    return
                self.homing_cmd_sent = True
                self.get_logger().info(
                    f"FINE_HOME: {3 - len(self.homing_axes_done)} axes @ {self.homing_fine_speed}dps")
            cmds = []
            if 'x' not in self.homing_axes_done:
                cmds.append((0x144, self.homing_fine_speed))
            if 'y' not in self.homing_axes_done:
                cmds.append((0x145, -self.homing_fine_speed))
            if 'yaw' not in self.homing_axes_done:
                cmds.append((0x147, -self.homing_fine_speed * 0.5))
            if cmds:
                self._send_homing_speeds(cmds)
            # Transition via _on_limit_triggered

        # === READY: 준비 위치로 이동 (X 3단계 분할) ===
        elif self.homing_state == HomingState.READY:
            if not self.homing_cmd_sent:
                self.flog("READY: moving to ready positions")
                self._move_axes_to_ready()
                self.homing_cmd_sent = True
                self.homing_state_start_time = time.monotonic()
            else:
                if now - self.homing_state_start_time < 1.0:
                    return
                tolerance = 5.0
                angle_map = {
                    0x144: self.stage_x_angle,
                    0x145: self.stage_y_angle,
                    0x146: self.stage_z_angle,
                    0x147: self.yaw_angle,
                }
                phase = getattr(self, 'ready_phase', 2)

                # X축 phase 1→2→3
                if phase in (1, 2) and hasattr(self, 'ready_x_final'):
                    x_cur = angle_map.get(0x144)
                    x_tgt = self.ready_targets.get(0x144)
                    if x_cur is not None and x_tgt is not None:
                        if abs(x_cur - x_tgt) <= tolerance:
                            if phase == 1:
                                self.ready_phase = 2
                                self._send_abs_command(
                                    0x144, self.ready_x_step2,
                                    self.stage_x_speed, 'Ready X(2/3)')
                                self.ready_targets[0x144] = self.ready_x_step2
                                self.homing_state_start_time = time.monotonic()
                                return
                            else:
                                self.ready_phase = 3
                                self._send_abs_command(
                                    0x144, self.ready_x_final,
                                    self.stage_x_speed, 'Ready X(3/3)')
                                self.ready_targets[0x144] = self.ready_x_final
                                self.homing_state_start_time = time.monotonic()
                                return

                all_reached = True
                for mid, tgt in getattr(self, 'ready_targets', {}).items():
                    cur = angle_map.get(mid)
                    if cur is None or abs(cur - tgt) > tolerance:
                        all_reached = False
                        break
                if all_reached:
                    self.get_logger().info("READY: all axes reached")
                    self.flog("READY: complete")
                    self._enter_state(HomingState.COMPLETE)
                elif now - self.homing_state_start_time > 15.0:
                    self.get_logger().warn("READY: timeout (15s), proceeding anyway")
                    self.flog("READY: timeout")
                    self._enter_state(HomingState.COMPLETE)

        # === COMPLETE ===
        elif self.homing_state == HomingState.COMPLETE:
            self.is_homed = True
            ref_data = {
                'x': self.homing_references.get(0x144, 0.0),
                'y': self.homing_references.get(0x145, 0.0),
                'z': self.homing_references.get(0x146, 0.0),
                'yaw': self.homing_references.get(0x147, 0.0),
            }
            self._publish_status(f"COMPLETE:{json.dumps(ref_data)}")
            elapsed = time.monotonic() - self.homing_start_time
            self.get_logger().info("=" * 60)
            self.get_logger().info(f"  HOMING COMPLETE ({elapsed:.1f}s)")
            self.get_logger().info(f"  References: {self.homing_references}")
            self.get_logger().info("=" * 60)
            self.flog(f"========== COMPLETE ({elapsed:.1f}s) refs={self.homing_references}")
            self.homing_state = HomingState.IDLE

        # === Z_ONLY_UP ===
        elif self.homing_state == HomingState.Z_ONLY_UP:
            if not self.homing_cmd_sent:
                if now - self.homing_state_start_time < 0.5:
                    if self.limit_sensors.get('z_min', False):
                        self.get_logger().info("Z_HOME: already at z_min, skip")
                        self._record_z_ref()
                        self._enter_state(HomingState.Z_ONLY_READY)
                    return
                if self.limit_sensors.get('z_min', False):
                    self.get_logger().info("Z_HOME: already at z_min")
                    self._record_z_ref()
                    self._enter_state(HomingState.Z_ONLY_READY)
                    return
                self._send_rel_command(0x146, -9999.0, self.homing_speed, 'Z_HOME: UP')
                self.homing_cmd_sent = True

        # === Z_ONLY_READY ===
        elif self.homing_state == HomingState.Z_ONLY_READY:
            if not self.homing_cmd_sent:
                if 0x146 in self.homing_references:
                    z_target = self.homing_references[0x146] + self.ready_z_mm * self.stage_z_step
                    self._send_abs_command(0x146, z_target, self.stage_z_speed, 'Z_HOME: READY')
                    self.homing_cmd_sent = True
                else:
                    self.get_logger().warn("Z_HOME: no Z reference")
                    self.homing_state = HomingState.IDLE
                    self._publish_status("Z_HOME_FAILED")
            else:
                if self.stage_z_angle is not None and 0x146 in self.homing_references:
                    z_target = self.homing_references[0x146] + self.ready_z_mm * self.stage_z_step
                    if abs(self.stage_z_angle - z_target) < 1.0:
                        self.get_logger().info("Z_HOME: complete")
                        self._publish_status("Z_HOME_COMPLETE")
                        self.homing_state = HomingState.IDLE
                elif now - self.homing_state_start_time > self.homing_timeout:
                    self.get_logger().warn("Z_HOME: timeout")
                    self._publish_status("Z_HOME_COMPLETE")
                    self.homing_state = HomingState.IDLE

    # ==================== L→R 자세복귀 ====================

    def _l2r_loop(self):
        """Left→Right 자세복귀 서브 상태머신

        Left(~393°)에서는 Y 이동 없이 173°까지 회전 안전.
        1/4: Yaw → -220° 상대 → ~173° (mid)
        2/4: Y → -200mm 상대 (y_min 방향, 간섭 방지)
        3/4: Yaw → -168° 상대 → ~5° (홈 근처)
        4/4: Y → y_min 리미트까지 이동
        """
        now = time.monotonic()
        tolerance = 5.0  # deg

        # 서브 단계 타임아웃 (개별 30초)
        if now - self.l2r_phase_start_time > self.homing_timeout:
            self.get_logger().error(f"L2R TIMEOUT in {self.l2r_phase.name}")
            self._homing_fail("L2R_TIMEOUT")
            return

        # Step 1/4: Yaw → ~173° (mid) — Left에서 Y 이동 없이 안전
        if self.l2r_phase == L2RPhase.YAW_MID:
            if not self.l2r_cmd_sent:
                if self.yaw_angle is not None:
                    yaw_delta = -(self.yaw_approach_deg - self.yaw_mid_deg)  # ~-220°
                    yaw_target = self.yaw_angle + yaw_delta
                    self._send_abs_command(0x147, yaw_target, self.yaw_speed,
                                           'L2R 1/4: Yaw→173(mid)')
                    self.l2r_target = yaw_target
                    self.l2r_cmd_sent = True
                else:
                    self._request_encoder(0x147, 0x92)
            else:
                if self.yaw_angle is not None:
                    if abs(self.yaw_angle - self.l2r_target) < tolerance:
                        self._l2r_next(L2RPhase.Y_CLEAR)
                    elif now - self.l2r_phase_start_time > 15.0:
                        self.get_logger().warn("L2R: YAW_MID timeout, proceeding")
                        self._l2r_next(L2RPhase.Y_CLEAR)

        # Step 2/4: Y → y_min 방향 속도이동 (2초간 100dps 또는 y_min 리미트)
        elif self.l2r_phase == L2RPhase.Y_CLEAR:
            if not self.l2r_cmd_sent:
                if self.limit_sensors.get('y_min', False):
                    self.get_logger().info("L2R 2/4: Y already at y_min, skip")
                    self._l2r_next(L2RPhase.YAW_HOME)
                    return
                self._send_speed_command(0x145, -self.homing_speed)
                self.l2r_cmd_sent = True
                self.get_logger().info("L2R 2/4: Y speed move y_min direction (8s)")
            else:
                if self.limit_sensors.get('y_min', False):
                    self._send_speed_command(0x145, 0.0)
                    self.get_logger().info("L2R 2/4: Y hit y_min")
                    self._l2r_next(L2RPhase.YAW_HOME)
                elif now - self.l2r_phase_start_time > 8.0:
                    self._send_speed_command(0x145, 0.0)
                    self.get_logger().info("L2R 2/4: Y 2s elapsed")
                    self._l2r_next(L2RPhase.YAW_HOME)

        # Step 3/4: Yaw → ~5° (홈 근처)
        elif self.l2r_phase == L2RPhase.YAW_HOME:
            if not self.l2r_cmd_sent:
                if self.yaw_angle is not None:
                    yaw_target = self.yaw_angle - self.yaw_mid_deg + 5.0  # ~-168°
                    self._send_abs_command(0x147, yaw_target, self.yaw_speed,
                                           'L2R 3/4: Yaw→home(~5)')
                    self.l2r_target = yaw_target
                    self.l2r_cmd_sent = True
                else:
                    self._request_encoder(0x147, 0x92)
            else:
                if self.yaw_angle is not None:
                    if abs(self.yaw_angle - self.l2r_target) < tolerance:
                        self._l2r_next(L2RPhase.Y_MIN)
                    elif now - self.l2r_phase_start_time > 15.0:
                        self._l2r_next(L2RPhase.Y_MIN)

        # Step 4/4: Y → y_min 리미트까지 이동
        elif self.l2r_phase == L2RPhase.Y_MIN:
            if not self.l2r_cmd_sent:
                if self.limit_sensors.get('y_min', False):
                    self.get_logger().info("L2R 4/4: Y already at y_min, skip")
                    self._l2r_next(L2RPhase.DONE)
                    return
                self._send_speed_command(0x145, -self.homing_speed)
                self.l2r_cmd_sent = True
                self.get_logger().info("L2R 4/4: Y moving to y_min")
            else:
                if self.limit_sensors.get('y_min', False):
                    self._send_speed_command(0x145, 0.0)
                    self.get_logger().info("L2R 4/4: Y reached y_min")
                    self._l2r_next(L2RPhase.DONE)
                else:
                    self._send_speed_command(0x145, -self.homing_speed)

        elif self.l2r_phase == L2RPhase.DONE:
            self.get_logger().info("L2R: pose return complete -> YAW_SAFE")
            self.flog("L2R complete")
            self._enter_state(HomingState.YAW_SAFE)

    def _l2r_next(self, next_phase: L2RPhase):
        self.get_logger().info(f"L2R: {self.l2r_phase.name} -> {next_phase.name}")
        self.flog(f"L2R: {self.l2r_phase.name} -> {next_phase.name}")
        self.l2r_phase = next_phase
        self.l2r_cmd_sent = False
        self.l2r_phase_start_time = time.monotonic()

    # ==================== Limit trigger ====================

    def _on_limit_triggered(self, sensor_name: str):
        """리미트 센서 rising edge 처리"""

        # Z_SAFE → X_SAFE
        if self.homing_state == HomingState.Z_SAFE and sensor_name == 'z_min':
            self._send_speed_command(0x146, 0.0)
            self._record_z_ref()
            self._sync_z_position()
            self.get_logger().info(f"Z_SAFE: reached z_min (ref={self.stage_z_angle})")
            self._enter_state(HomingState.X_SAFE)

        # X_SAFE → YAW_CHECK
        elif self.homing_state == HomingState.X_SAFE and sensor_name == 'x_min':
            self._send_speed_command(0x144, 0.0)
            self.get_logger().info("X_SAFE: reached x_min")
            self._enter_state(HomingState.YAW_CHECK)

        # YAW_SAFE → Y_SAFE
        elif self.homing_state == HomingState.YAW_SAFE and sensor_name == 'yaw_home':
            self._send_speed_command(0x147, 0.0)
            self.get_logger().info("YAW_SAFE: reached yaw_home")
            self._enter_state(HomingState.Y_SAFE)

        # Y_SAFE → BACK_OFF
        elif self.homing_state == HomingState.Y_SAFE and sensor_name == 'y_min':
            self._send_speed_command(0x145, 0.0)
            self.get_logger().info("Y_SAFE: reached y_min")
            self._enter_state(HomingState.BACK_OFF)

        # Z_ONLY_UP → Z_ONLY_READY
        elif self.homing_state == HomingState.Z_ONLY_UP and sensor_name == 'z_min':
            self._send_speed_command(0x146, 0.0)
            self._record_z_ref()
            self._sync_z_position()
            self.get_logger().info(f"Z_HOME: reached z_min (ref={self.stage_z_angle})")
            self._enter_state(HomingState.Z_ONLY_READY)

        # FINE_HOME: 축별 리미트 트리거
        elif self.homing_state == HomingState.FINE_HOME:
            if sensor_name == 'x_min' and 'x' not in self.homing_axes_done:
                self._send_homing_speeds([(0x144, 0.0)])
                self.homing_axes_done.add('x')
                if self.stage_x_angle is not None:
                    self.homing_references[0x144] = self.stage_x_angle
                self.get_logger().info(f"FINE: X reached x_min (ref={self.stage_x_angle})")

            elif sensor_name == 'y_min' and 'y' not in self.homing_axes_done:
                self._send_homing_speeds([(0x145, 0.0)])
                self.homing_axes_done.add('y')
                if self.stage_y_angle is not None:
                    self.homing_references[0x145] = self.stage_y_angle
                self.get_logger().info(f"FINE: Y reached y_min (ref={self.stage_y_angle})")

            elif sensor_name == 'yaw_home' and 'yaw' not in self.homing_axes_done:
                self._send_homing_speeds([(0x147, 0.0)])
                self.homing_axes_done.add('yaw')
                if self.yaw_angle is not None:
                    self.homing_references[0x147] = self.yaw_angle
                self.get_logger().info(f"FINE: Yaw reached home (ref={self.yaw_angle})")

            if len(self.homing_axes_done) >= 3:
                self.get_logger().info("FINE: all axes done -> READY")
                self._enter_state(HomingState.READY)

    # ==================== Helpers ====================

    def _is_yaw_left(self, enc):
        """0x90 wrap-around 고려한 Left/Right 판단

        Home(61857) → Max(25896) 경로: 61857 → 65535 → 0 → ... → 25896
        총 스트로크 = (65536 - home) + max = 29575 counts
        홈에서의 거리가 스트로크 절반 초과이면 Left
        """
        home = self.yaw_home_enc_90
        max_enc = self.yaw_max_enc_90

        # Home→Max 방향 전체 스트로크 (wrap 고려)
        if home > max_enc:
            total = (65536 - home) + max_enc
        else:
            total = max_enc - home

        # Home에서 현재까지 거리 (Home→Max 방향)
        if enc >= home:
            dist = enc - home
        else:
            dist = (65536 - home) + enc

        return dist > total / 2

    def _record_z_ref(self):
        if self.stage_z_angle is not None:
            self.homing_references[0x146] = self.stage_z_angle

    def _sync_z_position(self):
        """Z축 상대이동 -9999로 오염된 joint_positions 동기화"""
        if self.stage_z_angle is not None:
            self._send_abs_command(
                0x146, self.stage_z_angle, 0.0, 'Z sync')

    def _move_axes_to_ready(self):
        """호밍 완료 후 준비 위치로 이동 (X 3단계 분할)"""
        self.ready_targets = {}
        self.ready_phase = 1

        # X: 3단계 분할
        if 0x144 in self.homing_references:
            x_ref = self.homing_references[0x144]
            x_offset_deg = self.ready_x_mm * self.stage_x_step
            x_target = x_ref + x_offset_deg
            x_step1 = x_ref + (x_target - x_ref) / 3.0
            x_step2 = x_ref + (x_target - x_ref) * 2.0 / 3.0
            self.ready_x_final = x_target
            self.ready_x_step2 = x_step2
            self._send_abs_command(0x144, x_step1, self.stage_x_speed, 'Ready X(1/3)')
            self.ready_targets[0x144] = x_step1
            self.flog(f"  X: ref={x_ref:.1f} -> s1={x_step1:.1f} -> s2={x_step2:.1f} -> final={x_target:.1f}")

        # Y
        if 0x145 in self.homing_references:
            y_target = self.homing_references[0x145] + self.ready_y_mm * self.stage_y_step
            self._send_abs_command(0x145, y_target, self.stage_y_speed, 'Ready Y')
            self.ready_targets[0x145] = y_target

        # Z
        if 0x146 in self.homing_references:
            z_target = self.homing_references[0x146] + self.ready_z_mm * self.stage_z_step
            self._send_abs_command(0x146, z_target, self.stage_z_speed, 'Ready Z')
            self.ready_targets[0x146] = z_target

        # Yaw
        if 0x147 in self.homing_references:
            yaw_target = self.homing_references[0x147] + self.ready_yaw_deg
            self._send_abs_command(0x147, yaw_target, self.yaw_speed, 'Ready Yaw')
            self.ready_targets[0x147] = yaw_target

    def _set_ready_position(self):
        """현재 위치를 준비 위치로 기록"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("  SET READY POSITION")
        self.get_logger().info("=" * 60)

        for mid, name, step, angle in [
            (0x144, 'X', self.stage_x_step, self.stage_x_angle),
            (0x145, 'Y', self.stage_y_step, self.stage_y_angle),
            (0x146, 'Z', self.stage_z_step, self.stage_z_angle),
        ]:
            if mid in self.homing_references and angle is not None:
                offset_deg = angle - self.homing_references[mid]
                offset_mm = offset_deg / step if step != 0 else 0
                self.get_logger().info(
                    f"  {name}: ref={self.homing_references[mid]:.1f} -> "
                    f"cur={angle:.1f} = {offset_mm:.1f}mm")

        if 0x147 in self.homing_references and self.yaw_angle is not None:
            offset_deg = self.yaw_angle - self.homing_references[0x147]
            self.get_logger().info(
                f"  Yaw: ref={self.homing_references[0x147]:.1f} -> "
                f"cur={self.yaw_angle:.1f} = {offset_deg:.1f}deg")

        self.get_logger().info("=" * 60)
        self.get_logger().info("  Copy above values to can_devices.yaml ready_* parameters")
        self.get_logger().info("=" * 60)

    def destroy_node(self):
        if hasattr(self, '_log_file') and self._log_file:
            self._log_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HomingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

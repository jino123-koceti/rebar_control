#!/usr/bin/env python3
"""
Drive Controller Node
리모콘(RemoteControl) 또는 cmd_vel → DriveControl 변환

Tire Roller 방식:
- Manual 모드: 리모콘 조이스틱 값 직접 변환
- Auto 모드: cmd_vel (Twist) 사용
- Differential drive kinematics 적용
"""

from typing import Optional, List
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import Twist, PoseStamped
from rebar_base_interfaces.msg import DriveControl, RemoteControl, MotorFeedback
from std_msgs.msg import String, Bool
import json
import math
import time

from rebar_base_control.velocity_profiler import VelocityProfiler


class DriveController(Node):
    """리모콘과 cmd_vel을 DriveControl로 변환하는 통합 노드 (Tire Roller 방식)"""

    def __init__(self):
        super().__init__('drive_controller')
        # ★ 2026-09-22 워치독·램프 시간은 **단조시계**로 잰다. 벽시계(get_clock)는 NTP가
        #   시계를 **뒤로** 옮기면 경과시간이 음수가 돼 리모콘이 끊겨도 정지를 안 한다
        #   (RTC 백업전원이 없어 부팅마다 1970→현재로 시계가 크게 움직인다).
        self._steady = Clock(clock_type=ClockType.STEADY_TIME)

        # 파라미터 선언
        self.declare_parameter('wheel_base', 0.5)  # m, 바퀴 간 거리
        self.declare_parameter('max_linear_vel', 10.0)  # m/s
        self.declare_parameter('max_angular_vel', 2.0)  # rad/s
        self.declare_parameter('speed_scale_factor', 1.0)  # 속도 스케일링 (1.0 = 100%)
        self.declare_parameter('joystick_deadzone', 20)  # 조이스틱 데드존
        self.declare_parameter('joystick_center', 127)  # 조이스틱 중립값
        self.declare_parameter('publish_frequency', 20)  # Hz
        # 입력 워치독: 명령이 이 시간 이상 끊기면 정지 (0 = 비활성)
        # 모터측 0xB3(500ms)은 CAN 두절/PC 프리즈를 막지만,
        # "PC는 살아있고 리모콘 신호만 두절"은 못 막으므로 여기서 처리한다.
        # ★ 2026-09-22 0.3 → 0.5s. 리모콘 CAN 원본은 0x1E4/0x2E4 16.7Hz·최대간격 0.061s로
        #   깨끗한데(can3 직접 30초 실측), ROS 경로에서 0.31~0.35s 밀림이 며칠간 9회 있었고
        #   그때마다 순간 정지했다. 0.5s = 모터측 0xB3 통신두절 보호와 같은 값.
        self.declare_parameter('remote_timeout_sec', 0.5)
        self.declare_parameter('cmd_vel_timeout_sec', 0.5)
        # 데크끝 방향별 차단 (deck_edge_node → /deck_edge_block)
        # obstacle_pause처럼 전 방향을 래치하면 데크끝에서 후진 탈출까지 막혀 갇힌다.
        # → 나가면 안 되는 방향의 cmd_vel 성분만 0으로 만든다. manual(리모콘)은 미적용
        #   (_convert_remote_to_drive 경로) = 사람이 항상 빼낼 수 있다.
        # 범퍼 방향별 차단 (bumper_node → /bumper_block)
        # 범퍼는 **이미 부딪힌** 신호다. 그렇다고 전 방향을 막으면 빠져나올
        # 수단까지 사라져 그 자리에 갇힌다 → 부딪힌 방향만 막는다(데크끝과 동일).
        self.declare_parameter('bumper_block_enabled', True)
        self.declare_parameter('bumper_block_timeout_sec', 1.5)

        self.declare_parameter('deck_edge_block_enabled', True)
        self.declare_parameter('deck_edge_block_timeout_sec', 1.5)
        # cmd_vel linear.x 부호가 front 카메라 방향(전진)일 때 +1.0.
        # 부호 규약이 바뀌면 이 값만 -1.0으로 뒤집는다.
        self.declare_parameter('deck_edge_forward_sign', 1.0)
        # ★ 가감속 제한 + 전류 상한 (2026-08-14) — 모터 전류 피크를 낮춘다.
        #   실측: 평상시 주행 4.8A인데 **피크 28.3A**(6배), 20A 과전류 플래그 3회.
        #   그리고 **고부하 과도구간에서 시스템 프리즈가 재현**됨([[robot_freeze_safety]]).
        #   전류가 방아쇠라면 이걸 낮추는 것만으로 빈도가 준다 = 가설 검증도 겸한다.
        #   ⚠ 안전 정지(워치독/estop/idle)는 이 로직에 도달하기 전에 처리되므로 무영향.
        self.declare_parameter('accel_limit_mps2', 0.25)   # 가속 한계. 0이면 기능 끔
        self.declare_parameter('decel_limit_mps2', 0.8)    # 감속은 넉넉히(정지 지연 방지)
        # 전류 상한: 넘으면 속도지령을 스케일다운. 사용자 지정 6A에서 시작, 주행이
        # 힘들면 8A까지 올린다(그 이상은 올리지 말 것 — 28A 피크가 문제의 출발점).
        #
        # ── [2026-08-19 실측] 6A 유지하기로 결정. 근거를 남긴다 ──
        #   목표 사양 100mm/s로 올렸더니 **거버너가 47~55%로 깎아 69mm/s밖에 안 났다.**
        #   주행 143초 동안 12회 발동, 피크 11~12.8A. 즉 지금은 **거버너가 속도의
        #   지배적 제약**이다(모터 토크제한 180%는 훨씬 위에 있어 걸리지도 않는다).
        #   ★ 6A는 X4-10 정격 상전류 11.0A(진폭)의 **55%**에 불과하다 —
        #     프리즈 원인을 모르던 2026-08-14에 보수적으로 잡은 값이고,
        #     원인은 전기적 결합으로 밝혀져 8/18 젯슨 전원 분리로 해결됐다
        #     (고부하 48회 무사망, 그 뒤 주행토크도 80%→180%로 올림).
        #   → 8A로 올리면 약 92mm/s, 11A(정격)면 약 126mm/s가 나올 것으로 추정.
        #     **그러나 올리지 않기로 했다**(2026-08-19 사용자 판단). 거버너는 고장이
        #     아니라 설계대로 동작한 것이고, 속도보다 전류 여유를 우선한다.
        #   ⚠ 이 값은 **지속 전류**를 정할 뿐 피크를 못 막는다. 거버너는 사후 반응이라
        #     12.8A 피크는 제한과 무관하게 이미 발생한다.
        self.declare_parameter('current_limit_a', 6.0)
        self.declare_parameter('current_release_a', 4.5)   # 이 아래로 내려가면 회복
        self.declare_parameter('current_min_scale', 0.25)  # 최대 감속 비율(0.25=25%까지)
        # 횡이동(lateral) 제어는 joint_controller.py에서 처리 (0xA4 위치 제어)

        # 파라미터 가져오기
        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_linear = self.get_parameter('max_linear_vel').value
        self.max_angular = self.get_parameter('max_angular_vel').value
        self.speed_scale = self.get_parameter('speed_scale_factor').value
        self.joystick_deadzone = self.get_parameter('joystick_deadzone').value
        self.joystick_center = self.get_parameter('joystick_center').value
        self.publish_frequency = self.get_parameter('publish_frequency').value
        self.remote_timeout = self.get_parameter('remote_timeout_sec').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout_sec').value
        self.bumper_block_enabled = self.get_parameter('bumper_block_enabled').value
        self.bumper_block_timeout = self.get_parameter('bumper_block_timeout_sec').value
        self.deck_block_enabled = self.get_parameter('deck_edge_block_enabled').value
        self.deck_block_timeout = self.get_parameter('deck_edge_block_timeout_sec').value
        self.deck_forward_sign = float(self.get_parameter('deck_edge_forward_sign').value)
        self.accel_limit = float(self.get_parameter('accel_limit_mps2').value)
        self.decel_limit = float(self.get_parameter('decel_limit_mps2').value)
        self.current_limit = float(self.get_parameter('current_limit_a').value)
        self.current_release = float(self.get_parameter('current_release_a').value)
        self.current_min_scale = float(self.get_parameter('current_min_scale').value)
        self._slew_t = None            # 슬루율 계산용 직전 시각
        self._motor_a = {}             # 주행모터별 (피크 |전류|A, 갱신시각)
        self._cur_hold_sec = 0.3       # 피크 홀드 시간 — 콜백 주석 참조
        self._cur_scale = 1.0          # 전류 거버너 스케일 (0~1)

        # 상태 변수
        self.control_mode = 'idle'  # 'idle', 'manual', 'auto', 'emergency_stop', 'obstacle_pause'
        self.obstacle_paused = False
        self.pre_pause_mode = None  # PAUSE 전 모드 저장
        self.cmd_vel_msg = Twist()
        self.remote_control_msg = RemoteControl()
        self.last_drive_msg = DriveControl()

        # 입력 워치독 상태 (None = 아직 수신 없음 → stale 취급하여 정지)
        self.last_remote_time = None
        self.last_cmd_vel_time = None
        self.watchdog_tripped = False

        # 데크끝 방향별 차단 상태 (deck_edge_node 미실행 시 None → 차단 없음 = 기존 동작)
        self.bumper_block = None        # {'forward': bool, 'backward': bool, 'reason': str}
        self.last_bumper_block_time = None
        self.bumper_block_logged = None

        self.deck_block = None          # {'forward': bool, 'backward': bool, 'reason': str}
        self.last_deck_block_time = None
        self.deck_block_logged = None   # 로그 중복 억제용

        # 속도 측정 변수 (AN3 전진 시 1m 달성 측정)
        self.speed_test_active = False
        self.speed_test_start_time = None
        self.speed_test_start_pose = None
        self.speed_test_target_distance = 1.0  # 목표 거리 (m)
        self.current_pose = None
        self.speed_samples = []  # 속도 샘플 저장

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

        # Emergency stop 구독 (즉시 정지용)
        self.emergency_stop_sub = self.create_subscription(
            Bool,
            '/emergency_stop',
            self.emergency_stop_callback,
            10
        )

        # 장애물 PAUSE 구독
        self.obstacle_pause_sub = self.create_subscription(
            Bool,
            '/obstacle_pause',
            self.obstacle_pause_callback,
            10
        )

        # 범퍼 방향별 차단 구독 (bumper_node)
        self.bumper_block_sub = self.create_subscription(
            String,
            '/bumper_block',
            self.bumper_block_callback,
            10
        )

        # 데크끝 방향별 차단 구독 (deck_edge_node)
        self.deck_edge_block_sub = self.create_subscription(
            String,
            '/deck_edge_block',
            self.deck_edge_block_callback,
            10
        )

        # ZED 카메라 pose 구독 (속도 측정용)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/robot_pose',
            self.pose_callback,
            10
        )

        # 주행모터 전류 피드백 (전류 거버너용). can_parser가 0xA2 응답에서
        # 토크 전류를 파싱해 105Hz로 발행한다.
        self.motor_feedback_sub = self.create_subscription(
            MotorFeedback,
            '/motor_feedback',
            self.motor_feedback_callback,
            50
        )

        # 횡이동(lateral) 위치 피드백은 joint_controller.py에서 처리

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
        self.get_logger().info(f"  - Speed scale: {self.speed_scale * 100:.0f}%")
        self.get_logger().info(f"  - Publish frequency: {self.publish_frequency} Hz")

    def control_mode_callback(self, msg: String) -> None:
        """제어 모드 업데이트"""
        old_mode = self.control_mode
        self.control_mode = msg.data

        if old_mode != self.control_mode:
            self.get_logger().info(f"제어 모드 변경: {old_mode} → {self.control_mode}")

    def cmd_vel_callback(self, msg: Twist) -> None:
        """cmd_vel 저장 (Auto 모드에서 사용)"""
        self.cmd_vel_msg = msg
        self.last_cmd_vel_time = self._steady.now()

    def emergency_stop_callback(self, msg: Bool) -> None:
        """E-STOP 시 즉시 정지"""
        if msg.data:
            self.obstacle_paused = False  # ESTOP이 우선
            self.control_mode = 'emergency_stop'
            stop_msg = DriveControl()
            stop_msg.left_speed = 0.0
            stop_msg.right_speed = 0.0
            stop_msg.lateral_speed = 0.0
            self.drive_control_pub.publish(stop_msg)
            self.last_drive_msg = stop_msg

    def obstacle_pause_callback(self, msg: Bool) -> None:
        """장애물 감지 → 주행 일시정지/재개"""
        if self.control_mode == 'emergency_stop':
            return  # ESTOP 중에는 무시

        if msg.data and not self.obstacle_paused:
            # PAUSE: 현재 모드 저장 후 정지
            self.obstacle_paused = True
            self.pre_pause_mode = self.control_mode
            self.control_mode = 'obstacle_pause'
            self.get_logger().warn('장애물 감지 → 주행 PAUSE')
            stop_msg = DriveControl()
            stop_msg.left_speed = 0.0
            stop_msg.right_speed = 0.0
            stop_msg.lateral_speed = 0.0
            self.drive_control_pub.publish(stop_msg)
            self.last_drive_msg = stop_msg

        elif not msg.data and self.obstacle_paused:
            # RESUME: 이전 모드 복귀
            self.obstacle_paused = False
            self.control_mode = self.pre_pause_mode or 'idle'
            self.pre_pause_mode = None
            self.get_logger().info('장애물 해제 → 주행 RESUME')

    def bumper_block_callback(self, msg: String) -> None:
        """범퍼 방향별 차단 수신 (bumper_node). 스키마는 deck_edge_block과 같다."""
        try:
            data = json.loads(msg.data)
            block = {
                'forward': bool(data.get('forward', False)),
                'backward': bool(data.get('backward', False)),
                'reason': str(data.get('reason', '')),
            }
        except (ValueError, TypeError) as e:
            self.get_logger().error(f"bumper_block 파싱 실패: {e}",
                                    throttle_duration_sec=5.0)
            return                          # 직전 상태 유지 (fail-safe)

        self.bumper_block = block
        self.last_bumper_block_time = self._steady.now()

        key = (block['forward'], block['backward'])
        if key != self.bumper_block_logged:
            self.bumper_block_logged = key
            if block['forward'] or block['backward']:
                dirs = ' '.join(d for d, v in (('전진', block['forward']),
                                               ('후진', block['backward'])) if v)
                self.get_logger().warn(f"🛑 범퍼 차단: {dirs} ({block['reason']})")
            else:
                self.get_logger().info("✅ 범퍼 차단 해제 (전/후진 가능)")

    def _bumper_gate(self, linear: float) -> float:
        """범퍼 차단 적용: 부딪힌 방향이면 0, 아니면 그대로.

        데크끝 게이트와 같은 규약 — 인자/반환은 cmd_vel 원본 부호이고
        auto 경로에서만 호출한다(manual은 리모콘으로 빼낼 수 있어야 한다).
        신호가 stale이면 마지막 차단상태를 유지한다: 범퍼를 못 읽는 채로
        계속 밀고 나가는 것보다 멈춰 있는 편이 안전하다.
        """
        if not self.bumper_block_enabled or self.bumper_block is None:
            return linear                   # 노드 미실행 = 기존 동작 그대로

        if self.last_bumper_block_time is not None and self.bumper_block_timeout > 0:
            age = (self._steady.now() - self.last_bumper_block_time).nanoseconds / 1e9
            if age > self.bumper_block_timeout:
                self.get_logger().error(
                    f"⚠ bumper_block {age:.1f}s 끊김 → 마지막 차단상태 유지",
                    throttle_duration_sec=5.0)

        forward_component = self.deck_forward_sign * linear
        blocked = ((forward_component > 0 and self.bumper_block['forward']) or
                   (forward_component < 0 and self.bumper_block['backward']))
        if blocked:
            self.get_logger().warn(
                f"🛑 범퍼: {'전진' if forward_component > 0 else '후진'} 차단 "
                f"({self.bumper_block['reason']})", throttle_duration_sec=1.0)
            return 0.0
        return linear

    def deck_edge_block_callback(self, msg: String) -> None:
        """데크끝 방향별 차단 수신 (deck_edge_node).

        JSON: {"forward": bool, "backward": bool, "reason": str, ...}
        전 방향을 래치하는 obstacle_pause와 달리, 나가면 안 되는 방향만 막는다.
        """
        try:
            data = json.loads(msg.data)
            block = {
                'forward': bool(data.get('forward', False)),
                'backward': bool(data.get('backward', False)),
                'reason': str(data.get('reason', '')),
            }
        except (ValueError, TypeError) as e:
            self.get_logger().error(f"deck_edge_block 파싱 실패: {e}", throttle_duration_sec=5.0)
            return                          # 직전 상태 유지 (fail-safe)

        self.deck_block = block
        self.last_deck_block_time = self._steady.now()

        key = (block['forward'], block['backward'])
        if key != self.deck_block_logged:
            self.deck_block_logged = key
            if block['forward'] or block['backward']:
                dirs = ' '.join(d for d, v in (('전진', block['forward']),
                                               ('후진', block['backward'])) if v)
                self.get_logger().warn(f"🛑 데크끝 차단: {dirs} ({block['reason']})")
            else:
                self.get_logger().info("✅ 데크끝 차단 해제 (전/후진 가능)")

    def _deck_edge_gate(self, linear: float) -> float:
        """데크끝 차단 적용: 나가면 안 되는 방향이면 0, 아니면 그대로.

        인자/반환은 **cmd_vel 원본 부호**(linear.x). deck_edge_forward_sign을 곱한 값이
        양수면 front 카메라 방향이다. auto 경로에서만 호출된다(manual은 항상 탈출 가능).
        차단 정보가 stale이면 마지막 상태를 유지한다 — 감지가 죽었는데 데크 밖으로
        나가는 것보다 멈춰 있는 편이 안전하고, 리모콘 manual로 빼낼 수 있다.
        """
        if not self.deck_block_enabled or self.deck_block is None:
            return linear                   # 노드 미실행 = 기존 동작 그대로

        if self.last_deck_block_time is not None and self.deck_block_timeout > 0:
            age = (self._steady.now() - self.last_deck_block_time).nanoseconds / 1e9
            if age > self.deck_block_timeout:
                self.get_logger().error(
                    f"⚠ deck_edge_block {age:.1f}s 끊김 → 마지막 차단상태 유지",
                    throttle_duration_sec=5.0)

        forward_component = self.deck_forward_sign * linear
        blocked = ((forward_component > 0 and self.deck_block['forward']) or
                   (forward_component < 0 and self.deck_block['backward']))
        if blocked:
            self.get_logger().warn(
                f"🛑 데크끝: {'전진' if forward_component > 0 else '후진'} 차단 "
                f"({self.deck_block['reason']})", throttle_duration_sec=1.0)
            return 0.0
        return linear

    def remote_control_callback(self, msg: RemoteControl) -> None:
        """리모콘 신호 저장 (Manual 모드에서 사용)"""
        self.remote_control_msg = msg
        self.last_remote_time = self._steady.now()
        # S17/S18 횡이동 제어는 joint_controller.py에서 처리 (0xA4 위치 제어)

    def pose_callback(self, msg: PoseStamped) -> None:
        """ZED 카메라 pose 수신 (속도 측정용)"""
        self.current_pose = msg

        # 속도 측정 중이면 거리 체크
        if self.speed_test_active and self.speed_test_start_pose is not None:
            self._check_speed_test_progress()

    def publish_drive_control(self) -> None:
        """
        타이머 콜백: 현재 모드에 따라 DriveControl 발행

        - manual: 리모콘 조이스틱 → DriveControl
        - auto: cmd_vel → DriveControl
        - 기타: 정지
        """
        drive_msg = DriveControl()
        drive_msg.lateral_speed = 0.0

        # 입력 워치독: 명령이 끊긴 채로 마지막 값을 계속 내보내면 폭주한다.
        # stale이면 모드와 무관하게 정지시키고, 입력이 돌아오면 자동 복귀한다.
        stale = self._input_stale()
        if stale:
            if not self.watchdog_tripped:
                self.watchdog_tripped = True
                self.get_logger().error(
                    f"⛔ 입력 워치독 작동 ({stale}) → 주행 정지. 입력 복구 시 자동 재개"
                )
            drive_msg.left_speed = 0.0
            drive_msg.right_speed = 0.0
            drive_msg.lateral_speed = 0.0
            self.drive_control_pub.publish(drive_msg)
            self.last_drive_msg = drive_msg
            return

        if self.watchdog_tripped:
            self.watchdog_tripped = False
            self.get_logger().info("✅ 입력 복구 → 주행 재개")

        if self.control_mode == 'manual':
            # Manual 모드: 리모콘 조이스틱 변환
            drive_msg = self._convert_remote_to_drive()

            # 속도 측정: AN3 전진 시작/종료 감지
            self._handle_speed_test(drive_msg)

            # Manual 모드 조이스틱 로그 (1초 throttle, 움직일 때만)
            if abs(drive_msg.left_speed) > 0.01 or abs(drive_msg.right_speed) > 0.01:
                self.get_logger().info(
                    f"[Manual] left:{drive_msg.left_speed:.2f}m/s right:{drive_msg.right_speed:.2f}m/s "
                    f"(AN3:{self.remote_control_msg.joysticks[2]:.2f} AN4:{self.remote_control_msg.joysticks[3]:.2f})",
                    throttle_duration_sec=1.0
                )

        elif self.control_mode == 'auto' or self.control_mode == 'navigating':
            # Auto 모드: cmd_vel 변환
            drive_msg = self._convert_cmd_vel_to_drive()
            drive_msg.lateral_speed = 0.0
            # Auto 모드 로그 (1초 throttle, 움직일 때만)
            if abs(drive_msg.left_speed) > 0.01 or abs(drive_msg.right_speed) > 0.01:
                self.get_logger().info(
                    f"[Auto] left:{drive_msg.left_speed:.2f}m/s right:{drive_msg.right_speed:.2f}m/s "
                    f"(cmd_vel linear:{self.cmd_vel_msg.linear.x:.2f} angular:{self.cmd_vel_msg.angular.z:.2f})",
                    throttle_duration_sec=1.0
                )

        else:
            # idle, emergency_stop, obstacle_pause 등: 정지
            drive_msg.left_speed = 0.0
            drive_msg.right_speed = 0.0
            drive_msg.lateral_speed = 0.0

        # 횡이동(lateral)은 joint_controller.py에서 JointControl로 제어 (0xA4 위치 제어)

        # ★ 전류 상한 → 슬루율 제한 순서 (둘 다 전류 피크를 낮춘다)
        drive_msg = self._current_governor(drive_msg)
        drive_msg = self._apply_slew(drive_msg)

        # 발행
        self.drive_control_pub.publish(drive_msg)
        self.last_drive_msg = drive_msg

    def motor_feedback_callback(self, msg: MotorFeedback) -> None:
        """주행모터(0x41/0x42) 전류를 **피크 홀드**로 받는다. 전류 거버너 입력.

        ⚠ 왜 피크 홀드인가 (2026-08-14 버그): `can_parser`는 0xA2(속도제어) 응답 외에
           **0x90/0x92/0x94 엔코더 응답도 같은 토픽으로 발행**하는데 그것들은
           `current_current = 0` 이다. 마지막 값만 저장하면 **0짜리가 실제 전류를
           계속 덮어써** 거버너가 21A 주행 중에도 한 번도 발동하지 않았다(로그 0건,
           가짜 15A 주입으로는 정상 발동 확인 → 입력이 문제였음).
        → 최근 `_cur_hold_sec` 동안의 **최댓값을 유지**한다. 그 시간이 지나면 새로 시작.
        """
        if msg.motor_id not in (0x41, 0x42):
            return
        ma = abs(msg.current_current) / 1000.0
        if ma <= 0.0:
            return                      # 엔코더 응답(전류 없음)은 무시
        now = time.time()
        peak, t0 = self._motor_a.get(msg.motor_id, (0.0, 0.0))
        if ma > peak or now - t0 > self._cur_hold_sec:
            self._motor_a[msg.motor_id] = (ma, now)

    def _current_governor(self, msg: DriveControl) -> DriveControl:
        """전류가 상한을 넘으면 속도지령을 줄인다 (히스테리시스).

        ⚠ 왜: 슬루율만으론 **부하가 클 때의 전류**를 못 막는다. 궤도가 배근에서
           빠졌다 올라오는 구간처럼 속도는 그대로인데 토크만 치솟는 상황이
           프리즈와 재현성 있게 겹쳤다([[robot_freeze_safety]] 2026-08-14).
           → 전류를 직접 보고 지령을 깎는다.
        상한(`current_limit_a`)을 넘으면 스케일을 낮추고, `current_release_a`
        아래로 내려가야 회복한다(경계에서 떨리지 않게 히스테리시스).
        ⚠ 스케일다운은 **항상 안전한 방향**(느려짐)이라 정지 성능을 해치지 않는다.
        """
        if self.current_limit <= 0.0 or not self._motor_a:
            return msg
        # 홀드 시간이 지난 값은 버린다(오래된 피크로 계속 제한하지 않게)
        now = time.time()
        live = [v for v, t in self._motor_a.values()
                if now - t <= self._cur_hold_sec]
        if not live:
            self._cur_scale = min(1.0, self._cur_scale + 0.02)
            return msg
        peak = max(live)
        if peak > self.current_limit:
            # ⚠ **즉시 반응**한다 (2026-08-14 수정). 단계적으로 0.05씩 내리면 1.0→0.25에
            #   0.75초가 걸리는데, 궤도가 걸리는 순간의 전류 상승은 그보다 빠르다.
            #   실제로 6A 제한을 걸고도 15.98A까지 올라가 프리즈했다.
            #   → 초과 비율만큼 **한 번에** 깎는다(15.98A에서 6A 목표면 즉시 38%).
            #   회복은 그대로 완만하게(0.02씩) — 급복귀하면 다시 튀어오른다.
            self._cur_scale = max(self.current_min_scale,
                                  min(self._cur_scale, self.current_limit / peak))
            self.get_logger().warn(
                f"⚡ 전류 제한: {peak:.1f}A > {self.current_limit:.1f}A "
                f"→ 속도 {self._cur_scale*100:.0f}%",
                throttle_duration_sec=1.0)
        elif peak < self.current_release:
            self._cur_scale = min(1.0, self._cur_scale + 0.02)
        if self._cur_scale < 1.0:
            msg.left_speed *= self._cur_scale
            msg.right_speed *= self._cur_scale
        return msg

    def _apply_slew(self, msg: DriveControl) -> DriveControl:
        """좌우 속도에 가감속 제한을 건다.

        ⚠ 왜 (2026-08-14): 여기까지 오는 지령은 **계단**이다 — cmd_vel이 0 → 0.25m/s로
           한 번에 바뀌면 모터 속도제어기가 계단 응답을 하고, 그게 전류 피크가 된다.
           실측: 평상시 주행 4.8A인데 **피크 28.3A**(6배), 과전류 플래그(20A) 3회.
           그리고 **고부하 과도구간에서 시스템 프리즈가 재현**됐다([[robot_freeze_safety]]).
           → 지령을 부드럽게 만들어 피크를 낮춘다. 근본 원인 해결은 아니지만,
             프리즈 빈도가 줄면 "전류가 방아쇠"라는 가설의 검증도 된다.

        ⚠ **안전 정지는 절대 늦추지 않는다.** 워치독/estop/idle 경로는 이 함수에
           도달하기 전에 return 하거나 0을 직접 대입하므로 영향이 없고, 여기서도
           **감속은 accel보다 넉넉한 한계**를 쓴다. `decel_limit<=0`이면 감속 무제한.
        """
        if self.accel_limit <= 0.0:
            return msg                      # 0 이하면 기능 자체를 끔
        now = self._steady.now()
        if self._slew_t is None:
            self._slew_t = now
            return msg
        dt = (now - self._slew_t).nanoseconds / 1e9
        self._slew_t = now
        if dt <= 0.0 or dt > 0.5:           # 첫 호출/긴 공백이면 그대로 통과
            return msg

        prev = self.last_drive_msg
        for side in ('left_speed', 'right_speed'):
            tgt = getattr(msg, side)
            cur = getattr(prev, side, 0.0)
            # 속도 크기가 커지면 가속, 작아지면 감속으로 본다(부호 반전 포함)
            speeding_up = abs(tgt) > abs(cur)
            lim = self.accel_limit if speeding_up else self.decel_limit
            if lim <= 0.0:
                continue                    # 감속 무제한
            step = lim * dt
            if tgt - cur > step:
                setattr(msg, side, cur + step)
            elif cur - tgt > step:
                setattr(msg, side, cur - step)
        return msg

    def _input_stale(self):
        """구동 중인 모드의 입력이 끊겼는지 검사.

        끊겼으면 사유 문자열, 정상이면 None을 반환한다.
        주행 명령을 만드는 모드(manual/auto)만 검사한다 — idle/estop 등은
        어차피 정지 명령이라 워치독이 개입할 이유가 없다.
        """
        if self.control_mode == 'manual':
            return self._stale_reason('리모콘', self.last_remote_time,
                                      self.remote_timeout)
        if self.control_mode in ('auto', 'navigating'):
            return self._stale_reason('cmd_vel', self.last_cmd_vel_time,
                                      self.cmd_vel_timeout)
        return None

    def _stale_reason(self, name, last_time, timeout):
        if timeout <= 0.0:          # 0 이하면 워치독 비활성
            return None
        if last_time is None:
            return f'{name} 수신 이력 없음'
        age = (self._steady.now() - last_time).nanoseconds / 1e9
        if age > timeout:
            return f'{name} {age:.2f}s 두절 (임계 {timeout:.2f}s)'
        return None

    def _convert_remote_to_drive(self) -> DriveControl:
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

        # 데드존 + soft ramp 적용
        # 데드존 내: 0, 데드존 밖: 0부터 점진적으로 증가 (경계에서 뚝 끊김 방지)
        deadzone_normalized = self.joystick_deadzone / 127.0
        joy_linear = self._apply_deadzone_ramp(joy_linear, deadzone_normalized)
        joy_angular = self._apply_deadzone_ramp(joy_angular, deadzone_normalized)

        # 조이스틱 값 → 선속도/각속도 변환
        # AN3+: 전진, AN3-: 후진 (방향 반전)
        linear_velocity = joy_linear * self.max_linear

        # AN4+: CCW (왼쪽), AN4-: CW (오른쪽)
        angular_velocity = joy_angular * self.max_angular

        # Differential drive kinematics
        left_speed = linear_velocity - (angular_velocity * self.wheel_base / 2.0)
        right_speed = linear_velocity + (angular_velocity * self.wheel_base / 2.0)

        # 속도 스케일링 적용 (테스트용 속도 제한)
        drive_msg.left_speed = left_speed * self.speed_scale
        drive_msg.right_speed = right_speed * self.speed_scale
        drive_msg.lateral_speed = 0.0

        return drive_msg

    def _convert_cmd_vel_to_drive(self) -> DriveControl:
        """
        cmd_vel (Twist) → DriveControl 변환

        Differential Drive Kinematics:
        - v_left = v_linear - (omega * wheel_base / 2)
        - v_right = v_linear + (omega * wheel_base / 2)
        """
        msg = self.cmd_vel_msg
        drive_msg = DriveControl()

        try:
            # 방향별 차단 2단: 데크끝(나가면 안 되는 방향) + 범퍼(이미 부딪힌 방향).
            # 둘 다 **직진성분만** 0으로 만든다 — 선회와 반대방향은 살려둬야 탈출한다.
            # manual(리모콘)에는 적용하지 않는다: 어떤 경우에도 사람이 빼낼 수 있어야 한다.
            linear_cmd = self._bumper_gate(self._deck_edge_gate(msg.linear.x))
            # 입력 제한 (전후면 반전: cmd_vel 부호 반전)
            linear = -max(-self.max_linear, min(self.max_linear, linear_cmd))
            angular = max(-self.max_angular, min(self.max_angular, msg.angular.z))

            # Differential drive 변환
            left_speed = linear - (angular * self.wheel_base / 2.0)
            right_speed = linear + (angular * self.wheel_base / 2.0)

            # 속도 스케일링 적용 (테스트용 속도 제한)
            drive_msg.left_speed = left_speed * self.speed_scale
            drive_msg.right_speed = right_speed * self.speed_scale
            drive_msg.lateral_speed = 0.0

        except Exception as e:
            self.get_logger().error(f"cmd_vel 변환 오류: {e}")
            drive_msg.left_speed = 0.0
            drive_msg.right_speed = 0.0

        return drive_msg

    @staticmethod
    def _apply_deadzone_ramp(value: float, deadzone: float) -> float:
        """데드존 + soft ramp: 데드존 밖에서 0부터 점진적으로 증가.

        입력 범위 [-1, 1] → 출력 범위 [-1, 1]
        |value| < deadzone → 0
        |value| >= deadzone → 부호 유지, 0 ~ ±1로 리매핑
        """
        if abs(value) < deadzone:
            return 0.0
        sign = 1.0 if value > 0 else -1.0
        # deadzone~1.0 구간을 0~1.0으로 리매핑
        return sign * (abs(value) - deadzone) / (1.0 - deadzone)

    # ========== 속도 측정 관련 메서드 ==========

    def _handle_speed_test(self, drive_msg: DriveControl) -> None:
        """AN3 전진 시 속도 측정 시작/종료 처리"""
        # 전진 여부 판단 (left/right 모두 양수이고 일정 속도 이상)
        is_forward = (drive_msg.left_speed > 0.05 and drive_msg.right_speed > 0.05)

        if is_forward and not self.speed_test_active:
            # 전진 시작 - 속도 측정 시작
            self._start_speed_test()
        elif not is_forward and self.speed_test_active:
            # 전진 종료 - 속도 측정 중단
            self._abort_speed_test()

        # 측정 중 속도 샘플 수집
        if self.speed_test_active:
            avg_speed = (drive_msg.left_speed + drive_msg.right_speed) / 2.0
            self.speed_samples.append(avg_speed)

    def _start_speed_test(self) -> None:
        """속도 측정 시작"""
        if self.current_pose is None:
            self.get_logger().warn("⚠️ 속도 측정 시작 불가 - /robot_pose 수신 안됨")
            return

        self.speed_test_active = True
        self.speed_test_start_time = time.time()
        self.speed_test_start_pose = self.current_pose
        self.speed_samples = []

        start_x = self.current_pose.pose.position.x
        start_y = self.current_pose.pose.position.y

        self.get_logger().info("=" * 60)
        self.get_logger().info("🏁 [속도 측정 시작] AN3 전진 감지")
        self.get_logger().info(f"   시작 위치: ({start_x:.3f}, {start_y:.3f}) m")
        self.get_logger().info(f"   목표 거리: {self.speed_test_target_distance:.1f} m")
        self.get_logger().info("=" * 60)

    def _check_speed_test_progress(self) -> None:
        """속도 측정 진행 상황 체크 (1m 달성 확인)"""
        if not self.speed_test_active or self.current_pose is None:
            return

        start_x = self.speed_test_start_pose.pose.position.x
        start_y = self.speed_test_start_pose.pose.position.y
        curr_x = self.current_pose.pose.position.x
        curr_y = self.current_pose.pose.position.y

        # 이동 거리 계산
        distance = math.sqrt((curr_x - start_x)**2 + (curr_y - start_y)**2)

        # 0.25m 단위로 진행 상황 출력
        if hasattr(self, '_last_reported_distance'):
            if distance - self._last_reported_distance >= 0.25:
                elapsed = time.time() - self.speed_test_start_time
                avg_speed = distance / elapsed if elapsed > 0 else 0
                self.get_logger().info(
                    f"   📍 진행: {distance:.2f}m / {self.speed_test_target_distance:.1f}m "
                    f"(경과: {elapsed:.1f}s, 평균속도: {avg_speed:.2f}m/s)"
                )
                self._last_reported_distance = distance
        else:
            self._last_reported_distance = 0.0

        # 목표 거리 달성
        if distance >= self.speed_test_target_distance:
            self._complete_speed_test(distance)

    def _complete_speed_test(self, distance: float):
        """속도 측정 완료 (1m 달성)"""
        elapsed = time.time() - self.speed_test_start_time
        avg_speed = distance / elapsed if elapsed > 0 else 0

        # 명령 속도 평균
        cmd_avg_speed = sum(self.speed_samples) / len(self.speed_samples) if self.speed_samples else 0

        start_x = self.speed_test_start_pose.pose.position.x
        start_y = self.speed_test_start_pose.pose.position.y
        end_x = self.current_pose.pose.position.x
        end_y = self.current_pose.pose.position.y

        self.get_logger().info("=" * 60)
        self.get_logger().info("🎉 [속도 측정 완료] 1m 달성!")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"   시작 위치: ({start_x:.3f}, {start_y:.3f}) m")
        self.get_logger().info(f"   종료 위치: ({end_x:.3f}, {end_y:.3f}) m")
        self.get_logger().info(f"   이동 거리: {distance:.3f} m")
        self.get_logger().info(f"   소요 시간: {elapsed:.2f} 초")
        self.get_logger().info("-" * 60)
        self.get_logger().info(f"   ⭐ 실제 평균 속도: {avg_speed:.3f} m/s ({avg_speed*100:.1f} cm/s)")
        self.get_logger().info(f"   📊 명령 평균 속도: {cmd_avg_speed:.3f} m/s")
        self.get_logger().info(f"   📈 속도 효율: {(avg_speed/cmd_avg_speed*100):.1f}%" if cmd_avg_speed > 0 else "")
        self.get_logger().info("=" * 60)

        # 측정 종료
        self.speed_test_active = False
        self.speed_test_start_time = None
        self.speed_test_start_pose = None
        self.speed_samples = []
        if hasattr(self, '_last_reported_distance'):
            del self._last_reported_distance

    def _abort_speed_test(self) -> None:
        """속도 측정 중단 (전진 정지)"""
        if self.speed_test_start_pose is not None and self.current_pose is not None:
            start_x = self.speed_test_start_pose.pose.position.x
            start_y = self.speed_test_start_pose.pose.position.y
            curr_x = self.current_pose.pose.position.x
            curr_y = self.current_pose.pose.position.y
            distance = math.sqrt((curr_x - start_x)**2 + (curr_y - start_y)**2)
            elapsed = time.time() - self.speed_test_start_time if self.speed_test_start_time else 0

            self.get_logger().info("=" * 60)
            self.get_logger().info("⏹️ [속도 측정 중단] 전진 정지됨")
            self.get_logger().info(f"   이동 거리: {distance:.3f} m (목표: {self.speed_test_target_distance:.1f} m)")
            self.get_logger().info(f"   소요 시간: {elapsed:.2f} 초")
            if elapsed > 0 and distance > 0:
                avg_speed = distance / elapsed
                self.get_logger().info(f"   평균 속도: {avg_speed:.3f} m/s")
            self.get_logger().info("=" * 60)

        self.speed_test_active = False
        self.speed_test_start_time = None
        self.speed_test_start_pose = None
        self.speed_samples = []
        if hasattr(self, '_last_reported_distance'):
            del self._last_reported_distance


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

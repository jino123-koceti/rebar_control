#!/usr/bin/env python3
"""
Lateral Motion Control Module
횡이동 모터 (0x143) 모션 제어

기능:
- Manual 모드: S17/S18 버튼 → ±360° 회전 (12시 정렬 후)
- Auto 모드: /joint_control_cmd → ±360° 회전 (12시 정렬 후)
- 0x92 멀티턴 엔코더 기반 절대 위치 제어
- 완료 감지 및 신호 발행

로직:
1. 현재 0x92 멀티턴 각도 읽기 (예: 1000°)
2. 가장 가까운 12시(33.50 + n*360) 찾기 (최단 경로, 360° 이내)
3. 가장 가까운 12시로 이동 (절대 위치)
4. 거기서 ±360° 회전 (절대 위치)
"""

from typing import Optional, Callable
import time
from rclpy.node import Node
from rclpy.clock import Clock
from rebar_base_interfaces.msg import JointControl
from std_msgs.msg import String


class LateralMotionController:
    """횡이동 모터 (0x143) 모션 제어 클래스"""

    # Constants
    MOTOR_ID = 0x143
    DEFAULT_HOME_ANGLE = 33.50  # 12시 방향 각도 (0x94 기준)
    DEFAULT_STEP_DEG = 360.0  # 1회전 각도
    DEFAULT_MAX_SPEED = 200.0  # dps
    DEFAULT_ENCODER_CPR = 65536.0  # 0x90 단회전 (16bit)
    DEFAULT_POSITION_TOLERANCE = 1.0  # degrees
    DEFAULT_SETTLING_TIME = 0.5  # 명령 전송 후 완료 체크 대기 시간 (초)

    def __init__(
        self,
        node: Node,
        encoder_request_pub,
        joint_pub,
        lateral_complete_pub,
        home_encoder: int,
        home_angle: float = DEFAULT_HOME_ANGLE,
        step_deg: float = DEFAULT_STEP_DEG,
        max_speed: float = DEFAULT_MAX_SPEED,
        encoder_cpr: float = DEFAULT_ENCODER_CPR,
        position_tolerance: float = DEFAULT_POSITION_TOLERANCE,
        settling_time: float = DEFAULT_SETTLING_TIME,
    ):
        """
        Args:
            node: ROS2 노드 (로깅, 타이머용)
            encoder_request_pub: 엔코더 요청 publisher (/encoder_request)
            joint_pub: 관절 제어 publisher (/joint_control)
            lateral_complete_pub: 횡이동 완료 publisher (/lateral_motion_complete)
            home_encoder: 12시 위치의 0x90 엔코더 값
            home_angle: 12시 방향 각도 (0x94 기준, degrees)
            step_deg: 1회전 각도 (degrees)
            max_speed: 최대 속도 (dps)
            encoder_cpr: 엔코더 CPR (counts per revolution)
            position_tolerance: 허용 오차 (degrees)
            settling_time: 명령 전송 후 완료 체크 대기 시간 (초)
        """
        self._node = node
        self._encoder_request_pub = encoder_request_pub
        self._joint_pub = joint_pub
        self._lateral_complete_pub = lateral_complete_pub
        self._clock: Clock = node.get_clock()
        self._logger = node.get_logger()

        # Parameters
        self._home_encoder = home_encoder
        self._home_angle = home_angle
        self._step_deg = step_deg
        self._max_speed = max_speed
        self._encoder_cpr = encoder_cpr
        self._position_tolerance = position_tolerance
        self._settling_time = settling_time
        self._tolerance_counts = int(encoder_cpr * position_tolerance / 360.0)

        # Current state (외부에서 업데이트)
        self._current_encoder: Optional[int] = None
        self._current_angle_deg: Optional[float] = None  # 0x92 멀티턴
        self._current_angle_94: Optional[float] = None   # 0x94 단일 회전
        self._last_command_time = None

        # Auto 모드 상태
        self._auto_target_encoder: Optional[int] = None
        self._auto_in_progress = False
        self._auto_pending_rotation: Optional[float] = None
        self._auto_pending_velocity: Optional[float] = None
        self._auto_command_time: Optional[float] = None

        self._logger.info(
            f"🚀 LateralMotionController initialized: "
            f"step={step_deg}°, speed={max_speed} dps, home_encoder={home_encoder}"
        )

    @property
    def is_motion_in_progress(self) -> bool:
        """모션 진행 중 여부"""
        return self._auto_in_progress or self._auto_pending_rotation is not None

    def update_encoder(self, encoder_value: int) -> None:
        """0x90 엔코더 값 업데이트"""
        self._current_encoder = encoder_value & 0xFFFF

    def update_angle_deg(self, angle_deg: float) -> None:
        """0x92 멀티턴 각도 업데이트"""
        self._current_angle_deg = angle_deg

    def update_angle_94(self, angle_deg: float) -> None:
        """0x94 단일 회전 각도 업데이트"""
        self._current_angle_94 = angle_deg

    def update_home_encoder(self, home_encoder: int) -> None:
        """홈 엔코더 값 업데이트 (캘리브레이션 완료 시)"""
        self._home_encoder = home_encoder

    def execute_manual_rotation(self, direction: str, command_interval: float = 6.0) -> bool:
        """
        Manual 모드 1회전 (S17/S18)

        Args:
            direction: '+' (시계 방향) or '-' (반시계 방향)
            command_interval: 명령 간격 제한 (초)

        Returns:
            True: 명령 전송됨
            False: 명령 전송 안됨 (간격 제한 또는 데이터 부족)
        """
        # 명령 간격 제한 확인
        if self._last_command_time is not None:
            elapsed = (self._clock.now() - self._last_command_time).nanoseconds / 1e9
            if elapsed < command_interval:
                self._logger.warn(
                    f"⚠️ 명령 간격 부족: {elapsed:.1f}s < {command_interval}s"
                )
                return False

        # 현재 0x92 멀티턴 각도 필요
        if self._current_angle_deg is None:
            self._request_output_angle()
            self._logger.info("📡 0x92 멀티턴 각도 요청 중... (다음 입력에서 재시도)")
            return False

        current_angle = self._current_angle_deg

        # 가장 가까운 12시 찾기 및 이동
        nearest_home, delta_in_circle = self._find_nearest_home(current_angle)

        # 1단계: 12시로 정렬 (tolerance 이내가 아니면)
        if abs(delta_in_circle) > self._position_tolerance:
            self._logger.info(
                f"📍 12시 정렬: {current_angle:.1f}° → {nearest_home:.1f}° "
                f"(delta={delta_in_circle:.1f}°, 360° 이내)"
            )
            self._send_absolute_command(nearest_home, self._max_speed, '12시 정렬 (멀티턴)')
            self._last_command_time = self._clock.now()
            return True

        # 2단계: 12시에서 ±360° 회전
        rotation_deg = self._step_deg if direction == '+' else -self._step_deg
        target_angle = nearest_home + rotation_deg

        self._logger.info(
            f"🔄 12시{direction}{self._step_deg:.0f}° 회전: "
            f"0x92={current_angle:.1f}° → {target_angle:.1f}°, 0x90={self._current_encoder}"
        )

        self._send_absolute_command(target_angle, self._max_speed, f'12시{direction}{self._step_deg:.0f}° 회전')
        self._last_command_time = self._clock.now()
        return True

    def execute_auto_rotation(self, rotation_deg: float, velocity: float) -> bool:
        """
        Auto 모드 횡이동 명령 처리

        Args:
            rotation_deg: 회전 각도 (±360°)
            velocity: 속도 (dps)

        Returns:
            True: 명령 전송됨
            False: 데이터 부족 (재시도 필요)
        """
        # 현재 0x92 멀티턴 각도 필요
        if self._current_angle_deg is None:
            self._request_output_angle()
            self._logger.warn("📡 0x92 멀티턴 각도 요청 중...")
            return False

        current_angle = self._current_angle_deg

        # 가장 가까운 12시 찾기
        nearest_home, delta_in_circle = self._find_nearest_home(current_angle)

        # 1단계: 12시로 정렬 (tolerance 이내가 아니면)
        if abs(delta_in_circle) > self._position_tolerance:
            self._logger.info(
                f"📍 [Auto] 12시 정렬: {current_angle:.1f}° → {nearest_home:.1f}° "
                f"(delta={delta_in_circle:.1f}°)"
            )
            self._send_absolute_command(nearest_home, velocity, '[Auto] 12시 정렬')
            # 대기 중인 회전 저장
            self._auto_pending_rotation = rotation_deg
            self._auto_pending_velocity = velocity
            self._last_command_time = self._clock.now()
            return True

        # 2단계: 12시에서 회전
        target_angle = nearest_home + rotation_deg
        self._start_auto_rotation(target_angle, velocity, rotation_deg)
        return True

    def check_completion(self) -> bool:
        """
        Auto 모드 완료 감지 (주기적으로 호출)

        Returns:
            True: 완료됨
            False: 진행 중
        """
        # 1. 12시 정렬 완료 체크
        if self._auto_pending_rotation is not None:
            return self._check_home_alignment_completion()

        # 2. 360° 회전 완료 체크
        if not self._auto_in_progress:
            return False

        if self._auto_target_encoder is None or self._current_encoder is None:
            return False

        # Settling time 체크
        if self._auto_command_time is not None:
            elapsed = time.monotonic() - self._auto_command_time
            if elapsed < self._settling_time:
                self._logger.debug(
                    f"[Auto] Settling: {elapsed:.2f}s < {self._settling_time}s"
                )
                return False

        # 엔코더 차이 계산 (wrap-around 처리)
        delta = self._current_encoder - self._auto_target_encoder
        if delta > self._encoder_cpr / 2:
            delta -= self._encoder_cpr
        elif delta < -self._encoder_cpr / 2:
            delta += self._encoder_cpr

        # 목표 도달 체크
        if abs(delta) <= self._tolerance_counts:
            self._logger.info(
                f"✅ [Auto] 횡이동 완료! "
                f"(현재={self._current_encoder}, 목표={self._auto_target_encoder}, "
                f"오차={delta} counts ≈ {delta/self._encoder_cpr*360:.1f}°)"
            )

            # 완료 신호 발행
            complete_msg = String()
            complete_msg.data = "COMPLETE"
            self._lateral_complete_pub.publish(complete_msg)

            # 상태 초기화
            self._auto_in_progress = False
            self._auto_target_encoder = None
            self._auto_command_time = None
            return True

        return False

    def _find_nearest_home(self, current_angle: float) -> tuple:
        """
        가장 가까운 12시 위치 찾기

        Args:
            current_angle: 현재 0x92 멀티턴 각도

        Returns:
            (nearest_home, delta_in_circle): 가장 가까운 12시 각도, 현재 위치와의 차이
        """
        remainder = current_angle % 360.0
        delta_in_circle = self._home_angle - remainder

        # 최단 경로 (-180 ~ +180)
        if delta_in_circle > 180.0:
            delta_in_circle -= 360.0
        elif delta_in_circle < -180.0:
            delta_in_circle += 360.0

        nearest_home = current_angle + delta_in_circle
        return nearest_home, delta_in_circle

    def _check_home_alignment_completion(self) -> bool:
        """12시 정렬 완료 체크"""
        if self._current_angle_94 is None:
            self._request_single_circle_angle()
            return False

        # 12시까지 오차 확인
        delta_94 = abs(self._current_angle_94 - self._home_angle)
        if delta_94 > 180.0:
            delta_94 = 360.0 - delta_94

        if delta_94 <= self._position_tolerance:
            # 12시 정렬 완료! → 대기 중인 회전 시작
            rotation_deg = self._auto_pending_rotation
            velocity = self._auto_pending_velocity

            self._logger.info(
                f"✅ [Auto] 12시 정렬 완료 (0x94={self._current_angle_94:.1f}° ≈ {self._home_angle}°)"
            )

            # 대기 상태 초기화
            self._auto_pending_rotation = None
            self._auto_pending_velocity = None

            # 2단계: 360° 회전 시작
            if self._current_angle_deg is not None:
                target_angle = self._current_angle_deg + rotation_deg
                self._start_auto_rotation(target_angle, velocity, rotation_deg)
            else:
                self._request_output_angle()
                self._logger.warn("📡 0x92 각도 요청 중...")

        return False

    def _start_auto_rotation(self, target_angle: float, velocity: float, rotation_deg: float) -> None:
        """Auto 모드 360° 회전 시작"""
        self._auto_target_encoder = self._home_encoder
        self._auto_in_progress = True
        self._auto_command_time = time.monotonic()

        self._logger.info(
            f"🔄 [Auto] 12시{'+' if rotation_deg > 0 else ''}{rotation_deg:.0f}° 회전: "
            f"0x92={self._current_angle_deg:.1f}° → {target_angle:.1f}°"
        )

        self._send_absolute_command(
            target_angle,
            velocity,
            f'[Auto] 12시{rotation_deg:+.0f}° 회전'
        )
        self._last_command_time = self._clock.now()

    def _request_output_angle(self) -> None:
        """0x92 출력축 각도 읽기 요청"""
        self._logger.debug(f"[REQ 0x92] motor:0x{self.MOTOR_ID:03X}")
        msg = JointControl()
        msg.joint_id = self.MOTOR_ID
        msg.position = 0.0
        msg.velocity = 0.0
        msg.control_mode = 0x92
        self._encoder_request_pub.publish(msg)

    def _request_single_circle_angle(self) -> None:
        """0x94 단일 회전 각도 읽기 요청"""
        self._logger.debug(f"[REQ 0x94] motor:0x{self.MOTOR_ID:03X}")
        msg = JointControl()
        msg.joint_id = self.MOTOR_ID
        msg.position = 0.0
        msg.velocity = 0.0
        msg.control_mode = 0x94
        self._encoder_request_pub.publish(msg)

    def _send_absolute_command(self, position_deg: float, velocity: float, name: str) -> None:
        """절대 위치 제어 명령 전송"""
        msg = JointControl()
        msg.joint_id = self.MOTOR_ID
        msg.position = position_deg
        msg.velocity = velocity
        msg.control_mode = JointControl.MODE_ABSOLUTE
        self._joint_pub.publish(msg)

        self._logger.info(
            f'{name}: 0x{self.MOTOR_ID:03X} -> {position_deg:.1f}° @ {velocity:.0f} dps (ABS)'
        )

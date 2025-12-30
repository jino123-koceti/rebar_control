#!/usr/bin/env python3
"""
Lateral Encoder Calibration Module
횡이동 모터 (0x143) 12시 방향 엔코더 캘리브레이션

기능:
- 0x94 (단일 회전 각도) 기반 12시 위치(33.50°) 감지
- 0x90 (싱글턴 엔코더) 홈 값 설정
- 12시 위치로 자동 이동 및 홈 설정

Note:
- 향후 0x144~0x147 (XYZ 스테이지 + Yaw) 캘리브레이션은
  stage_calibration.py로 별도 구현 예정
"""

from typing import Optional, Callable
from rclpy.node import Node
from rclpy.clock import Clock
from rebar_base_interfaces.msg import JointControl


class LateralEncoderCalibration:
    """횡이동 모터 (0x143) 엔코더 캘리브레이션 클래스"""

    # Constants
    MOTOR_ID = 0x143
    DEFAULT_HOME_ANGLE_94 = 33.50  # 12시 방향 각도 (0x94 기준)
    DEFAULT_HOME_ENCODER_90 = 24399  # 홈 엔코더 값 (2025-12-26 캘리브레이션)
    DEFAULT_ENCODER_CPR = 65536.0  # 0x90 단회전 (16bit) → 0~65535 counts
    DEFAULT_POSITION_TOLERANCE = 1.0  # 허용 오차 (degrees)

    def __init__(
        self,
        node: Node,
        encoder_request_pub,
        joint_pub,
        home_angle_94: float = DEFAULT_HOME_ANGLE_94,
        home_encoder_90: int = DEFAULT_HOME_ENCODER_90,
        encoder_cpr: float = DEFAULT_ENCODER_CPR,
        position_tolerance: float = DEFAULT_POSITION_TOLERANCE,
        lateral_speed: float = 200.0,
        on_home_set_callback: Optional[Callable[[int], None]] = None,
    ):
        """
        Args:
            node: ROS2 노드 (로깅, 타이머용)
            encoder_request_pub: 엔코더 요청 publisher (/encoder_request)
            joint_pub: 관절 제어 publisher (/joint_control)
            home_angle_94: 12시 방향 목표 각도 (0x94 기준, degrees)
            home_encoder_90: 12시 위치의 0x90 엔코더 값 (고정값)
            encoder_cpr: 엔코더 CPR (counts per revolution)
            position_tolerance: 허용 오차 (degrees)
            lateral_speed: 횡이동 속도 (dps)
            on_home_set_callback: 홈 설정 완료 시 콜백 (encoder_value)
        """
        self._node = node
        self._encoder_request_pub = encoder_request_pub
        self._joint_pub = joint_pub
        self._clock: Clock = node.get_clock()
        self._logger = node.get_logger()

        # Parameters
        self._home_angle_94_target = home_angle_94
        self._home_encoder_fixed = home_encoder_90
        self._encoder_cpr = encoder_cpr
        self._position_tolerance = position_tolerance
        self._lateral_speed = lateral_speed
        self._on_home_set_callback = on_home_set_callback

        # State
        self._home_encoder: Optional[int] = home_encoder_90  # 고정값으로 초기화
        self._current_encoder: Optional[int] = None
        self._current_angle_94: Optional[float] = None
        self._calibration_complete = False

        self._logger.info(
            f"🏠 LateralEncoderCalibration initialized: "
            f"home_angle={home_angle_94}°, home_encoder={home_encoder_90}"
        )

    @property
    def home_encoder(self) -> Optional[int]:
        """현재 홈 엔코더 값"""
        return self._home_encoder

    @property
    def is_calibrated(self) -> bool:
        """캘리브레이션 완료 여부"""
        return self._home_encoder is not None

    @property
    def home_angle_94_target(self) -> float:
        """12시 목표 각도 (0x94 기준)"""
        return self._home_angle_94_target

    def update_encoder(self, encoder_value: int) -> None:
        """0x90 엔코더 값 업데이트 (MotorFeedback 콜백에서 호출)"""
        self._current_encoder = encoder_value & 0xFFFF

    def update_angle_94(self, angle_deg: float) -> None:
        """0x94 단일 회전 각도 업데이트 (MotorFeedback 콜백에서 호출)"""
        self._current_angle_94 = angle_deg

    def calibrate_once(self) -> bool:
        """
        1회성 캘리브레이션 시도

        Returns:
            True: 캘리브레이션 완료 또는 이미 완료
            False: 아직 완료되지 않음 (재시도 필요)
        """
        # 이미 완료됨
        if self._home_encoder is not None:
            if not self._calibration_complete:
                self._calibration_complete = True
                self._logger.info(
                    f"🏠 Home encoder already set: {self._home_encoder} counts"
                )
            return True

        # 0x94와 0x90 모두 있으면 홈 엔코더 설정
        if self._current_angle_94 is not None and self._current_encoder is not None:
            delta_94 = abs(self._current_angle_94 - self._home_angle_94_target)

            if delta_94 < self._position_tolerance:
                # 현재 위치가 12시 근처 → 홈 설정
                self._home_encoder = self._current_encoder
                self._calibration_complete = True

                self._logger.info(
                    f"🏠 Home encoder set: {self._home_encoder} counts "
                    f"(0x94={self._current_angle_94:.2f}° ≈ {self._home_angle_94_target}°)"
                )
                self._logger.info(
                    f"   ✅ Home position calibrated at 12 o'clock ({self._home_angle_94_target}°)!"
                )

                if self._on_home_set_callback:
                    self._on_home_set_callback(self._home_encoder)

                return True
            else:
                # 현재 위치가 12시가 아님 → 대기
                self._logger.debug(
                    f"📍 Currently at 0x94={self._current_angle_94:.1f}° "
                    f"(target: {self._home_angle_94_target}°), "
                    f"waiting for 12 o'clock to set home encoder..."
                )
                return False

        # 데이터 없음 → 요청
        if self._current_angle_94 is None:
            self._request_angle_94()
        if self._current_encoder is None:
            self._request_encoder_90()

        self._logger.debug(
            "Home calibration pending (waiting for 0x94 and 0x90 data)"
        )
        return False

    def move_to_home(self) -> bool:
        """
        12시 위치(33.50°)로 이동

        Returns:
            True: 이미 12시 위치에 있음
            False: 이동 명령 전송됨 (아직 도착 안함)
        """
        if self._current_angle_94 is None:
            self._request_angle_94()
            return False

        # 현재 0x94와 목표(33.50°)의 차이 계산
        delta_94 = self._current_angle_94 - self._home_angle_94_target

        # 최단 경로 (-180 ~ +180)
        while delta_94 > 180:
            delta_94 -= 360
        while delta_94 < -180:
            delta_94 += 360

        # 오차가 tolerance 이내면 완료
        if abs(delta_94) < self._position_tolerance:
            if self._current_encoder is not None and self._home_encoder is None:
                self._home_encoder = self._current_encoder
                self._calibration_complete = True
                self._logger.info(
                    f"🏠 Home encoder set: {self._home_encoder} counts "
                    f"(0x94={self._current_angle_94:.2f}° ≈ {self._home_angle_94_target}°)"
                )
                if self._on_home_set_callback:
                    self._on_home_set_callback(self._home_encoder)
            return True

        # 12시로 이동 (상대 위치 제어)
        self._logger.info(
            f"🔄 Moving to 12 o'clock ({self._home_angle_94_target}°): "
            f"current={self._current_angle_94:.1f}°, "
            f"delta={delta_94:.1f}°"
        )
        self._send_relative_command(-delta_94)
        return False

    def _request_encoder_90(self) -> None:
        """0x90 엔코더 읽기 요청"""
        self._logger.debug(f"[REQ 0x90] motor:0x{self.MOTOR_ID:03X}")
        msg = JointControl()
        msg.joint_id = self.MOTOR_ID
        msg.position = 0.0
        msg.velocity = 0.0
        msg.control_mode = 0x90
        self._encoder_request_pub.publish(msg)

    def _request_angle_94(self) -> None:
        """0x94 단일 회전 각도 읽기 요청"""
        self._logger.debug(f"[REQ 0x94] motor:0x{self.MOTOR_ID:03X}")
        msg = JointControl()
        msg.joint_id = self.MOTOR_ID
        msg.position = 0.0
        msg.velocity = 0.0
        msg.control_mode = 0x94
        self._encoder_request_pub.publish(msg)

    def _send_relative_command(self, delta_deg: float) -> None:
        """상대 위치 제어 명령 전송"""
        msg = JointControl()
        msg.joint_id = self.MOTOR_ID
        msg.position = delta_deg
        msg.velocity = self._lateral_speed
        msg.control_mode = JointControl.MODE_RELATIVE
        self._joint_pub.publish(msg)

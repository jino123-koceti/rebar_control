#!/usr/bin/env python3
"""
결속 오케스트레이터 노드 (Tying Orchestrator Node)

비전 검출 → 스테이지 XY 이동 → Z축 하강 → 트리거 → Z축 상승
6포인트(2x3) 순회 자동화

트리거:
- 외부 UI: rebar/command → zenoh_client → /mission/command {"command":"TYING_START"}
- 내부: /control_mode가 "tying"으로 전환 시 (호환성 유지)

흐름:
1. TYING_START 명령 수신 시 시작
2. /rebar/detect_crossings 서비스로 교차점 검출 (양쪽 카메라)
3. 범위 필터링 (X<0, Y<0, X>max, Y>max 제거)
4. Z축 오프셋 이동 (홈→대기위치)
5. Right cam (yaw -3°): P1(Xmax)→P2→P3(Xmin) [X 내림차순]
   각 포인트: XY이동 → Z 50mm하강 → 트리거 → Z 50mm상승
6. 자세 변경: X:홈,Y:near → Yaw:mid → Y:far → Yaw:left_cam
7. Left cam (yaw 368°): P4(Xmax)→P5→P6(Xmin) [X 내림차순]
8. 자세 복귀: X:홈,Y:far → Yaw:mid → Y:near → Yaw:right_cam
9. Z축 홈 복귀 (대기위치→홈)
10. 스테이지 원점 복귀

상태 피드백:
- /tying/status (JSON) → rebar_publisher → /mission/status → zenoh → rebar/status → UI
- 필드: tying_state, tying_progress, tying_message, tying_result
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rebar_base_interfaces.msg import JointControl, MotorFeedback
from rebar_base_interfaces.srv import DetectCrossings
import logging
import os
from datetime import datetime
from std_msgs.msg import String, Float32, Bool
import json
import time
import numpy as np
from rebar_vision.tying_common import ActionType, Action, TyingState
from rebar_vision.z_torque_monitor import ZTorqueMonitor
from rebar_vision.action_builder import ActionBuilder
from rebar_vision.detection_manager import DetectionManager


class TyingOrchestratorNode(Node):
    """비전 기반 6포인트 자동 결속 오케스트레이터"""

    def __init__(self):
        super().__init__('tying_orchestrator')

        # 파일 로거 설정
        self._setup_file_logger()

        # ============================================
        # 파라미터 선언
        # ============================================
        self.declare_parameter('deg_per_mm_x', 4.497)
        self.declare_parameter('deg_per_mm_y', 4.462)
        self.declare_parameter('stage_max_speed_dps', 200.0)
        self.declare_parameter('stage_settling_time', 1.5)
        self.declare_parameter('max_stage_timeout', 10.0)
        self.declare_parameter('position_tolerance_deg', 1.0)
        self.declare_parameter('max_stage_x_mm', 411.0)
        self.declare_parameter('max_stage_y_mm', 288.0)
        self.declare_parameter('max_yaw_deg', 399.0)
        # Y축 자세별 가동 범위 (2026-03-18 실측)
        self.declare_parameter('right_cam_y_min_mm', 0.0)
        self.declare_parameter('right_cam_y_max_mm', 142.0)
        self.declare_parameter('left_cam_y_min_mm', 124.0)
        self.declare_parameter('left_cam_y_max_mm', 288.0)
        self.declare_parameter('inter_point_delay', 0.5)
        self.declare_parameter('detection_retry_count', 3)
        self.declare_parameter('detection_retry_delay', 1.0)
        # Yaw 자세 변경 파라미터 (홈 기준 오프셋)
        self.declare_parameter('yaw_right_cam_offset', -3.0)   # 홈 기준 -3°
        self.declare_parameter('yaw_left_cam_offset', 393.0)    # 홈 기준 +393° (최종 작업자세)
        self.declare_parameter('yaw_left_cam_approach_offset', 390.0)  # 홈 기준 +390° (접근자세)
        self.declare_parameter('yaw_mid_offset', 173.0)        # 홈 기준 +173°
        self.declare_parameter('pose_change_y_near_mm', 100.0)
        self.declare_parameter('pose_change_y_far_mm', 250.0)
        self.declare_parameter('yaw_settling_time', 2.0)
        # Z축 결속 파라미터
        self.declare_parameter('z_home_offset_mm', 100.0)      # 호밍 후 Z 대기위치 (홈에서 하강)
        self.declare_parameter('tying_z_down_mm', 50.0)        # 결속 시 추가 Z 하강 (대기위치에서)
        self.declare_parameter('deg_per_mm_z', 11.0)           # Z축: 1mm = 11.0° (실측 보정 필요)
        self.declare_parameter('z_speed_dps', 200.0)           # Z축 속도 (degree/s)
        self.declare_parameter('z_settle_margin', 1.0)         # Z축 이동 후 추가 대기 (초)
        self.declare_parameter('z_early_xy_mm', 120.0)         # Z상승 중 이 높이 오르면 XY 선행 시작 (mm)
        # Z축 하강 토크 모니터링 파라미터
        self.declare_parameter('z_torque_base_mA', 500)        # 토크 임계값 기본값 (mA)
        self.declare_parameter('z_torque_scale_mA', 30)        # 속도 1%당 추가 임계값 (mA)
        self.declare_parameter('z_torque_delay_ratio', 0.4)      # 하강시간 대비 체크 지연 비율
        self.declare_parameter('z_torque_poll_hz', 20.0)       # 0x9C 폴링 주기 (Hz)
        self.declare_parameter('z_torque_decel_mA', 1200)      # 감속 구간 전류 임계값 (mA)
        self.declare_parameter('z_torque_decel_ratio', 0.65)   # 감속 구간 시작 비율 (z_wait_time 대비)
        self.declare_parameter('z_torque_decel_count', 2)      # 연속 초과 횟수 (오탐 방지)
        # 트리거 파라미터
        self.declare_parameter('trigger_speed', 1.0)           # 트리거 모터 속도 (-1.0 ~ 1.0)
        self.declare_parameter('trigger_duration', 1.0)        # 트리거 동작 시간 (초)
        # 멀티검출 파라미터
        self.declare_parameter('multi_detect_tries', 5)        # 다중 검출 최대 시도 횟수
        self.declare_parameter('multi_detect_delay', 0.3)      # 검출 간 대기 시간 (초)
        self.declare_parameter('cluster_distance_mm', 40.0)    # 클러스터링 거리 임계값 (mm)
        self.declare_parameter('expected_points_per_cam', 3)   # 카메라당 기대 포인트 수
        self.declare_parameter('detection_confidence', 0.3)    # 검출 confidence 임계값
        self.declare_parameter('rebar_spacing_mm', 195.0)      # 철근 배근 간격 (mm)

        # 파라미터 로드
        self.deg_per_mm_x = self.get_parameter('deg_per_mm_x').value
        self.deg_per_mm_y = self.get_parameter('deg_per_mm_y').value
        self.stage_max_speed = self.get_parameter('stage_max_speed_dps').value
        self.stage_settling_time = self.get_parameter('stage_settling_time').value
        self.max_stage_timeout = self.get_parameter('max_stage_timeout').value
        self.position_tolerance = self.get_parameter('position_tolerance_deg').value
        self.max_stage_x_mm = self.get_parameter('max_stage_x_mm').value
        self.max_stage_y_mm = self.get_parameter('max_stage_y_mm').value
        self.max_yaw_deg = self.get_parameter('max_yaw_deg').value
        self.right_cam_y_min = self.get_parameter('right_cam_y_min_mm').value
        self.right_cam_y_max = self.get_parameter('right_cam_y_max_mm').value
        self.left_cam_y_min = self.get_parameter('left_cam_y_min_mm').value
        self.left_cam_y_max = self.get_parameter('left_cam_y_max_mm').value
        self.inter_point_delay = self.get_parameter('inter_point_delay').value
        self.detection_retry_count = self.get_parameter('detection_retry_count').value
        self.detection_retry_delay = self.get_parameter('detection_retry_delay').value
        self.yaw_right_cam_offset = self.get_parameter('yaw_right_cam_offset').value
        self.yaw_left_cam_offset = self.get_parameter('yaw_left_cam_offset').value
        self.yaw_left_cam_approach_offset = self.get_parameter('yaw_left_cam_approach_offset').value
        self.yaw_mid_offset = self.get_parameter('yaw_mid_offset').value
        self.pose_change_y_near = self.get_parameter('pose_change_y_near_mm').value
        self.pose_change_y_far = self.get_parameter('pose_change_y_far_mm').value
        self.yaw_settling_time = self.get_parameter('yaw_settling_time').value
        self.z_home_offset_mm = self.get_parameter('z_home_offset_mm').value
        self.tying_z_down_mm = self.get_parameter('tying_z_down_mm').value
        self.deg_per_mm_z = self.get_parameter('deg_per_mm_z').value
        self.z_speed = self.get_parameter('z_speed_dps').value
        self.z_settle_margin = self.get_parameter('z_settle_margin').value
        self.z_early_xy_mm = self.get_parameter('z_early_xy_mm').value
        self.z_torque_base_mA = self.get_parameter('z_torque_base_mA').value
        self.z_torque_scale_mA = self.get_parameter('z_torque_scale_mA').value
        self.z_torque_delay_ratio = self.get_parameter('z_torque_delay_ratio').value
        self.z_torque_poll_hz = self.get_parameter('z_torque_poll_hz').value
        self.z_torque_decel_mA = self.get_parameter('z_torque_decel_mA').value
        self.z_torque_decel_ratio = self.get_parameter('z_torque_decel_ratio').value
        self.z_torque_decel_count = self.get_parameter('z_torque_decel_count').value
        self.trigger_speed_val = self.get_parameter('trigger_speed').value
        self.trigger_duration = self.get_parameter('trigger_duration').value
        self.multi_detect_tries = self.get_parameter('multi_detect_tries').value
        self.multi_detect_delay = self.get_parameter('multi_detect_delay').value
        self.cluster_distance_mm = self.get_parameter('cluster_distance_mm').value
        self.expected_points_per_cam = self.get_parameter('expected_points_per_cam').value
        self.detection_confidence = self.get_parameter('detection_confidence').value
        self.rebar_spacing_mm = self.get_parameter('rebar_spacing_mm').value

        # ============================================
        # 상태 변수
        # ============================================
        self.state = TyingState.IDLE
        self.state_start_time = None
        self.control_mode = 'idle'
        self.prev_control_mode = 'idle'
        self.paused_from_state = None  # PAUSE 전 상태 저장

        # 작업 속도 (0~100%, 기본 50%)
        self.speed_percent = 50.0
        self.effective_stage_speed = self.stage_max_speed * 0.5
        self.effective_z_speed = self.z_speed * 0.5

        # 액션 큐
        self.action_queue: List[Action] = []
        self.current_action_index = 0
        self.current_settle_time = 0.0
        self.total_points = 0
        self.completed_points = 0  # 결속 완료 포인트 수 (진행률 계산)

        # 모터 현재 위치 추적 (goal 도달 판단용)
        self.motor_positions = {}  # {motor_id: current_deg}
        self.goal_targets = {}     # {motor_id: target_deg} - 현재 이동 목표

        # 호밍 레퍼런스 (홈 위치의 모터 절대각도)
        self.home_ref_x_deg = None  # 호밍 완료 시 설정됨
        self.home_ref_y_deg = None
        self.home_ref_yaw_deg = None

        # Z축 대기 시간 (이동 거리 기반 계산)
        self.z_wait_time = 0.0

        # 액션 큐 생성 모듈 (ROS 의존성 없음, 먼저 초기화 가능)
        self.action_builder = ActionBuilder({
            'tying_z_down_mm': self.tying_z_down_mm,
            'x_home_mm': 0.0,  # _start_detection에서 업데이트
            'pose_change_y_near': self.pose_change_y_near,
            'pose_change_y_far': self.pose_change_y_far,
            'yaw_right_cam': 0.0,  # _start_detection에서 업데이트
            'yaw_left_cam': 0.0,
            'yaw_left_cam_approach': 0.0,
            'yaw_mid': 0.0,
            'home_ref_yaw_deg': 0.0,
        })

        # 트리거 상태
        self.trigger_phase = 0           # 0=fire, 1=wait_fire, 2=return, 3=wait_return
        self.trigger_phase_start = 0.0

        # 검출 상태 (멀티검출)
        # 검출 상태는 DetectionManager가 관리

        # 2단계 검출/결속
        self.tying_phase = 1             # 1=현재자세, 2=자세변경 후
        self._phase2_cam = None          # 자세변경 완료 후 검출할 카메라

        # 반복 모드
        self.repeat_mode = False          # TYING_START repeat=ON 시 True
        self.tying_pass = 'normal'        # 'normal', 'forward', 'return'
        self.tying_direction = 'forward'  # 'forward' (우→좌), 'reverse' (좌→우)

        # 자세 상태 추적: 'right', 'left', 'unknown'
        # 호밍 완료 시 'right'으로 초기화, 자세변경/복귀 수행 시 업데이트
        self.current_pose_state = 'unknown'

        # 피드백 메시지 (UI 표시용)
        self.tying_message = ''
        self.tying_result = ''

        # ============================================
        # ROS 인터페이스
        # ============================================
        self.cb_group = ReentrantCallbackGroup()

        self.detect_client = self.create_client(
            DetectCrossings, '/rebar/detect_crossings',
            callback_group=self.cb_group
        )

        # 외부 UI 명령 수신 (Zenoh → zenoh_client → /mission/command)
        self.mission_cmd_sub = self.create_subscription(
            String, '/mission/command',
            self._mission_command_cb, 10
        )

        # 내부 제어 모드 (호환성 유지)
        self.control_mode_sub = self.create_subscription(
            String, '/control_mode',
            self._control_mode_cb, 10
        )

        self.motor_feedback_sub = self.create_subscription(
            MotorFeedback, '/motor_feedback',
            self._motor_feedback_cb, 10
        )

        # 호밍 완료 시 레퍼런스 수신
        self.homing_status_sub = self.create_subscription(
            String, '/homing_status',
            self._homing_status_cb, 10
        )

        # 장애물 PAUSE 구독
        self.obstacle_pause_sub = self.create_subscription(
            Bool, '/obstacle_pause',
            self._obstacle_pause_cb, 10
        )

        self.joint_pub = self.create_publisher(
            JointControl, '/joint_control', 10
        )

        # 0x9C 모터 상태 요청용 (Z축 토크 폴링)
        self.encoder_request_pub = self.create_publisher(
            JointControl, '/encoder_request', 10
        )

        self.motion_cmd_pub = self.create_publisher(
            String, '/rebar_motion_cmd', 10
        )

        # 트리거 모터 (Pololu, /motor_0/vel)
        self.trigger_pub = self.create_publisher(
            Float32, '/motor_0/vel', 10
        )

        # 결속 상태 피드백 (JSON) → rebar_publisher가 /mission/status에 병합 → UI
        self.status_pub = self.create_publisher(
            String, '/tying/status', 10
        )

        # 상태 머신 타이머 (20Hz)
        self.timer = self.create_timer(0.05, self._state_machine_loop)

        # Z축 토크 모니터링 모듈 (퍼블리셔 생성 이후 초기화)
        self.z_monitor = ZTorqueMonitor(self, {
            'z_torque_base_mA': self.z_torque_base_mA,
            'z_torque_scale_mA': self.z_torque_scale_mA,
            'z_torque_delay_ratio': self.z_torque_delay_ratio,
            'z_torque_poll_hz': self.z_torque_poll_hz,
            'z_torque_decel_mA': self.z_torque_decel_mA,
            'z_torque_decel_ratio': self.z_torque_decel_ratio,
            'z_torque_decel_count': self.z_torque_decel_count,
        }, self.encoder_request_pub, self.joint_pub)

        # 검출 관리 모듈 (서비스 클라이언트 생성 이후 초기화)
        self.detection_mgr = DetectionManager(self, self.detect_client, {
            'multi_detect_tries': self.multi_detect_tries,
            'multi_detect_delay': self.multi_detect_delay,
            'detection_confidence': self.detection_confidence,
            'expected_points_per_cam': self.expected_points_per_cam,
            'cluster_distance_mm': self.cluster_distance_mm,
            'rebar_spacing_mm': self.rebar_spacing_mm,
            'max_stage_x_mm': self.max_stage_x_mm,
            'max_stage_y_mm': self.max_stage_y_mm,
            'right_cam_y_min': self.right_cam_y_min,
            'right_cam_y_max': self.right_cam_y_max,
            'left_cam_y_min': self.left_cam_y_min,
            'left_cam_y_max': self.left_cam_y_max,
        })

        self.get_logger().info('=' * 60)
        self.get_logger().info('Tying Orchestrator 초기화')
        self.get_logger().info(
            f'  X: {self.deg_per_mm_x} deg/mm, max {self.max_stage_x_mm}mm')
        self.get_logger().info(
            f'  Y: {self.deg_per_mm_y} deg/mm, max {self.max_stage_y_mm}mm')
        self.get_logger().info(
            f'  Z: {self.deg_per_mm_z} deg/mm, offset={self.z_home_offset_mm}mm, '
            f'tying={self.tying_z_down_mm}mm, speed={self.z_speed}dps')
        self.get_logger().info(
            f'  Trigger: speed={self.trigger_speed_val}, '
            f'duration={self.trigger_duration}s')
        self.get_logger().info(
            f'  Yaw offset: right={self.yaw_right_cam_offset}°, '
            f'left={self.yaw_left_cam_offset}°(접근:{self.yaw_left_cam_approach_offset}°), '
            f'mid={self.yaw_mid_offset}°')
        self.get_logger().info(
            f'  Pose Y: near={self.pose_change_y_near}mm, '
            f'far={self.pose_change_y_far}mm')
        self.get_logger().info(
            f'  Speed: {self.stage_max_speed} dps, '
            f'settle: {self.stage_settling_time}s, '
            f'yaw settle: {self.yaw_settling_time}s')
        self.get_logger().info(
            f'  Multi-detect: tries={self.multi_detect_tries}, '
            f'cluster={self.cluster_distance_mm}mm, '
            f'expected/cam={self.expected_points_per_cam}, '
            f'conf={self.detection_confidence}')
        self.get_logger().info('=' * 60)
        self.flog("=== Tying Orchestrator 시작 ===")

    def _setup_file_logger(self):
        """파일 로거 설정 (/tmp/tying_orchestrator_YYYYMMDD_HHMMSS.log)"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._log_path = f'/tmp/tying_orchestrator_{timestamp}.log'
        self._file_logger = logging.getLogger('tying_orchestrator_file')
        self._file_logger.setLevel(logging.DEBUG)
        self._file_logger.handlers.clear()
        fh = logging.FileHandler(self._log_path, mode='a', encoding='utf-8')
        fh.setFormatter(logging.Formatter('%(asctime)s.%(msecs)03d %(message)s', datefmt='%H:%M:%S'))
        self._file_logger.addHandler(fh)
        self._file_handler = fh
        self.get_logger().info(f"📝 파일 로그: {self._log_path}")

    def flog(self, msg):
        """파일 로거에 기록 + flush"""
        self._file_logger.info(msg)
        self._file_handler.flush()

    # ============================================
    # 콜백
    # ============================================
    def _obstacle_pause_cb(self, msg: Bool):
        """장애물 감지 → 결속 일시정지/재개"""
        if msg.data:
            # PAUSE
            if self.state in (TyingState.EXECUTING_ACTION,
                              TyingState.WAITING_SETTLE,
                              TyingState.WAITING_Z,
                              TyingState.WAITING_Z_XY,
                              TyingState.WAITING_TRIGGER):
                self.get_logger().warn('장애물 감지 → 결속 PAUSE')
                self.paused_from_state = self.state
                self._transition_to(TyingState.PAUSED)
                self.tying_message = '장애물 감지 - 일시 정지'
                self._publish_tying_feedback()
        else:
            # RESUME
            if self.state == TyingState.PAUSED and self.paused_from_state is not None:
                self.get_logger().info('장애물 해제 → 결속 RESUME')
                resume_state = self.paused_from_state or TyingState.EXECUTING_ACTION
                self.paused_from_state = None
                self.tying_message = '재개'
                self._transition_to(resume_state)
                self._publish_tying_feedback()

    def _mission_command_cb(self, msg: String):
        """외부 UI 명령 처리 (/mission/command ← Zenoh rebar/command)

        지원 명령:
        - {"command": "TYING_START"} → 결속 시작
        - {"command": "PAUSE"}       → 일시 정지
        - {"command": "RESUME"}      → 재개
        - {"command": "CANCEL"}      → 작업 중단
        - {"command": "EMERGENCY_STOP"} → 비상 정지
        """
        command = msg.data
        self.flog(f"CMD 수신: {command[:80]}")
        if not command.startswith('{'):
            return  # JSON이 아닌 명령은 무시

        try:
            data = json.loads(command)
        except json.JSONDecodeError:
            return

        cmd = data.get('command', '')

        if cmd == 'TYING_START':
            if self.state == TyingState.IDLE:
                # speed 파라미터: 0~100 (기본 50, 설정 dps의 백분율)
                speed_pct = data.get('speed', 50)
                speed_pct = max(1, min(100, int(speed_pct)))
                self.speed_percent = float(speed_pct)
                self.effective_stage_speed = (
                    self.stage_max_speed * self.speed_percent / 100.0)
                self.effective_z_speed = (
                    self.z_speed * self.speed_percent / 100.0)
                # repeat 파라미터: "ON"/"OFF" (기본 OFF)
                repeat_str = str(data.get('repeat', 'OFF')).upper()
                self.repeat_mode = (repeat_str == 'ON')
                self.tying_pass = 'forward' if self.repeat_mode else 'normal'
                # direction 파라미터: "forward" (우→좌, 기본) / "reverse" (좌→우)
                self.tying_direction = data.get('direction', 'forward')
                self.get_logger().info('=' * 60)
                self.get_logger().info(
                    f'[UI] TYING_START 수신 - 속도 {speed_pct}% '
                    f'(XY:{self.effective_stage_speed:.0f}dps, '
                    f'Z:{self.effective_z_speed:.0f}dps)'
                    f' [방향: {self.tying_direction}]'
                    f'{" [반복모드]" if self.repeat_mode else ""}')
                self.get_logger().info('=' * 60)
                self.flog(f"TYING_START: speed={speed_pct}% direction={self.tying_direction}")
                self._start_detection()
            else:
                self.get_logger().warn(
                    f'TYING_START 무시: 현재 상태 {self.state.name}')

        elif cmd == 'PAUSE':
            if self.state in (TyingState.EXECUTING_ACTION,
                              TyingState.WAITING_SETTLE,
                              TyingState.WAITING_Z,
                              TyingState.WAITING_Z_XY,
                              TyingState.WAITING_TRIGGER):
                self.get_logger().info('[UI] PAUSE 수신 - 일시 정지')
                self.paused_from_state = self.state
                self._transition_to(TyingState.PAUSED)
                self.tying_message = '일시 정지됨'
                self._publish_tying_feedback()

        elif cmd == 'RESUME':
            if self.state == TyingState.PAUSED:
                self.get_logger().info('[UI] RESUME 수신 - 재개')
                resume_state = self.paused_from_state or TyingState.EXECUTING_ACTION
                self.paused_from_state = None
                self.tying_message = '재개'
                self._transition_to(resume_state)
                self._publish_tying_feedback()

        elif cmd in ('CANCEL', 'EMERGENCY_STOP'):
            if self.state != TyingState.IDLE:
                self.get_logger().warn(f'[UI] {cmd} 수신 - 작업 중단')
                self._handle_cancel()

        elif cmd.startswith('CHN_POS='):
            target_pose = cmd.split('=')[1].lower()
            if self.state != TyingState.IDLE:
                self.get_logger().warn(
                    f'CHN_POS 무시: 현재 상태 {self.state.name}')
                return
            if self.home_ref_x_deg is None:
                self.get_logger().error(
                    'CHN_POS 실패: 호밍 레퍼런스 미수신 (호밍 먼저 수행 필요)')
                self.flog(f"CHN_POS={target_pose} 실패: 호밍 미수행")
                return
            if target_pose == self.current_pose_state:
                self.get_logger().info(
                    f'CHN_POS: 이미 {target_pose} 자세 → 무시')
                self.flog(f"CHN_POS={target_pose} 이미 해당 자세")
                return
            # 속도 설정 (기본 30%)
            speed_pct = data.get('speed', 30)
            speed_pct = max(1, min(100, int(speed_pct)))
            self.speed_percent = float(speed_pct)
            self.effective_stage_speed = (
                self.stage_max_speed * self.speed_percent / 100.0)
            # Yaw 절대값 계산
            self.yaw_right_cam = self.home_ref_yaw_deg + self.yaw_right_cam_offset
            self.yaw_left_cam = self.home_ref_yaw_deg + self.yaw_left_cam_offset
            self.yaw_left_cam_approach = self.home_ref_yaw_deg + self.yaw_left_cam_approach_offset
            self.yaw_mid = self.home_ref_yaw_deg + self.yaw_mid_offset
            # 홈 위치 초기화
            self.x_home_mm = 0.0
            self.y_home_mm = 0.0
            # 액션 큐 생성
            self.action_queue = []
            self.current_action_index = 0
            self.completed_points = 0
            self.total_points = 0
            if target_pose == 'left':
                self._append_pose_change_right_to_left()
                self.current_pose_state = 'left'
            else:
                self._append_pose_change_left_to_right()
                self.current_pose_state = 'right'
            self.get_logger().info(
                f'[UI] CHN_POS={target_pose} 수신 - '
                f'{len(self.action_queue)}개 액션 생성')
            self.flog(f"CHN_POS={target_pose}: {len(self.action_queue)}개 액션")
            self._transition_to(TyingState.EXECUTING_ACTION)

    def _control_mode_cb(self, msg: String):
        """내부 제어 모드 변경 → tying 시 자동 시작 (호환성)"""
        self.prev_control_mode = self.control_mode
        self.control_mode = msg.data

        if (self.control_mode == 'tying'
                and self.prev_control_mode != 'tying'
                and self.state == TyingState.IDLE):
            # 기본 속도 100%, 반복 OFF
            self.speed_percent = 100.0
            self.effective_stage_speed = self.stage_max_speed * 1.0
            self.effective_z_speed = self.z_speed * 1.0
            self.repeat_mode = False
            self.tying_pass = 'normal'
            self.get_logger().info('=' * 60)
            self.get_logger().info(
                f'[AUTO] Tying 모드 진입 - 속도 100% '
                f'(XY:{self.effective_stage_speed:.0f}dps)')
            self.get_logger().info('=' * 60)
            self._start_detection()

    def _homing_status_cb(self, msg: String):
        """호밍 완료 시 레퍼런스 각도 저장 + Z축 오프셋 자동 하강

        메시지 형식: 'COMPLETE:{"x": 1629.5, "y": -22.6, "z": ..., "yaw": ...}'
        """
        if msg.data.startswith('COMPLETE:'):
            try:
                ref = json.loads(msg.data[len('COMPLETE:'):])
                self.home_ref_x_deg = ref.get('x', 0.0)
                self.home_ref_y_deg = ref.get('y', 0.0)
                self.home_ref_yaw_deg = ref.get('yaw', 0.0)
                self.get_logger().info(
                    f'호밍 레퍼런스 수신: X={self.home_ref_x_deg:.1f}°, '
                    f'Y={self.home_ref_y_deg:.1f}°, '
                    f'Yaw={self.home_ref_yaw_deg:.1f}°')

                # 호밍 완료 시 자세 상태 초기화 (Right 카메라 자세)
                self.current_pose_state = 'right'
                self.get_logger().info(
                    f'자세 상태 초기화: {self.current_pose_state}')

                # Z축 오프셋 자동 하강 (대기위치로 이동)
                z_offset_deg = self.z_home_offset_mm * self.deg_per_mm_z
                self._send_z_relative(z_offset_deg)
                self.get_logger().info(
                    f'Z축 오프셋 하강: {self.z_home_offset_mm}mm '
                    f'({z_offset_deg:.1f}°)')
            except Exception as e:
                self.get_logger().warn(f'호밍 레퍼런스 파싱 실패: {e}')

    def _motor_feedback_cb(self, msg: MotorFeedback):
        """모터 피드백으로 현재 위치 추적 (goal 도달 판단용)

        motor_id: 0x44(X), 0x45(Y), 0x47(Yaw)
        status == 0x92 (멀티턴 절대각도) 응답만 사용
        0xA2/0xA4 응답은 싱글턴(0~360°)이라 멀티턴 목표와 비교 불가
        """
        if msg.motor_id in (0x44, 0x45, 0x47) and msg.status == 0x92:
            self.motor_positions[msg.motor_id] = msg.current_position
        # Z축 (0x46) 0x9C 응답 → 토크 모니터링 모듈에 전달
        if msg.motor_id == 0x46 and msg.status == 0x9C:
            self.z_monitor.update_feedback(msg.current_current, msg.current_speed)

    # ============================================
    # 상태 머신
    # ============================================
    def _state_machine_loop(self):
        """20Hz 상태 머신 메인 루프"""
        elapsed = self._elapsed()

        if self.state == TyingState.IDLE:
            return

        elif self.state == TyingState.DETECTING:
            self._handle_detecting()

        elif self.state == TyingState.EXECUTING_ACTION:
            self._execute_current_action()

        elif self.state == TyingState.WAITING_SETTLE:
            if elapsed < 0.3:
                return  # 최소 대기 (명령 전파)
            if self._check_goal_reached():
                pos_str = ', '.join(
                    f'0x{mid:02X}={self.motor_positions.get(mid, 0):.1f}°'
                    for mid in self.goal_targets)
                self.get_logger().info(
                    f'    목표 도달 ({elapsed:.1f}s) [{pos_str}]')
                self._advance_to_next_action()
            elif elapsed >= self.max_stage_timeout:
                pos_str = ', '.join(
                    f'0x{mid:02X}={self.motor_positions.get(mid, "?")}'
                    f'→{tgt:.1f}°'
                    for mid, tgt in self.goal_targets.items())
                self.get_logger().warn(
                    f'    이동 타임아웃 ({self.max_stage_timeout}s) [{pos_str}]')
                self._advance_to_next_action()

        elif self.state == TyingState.WAITING_Z:
            # Z 하강 중 토크 과부하 감지
            overload = self.z_monitor.check_overload(elapsed)
            if overload:
                self.z_monitor.log_summary(overload)
                self.z_monitor.stop()
                self._handle_z_overload()
                return
            if elapsed >= self.z_wait_time:
                self.z_monitor.log_summary('OK')
                self.z_monitor.stop()
                self.get_logger().info(
                    f'    Z축 이동 완료 ({elapsed:.1f}s)')
                self._advance_to_next_action()

        elif self.state == TyingState.WAITING_Z_XY:
            # Z상승 중 조기 XY 시작
            if not self.z_xy_started and elapsed >= self.z_early_xy_time:
                action = self.action_queue[self.current_action_index]
                self._send_xy_absolute(action.x_mm, action.y_mm)
                self.z_xy_started = True
                self.get_logger().info(
                    f'    Z↑ {self.z_early_xy_mm}mm 도달 ({elapsed:.1f}s) → XY 선행 이동 시작')
            # Z 완료 + XY 완료 모두 확인
            z_done = (elapsed >= self.z_wait_time)
            xy_done = self._check_goal_reached() if self.z_xy_started else False
            if z_done and xy_done:
                self.get_logger().info(
                    f'    Z↑+XY 완료 ({elapsed:.1f}s)')
                self._advance_to_next_action()
            elif z_done and not xy_done and elapsed > self.max_stage_timeout:
                self.get_logger().warn(
                    f'    Z↑+XY: XY 타임아웃 ({elapsed:.1f}s)')
                self._advance_to_next_action()

        elif self.state == TyingState.WAITING_TRIGGER:
            self._handle_trigger_phase()

        elif self.state == TyingState.PAUSED:
            return  # RESUME 명령 대기

        elif self.state == TyingState.COMPLETE:
            self._handle_complete()

        elif self.state == TyingState.ERROR:
            if elapsed >= 3.0:
                self._transition_to(TyingState.IDLE)
                self._publish_tying_feedback()

    # ============================================
    # 검출 처리
    # ============================================
    def _start_detection(self):
        """검출 시작 (상태 초기화)

        2단계 검출/결속:
          tying_phase=1: 현재 자세 카메라로 검출 → 결속
          tying_phase=2: 자세변경 후 다른 카메라로 검출 → 결속
        """
        if self.home_ref_x_deg is None:
            self.get_logger().error(
                '호밍 레퍼런스 미수신 - TYING_START 불가 (호밍 먼저 수행 필요)')
            return
        # 홈 위치 = 상대좌표 0mm (home_ref 기준)
        self.x_home_mm = 0.0
        self.y_home_mm = 0.0
        self.get_logger().info(
            f'홈 위치: X={self.x_home_mm:.1f}mm, Y={self.y_home_mm:.1f}mm')
        # Yaw 절대값 계산 (홈 레퍼런스 + 오프셋)
        self.yaw_right_cam = self.home_ref_yaw_deg + self.yaw_right_cam_offset
        self.yaw_left_cam = self.home_ref_yaw_deg + self.yaw_left_cam_offset
        self.yaw_left_cam_approach = self.home_ref_yaw_deg + self.yaw_left_cam_approach_offset
        self.yaw_mid = self.home_ref_yaw_deg + self.yaw_mid_offset
        self.get_logger().info(
            f'Yaw 절대값: right={self.yaw_right_cam:.1f}°, '
            f'mid={self.yaw_mid:.1f}°, left={self.yaw_left_cam:.1f}° '
            f'(접근: {self.yaw_left_cam_approach:.1f}°)')

        self.action_queue = []
        self.current_action_index = 0
        self.total_points = 0
        self.completed_points = 0
        self.tying_message = '교차점 멀티검출 중...'
        self.tying_result = ''

        # 2단계 검출: 현재 자세에 맞는 카메라만 검출
        self.tying_phase = 1
        if self.current_pose_state == 'left':
            cam_side = 'left'
        else:
            cam_side = 'right'
        self.get_logger().info(
            f'[Phase 1] {cam_side} 카메라 검출 시작 (현재 자세: {self.current_pose_state})')
        self.flog(f"PHASE1_START: pose={self.current_pose_state} cam={cam_side}")
        self.detection_mgr.start(camera_side=cam_side)
        self._transition_to(TyingState.DETECTING)
        self._publish_tying_feedback()

    def _handle_detecting(self):
        """멀티검출 처리 (DetectionManager 위임).

        Phase 1: 현재 자세 카메라 검출 → X큰→작은 순 결속
        Phase 2: 자세변경 후 카메라 검출 → X작은→큰 순 결속
        """
        result = self.detection_mgr.update()
        if result is None:
            return  # 진행 중
        if result == 'error':
            self._handle_error(self.detection_mgr.error)
            return

        # 검출 완료: result는 포인트 리스트 [(label, x, y, side), ...]
        points = result
        self.total_points += len(points)

        # Phase에 따라 결속 순서 결정
        if self.tying_phase == 1:
            # Phase 1: X 큰→작은 (자세변경 시 X=home 근처에서 끝나도록)
            points.sort(key=lambda p: p[1], reverse=True)  # p[1] = x_mm
        else:
            # Phase 2: X 작은→큰 (자세변경 후 X=home 근처에서 시작)
            points.sort(key=lambda p: p[1])

        # 라벨 재부여 (정렬 후)
        points = [(f'P{i+1}', x, y, side) for i, (_, x, y, side) in enumerate(points)]

        # 액션 큐 생성 (단일 자세 포인트만, 자세변경 없음)
        self.action_builder.update_params(
            x_home_mm=self.x_home_mm,
            yaw_right_cam=self.yaw_right_cam,
            yaw_left_cam=self.yaw_left_cam,
            yaw_left_cam_approach=self.yaw_left_cam_approach,
            yaw_mid=self.yaw_mid,
            home_ref_yaw_deg=self.home_ref_yaw_deg,
        )
        self.action_queue = []
        for label, x, y, side in points:
            self.action_queue.extend(
                self.action_builder._tying_actions(label, x, y))
        self.action_queue = self.action_builder._optimize_z_xy_overlap(
            self.action_queue, logger=self.get_logger())
        self.current_action_index = 0

        phase_label = f'Phase {self.tying_phase}'
        order = 'X큰→작은' if self.tying_phase == 1 else 'X작은→큰'
        self.get_logger().info(
            f'[{phase_label}] 검출 {len(points)}개 포인트, '
            f'{len(self.action_queue)}개 액션 ({order})')
        self.flog(f"검출 완료 [{phase_label}]: {len(points)}개 포인트, "
                  f"{len(self.action_queue)}개 액션 [{order}]")
        for label, x, y, side in points:
            self.flog(f"  {label}: ({x:.1f}, {y:.1f})mm [{side}]")
        self._transition_to(TyingState.EXECUTING_ACTION)

    # ============================================
    # 액션 큐 구축
    # ============================================
    def _build_action_queue(self, points, pass_type='normal'):
        """포인트 목록으로부터 실행할 액션 큐 생성 (ActionBuilder 위임)."""
        # ActionBuilder 파라미터 업데이트 (런타임 값)
        self.action_builder.update_params(
            x_home_mm=self.x_home_mm,
            yaw_right_cam=self.yaw_right_cam,
            yaw_left_cam=self.yaw_left_cam,
            yaw_left_cam_approach=self.yaw_left_cam_approach,
            yaw_mid=self.yaw_mid,
            home_ref_yaw_deg=self.home_ref_yaw_deg,
        )
        self.action_queue, self.current_pose_state = self.action_builder.build(
            points, self.current_pose_state, self.tying_direction,
            logger=self.get_logger()
        )

    # ============================================
    # 액션 실행
    # ============================================
    def _execute_current_action(self):
        """현재 액션 실행"""
        if self.current_action_index >= len(self.action_queue):
            self._transition_to(TyingState.COMPLETE)
            return

        action = self.action_queue[self.current_action_index]

        if action.action_type == ActionType.LOG:
            self.get_logger().info(action.message)
            self._advance_to_next_action()

        elif action.action_type == ActionType.MOVE_XY:
            self._send_xy_absolute(action.x_mm, action.y_mm)
            self.get_logger().info(f'  이동: {action.message}')
            if action.point_label:
                self.tying_message = f'{action.point_label} 이동 중...'
            else:
                self.tying_message = action.message
            self._publish_tying_feedback()
            # goal 설정 (mm → deg)
            self.goal_targets = {
                0x44: self.home_ref_x_deg - action.x_mm * self.deg_per_mm_x,
                0x45: self.home_ref_y_deg + action.y_mm * self.deg_per_mm_y,
            }
            self._transition_to(TyingState.WAITING_SETTLE)

        elif action.action_type == ActionType.MOVE_YAW:
            self._send_yaw_absolute(action.yaw_deg)
            self.get_logger().info(f'  Yaw: {action.message}')
            self.tying_message = action.message
            self._publish_tying_feedback()
            # goal 설정
            self.goal_targets = {
                0x47: action.yaw_deg,
            }
            self._transition_to(TyingState.WAITING_SETTLE)

        elif action.action_type == ActionType.MOVE_Z_DOWN:
            z_deg = action.z_mm * self.deg_per_mm_z
            self._send_z_relative(z_deg)  # 양수 = 하강
            self.get_logger().info(
                f'  Z하강: {action.message} ({z_deg:.1f}°)')
            self.tying_message = action.message
            self._publish_tying_feedback()
            # 이동 시간 계산: 거리/속도 + 마진
            self.z_wait_time = z_deg / self.effective_z_speed + self.z_settle_margin
            # 토크 모니터링 시작
            self.z_monitor.start(self.z_wait_time, self.speed_percent, self.effective_z_speed)
            self._transition_to(TyingState.WAITING_Z)

        elif action.action_type == ActionType.MOVE_Z_UP:
            z_deg = action.z_mm * self.deg_per_mm_z
            self._send_z_relative(-z_deg)  # 음수 = 상승
            self.get_logger().info(
                f'  Z상승: {action.message} (-{z_deg:.1f}°)')
            self.tying_message = action.message
            self._publish_tying_feedback()
            # 이동 시간 계산
            self.z_wait_time = z_deg / self.effective_z_speed + self.z_settle_margin
            self.z_monitor.z_descending = False  # 상승 시에는 토크 체크 안 함
            self._transition_to(TyingState.WAITING_Z)

        elif action.action_type == ActionType.MOVE_Z_UP_WITH_XY:
            # Phase 1: Z상승 시작
            z_deg = action.z_mm * self.deg_per_mm_z
            self._send_z_relative(-z_deg)  # 음수 = 상승
            self.get_logger().info(
                f'  Z↑+XY: {action.message} (Z:-{z_deg:.1f}°)')
            self.tying_message = action.message
            self._publish_tying_feedback()
            # Z 전체 시간과 조기 XY 시작 시간 계산
            self.z_wait_time = z_deg / self.effective_z_speed + self.z_settle_margin
            early_z_deg = self.z_early_xy_mm * self.deg_per_mm_z
            self.z_early_xy_time = early_z_deg / self.effective_z_speed
            self.z_xy_started = False
            # goal 설정 (XY 도달 확인용)
            self.goal_targets = {
                0x44: self.home_ref_x_deg - action.x_mm * self.deg_per_mm_x,
                0x45: self.home_ref_y_deg + action.y_mm * self.deg_per_mm_y,
            }
            self._transition_to(TyingState.WAITING_Z_XY)

        elif action.action_type == ActionType.TRIGGER:
            self.get_logger().info(f'  트리거: {action.message}')
            self.tying_message = action.message
            self._publish_tying_feedback()
            # 트리거 다단계 시작
            self.trigger_phase = 0
            self.trigger_phase_start = time.monotonic()
            self._send_trigger(self.trigger_speed_val)
            self.get_logger().info('    트리거 발사')
            self._transition_to(TyingState.WAITING_TRIGGER)

    def _advance_to_next_action(self):
        """다음 액션으로 진행"""
        # Z축 상승 완료 시 = 1포인트 결속 완료 → 진행률 업데이트
        if self.current_action_index < len(self.action_queue):
            action = self.action_queue[self.current_action_index]
            if (action.action_type in (ActionType.MOVE_Z_UP, ActionType.MOVE_Z_UP_WITH_XY)
                    and action.point_label):
                self.completed_points += 1
                self.tying_message = (
                    f'{action.point_label} 완료 '
                    f'({self.completed_points}/{self.total_points})')

        self.current_action_index += 1
        self._publish_tying_feedback()
        if self.current_action_index >= len(self.action_queue):
            self._transition_to(TyingState.COMPLETE)
        else:
            self._transition_to(TyingState.EXECUTING_ACTION)

    def _check_goal_reached(self) -> bool:
        """모든 goal_targets 모터가 허용 오차 이내인지 확인"""
        if not self.goal_targets:
            return True

        for motor_id, target_deg in self.goal_targets.items():
            current = self.motor_positions.get(motor_id)
            if current is None:
                return False  # 피드백 아직 없음
            if abs(current - target_deg) > self.position_tolerance:
                return False
        return True

    # ============================================
    # 트리거 다단계 처리
    # ============================================
    def _handle_trigger_phase(self):
        """트리거 동작 다단계 처리

        Phase 0: 발사 (trigger_speed, trigger_duration 동안)
        Phase 1: 정지 후 역방향 원복
        Phase 2: 역방향 원복 (trigger_duration 동안)
        Phase 3: 정지 → 완료
        """
        phase_elapsed = time.monotonic() - self.trigger_phase_start

        if self.trigger_phase == 0:
            # 발사 중 → trigger_duration 후 정지
            if phase_elapsed >= self.trigger_duration:
                self._send_trigger(0.0)
                self.get_logger().info('    트리거 정지')
                self.trigger_phase = 1
                self.trigger_phase_start = time.monotonic()

        elif self.trigger_phase == 1:
            # 정지 후 0.1초 대기 → 역방향
            if phase_elapsed >= 0.1:
                self._send_trigger(-self.trigger_speed_val)
                self.get_logger().info('    트리거 원복')
                self.trigger_phase = 2
                self.trigger_phase_start = time.monotonic()

        elif self.trigger_phase == 2:
            # 역방향 원복 중 → trigger_duration 후 정지
            if phase_elapsed >= self.trigger_duration:
                self._send_trigger(0.0)
                self.get_logger().info('    트리거 완료')
                self.trigger_phase = 3
                self.trigger_phase_start = time.monotonic()

        elif self.trigger_phase == 3:
            # 정지 확인 후 다음 액션
            if phase_elapsed >= 0.1:
                self._advance_to_next_action()

    # ============================================
    # 모터 명령
    # ============================================
    def _send_xy_absolute(self, x_mm: float, y_mm: float):
        """XY 스테이지 절대위치 명령 (mm → deg, 홈 기준 상대좌표)."""
        x_deg = self.home_ref_x_deg - x_mm * self.deg_per_mm_x
        y_deg = self.home_ref_y_deg + y_mm * self.deg_per_mm_y

        # X축 (0x144)
        x_msg = JointControl()
        x_msg.joint_id = 0x144
        x_msg.position = x_deg
        x_msg.velocity = self.effective_stage_speed
        x_msg.control_mode = JointControl.MODE_ABSOLUTE
        self.joint_pub.publish(x_msg)

        self.get_logger().info(
            f'    X cmd: {x_mm:.1f}mm → {x_deg:.1f}°')

        # Y축 (0x145) - 약간의 딜레이로 CAN 버스 충돌 방지
        time.sleep(0.05)
        y_msg = JointControl()
        y_msg.joint_id = 0x145
        y_msg.position = y_deg
        y_msg.velocity = self.effective_stage_speed
        y_msg.control_mode = JointControl.MODE_ABSOLUTE
        self.joint_pub.publish(y_msg)

        self.get_logger().info(
            f'    Y cmd: {y_mm:.1f}mm → {y_deg:.1f}°')

    def _send_yaw_absolute(self, yaw_deg: float):
        """Yaw 절대각도 명령 (0x92 멀티턴)."""
        yaw_msg = JointControl()
        yaw_msg.joint_id = 0x147
        yaw_msg.position = yaw_deg
        yaw_msg.velocity = self.effective_stage_speed
        yaw_msg.control_mode = JointControl.MODE_ABSOLUTE
        self.joint_pub.publish(yaw_msg)

        self.get_logger().debug(f'    Yaw cmd: {yaw_deg:.1f}°')

    def _send_z_relative(self, delta_deg: float):
        """Z축 상대 이동 명령 (0x146). 양수=하강, 음수=상승."""
        msg = JointControl()
        msg.joint_id = 0x146
        msg.position = delta_deg
        msg.velocity = self.effective_z_speed
        msg.control_mode = JointControl.MODE_RELATIVE
        self.joint_pub.publish(msg)

    def _send_trigger(self, speed: float):
        """트리거 모터 속도 명령 (Pololu /motor_0/vel)."""
        msg = Float32()
        msg.data = speed
        self.trigger_pub.publish(msg)

    def _handle_z_overload(self):
        """Z축 토크 과부하 시: Z정지 → 트리거 스킵 → Z상승.

        액션 큐: ... → MOVE_Z_DOWN(현재) → TRIGGER → MOVE_Z_UP → ...
        TRIGGER를 스킵하고 MOVE_Z_UP으로 점프한다.
        MOVE_Z_UP은 tying_z_down_mm 전체를 상승 시도하지만,
        실제 하강 거리보다 크더라도 상부 리밋 센서가 보호해준다.
        """
        # Z축 즉시 정지
        self.z_monitor.stop_z_motor()

        # TRIGGER 액션 찾아서 스킵 → MOVE_Z_UP으로 점프
        idx = self.current_action_index + 1
        while idx < len(self.action_queue):
            if self.action_queue[idx].action_type == ActionType.TRIGGER:
                # TRIGGER를 건너뛰고 그 다음(MOVE_Z_UP)으로
                self.current_action_index = idx  # _advance 에서 +1 → MOVE_Z_UP
                self.get_logger().warn(
                    f'    트리거 스킵 → 액션[{idx + 1}] Z상승으로 전환')
                break
            idx += 1

        self._advance_to_next_action()

    # ============================================
    # 완료 / 에러 / 취소 처리
    # ============================================
    def _handle_complete(self):
        """결속 시퀀스 완료

        Phase 1 완료 → 자세변경 + Phase 2 검출 재진입
        Phase 2 완료 → 진짜 완료 (또는 repeat 처리)
        """
        # 자세변경 완료 → Phase 2 검출 시작
        if hasattr(self, '_phase2_cam') and self._phase2_cam is not None:
            cam = self._phase2_cam
            self._phase2_cam = None  # 1회만 사용
            self.get_logger().info(
                f'[Phase 2] {cam} 카메라 검출 시작 (자세변경 완료)')
            self.flog(f"PHASE2_START: pose={self.current_pose_state} cam={cam}")
            self.action_queue = []
            self.current_action_index = 0
            self.detection_mgr.start(camera_side=cam)
            self.tying_message = f'{cam} 카메라 검출 중...'
            self._transition_to(TyingState.DETECTING)
            self._publish_tying_feedback()
            return

        # Phase 1 완료 → 자세변경 + Phase 2 검출
        if self.tying_phase == 1:
            phase1_completed = self.completed_points
            self.get_logger().info('=' * 60)
            self.get_logger().info(
                f'[Phase 1 완료] {phase1_completed}개 체결, '
                f'자세변경 → Phase 2 검출 시작')
            self.get_logger().info('=' * 60)
            self.flog(f"PHASE1_DONE: completed={phase1_completed}")

            # 자세변경 액션 생성 + 실행
            if self.current_pose_state == 'right':
                next_cam = 'left'
                next_pose = 'left'
            else:
                next_cam = 'right'
                next_pose = 'right'

            self.action_builder.update_params(
                x_home_mm=self.x_home_mm,
                yaw_right_cam=self.yaw_right_cam,
                yaw_left_cam=self.yaw_left_cam,
                yaw_left_cam_approach=self.yaw_left_cam_approach,
                yaw_mid=self.yaw_mid,
                home_ref_yaw_deg=self.home_ref_yaw_deg,
            )
            pose_actions = self.action_builder.build_pose_change(
                self.current_pose_state, next_pose)
            self.current_pose_state = next_pose

            # 자세변경 액션 실행 후 Phase 2 검출로 전환하기 위해
            # _phase2_cam을 저장해두고, 자세변경 완료 시 검출 시작
            self._phase2_cam = next_cam
            self.tying_phase = 2
            self.action_queue = pose_actions
            self.current_action_index = 0
            self.tying_message = f'자세변경 중 ({self.current_pose_state})...'
            self._transition_to(TyingState.EXECUTING_ACTION)
            self._publish_tying_feedback()
            return

        # Phase 2 완료 → repeat 또는 최종 완료
        if self.repeat_mode and self.tying_pass == 'forward':
            forward_completed = self.completed_points
            self.get_logger().info('=' * 60)
            self.get_logger().info(
                f'[REPEAT] 전진 패스 완료 ({forward_completed}개 체결), '
                f'복귀 패스 재검출 시작')
            self.get_logger().info('=' * 60)
            self.tying_pass = 'return'
            self.tying_direction = 'reverse'
            self._forward_completed_points = forward_completed
            # 현재 자세 카메라로 Phase 1 재시작
            self.tying_phase = 1
            cam_side = 'left' if self.current_pose_state == 'left' else 'right'
            self.action_queue = []
            self.current_action_index = 0
            self.detection_mgr.start(camera_side=cam_side)
            self.tying_message = '복귀 패스 교차점 멀티검출 중...'
            self.flog(f"REPEAT_RETURN: pose={self.current_pose_state} cam={cam_side}")
            self._transition_to(TyingState.DETECTING)
            self._publish_tying_feedback()
            return

        # 일반 완료 (normal 또는 return 패스)
        total = self.completed_points
        if self.repeat_mode and self.tying_pass == 'return':
            total_label = (
                f'전진 {self._forward_completed_points}개 + '
                f'복귀 {self.completed_points - self._forward_completed_points}개')
        else:
            total_label = f'{self.completed_points}개'

        self.get_logger().info('=' * 60)
        self.get_logger().info(
            f'[AUTO] 자동 결속 시퀀스 완료! '
            f'({total_label} / {self.total_points}개 포인트)')
        self.get_logger().info('=' * 60)
        self.flog(f"결속 완료: {total_label} / {self.total_points}개 → TYING_COMPLETE 발행")

        msg = String()
        msg.data = f'TYING_COMPLETE:{self.completed_points}'
        self.motion_cmd_pub.publish(msg)

        self.tying_message = '결속 완료'
        self.tying_result = (
            f'결속 완료 ({self.completed_points}/{self.total_points} 포인트)')
        self._publish_tying_feedback()
        self._transition_to(TyingState.IDLE)

    def _handle_error(self, error_msg):
        """에러 처리 (navigator 블록 방지를 위해 완료 신호 발행)"""
        self.z_monitor.stop()
        self.get_logger().error(f'[ERROR] {error_msg}')
        self.flog(f"ERROR: {error_msg} → TYING_COMPLETE 발행")

        msg = String()
        msg.data = 'TYING_COMPLETE:0'
        self.motion_cmd_pub.publish(msg)

        self.tying_message = error_msg
        self.tying_result = f'결속 실패: {error_msg}'
        self._transition_to(TyingState.ERROR)
        self._publish_tying_feedback()

    def _handle_cancel(self):
        """작업 취소 처리"""
        # 토크 모니터링 정지
        self.z_monitor.stop()
        # 모든 스테이지 모터 즉시 정지
        for motor_id in [0x144, 0x145, 0x146, 0x147]:
            stop_msg = JointControl()
            stop_msg.joint_id = motor_id
            stop_msg.position = 0.0
            stop_msg.velocity = 0.0
            stop_msg.control_mode = JointControl.MODE_SPEED
            self.joint_pub.publish(stop_msg)
        self.get_logger().warn('모든 스테이지 모터 정지 명령 전송')

        # 트리거 정지 (안전)
        self._send_trigger(0.0)

        msg = String()
        msg.data = f'TYING_COMPLETE:{self.completed_points}'
        self.motion_cmd_pub.publish(msg)

        self.tying_message = '작업 취소됨'
        self.tying_result = (
            f'취소됨 ({self.completed_points}/{self.total_points} 포인트 완료)')
        self._transition_to(TyingState.IDLE)
        self._publish_tying_feedback()

    # ============================================
    # 유틸리티
    # ============================================
    def _transition_to(self, new_state: TyingState):
        """상태 전환"""
        if self.state != new_state:
            self.get_logger().debug(
                f'상태: {self.state.name} → {new_state.name}')
            self.flog(f"상태: {self.state.name} → {new_state.name}")
        self.state = new_state
        self.state_start_time = time.monotonic()

    def _elapsed(self):
        """현재 상태 경과 시간"""
        if self.state_start_time is None:
            return 0.0
        return time.monotonic() - self.state_start_time

    def _publish_tying_feedback(self):
        """결속 상태 피드백 발행 (JSON)

        UI가 rebar/status를 통해 수신하는 필드:
        - tying_state: idle / tying / success / failed
        - tying_progress: 0~100 (%)
        - tying_message: 현재 동작 설명
        - tying_result: 최종 결과 요약
        """
        # 상태 매핑
        state_map = {
            TyingState.IDLE: 'idle',
            TyingState.DETECTING: 'tying',
            TyingState.EXECUTING_ACTION: 'tying',
            TyingState.WAITING_SETTLE: 'tying',
            TyingState.WAITING_Z: 'tying',
            TyingState.WAITING_Z_XY: 'tying',
            TyingState.WAITING_TRIGGER: 'tying',
            TyingState.PAUSED: 'tying',
            TyingState.COMPLETE: 'success',
            TyingState.ERROR: 'failed',
        }
        tying_state = state_map.get(self.state, 'idle')

        # 진행률 계산
        if self.total_points > 0:
            progress = int(
                (self.completed_points / self.total_points) * 100)
        else:
            progress = 0

        feedback = {
            'tying_state': tying_state,
            'tying_progress': progress,
            'tying_message': self.tying_message,
            'tying_result': self.tying_result,
        }

        msg = String()
        msg.data = json.dumps(feedback)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TyingOrchestratorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('종료 중...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

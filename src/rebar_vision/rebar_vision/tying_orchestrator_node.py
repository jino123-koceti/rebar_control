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
from enum import Enum, auto
from dataclasses import dataclass, field
from typing import List


class ActionType(Enum):
    """액션 큐 항목 유형"""
    MOVE_XY = auto()             # XY 스테이지 절대위치 이동 (mm)
    MOVE_YAW = auto()            # Yaw 절대각도 이동 (deg, 0x92 멀티턴)
    MOVE_Z_DOWN = auto()         # Z축 하강 (상대 이동, mm)
    MOVE_Z_UP = auto()           # Z축 상승 (상대 이동, mm)
    MOVE_Z_UP_WITH_XY = auto()   # Z상승 + 조기 XY이동 (연속 동작)
    TRIGGER = auto()             # 트리거 동작 (발사 → 정지 → 원복 → 정지)
    LOG = auto()                 # 로그 출력


@dataclass
class Action:
    """실행할 액션 정의"""
    action_type: ActionType
    x_mm: float = 0.0
    y_mm: float = 0.0
    yaw_deg: float = 0.0
    z_mm: float = 0.0            # Z축 이동 거리 (mm)
    message: str = ''
    point_label: str = ''


class TyingState(Enum):
    """오케스트레이터 상태"""
    IDLE = auto()
    DETECTING = auto()
    EXECUTING_ACTION = auto()
    WAITING_SETTLE = auto()
    WAITING_Z = auto()           # Z축 이동 대기 (계산된 시간)
    WAITING_Z_XY = auto()        # Z상승 중 + XY 병렬 이동 대기
    WAITING_TRIGGER = auto()     # 트리거 동작 대기 (다단계)
    PAUSED = auto()
    COMPLETE = auto()
    ERROR = auto()


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

        # 트리거 상태
        self.trigger_phase = 0           # 0=fire, 1=wait_fire, 2=return, 3=wait_return
        self.trigger_phase_start = 0.0

        # 검출 상태 (멀티검출)
        self.detection_future = None
        self.detection_retry = 0
        self.multi_detect_count = 0          # 현재 검출 시도 횟수
        self.accumulated_points = []         # 누적 검출 포인트
        self.multi_detect_wait_until = 0.0   # 다음 검출까지 대기 시각

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
            if elapsed >= self.z_wait_time:
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
        """검출 시작 (상태 초기화)"""
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
        self.detection_future = None
        self.detection_retry = 0
        self.multi_detect_count = 0
        self.accumulated_points = []
        self.multi_detect_wait_until = 0.0
        self._detect_camera_phase = 'right'
        self.total_points = 0
        self.completed_points = 0
        self.tying_message = '교차점 멀티검출 중...'
        self.tying_result = ''
        self._transition_to(TyingState.DETECTING)
        self._publish_tying_feedback()

    def _handle_detecting(self):
        """멀티검출 서비스 비동기 호출 처리

        최대 multi_detect_tries회 검출을 반복하여 포인트를 누적하고,
        클러스터링 → 보간 → 추세선 보정을 수행한다.
        카메라별 개별 호출 (right=2, left=1) 로 _filter_far_side 회피.
        """
        # 다음 검출까지 대기 중
        if time.monotonic() < self.multi_detect_wait_until:
            return

        if self.detection_future is None:
            if not self.detect_client.service_is_ready():
                if self._elapsed() >= 5.0:
                    self._handle_error('검출 서비스 미가용')
                return

            # right(2) → left(1) 교대 호출
            if not hasattr(self, '_detect_camera_phase'):
                self._detect_camera_phase = 'right'

            if self._detect_camera_phase == 'right':
                cam_sel = 2
                cam_label = 'R'
            else:
                cam_sel = 1
                cam_label = 'L'

            request = DetectCrossings.Request()
            request.camera_selection = cam_sel
            request.confidence_threshold = self.detection_confidence
            request.expected_count = 3
            self.detection_future = self.detect_client.call_async(request)

            if self._detect_camera_phase == 'right':
                self.multi_detect_count += 1
            self.get_logger().info(
                f'멀티검출 [{self.multi_detect_count}/{self.multi_detect_tries}] '
                f'({cam_label}) 서비스 호출...')
            return

        if not self.detection_future.done():
            return

        try:
            result = self.detection_future.result()
            self.detection_future = None
            cam_label = 'R' if self._detect_camera_phase == 'right' else 'L'

            if result.success and len(result.grid.detections) > 0:
                dets = list(result.grid.detections)
                self.get_logger().info(
                    f'  [{self.multi_detect_count}/{self.multi_detect_tries}] '
                    f'({cam_label}) {len(dets)}개 검출 '
                    f'({result.detection_time_ms:.0f}ms)')

                # 포인트 누적 (범위 필터링은 _build_final_points에서 수행)
                for det in dets:
                    cam_tag = 'R' if det.camera_id == 1 else 'L'
                    self.get_logger().info(
                        f'    누적: ({det.x:.1f}, {det.y:.1f}) [{cam_tag}]')
                    self.accumulated_points.append({
                        'x': det.x,
                        'y': det.y,
                        'camera_id': det.camera_id,
                    })
            else:
                msg_text = result.message if result else '응답 없음'
                self.get_logger().warn(
                    f'  [{self.multi_detect_count}/{self.multi_detect_tries}] '
                    f'({cam_label}) 검출 실패: {msg_text}')

            # 카메라 교대
            if self._detect_camera_phase == 'right':
                # right 완료 → left 호출 (대기 없이 즉시)
                self._detect_camera_phase = 'left'
                return
            else:
                # left 완료 → 1회 시도 완료
                self._detect_camera_phase = 'right'

            # 조기 종료 체크: 양쪽 카메라 모두 충분한 클러스터 확보
            if self.multi_detect_count >= 2:
                right_pts = [p for p in self.accumulated_points
                             if p['camera_id'] == 1]
                left_pts = [p for p in self.accumulated_points
                            if p['camera_id'] == 0]
                right_clusters = self._cluster_points(right_pts)
                left_clusters = self._cluster_points(left_pts)
                if (len(right_clusters) >= self.expected_points_per_cam
                        and len(left_clusters) >= self.expected_points_per_cam):
                    self.get_logger().info(
                        f'  양쪽 카메라 각 {self.expected_points_per_cam}개 '
                        f'클러스터 확보 → 조기 종료')
                    self._finalize_multi_detection()
                    return

            # 최대 시도 횟수 도달 시 최종 처리
            if self.multi_detect_count >= self.multi_detect_tries:
                if not self.accumulated_points:
                    self._handle_error(
                        f'멀티검출 {self.multi_detect_tries}회 모두 실패')
                else:
                    self._finalize_multi_detection()
            else:
                # 다음 검출 전 대기
                self.multi_detect_wait_until = (
                    time.monotonic() + self.multi_detect_delay)

        except Exception as e:
            self.detection_future = None
            self._handle_error(f'검출 서비스 예외: {e}')

    def _finalize_multi_detection(self):
        """멀티검출 누적 데이터를 클러스터링 → 보간 → 추세선 보정하여 최종 포인트 생성"""
        # 카메라별 분리 (캘리브레이션 모델이 이미 로봇 좌표계로 변환)
        right_pts = [p for p in self.accumulated_points if p['camera_id'] == 1]
        left_pts = [p for p in self.accumulated_points if p['camera_id'] == 0]

        self.get_logger().info(
            f'멀티검출 완료: {len(self.accumulated_points)}개 누적 '
            f'(R:{len(right_pts)}, L:{len(left_pts)})')

        # 카메라별 클러스터링 + 보간 + 추세선
        right_final = self._process_camera_points(right_pts, 'right')
        left_final = self._process_camera_points(left_pts, 'left')

        if not right_final and not left_final:
            self._handle_error('유효한 교차점 없음 (클러스터링 후)')
            return

        # 범위 필터링 + 클램핑 + P1~P6 라벨 부여
        points = self._build_final_points(right_final, left_final)

        if len(points) == 0:
            self._handle_error('유효한 교차점 없음 (범위 필터링 후)')
            return

        self._build_action_queue(points, pass_type=self.tying_pass)
        self.get_logger().info(
            f'검출 최종: {len(points)}개 포인트, '
            f'{len(self.action_queue)}개 액션 생성'
            f' [{self.tying_pass} 패스]')
        self.flog(f"검출 완료: {len(points)}개 포인트, {len(self.action_queue)}개 액션 [{self.tying_pass}]")
        for label, x, y, side in points:
            self.flog(f"  {label}: ({x:.1f}, {y:.1f})mm [{side}]")
        self._transition_to(TyingState.EXECUTING_ACTION)

    def _process_camera_points(self, pts, camera_side):
        """카메라 한쪽의 누적 포인트를 클러스터링 → 보간 → 추세선 보정

        Returns:
            list of (x_mm, y_mm) 보정된 좌표 (X 내림차순)
        """
        if not pts:
            self.get_logger().warn(f'  [{camera_side}] 검출 포인트 없음')
            return []

        # 클러스터링
        clusters = self._cluster_points(pts)
        self.get_logger().info(
            f'  [{camera_side}] 클러스터링: {len(pts)}개 → {len(clusters)}개')

        # 작업영역 범위 밖 클러스터 사전 제거 (보간 전)
        margin = 30.0  # 클러스터 중심이 범위+마진 밖이면 노이즈로 판단
        # 자세별 Y 가동 범위
        if camera_side == 'right':
            y_min_limit = self.right_cam_y_min - margin
            y_max_limit = self.right_cam_y_max + margin
        else:
            y_min_limit = self.left_cam_y_min - margin
            y_max_limit = self.left_cam_y_max + margin
        valid_clusters = []
        for c in clusters:
            if (c['x'] < -margin or c['x'] > self.max_stage_x_mm + margin
                    or c['y'] < y_min_limit
                    or c['y'] > y_max_limit):
                self.get_logger().info(
                    f'  [{camera_side}] 범위 외 클러스터 제거: '
                    f'X={c["x"]:.1f}, Y={c["y"]:.1f}')
                continue
            valid_clusters.append(c)
        clusters = valid_clusters

        # 보간 (기대 수보다 적으면)
        before = len(clusters)
        clusters = self._interpolate_missing(clusters)
        interp_count = len(clusters) - before
        if interp_count > 0:
            self.get_logger().info(
                f'  [{camera_side}] 보간: {interp_count}개 추론 추가')

        # X 내림차순 정렬
        clusters.sort(key=lambda c: c['x'], reverse=True)

        # 추세선 피팅 + 보정
        trendline = self._fit_trendline(clusters)
        if trendline is not None:
            corrected = trendline['corrected']
            avg_err = np.mean(trendline['errors'])
            slope, intercept, axis = trendline['line_eq']
            if axis == 'x':
                eq = f'Y={slope:.4f}*X+{intercept:.1f}'
            else:
                eq = f'X={slope:.4f}*Y+{intercept:.1f}'
            self.get_logger().info(
                f'  [{camera_side}] 추세선: {eq}, '
                f'평균오차: {avg_err:.1f}mm')
            for i, (cx, cy) in enumerate(corrected):
                orig = clusters[i]
                src = 'interp' if orig.get('source') == 'interpolated' else 'hits:{}'.format(orig['hit_count'])
                self.get_logger().info(
                    '    P{}: ({:.1f},{:.1f}) → ({:.1f},{:.1f}) err:{:.1f}mm [{}]'.format(
                        i+1, orig['x'], orig['y'], cx, cy,
                        trendline['errors'][i], src))

            # 이상치 제거 + 재피팅 (err > 100mm)
            outlier_threshold = 100.0
            outlier_indices = [i for i, e in enumerate(trendline['errors']) if e > outlier_threshold]
            if outlier_indices and len(clusters) - len(outlier_indices) >= 2:
                for idx in outlier_indices:
                    self.get_logger().warn(
                        f'  [{camera_side}] 이상치 제거: P{idx+1} '
                        f'({clusters[idx]["x"]:.1f}, {clusters[idx]["y"]:.1f}) '
                        f'err:{trendline["errors"][idx]:.1f}mm > {outlier_threshold}mm')
                clusters = [c for i, c in enumerate(clusters) if i not in outlier_indices]
                # 재피팅
                trendline2 = self._fit_trendline(clusters)
                if trendline2 is not None:
                    corrected = trendline2['corrected']
                    avg_err2 = np.mean(trendline2['errors'])
                    s2, i2, ax2 = trendline2['line_eq']
                    eq2 = f'Y={s2:.4f}*X+{i2:.1f}' if ax2 == 'x' else f'X={s2:.4f}*Y+{i2:.1f}'
                    self.get_logger().info(
                        f'  [{camera_side}] 재피팅: {eq2}, '
                        f'평균오차: {avg_err:.1f}→{avg_err2:.1f}mm')
                    for i, (cx, cy) in enumerate(corrected):
                        orig = clusters[i]
                        src = 'interp' if orig.get('source') == 'interpolated' else 'hits:{}'.format(orig['hit_count'])
                        self.get_logger().info(
                            '    P{}: ({:.1f},{:.1f}) → ({:.1f},{:.1f}) err:{:.1f}mm [{}]'.format(
                                i+1, orig['x'], orig['y'], cx, cy,
                                trendline2['errors'][i], src))

            return corrected
        else:
            # 추세선 불가 (포인트 1개) → 원본 좌표
            self.get_logger().info(
                f'  [{camera_side}] 추세선 불가 → 원본 좌표 사용')
            return [(c['x'], c['y']) for c in clusters]

    def _build_final_points(self, right_coords, left_coords):
        """보정된 좌표로 최종 포인트 리스트 생성 (범위 필터링 + 클램핑)

        Returns:
            list of (label, x_mm, y_mm, side)
        """
        clamp_margin = 30.0
        points = []

        # Right: X내림차순(큰→작은), Left: X오름차순(작은→큰)
        left_coords_asc = list(reversed(left_coords))
        for side, coords in [('right', right_coords), ('left', left_coords_asc)]:
            # 자세별 Y 가동 범위
            if side == 'right':
                y_min_safe = self.right_cam_y_min
                y_max_safe = self.right_cam_y_max
            else:
                y_min_safe = self.left_cam_y_min
                y_max_safe = self.left_cam_y_max
            for x, y in coords:
                # 음수 좌표 스킵
                if x < 0 or y < 0:
                    self.get_logger().info(
                        f'  음수 스킵: X={x:.1f}mm, Y={y:.1f}mm')
                    continue
                if x > self.max_stage_x_mm or y > self.max_stage_y_mm:
                    self.get_logger().warn(
                        f'  범위 외 제거: X={x:.1f}, Y={y:.1f} (초과)')
                    continue
                # 자세별 Y 범위 필터링
                if y < y_min_safe or y > y_max_safe:
                    self.get_logger().info(
                        f'  [{side}] Y 범위 외 스킵: Y={y:.1f}mm '
                        f'(허용: {y_min_safe}~{y_max_safe}mm)')
                    continue
                points.append((None, x, y, side))

        # 라벨 부여 (P1~)
        labeled = []
        idx = 1
        for _, x, y, side in points:
            labeled.append((f'P{idx}', x, y, side))
            idx += 1

        if self.tying_pass == 'return':
            # 복귀 패스: 전진 포인트에 추가
            self.total_points += len(labeled)
        else:
            self.total_points = len(labeled)

        # 포인트 로그
        pass_label = {'forward': '전진', 'return': '복귀'}.get(
            self.tying_pass, '')
        self.get_logger().info(
            f'  {pass_label}경로: {" → ".join(p[0] for p in labeled)}')
        for label, x, y, side in labeled:
            self.get_logger().info(
                f'    [{label}] X={x:.1f}mm Y={y:.1f}mm ({side})')

        return labeled

    # ============================================
    # 클러스터링 / 보간 / 추세선
    # ============================================
    def _cluster_points(self, pts):
        """포인트 리스트를 거리 기반 클러스터링.

        Args:
            pts: list of dict with 'x', 'y' keys

        Returns:
            list of dict: 클러스터 평균 좌표
        """
        dist_mm = self.cluster_distance_mm
        used = set()
        clusters = []
        for i in range(len(pts)):
            if i in used:
                continue
            group = [i]
            used.add(i)
            for j in range(i + 1, len(pts)):
                if j in used:
                    continue
                dx = pts[i]['x'] - pts[j]['x']
                dy = pts[i]['y'] - pts[j]['y']
                if (dx * dx + dy * dy) ** 0.5 < dist_mm:
                    group.append(j)
                    used.add(j)
            clusters.append(group)

        results = []
        for group in clusters:
            gp = [pts[i] for i in group]
            results.append({
                'x': np.mean([p['x'] for p in gp]),
                'y': np.mean([p['y'] for p in gp]),
                'hit_count': len(group),
                'source': 'detected',
            })
        return results

    def _interpolate_missing(self, clusters):
        """클러스터가 기대 수보다 적으면 195mm 간격 보간으로 누락 포인트 추론.

        2개 포인트 간 거리를 분석하여 누락 위치를 판단:
        - 거리 ≈ 1*spacing → 한쪽 끝에 누락 (작업영역에 맞는 방향으로 추가)
        - 거리 ≈ 2*spacing → 중간에 누락
        """
        expected = self.expected_points_per_cam
        if len(clusters) >= expected or len(clusters) < 1:
            return clusters

        # X 내림차순 정렬
        clusters.sort(key=lambda c: c['x'], reverse=True)

        x_vals = [c['x'] for c in clusters]
        y_avg = np.mean([c['y'] for c in clusters])

        if len(clusters) == 1:
            # 1개만 검출 → spacing 간격으로 양쪽에 보간 (작업영역 내)
            spacing = self.rebar_spacing_mm
            base_x = clusters[0]['x']
            candidates = [base_x - spacing, base_x + spacing]
            for cx in candidates:
                if 0 <= cx <= self.max_stage_x_mm and len(clusters) < expected:
                    self.get_logger().info(
                        f'  보간(1pt): X={cx:.1f}mm 추가 '
                        f'(기준 {base_x:.1f}mm ± {spacing}mm)')
                    clusters.append({
                        'x': cx,
                        'y': y_avg,
                        'hit_count': 0,
                        'source': 'interpolated',
                    })

        elif len(clusters) == 2:
            spacing = self.rebar_spacing_mm  # 195mm 철근 배근 간격
            x_max, x_min = max(x_vals), min(x_vals)
            gap = x_max - x_min

            if gap > spacing * 1.5:
                # 간격이 2*spacing에 가까움 → 중간에 누락
                mid_x = (x_max + x_min) / 2.0
                self.get_logger().info(
                    f'  보간: 간격 {gap:.0f}mm ≈ 2*{spacing}mm → '
                    f'중간 X={mid_x:.1f}mm 추가')
                clusters.append({
                    'x': mid_x,
                    'y': y_avg,
                    'hit_count': 0,
                    'source': 'interpolated',
                })
            else:
                # 간격이 1*spacing에 가까움 → 끝에 누락
                # 작업영역(0~max_stage_x_mm) 안에 들어오는 방향으로 추가
                candidate_high = x_max + spacing
                candidate_low = x_min - spacing
                # 작업영역 내에 있는 쪽 선택
                high_valid = 0 <= candidate_high <= self.max_stage_x_mm
                low_valid = 0 <= candidate_low <= self.max_stage_x_mm
                if high_valid and not low_valid:
                    new_x = candidate_high
                elif low_valid and not high_valid:
                    new_x = candidate_low
                elif high_valid and low_valid:
                    # 둘 다 유효하면 작업영역 중심에 가까운 쪽
                    center = self.max_stage_x_mm / 2.0
                    new_x = (candidate_high
                             if abs(candidate_high - center)
                             < abs(candidate_low - center)
                             else candidate_low)
                else:
                    # 둘 다 범위 밖 → 등간격 fallback
                    new_x = x_max + spacing
                self.get_logger().info(
                    f'  보간: 간격 {gap:.0f}mm ≈ 1*{spacing}mm → '
                    f'X={new_x:.1f}mm 추가')
                clusters.append({
                    'x': new_x,
                    'y': y_avg,
                    'hit_count': 0,
                    'source': 'interpolated',
                })

        clusters.sort(key=lambda c: c['x'], reverse=True)
        return clusters

    def _fit_trendline(self, clusters):
        """검출된 포인트들로 직선 추세선 피팅 (로봇 좌표 X,Y 기반).

        Returns:
            dict with 'corrected', 'line_eq', 'errors' or None
        """
        pts = [(c['x'], c['y']) for c in clusters]
        if len(pts) < 2:
            return None

        xs = np.array([p[0] for p in pts])
        ys = np.array([p[1] for p in pts])

        x_range = xs.max() - xs.min()
        y_range = ys.max() - ys.min()

        if x_range >= y_range:
            coeffs = np.polyfit(xs, ys, 1)
            slope, intercept = coeffs
            axis = 'x'
            a, b, c = slope, -1.0, intercept
        else:
            coeffs = np.polyfit(ys, xs, 1)
            slope, intercept = coeffs
            axis = 'y'
            a, b, c = -1.0, slope, intercept

        corrected = []
        errors = []
        for x0, y0 in pts:
            x_proj = (b * (b * x0 - a * y0) - a * c) / (a * a + b * b)
            y_proj = (a * (-b * x0 + a * y0) - b * c) / (a * a + b * b)
            corrected.append((x_proj, y_proj))
            dist = abs(a * x0 + b * y0 + c) / np.sqrt(a * a + b * b)
            errors.append(dist)

        return {
            'corrected': corrected,
            'line_eq': (slope, intercept, axis),
            'errors': errors,
        }

    # ============================================
    # 액션 큐 구축
    # ============================================
    def _build_action_queue(self, points, pass_type='normal'):
        """포인트 목록으로부터 실행할 액션 큐 생성.

        Args:
            points: list of (label, x_mm, y_mm, side)
            pass_type: 'normal' (단일), 'forward' (반복 전진), 'return' (반복 복귀)

        current_pose_state를 기반으로 자세변경/복귀 필요 여부를 판단.
        tying_direction에 따라:
        - 'forward': Right(P1→P2→P3) → 자세변경(우→좌) → Left(P4→P5→P6)
        - 'reverse': Left(P6→P5→P4) → 자세변경(좌→우) → Right(P3→P2→P1)
        """
        self.action_queue = []

        right_pts = [(l, x, y) for l, x, y, s in points if s == 'right']
        left_pts = [(l, x, y) for l, x, y, s in points if s == 'left']

        self.get_logger().info(
            f'자세 상태: {self.current_pose_state} | '
            f'방향: {self.tying_direction} | 패스: {self.tying_pass} | '
            f'Right: {len(right_pts)}개, Left: {len(left_pts)}개')

        if self.tying_direction == 'reverse':
            # reverse: Left(역순) → 자세변경(좌→우) → Right(역순)
            left_reversed = list(reversed(left_pts))
            right_reversed = list(reversed(right_pts))

            self.action_queue.append(Action(
                action_type=ActionType.LOG,
                message='=== reverse: Left(P6→P4) → 자세변경 → Right(P3→P1) ===',
            ))

            # Left 결속 전: 현재 Right 자세이면 Left로 전환 필요
            if left_reversed and self.current_pose_state != 'left':
                self._append_pose_change_right_to_left()
                self.current_pose_state = 'left'

            for label, x, y in left_reversed:
                self._append_tying_actions(label, x, y)

            # Right 결속 전: 현재 Left 자세이면 Right로 전환 필요
            if right_reversed and self.current_pose_state == 'left':
                self._append_pose_change_left_to_right()
                self.current_pose_state = 'right'

            for label, x, y in right_reversed:
                self._append_tying_actions(label, x, y)

            # 최종 자세 상태 업데이트
            if right_reversed:
                self.current_pose_state = 'right'
            elif left_reversed:
                self.current_pose_state = 'left'

        else:
            # forward (기본): Right(P1→P3) → 자세변경(우→좌) → Left(P4→P6)
            self.action_queue.append(Action(
                action_type=ActionType.LOG,
                message='=== forward: Right(P1→P3) → 자세변경 → Left(P4→P6) ===',
            ))

            # Right 결속 전: 현재 Left 자세이면 Right로 전환 필요
            if right_pts and self.current_pose_state == 'left':
                self._append_pose_change_left_to_right()
                self.current_pose_state = 'right'

            for label, x, y in right_pts:
                self._append_tying_actions(label, x, y)

            # Left 결속 전: 현재 Right 자세이면 Left로 전환 필요
            if left_pts and self.current_pose_state != 'left':
                self._append_pose_change_right_to_left()
                self.current_pose_state = 'left'

            for label, x, y in left_pts:
                self._append_tying_actions(label, x, y)

            # 최종 자세 상태 업데이트
            if left_pts:
                self.current_pose_state = 'left'
            elif right_pts:
                self.current_pose_state = 'right'

        # 후처리: Z_UP + MOVE_XY 연속 패턴 → MOVE_Z_UP_WITH_XY로 합치기
        self._optimize_z_xy_overlap()

    def _optimize_z_xy_overlap(self):
        """Z_UP 직후 MOVE_XY가 오는 패턴을 MOVE_Z_UP_WITH_XY로 합침.

        이렇게 하면 Z 상승 중 일정 높이(z_early_xy_mm) 도달 시
        XY 이동을 미리 시작하여 사이클 타임을 줄입니다.
        """
        optimized = []
        i = 0
        merged_count = 0
        while i < len(self.action_queue):
            action = self.action_queue[i]
            # Z_UP 다음이 MOVE_XY이면 합치기
            if (action.action_type == ActionType.MOVE_Z_UP
                    and i + 1 < len(self.action_queue)
                    and self.action_queue[i + 1].action_type == ActionType.MOVE_XY):
                next_action = self.action_queue[i + 1]
                optimized.append(Action(
                    action_type=ActionType.MOVE_Z_UP_WITH_XY,
                    z_mm=action.z_mm,
                    x_mm=next_action.x_mm,
                    y_mm=next_action.y_mm,
                    point_label=action.point_label,
                    message=f'{action.point_label}: Z↑{action.z_mm}mm + '
                            f'{next_action.point_label} XY({next_action.x_mm:.1f},{next_action.y_mm:.1f})',
                ))
                i += 2  # 두 액션을 하나로
                merged_count += 1
            else:
                optimized.append(action)
                i += 1

        if merged_count > 0:
            self.action_queue = optimized
            self.get_logger().info(
                f'  [최적화] Z↑+XY 병합: {merged_count}개 (액션 {len(self.action_queue)}개)')

    def _append_tying_actions(self, label, x, y):
        """한 포인트에 대한 결속 액션 시퀀스 추가.

        XY이동 → Z하강(tying_z_down_mm) → 트리거 → Z상승(tying_z_down_mm)
        """
        # XY 이동
        self.action_queue.append(Action(
            action_type=ActionType.MOVE_XY,
            x_mm=x, y_mm=y,
            point_label=label,
            message=f'{label}: X={x:.1f}mm, Y={y:.1f}mm',
        ))
        # Z축 하강 (대기위치에서 추가 하강)
        self.action_queue.append(Action(
            action_type=ActionType.MOVE_Z_DOWN,
            z_mm=self.tying_z_down_mm,
            point_label=label,
            message=f'{label}: Z {self.tying_z_down_mm}mm 하강',
        ))
        # 트리거 동작
        self.action_queue.append(Action(
            action_type=ActionType.TRIGGER,
            point_label=label,
            message=f'{label}: 트리거',
        ))
        # Z축 상승 (대기위치로 복귀)
        self.action_queue.append(Action(
            action_type=ActionType.MOVE_Z_UP,
            z_mm=self.tying_z_down_mm,
            point_label=label,
            message=f'{label}: Z {self.tying_z_down_mm}mm 상승',
        ))

    def _append_pose_change_right_to_left(self):
        """P3→P4 자세 변경 시퀀스 (Right → Left 전환)

        1) X:홈, Y:near → 2) Yaw:mid → 3) Y:far → 4) Yaw:접근(390°) → 5) Yaw:left_cam(393°)
        """
        self.action_queue.append(Action(
            action_type=ActionType.LOG,
            message='=== 자세 변경: Right → Left ===',
        ))
        # Step 1: 안전 위치로 이동 (X→홈)
        self.action_queue.append(Action(
            action_type=ActionType.MOVE_XY,
            x_mm=self.x_home_mm, y_mm=self.pose_change_y_near,
            message=f'자세변경 1/5: X={self.x_home_mm:.1f}mm(홈), Y={self.pose_change_y_near}mm',
        ))
        # Step 2: Yaw 중간 경유
        self.action_queue.append(Action(
            action_type=ActionType.MOVE_YAW,
            yaw_deg=self.yaw_mid,
            message=f'자세변경 2/5: Yaw → {self.yaw_mid}°',
        ))
        # Step 3: Y 원거리 위치
        self.action_queue.append(Action(
            action_type=ActionType.MOVE_XY,
            x_mm=self.x_home_mm, y_mm=self.pose_change_y_far,
            message=f'자세변경 3/5: X={self.x_home_mm:.1f}mm(홈), Y → {self.pose_change_y_far}mm',
        ))
        # Step 4: Yaw Left cam 접근자세 (긴 거리, 오버슈트 방지)
        self.action_queue.append(Action(
            action_type=ActionType.MOVE_YAW,
            yaw_deg=self.yaw_left_cam_approach,
            message=f'자세변경 4/5: Yaw → {self.yaw_left_cam_approach}° (접근)',
        ))
        # Step 5: Yaw Left cam 최종자세 (짧은 거리, 정밀 접근)
        self.action_queue.append(Action(
            action_type=ActionType.MOVE_YAW,
            yaw_deg=self.yaw_left_cam,
            message=f'자세변경 5/5: Yaw → {self.yaw_left_cam}° (최종)',
        ))

    def _append_pose_change_left_to_right(self):
        """P6→P1 자세 복귀 시퀀스 (Left → Right 복귀)

        1) X:홈, Y:far → 2) Yaw:mid → 3) Y:near → 4) Yaw:홈+5° → 5) Yaw:right_cam
        """
        self.action_queue.append(Action(
            action_type=ActionType.LOG,
            message='=== 자세 복귀: Left → Right ===',
        ))
        # Step 1: 안전 위치로 이동 (X→홈)
        self.action_queue.append(Action(
            action_type=ActionType.MOVE_XY,
            x_mm=self.x_home_mm, y_mm=self.pose_change_y_far,
            message=f'자세복귀 1/5: X={self.x_home_mm:.1f}mm(홈), Y={self.pose_change_y_far}mm',
        ))
        # Step 2: Yaw 중간 경유
        self.action_queue.append(Action(
            action_type=ActionType.MOVE_YAW,
            yaw_deg=self.yaw_mid,
            message=f'자세복귀 2/5: Yaw → {self.yaw_mid}°',
        ))
        # Step 3: Y 근접 위치
        self.action_queue.append(Action(
            action_type=ActionType.MOVE_XY,
            x_mm=self.x_home_mm, y_mm=self.pose_change_y_near,
            message=f'자세복귀 3/5: X={self.x_home_mm:.1f}mm(홈), Y → {self.pose_change_y_near}mm',
        ))
        # Step 4: Yaw 경유 (홈+5°, 이동량 감소)
        yaw_near_home = self.home_ref_yaw_deg + 5.0
        self.action_queue.append(Action(
            action_type=ActionType.MOVE_YAW,
            yaw_deg=yaw_near_home,
            message=f'자세복귀 4/5: Yaw → {yaw_near_home:.1f}° (홈+5°)',
        ))
        # Step 5: Yaw Right cam 자세
        self.action_queue.append(Action(
            action_type=ActionType.MOVE_YAW,
            yaw_deg=self.yaw_right_cam,
            message=f'자세복귀 5/5: Yaw → {self.yaw_right_cam}°',
        ))

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

    # ============================================
    # 완료 / 에러 / 취소 처리
    # ============================================
    def _handle_complete(self):
        """결속 시퀀스 완료"""
        # 반복모드 전진 패스 완료 → 복귀 패스 재검출 시작
        if self.repeat_mode and self.tying_pass == 'forward':
            forward_completed = self.completed_points
            self.get_logger().info('=' * 60)
            self.get_logger().info(
                f'[REPEAT] 전진 패스 완료 ({forward_completed}개 체결), '
                f'복귀 패스 재검출 시작')
            self.get_logger().info('=' * 60)
            self.tying_pass = 'return'
            self.tying_direction = 'reverse'  # return 패스는 reverse 방향
            self._forward_completed_points = forward_completed
            # 검출 상태 초기화 (재검출)
            self.action_queue = []
            self.current_action_index = 0
            self.detection_future = None
            self.detection_retry = 0
            self.multi_detect_count = 0
            self.accumulated_points = []
            self.multi_detect_wait_until = 0.0
            self._detect_camera_phase = 'right'
            self.tying_message = '복귀 패스 교차점 멀티검출 중...'
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

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
from rclpy.qos import qos_profile_sensor_data
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
        self.declare_parameter('detect_x_min_mm', 30.0)   # 검출 X 최소값(이하 클러스터 제거)
        self.declare_parameter('max_yaw_deg', 399.0)
        # Y축 자세별 가동 범위 (2026-03-18 실측)
        self.declare_parameter('right_cam_y_min_mm', 0.0)
        self.declare_parameter('right_cam_y_max_mm', 142.0)
        self.declare_parameter('left_cam_y_min_mm', 124.0)
        self.declare_parameter('left_cam_y_max_mm', 288.0)
        # 좌측자세 x 하한(mm): 저-X 결속점이 검출오차로 배근에 부딪혀 스킵 (0=필터 없음)
        self.declare_parameter('left_min_x_mm', 100.0)
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
        # 자세변경 직후(X가 0까지 줄었다 증가) 첫 포인트의 X 백래쉬 보정량(mm)
        self.declare_parameter('pose_change_backlash_x_mm', 5.0)
        self.declare_parameter('yaw_settling_time', 2.0)
        # Z축 결속 파라미터
        self.declare_parameter('z_home_offset_mm', 100.0)      # 호밍 후 Z 대기위치 (홈에서 하강)
        self.declare_parameter('ready_pose_x_mm', 300.0)       # 검출자세 X (Orbbec: 0=홈)
        self.declare_parameter('ready_pose_y_mm', 120.0)       # 우측 검출자세 Y (Orbbec: 100)
        self.declare_parameter('detect_left_y_mm', 160.0)      # 좌측 검출자세 Y (Orbbec, Y≥124)
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
        # 카메라 모드: 1=양카메라(자세변경 후 Phase2), 2=우측카메라 전용(자세변경 없음)
        self.declare_parameter('camera_mode', 1)
        # Z축 모션 스킵(테스트용): true면 결속 시퀀스에서 Z 하강/상승 생략(XY·yaw·트리거만)
        self.declare_parameter('skip_z_motion', False)
        self.declare_parameter('rebar_spacing_mm', 195.0)      # 철근 배근 간격 (mm)

        # 파라미터 로드
        self.deg_per_mm_x = self.get_parameter('deg_per_mm_x').value
        self.deg_per_mm_y = self.get_parameter('deg_per_mm_y').value
        self.stage_max_speed = self.get_parameter('stage_max_speed_dps').value
        self.stage_settling_time = self.get_parameter('stage_settling_time').value
        self.max_stage_timeout = self.get_parameter('max_stage_timeout').value
        self.position_tolerance = self.get_parameter('position_tolerance_deg').value
        self.max_stage_x_mm = self.get_parameter('max_stage_x_mm').value
        self.detect_x_min_mm = self.get_parameter('detect_x_min_mm').value
        self.max_stage_y_mm = self.get_parameter('max_stage_y_mm').value
        self.max_yaw_deg = self.get_parameter('max_yaw_deg').value
        self.right_cam_y_min = self.get_parameter('right_cam_y_min_mm').value
        self.right_cam_y_max = self.get_parameter('right_cam_y_max_mm').value
        self.left_cam_y_min = self.get_parameter('left_cam_y_min_mm').value
        self.left_cam_y_max = self.get_parameter('left_cam_y_max_mm').value
        self.left_min_x_mm = self.get_parameter('left_min_x_mm').value
        self.inter_point_delay = self.get_parameter('inter_point_delay').value
        self.detection_retry_count = self.get_parameter('detection_retry_count').value
        self.detection_retry_delay = self.get_parameter('detection_retry_delay').value
        self.yaw_right_cam_offset = self.get_parameter('yaw_right_cam_offset').value
        self.yaw_left_cam_offset = self.get_parameter('yaw_left_cam_offset').value
        self.yaw_left_cam_approach_offset = self.get_parameter('yaw_left_cam_approach_offset').value
        self.yaw_mid_offset = self.get_parameter('yaw_mid_offset').value
        self.pose_change_y_near = self.get_parameter('pose_change_y_near_mm').value
        self.pose_change_y_far = self.get_parameter('pose_change_y_far_mm').value
        self.pose_change_backlash_x_mm = self.get_parameter('pose_change_backlash_x_mm').value
        self.yaw_settling_time = self.get_parameter('yaw_settling_time').value
        self.z_home_offset_mm = self.get_parameter('z_home_offset_mm').value
        self.ready_pose_x_mm = self.get_parameter('ready_pose_x_mm').value
        self.ready_pose_y_mm = self.get_parameter('ready_pose_y_mm').value
        self.detect_left_y_mm = self.get_parameter('detect_left_y_mm').value
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
        self.camera_mode = int(self.get_parameter('camera_mode').value)
        self.skip_z_motion = bool(self.get_parameter('skip_z_motion').value)
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
        self._pose_change_only = False   # CHN_POS 단독 자세변경(완료 시 Phase2 안 타고 IDLE 종료)
        self._servo_single = False       # [Stage2] SERVO_ONE 단일점 서보결속(완료 시 IDLE)
        self._servo_running = False      # [Stage2] 서보 스레드 중복 방지 (핸들러서 즉시 세팅)
        self._servo_abort = False        # [Stage3] 스캔 중단 플래그 (CANCEL 시 True)
        self._servo_session_tied = 0     # [통합] 서보결속 세션 누적 결속수 (Phase1+2 합산)
        self._servo_session_total = 0    # [통합] 서보결속 세션 누적 포인트수

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
            'detect_x_min_mm': self.detect_x_min_mm,
            'right_cam_y_min': self.right_cam_y_min,
            'right_cam_y_max': self.right_cam_y_max,
            'left_cam_y_min': self.left_cam_y_min,
            'left_cam_y_max': self.left_cam_y_max,
        })

        # === [Stage1] 툴캠 서보 미세보정 인프라 (servo_mode일 때만 로드) ===
        self.declare_parameter('servo_mode', False)
        self.servo_mode = self.get_parameter('servo_mode').value
        self.declare_parameter('scan_velocity_dps', 80.0)  # [Stage3] 연속스캔 속도
        self.scan_velocity = self.get_parameter('scan_velocity_dps').value
        # 좌측자세 서보 코스이동 Y (우측=ready_pose_y, 좌측=이 값. 좌측 캘리브 시 튜닝)
        self.declare_parameter('servo_ready_y_left_mm', 168.0)
        self.servo_ready_y_left = self.get_parameter('servo_ready_y_left_mm').value
        # pose별 툴캠 목표픽셀/게인 (right/left). _init_servo에서 파일 로드해 채움.
        # self.toolcam_target/self.toolcam_Jinv = 현재 활성 pose값 (_select_pose_servo가 세팅)
        self.toolcam_targets = {}   # {'right': np.array([u,v]), 'left': ...}
        self.toolcam_Jinvs = {}     # {'right': 2x2, 'left': ...}
        self.toolcam_target = np.array([694.0, 468.0])  # 활성 목표픽셀 (기본 우측)
        self.toolcam_buf = []
        self.toolcam_model = None
        self.toolcam_Jinv = None
        if self.servo_mode:
            self._init_servo()

        # === [Orbbec] 교차점 검출+호모그래피 결속 (orbbec_mode) ===
        # 상단 Orbbec 1대로 전영역 검출 → 호모그래피로 로봇XY → 지그재그 직접결속
        # (비주얼서보잉 대체). 기존 zedxmini/서보 흐름은 보존.
        self.declare_parameter('orbbec_mode', False)
        self.orbbec_mode = self.get_parameter('orbbec_mode').value
        # WP 도달 직후 base 정착 대기 (주행 정지 관성 → 블러/위치오차 방지)
        self.declare_parameter('orbbec_settle_sec', 0.8)
        self.orbbec_settle_sec = self.get_parameter('orbbec_settle_sec').value
        self.orbbec = None
        self._orbbec_running = False   # runner 스레드 중복 방지
        if self.orbbec_mode:
            self._init_orbbec()

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
        cam_mode_str = '2(우측전용·자세변경X)' if self.camera_mode == 2 else '1(양카메라·자세변경)'
        self.get_logger().info(f'  Camera mode: {cam_mode_str}')
        if self.skip_z_motion and not getattr(self, "_force_z", False):
            self.get_logger().warn('  ⚠️ skip_z_motion=ON → Z 하강/상승 생략(시연용, 실제 결속 안 됨)')
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
            if self._servo_running or self._orbbec_running:
                self.get_logger().warn('TYING_START 무시: 결속 진행중')
                return
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
                # 사전지정 결속포인트 [[x,y],...] mm (있으면 검출 대신 사용, Z 강제 ON)
                self._predefined_points = data.get('points', []) or []
                self.get_logger().info('=' * 60)
                self.get_logger().info(
                    f'[UI] TYING_START 수신 - 속도 {speed_pct}% '
                    f'(XY:{self.effective_stage_speed:.0f}dps, '
                    f'Z:{self.effective_z_speed:.0f}dps)'
                    f' [방향: {self.tying_direction}]'
                    f'{" [반복모드]" if self.repeat_mode else ""}')
                self.get_logger().info('=' * 60)
                self.flog(f"TYING_START: speed={speed_pct}% direction={self.tying_direction}")
                if self.orbbec_mode:
                    # [Orbbec] 지그재그 직접결속 (검출→호모그래피→결속, 서보 없음)
                    self._orbbec_running = True
                    self._servo_abort = False
                    self._force_z = True
                    import threading
                    threading.Thread(target=self._run_orbbec_tying, daemon=True).start()
                else:
                    self._start_detection()
            else:
                self.get_logger().warn(
                    f'TYING_START 무시: 현재 상태 {self.state.name}')

        elif cmd == 'SERVO_ONE':
            # [Stage2] 단일점 툴캠 서보결속: 현재 위치에서 정렬 → 결속 (스캔 없음)
            if not self.servo_mode:
                self.get_logger().warn('SERVO_ONE 무시: servo_mode=false'); return
            if self._servo_running:
                self.get_logger().warn('SERVO_ONE 무시: 서보 이미 진행중'); return
            if self.state != TyingState.IDLE:
                self.get_logger().warn(f'SERVO_ONE 무시: 상태 {self.state.name}'); return
            if self.home_ref_x_deg is None:
                self.get_logger().error('SERVO_ONE 실패: 호밍 미수행'); return
            speed_pct = max(1, min(100, int(data.get('speed', 50))))
            self.speed_percent = float(speed_pct)
            self.effective_stage_speed = self.stage_max_speed * speed_pct / 100.0
            self.effective_z_speed = self.z_speed * speed_pct / 100.0
            self._force_z = True   # 실제 Z 결속 (skip_z_motion 무시)
            self._servo_running = True   # 중복 방지 (스레드 시작 전 즉시 세팅)
            self.get_logger().info('[UI] SERVO_ONE 수신 - 단일점 서보결속')
            import threading
            threading.Thread(target=self._run_servo_one, daemon=True).start()

        elif cmd == 'SERVO_SCAN':
            # [Stage3a] Y=50 고정, X를 to까지 연속스캔하며 검출→서보결속
            self.flog(f"SERVO_SCAN 체크: servo_mode={self.servo_mode} "
                      f"state={self.state.name} home_ref={self.home_ref_x_deg}")
            if not self.servo_mode:
                self.get_logger().warn('SERVO_SCAN 무시: servo_mode=false')
                self.flog('SERVO_SCAN 무시: servo_mode=false'); return
            if self.state != TyingState.IDLE:
                self.get_logger().warn(f'SERVO_SCAN 무시: 상태 {self.state.name}')
                self.flog(f'SERVO_SCAN 무시: state={self.state.name}'); return
            if self.home_ref_x_deg is None:
                self.get_logger().error('SERVO_SCAN 실패: 호밍 미수행')
                self.flog('SERVO_SCAN 실패: 호밍 미수행'); return
            speed_pct = max(1, min(100, int(data.get('speed', 50))))
            self.speed_percent = float(speed_pct)
            self.effective_stage_speed = self.stage_max_speed * speed_pct / 100.0
            self.effective_z_speed = self.z_speed * speed_pct / 100.0
            self._force_z = True
            self._servo_abort = False
            scan_to = float(data.get('to', 0.0))     # 스캔 목표 X(mm)
            scan_y = float(data.get('y', 50.0))      # 스캔행 Y(mm)
            self.get_logger().info(f'[UI] SERVO_SCAN 수신 → X={scan_to} Y={scan_y}')
            self.flog(f'SERVO_SCAN 스레드 시작 → X={scan_to} Y={scan_y}')
            import threading
            threading.Thread(target=self._run_servo_scan,
                             args=(scan_to, scan_y), daemon=True).start()

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
            self._servo_abort = True   # [Stage3] 스캔 스레드 중단 신호
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
            # 자세변경 액션 생성 (Phase1→2 전환과 동일하게 action_builder 사용)
            self.action_builder.update_params(
                x_home_mm=self.x_home_mm,
                yaw_right_cam=self.yaw_right_cam,
                yaw_left_cam=self.yaw_left_cam,
                yaw_left_cam_approach=self.yaw_left_cam_approach,
                yaw_mid=self.yaw_mid,
                home_ref_yaw_deg=self.home_ref_yaw_deg,
            )
            self.action_queue = self.action_builder.build_pose_change(
                self.current_pose_state, target_pose)
            self.current_pose_state = target_pose
            # 단독 자세변경: 완료 시 Phase2 검출로 넘어가지 않고 IDLE로 종료
            self._pose_change_only = True
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

                # 검출자세 이동 (X,Y,Z 같이). homing ready 명령과의 race 회피 위해
                # 짧은 지연 후 별도 스레드에서 Z하강+XY를 함께 보내고 도달까지 대기 → 호밍완료.
                import threading
                threading.Thread(target=self._goto_detect_pose_after_homing,
                                 daemon=True).start()
            except Exception as e:
                self.get_logger().warn(f'호밍 레퍼런스 파싱 실패: {e}')

    def _goto_detect_pose_after_homing(self):
        """호밍 완료 후 검출자세로 이동 (Z를 XY와 같이, 도달까지 대기 → 호밍완료).
        homing ready 명령 정착 후(지연) 실행해 Z 명령 race 회피. Z 유닛 기존 유지."""
        time.sleep(0.6)   # homing ready 명령 정착 대기 (Z→home 명령과 충돌 회피)
        self.effective_stage_speed = self.stage_max_speed * 0.5
        self.effective_z_speed = self.z_speed * 0.5
        z_offset_deg = self.z_home_offset_mm * self.deg_per_mm_z
        self._send_z_relative(z_offset_deg)     # 홈서 z_home_offset 하강 (양수=하강)
        time.sleep(0.05)
        self._send_xy_absolute(self.ready_pose_x_mm, self.ready_pose_y_mm)
        self.get_logger().info(
            f'검출자세 이동: X{self.ready_pose_x_mm} Y{self.ready_pose_y_mm} '
            f'Z+{self.z_home_offset_mm}mm({z_offset_deg:.0f}°)')
        self.flog(f'[호밍] 검출자세 이동 X{self.ready_pose_x_mm} '
                  f'Y{self.ready_pose_y_mm} Z+{z_offset_deg:.0f}°')
        # 검출자세(XY) 도달까지 대기 → 호밍 완료
        self._servo_wait_arrival(self.ready_pose_x_mm, self.ready_pose_y_mm,
                                 timeout=15.0)
        self.get_logger().info('[호밍] 검출자세 도달 → 호밍 완료')
        self.flog('[호밍] 검출자세 도달 = 호밍완료')

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
        self._servo_session_tied = 0     # 서보결속 세션 누적 리셋 (새 결속 시작)
        self._servo_session_total = 0
        self.tying_message = '교차점 멀티검출 중...'
        self.tying_result = ''

        # 사전지정 결속포인트가 있으면 검출 스킵하고 그 좌표로 결속 (Z 강제 ON)
        self._force_z = False
        if getattr(self, '_predefined_points', None):
            self._start_predefined_tying(self._predefined_points)
            return

        # 2단계 검출: 현재 자세에 맞는 카메라만 검출
        self.tying_phase = 1
        self._pose_change_only = False  # 결속 시작 시 단독 자세변경 플래그 해제
        if self.camera_mode == 2:
            cam_side = 'right'  # 우측카메라 전용 모드: 자세 무관 항상 우측
        elif self.current_pose_state == 'left':
            cam_side = 'left'
        else:
            cam_side = 'right'
        self.get_logger().info(
            f'[Phase 1] {cam_side} 카메라 검출 시작 (현재 자세: {self.current_pose_state})')
        self.flog(f"PHASE1_START: pose={self.current_pose_state} cam={cam_side}")
        self.detection_mgr.start(camera_side=cam_side)
        self._transition_to(TyingState.DETECTING)
        self._publish_tying_feedback()

    def _start_predefined_tying(self, points):
        """사전지정 결속포인트로 결속 (카메라 검출 스킵, Z 강제 ON)."""
        self._force_z = True  # 이번 결속은 전역 skip_z_motion 무시하고 Z 수행
        pts = [(f'P{i+1}', float(p[0]), float(p[1]), 'right')
               for i, p in enumerate(points) if len(p) >= 2]
        # 웨이포인트 방향으로 X 정렬 (지그재그)
        big_to_small = (self.tying_direction == 'forward')
        pts.sort(key=lambda t: t[1], reverse=big_to_small)
        pts = [(f'P{i+1}', x, y, s) for i, (_, x, y, s) in enumerate(pts)]
        self.total_points = len(pts)
        self.action_builder.update_params(
            x_home_mm=self.x_home_mm, yaw_right_cam=self.yaw_right_cam,
            yaw_left_cam=self.yaw_left_cam,
            yaw_left_cam_approach=self.yaw_left_cam_approach,
            yaw_mid=self.yaw_mid, home_ref_yaw_deg=self.home_ref_yaw_deg)
        self.action_queue = []
        for label, x, y, _s in pts:
            self.action_queue.extend(self.action_builder._tying_actions(label, x, y))
        self.action_queue = self.action_builder._optimize_z_xy_overlap(
            self.action_queue, logger=self.get_logger())
        self.current_action_index = 0
        self.tying_phase = 1
        self.get_logger().info(f'[사전지정] {len(pts)}개 포인트 결속 (검출 스킵, Z ON)')
        self.flog(f"PREDEFINED_TYING: {[(round(x), round(y)) for _, x, y, _s in pts]}")
        self._transition_to(TyingState.EXECUTING_ACTION)

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

        # [통합] servo_mode: 툴캠 서보결속 흐름 (스캔 폐기, 검출X→코스이동→서보→결속)
        # servo_mode=false면 아래 기존 액션큐 흐름 그대로 사용 (보존).
        if self.servo_mode and points:
            self._servo_running = True
            self._servo_abort = False
            import threading
            threading.Thread(target=self._run_servo_detected,
                             args=(points,), daemon=True).start()
            self.get_logger().info(f'[SERVO_TIE] servo_mode 서보결속 흐름 시작 ({len(points)}개)')
            self.flog(f'[SERVO_TIE] servo_mode 흐름 시작 ({len(points)}개)')
            self._transition_to(TyingState.IDLE)   # 이후 서보 스레드가 관리
            return

        self.total_points += len(points)

        # 결속 순서 결정
        if self.camera_mode == 2:
            # 우측전용: 웨이포인트 방향으로 X정렬 교대 (지그재그, 이동 최소화)
            #   forward=X큰→작은, reverse=X작은→큰 (rebar_controller가 WP마다 교대 전송)
            big_to_small = (self.tying_direction == 'forward')
            points.sort(key=lambda p: p[1], reverse=big_to_small)
        elif self.tying_phase == 1:
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
            if self.skip_z_motion and not getattr(self, "_force_z", False):
                self.get_logger().info(f'  Z하강 SKIP (skip_z_motion): {action.message}')
                self._advance_to_next_action()
                return
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
            if self.skip_z_motion and not getattr(self, "_force_z", False):
                self.get_logger().info(f'  Z상승 SKIP (skip_z_motion): {action.message}')
                self._advance_to_next_action()
                return
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
            if self.skip_z_motion and not getattr(self, "_force_z", False):
                # Z 스킵 → XY만 이동
                self._send_xy_absolute(action.x_mm, action.y_mm)
                self.get_logger().info(f'  Z↑+XY → XY만(Z skip): {action.message}')
                self.goal_targets = {
                    0x44: self.home_ref_x_deg - action.x_mm * self.deg_per_mm_x,
                    0x45: self.home_ref_y_deg + action.y_mm * self.deg_per_mm_y,
                }
                self.tying_message = action.message
                self._publish_tying_feedback()
                self._transition_to(TyingState.WAITING_SETTLE)
                return
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

    # ============================================
    # [Stage1] 툴캠 서보 미세보정 (servo_mode)
    # ============================================
    def _init_servo(self):
        """툴캠 YOLO + pose별 게인/목표픽셀 로드 + zedxone 구독. 실패 시 servo_mode off.

        pose별 게인 파일: toolcam_gain_{right,left}.yaml (각 Jinv + target_pixel).
        우측 파일 없으면 구버전 toolcam_gain.yaml을 우측으로 fallback.
        좌측 파일 없으면 좌측 서보는 미가용(경고) — 우측만 동작.
        """
        import os
        try:
            from sensor_msgs.msg import Image as _Img
            from cv_bridge import CvBridge
            from ultralytics import YOLO
            import yaml
            self._cvbridge = CvBridge()
            self.toolcam_model = YOLO(
                '/home/koceti/ros2_ws/src/rebar_vision/model/toolcam_crossing.pt')
            cdir = '/home/koceti/ros2_ws/data/calibration'
            for pose in ('right', 'left'):
                path = os.path.join(cdir, f'toolcam_gain_{pose}.yaml')
                if not os.path.exists(path) and pose == 'right':
                    path = os.path.join(cdir, 'toolcam_gain.yaml')  # 구버전 fallback
                if not os.path.exists(path):
                    self.get_logger().warn(
                        f'  [servo] {pose} 게인파일 없음 → {pose} 자세 서보 미가용 '
                        f'(캘리브 후 toolcam_gain_{pose}.yaml 생성 필요)')
                    continue
                g = yaml.safe_load(open(path))['toolcam_gain']
                self.toolcam_Jinvs[pose] = np.array(g['Jinv_mm_per_px'])
                self.toolcam_targets[pose] = np.array(g['target_pixel'], dtype=float)
                self.get_logger().info(
                    f'  [servo] {pose} 게인 로드: target={g["target_pixel"]} '
                    f'({os.path.basename(path)})')
            if not self.toolcam_Jinvs:
                raise RuntimeError('로드된 게인 파일 없음')
            # 기본 활성 pose = 우측(있으면), 없으면 로드된 첫 pose
            if not self._select_pose_servo('right'):
                first = next(iter(self.toolcam_Jinvs))
                self.toolcam_Jinv = self.toolcam_Jinvs[first]
                self.toolcam_target = self.toolcam_targets[first]
            self.create_subscription(
                _Img, '/zedxone/zed_node/rgb/rect/image',
                self._toolcam_cb, qos_profile_sensor_data)
            self.get_logger().info(
                f'  [servo] 툴캠 YOLO+게인 로드 ✅ (servo_mode ON, '
                f'pose={list(self.toolcam_Jinvs.keys())})')
        except Exception as e:
            self.get_logger().error(f'  [servo] 초기화 실패 → servo_mode OFF: {e}')
            self.servo_mode = False

    def _select_pose_servo(self, pose):
        """현재 자세에 맞는 툴캠 목표픽셀/게인을 활성화. 미가용이면 False."""
        pose = pose if pose in ('right', 'left') else 'right'
        if pose not in self.toolcam_Jinvs:
            # 요청 pose 미가용 → 가용한 pose 중 하나라도 있으면 유지, 없으면 False
            return False
        self.toolcam_Jinv = self.toolcam_Jinvs[pose]
        self.toolcam_target = self.toolcam_targets[pose]
        return True

    def _toolcam_cb(self, msg):
        img = self._cvbridge.imgmsg_to_cv2(msg, 'bgr8')
        self.toolcam_buf.append(img)
        if len(self.toolcam_buf) > 12:
            self.toolcam_buf.pop(0)

    def _toolcam_detect(self, n=8, near=None):
        """툴캠 교차점 검출 (n샘플 중앙값). near 주어지면 그 점 최근접 교차점
        (서보 중 한 교차점 추적용), 없으면 목표 최근접. 별도 스레드에서 호출."""
        ref = near if near is not None else self.toolcam_target
        pts = []
        for _ in range(n):
            if not self.toolcam_buf:
                time.sleep(0.1); continue
            frame = self.toolcam_buf[-1]
            r = self.toolcam_model(frame, conf=0.15, verbose=False)[0]
            cs = [(float(b.xyxy[0][0]+b.xyxy[0][2])/2,
                   float(b.xyxy[0][1]+b.xyxy[0][3])/2) for b in r.boxes]
            if cs:
                cs.sort(key=lambda c: np.hypot(c[0]-ref[0], c[1]-ref[1]))
                pts.append(cs[0])
            time.sleep(0.06)
        if len(pts) < 2:
            return None
        a = np.array(pts); med = np.median(a, 0)
        keep = a[np.linalg.norm(a - med, axis=1) < 40]
        return keep.mean(0) if len(keep) else med

    def _servo_align(self, max_iter=12, tol_px=15.0, gain=0.8, max_step_mm=35.0):
        """검출→ΔXY=Jinv·오차→XY이동→반복. 별도 스레드에서 호출.
        검출 놓치면(가려짐/지나침) 직전 성공 위치로 스텝백 + 스텝 절반 재이동.
        반환 (converged: bool, final_err_px: float|None)."""
        if self.home_ref_x_deg is None:
            self.get_logger().error('  [servo] 호밍 레퍼런스 없음'); return False, None
        epx = None
        track = None       # 잠근 교차점 추적 (격자 갈아타기 방지)
        prev_xy = None     # 마지막 검출 성공 스테이지 위치 (스텝백 복귀 목적지)
        last_cross = None  # 마지막 검출 성공 픽셀
        target_est = None  # 예측: 교차점을 목표픽셀에 놓을 스테이지 위치 (검출마다 갱신)
        step_scale = 1.0   # 스텝백 시 절반씩 감소 (2회: 1.0→0.5→0.25)
        for it in range(max_iter):
            # 직전 교차점에서 너무 멀리 튄 검출(=다른 격자점)은 거부하고 재검출
            cross = None
            for _ in range(4):
                c = self._toolcam_detect(near=track, n=5)
                if c is None:
                    continue
                if track is None or float(np.hypot(c[0]-track[0], c[1]-track[1])) < 300:
                    cross = c; break
                self.flog(f'  [s{it+1}] 점프거부({c[0]:.0f},{c[1]:.0f}) '
                          f'track({track[0]:.0f},{track[1]:.0f})')
            if cross is None:
                # 스텝백 2회(1.0→0.5→0.25): 직전 성공 위치로 복귀 + 스텝 절반 후 재시도
                # (더 작은 스텝으로 가림경계에 더 가까이 검출 → 예측 정밀도↑)
                if prev_xy is not None and step_scale > 0.25:
                    step_scale *= 0.5
                    self.get_logger().warn(
                        f'  [servo {it+1}] 검출놓침 → 스텝백 복귀 (스텝 x{step_scale:.2f})')
                    self.flog(f'  [s{it+1}] 검출놓침 → 스텝백 복귀'
                              f'({prev_xy[0]:.0f},{prev_xy[1]:.0f}) 스텝x{step_scale:.2f}')
                    self._send_xy_absolute(prev_xy[0], prev_xy[1])
                    self._servo_wait_arrival(prev_xy[0], prev_xy[1])
                    time.sleep(0.5); self.toolcam_buf.clear(); time.sleep(0.3)
                    continue
                # 2회 스텝백 후에도 놓침: 예측 지점(교차점을 목표에 놓을 위치)으로 오픈루프 결속
                # target_est = 검출마다 갱신된 예측 = 마지막 검출 기준 목표정렬 위치
                if target_est is not None and epx is not None and epx < tol_px * 15:
                    nx, ny = target_est
                    self.get_logger().info(
                        f'  [servo {it+1}] 2회 스텝백후 놓침(마지막 {epx:.0f}px) → 예측지점 오픈루프 결속')
                    self.flog(f'  [s{it+1}] 2회스텝백후놓침{epx:.0f}px → 예측지점({nx:.0f},{ny:.0f}) 오픈루프 결속')
                    self._send_xy_absolute(nx, ny)
                    self._servo_wait_arrival(nx, ny)
                    return True, epx
                # 목표서 먼데 놓침(=진짜 문제) → 실패
                self.get_logger().warn(f'  [servo {it+1}] 추적실패 (목표원거리 best={epx})')
                self.flog(f'  [s{it+1}] 추적실패 (목표원거리 best={epx})')
                self._save_servo_debug(it+1, track)
                return False, epx
            err = self.toolcam_target - cross
            epx = float(np.hypot(*err))
            track = cross   # 다음 반복은 이 교차점 추적 (격자 갈아타기 방지)
            self.get_logger().info(
                f'  [servo {it+1}] 교차점=({cross[0]:.0f},{cross[1]:.0f}) '
                f'오차={epx:.0f}px')
            self.flog(f'  [servo {it+1}] 교차점({cross[0]:.0f},{cross[1]:.0f}) '
                      f'오차{epx:.0f}px 스텝x{step_scale:.2f}')
            if epx < tol_px:
                self.get_logger().info(f'  [servo] ✅ 수렴 ({epx:.0f}px)')
                return True, epx
            dxy = gain * step_scale * (self.toolcam_Jinv @ err)  # mm (스텝백 시 축소)
            mag = float(np.hypot(*dxy))
            if mag > max_step_mm:
                dxy = dxy / mag * max_step_mm
            if 0x44 not in self.motor_positions or 0x45 not in self.motor_positions:
                self.get_logger().warn('  [servo] 모터위치 미수신'); return False, epx
            cur_x_mm = (self.home_ref_x_deg - self.motor_positions[0x44]) / self.deg_per_mm_x
            cur_y_mm = (self.motor_positions[0x45] - self.home_ref_y_deg) / self.deg_per_mm_y
            prev_xy = (cur_x_mm, cur_y_mm)   # 이 위치는 검출 성공 → 스텝백 복귀 기준
            last_cross = cross               # 이 위치서 본 교차점 픽셀
            # 예측 갱신: 이 검출 기준, 교차점을 목표픽셀에 놓을 스테이지 위치 (게인)
            ol_est = self.toolcam_Jinv @ (self.toolcam_target - cross)  # full gain
            target_est = (min(max(cur_x_mm + ol_est[0], 0.0), self.max_stage_x_mm),
                          min(max(cur_y_mm + ol_est[1], 0.0), self.max_stage_y_mm))
            new_x_mm = min(max(cur_x_mm + dxy[0], 0.0), self.max_stage_x_mm)
            new_y_mm = min(max(cur_y_mm + dxy[1], 0.0), self.max_stage_y_mm)
            self.flog(f'    [s{it+1}] cur({cur_x_mm:.0f},{cur_y_mm:.0f}) '
                      f'dxy({dxy[0]:+.1f},{dxy[1]:+.1f}) cmd({new_x_mm:.0f},{new_y_mm:.0f})')
            self._send_xy_absolute(new_x_mm, new_y_mm)
            self._servo_wait_arrival(new_x_mm, new_y_mm)
            act = self._servo_cur_xy_mm()
            if act:
                self.flog(f'    [s{it+1}] 실제도착({act[0]:.0f},{act[1]:.0f})')
            # 모션블러 방지: 정지 진동 안정 → 이동중 찍힌 흐린 프레임 버림 →
            # 선명한 새 프레임 채워지길 대기 (목표근처 교차점 미검출 원인 제거)
            time.sleep(0.5)
            self.toolcam_buf.clear()
            time.sleep(0.3)
        return False, epx

    def _servo_wait_arrival(self, x_mm, y_mm, tol_deg=1.0, timeout=6.0):
        """XY 도착 대기 (모터위치 기준). 메인 spin이 motor_positions 갱신."""
        x_deg = self.home_ref_x_deg - x_mm * self.deg_per_mm_x
        y_deg = self.home_ref_y_deg + y_mm * self.deg_per_mm_y
        t = time.time()
        while time.time() - t < timeout:
            if (abs(self.motor_positions.get(0x44, 1e9) - x_deg) < tol_deg and
                    abs(self.motor_positions.get(0x45, 1e9) - y_deg) < tol_deg):
                return True
            time.sleep(0.1)
        return False

    def _run_servo_one(self):
        """[Stage2] 단일점 서보결속 (별도 스레드): 정렬 → 결속 액션 주입.
        결속(Z하강/트리거/Z상승)은 기존 상태머신+z_monitor 재사용 (안전)."""
        try:
            pose = self.current_pose_state
            if not self._select_pose_servo(pose):
                self.get_logger().error(
                    f'[SERVO_ONE] {pose} 자세 게인 미가용 (toolcam_gain_{pose}.yaml 필요)')
                return
            self.get_logger().info(f'[SERVO_ONE] 서보 정렬 시작 (자세={pose})')
            converged, err = self._servo_align()
            if not converged:
                self.get_logger().warn(
                    f'[SERVO_ONE] 정렬 미수렴(오차 {err}) → 결속 안 함')
                return
            self.get_logger().info(f'[SERVO_ONE] 정렬 완료({err:.0f}px) → 결속')
            self._servo_tie_blocking()
            self.get_logger().info('[SERVO_ONE] 종료')
        finally:
            self._servo_running = False   # 성공/실패 모두 플래그 해제

    def _run_servo_detected(self, points):
        """[통합] zedxmini 검출 크로싱 순회: (검출X, 코스Y, Z유지) 코스이동 → 툴캠서보 → 결속.
        스캔(_run_servo_scan) 대신 검출 X로 직접 이동. Y=코스Y 고정(서보가 XY 미세정렬).
        SERVO_ONE 로직(서보정렬+결속)을 크로싱마다 반복. 별도 스레드에서 호출.

        pose별: 현재 자세(right/left)에 맞는 목표픽셀/게인 선택 + 코스Y 선택.
        camera_mode=1은 완료 시 COMPLETE로 넘겨 자세변경+Phase2를 기존 상태머신이 처리.
        """
        tied = 0
        n_pts = 0
        try:
            pose = self.current_pose_state
            if not self._select_pose_servo(pose):
                self.get_logger().error(
                    f'[SERVO_TIE] {pose} 자세 게인 미가용 → 결속 불가 '
                    f'(toolcam_gain_{pose}.yaml 캘리브 필요)')
                self.flog(f'[SERVO_TIE] {pose} 게인 미가용 → 중단')
                return
            # pose별 코스 이동 Y (우측=ready_pose_y, 좌측=servo_ready_y_left)
            coarse_y = (self.servo_ready_y_left if pose == 'left'
                        else self.ready_pose_y_mm)
            # 이동 최소화: 방향 기반 X 정렬 (forward=X큰→작은)
            pts = sorted(points, key=lambda p: p[1],
                         reverse=(self.tying_direction == 'forward'))
            n_pts = len(pts)
            self.get_logger().info(
                f'[SERVO_TIE] {len(pts)}개 크로싱 서보결속 시작 '
                f'(자세={pose}, 코스Y={coarse_y:.0f}, 목표px={self.toolcam_target})')
            self.flog(f'[SERVO_TIE] 시작 {len(pts)}개 (방향={self.tying_direction}, '
                      f'자세={pose}, 코스Y={coarse_y:.0f})')
            for label, x, y, side in pts:
                if self._servo_abort:
                    self.flog('[SERVO_TIE] 중단(abort)'); break
                # 코스 이동: 검출 X + 코스 Y (Z는 준비자세 유지, 서보가 XY 미세정렬)
                self.get_logger().info(
                    f'[SERVO_TIE] {label} 코스이동 → X={x:.0f}, Y={coarse_y:.0f}')
                self.flog(f'[SERVO_TIE] {label} 코스이동 X{x:.0f} Y{coarse_y:.0f}')
                self._send_xy_absolute(x, coarse_y)
                self._servo_wait_arrival(x, coarse_y)
                time.sleep(0.5); self.toolcam_buf.clear(); time.sleep(0.3)
                # 툴캠 서보 미세정렬
                conv, e = self._servo_align()
                if conv:
                    self._servo_tie_blocking()
                    tied += 1
                    self.get_logger().info(f'[SERVO_TIE] ✅ {label} 결속 (총 {tied}/{len(pts)})')
                    self.flog(f'[SERVO_TIE] ✅{label} 결속 총{tied}')
                else:
                    self.get_logger().warn(f'[SERVO_TIE] {label} 정렬 미수렴(err {e}) → 스킵')
                    self.flog(f'[SERVO_TIE] {label} 미수렴({e}) 스킵')
            self.get_logger().info(f'[SERVO_TIE] 완료: 결속 {tied}/{len(pts)}')
            self.flog(f'[SERVO_TIE] 완료 {tied}/{len(pts)}')
        finally:
            self._servo_running = False
            # 세션 누적(자세별 결속 합산) — _servo_tie_blocking이 매번 리셋하므로 재설정
            self._servo_session_tied += tied
            self._servo_session_total += n_pts
            self.completed_points = self._servo_session_tied
            self.total_points = self._servo_session_total
            if self.camera_mode == 2:
                # 우측전용: 준비자세 복귀 후 IDLE (기존 동작 유지)
                self._send_xy_absolute(self.ready_pose_x_mm, self.ready_pose_y_mm)
                self._transition_to(TyingState.IDLE)
            else:
                # 양카메라: phase 상태머신에 넘김 (Phase1→자세변경→Phase2→최종)
                # _handle_complete가 tying_phase/camera_mode 보고 다음 단계 결정
                self.flog(f'[SERVO_TIE] Phase{self.tying_phase} 완료 → COMPLETE (phase 머신)')
                self._transition_to(TyingState.COMPLETE)

    def _servo_tie_blocking(self):
        """결속 액션 주입(Z하강/트리거/Z상승, XY는 이미 정렬) + 완료(IDLE) 대기.
        기존 상태머신+z_monitor 재사용 (안전). 서보스레드에서 호출."""
        self._servo_single = True
        self.total_points = 1
        self.completed_points = 0
        self.action_queue = [
            Action(action_type=ActionType.MOVE_Z_DOWN, z_mm=self.tying_z_down_mm,
                   point_label='SERVO', message='서보결속 Z하강'),
            Action(action_type=ActionType.TRIGGER,
                   point_label='SERVO', message='서보결속 트리거'),
            Action(action_type=ActionType.MOVE_Z_UP, z_mm=self.tying_z_down_mm,
                   point_label='SERVO', message='서보결속 Z상승'),
        ]
        self.current_action_index = 0
        self._transition_to(TyingState.EXECUTING_ACTION)
        t = time.time()
        while self.state != TyingState.IDLE and time.time() - t < 40:
            time.sleep(0.2)

    # ============================================
    # [Orbbec] 교차점 검출+호모그래피 지그재그 결속 (비주얼서보잉 대체)
    # ============================================
    def _init_orbbec(self):
        """Orbbec 로컬라이저(YOLO+호모그래피) 로드. 실패 시 orbbec_mode off."""
        try:
            from rebar_vision.orbbec_detector import OrbbecLocalizer
            self.orbbec = OrbbecLocalizer(
                self,
                model_path='/home/koceti/ros2_ws/src/rebar_vision/model/orbbec_crossing.pt',
                homo_path='/home/koceti/ros2_ws/data/calibration/homography_orbbec.yaml',
                color_topic='/camera/color/image_raw',
                yrange={'r': (self.right_cam_y_min, self.right_cam_y_max),
                        'l': (self.left_cam_y_min, self.left_cam_y_max)},
                x_min=0.0, x_max=self.max_stage_x_mm,
                left_x_min=self.left_min_x_mm)
            self.get_logger().info('  [orbbec] 로컬라이저 로드 ✅ (orbbec_mode ON)')
        except Exception as e:
            self.get_logger().error(f'  [orbbec] 초기화 실패 → orbbec_mode OFF: {e}')
            self.orbbec_mode = False

    @staticmethod
    def _pose_char(pose_state):
        return 'l' if pose_state == 'left' else 'r'

    def _orbbec_setup_yaw(self):
        """자세변경용 yaw 절대값 준비 (_start_detection과 동일)."""
        self.yaw_right_cam = self.home_ref_yaw_deg + self.yaw_right_cam_offset
        self.yaw_left_cam = self.home_ref_yaw_deg + self.yaw_left_cam_offset
        self.yaw_left_cam_approach = self.home_ref_yaw_deg + self.yaw_left_cam_approach_offset
        self.yaw_mid = self.home_ref_yaw_deg + self.yaw_mid_offset
        self.x_home_mm = 0.0

    def _orbbec_pose_change(self, next_pose):
        """기존 build_pose_change(무수정) 자세변경 액션 주입 + 완료대기."""
        self.action_builder.update_params(
            x_home_mm=self.x_home_mm, yaw_right_cam=self.yaw_right_cam,
            yaw_left_cam=self.yaw_left_cam,
            yaw_left_cam_approach=self.yaw_left_cam_approach,
            yaw_mid=self.yaw_mid, home_ref_yaw_deg=self.home_ref_yaw_deg)
        pose_actions = self.action_builder.build_pose_change(
            self.current_pose_state, next_pose)
        self.get_logger().info(f'[ORBBEC] 자세변경 {self.current_pose_state}→{next_pose}')
        self.flog(f'[ORBBEC] 자세변경 {self.current_pose_state}→{next_pose}')
        self._servo_single = True   # _handle_complete → IDLE (phase로직 스킵)
        self.action_queue = pose_actions
        self.current_action_index = 0
        self._transition_to(TyingState.EXECUTING_ACTION)
        t = time.time()
        while self.state != TyingState.IDLE and time.time() - t < 60:
            time.sleep(0.2)
        self.current_pose_state = next_pose

    def _run_orbbec_tying(self):
        """[Orbbec] WP 결속: 검출→자세별 분류→지그재그 결속. 별도 스레드.
        도착자세(current_pose_state)부터 처리, 첫세그 X max→0, 자세변경(x=0),
        둘째세그 X 0→max. 빈 자세 스킵. 완료 후 검출자세(x=0) 복귀 + TYING_COMPLETE.
        """
        tied = 0
        try:
            if self.home_ref_x_deg is None:
                self.get_logger().error('[ORBBEC] 호밍 미수행 → 중단')
                self.flog('[ORBBEC] 호밍 미수행 중단'); return
            if self.orbbec is None or not self.orbbec.ready():
                self.get_logger().error('[ORBBEC] 로컬라이저/영상 미준비 → 중단')
                self.flog('[ORBBEC] 미준비 중단'); return
            self._orbbec_setup_yaw()
            cur = self._pose_char(self.current_pose_state)
            # base 정착 대기 + 버퍼클리어 → 주행 정지 직후 블러/위치오차 방지
            self.get_logger().info(
                f'[ORBBEC] base 정착 대기 {self.orbbec_settle_sec:.1f}s + 버퍼클리어')
            self.flog(f'[ORBBEC] 정착대기 {self.orbbec_settle_sec:.1f}s')
            time.sleep(self.orbbec_settle_sec)
            self.orbbec.buf.clear()
            time.sleep(0.3)                            # 신선 프레임 채우기
            sets = self.orbbec.localize(cur)           # {'r':[(x,y,u,v)], 'l':[...]}
            order = [cur, ('l' if cur == 'r' else 'r')]
            self.get_logger().info(
                f'[ORBBEC] WP 결속 시작 (도착자세={cur}, 순서={order})')
            self.flog(f'[ORBBEC] 시작 도착={cur} 우{len(sets["r"])}좌{len(sets["l"])}')
            first = True
            for P in order:
                if self._servo_abort:
                    self.flog('[ORBBEC] 중단(abort)'); break
                seg = sets[P]
                if not seg:
                    self.get_logger().info(f'[ORBBEC] {P}자세 포인트 없음 → 스킵')
                    self.flog(f'[ORBBEC] {P} 스킵(0점)'); continue
                pose_just_changed = False
                if self._pose_char(self.current_pose_state) != P:
                    self._orbbec_pose_change('right' if P == 'r' else 'left')
                    pose_just_changed = True   # 자세변경 직후 첫 포인트에 X 백래쉬 보정
                # 첫 세그=X 큰것→0(reverse), 둘째=0→큰것 (자세변경은 x=0에서)
                seg_sorted = sorted(seg, key=lambda c: c[0], reverse=first)
                self.get_logger().info(
                    f'[ORBBEC] {P}자세 {len(seg_sorted)}점 결속 '
                    f'(X {"max→0" if first else "0→max"})')
                self.flog(f'[ORBBEC] {P} {len(seg_sorted)}점 '
                          f'X{"내림" if first else "오름"}')
                for (x, y, u, v) in seg_sorted:
                    if self._servo_abort:
                        break
                    x_cmd = x
                    if pose_just_changed and self.pose_change_backlash_x_mm:
                        # 자세변경 시 X가 0까지 줄었다 다시 증가 → 백래쉬 언더슈트.
                        # 자세변경 직후 첫 포인트에만 X를 조금 더 밀어 보정.
                        x_cmd = x + self.pose_change_backlash_x_mm
                        self.get_logger().info(
                            f'[ORBBEC] 자세변경 직후 X 백래쉬 보정: '
                            f'{x:.0f}→{x_cmd:.0f}mm (+{self.pose_change_backlash_x_mm:.0f})')
                        self.flog(f'[ORBBEC] 백래쉬보정 X {x:.0f}→{x_cmd:.0f}')
                    pose_just_changed = False
                    self._send_xy_absolute(x_cmd, y)
                    self._servo_wait_arrival(x_cmd, y)
                    self._force_z = True
                    self._servo_tie_blocking()      # Z하강/트리거/Z상승
                    tied += 1
                    self.get_logger().info(
                        f'[ORBBEC] ✅ ({x:.0f},{y:.0f}) 결속 (총 {tied})')
                    self.flog(f'[ORBBEC] ✅({x:.0f},{y:.0f}) 총{tied}')
                first = False
            self.get_logger().info(f'[ORBBEC] WP 결속 완료: {tied}점 '
                                   f'(끝자세={self.current_pose_state})')
            self.flog(f'[ORBBEC] 완료 {tied}점 끝자세={self.current_pose_state}')
        finally:
            self._orbbec_running = False
            # 검출자세(x=0) 복귀 — 끝자세에 맞는 Y (다음 WP 검출 겸 주행위치).
            # ⚠️ 미호밍(home_ref None)이면 _send_xy_absolute가 None연산 예외 → 아래
            #    TYING_COMPLETE 발행을 막아 navigator가 무한대기했음 → 가드 필수.
            cur = self._pose_char(self.current_pose_state)
            dy = self.detect_left_y_mm if cur == 'l' else self.ready_pose_y_mm
            if self.home_ref_x_deg is not None:
                try:
                    self._send_xy_absolute(self.ready_pose_x_mm, dy)
                except Exception as e:
                    self.get_logger().error(f'[ORBBEC] 검출자세 복귀 실패: {e}')
                    self.flog(f'[ORBBEC] 검출자세 복귀 실패: {e}')
            else:
                self.flog('[ORBBEC] 미호밍 → 검출자세 복귀 스킵 (TYING_COMPLETE는 발행)')
            self.completed_points = tied
            self.total_points = tied
            msg = String()
            msg.data = f'TYING_COMPLETE:{tied}'
            self.motion_cmd_pub.publish(msg)
            self.flog(f'[ORBBEC] TYING_COMPLETE:{tied} 발행 → 검출자세({dy:.0f}) 복귀')
            self._transition_to(TyingState.IDLE)

    def _save_servo_debug(self, it, track):
        """추적실패 순간 툴캠 뷰 + 검출 + 목표/track 마킹 저장 (진단용)."""
        try:
            import cv2
            if not self.toolcam_buf:
                return
            img = self.toolcam_buf[-1].copy()
            r = self.toolcam_model(img, conf=0.15, verbose=False)[0]
            for b in r.boxes:
                cx = int((b.xyxy[0][0]+b.xyxy[0][2])/2)
                cy = int((b.xyxy[0][1]+b.xyxy[0][3])/2)
                cv2.circle(img, (cx, cy), 16, (0, 255, 0), 3)
                cv2.putText(img, f'{float(b.conf[0]):.2f}', (cx+18, cy),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            tg = (int(self.toolcam_target[0]), int(self.toolcam_target[1]))
            cv2.drawMarker(img, tg, (255, 0, 255), cv2.MARKER_TILTED_CROSS, 40, 4)
            cv2.putText(img, 'TARGET', (tg[0]+22, tg[1]), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (255, 0, 255), 2)
            if track is not None:
                tk = (int(track[0]), int(track[1]))
                cv2.drawMarker(img, tk, (0, 165, 255), cv2.MARKER_CROSS, 40, 3)
                cv2.putText(img, 'track', (tk[0]+22, tk[1]+20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
            path = '/home/koceti/ros2_ws/data/calibration/servo_fail_view.png'
            cv2.imwrite(path, img)
            self.flog(f'  [s{it}] 디버그뷰 저장: {path}')
        except Exception as e:
            self.flog(f'  디버그뷰 실패: {e}')

    def _servo_cur_xy_mm(self):
        """현재 스테이지 XY(mm). 모터위치 미수신 시 None."""
        if 0x44 not in self.motor_positions or 0x45 not in self.motor_positions:
            return None
        x = (self.home_ref_x_deg - self.motor_positions[0x44]) / self.deg_per_mm_x
        y = (self.motor_positions[0x45] - self.home_ref_y_deg) / self.deg_per_mm_y
        return [x, y]

    def _scan_goto(self, x_mm, y_mm):
        """XY 이동 + 도착 대기 (일반 속도, 범위 클램프)."""
        x_mm = min(max(x_mm, 0.0), self.max_stage_x_mm)
        y_mm = min(max(y_mm, 0.0), self.max_stage_y_mm)
        self._send_xy_absolute(x_mm, y_mm)
        self._servo_wait_arrival(x_mm, y_mm)

    def _scan_move_cmd(self, x_mm, y_mm):
        """저속 연속이동 명령 (대기 없음) — 스캔용."""
        x_mm = min(max(x_mm, 0.0), self.max_stage_x_mm)
        y_mm = min(max(y_mm, 0.0), self.max_stage_y_mm)
        x_deg = self.home_ref_x_deg - x_mm * self.deg_per_mm_x
        y_deg = self.home_ref_y_deg + y_mm * self.deg_per_mm_y
        for jid, deg in ((0x144, x_deg), (0x145, y_deg)):
            m = JointControl()
            m.joint_id = jid; m.position = float(deg)
            m.velocity = float(self.scan_velocity)
            m.control_mode = JointControl.MODE_ABSOLUTE
            self.joint_pub.publish(m); time.sleep(0.05)

    def _run_servo_scan(self, scan_to_mm, scan_y=50.0, step_mm=15.0,
                        tie_zone_px=160.0, margin_mm=100.0):
        """[Stage3a] 스텝 스캔: 한 스텝 이동→완전정지→검출→교차점이면 서보+결속.
        (연속이동 정지충돌 방지). 방향기반 이중결속방지(margin)."""
        cur = self._servo_cur_xy_mm()
        if cur is None:
            self.get_logger().error('[SCAN] 모터위치 미수신')
            self.flog('[SCAN] 모터위치 미수신 → 종료'); return
        scan_dir = 1.0 if scan_to_mm > cur[0] else -1.0
        self.get_logger().info(
            f'[SCAN] X {cur[0]:.0f}→{scan_to_mm:.0f} (Y={scan_y}) dir={scan_dir:+.0f} step{step_mm}')
        self.flog(f'[SCAN] 시작 X{cur[0]:.0f}→{scan_to_mm:.0f} Y{scan_y} dir{scan_dir:+.0f}')
        self._scan_goto(cur[0], scan_y)          # Y를 스캔행으로
        tied_x = []
        while not self._servo_abort:
            cur = self._servo_cur_xy_mm()
            if cur is None:
                break
            if (cur[0] - scan_to_mm) * scan_dir >= -3:
                self.get_logger().info('[SCAN] 스캔끝 도달 → 완료'); break
            # 완전 정지 상태에서 깨끗하게 검출
            time.sleep(0.2)
            cross = self._toolcam_detect(n=5)
            if cross is not None:
                err = float(np.hypot(*(self.toolcam_target - cross)))
                near_tied = any(abs(cur[0] - tx) < margin_mm for tx in tied_x)
                self.flog(f'[SCAN] X{cur[0]:.0f} err{err:.0f}px near_tied{near_tied}')
                if err < tie_zone_px and not near_tied:
                    self.get_logger().info(
                        f'[SCAN] 교차점 @X={cur[0]:.0f} err{err:.0f}px → 서보')
                    self.flog(f'[SCAN] 교차점진입 @X{cur[0]:.0f} err{err:.0f} → 서보')
                    conv, e = self._servo_align()
                    if conv:
                        self._servo_tie_blocking()
                        txv = self._servo_cur_xy_mm()
                        txv = txv[0] if txv else cur[0]
                        tied_x.append(txv)
                        self.get_logger().info(
                            f'[SCAN] ✅ 결속 @X={txv:.0f} (총 {len(tied_x)})')
                        self.flog(f'[SCAN] ✅결속 @X{txv:.0f} 총{len(tied_x)}')
                        self._scan_goto(txv + scan_dir*margin_mm, scan_y)  # margin+Y복귀
                        continue
                    else:
                        self.get_logger().warn(f'[SCAN] 정렬 미수렴(err{e:.0f}) → 진행')
                        self.flog(f'[SCAN] 정렬미수렴 err{e:.0f} → 진행')
            # 다음 스텝 이동
            self._scan_goto(cur[0] + scan_dir*step_mm, scan_y)
        self.get_logger().info(f'[SCAN] 종료 (결속 {len(tied_x)}점)')
        self.flog(f'[SCAN] 종료 결속{len(tied_x)}점')

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
        """Z축 토크 과부하 시: Z정지 → 트리거 스킵 → Z상승(순수 상승만).

        액션 큐: ... → MOVE_Z_DOWN(현재) → TRIGGER → MOVE_Z_UP(_WITH_XY) → ...
        TRIGGER를 스킵하고 Z상승으로 점프한다.
        MOVE_Z_UP_WITH_XY인 경우 순수 MOVE_Z_UP으로 교체하여
        상승 완료 전 XY 이동을 방지한다 (공구가 철근에 걸리는 문제 방지).
        """
        # Z축 즉시 정지
        self.z_monitor.stop_z_motor()

        # TRIGGER 액션 찾아서 스킵 → MOVE_Z_UP으로 점프
        idx = self.current_action_index + 1
        while idx < len(self.action_queue):
            if self.action_queue[idx].action_type == ActionType.TRIGGER:
                # TRIGGER 다음 액션이 Z_UP_WITH_XY이면 순수 Z_UP으로 교체
                z_up_idx = idx + 1
                if (z_up_idx < len(self.action_queue)
                        and self.action_queue[z_up_idx].action_type
                        == ActionType.MOVE_Z_UP_WITH_XY):
                    orig = self.action_queue[z_up_idx]
                    self.action_queue[z_up_idx] = Action(
                        action_type=ActionType.MOVE_Z_UP,
                        z_mm=orig.z_mm,
                        point_label=orig.point_label,
                        message=f'{orig.point_label}: Z {orig.z_mm}mm 상승 (과부하복구)',
                    )
                    self.get_logger().warn(
                        f'    Z_UP_WITH_XY → Z_UP 교체 (XY 선행이동 방지)')
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
        # [Stage2] SERVO_ONE 단일점 서보결속 완료 → Phase2/웨이포인트 안 타고 IDLE
        if self._servo_single:
            self._servo_single = False
            self.get_logger().info('[SERVO_ONE] 결속 완료 → IDLE')
            self.flog('SERVO_ONE_DONE')
            self.action_queue = []
            self.current_action_index = 0
            self.tying_message = '서보결속 완료'
            self._transition_to(TyingState.IDLE)
            self._publish_tying_feedback()
            return

        # CHN_POS 단독 자세변경 완료 → Phase2 안 타고 바로 IDLE 종료
        if self._pose_change_only:
            self._pose_change_only = False
            self.get_logger().info(
                f'[UI] CHN_POS 자세변경 완료 → 현재 자세: {self.current_pose_state}')
            self.flog(f"CHN_POS_DONE: pose={self.current_pose_state}")
            self.action_queue = []
            self.current_action_index = 0
            self.tying_message = f'{self.current_pose_state} 자세변경 완료'
            self._transition_to(TyingState.IDLE)
            self._publish_tying_feedback()
            return

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
        # (camera_mode==2: 우측전용 → 자세변경/Phase2 건너뛰고 바로 최종완료로 fall-through)
        if self.tying_phase == 1 and self.camera_mode != 2:
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
        self._pose_change_only = False
        self._phase2_cam = None
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

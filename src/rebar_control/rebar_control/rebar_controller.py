#!/usr/bin/env python3
"""
Rebar Controller Node
경로 추종 제어

navigator로부터 목표 위치를 받아 엔코더 odometry 기반으로
/cmd_vel을 발행하여 목표 지점까지 주행합니다.

제어 알고리즘: Simple PID

구독:
- /encoder_odom (PoseStamped) - 엔코더 odometry (주 제어용)
- /robot_pose (PoseStamped) - ZED X VSLAM (보조/모니터링)
- /mission/target_pose (PoseStamped) - navigator에서

발행:
- /cmd_vel (Twist) - drive_controller로
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from rebar_base_interfaces.msg import Waypoint, WaypointArray, JointControl
import json
import math
import time
import logging
import os
from datetime import datetime


class RebarController(Node):
    """경로 추종 제어 노드"""

    def __init__(self):
        super().__init__('rebar_controller')

        # 파일 로거 설정
        self._setup_file_logger()

        # 파라미터 선언
        self.declare_parameter('control_rate', 20.0)  # Hz
        self.declare_parameter('max_linear_vel', 0.25)  # m/s (0.5 → 0.25 감속)
        self.declare_parameter('max_angular_vel', 1.0)  # rad/s

        self.declare_parameter('distance_tolerance', 0.001)  # m (1mm) - 0x92 position 기반 정밀 제어
        self.declare_parameter('heading_tolerance', 0.1)  # rad (~6도)
        self.declare_parameter('max_tying_points', 100)  # repeat 모드 종료 기준 (총 결속 포인트)

        # PID 파라미터
        self.declare_parameter('min_linear_vel', 0.01)  # m/s (모터 stiction 극복 최소 속도)
        self.declare_parameter('kp_linear', 1.0)  # 0.5 → 1.0 (응답성 향상)
        self.declare_parameter('ki_linear', 0.05)  # 0.0 → 0.05 (정상상태 오차 보정)
        self.declare_parameter('kd_linear', 0.1)
        self.declare_parameter('integral_limit', 0.2)  # Anti-windup 한계값

        self.declare_parameter('kp_angular', 1.0)
        self.declare_parameter('ki_angular', 0.1)  # 0.0 → 0.1 (heading 정상상태 오차 보정)
        self.declare_parameter('kd_angular', 0.2)
        self.declare_parameter('integral_heading_limit', 0.3)  # Anti-windup for heading

        # 파라미터 가져오기
        self.control_rate = self.get_parameter('control_rate').value
        self.max_linear = self.get_parameter('max_linear_vel').value
        self.max_angular = self.get_parameter('max_angular_vel').value
        self.min_linear = self.get_parameter('min_linear_vel').value

        self.distance_tolerance = self.get_parameter('distance_tolerance').value
        self.heading_tolerance = self.get_parameter('heading_tolerance').value

        self.kp_linear = self.get_parameter('kp_linear').value
        self.ki_linear = self.get_parameter('ki_linear').value
        self.integral_limit = self.get_parameter('integral_limit').value
        self.kd_linear = self.get_parameter('kd_linear').value

        self.kp_angular = self.get_parameter('kp_angular').value
        self.ki_angular = self.get_parameter('ki_angular').value
        self.kd_angular = self.get_parameter('kd_angular').value
        self.integral_heading_limit = self.get_parameter('integral_heading_limit').value

        # 현재 상태
        self.current_pose = None
        self.target_pose = None
        self.target_waypoint = None  # Enhanced Waypoint (motion type 포함)
        self.current_motion_type = Waypoint.MOTION_DIFFERENTIAL  # 기본값
        self.first_pose_received = False  # 최초 pose 수신 플래그
        self.reference_heading = None  # 기준 heading (첫 목표 수신 시 현재 heading으로 설정)
        self.waypoint_reached_sent = False  # 웨이포인트 도달 알림 발행 플래그 (중복 방지)
        self.min_approach_distance = float('inf')  # 최근접 거리 추적

        # 웨이포인트 도달 후 결속 프로세스
        self.tying_in_progress = False  # 결속 진행 중 플래그
        self.tying_complete_received = False  # TYING_COMPLETE 수신 플래그
        self.waypoint_reached_time = 0.0  # 도달 확정 시각
        self.waypoint_dwelling = False  # 대기 중 플래그

        # Repeat 모드 (핑퐁)
        self.repeat_mode = False  # repeat ON/OFF
        self.max_tying_points = self.get_parameter('max_tying_points').value
        self.total_tying_points = 0  # 총 결속 포인트 누적
        self.total_wp_count = 0  # 전체 WP 처리 카운트 (교번 방향용)
        self.original_waypoints_x = []  # 원본 WP 저장 (핑퐁용)
        self.original_waypoints_y = []
        self.original_waypoints_motion_type = []
        self.original_waypoints_max_speed = []
        self.pingpong_lap = 0  # 핑퐁 회차 (0=1회차 정방향, 1=역방향, 2=정방향...)

        # VSLAM 트래킹 유실 감지
        self.vslam_last_pose = None  # 마지막 수신 위치
        self.vslam_last_pose_time = 0.0  # 마지막 위치 변화 시각
        self.vslam_last_update_time = 0.0  # 마지막 pose 수신 시각
        self.vslam_frozen_threshold = 3.0  # pose 수신 중단 판정 시간 (초)
        self.vslam_move_check_interval = 5.0  # 이동 중 위치 변화 체크 주기 (초)
        self.vslam_move_check_pose = None  # 이동 체크 시작 위치
        self.vslam_move_check_time = 0.0  # 이동 체크 시작 시각
        self.vslam_min_move_distance = 0.01  # 이동 체크 주기 동안 최소 이동량 (10mm)
        self.last_cmd_linear = 0.0  # 마지막 발행한 cmd_vel linear
        self.vslam_jump_threshold = 0.5  # 위치 점프 판정 거리 (m) - 1사이클에 이 이상 이동 불가
        self.vslam_lost = False  # 트래킹 유실 상태
        self.vslam_lost_time = 0.0  # 유실 감지 시각

        # Lateral motion 파라미터
        self.lateral_tolerance = 0.005  # 5mm
        self.lateral_speed_dps = 200.0  # degrees per second
        self.mm_per_rotation = 44.0  # 44mm = 360도 = 1회전 (실측: 9회전=396mm)
        self.lateral_y_offset = 0.0  # 횡이동 누적 Y 오프셋 (m)

        # 횡이동 분할 실행 상태
        self.lateral_total_rotations = 0  # 총 회전 횟수
        self.lateral_current_rotation = 0  # 현재 실행 중인 회전 인덱스
        self.lateral_command_sent = False  # 현재 회전 명령 전송 플래그
        self.lateral_start_time = None  # 현재 회전 시작 시간
        self.lateral_rotation_duration = 360.0 / 200.0 + 0.5  # 1회전 예상 시간 (초) + 안전 마진

        # 미션 오프셋 (첫 waypoint를 현재 위치로)
        self.first_waypoint_of_mission = True
        self.mission_offset_x = 0.0
        self.mission_offset_y = 0.0

        # 전체 경로 관리 (tire_roller style - 인덱스 기반)
        self.waypoint_array_x = []  # X 좌표 배열
        self.waypoint_array_y = []  # Y 좌표 배열
        self.waypoint_array_motion_type = []  # Motion type 배열
        self.waypoint_array_max_speed = []  # Max speed 배열
        self.current_waypoint_index = 0  # 현재 웨이포인트 인덱스
        self.path_received = False  # 경로 수신 플래그

        # 경로 방향 (웨이포인트에서 결정, heading_error로 판단하지 않음)
        self.path_is_backward = False

        # PID 상태
        self.prev_distance_error = 0.0
        self.prev_heading_error = 0.0
        self.integral_distance = 0.0
        self.integral_heading = 0.0

        # 엔코더 odometry 상태
        self.encoder_pose = None  # 엔코더 기반 위치 (주 제어용)

        # 미션 원점 (encoder 리셋 대신 상대좌표 계산용)
        self.mission_origin_x = 0.0
        self.mission_origin_y = 0.0
        self.mission_origin_yaw = 0.0

        # ROS2 구독자
        # 엔코더 odometry (주 제어용)
        self.encoder_odom_sub = self.create_subscription(
            PoseStamped,
            '/encoder_odom',
            self.encoder_odom_callback,
            10
        )

        # VSLAM (보조/모니터링용, 기존 호환)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/robot_pose',
            self.pose_callback,
            10
        )

        self.target_sub = self.create_subscription(
            PoseStamped,
            '/mission/target_pose',
            self.target_callback,
            10
        )

        # Mission command 구독 (CANCEL 등)
        self.mission_cmd_sub = self.create_subscription(
            String,
            '/mission/command',
            self.mission_command_callback,
            10
        )

        # Enhanced Waypoint 구독 (motion type 포함) - 하위호환용
        self.waypoint_sub = self.create_subscription(
            Waypoint,
            '/mission/enhanced_target',
            self.waypoint_callback,
            10
        )

        # WaypointArray 구독 (tire_roller style - 전체 경로 한번에 수신)
        self.waypoint_array_sub = self.create_subscription(
            WaypointArray,
            '/mission/waypoint_array',
            self.waypoint_array_callback,
            10
        )

        # 미션 명령 수신 (STOP/ABORT 시 목표 클리어)
        self.command_sub = self.create_subscription(
            String,
            '/mission/command',
            self.command_callback,
            10
        )

        # 횡이동 완료 신호 구독 (joint_controller로부터)
        self.lateral_complete_sub = self.create_subscription(
            String,
            '/lateral_motion_complete',
            self.lateral_complete_callback,
            10
        )

        # 결속 완료 신호 구독 (tying_orchestrator로부터)
        self.tying_complete_sub = self.create_subscription(
            String,
            '/rebar_motion_cmd',
            self.tying_complete_callback,
            10
        )

        # 미션 피드백 구독 (mission_done 감지용)
        self.feedback_sub = self.create_subscription(
            String,
            '/mission/feedback',
            self.feedback_callback,
            10
        )

        # ROS2 발행자
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # 웨이포인트 도달 알림 발행
        self.waypoint_reached_pub = self.create_publisher(
            String,
            '/mission/waypoint_reached',
            10
        )

        # Lateral motion을 위한 joint_control 발행
        self.joint_pub = self.create_publisher(
            JointControl,
            '/joint_control_cmd',
            10
        )

        # 카메라 방향 제어 (pose_mux에 전진/후진 알림)
        self.travel_direction_pub = self.create_publisher(
            String,
            '/travel_direction',
            10
        )

        # 결속 명령 발행 (tying_orchestrator에 TYING_START 전달)
        self.mission_command_pub = self.create_publisher(
            String,
            '/mission/command',
            10
        )

        # 제어 루프 타이머
        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.get_logger().info("Rebar Controller 노드 초기화 완료")
        self.get_logger().info(f"  - 제어 주기: {self.control_rate} Hz")
        self.get_logger().info(f"  - 최대 선속도: {self.max_linear} m/s")
        self.get_logger().info(f"  - 최대 각속도: {self.max_angular} rad/s")
        self.flog(f"=== Rebar Controller 시작 ===")
        self.flog(f"제어 주기: {self.control_rate}Hz, max_lin: {self.max_linear}, max_ang: {self.max_angular}")

    def _setup_file_logger(self):
        """파일 로거 설정 (/tmp/rebar_controller_YYYYMMDD_HHMMSS.log)"""
        log_dir = '/tmp'
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._log_path = os.path.join(log_dir, f'rebar_controller_{timestamp}.log')
        self._file_logger = logging.getLogger('rebar_controller_file')
        self._file_logger.setLevel(logging.DEBUG)
        # 기존 핸들러 제거 (중복 방지)
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

    def waypoint_array_callback(self, msg: WaypointArray):
        """
        전체 경로 수신 (tire_roller style)

        Navigator가 전체 웨이포인트 배열을 한번에 전달
        이후 내부 인덱스로 진행 관리 (외부 간섭 없음)
        """
        # VSLAM 유실 상태 리셋
        self.vslam_lost = False
        self.vslam_last_pose_time = time.monotonic()

        # 미션 원점 저장 (encoder 리셋 대신 현재 위치를 origin으로)
        if self.encoder_pose is not None:
            self.mission_origin_x = self.encoder_pose.pose.position.x
            self.mission_origin_y = self.encoder_pose.pose.position.y
            self.mission_origin_yaw = self._quaternion_to_yaw(self.encoder_pose.pose.orientation)
        else:
            self.mission_origin_x = 0.0
            self.mission_origin_y = 0.0
            self.mission_origin_yaw = 0.0

        # 경로 저장 (상대좌표 - navigator가 current_pose 기준으로 변환 완료)
        self.waypoint_array_x = list(msg.x)
        self.waypoint_array_y = list(msg.y)
        self.waypoint_array_motion_type = list(msg.motion_type)
        self.waypoint_array_max_speed = list(msg.max_speed)
        self.current_waypoint_index = 0
        self.path_received = True

        # 원본 WP 저장 (repeat 핑퐁용)
        self.original_waypoints_x = list(msg.x)
        self.original_waypoints_y = list(msg.y)
        self.original_waypoints_motion_type = list(msg.motion_type)
        self.original_waypoints_max_speed = list(msg.max_speed)
        self.pingpong_lap = 0
        self.total_tying_points = 0

        # 경로 방향 결정 (WP[0]≈현재위치이므로 WP[1]의 X 부호로 판정)
        if len(self.waypoint_array_x) > 1:
            self.path_is_backward = (self.waypoint_array_x[1] < 0)
        elif len(self.waypoint_array_x) == 1:
            self.path_is_backward = (self.waypoint_array_x[0] < 0)
        else:
            self.path_is_backward = False

        # 카메라 방향 발행 (pose_mux에 전달 → 미션 중 카메라 고정)
        direction_msg = String()
        direction_msg.data = 'backward' if self.path_is_backward else 'forward'
        self.travel_direction_pub.publish(direction_msg)
        self.get_logger().info(f"📷 카메라 방향 설정: {direction_msg.data}")

        # reference_heading = 로봇의 현재 facing 방향 (직진 유지 기준)
        self.reference_heading = None

        # 엔코더 odom 기반: 상대좌표 직접 사용 (heading 회전 변환 불필요)
        if len(self.waypoint_array_x) > 0:
            direction_str = "후진" if self.path_is_backward else "전진"
            self.get_logger().info("=" * 70)
            self.get_logger().info(f"📥 전체 경로 수신: {len(self.waypoint_array_x)}개 웨이포인트 [{direction_str}] [엔코더 odom]")
            self.get_logger().info(f"  미션 origin: ({self.mission_origin_x:.3f}, {self.mission_origin_y:.3f}) m")
            self.get_logger().info(f"  첫 WP: ({self.waypoint_array_x[0]:.3f}, {self.waypoint_array_y[0]:.3f}) m")
            self.flog(f"========== 경로 수신: {len(self.waypoint_array_x)}개 WP [{direction_str}] ==========")
            self.flog(f"미션 origin: ({self.mission_origin_x:.3f}, {self.mission_origin_y:.3f}) m")
            for i in range(len(self.waypoint_array_x)):
                mt = "LAT" if self.waypoint_array_motion_type[i] == Waypoint.MOTION_LATERAL else "DIFF"
                self.flog(f"  WP[{i}]: ({self.waypoint_array_x[i]:.3f}, {self.waypoint_array_y[i]:.3f}) @ {mt} spd={self.waypoint_array_max_speed[i]:.2f}")

            # 오프셋 불필요 (상대좌표 직접 사용)
            self.mission_offset_x = 0.0
            self.mission_offset_y = 0.0

            if len(self.waypoint_array_x) > 1:
                last_idx = len(self.waypoint_array_x) - 1
                self.get_logger().info(
                    f"  최종 WP: ({self.waypoint_array_x[last_idx]:.3f}, "
                    f"{self.waypoint_array_y[last_idx]:.3f}) m"
                )
            self.get_logger().info("=" * 70)

            self.first_waypoint_of_mission = False

        # 첫 번째 웨이포인트로 이동 시작
        self._set_current_waypoint_as_target()

    def _set_current_waypoint_as_target(self):
        """현재 인덱스의 웨이포인트를 목표로 설정"""
        if self.current_waypoint_index >= len(self.waypoint_array_x):
            # repeat 모드: 핑퐁 반복
            if self.repeat_mode and self.total_tying_points < self.max_tying_points:
                self._start_pingpong_next_lap()
                return

            # 모든 웨이포인트 완료
            if self.repeat_mode:
                self.get_logger().info(
                    f"미션 완료! 총 {self.total_tying_points}pt 결속 "
                    f"(목표 {self.max_tying_points}pt, {self.pingpong_lap + 1}회차)")
                self.flog(f"========== 미션 완료: {self.total_tying_points}pt / "
                          f"{self.max_tying_points}pt ({self.pingpong_lap + 1}회차) ==========")
            else:
                self.get_logger().info("모든 웨이포인트 완료!")
                self.flog("========== 미션 완료: 모든 웨이포인트 도달 ==========")
            self._publish_mission_complete()
            return

        idx = self.current_waypoint_index
        x = self.waypoint_array_x[idx] + self.mission_offset_x
        y = self.waypoint_array_y[idx] + self.mission_offset_y
        motion_type = self.waypoint_array_motion_type[idx]
        max_speed = self.waypoint_array_max_speed[idx]

        # 현재 motion type 업데이트
        old_motion_type = self.current_motion_type
        self.current_motion_type = motion_type

        # DIFFERENTIAL 웨이포인트마다 전진/후진 방향 재판정
        if motion_type != Waypoint.MOTION_LATERAL and self.encoder_pose is not None:
            curr_x = self.encoder_pose.pose.position.x - self.mission_origin_x
            dx = x - curr_x
            self.path_is_backward = (dx < 0)
            self.get_logger().info(
                f"📷 경로 방향: {'후진' if self.path_is_backward else '전진'} (dx={dx*1000:.0f}mm)"
            )

        # motion_type이 변경되면 횡이동 상태 + heading PID 리셋
        if old_motion_type != motion_type:
            self.lateral_total_rotations = 0
            self.lateral_current_rotation = 0
            self.lateral_command_sent = False
            self.lateral_start_time = None
            # heading PID 상태 초기화 (DIFFERENTIAL→LATERAL 전환 시 적분값 잔류 방지)
            self.integral_heading = 0.0
            self.prev_heading_error = 0.0
            self.integral_distance = 0.0
            # 즉시 정지 (stale cmd_vel 방지)
            self.publish_cmd_vel(0.0, 0.0)

        # target_pose 설정
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'odom'
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.orientation.w = 1.0
        self.target_pose = pose_msg

        # 횡이동 속도 업데이트
        if motion_type == Waypoint.MOTION_LATERAL:
            self.lateral_speed_dps = max_speed

        # 상태 리셋
        self.waypoint_reached_sent = False
        if hasattr(self, '_waypoint_reached_published'):
            delattr(self, '_waypoint_reached_published')

        # 기준 heading 설정 (첫 번째 웨이포인트에서만)
        # 로봇의 현재 facing 방향 = 직진 유지 기준 (atan2 아님)
        if self.reference_heading is None and self.current_pose is not None:
            self.reference_heading = self._quaternion_to_yaw(self.current_pose.pose.orientation)

        motion_str = "LATERAL" if motion_type == Waypoint.MOTION_LATERAL else "DIFFERENTIAL"
        self.get_logger().info(
            f"📍 웨이포인트 [{idx + 1}/{len(self.waypoint_array_x)}]: "
            f"({x:.3f}, {y:.3f}) @ {motion_str}"
        )

    def _start_pingpong_next_lap(self):
        """핑퐁 모드: 경로를 역순으로 뒤집어 다음 회차 시작

        예: 정방향 WP[0]→WP[1]→WP[2] 완료 후
            역방향 WP[1]→WP[0] (양 끝 중복 제거)
            다시 정방향 WP[1]→WP[2]
            ...
        """
        self.pingpong_lap += 1
        n = len(self.original_waypoints_x)

        if self.pingpong_lap % 2 == 1:
            # 역방향: 끝에서 두번째 → 처음 (마지막=현재위치 제외)
            indices = list(range(n - 2, -1, -1))
        else:
            # 정방향: 두번째 → 끝 (처음=현재위치 제외)
            indices = list(range(1, n))

        self.waypoint_array_x = [self.original_waypoints_x[i] for i in indices]
        self.waypoint_array_y = [self.original_waypoints_y[i] for i in indices]
        self.waypoint_array_motion_type = [self.original_waypoints_motion_type[i] for i in indices]
        self.waypoint_array_max_speed = [self.original_waypoints_max_speed[i] for i in indices]
        self.current_waypoint_index = 0

        # 경로 방향 업데이트
        if len(self.waypoint_array_x) > 0:
            # 현재 위치에서 첫 WP까지의 방향으로 판정
            curr_x = self.encoder_pose.pose.position.x - self.mission_origin_x if self.encoder_pose else 0.0
            dx = self.waypoint_array_x[0] - curr_x
            self.path_is_backward = (dx < 0)

        direction_str = "후진" if self.path_is_backward else "전진"
        self.get_logger().info(
            f"핑퐁 {self.pingpong_lap + 1}회차: {len(self.waypoint_array_x)}개 WP [{direction_str}] "
            f"(누적 {self.total_tying_points}/{self.max_tying_points}pt)")
        self.flog(
            f"========== 핑퐁 {self.pingpong_lap + 1}회차: {len(self.waypoint_array_x)}WP "
            f"[{direction_str}] 누적={self.total_tying_points}pt ==========")
        for i in range(len(self.waypoint_array_x)):
            self.flog(f"  WP[{i}]: ({self.waypoint_array_x[i]:.3f}, {self.waypoint_array_y[i]:.3f})")

        # 카메라 방향 업데이트
        direction_msg = String()
        direction_msg.data = 'backward' if self.path_is_backward else 'forward'
        self.travel_direction_pub.publish(direction_msg)

        # 첫 WP 설정
        self._set_current_waypoint_as_target()

    def _advance_to_next_waypoint(self):
        """다음 웨이포인트로 이동"""
        # 현재 웨이포인트 완료 알림
        self._publish_waypoint_index_reached(self.current_waypoint_index)

        # 최근접 거리 추적 리셋
        self.min_approach_distance = float('inf')

        # 플래그 리셋 (다음 WP 추종 시작을 위해)
        self.waypoint_reached_sent = False

        # 인덱스 증가
        self.current_waypoint_index += 1

        # 다음 웨이포인트 설정
        self._set_current_waypoint_as_target()

    def _publish_waypoint_index_reached(self, index):
        """웨이포인트 인덱스 도달 알림 (새 방식)"""
        msg = String()
        msg.data = f"INDEX:{index}"
        self.waypoint_reached_pub.publish(msg)

    def _publish_mission_complete(self):
        """미션 완료 알림"""
        msg = String()
        msg.data = "MISSION_COMPLETE"
        self.waypoint_reached_pub.publish(msg)

        # 상태 리셋
        self._reset_mission_state()

    def _reset_mission_state(self):
        """미션 상태 리셋"""
        self.waypoint_array_x = []
        self.waypoint_array_y = []
        self.waypoint_array_motion_type = []
        self.waypoint_array_max_speed = []
        self.current_waypoint_index = 0
        self.path_received = False
        self.first_waypoint_of_mission = True
        self.mission_offset_x = 0.0
        self.mission_offset_y = 0.0
        self.reference_heading = None
        self.target_pose = None
        self.lateral_total_rotations = 0
        self.min_approach_distance = float('inf')
        self.lateral_current_rotation = 0
        self.lateral_command_sent = False
        self.path_is_backward = False
        self.vslam_lost = False
        self.vslam_last_pose = None
        self.vslam_last_pose_time = 0.0
        self.tying_in_progress = False
        self.tying_complete_received = False
        self.waypoint_reached_time = 0.0
        self.repeat_mode = False
        self.total_tying_points = 0
        self.total_wp_count = 0
        self.original_waypoints_x = []
        self.original_waypoints_y = []
        self.original_waypoints_motion_type = []
        self.original_waypoints_max_speed = []
        self.pingpong_lap = 0
        # 미션 종료 시 카메라를 forward로 복귀
        direction_msg = String()
        direction_msg.data = 'forward'
        self.travel_direction_pub.publish(direction_msg)

    def mission_command_callback(self, msg: String):
        """미션 명령 처리 (CANCEL, SET_REPEAT 등)"""
        command = msg.data

        # JSON 명령 처리
        if command.startswith('{'):
            try:
                data = json.loads(command)
                if data.get('command') == 'SET_REPEAT':
                    self.repeat_mode = bool(data.get('repeat', False))
                    self.get_logger().info(f"Repeat 모드: {'ON' if self.repeat_mode else 'OFF'} "
                                           f"(max={self.max_tying_points}pt)")
                    self.flog(f"SET_REPEAT: {'ON' if self.repeat_mode else 'OFF'} max={self.max_tying_points}")
                    return
            except json.JSONDecodeError:
                pass

        if "CANCEL" in command or "STOP" in command or "ABORT" in command:
            self.get_logger().warn(f"🛑 긴급 정지 명령 수신: {command}")
            self.flog(f"========== 긴급 정지: {command} ==========")

            # 즉시 정지
            self.publish_cmd_vel(0.0, 0.0)

            # 목표 초기화
            self.target_pose = None
            self.target_waypoint = None

            # 미션 플래그 리셋
            self._reset_mission_state()
            self.waypoint_reached_sent = False

            self.get_logger().info("✅ 정지 완료 및 상태 초기화")

    def encoder_odom_callback(self, msg: PoseStamped):
        """엔코더 odometry 업데이트 (주 제어용)

        전후면 반전 보정: encoder_odom은 물리적 방향 그대로 출력하므로
        제어용으로 부호 반전하여 cmd_vel +전진 = encoder_x 증가와 일치시킴
        """
        inverted = PoseStamped()
        inverted.header = msg.header
        inverted.pose.position.x = -msg.pose.position.x
        inverted.pose.position.y = -msg.pose.position.y + self.lateral_y_offset  # 횡이동 오프셋 반영
        inverted.pose.position.z = msg.pose.position.z
        # yaw도 반전 (π 회전)
        yaw = self._quaternion_to_yaw(msg.pose.orientation)
        inv_yaw = yaw + math.pi
        inv_yaw = math.atan2(math.sin(inv_yaw), math.cos(inv_yaw))
        inverted.pose.orientation.z = math.sin(inv_yaw / 2.0)
        inverted.pose.orientation.w = math.cos(inv_yaw / 2.0)
        self.encoder_pose = inverted

    def pose_callback(self, msg):
        """현재 위치 업데이트 (ZED X에서, VSLAM 모니터링용)"""
        # 최초 pose 수신 시 초기 상태 출력
        if not self.first_pose_received:
            self.first_pose_received = True
            x = msg.pose.position.x
            y = msg.pose.position.y
            z = msg.pose.position.z
            yaw = self._quaternion_to_yaw(msg.pose.orientation)
            qx = msg.pose.orientation.x
            qy = msg.pose.orientation.y
            qz = msg.pose.orientation.z
            qw = msg.pose.orientation.w

            self.get_logger().info("=" * 70)
            self.get_logger().info("📡 [최초 /robot_pose 수신]")
            self.get_logger().info(f"  Position: ({x:.4f}, {y:.4f}, {z:.4f}) m")
            self.get_logger().info(
                f"  Orientation (quaternion): x={qx:.4f}, y={qy:.4f}, z={qz:.4f}, w={qw:.4f}"
            )
            self.get_logger().info(
                f"  Yaw (heading): {math.degrees(yaw):+7.2f}° ({yaw:+.4f} rad)"
            )
            if abs(yaw) > 0.01:
                self.get_logger().warn(
                    f"  ⚠️  초기 yaw가 0이 아닙니다! "
                    f"ZED tracking reset 후에도 orientation이 유지되고 있습니다."
                )
            else:
                self.get_logger().info("  ✅ 초기 yaw ≈ 0 (정상)")
            self.get_logger().info("=" * 70)

        self.current_pose = msg

        # VSLAM 트래킹 유실 감지
        now = time.monotonic()
        curr_x = msg.pose.position.x
        curr_y = msg.pose.position.y

        if self.vslam_last_pose is not None:
            lx, ly = self.vslam_last_pose
            move = math.sqrt((curr_x - lx)**2 + (curr_y - ly)**2)

            # 위치 점프 감지 (1사이클에 물리적으로 불가능한 이동)
            dt_since_update = now - self.vslam_last_update_time
            if dt_since_update > 0 and move > self.vslam_jump_threshold and dt_since_update < 1.0:
                if not self.vslam_lost:
                    self.vslam_lost = True
                    self.vslam_lost_time = now
                    self.get_logger().error(
                        f"⚠️ VSLAM 위치 점프 감지! "
                        f"({lx:.3f},{ly:.3f})→({curr_x:.3f},{curr_y:.3f}) "
                        f"이동={move*1000:.0f}mm / {dt_since_update*1000:.0f}ms"
                    )

        # pose 수신 자체가 살아있으면 타이머 갱신 (위치 변화량 무관)
        self.vslam_last_pose_time = now

        # VSLAM 유실 상태에서 pose가 계속 수신되면 복구
        if self.vslam_lost and not self._is_vslam_jump_lost():
            self.vslam_lost = False
            self.get_logger().info(
                f"✅ VSLAM 트래킹 복구 ({now - self.vslam_lost_time:.1f}초 후)"
            )

        self.vslam_last_pose = (curr_x, curr_y)
        self.vslam_last_update_time = now

    def waypoint_callback(self, msg):
        """Enhanced Waypoint 수신 (motion type 포함)"""
        # 새로운 waypoint인지 확인 (중복 로그 방지)
        is_new_waypoint = (
            self.target_waypoint is None or
            abs(self.target_waypoint.x - msg.x) > 0.001 or
            abs(self.target_waypoint.y - msg.y) > 0.001 or
            self.target_waypoint.motion_type != msg.motion_type
        )

        # motion_type이 변경되면 횡이동 상태 + heading PID 강제 리셋
        if self.target_waypoint is not None and self.target_waypoint.motion_type != msg.motion_type:
            self.lateral_total_rotations = 0
            self.lateral_current_rotation = 0
            self.lateral_command_sent = False
            self.lateral_start_time = None
            self.integral_heading = 0.0
            self.prev_heading_error = 0.0
            self.integral_distance = 0.0
            self.publish_cmd_vel(0.0, 0.0)

        self.target_waypoint = msg
        self.current_motion_type = msg.motion_type

        # PoseStamped도 업데이트 (하위 호환성)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'odom'
        pose_msg.pose.position.x = msg.x
        pose_msg.pose.position.y = msg.y
        pose_msg.pose.orientation.w = 1.0

        # target_callback 로직 재사용
        self.target_callback(pose_msg)

        # 새로운 waypoint일 때만 로그 출력
        if is_new_waypoint:
            motion_str = "LATERAL" if msg.motion_type == Waypoint.MOTION_LATERAL else "DIFFERENTIAL"
            self.get_logger().info(
                f"📍 Waypoint received: ({msg.x:.3f}, {msg.y:.3f}) @ {motion_str}"
            )

    def target_callback(self, msg):
        """목표 위치 업데이트 (navigator에서)"""

        # 첫 waypoint 수신 시 오프셋 계산 (현재 위치를 원점으로)
        if self.first_waypoint_of_mission and self.current_pose is not None:
            curr_x = self.current_pose.pose.position.x
            curr_y = self.current_pose.pose.position.y

            self.mission_offset_x = curr_x - msg.pose.position.x
            self.mission_offset_y = curr_y - msg.pose.position.y

            self.get_logger().info("=" * 70)
            self.get_logger().info("📍 [미션 오프셋 설정]")
            self.get_logger().info(f"  현재 위치: ({curr_x:.3f}, {curr_y:.3f}) m")
            self.get_logger().info(f"  첫 WP (원본): ({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}) m")
            self.get_logger().info(f"  오프셋: ({self.mission_offset_x:.3f}, {self.mission_offset_y:.3f}) m")
            self.get_logger().info("  → 이후 모든 waypoint에 오프셋 적용")
            self.get_logger().info("=" * 70)

            self.first_waypoint_of_mission = False

        # 오프셋 적용하여 adjusted target 생성
        adjusted_msg = PoseStamped()
        adjusted_msg.header = msg.header
        adjusted_msg.pose = msg.pose
        adjusted_msg.pose.position.x = msg.pose.position.x + self.mission_offset_x
        adjusted_msg.pose.position.y = msg.pose.position.y + self.mission_offset_y

        old_target = self.target_pose

        # 목표가 변경되었는지 확인 (adjusted 기준)
        target_changed = False
        if old_target is None or \
           abs(old_target.pose.position.x - adjusted_msg.pose.position.x) > 0.01 or \
           abs(old_target.pose.position.y - adjusted_msg.pose.position.y) > 0.01:
            target_changed = True

        self.target_pose = adjusted_msg

        # 새 목표가 설정되면 도달 플래그 리셋
        if target_changed:
            self.waypoint_reached_sent = False
            self.path_is_backward = False

            # 횡이동 진행 중이 아닐 때만 상태 리셋
            # (navigator가 주기적으로 같은 웨이포인트를 재발행하므로, 횡이동 중 리셋 방지)
            if self.lateral_total_rotations == 0:
                self.lateral_command_sent = False
                self.lateral_start_time = None
                if hasattr(self, '_waypoint_reached_published'):
                    delattr(self, '_waypoint_reached_published')

        if old_target is None or \
           abs(old_target.pose.position.x - msg.pose.position.x) > 0.01 or \
           abs(old_target.pose.position.y - msg.pose.position.y) > 0.01:

            # 🔍 검증: 현재 위치와 heading 상태 출력
            if self.current_pose is not None:
                curr_x = self.current_pose.pose.position.x
                curr_y = self.current_pose.pose.position.y
                curr_yaw = self._quaternion_to_yaw(self.current_pose.pose.orientation)

                target_x = msg.pose.position.x
                target_y = msg.pose.position.y
                dx = target_x - curr_x
                dy = target_y - curr_y
                distance = math.sqrt(dx**2 + dy**2)
                target_yaw = math.atan2(dy, dx)
                
                # 첫 번째 목표 수신 시 목표 방향을 기준 heading으로 설정
                # (첫 목표에 대한 heading 오차를 0°로 만들어 즉시 전진 가능)
                if self.reference_heading is None:
                    self.reference_heading = target_yaw
                    self.get_logger().info("=" * 70)
                    self.get_logger().info("🎯 [기준 Heading 설정]")
                    self.get_logger().info(
                        f"  현재 heading: {math.degrees(curr_yaw):+7.2f}° ({curr_yaw:+.4f} rad)"
                    )
                    self.get_logger().info(
                        f"  목표 방향: {math.degrees(target_yaw):+7.2f}° ({target_yaw:+.4f} rad)"
                    )
                    self.get_logger().info(
                        f"  → 목표 방향을 기준 heading으로 설정 (첫 목표에 대한 heading 오차 = 0°)"
                    )
                    self.get_logger().info("=" * 70)
                
                # 기준 heading에 상대적인 heading 오차 계산
                relative_curr_yaw = self._normalize_angle(curr_yaw - self.reference_heading)
                relative_target_yaw = self._normalize_angle(target_yaw - self.reference_heading)
                heading_error = self._normalize_angle(relative_target_yaw - relative_curr_yaw)

                self.get_logger().info("=" * 70)
                self.get_logger().info("📍 [새 목표 수신 - 초기 상태 분석]")
                self.get_logger().info(
                    f"  현재 위치: ({curr_x:.4f}, {curr_y:.4f}) m"
                )
                self.get_logger().info(
                    f"  현재 방향 (절대): {math.degrees(curr_yaw):+7.2f}° ({curr_yaw:+.4f} rad)"
                )
                if self.reference_heading is not None:
                    self.get_logger().info(
                        f"  기준 heading: {math.degrees(self.reference_heading):+7.2f}° ({self.reference_heading:+.4f} rad)"
                    )
                    self.get_logger().info(
                        f"  현재 방향 (상대): {math.degrees(relative_curr_yaw):+7.2f}° ({relative_curr_yaw:+.4f} rad)"
                    )
                self.get_logger().info(
                    f"  목표 위치: ({target_x:.4f}, {target_y:.4f}) m"
                )
                self.get_logger().info(
                    f"  이동 벡터: dx={dx:+.4f}, dy={dy:+.4f}, distance={distance:.4f} m"
                )
                self.get_logger().info(
                    f"  목표 방향 (절대): {math.degrees(target_yaw):+7.2f}° ({target_yaw:+.4f} rad)"
                )
                if self.reference_heading is not None:
                    self.get_logger().info(
                        f"  목표 방향 (상대): {math.degrees(relative_target_yaw):+7.2f}° ({relative_target_yaw:+.4f} rad)"
                    )
                self.get_logger().info(
                    f"  🔄 Heading 오차 (상대): {math.degrees(heading_error):+7.2f}° ({heading_error:+.4f} rad)"
                )
                self.get_logger().info(
                    f"  예상 각속도: {self.kp_angular * heading_error:.4f} rad/s "
                    f"(kp_angular × heading_error)"
                )

                # Heading tolerance 체크
                if abs(heading_error) > self.heading_tolerance:
                    self.get_logger().info(
                        f"  ⚠️  |heading_error| > tolerance ({math.degrees(self.heading_tolerance):.1f}°) "
                        f"→ 제자리 회전 먼저 수행"
                    )
                else:
                    self.get_logger().info(
                        f"  ✅ |heading_error| ≤ tolerance ({math.degrees(self.heading_tolerance):.1f}°) "
                        f"→ 회전+전진 동시 가능"
                    )
                self.get_logger().info("=" * 70)
            else:
                self.get_logger().info(
                    f"🎯 새 목표: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}) "
                    f"(현재 위치 정보 없음)"
                )

            # PID 초기화
            self.prev_distance_error = 0.0
            self.prev_heading_error = 0.0
            self.integral_distance = 0.0
            self.integral_heading = 0.0

    def control_loop(self):
        """
        제어 루프 (20Hz)

        Motion type에 따라:
        - DIFFERENTIAL: PID 제어로 전진/후진
        - LATERAL: 횡이동 제어 (joint_control)
        """
        if self.encoder_pose is None or self.target_pose is None:
            # 엔코더 odometry 또는 목표가 없으면 정지
            self.publish_cmd_vel(0.0, 0.0)
            return

        # 결속 진행 중에는 주행 정지, TYING_COMPLETE 수신 시 다음 WP로 진행
        if self.tying_in_progress:
            self.publish_cmd_vel(0.0, 0.0)
            if self.tying_complete_received:
                self.tying_in_progress = False
                self.tying_complete_received = False
                elapsed = time.monotonic() - self.waypoint_reached_time
                self.get_logger().info(
                    f"✅ 결속 완료 ({elapsed:.1f}초) → 다음 웨이포인트로 진행"
                )
                self.flog(f"WP[{self.current_waypoint_index}] 결속 완료 ({elapsed:.1f}초) → 다음 WP")
                if self.path_received and len(self.waypoint_array_x) > 0:
                    self._advance_to_next_waypoint()
                else:
                    self.publish_waypoint_reached()
            return

        # Motion type에 따라 제어 방식 선택
        if self.current_motion_type == Waypoint.MOTION_LATERAL:
            self._execute_lateral_motion()
        else:
            self._execute_differential_motion()

    def _execute_differential_motion(self):
        """Differential drive 기반 전진/후진 제어

        제어 원칙:
        - 전진/후진은 경로(웨이포인트)에서 결정 (heading_error 기반 판단 안 함)
        - 엔코더 odometry 기반 위치 추적 (VSLAM drift 영향 없음)
        - 제자리 선회 없음 (횡이동은 별도 모션으로 처리)
        """
        # 현재 위치 (엔코더 odometry - mission_origin 기준 상대좌표)
        curr_x = self.encoder_pose.pose.position.x - self.mission_origin_x
        curr_y = self.encoder_pose.pose.position.y - self.mission_origin_y
        curr_yaw = self._quaternion_to_yaw(self.encoder_pose.pose.orientation)

        # 목표 위치
        target_x = self.target_pose.pose.position.x
        target_y = self.target_pose.pose.position.y

        # 목표까지 거리 (유클리드)
        dx = target_x - curr_x
        dy = target_y - curr_y
        distance = math.sqrt(dx**2 + dy**2)

        # 경로 방향 along-path 거리 계산 (Y드리프트에 강건한 도달 판정)
        # 이전 WP → 현재 WP 방향으로 투영하여 남은 along-path 거리 산출
        along_path_remaining = distance  # fallback: 유클리드 거리
        idx = self.current_waypoint_index
        # along-path 계산: 이전WP→현재WP 방향 사용, WP[0]이면 현재WP→다음WP 방향 사용
        path_ref_valid = False
        if idx > 0 and idx < len(self.waypoint_array_x):
            prev_x = self.waypoint_array_x[idx - 1]
            prev_y = self.waypoint_array_y[idx - 1]
            path_dx = target_x - prev_x
            path_dy = target_y - prev_y
            path_ref_valid = True
        elif idx == 0 and len(self.waypoint_array_x) > 1:
            # WP[0]: 다음 WP 방향으로 경로 방향 추정
            next_x = self.waypoint_array_x[1]
            next_y = self.waypoint_array_y[1]
            path_dx = next_x - target_x
            path_dy = next_y - target_y
            path_ref_valid = True
        else:
            # 단일 WP: 출발점(0,0) → 목표 방향으로 경로 방향 설정
            path_dx = target_x
            path_dy = target_y
            path_ref_valid = True
        if path_ref_valid:
            # 경로 방향 벡터
            path_len = math.sqrt(path_dx**2 + path_dy**2)
            if path_len > 0.001:
                # 경로 방향 단위벡터
                path_ux = path_dx / path_len
                path_uy = path_dy / path_len
                # 현재 위치 → 목표까지 벡터를 경로 방향에 투영
                along_path_remaining = dx * path_ux + dy * path_uy

        # 최근접 거리 추적
        self.min_approach_distance = min(self.min_approach_distance, distance)

        # 목표 도달 확인
        waypoint_reached = False

        if distance < self.distance_tolerance:
            waypoint_reached = True
        elif along_path_remaining <= 0.0:
            # 경로 방향으로 목표 지점을 통과함 (along-path 기준)
            self.get_logger().info(
                f"📍 경로상 WP 통과: along={along_path_remaining*1000:.1f}mm, "
                f"유클리드={distance*1000:.1f}mm → 도달 판정"
            )
            waypoint_reached = True

        if waypoint_reached:
            if not self.waypoint_reached_sent:
                self.publish_cmd_vel(0.0, 0.0)
                self.waypoint_reached_sent = True
                self.waypoint_reached_time = time.monotonic()
                # VSLAM 동결 감지 타이머 리셋 (정지 중 동결 오탐 방지)
                self.vslam_last_pose_time = time.monotonic()

                # 결속 방향 결정: 전체 WP 카운트 기반 교번 (핑퐁에서도 연속 교번)
                tying_dir = 'forward' if self.total_wp_count % 2 == 0 else 'reverse'
                self.total_wp_count += 1

                self.get_logger().info(
                    f"🎯 웨이포인트 도달: 거리={distance*1000:.1f}mm "
                    f"(min={self.min_approach_distance*1000:.1f}mm) "
                    f"→ 결속 시작 [direction={tying_dir}]"
                )
                self.flog(
                    f"WP[{self.current_waypoint_index}] 도달: enc=({curr_x:.3f},{curr_y:.3f}) "
                    f"dist={distance*1000:.1f}mm → TYING_START direction={tying_dir}"
                )

                # TYING_START 발행
                tying_cmd = json.dumps({
                    'command': 'TYING_START',
                    'speed': 100,
                    'direction': tying_dir,
                })
                cmd_msg = String()
                cmd_msg.data = tying_cmd
                self.mission_command_pub.publish(cmd_msg)
                self.tying_in_progress = True
                self.tying_complete_received = False

            elif self.tying_in_progress:
                # 결속 진행 중 → TYING_COMPLETE 대기
                self.publish_cmd_vel(0.0, 0.0)
                self.vslam_last_pose_time = time.monotonic()

                if self.tying_complete_received:
                    # 결속 완료 → 다음 WP로 진행
                    self.tying_in_progress = False
                    self.tying_complete_received = False
                    elapsed = time.monotonic() - self.waypoint_reached_time
                    self.get_logger().info(
                        f"✅ 결속 완료 ({elapsed:.1f}초) → 다음 웨이포인트로 진행"
                    )
                    self.flog(f"WP[{self.current_waypoint_index}] 결속 완료 ({elapsed:.1f}초) → 다음 WP")
                    if self.path_received and len(self.waypoint_array_x) > 0:
                        self._advance_to_next_waypoint()
                    else:
                        self.publish_waypoint_reached()
                else:
                    elapsed = time.monotonic() - self.waypoint_reached_time
                    self.get_logger().info(
                        f"🔧 결속 진행 중... ({elapsed:.0f}초 경과)",
                        throttle_duration_sec=5.0
                    )
                return
            else:
                self.publish_cmd_vel(0.0, 0.0)
                return
        else:
            if self.waypoint_reached_sent:
                self.waypoint_reached_sent = False
                if hasattr(self, '_waypoint_reached_published'):
                    delattr(self, '_waypoint_reached_published')

        # Heading 보정 비활성 (직선 경로)
        # VSLAM heading drift가 심한 환경에서는 angular 보정이 오히려 역효과
        # 순수 거리 기반 직진/직후진만 수행, tolerance 내 도달 시 다음 WP로 진행
        heading_error = 0.0

        # PID 제어
        dt = 1.0 / self.control_rate

        # Distance PID
        self.integral_distance += distance * dt
        self.integral_distance = max(-self.integral_limit, min(self.integral_limit, self.integral_distance))
        derivative_distance = (distance - self.prev_distance_error) / dt
        linear_vel = (
            self.kp_linear * distance +
            self.ki_linear * self.integral_distance +
            self.kd_linear * derivative_distance
        )
        self.prev_distance_error = distance

        # min_speed 보장: tolerance 밖이면 최소 속도 이상 유지 (stiction 극복)
        if abs(linear_vel) < self.min_linear and distance > self.distance_tolerance:
            linear_vel = self.min_linear

        # 후진이면 선속도 반전
        if self.path_is_backward:
            linear_vel = -abs(linear_vel)

        # Heading PID (bearing 기반 목표 방향 추종)
        self.integral_heading += heading_error * dt
        self.integral_heading = max(-self.integral_heading_limit, min(self.integral_heading_limit, self.integral_heading))
        derivative_heading = (heading_error - self.prev_heading_error) / dt
        angular_vel = (
            self.kp_angular * heading_error +
            self.ki_angular * self.integral_heading +
            self.kd_angular * derivative_heading
        )
        self.prev_heading_error = heading_error

        # 속도 제한
        linear_vel = max(-self.max_linear, min(self.max_linear, linear_vel))
        angular_vel = max(-self.max_angular, min(self.max_angular, angular_vel))

        # 로깅
        mode_str = "후진" if self.path_is_backward else "전진"
        # VSLAM 위치도 참고 출력
        vslam_str = ""
        if self.current_pose is not None:
            vx = self.current_pose.pose.position.x
            vy = self.current_pose.pose.position.y
            vslam_str = f" | vslam=({vx:.3f},{vy:.3f})"
        self.get_logger().info(
            f"[{mode_str}] enc: ({curr_x:.3f}, {curr_y:.3f}) yaw={math.degrees(curr_yaw):.1f}° | "
            f"목표: ({target_x:.3f}, {target_y:.3f}) | "
            f"거리: {distance*1000:.1f}mm | "
            f"cmd: lin={linear_vel:.3f}, ang={angular_vel:.3f}",
            throttle_duration_sec=1.0
        )
        # 파일 로그 (1Hz throttle 대신 매 호출 기록 - 20Hz)
        self.flog(
            f"[{mode_str}] WP[{self.current_waypoint_index}] "
            f"enc=({curr_x:.3f},{curr_y:.3f}) θ={math.degrees(curr_yaw):.1f}° "
            f"→ tgt=({target_x:.3f},{target_y:.3f}) "
            f"dist={distance*1000:.1f}mm along={along_path_remaining*1000:.1f}mm "
            f"cmd=({linear_vel:.3f},{angular_vel:.3f})"
            f"{vslam_str}"
        )

        self.publish_cmd_vel(linear_vel, angular_vel)

    def _is_vslam_jump_lost(self):
        """점프 감지에 의한 VSLAM 유실인지 확인"""
        # 점프 감지로 유실된 경우, pose 수신만으로는 복구 불가
        # → pose_callback에서 점프 없이 정상 수신되면 자동 복구
        return False

    def publish_cmd_vel(self, linear, angular):
        """cmd_vel 발행 (drive_controller에서 linear 반전 처리함)"""
        self.last_cmd_linear = linear
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular

        self.cmd_vel_pub.publish(msg)

    def publish_waypoint_reached(self):
        """웨이포인트 도달 알림"""
        msg = String()
        # 좌표 정보 포함 (optional)
        if self.target_pose:
            x = self.target_pose.pose.position.x
            y = self.target_pose.pose.position.y
            msg.data = f"{x:.3f},{y:.3f}"
        else:
            msg.data = "reached"
        self.waypoint_reached_pub.publish(msg)

    def command_callback(self, msg: String):
        """미션 명령 처리 (STOP/ABORT/E-STOP 시 목표 클리어)"""
        command = msg.data
        if command in ("STOP", "ABORT_MISSION", "E-STOP"):
            self.target_pose = None
            self.reference_heading = None  # 기준 heading도 초기화
            self.publish_cmd_vel(0.0, 0.0)

    def tying_complete_callback(self, msg: String):
        """결속 완료 신호 수신 (tying_orchestrator로부터)

        형식: 'TYING_COMPLETE:N' (N=결속 포인트 수) 또는 'TYING_COMPLETE'
        """
        if not self.tying_in_progress:
            return
        data = msg.data
        if data.startswith('TYING_COMPLETE'):
            self.tying_complete_received = True
            # 포인트 수 파싱
            pts = 0
            if ':' in data:
                try:
                    pts = int(data.split(':')[1])
                except (ValueError, IndexError):
                    pts = 0
            self.total_tying_points += pts
            elapsed = time.monotonic() - self.waypoint_reached_time
            self.get_logger().info(
                f"TYING_COMPLETE 수신: {pts}pt (누적 {self.total_tying_points}/{self.max_tying_points}pt) "
                f"WP[{self.current_waypoint_index}] {elapsed:.1f}초")
            self.flog(
                f"TYING_COMPLETE 수신 (WP[{self.current_waypoint_index}]) "
                f"{pts}pt 누적={self.total_tying_points}/{self.max_tying_points}")

    def feedback_callback(self, msg: String):
        """미션 피드백 처리 (mission_done 감지용)"""
        try:
            feedback = json.loads(msg.data)
            state = feedback.get('state', '')

            # 미션 완료 또는 idle 상태가 되면 다음 미션을 위해 상태 리셋
            # repeat 핑퐁 진행 중이면 무시 (내부에서 미션 관리)
            if state in ('mission_done', 'idle'):
                if self.path_received:
                    return  # 미션 진행 중 (핑퐁 포함) → 리셋 금지
                if not self.first_waypoint_of_mission:
                    # 미션이 완료되었으므로 다음 미션을 위해 리셋
                    self.first_waypoint_of_mission = True
                    self.mission_offset_x = 0.0
                    self.mission_offset_y = 0.0
                    self.reference_heading = None
                    self.target_pose = None
                    self.target_waypoint = None
                    self.waypoint_reached_sent = False
                    self.path_is_backward = False

                    # 횡이동 상태 리셋
                    self.lateral_total_rotations = 0
                    self.lateral_current_rotation = 0
                    self.lateral_command_sent = False
                    self.lateral_start_time = None

                    self.get_logger().info(f"🔄 미션 상태 리셋 (state={state}) - 다음 미션 준비 완료")

        except Exception as e:
            pass  # JSON 파싱 실패 시 무시

    def lateral_complete_callback(self, msg: String):
        """횡이동 회전 완료 신호 처리 (joint_controller로부터, 매 회전마다 호출)"""
        if not msg.data.startswith("AUTO:"):
            return

        # 현재 횡이동 중이 아니면 무시
        if self.lateral_total_rotations == 0:
            return

        # 완료된 회전수 누적 + Y 오프셋 업데이트
        try:
            parts = msg.data.split(':')
            direction = parts[1]
            turns = int(float(parts[2]))
        except (IndexError, ValueError):
            direction = '+'
            turns = 1
        self.lateral_current_rotation += turns

        # encoder_pose Y 보정용 오프셋 누적 (m 단위)
        dy_m = turns * self.mm_per_rotation / 1000.0
        if direction == '+':
            self.lateral_y_offset += dy_m
        else:
            self.lateral_y_offset -= dy_m

        self.get_logger().info(
            f"🔄 횡이동 [{self.lateral_current_rotation}/{self.lateral_total_rotations}] 회전 완료"
        )

        if self.lateral_current_rotation >= self.lateral_total_rotations:
            self._complete_lateral_motion()

    def _complete_lateral_motion(self):
        """
        횡이동 완료 처리

        엔코더 기반으로 정확한 이동 거리를 계산하고 로그 출력
        (위치 보정은 하지 않음 - ZED odometry와 충돌 방지)
        """
        # 엔코더 기반 정확한 이동량 계산
        actual_dy = self.lateral_total_rotations * self.mm_per_rotation / 1000.0 * self.lateral_rotation_sign

        # 시작 위치가 저장되어 있다면 오차 계산 (로그용)
        if hasattr(self, 'lateral_start_y') and self.current_pose is not None:
            zed_y = self.current_pose.pose.position.y
            zed_dy = zed_y - self.lateral_start_y

            self.get_logger().info(
                f"✅ 횡이동 완료! 엔코더={actual_dy*1000:.1f}mm, ZED={zed_dy*1000:.1f}mm, "
                f"오차={(zed_dy - actual_dy)*1000:.1f}mm"
            )

            # 위치 보정은 하지 않음 (ZED odometry 사용)
            # 대신 mission_offset_y를 조정하여 다음 웨이포인트 계산에 반영
            # 엔코더 이동량과 ZED 이동량의 차이만큼 오프셋 보정
            encoder_zed_diff = actual_dy - zed_dy
            self.mission_offset_y += encoder_zed_diff

            self.get_logger().info(
                f"📍 오프셋 보정: Y offset += {encoder_zed_diff*1000:.1f}mm "
                f"(총 오프셋: {self.mission_offset_y*1000:.1f}mm)"
            )
        else:
            self.get_logger().info(
                f"✅ 횡이동 완료: {self.lateral_total_rotations}회전 완료 (엔코더 기반)"
            )

        # 웨이포인트 도달 처리
        self.publish_cmd_vel(0.0, 0.0)
        self.waypoint_reached_sent = True
        self.waypoint_reached_time = time.monotonic()

        # 상태 초기화
        self.lateral_total_rotations = 0
        self.lateral_current_rotation = 0
        self.lateral_command_sent = False
        self.lateral_start_time = None
        if hasattr(self, 'lateral_start_y'):
            delattr(self, 'lateral_start_y')

        # 결속 시작 (DIFFERENTIAL 도달과 동일)
        tying_dir = 'forward' if self.total_wp_count % 2 == 0 else 'reverse'
        self.total_wp_count += 1

        self.get_logger().info(
            f"🎯 횡이동 웨이포인트 도달 → 결속 시작 [direction={tying_dir}]"
        )

        tying_cmd = json.dumps({
            'command': 'TYING_START',
            'speed': 100,
            'direction': tying_dir,
        })
        cmd_msg = String()
        cmd_msg.data = tying_cmd
        self.mission_command_pub.publish(cmd_msg)
        self.tying_in_progress = True
        self.tying_complete_received = False

        # 결속 완료 후 _advance_to_next_waypoint()는 tying_complete_callback에서 처리
        # (DIFFERENTIAL 도달과 동일 흐름)

    def _quaternion_to_yaw(self, q):
        """Quaternion → Yaw (radian) 변환"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def _execute_lateral_motion(self):
        """
        횡이동 제어 (Y축 이동, 회전 없음)

        N회전을 한 번에 전송 (가감속 1회).
        예: 396mm → 9회전 x 360° = 3240° 일괄 전송
        엔코더 기반 완료 감지 + 위치 보정
        """
        curr_y = self.encoder_pose.pose.position.y
        target_y = self.target_pose.pose.position.y
        dy = target_y - curr_y

        # 최초 호출 시: 총 회전 횟수 계산 + 일괄 전송
        if self.lateral_total_rotations == 0:
            # 시작 위치 저장 (위치 보정용)
            self.lateral_start_y = curr_y

            # dy를 44mm 단위로 분할 (반올림)
            dy_mm = dy * 1000.0
            self.lateral_total_rotations = int(round(abs(dy_mm) / self.mm_per_rotation))
            self.lateral_current_rotation = 0

            if self.lateral_total_rotations == 0:
                # 이동 거리가 너무 작음 (< 22mm) → 즉시 완료
                self.get_logger().info(
                    f"✅ 횡이동 완료: dy={dy_mm:.1f}mm < {self.mm_per_rotation/2:.0f}mm (skip)"
                )
                self.publish_cmd_vel(0.0, 0.0)
                self.waypoint_reached_sent = True

                # 인덱스 기반 모드면 다음 웨이포인트로 자동 이동
                if self.path_received and len(self.waypoint_array_x) > 0:
                    self._advance_to_next_waypoint()
                else:
                    self.publish_waypoint_reached()
                return

            # 회전 방향 결정
            self.lateral_rotation_sign = 1.0 if dy > 0 else -1.0

            # N회전 일괄 전송 (가감속 1회)
            rotation_deg = 360.0 * self.lateral_total_rotations * self.lateral_rotation_sign

            msg = JointControl()
            msg.joint_id = 0x143
            msg.position = rotation_deg
            msg.velocity = self.lateral_speed_dps
            msg.control_mode = JointControl.MODE_RELATIVE

            self.joint_pub.publish(msg)
            self.lateral_command_sent = True

            self.get_logger().info(
                f"🔄 횡이동 시작: dy={dy_mm:.1f}mm → {self.lateral_total_rotations}회전 "
                f"({rotation_deg:+.0f}° 일괄 @ {self.lateral_speed_dps} dps)"
            )

        # 횡이동 중에는 주행 모터 완전 정지 (제자리 선회 방지, heading 보정도 안 함)
        self.publish_cmd_vel(0.0, 0.0)
        # 완료 감지는 lateral_complete_callback에서 처리됨 (엔코더 기반)

    def _normalize_angle(self, angle):
        """각도 정규화 (-π ~ π)"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = RebarController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

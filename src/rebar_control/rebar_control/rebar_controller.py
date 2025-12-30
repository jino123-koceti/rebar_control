#!/usr/bin/env python3
"""
Rebar Controller Node
경로 추종 제어

navigator로부터 목표 위치를 받아 ZED X 위치 기반으로
/cmd_vel을 발행하여 목표 지점까지 주행합니다.

제어 알고리즘: Simple PID

구독:
- /robot_pose (PoseStamped) - ZED X에서
- /mission/target_pose (PoseStamped) - navigator에서

발행:
- /cmd_vel (Twist) - drive_controller로
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from rebar_base_interfaces.msg import Waypoint, WaypointArray, JointControl
import math


class RebarController(Node):
    """경로 추종 제어 노드"""

    def __init__(self):
        super().__init__('rebar_controller')

        # 파라미터 선언
        self.declare_parameter('control_rate', 20.0)  # Hz
        self.declare_parameter('max_linear_vel', 0.5)  # m/s
        self.declare_parameter('max_angular_vel', 1.0)  # rad/s

        self.declare_parameter('distance_tolerance', 0.025)  # m (25mm)
        self.declare_parameter('heading_tolerance', 0.1)  # rad (~6도)

        # PID 파라미터
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

        # Lateral motion 파라미터
        self.lateral_tolerance = 0.005  # 5mm
        self.lateral_speed_dps = 200.0  # degrees per second
        self.mm_per_rotation = 50.0  # 50mm = 360도 = 1회전

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

        # 후진 모드 상태 유지 (목표 근처에서 heading 오차 불안정 방지)
        self.backward_mode_locked = False  # 후진 모드 잠금 플래그
        self.backward_prev_distance = None  # 후진 시 이전 거리 (지나침 감지용)

        # PID 상태
        self.prev_distance_error = 0.0
        self.prev_heading_error = 0.0
        self.integral_distance = 0.0
        self.integral_heading = 0.0

        # ROS2 구독자
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

        # 제어 루프 타이머
        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.get_logger().info("Rebar Controller 노드 초기화 완료")
        self.get_logger().info(f"  - 제어 주기: {self.control_rate} Hz")
        self.get_logger().info(f"  - 최대 선속도: {self.max_linear} m/s")
        self.get_logger().info(f"  - 최대 각속도: {self.max_angular} rad/s")

    def waypoint_array_callback(self, msg: WaypointArray):
        """
        전체 경로 수신 (tire_roller style)

        Navigator가 전체 웨이포인트 배열을 한번에 전달
        이후 내부 인덱스로 진행 관리 (외부 간섭 없음)
        """
        # 경로 저장
        self.waypoint_array_x = list(msg.x)
        self.waypoint_array_y = list(msg.y)
        self.waypoint_array_motion_type = list(msg.motion_type)
        self.waypoint_array_max_speed = list(msg.max_speed)
        self.current_waypoint_index = 0
        self.path_received = True

        # 새 경로 수신 시 reference_heading 리셋 (매 경로마다 새로 계산)
        self.reference_heading = None
        self.backward_mode_locked = False
        self.backward_prev_distance = None

        # 미션 오프셋 계산 (첫 웨이포인트를 현재 위치로)
        if self.current_pose is not None and len(self.waypoint_array_x) > 0:
            curr_x = self.current_pose.pose.position.x
            curr_y = self.current_pose.pose.position.y

            self.mission_offset_x = curr_x - self.waypoint_array_x[0]
            self.mission_offset_y = curr_y - self.waypoint_array_y[0]

            self.get_logger().info("=" * 70)
            self.get_logger().info(f"📥 전체 경로 수신: {len(self.waypoint_array_x)}개 웨이포인트")
            self.get_logger().info(f"  현재 위치: ({curr_x:.3f}, {curr_y:.3f}) m")
            self.get_logger().info(f"  첫 WP: ({self.waypoint_array_x[0]:.3f}, {self.waypoint_array_y[0]:.3f}) m")
            self.get_logger().info(f"  오프셋: ({self.mission_offset_x:.3f}, {self.mission_offset_y:.3f}) m")
            self.get_logger().info("=" * 70)

            self.first_waypoint_of_mission = False

        # 첫 번째 웨이포인트로 이동 시작
        self._set_current_waypoint_as_target()

    def _set_current_waypoint_as_target(self):
        """현재 인덱스의 웨이포인트를 목표로 설정"""
        if self.current_waypoint_index >= len(self.waypoint_array_x):
            # 모든 웨이포인트 완료
            self.get_logger().info("✅ 모든 웨이포인트 완료!")
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

        # motion_type이 변경되면 횡이동 상태 리셋
        if old_motion_type != motion_type:
            self.lateral_total_rotations = 0
            self.lateral_current_rotation = 0
            self.lateral_command_sent = False
            self.lateral_start_time = None

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
        self.backward_mode_locked = False  # 후진 모드 잠금 해제 (새 웨이포인트)
        self.backward_prev_distance = None
        if hasattr(self, '_waypoint_reached_published'):
            delattr(self, '_waypoint_reached_published')

        # 기준 heading 설정 (첫 번째 웨이포인트에서만)
        if self.reference_heading is None and self.current_pose is not None:
            curr_x = self.current_pose.pose.position.x
            curr_y = self.current_pose.pose.position.y
            dx = x - curr_x
            dy = y - curr_y
            if math.sqrt(dx**2 + dy**2) > 0.01:
                self.reference_heading = math.atan2(dy, dx)

        motion_str = "LATERAL" if motion_type == Waypoint.MOTION_LATERAL else "DIFFERENTIAL"
        self.get_logger().info(
            f"📍 웨이포인트 [{idx + 1}/{len(self.waypoint_array_x)}]: "
            f"({x:.3f}, {y:.3f}) @ {motion_str}"
        )

    def _advance_to_next_waypoint(self):
        """다음 웨이포인트로 이동"""
        # 현재 웨이포인트 완료 알림
        self._publish_waypoint_index_reached(self.current_waypoint_index)

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
        self.lateral_current_rotation = 0
        self.lateral_command_sent = False
        self.backward_mode_locked = False
        self.backward_prev_distance = None

    def mission_command_callback(self, msg: String):
        """미션 명령 처리 (CANCEL 등)"""
        command = msg.data

        if "CANCEL" in command or "STOP" in command or "ABORT" in command:
            self.get_logger().warn(f"🛑 긴급 정지 명령 수신: {command}")

            # 즉시 정지
            self.publish_cmd_vel(0.0, 0.0)

            # 목표 초기화
            self.target_pose = None
            self.target_waypoint = None

            # 미션 플래그 리셋
            self._reset_mission_state()
            self.waypoint_reached_sent = False

            self.get_logger().info("✅ 정지 완료 및 상태 초기화")

    def pose_callback(self, msg):
        """현재 위치 업데이트 (ZED X에서)"""
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

    def waypoint_callback(self, msg):
        """Enhanced Waypoint 수신 (motion type 포함)"""
        # 새로운 waypoint인지 확인 (중복 로그 방지)
        is_new_waypoint = (
            self.target_waypoint is None or
            abs(self.target_waypoint.x - msg.x) > 0.001 or
            abs(self.target_waypoint.y - msg.y) > 0.001 or
            self.target_waypoint.motion_type != msg.motion_type
        )

        # motion_type이 변경되면 횡이동 상태 강제 리셋
        # (LATERAL → DIFFERENTIAL 전환 시 이전 횡이동 상태가 남아있으면 안됨)
        if self.target_waypoint is not None and self.target_waypoint.motion_type != msg.motion_type:
            self.lateral_total_rotations = 0
            self.lateral_current_rotation = 0
            self.lateral_command_sent = False
            self.lateral_start_time = None

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
            self.backward_mode_locked = False  # 후진 모드 잠금 해제 (새 목표)
            self.backward_prev_distance = None

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
        if self.current_pose is None or self.target_pose is None:
            # 위치 또는 목표가 없으면 정지
            self.publish_cmd_vel(0.0, 0.0)
            return

        # Motion type에 따라 제어 방식 선택
        if self.current_motion_type == Waypoint.MOTION_LATERAL:
            self._execute_lateral_motion()
        else:
            self._execute_differential_motion()

    def _execute_differential_motion(self):
        """Differential drive 기반 전진/후진 제어"""
        # 현재 위치
        curr_x = self.current_pose.pose.position.x
        curr_y = self.current_pose.pose.position.y
        curr_yaw = self._quaternion_to_yaw(self.current_pose.pose.orientation)

        # 목표 위치
        target_x = self.target_pose.pose.position.x
        target_y = self.target_pose.pose.position.y

        # 목표까지 거리 및 각도
        dx = target_x - curr_x
        dy = target_y - curr_y
        distance = math.sqrt(dx**2 + dy**2)
        target_yaw = math.atan2(dy, dx)

        # 기준 heading이 설정되어 있으면 상대 heading으로 계산
        if self.reference_heading is not None:
            # 기준 heading에 상대적인 heading 계산
            relative_curr_yaw = self._normalize_angle(curr_yaw - self.reference_heading)
            relative_target_yaw = self._normalize_angle(target_yaw - self.reference_heading)
            # Heading 오차 (상대 heading 기준)
            heading_error = self._normalize_angle(relative_target_yaw - relative_curr_yaw)
        else:
            # 기준 heading이 아직 설정되지 않았으면 절대 heading 사용
            heading_error = self._normalize_angle(target_yaw - curr_yaw)
        
        # 후진 상황 감지: heading 오차가 180° 근처일 때 (카메라가 전방을 향해 있어서)
        # heading 오차가 150°~180° 또는 -150°~-180° 범위에 있으면 후진으로 판단
        #
        # 중요: 한번 후진 모드가 감지되면 유지하되, 목표를 지나치면 해제
        abs_heading_error_for_backward = abs(heading_error)
        should_be_backward = (abs_heading_error_for_backward > math.radians(150) and
                             abs_heading_error_for_backward <= math.radians(180))

        # 후진 모드 잠금/해제 로직
        if should_be_backward and not self.backward_mode_locked:
            # 후진 모드 시작
            self.backward_mode_locked = True
            self.backward_prev_distance = distance
            self.get_logger().info(
                f"🔙 후진 모드 시작: heading_error={math.degrees(heading_error):+.1f}°, 거리={distance*1000:.1f}mm"
            )
        elif self.backward_mode_locked:
            # 목표 지나침 감지: heading_error가 0°~30° 범위 AND 거리가 증가 중
            # (90°는 측면 오차일 수 있으므로 더 엄격하게 0~30°로 변경)
            distance_increasing = (self.backward_prev_distance is not None and
                                   distance > self.backward_prev_distance + 0.01)  # 10mm 이상 증가
            heading_near_forward = abs_heading_error_for_backward < math.radians(30)

            if heading_near_forward and distance_increasing:
                # 목표를 지나친 것으로 판단 (heading ~0° + 거리 증가)
                self.backward_mode_locked = False
                self.backward_prev_distance = None
                self.get_logger().info(
                    f"⚠️ 목표 지나침 감지: heading_error={math.degrees(heading_error):+.1f}°, "
                    f"거리={distance*1000:.1f}mm (증가 중) → 전진 모드로 전환"
                )
            else:
                # 거리 업데이트 (감소 추적용)
                self.backward_prev_distance = distance

        is_backward_movement = self.backward_mode_locked

        # 목표 도달 확인 (안정적인 판정: 여러 번 체크 후 도달 확정)
        if distance < self.distance_tolerance:
            if not self.waypoint_reached_sent:
                # 첫 도달 시 정지 명령만 발행 (아직 알림은 보내지 않음)
                self.publish_cmd_vel(0.0, 0.0)
                # 연속으로 tolerance 내에 있으면 도달 확정
                self.waypoint_reached_sent = True
                self.get_logger().info(
                    f"🎯 목표 근접: 거리={distance*1000:.1f}mm < tolerance={self.distance_tolerance*1000:.0f}mm, "
                    f"도달 확인 중..."
                )
            elif self.waypoint_reached_sent and not hasattr(self, '_waypoint_reached_published'):
                # 이미 tolerance 내에 있었고 계속 유지 중이면 도달 확정 (한 번만)
                self.publish_cmd_vel(0.0, 0.0)
                self._waypoint_reached_published = True  # 발행 완료 플래그
                self.get_logger().info(
                    f"✅ 웨이포인트 도달 확정: 거리={distance*1000:.1f}mm"
                )

                # 인덱스 기반 모드면 다음 웨이포인트로 자동 이동
                if self.path_received and len(self.waypoint_array_x) > 0:
                    self._advance_to_next_waypoint()
                else:
                    # 하위호환: 기존 방식 (navigator가 다음 웨이포인트 발행)
                    self.publish_waypoint_reached()
                return
            else:
                # 이미 도달 알림을 보냈으면 정지만 유지
                self.publish_cmd_vel(0.0, 0.0)
                return
        else:
            # 거리가 tolerance를 벗어나면 플래그 리셋
            if self.waypoint_reached_sent:
                self.waypoint_reached_sent = False
                if hasattr(self, '_waypoint_reached_published'):
                    delattr(self, '_waypoint_reached_published')
                self.get_logger().debug(
                    f"📍 목표에서 벗어남: 거리={distance*1000:.1f}mm > tolerance={self.distance_tolerance*1000:.0f}mm"
                )

        # PID 제어
        dt = 1.0 / self.control_rate

        # Distance PID with Anti-windup
        self.integral_distance += distance * dt
        # Anti-windup: 적분 누적 제한 (발산 방지)
        self.integral_distance = max(-self.integral_limit, min(self.integral_limit, self.integral_distance))

        derivative_distance = (distance - self.prev_distance_error) / dt
        linear_vel = (
            self.kp_linear * distance +
            self.ki_linear * self.integral_distance +
            self.kd_linear * derivative_distance
        )
        self.prev_distance_error = distance

        # 후진 상황 처리: heading 오차가 180° 근처일 때
        # (카메라가 전방을 향해 있어서 목표가 뒤에 있으면 heading 오차가 180°로 계산됨)
        if is_backward_movement:
            # 후진 시 heading 보정: 180°를 기준으로 오차 계산
            # heading_error가 175° → 후진 기준 -5° (약간 왼쪽으로 틀어짐)
            # heading_error가 -175° → 후진 기준 +5° (약간 오른쪽으로 틀어짐)
            if heading_error > 0:
                backward_heading_error = heading_error - math.pi  # 175° → -5°
            else:
                backward_heading_error = heading_error + math.pi  # -175° → +5°

            # 후진 시 heading PID (방향 보정) with Anti-windup
            self.integral_heading += backward_heading_error * dt
            self.integral_heading = max(-self.integral_heading_limit, min(self.integral_heading_limit, self.integral_heading))
            derivative_heading = (backward_heading_error - self.prev_heading_error) / dt
            angular_vel = (
                self.kp_angular * backward_heading_error +
                self.ki_angular * self.integral_heading +
                self.kd_angular * derivative_heading
            )
            self.prev_heading_error = backward_heading_error

            # 후진 시 목표 근접하면 angular 속도 감소 (heading_error ±180° 경계 불안정 방지)
            # 단, heading 보정은 어느정도 유지
            backward_short_distance = 0.05  # 50mm
            if distance < backward_short_distance:
                angular_vel *= 0.2  # 각속도 20%로 감소 (10% → 20%)
            elif distance < 0.10:  # 100mm 이하
                angular_vel *= 0.5  # 각속도 50%로 감소 (30% → 50%)

            # 선속도를 음수로 변환 (후진)
            # pose_mux가 cmd_vel의 linear.x 부호를 보고 backward 카메라를 사용함
            linear_vel = -abs(linear_vel)

            self.get_logger().debug(
                f"🔙 후진 모드: 거리={distance*1000:.1f}mm, "
                f"heading_err={math.degrees(backward_heading_error):+.1f}°, "
                f"linear={linear_vel:.3f}, angular={angular_vel:.3f}"
            )
        else:
            # 전진/측면 이동: 정상 PID 제어
            # Heading PID with Anti-windup
            self.integral_heading += heading_error * dt
            self.integral_heading = max(-self.integral_heading_limit, min(self.integral_heading_limit, self.integral_heading))
            derivative_heading = (heading_error - self.prev_heading_error) / dt
            angular_vel = (
                self.kp_angular * heading_error +
                self.ki_angular * self.integral_heading +
                self.kd_angular * derivative_heading
            )
            self.prev_heading_error = heading_error

        # 후진 모드가 아닐 때만 heading 기반 선속도 스케일링 적용
        # (후진 시에는 backward_heading_error를 사용하므로 별도 처리)
        if not is_backward_movement:
            # 짧은 거리일 때는 heading 오차를 무시하고 직진만 수행
            # (Visual Odometry 누적 오차로 인한 불필요한 회전 방지)
            short_distance_threshold = 0.15  # 15cm 이하일 때
            if distance < short_distance_threshold:
                # 짧은 거리: heading 오차 무시, 직진만 수행
                # 각속도는 최소화 (heading 오차가 매우 클 때만 작은 각속도)
                abs_heading_error = abs(heading_error)
                if abs_heading_error > math.pi / 2.0:  # 90도 이상일 때만 각속도 적용
                    # 각속도는 유지하되, 선속도는 거리 기반으로만 계산
                    pass
                else:
                    # 90도 이하: 각속도 감소 (직진 우선, 단 heading 보정 유지)
                    angular_vel *= 0.5  # 각속도 50%로 감소 (30% → 50%)
            else:
                # 긴 거리: 기존 로직 (heading 오차에 따라 선속도 스케일링)
                abs_heading_error = abs(heading_error)
                if abs_heading_error > self.heading_tolerance:
                    # heading 오차가 tolerance보다 크면 선속도를 점진적으로 감소
                    # cos 함수를 사용하여 부드러운 전환
                    # tolerance 이하: 1.0, π/2 이상: 0.0
                    max_heading_for_scaling = math.pi / 2.0  # 90도
                    if abs_heading_error >= max_heading_for_scaling:
                        heading_scale = 0.0  # 90도 이상이면 선속도 0 (제자리 회전)
                    else:
                        # heading 오차에 비례하여 선속도 스케일링 (0.0 ~ 1.0)
                        # tolerance에서 1.0, max_heading_for_scaling에서 0.0
                        normalized_error = (abs_heading_error - self.heading_tolerance) / (
                            max_heading_for_scaling - self.heading_tolerance
                        )
                        # cos 함수로 부드러운 감소 (0 ~ π/2 범위)
                        heading_scale = math.cos(normalized_error * math.pi / 2.0)
                    linear_vel *= heading_scale
                # else: heading 오차가 tolerance 이하면 선속도 100% 유지 (회전+전진 동시)

        # 속도 제한
        linear_vel = max(-self.max_linear, min(self.max_linear, linear_vel))
        angular_vel = max(-self.max_angular, min(self.max_angular, angular_vel))

        # VSLAM 정보 및 거리 오차 출력 (터미널 로깅)
        mode_str = "후진" if is_backward_movement else "전진"
        self.get_logger().info(
            f"[{mode_str}] VSLAM: ({curr_x:.3f}, {curr_y:.3f}) yaw={math.degrees(curr_yaw):.1f}° | "
            f"목표: ({target_x:.3f}, {target_y:.3f}) | "
            f"거리오차: {distance*1000:.1f}mm, 헤딩오차: {math.degrees(heading_error):.1f}° | "
            f"cmd: lin={linear_vel:.3f}, ang={angular_vel:.3f}"
        )

        # cmd_vel 발행
        self.publish_cmd_vel(linear_vel, angular_vel)

    def publish_cmd_vel(self, linear, angular):
        """cmd_vel 발행"""
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

    def feedback_callback(self, msg: String):
        """미션 피드백 처리 (mission_done 감지용)"""
        try:
            import json
            feedback = json.loads(msg.data)
            state = feedback.get('state', '')

            # 미션 완료 또는 idle 상태가 되면 다음 미션을 위해 상태 리셋
            if state in ('mission_done', 'idle'):
                if not self.first_waypoint_of_mission:
                    # 미션이 완료되었으므로 다음 미션을 위해 리셋
                    self.first_waypoint_of_mission = True
                    self.mission_offset_x = 0.0
                    self.mission_offset_y = 0.0
                    self.reference_heading = None
                    self.target_pose = None
                    self.target_waypoint = None
                    self.waypoint_reached_sent = False
                    self.backward_mode_locked = False
                    self.backward_prev_distance = None

                    # 횡이동 상태 리셋
                    self.lateral_total_rotations = 0
                    self.lateral_current_rotation = 0
                    self.lateral_command_sent = False
                    self.lateral_start_time = None

                    self.get_logger().info(f"🔄 미션 상태 리셋 (state={state}) - 다음 미션 준비 완료")

        except Exception as e:
            pass  # JSON 파싱 실패 시 무시

    def lateral_complete_callback(self, msg: String):
        """횡이동 완료 신호 처리 (joint_controller로부터)"""
        if msg.data != "COMPLETE":
            return

        # 현재 횡이동 중이 아니면 무시
        if self.lateral_total_rotations == 0:
            return

        # 현재 회전 완료
        self.lateral_current_rotation += 1

        self.get_logger().info(
            f"  ✅ [{self.lateral_current_rotation}/{self.lateral_total_rotations}] 회전 완료 (엔코더 기반)"
        )

        if self.lateral_current_rotation >= self.lateral_total_rotations:
            # 모든 회전 완료 → 위치 보정 + 웨이포인트 도달
            self._complete_lateral_motion()
        else:
            # 다음 회전 준비
            self.lateral_command_sent = False
            self.lateral_start_time = None

    def _complete_lateral_motion(self):
        """
        횡이동 완료 처리

        엔코더 기반으로 정확한 이동 거리를 계산하고 로그 출력
        (위치 보정은 하지 않음 - ZED odometry와 충돌 방지)
        """
        # 엔코더 기반 정확한 이동량 계산
        actual_dy = self.lateral_total_rotations * self.mm_per_rotation / 1000.0 * self.lateral_rotation_sign

        # 시작 위치가 저장되어 있다면 오차 계산 (로그용)
        if hasattr(self, 'lateral_start_y'):
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

        # 상태 초기화
        self.lateral_total_rotations = 0
        self.lateral_current_rotation = 0
        self.lateral_command_sent = False
        self.lateral_start_time = None
        if hasattr(self, 'lateral_start_y'):
            delattr(self, 'lateral_start_y')

        # 인덱스 기반 모드면 다음 웨이포인트로 자동 이동
        if self.path_received and len(self.waypoint_array_x) > 0:
            self._advance_to_next_waypoint()
        else:
            # 하위호환: 기존 방식 (navigator가 다음 웨이포인트 발행)
            self.publish_waypoint_reached()

    def _quaternion_to_yaw(self, q):
        """Quaternion → Yaw (radian) 변환"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def _execute_lateral_motion(self):
        """
        횡이동 제어 (Y축 이동, 회전 없음)

        50mm 단위로 분할하여 1회전(360°)씩 실행.
        예: 100mm → 360° + 360° (2회 전송)
        엔코더 기반 완료 감지 + 위치 보정
        """
        curr_y = self.current_pose.pose.position.y
        target_y = self.target_pose.pose.position.y
        dy = target_y - curr_y

        # 최초 호출 시: 총 회전 횟수 계산
        if self.lateral_total_rotations == 0:
            # 시작 위치 저장 (위치 보정용)
            self.lateral_start_y = curr_y

            # dy를 50mm 단위로 분할 (반올림)
            dy_mm = dy * 1000.0
            self.lateral_total_rotations = int(round(abs(dy_mm) / self.mm_per_rotation))
            self.lateral_current_rotation = 0

            if self.lateral_total_rotations == 0:
                # 이동 거리가 너무 작음 (< 25mm) → 즉시 완료
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

            self.get_logger().info(
                f"🔄 횡이동 시작: dy={dy_mm:.1f}mm → {self.lateral_total_rotations}회전 "
                f"({self.lateral_total_rotations}x360° @ {self.lateral_speed_dps} dps, 엔코더 기반)"
            )

        # 현재 회전 명령 전송 (한 번만)
        if not self.lateral_command_sent:
            rotation_deg = 360.0 * self.lateral_rotation_sign

            msg = JointControl()
            msg.joint_id = 0x143
            msg.position = rotation_deg
            msg.velocity = self.lateral_speed_dps
            msg.control_mode = JointControl.MODE_RELATIVE

            self.joint_pub.publish(msg)
            self.lateral_command_sent = True

            self.get_logger().info(
                f"🔄 횡이동 [{self.lateral_current_rotation + 1}/{self.lateral_total_rotations}]: "
                f"{rotation_deg:+.0f}° 전송 (완료 대기 중...)"
            )

        # 완료 감지는 lateral_complete_callback에서 처리됨 (엔코더 기반)

        # Heading 유지를 위한 cmd_vel 발행 (각속도만)
        if self.reference_heading is not None:
            curr_yaw = self._quaternion_to_yaw(self.current_pose.pose.orientation)
            heading_error = self._normalize_angle(self.reference_heading - curr_yaw)
            angular_z = heading_error * 1.0  # kp_angular
            self.publish_cmd_vel(0.0, angular_z)

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

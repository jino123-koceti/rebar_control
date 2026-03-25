#!/usr/bin/env python3
"""
Encoder Odometry Node
주행 모터(0x141, 0x142) 엔코더 피드백 기반 odometry 생성

odom_mode 파라미터로 위치 계산 방식 선택:
- 'speed': dps 적분 방식 (기존) - 0xA2 응답의 speed 필드 사용
- 'position': 0x92 multiturn position 차분 방식 (신규) - 절대 각도 기반, 드리프트 없음

공통:
- Differential drive kinematics로 위치/heading 계산
- /encoder_odom (PoseStamped) 발행
"""

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rebar_base_interfaces.msg import MotorFeedback
from std_msgs.msg import String


class EncoderOdom(Node):
    def __init__(self):
        super().__init__('encoder_odom')

        # 파라미터
        self.declare_parameter('wheel_radius', 0.02865)  # m (1 rev = 0.18m)
        self.declare_parameter('wheel_base', 0.5)  # m (바퀴 간 거리)
        self.declare_parameter('publish_rate', 20.0)  # Hz
        self.declare_parameter('left_motor_id', 0x41)  # CAN parser에서 변환된 ID
        self.declare_parameter('right_motor_id', 0x42)
        # dps → 실제 이동거리 보정 계수 (캘리브레이션 후 조정, speed 모드 전용)
        self.declare_parameter('dps_to_mps_scale', 1.0)
        # 오도메트리 모드: 'speed' (기존 dps 적분) / 'position' (0x92 각도 차분)
        self.declare_parameter('odom_mode', 'position')

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.left_motor_id = self.get_parameter('left_motor_id').value
        self.right_motor_id = self.get_parameter('right_motor_id').value
        self.dps_to_mps_scale = self.get_parameter('dps_to_mps_scale').value
        self.odom_mode = self.get_parameter('odom_mode').value

        # 상태 변수 (공통)
        self.x = 0.0  # m
        self.y = 0.0  # m
        self.theta = 0.0  # rad

        # ===== speed 모드 상태 변수 (기존) =====
        self.left_speed_dps = 0.0  # 최신 좌측 모터 속도 (dps)
        self.right_speed_dps = 0.0  # 최신 우측 모터 속도 (dps)
        self.last_update_time = None

        # 모터 피드백 수신 타임스탬프 (alive 체크)
        self.left_last_time = 0.0
        self.right_last_time = 0.0

        # ===== position 모드 상태 변수 (신규) =====
        # 0x92 multiturn angle (degree) - 절대값
        self.left_angle_deg = None   # 최신 좌측 모터 각도 (초기값 None = 미수신)
        self.right_angle_deg = None  # 최신 우측 모터 각도
        self.left_prev_angle_deg = None   # 이전 주기 좌측 각도
        self.right_prev_angle_deg = None  # 이전 주기 우측 각도
        self.left_angle_updated = False   # 이번 주기에 새 값 수신 여부
        self.right_angle_updated = False
        # mm per degree: 바퀴 둘레(2πr) / 360°
        self.mm_per_deg = (2.0 * math.pi * self.wheel_radius * 1000.0) / 360.0

        # Subscribers
        self.motor_feedback_sub = self.create_subscription(
            MotorFeedback,
            '/motor_feedback',
            self.motor_feedback_callback,
            10
        )

        # 리셋 명령 구독
        self.reset_sub = self.create_subscription(
            String,
            '/encoder_odom/reset',
            self.reset_callback,
            10
        )

        # Publishers
        self.odom_pub = self.create_publisher(
            PoseStamped,
            '/encoder_odom',
            10
        )

        # 주기적 odometry 계산 및 발행
        self.timer = self.create_timer(1.0 / self.publish_rate, self.update_odom)

        self.get_logger().info("Encoder Odometry 노드 초기화 완료")
        self.get_logger().info(f"  - odom_mode: {self.odom_mode}")
        self.get_logger().info(f"  - wheel_radius: {self.wheel_radius} m")
        self.get_logger().info(f"  - wheel_base: {self.wheel_base} m")
        self.get_logger().info(f"  - mm_per_deg: {self.mm_per_deg:.4f} mm/°")
        if self.odom_mode == 'speed':
            self.get_logger().info(f"  - dps_to_mps_scale: {self.dps_to_mps_scale}")
        self.get_logger().info(f"  - publish_rate: {self.publish_rate} Hz")

    def motor_feedback_callback(self, msg: MotorFeedback):
        """모터 피드백에서 좌우 drive motor 데이터 추출"""
        now = time.monotonic()

        if self.odom_mode == 'position':
            # position 모드: 0x92 응답에서 multiturn angle 추출
            if msg.status == 0x92:
                if msg.motor_id == self.left_motor_id:
                    self.left_angle_deg = msg.current_position  # degree
                    self.left_angle_updated = True
                    self.left_last_time = now
                elif msg.motor_id == self.right_motor_id:
                    # 오른쪽 모터: 물리적으로 반대 방향 장착 → 부호 반전
                    self.right_angle_deg = -msg.current_position
                    self.right_angle_updated = True
                    self.right_last_time = now
        else:
            # speed 모드 (기존): 0xA2 응답에서 speed 추출
            if msg.motor_id == self.left_motor_id and msg.status == 0xA2:
                self.left_speed_dps = msg.current_speed  # dps (int16)
                self.left_last_time = now
            elif msg.motor_id == self.right_motor_id and msg.status == 0xA2:
                # 오른쪽 모터: can_sender에서 부호 반전하여 명령 → 피드백도 반전되어 옴
                self.right_speed_dps = -msg.current_speed  # 부호 복원
                self.right_last_time = now

    def update_odom(self):
        """주기적 odometry 계산 및 발행"""
        if self.odom_mode == 'position':
            self._update_odom_position()
        else:
            self._update_odom_speed()

    def _update_odom_position(self):
        """position 모드: 0x92 multiturn angle 차분 기반 odometry"""
        # 양쪽 모두 최초 수신 전이면 스킵
        if self.left_angle_deg is None or self.right_angle_deg is None:
            return

        # 이전 값이 없으면 초기화 (첫 주기)
        if self.left_prev_angle_deg is None:
            self.left_prev_angle_deg = self.left_angle_deg
            self.right_prev_angle_deg = self.right_angle_deg
            return

        # 새 데이터가 양쪽 모두 없으면 발행만 (위치 변화 없음)
        if not self.left_angle_updated and not self.right_angle_updated:
            self._publish_pose()
            return

        # 각도 차분 계산 (degree)
        d_left_deg = self.left_angle_deg - self.left_prev_angle_deg
        d_right_deg = self.right_angle_deg - self.right_prev_angle_deg

        # 이전 값 업데이트
        self.left_prev_angle_deg = self.left_angle_deg
        self.right_prev_angle_deg = self.right_angle_deg
        self.left_angle_updated = False
        self.right_angle_updated = False

        # 각도 → 이동거리 (mm → m)
        d_left_m = d_left_deg * self.mm_per_deg / 1000.0
        d_right_m = d_right_deg * self.mm_per_deg / 1000.0

        # Differential drive kinematics
        d_linear = (d_left_m + d_right_m) / 2.0
        d_theta = (d_right_m - d_left_m) / self.wheel_base

        # 위치 적분 (2nd order midpoint)
        half_dtheta = d_theta / 2.0
        self.x += d_linear * math.cos(self.theta + half_dtheta)
        self.y += d_linear * math.sin(self.theta + half_dtheta)
        self.theta += d_theta

        # theta 정규화 (-pi ~ pi)
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # 발행
        self._publish_pose()

        # 디버그 로깅 (움직일 때만, 1초 throttle)
        if abs(d_linear) > 0.0005 or abs(d_theta) > 0.001:
            self.get_logger().info(
                f"[ODOM:pos] x={self.x:.3f} y={self.y:.3f} θ={math.degrees(self.theta):.1f}° "
                f"| Δ={d_linear*1000:.1f}mm Δθ={math.degrees(d_theta):.2f}° "
                f"| L={self.left_angle_deg:.1f}° R={self.right_angle_deg:.1f}°",
                throttle_duration_sec=1.0
            )

    def _update_odom_speed(self):
        """speed 모드 (기존): dps 적분 기반 odometry"""
        now = time.monotonic()

        if self.last_update_time is None:
            self.last_update_time = now
            return

        dt = now - self.last_update_time
        self.last_update_time = now

        if dt <= 0 or dt > 1.0:
            return  # 비정상 dt 무시

        # 모터 피드백 alive 체크 (0.5초 이상 수신 없으면 속도 0으로)
        if now - self.left_last_time > 0.5:
            self.left_speed_dps = 0.0
        if now - self.right_last_time > 0.5:
            self.right_speed_dps = 0.0

        # dps → m/s 변환
        left_omega = self.left_speed_dps * math.pi / 180.0  # rad/s
        right_omega = self.right_speed_dps * math.pi / 180.0

        left_v = left_omega * self.wheel_radius * self.dps_to_mps_scale
        right_v = right_omega * self.wheel_radius * self.dps_to_mps_scale

        # Differential drive kinematics
        # 물리적 방향 그대로 출력 (UI 표시와 일치)
        # 제어 좌표 변환은 rebar_controller/navigator에서 처리
        v_linear = (left_v + right_v) / 2.0
        v_angular = (right_v - left_v) / self.wheel_base

        # 위치 적분 (2nd order midpoint)
        half_dtheta = v_angular * dt / 2.0
        self.x += v_linear * dt * math.cos(self.theta + half_dtheta)
        self.y += v_linear * dt * math.sin(self.theta + half_dtheta)
        self.theta += v_angular * dt

        # theta 정규화 (-pi ~ pi)
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # 발행
        self._publish_pose()

        # 디버그 로깅 (움직일 때만, 1초 throttle)
        if abs(v_linear) > 0.001 or abs(v_angular) > 0.01:
            self.get_logger().info(
                f"[ODOM:spd] x={self.x:.3f} y={self.y:.3f} θ={math.degrees(self.theta):.1f}° "
                f"| v={v_linear:.3f}m/s ω={math.degrees(v_angular):.1f}°/s "
                f"| L={self.left_speed_dps:.0f}dps R={self.right_speed_dps:.0f}dps",
                throttle_duration_sec=1.0
            )

    def _publish_pose(self):
        """PoseStamped 메시지 발행"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'odom'
        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.position.z = 0.0

        # yaw → quaternion
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = math.sin(self.theta / 2.0)
        pose_msg.pose.orientation.w = math.cos(self.theta / 2.0)

        self.odom_pub.publish(pose_msg)

    def reset_callback(self, msg: String):
        """odometry 리셋 (새 미션 시작 시)"""
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        if self.odom_mode == 'position':
            # position 모드: 현재 각도를 기준점으로 재설정
            if self.left_angle_deg is not None:
                self.left_prev_angle_deg = self.left_angle_deg
            if self.right_angle_deg is not None:
                self.right_prev_angle_deg = self.right_angle_deg
            self.left_angle_updated = False
            self.right_angle_updated = False
        else:
            self.last_update_time = time.monotonic()

        self.get_logger().info(f"Odometry 리셋 [{self.odom_mode}]: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdom()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Encoder Odometry Node
주행 모터(0x141, 0x142) 엔코더 피드백 기반 odometry 생성

- /motor_feedback에서 좌우 모터 speed(dps) 수신
- Differential drive kinematics로 위치/heading 적분
- /encoder_odom (PoseStamped) 발행

dps → m/s 변환:
  omega_rad = speed_dps * pi / 180
  v_wheel = omega_rad * wheel_radius
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
        # dps → 실제 이동거리 보정 계수 (캘리브레이션 후 조정)
        self.declare_parameter('dps_to_mps_scale', 1.0)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.left_motor_id = self.get_parameter('left_motor_id').value
        self.right_motor_id = self.get_parameter('right_motor_id').value
        self.dps_to_mps_scale = self.get_parameter('dps_to_mps_scale').value

        # 상태 변수
        self.x = 0.0  # m
        self.y = 0.0  # m
        self.theta = 0.0  # rad

        self.left_speed_dps = 0.0  # 최신 좌측 모터 속도 (dps)
        self.right_speed_dps = 0.0  # 최신 우측 모터 속도 (dps)
        self.last_update_time = None

        # 모터 피드백 수신 타임스탬프 (alive 체크)
        self.left_last_time = 0.0
        self.right_last_time = 0.0

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
        self.get_logger().info(f"  - wheel_radius: {self.wheel_radius} m")
        self.get_logger().info(f"  - wheel_base: {self.wheel_base} m")
        self.get_logger().info(f"  - dps_to_mps_scale: {self.dps_to_mps_scale}")
        self.get_logger().info(f"  - publish_rate: {self.publish_rate} Hz")

    def motor_feedback_callback(self, msg: MotorFeedback):
        """모터 피드백에서 좌우 drive motor speed 추출"""
        now = time.monotonic()

        if msg.motor_id == self.left_motor_id and msg.status == 0xA2:
            self.left_speed_dps = msg.current_speed  # dps (int16)
            self.left_last_time = now
        elif msg.motor_id == self.right_motor_id and msg.status == 0xA2:
            # 오른쪽 모터: can_sender에서 부호 반전하여 명령 → 피드백도 반전되어 옴
            self.right_speed_dps = -msg.current_speed  # 부호 복원
            self.right_last_time = now

    def update_odom(self):
        """주기적 odometry 적분 및 발행"""
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

        # PoseStamped 발행
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

        # 디버그 로깅 (움직일 때만, 1초 throttle)
        if abs(v_linear) > 0.001 or abs(v_angular) > 0.01:
            self.get_logger().info(
                f"[ODOM] x={self.x:.3f} y={self.y:.3f} θ={math.degrees(self.theta):.1f}° "
                f"| v={v_linear:.3f}m/s ω={math.degrees(v_angular):.1f}°/s "
                f"| L={self.left_speed_dps:.0f}dps R={self.right_speed_dps:.0f}dps",
                throttle_duration_sec=1.0
            )

    def reset_callback(self, msg: String):
        """odometry 리셋 (새 미션 시작 시)"""
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_update_time = time.monotonic()
        self.get_logger().info(f"🔄 Odometry 리셋: {msg.data}")


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

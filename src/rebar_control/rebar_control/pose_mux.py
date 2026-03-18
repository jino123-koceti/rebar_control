#!/usr/bin/env python3
"""
Pose Mux Node

Selects between front/back odometry topics based on driving direction
and republishes a unified PoseStamped on /robot_pose (configurable).

Direction source (default): cmd_vel linear.x sign.
When |linear.x| <= velocity_deadband, keeps last selection; optionally
prefers front sensor when stationary.

Back camera is mounted 180° opposite to front camera, so its odom
is transformed (X, Y negated, yaw rotated 180°) to match front frame.

Camera offset compensation:
  Each camera is mounted at a fixed offset from the robot center along the
  forward axis. During in-place rotation the camera traces an arc, causing
  VSLAM to report translational movement. We compensate by transforming
  camera position → robot center using the heading angle:
    robot_x = cam_x - offset * cos(yaw)
    robot_y = cam_y - offset * sin(yaw)
"""

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class PoseMux(Node):
    """Select odom from front/back sensors and publish a unified pose."""

    def __init__(self):
        super().__init__('pose_mux')

        # Parameters
        self.declare_parameter('front_odom_topic', '/zed_front/odom')
        self.declare_parameter('back_odom_topic', '/zed_back/odom')
        self.declare_parameter('output_topic', '/robot_pose')
        self.declare_parameter('direction_source', 'cmd_vel')  # cmd_vel | topic
        self.declare_parameter('direction_topic', '/travel_direction')  # used when direction_source == 'topic'
        self.declare_parameter('velocity_deadband', 0.02)  # m/s
        self.declare_parameter('prefer_front_when_stationary', True)
        self.declare_parameter('output_frame', 'odom')
        self.declare_parameter('front_camera_offset', -0.49)   # front cam offset from robot center (m)
        self.declare_parameter('back_camera_offset', 0.49)     # back cam offset from robot center (m)
        self.declare_parameter('min_switch_hold_sec', 1.0)     # 카메라 전환 최소 유지 시간 (초)

        self.front_topic = self.get_parameter('front_odom_topic').value
        self.back_topic = self.get_parameter('back_odom_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.direction_source = self.get_parameter('direction_source').value
        self.direction_topic = self.get_parameter('direction_topic').value
        self.velocity_deadband = float(self.get_parameter('velocity_deadband').value)
        self.prefer_front_when_stationary = bool(
            self.get_parameter('prefer_front_when_stationary').value
        )
        self.output_frame = self.get_parameter('output_frame').value
        self.front_camera_offset = float(self.get_parameter('front_camera_offset').value)
        self.back_camera_offset = float(self.get_parameter('back_camera_offset').value)
        self.min_switch_hold_sec = float(self.get_parameter('min_switch_hold_sec').value)

        # Internal state
        self.last_direction = 'forward'  # forward | backward
        self.active_source = 'front'     # 현재 발행 중인 소스
        self.last_speed = 0.0
        self.last_switch_time = 0.0      # 마지막 카메라 전환 시각

        # 전환 오프셋 보정
        self.last_front_pose = None  # 최신 front 카메라 로봇 중심 pose (x,y,yaw)
        self.last_back_pose = None   # 최신 back 카메라 로봇 중심 pose (x,y,yaw)
        self.offset_x = 0.0  # back pose에 더할 보정값
        self.offset_y = 0.0
        self.front_drift_x = 0.0  # front pose에 더할 보정값
        self.front_drift_y = 0.0

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, self.output_topic, 10)

        # Subscriptions
        self.front_sub = self.create_subscription(
            Odometry, self.front_topic, self.front_odom_callback, 10
        )
        self.back_sub = self.create_subscription(
            Odometry, self.back_topic, self.back_odom_callback, 10
        )

        if self.direction_source == 'cmd_vel':
            self.cmd_vel_sub = self.create_subscription(
                Twist, '/cmd_vel', self.cmd_vel_callback, 10
            )
        elif self.direction_source == 'topic':
            self.dir_topic_sub = self.create_subscription(
                String, self.direction_topic, self.direction_topic_callback, 10
            )
        else:
            self.get_logger().warn(
                f"Unknown direction_source '{self.direction_source}', defaulting to cmd_vel"
            )
            self.direction_source = 'cmd_vel'
            self.cmd_vel_sub = self.create_subscription(
                Twist, '/cmd_vel', self.cmd_vel_callback, 10
            )

        self.get_logger().info(
            f"PoseMux started. front='{self.front_topic}', back='{self.back_topic}', output='{self.output_topic}'"
        )
        self.get_logger().info(
            f"Camera offsets: front={self.front_camera_offset:.3f}m, back={self.back_camera_offset:.3f}m"
        )

    # Direction updates
    def cmd_vel_callback(self, msg: Twist):
        vx = msg.linear.x
        self.last_speed = vx
        # rebar_controller: positive cmd_vel = forward, negative = backward
        # drive_controller가 linear 반전 처리하므로 여기서는 원래 부호 기준
        if vx > self.velocity_deadband:
            self.last_direction = 'forward'
        elif vx < -self.velocity_deadband:
            self.last_direction = 'backward'
        # within deadband: keep last_direction

    def direction_topic_callback(self, msg: String):
        data = msg.data.lower()
        if 'back' in data:
            self.last_direction = 'backward'
        elif 'forward' in data or 'front' in data:
            self.last_direction = 'forward'
        # otherwise ignore

    # Odom handling
    def _get_robot_pose_from_front(self, odom: Odometry):
        """front odom → 로봇 중심 좌표 (x, y, yaw) 계산 (반전+오프셋 보정)"""
        x = -odom.pose.pose.position.x
        y = -odom.pose.pose.position.y
        yaw = self._quat_to_yaw(odom.pose.pose.orientation)
        x -= self.front_camera_offset * math.cos(yaw)
        y -= self.front_camera_offset * math.sin(yaw)
        return x, y, yaw

    def _get_robot_pose_from_back(self, odom: Odometry):
        """back odom → 로봇 중심 좌표 (x, y, yaw) 계산 (오프셋 보정)"""
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        yaw = self._quat_to_yaw(odom.pose.pose.orientation)
        x -= self.back_camera_offset * math.cos(yaw)
        y -= self.back_camera_offset * math.sin(yaw)
        return x, y, yaw

    def _can_switch(self) -> bool:
        """최소 유지 시간이 경과했는지 확인"""
        return (time.monotonic() - self.last_switch_time) >= self.min_switch_hold_sec

    def front_odom_callback(self, msg: Odometry):
        # 항상 최신 front pose 기록
        self.last_front_pose = self._get_robot_pose_from_front(msg)

        if self.active_source == 'front':
            # 현재 front 사용 중 → 그대로 발행
            self.publish_pose_transformed(msg)
        elif self.should_use_front() and self._can_switch():
            # 전환 조건 충족 + 최소 유지 시간 경과 → 전환
            self._switch_to('front')
            self.publish_pose_transformed(msg)

    def back_odom_callback(self, msg: Odometry):
        # 항상 최신 back pose 기록
        self.last_back_pose = self._get_robot_pose_from_back(msg)

        if self.active_source == 'back':
            # 현재 back 사용 중 → 그대로 발행
            self.publish_pose_with_offset(msg)
        elif self.should_use_back() and self._can_switch():
            # 전환 조건 충족 + 최소 유지 시간 경과 → 전환
            self._switch_to('back')
            self.publish_pose_with_offset(msg)

    def _switch_to(self, new_source: str):
        """카메라 전환 시 오프셋 계산"""
        old_source = self.active_source

        if new_source == 'back' and self.last_front_pose and self.last_back_pose:
            # front → back: front pose를 기준으로 back에 오프셋 적용
            fx, fy, _ = self.last_front_pose
            bx, by, _ = self.last_back_pose
            self.offset_x = fx - bx
            self.offset_y = fy - by
            self.get_logger().info(
                f'📷 카메라 전환: {old_source}→{new_source}, '
                f'오프셋=({self.offset_x:.3f}, {self.offset_y:.3f})m'
            )
        elif new_source == 'front' and self.last_front_pose and self.last_back_pose:
            # back → front: back+offset을 기준으로 front 오프셋 리셋
            bx, by, _ = self.last_back_pose
            corrected_x = bx + self.offset_x
            corrected_y = by + self.offset_y
            fx, fy, _ = self.last_front_pose
            # front에는 오프셋 없이 사용하되, 누적 보정값 전달 안 함
            # front의 odom이 독립적이므로 전환 시점 차이만 보정
            old_offset_x = self.offset_x
            old_offset_y = self.offset_y
            self.offset_x = 0.0
            self.offset_y = 0.0
            # front pose와 보정된 back pose의 차이로 front에도 보정 필요 시 적용
            drift_x = corrected_x - fx
            drift_y = corrected_y - fy
            if abs(drift_x) > 0.01 or abs(drift_y) > 0.01:
                self.front_drift_x = drift_x
                self.front_drift_y = drift_y
            else:
                self.front_drift_x = 0.0
                self.front_drift_y = 0.0
            self.get_logger().info(
                f'📷 카메라 전환: {old_source}→{new_source}, '
                f'front보정=({self.front_drift_x:.3f}, {self.front_drift_y:.3f})m'
            )

        self.active_source = new_source
        self.last_switch_time = time.monotonic()

    def transform_odom(self, odom: Odometry) -> Odometry:
        """전후면 반전 보정: front 카메라의 +X가 로봇 새 전진 방향과 반대이므로 X/Y 반전.

        - Position: X → -X, Y → -Y (Z unchanged)
        - Orientation: 유지 (rebar_controller가 heading_error로 전진/후진 판단)
        """
        transformed = Odometry()
        transformed.header = odom.header
        transformed.child_frame_id = odom.child_frame_id

        transformed.pose.pose.position.x = -odom.pose.pose.position.x
        transformed.pose.pose.position.y = -odom.pose.pose.position.y
        transformed.pose.pose.position.z = odom.pose.pose.position.z
        transformed.pose.pose.orientation = odom.pose.pose.orientation

        transformed.pose.covariance = odom.pose.covariance
        transformed.twist = odom.twist

        return transformed

    @staticmethod
    def _quat_to_yaw(q):
        """Quaternion → yaw (rad)"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _compensate_camera_offset(self, pose_msg, camera_offset):
        """카메라 위치 → 로봇 중심 위치로 변환

        camera is at robot_center + offset * [cos(yaw), sin(yaw)]
        so: robot_center = camera_pos - offset * [cos(yaw), sin(yaw)]
        """
        yaw = self._quat_to_yaw(pose_msg.orientation)
        pose_msg.position.x -= camera_offset * math.cos(yaw)
        pose_msg.position.y -= camera_offset * math.sin(yaw)
        return pose_msg

    def publish_pose_transformed(self, odom: Odometry):
        """Publish front camera odom with X/Y negation + offset compensation + drift correction."""
        transformed = self.transform_odom(odom)
        pose = PoseStamped()
        pose.header.stamp = transformed.header.stamp
        pose.header.frame_id = self.output_frame
        pose.pose = self._compensate_camera_offset(
            transformed.pose.pose, self.front_camera_offset
        )
        # back→front 전환 시 누적 보정
        if hasattr(self, 'front_drift_x'):
            pose.pose.position.x += self.front_drift_x
            pose.pose.position.y += self.front_drift_y
        self.pose_pub.publish(pose)

    def should_use_front(self) -> bool:
        if abs(self.last_speed) <= self.velocity_deadband:
            return self.prefer_front_when_stationary or self.last_direction == 'forward'
        return self.last_direction == 'forward'

    def should_use_back(self) -> bool:
        if abs(self.last_speed) <= self.velocity_deadband:
            return (not self.prefer_front_when_stationary) and self.last_direction == 'backward'
        return self.last_direction == 'backward'

    def publish_pose_with_offset(self, odom: Odometry):
        """Publish back camera odom with offset compensation + switching offset."""
        pose = PoseStamped()
        pose.header.stamp = odom.header.stamp
        pose.header.frame_id = self.output_frame
        pose.pose = self._compensate_camera_offset(
            odom.pose.pose, self.back_camera_offset
        )
        # front→back 전환 시 오프셋 보정
        pose.pose.position.x += self.offset_x
        pose.pose.position.y += self.offset_y
        self.pose_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = PoseMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

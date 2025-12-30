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
"""

import math
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

        # Internal state
        self.last_direction = 'forward'  # forward | backward
        self.last_speed = 0.0

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

    # Direction updates
    def cmd_vel_callback(self, msg: Twist):
        vx = msg.linear.x
        self.last_speed = vx
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
    def front_odom_callback(self, msg: Odometry):
        if self.should_use_front():
            self.publish_pose(msg)

    def back_odom_callback(self, msg: Odometry):
        if self.should_use_back():
            self.publish_pose_back_transformed(msg)

    def transform_back_odom(self, odom: Odometry) -> Odometry:
        """Transform back camera odom to front camera frame.

        Back camera is mounted opposite to front, so:
        - Position: X → -X, Y → -Y (Z unchanged)
        - Orientation: NOT transformed (keep original)
          → rebar_controller uses heading error ~180° to detect backward motion
          → If we transform heading, it causes oscillation between forward/backward
        """
        transformed = Odometry()
        transformed.header = odom.header
        transformed.child_frame_id = odom.child_frame_id

        # Negate X and Y position
        transformed.pose.pose.position.x = -odom.pose.pose.position.x
        transformed.pose.pose.position.y = -odom.pose.pose.position.y
        transformed.pose.pose.position.z = odom.pose.pose.position.z

        # Keep orientation as-is (NO 180° rotation)
        # This way, rebar_controller sees heading_error ~180° and correctly
        # switches to backward mode, sending negative linear.x
        transformed.pose.pose.orientation = odom.pose.pose.orientation

        # Copy covariance (simplified - proper transform would rotate covariance too)
        transformed.pose.covariance = odom.pose.covariance
        transformed.twist = odom.twist

        return transformed

    def publish_pose_back_transformed(self, odom: Odometry):
        """Publish back camera odom with 180° transformation."""
        transformed = self.transform_back_odom(odom)
        pose = PoseStamped()
        pose.header.stamp = transformed.header.stamp
        pose.header.frame_id = self.output_frame
        pose.pose = transformed.pose.pose
        self.pose_pub.publish(pose)

    def should_use_front(self) -> bool:
        if abs(self.last_speed) <= self.velocity_deadband:
            return self.prefer_front_when_stationary or self.last_direction == 'forward'
        return self.last_direction == 'forward'

    def should_use_back(self) -> bool:
        if abs(self.last_speed) <= self.velocity_deadband:
            return (not self.prefer_front_when_stationary) and self.last_direction == 'backward'
        return self.last_direction == 'backward'

    def publish_pose(self, odom: Odometry):
        pose = PoseStamped()
        pose.header.stamp = odom.header.stamp
        pose.header.frame_id = self.output_frame
        pose.pose = odom.pose.pose
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

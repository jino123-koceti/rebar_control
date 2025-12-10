#!/usr/bin/env python3
"""
Temporary cmd_vel relay node
/cmd_vel_precise -> /cmd_vel
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')

        self.sub = self.create_subscription(
            Twist, '/cmd_vel_precise', self.callback, 10
        )
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('cmd_vel relay: /cmd_vel_precise -> /cmd_vel')

    def callback(self, msg):
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

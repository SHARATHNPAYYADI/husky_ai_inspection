#!/usr/bin/env python3
"""
Odom Deadband Filter Node
--------------------------
Filters out physics noise from Isaac Sim odometry before it reaches Nav2/AMCL.

- Zeroes out z-axis linear and x/y angular (ground robot can't fly or roll/pitch)
- Applies deadband threshold to linear x and angular z
- Republishes clean odom from /odom_raw → /odom

Usage:
    1. In Isaac Sim OmniGraph, change the ROS2 Publish Odometry topic from /odom → /odom_raw
    2. Run this node alongside Nav2:
       ros2 run <your_package> odom_deadband
    3. All Nav2 nodes will now receive clean odometry on /odom
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomDeadband(Node):
    def __init__(self):
        super().__init__('odom_deadband')

        # Deadband thresholds — tune these if needed
        self.LINEAR_DEADBAND  = 0.005   # m/s  — ignore linear x below this
        self.ANGULAR_DEADBAND = 0.05    # rad/s — ignore angular z below this

        self.sub = self.create_subscription(
            Odometry,
            '/odom_raw',
            self.odom_callback,
            10
        )

        self.pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        self.get_logger().info('✅ Odom Deadband Filter started')
        self.get_logger().info(f'   Subscribing:  /odom_raw')
        self.get_logger().info(f'   Publishing:   /odom')
        self.get_logger().info(f'   Linear deadband:  {self.LINEAR_DEADBAND} m/s')
        self.get_logger().info(f'   Angular deadband: {self.ANGULAR_DEADBAND} rad/s')

    def odom_callback(self, msg: Odometry):

        # ── Ground robot constraints ──────────────────────────────────────
        # Husky cannot fly, roll, or pitch — always zero these out
        msg.twist.twist.linear.z  = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0

        # ── Linear X deadband ─────────────────────────────────────────────
        if abs(msg.twist.twist.linear.x) < self.LINEAR_DEADBAND:
            msg.twist.twist.linear.x = 0.0
            msg.twist.twist.linear.y = 0.0  # y should always be 0 for diff drive

        # ── Angular Z deadband ────────────────────────────────────────────
        # This is the main culprit — phantom 0.37 rad/s yaw from physics noise
        if abs(msg.twist.twist.angular.z) < self.ANGULAR_DEADBAND:
            msg.twist.twist.angular.z = 0.0

        # ── Publish clean odom ────────────────────────────────────────────
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomDeadband()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down odom_deadband node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
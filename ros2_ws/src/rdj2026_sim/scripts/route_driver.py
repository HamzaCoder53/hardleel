#!/usr/bin/env python3
"""Drive a fixed waypoint route using the simulator's ground-truth pose (/ground_truth).

Steering uses the TRUE pose, so the localization method under test cannot change where the robot goes -
every method sees exactly the same drive. Publishes /cmd_vel, logs progress, exits when the route is done.

Parameter `route`: flat list [x1, y1, x2, y2, ...] in world metres.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class RouteDriver(Node):
    def __init__(self):
        super().__init__('route_driver')
        flat = list(self.declare_parameter('route', [0.0, 0.0]).value)
        self.route = list(zip(flat[0::2], flat[1::2]))
        self.v = float(self.declare_parameter('speed', 0.25).value)
        self.tol = float(self.declare_parameter('tolerance', 0.08).value)
        self.i = 0
        self.pose = None
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'ground_truth', self.on_pose, 10)
        self.create_timer(0.05, self.step)
        self.done = False

    def on_pose(self, m):
        q = m.pose.pose.orientation
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y,
                     math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def step(self):
        if self.pose is None or self.done:
            return
        x, y, th = self.pose
        gx, gy = self.route[self.i]
        dx, dy = gx - x, gy - y
        dist = math.hypot(dx, dy)
        if dist < self.tol:
            self.get_logger().info(f'waypoint {self.i + 1}/{len(self.route)} reached ({gx:.2f}, {gy:.2f})')
            self.i += 1
            if self.i >= len(self.route):
                self.pub.publish(Twist())
                self.get_logger().info('ROUTE DONE')
                self.done = True
            return
        err = (math.atan2(dy, dx) - th + math.pi) % (2 * math.pi) - math.pi
        cmd = Twist()
        cmd.angular.z = max(-1.5, min(1.5, 2.5 * err))
        cmd.linear.x = 0.0 if abs(err) > 0.5 else self.v * min(1.0, dist / 0.3) * (1 - abs(err) / 0.5 * 0.7)
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = RouteDriver()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

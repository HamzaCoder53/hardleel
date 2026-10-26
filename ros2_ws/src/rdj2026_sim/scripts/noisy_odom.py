#!/usr/bin/env python3
"""Drifting odometry, to mimic hardleel without wheel encoders.

Integrates the simulator's /odom twist with a heading drift (gyro bias), a distance scale error and
random noise, and publishes the result as the odom->base_link TF and /odom_noisy.
Use with sim.launch.py noisy_odom:=true (which stops the perfect diff-drive TF from being bridged).

Parameters: drift_deg_per_min (gyro bias), scale (distance factor), noise_lin (m/s), noise_ang (rad/s), seed
"""
import math
import random

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class NoisyOdom(Node):
    def __init__(self):
        super().__init__('noisy_odom')
        self.drift = math.radians(float(self.declare_parameter('drift_deg_per_min', 4.0).value)) / 60.0
        self.scale = float(self.declare_parameter('scale', 1.05).value)
        self.noise_lin = float(self.declare_parameter('noise_lin', 0.02).value)
        self.noise_ang = float(self.declare_parameter('noise_ang', 0.03).value)
        random.seed(int(self.declare_parameter('seed', 1).value))
        self.x = self.y = self.th = 0.0
        self.last = None
        self.tf = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, 'odom_noisy', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 20)
        self.get_logger().info(f'drift {math.degrees(self.drift) * 60:.1f} deg/min, scale {self.scale}')

    def on_odom(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.last is None:
            self.last = t
            return
        dt = t - self.last
        self.last = t
        if dt <= 0 or dt > 0.5:
            return
        v = m.twist.twist.linear.x * self.scale + random.gauss(0, self.noise_lin) * (m.twist.twist.linear.x != 0)
        w = m.twist.twist.angular.z + self.drift + random.gauss(0, self.noise_ang) * (abs(m.twist.twist.angular.z) > 1e-3)
        self.th += w * dt
        self.x += v * math.cos(self.th) * dt
        self.y += v * math.sin(self.th) * dt

        ts = TransformStamped()
        ts.header.stamp = m.header.stamp
        ts.header.frame_id, ts.child_frame_id = 'odom', 'base_link'
        ts.transform.translation.x, ts.transform.translation.y = self.x, self.y
        ts.transform.rotation.z, ts.transform.rotation.w = math.sin(self.th / 2), math.cos(self.th / 2)
        self.tf.sendTransform(ts)
        o = Odometry()
        o.header = ts.header
        o.child_frame_id = 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation = ts.transform.rotation
        o.twist = m.twist
        self.pub.publish(o)


def main():
    rclpy.init()
    node = NoisyOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

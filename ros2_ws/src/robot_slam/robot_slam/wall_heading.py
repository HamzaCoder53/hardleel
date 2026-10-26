#!/usr/bin/env python3
"""wall_heading: absolute heading from the square course walls (all walls at 0/90 deg in the map frame).

Per scan the dominant wall angle a (robot frame, modulo 90 deg; loc_core.dominant_wall_angle) gives
yaw = -a + k*90 deg; the multiple k comes from the current yaw estimate (/odom from the EKF, which integrates the
gyro). Published as /wall_heading/imu (sensor_msgs/Imu, orientation = yaw only, other covariances -1) for the EKF
(config/ekf_nav_v2.yaml imu1). Only published when the robot is level (/imu/data within tilt_deg of boot), turning
slowly (|gyro z| < max_yaw_rate_dps), the walls are clear (strength >= min_strength, >= min_segments straight
segments) and the result is within max_jump_deg of the estimate (else counted as a reject; clutter / a worker).
/wall_heading/state (String JSON) once a second: published / rejected counts and the last values.
"""
import json
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String

from robot_slam import loc_core as lc
from robot_slam.scan_gate import rp_from_quat


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class WallHeading(Node):
    def __init__(self):
        super().__init__('wall_heading')
        p = lambda n, v: self.declare_parameter(n, v).value
        self.tilt = math.radians(float(p('tilt_deg', 3.0)))
        self.max_rate = math.radians(float(p('max_yaw_rate_dps', 20.0)))
        self.min_strength = float(p('min_strength', 0.8))
        self.min_segments = int(p('min_segments', 4))
        self.max_jump = math.radians(float(p('max_jump_deg', 10.0)))
        self.sigma = math.radians(float(p('sigma_deg', 1.5)))
        self.rmax = float(p('max_range_m', 5.0))
        self.yaw_ref = None
        self.rate = 0.0
        self.boot, self.rp0, self.rp = [], None, (0.0, 0.0)
        self.counts = {'published': 0, 'rejected': 0, 'skipped': 0}
        self.last = {}
        self.pub = self.create_publisher(Imu, '/wall_heading/imu', 10)
        self.state_pub = self.create_publisher(String, '/wall_heading/state', 10)
        self.create_subscription(Odometry, str(p('odom_topic', '/odom')), self.on_odom, 10)
        self.create_subscription(Imu, '/imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu/data_raw', lambda m: setattr(self, 'rate', m.angular_velocity.z),
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, str(p('scan_topic', '/scan_filtered')), self.on_scan, qos_profile_sensor_data)
        self.create_timer(1.0, self.report)

    def on_odom(self, m):
        self.yaw_ref = yaw_of(m.pose.pose.orientation)

    def on_imu(self, m):
        if m.orientation_covariance[0] < 0:
            return
        r = rp_from_quat(m.orientation)
        if self.rp0 is None:
            self.boot.append(r)
            if len(self.boot) >= 50:
                self.rp0 = tuple(sum(v[i] for v in self.boot) / len(self.boot) for i in range(2))
            return
        self.rp = (r[0] - self.rp0[0], r[1] - self.rp0[1])

    def on_scan(self, m):
        if self.yaw_ref is None or self.rp0 is None or max(abs(self.rp[0]), abs(self.rp[1])) > self.tilt \
                or abs(self.rate) > self.max_rate:
            self.counts['skipped'] += 1
            return
        pts = lc.scan_points(m.ranges, m.angle_min, m.angle_increment, 0.15, self.rmax)
        a, strength, nseg = lc.dominant_wall_angle(pts)
        if a is None or strength < self.min_strength or nseg < self.min_segments:
            self.counts['skipped'] += 1
            return
        yaw = lc.heading_from_walls(a, self.yaw_ref)
        jump = lc.wrap(yaw - self.yaw_ref)
        self.last = {'yaw_deg': round(math.degrees(yaw), 2), 'jump_deg': round(math.degrees(jump), 2),
                     'strength': round(strength, 3), 'segments': nseg}
        if abs(jump) > self.max_jump:
            self.counts['rejected'] += 1
            return
        out = Imu()
        out.header.stamp = m.header.stamp
        out.header.frame_id = 'base_link'
        out.orientation.z, out.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        var = (self.sigma / max(strength, 0.3)) ** 2
        out.orientation_covariance = [1e6, 0.0, 0.0, 0.0, 1e6, 0.0, 0.0, 0.0, var]
        out.angular_velocity_covariance[0] = -1.0
        out.linear_acceleration_covariance[0] = -1.0
        self.pub.publish(out)
        self.counts['published'] += 1

    def report(self):
        self.state_pub.publish(String(data=json.dumps({**self.counts, **self.last})))


def main():
    rclpy.init()
    n = WallHeading()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

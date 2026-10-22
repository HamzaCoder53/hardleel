"""Synthetic /scan + /imu/data_raw + /imu/data for bench-testing brain.launch.py without the robot.

The robot sits still at (x0, y0) inside a 4 x 3 m walled room with a pillar, so Cartographer gets a
recognisable scan. If /cmd_vel arrives the fake robot integrates it (no physics), so Nav2 can be seen
"driving". Run on a private domain, never 7:
    ROS_DOMAIN_ID=42 python3 fake_sensors.py
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan

WALLS = [((-1.0, -1.5), (3.0, -1.5)), ((3.0, -1.5), (3.0, 1.5)), ((3.0, 1.5), (-1.0, 1.5)),
         ((-1.0, 1.5), (-1.0, -1.5)),
         # pillar and a short wall so the room is not symmetric
         ((1.2, 0.3), (1.5, 0.3)), ((1.5, 0.3), (1.5, 0.6)), ((1.5, 0.6), (1.2, 0.6)), ((1.2, 0.6), (1.2, 0.3)),
         ((2.2, -1.5), (2.2, -0.7))]


def ray(px, py, a, rmax=12.0):
    dx, dy = math.cos(a), math.sin(a)
    best = rmax
    for (x1, y1), (x2, y2) in WALLS:
        ex, ey = x2 - x1, y2 - y1
        den = dx * ey - dy * ex
        if abs(den) < 1e-9:
            continue
        t = ((x1 - px) * ey - (y1 - py) * ex) / den
        u = ((x1 - px) * dy - (y1 - py) * dx) / den
        if t > 0 and 0 <= u <= 1:
            best = min(best, t)
    return best


class Fake(Node):
    def __init__(self):
        super().__init__('fake_sensors')
        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.v = self.w = 0.0
        self.n = int(self.declare_parameter('points', 720).value)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.raw_pub = self.create_publisher(Imu, '/imu/data_raw', qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_timer(1 / 5.5, self.scan)
        self.create_timer(0.01, self.imu)

    def on_cmd(self, m):
        self.v, self.w = m.linear.x, m.angular.z

    def imu(self):
        self.x += self.v * 0.01 * math.cos(self.th)
        self.y += self.v * 0.01 * math.sin(self.th)
        self.th += self.w * 0.01
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'imu_link'
        m.linear_acceleration.z = 9.81
        m.angular_velocity.z = self.w
        m.orientation.z, m.orientation.w = math.sin(self.th / 2), math.cos(self.th / 2)
        self.raw_pub.publish(m)
        self.imu_pub.publish(m)

    def scan(self):
        m = LaserScan()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'laser'
        m.angle_min, m.angle_max = -math.pi, math.pi - 2 * math.pi / self.n
        m.angle_increment = 2 * math.pi / self.n
        m.scan_time, m.time_increment = 1 / 5.5, 1 / 5.5 / self.n
        m.range_min, m.range_max = 0.15, 12.0
        m.ranges = [ray(self.x, self.y, self.th + m.angle_min + i * m.angle_increment) for i in range(self.n)]
        self.scan_pub.publish(m)


def main():
    rclpy.init()
    rclpy.spin(Fake())


if __name__ == '__main__':
    main()

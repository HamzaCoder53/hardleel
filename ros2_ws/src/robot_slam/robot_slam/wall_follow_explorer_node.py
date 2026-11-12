#!/usr/bin/env python3
"""Left-wall-following explorer for the closed mapping arena.

Publishes /cmd_vel_raw (the guardian turns that into a safe /cmd_vel: bumper,
ramp slow-down, gyro damping). Stops and exits when the map stops growing, or
after max_time_s, so a launch script can save the map afterwards.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid


def sector_min(scan, lo_deg, hi_deg):
    """Smallest valid range between lo_deg and hi_deg (0 = straight ahead, + = left)."""
    best = float('inf')
    for i, r in enumerate(scan.ranges):
        if math.isnan(r) or math.isinf(r) or r < 0.15:
            continue
        a = math.degrees(math.atan2(math.sin(scan.angle_min + i * scan.angle_increment),
                                    math.cos(scan.angle_min + i * scan.angle_increment)))
        if lo_deg <= a <= hi_deg:
            best = min(best, r)
    return best


class WallFollowExplorer(Node):
    def __init__(self):
        super().__init__('wall_follow_explorer')
        self.declare_parameter('scan_topic', '/scan_filtered')
        self.declare_parameter('wall_dist_m', 0.45)    # desired distance to the left wall
        self.declare_parameter('front_stop_m', 0.50)   # wall ahead closer than this -> turn right
        self.declare_parameter('lost_wall_m', 1.20)    # left wall further than this -> arc left to find it
        self.declare_parameter('speed', 0.15)
        self.declare_parameter('turn_speed', 0.35)
        self.declare_parameter('kp', 1.4)
        self.declare_parameter('min_time_s', 90.0)     # never finish before this
        self.declare_parameter('max_time_s', 420.0)    # hard stop
        self.declare_parameter('stall_window_s', 40.0) # finish if the map grew < stall_growth over this
        self.declare_parameter('stall_growth', 0.01)
        g = lambda n: self.get_parameter(n).value
        self.d = g('wall_dist_m'); self.front_stop = g('front_stop_m')
        self.lost = g('lost_wall_m'); self.v = g('speed'); self.w = g('turn_speed'); self.kp = g('kp')
        self.min_t = g('min_time_s'); self.max_t = g('max_time_s')
        self.win = g('stall_window_s'); self.grow = g('stall_growth')

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(LaserScan, g('scan_topic'), self.on_scan, qos)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, 1)
        self.pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.t0 = time.time()
        self.known_hist = []          # (time, known cell count)
        self.finished = False
        self.last_log = 0.0
        self.get_logger().info('Wall-follow explorer started (left wall).')

    def on_map(self, msg):
        known = sum(1 for c in msg.data if c >= 0)
        self.known_hist.append((time.time(), known))
        self.known_hist = [h for h in self.known_hist if h[0] > time.time() - 2 * self.win]

    def map_stalled(self):
        if len(self.known_hist) < 2 or self.known_hist[0][0] > time.time() - self.win:
            return False
        old = self.known_hist[0][1]
        return old > 0 and (self.known_hist[-1][1] - old) / old < self.grow

    def stop_and_finish(self, why):
        self.get_logger().info(f'Exploration finished: {why}')
        self.pub.publish(Twist())
        self.finished = True

    def on_scan(self, scan):
        if self.finished:
            return
        t = time.time() - self.t0
        if t > self.max_t:
            return self.stop_and_finish('time limit')
        if t > self.min_t and self.map_stalled():
            return self.stop_and_finish('map stopped growing')

        front = sector_min(scan, -25, 25)
        left = sector_min(scan, 60, 120)
        front_left = sector_min(scan, 25, 60)
        cmd = Twist()
        if front < self.front_stop:                      # wall or corner ahead: turn right in place
            cmd.angular.z = -self.w
        elif min(left, front_left) > self.lost:          # no wall on the left: arc left to find one
            cmd.linear.x = self.v * 0.7
            cmd.angular.z = self.w * 0.8
        else:                                            # follow the wall at wall_dist_m
            side = min(left, front_left * 0.8)           # front-left sees convex corners early
            err = side - self.d
            cmd.linear.x = self.v
            cmd.angular.z = max(-self.w, min(self.w, self.kp * err))
        self.pub.publish(cmd)
        if time.time() - self.last_log > 3.0:
            self.last_log = time.time()
            self.get_logger().info(f'front={front:.2f} fl={front_left:.2f} left={left:.2f} -> v={cmd.linear.x:.2f} w={cmd.angular.z:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowExplorer()
    while rclpy.ok() and not node.finished:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

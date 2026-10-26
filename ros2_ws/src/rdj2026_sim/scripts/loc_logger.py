#!/usr/bin/env python3
"""Log localization error: the map->base_link estimate vs the simulator's ground truth.

The map frame of a run is assumed to start at the robot's start pose (true for Cartographer, and for
AMCL when its initial pose is the map origin). Ground truth is re-expressed relative to the first pose.
Writes CSV (t, err_xy_m, err_yaw_deg, est_x, est_y, true_x, true_y) to `csv` and prints a summary on exit.
"""
import csv
import math
import statistics

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class LocLogger(Node):
    def __init__(self):
        super().__init__('loc_logger')
        self.path = self.declare_parameter('csv', '/tmp/loc.csv').value
        # world pose where the map frame starts (the start pad); default = first ground-truth pose seen
        o = list(self.declare_parameter('origin', [0.0, 0.0, 0.0, 0.0]).value)   # [x, y, yaw_deg, use(1/0)]
        self.buf = Buffer()
        TransformListener(self.buf, self)
        self.origin = (o[0], o[1], math.radians(o[2])) if len(o) > 3 and o[3] else None
        self.truth = None
        self.rows = []
        self.create_subscription(Odometry, 'ground_truth', self.on_truth, 10)
        self.create_timer(0.5, self.sample)

    def on_truth(self, m):
        p = (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))
        if self.origin is None:
            self.origin = p
        ox, oy, oth = self.origin
        dx, dy = p[0] - ox, p[1] - oy
        c, s = math.cos(-oth), math.sin(-oth)
        self.truth = (c * dx - s * dy, s * dx + c * dy, p[2] - oth)

    def sample(self):
        if self.truth is None:
            return
        try:
            t = self.buf.lookup_transform('map', 'base_link', Time())
        except Exception:
            return
        ex, ey = t.transform.translation.x, t.transform.translation.y
        eth = yaw_of(t.transform.rotation)
        tx, ty, tth = self.truth
        e_xy = math.hypot(ex - tx, ey - ty)
        e_th = abs((math.degrees(eth - tth) + 180) % 360 - 180)
        now = self.get_clock().now().nanoseconds * 1e-9
        self.rows.append((now, e_xy, e_th, ex, ey, tx, ty))

    def summary(self):
        with open(self.path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'err_xy_m', 'err_yaw_deg', 'est_x', 'est_y', 'true_x', 'true_y'])
            w.writerows(self.rows)
        if not self.rows:
            print('no samples')
            return
        exy = [r[1] for r in self.rows]
        eth = [r[2] for r in self.rows]
        print(f'samples {len(exy)} | position error: mean {statistics.mean(exy):.3f} m, '
              f'median {statistics.median(exy):.3f} m, max {max(exy):.3f} m | heading error: mean '
              f'{statistics.mean(eth):.1f} deg, max {max(eth):.1f} deg | csv {self.path}')


def main():
    rclpy.init()
    node = LocLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

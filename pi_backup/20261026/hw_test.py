#!/usr/bin/env python3
"""Hardware test for hardleel. Phase 'static' = no motors. Phase 'motion' = short bounded moves.
Usage: hw_test.py static | motion
Needs running: imu_serial_node (/imu/data_raw), sllidar (/scan), motor_bridge (/cmd_vel), optional pi_cam (/camera/image_raw/compressed).
Safety: every move is <= 1.5 s, commands are streamed at 10 Hz (bridge cmd_timeout 0.4 s + Pico watchdog), any lidar return
< STOP_M in the travel direction aborts, zero velocity is always sent at the end (finally).
"""
import json
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Imu, LaserScan

STOP_M = 0.30
OUT = {}


class T(Node):
    def __init__(self):
        super().__init__('hw_test')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan = None
        self.scan_t = []
        self.imu_t = []
        self.gz = []
        self.az = []
        self.ax = []
        self.ay = []
        self.cam_t = []
        self.cam_bytes = []
        self.last_img = None
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Imu, 'imu/data_raw', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, 'camera/image_raw/compressed', self.on_cam,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def on_scan(self, m):
        self.scan = m
        self.scan_t.append(time.time())

    def on_imu(self, m):
        self.imu_t.append(time.time())
        self.gz.append((time.time(), m.angular_velocity.z))
        self.ax.append(m.linear_acceleration.x)
        self.ay.append(m.linear_acceleration.y)
        self.az.append(m.linear_acceleration.z)

    def on_cam(self, m):
        self.cam_t.append(time.time())
        self.cam_bytes.append(len(m.data))
        self.last_img = bytes(m.data)

    def spin_for(self, s):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def send(self, lin, ang, s):
        """Stream a velocity for s seconds; abort on a close lidar return."""
        end = time.time() + s
        aborted = None
        while time.time() < end:
            if self.scan is not None and lin != 0.0:
                d = self.sector_min(0.0 if lin > 0 else math.pi)
                if d is not None and d < STOP_M:
                    aborted = f'obstacle {d:.2f} m'
                    break
            t = Twist()
            t.linear.x = float(lin)
            t.angular.z = float(ang)
            self.pub.publish(t)
            self.spin_for(0.1)
        self.stop()
        return aborted

    def stop(self):
        for _ in range(3):
            self.pub.publish(Twist())
            self.spin_for(0.05)

    def sector_min(self, centre, half=math.radians(25)):
        """Smallest valid range within +-half of angle `centre` (laser frame)."""
        m = self.scan
        r = np.array(m.ranges)
        a = m.angle_min + np.arange(len(r)) * m.angle_increment
        d = np.abs(np.arctan2(np.sin(a - centre), np.cos(a - centre)))
        ok = (d < half) & np.isfinite(r) & (r > 0.12)
        return float(r[ok].min()) if ok.any() else None

    def profile(self):
        """Median range in 10-degree sectors, laser frame, for locating the direction of travel."""
        m = self.scan
        r = np.array(m.ranges)
        a = np.degrees(m.angle_min + np.arange(len(r)) * m.angle_increment) % 360
        out = {}
        for s in range(0, 360, 10):
            sel = (a >= s) & (a < s + 10) & np.isfinite(r) & (r > 0.12)
            out[s] = float(np.median(r[sel])) if sel.any() else float('nan')
        return out

    def yaw_deg_between(self, t0, t1, bias):
        tot = 0.0
        pts = [(t, g) for t, g in self.gz if t0 <= t <= t1]
        for (ta, ga), (tb, gb) in zip(pts, pts[1:]):
            tot += (0.5 * (ga + gb) - bias) * (tb - ta)
        return math.degrees(tot)


def rate(ts):
    return (len(ts) - 1) / (ts[-1] - ts[0]) if len(ts) > 2 else 0.0


def static(n):
    n.spin_for(15.0)
    OUT['scan_hz'] = round(rate(n.scan_t), 2)
    OUT['imu_hz'] = round(rate(n.imu_t), 1)
    g = np.array([v for _, v in n.gz])
    OUT['gyro_z_mean_rad_s'] = round(float(g.mean()), 5) if len(g) else None
    OUT['gyro_z_std_rad_s'] = round(float(g.std()), 5) if len(g) else None
    if n.az:
        a = np.array([n.ax, n.ay, n.az])
        OUT['accel_mean'] = [round(float(x), 3) for x in a.mean(axis=1)]
        OUT['accel_norm'] = round(float(np.linalg.norm(a.mean(axis=1))), 3)
        OUT['tilt_deg_from_az'] = round(math.degrees(math.acos(min(1, abs(a.mean(axis=1)[2]) / np.linalg.norm(a.mean(axis=1))))), 2)
    if n.scan is not None:
        r = np.array(n.scan.ranges)
        OUT['scan_points'] = len(r)
        OUT['scan_valid_pct'] = round(100 * float(np.isfinite(r).mean()), 1)
        OUT['scan_min_max'] = [round(float(np.nanmin(r[np.isfinite(r)])), 2), round(float(np.nanmax(r[np.isfinite(r)])), 2)]
        OUT['range_profile_10deg'] = {k: round(v, 2) for k, v in n.profile().items()}
    OUT['cam_hz'] = round(rate(n.cam_t), 1)
    OUT['cam_kB_per_frame'] = round(float(np.mean(n.cam_bytes)) / 1000, 1) if n.cam_bytes else None
    if n.last_img:
        open('/tmp/hw_cam.jpg', 'wb').write(n.last_img)
        OUT['cam_frame'] = '/tmp/hw_cam.jpg'


def motion(n):
    n.spin_for(3.0)
    gz = np.array([v for _, v in n.gz])
    bias = float(gz.mean())
    OUT['gyro_bias_rad_s'] = round(bias, 5)
    OUT['moves'] = []
    plan = []
    for v in (0.25, 0.35, 0.5):
        plan += [('fwd', v, 0.0), ('back', -v, 0.0)]
    plan += [('spin_left', 0.0, 0.8), ('spin_right', 0.0, -0.8), ('spin_left', 0.0, 1.2), ('spin_right', 0.0, -1.2)]
    for name, lin, ang in plan:
        n.stop()
        n.spin_for(1.0)
        before = n.profile()
        t0 = time.time()
        ab = n.send(lin, ang, 1.2)
        t1 = time.time()
        n.spin_for(0.8)
        after = n.profile()
        yaw = n.yaw_deg_between(t0, t1 + 0.4, bias)
        # sector whose range changed most (the direction we moved in the laser frame)
        diffs = {k: after[k] - before[k] for k in before if not (math.isnan(before[k]) or math.isnan(after[k]))}
        fwd = min(diffs, key=diffs.get) if diffs else None
        back = max(diffs, key=diffs.get) if diffs else None
        OUT['moves'].append({
            'name': name, 'cmd_lin': lin, 'cmd_ang': ang, 'secs': 1.2, 'aborted': ab,
            'yaw_deg': round(yaw, 1),
            'range_change_m_at_sector': {'closest_sector_deg': fwd, 'delta': round(diffs[fwd], 3) if fwd is not None else None,
                                         'farthest_sector_deg': back, 'delta2': round(diffs[back], 3) if back is not None else None},
        })
        if ab:
            break
    n.stop()


def main():
    rclpy.init()
    n = T()
    try:
        n.spin_for(2.0)
        if sys.argv[1] == 'static':
            static(n)
        elif n.scan is None:
            OUT['error'] = 'no /scan - motion test needs the lidar'
        else:
            motion(n)
    finally:
        n.stop()
        print('RESULT ' + json.dumps(OUT))
        rclpy.shutdown()


main()

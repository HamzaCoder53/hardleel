#!/usr/bin/env python3
"""scan_gate: the scans Cartographer may use. /scan_filtered -> /scan_mapping, BLOCKED while
  - the robot is inside a ramp zone (course_geometry.yaml ramp_zones, pose from TF map -> base_link),
  - |pitch| or |roll| (relative to the boot pose; tilt_source 'accel' = low-passed /imu/data_raw gravity vector,
    'orientation' = /imu/data Madgwick quaternion) exceeds tilt_deg (3),
  - |yaw rate| (/imu/data_raw gyro z) exceeds max_yaw_rate_dps (turning: motion-blurred scans).
It opens again only after settle_s (0.5) of level and steady. Nav2, the collision monitor and the ramp dash keep
the raw scans. /scan_gate/state (String): 'open' or 'blocked:<reason>' on every change (logged).
Params: in_topic, out_topic, tilt_deg, max_yaw_rate_dps, settle_s, geometry_file, use_ramp_zones, boot_samples.
"""
import math
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


def rp_from_quat(q):
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
    return roll, pitch


def rp_from_accel(ax, ay, az):
    """Roll/pitch (rad, REP-103 like rp_from_quat) from the gravity vector of a still or slowly moving IMU."""
    return math.atan2(ay, az), math.atan2(-ax, math.sqrt(ay * ay + az * az))


class GateLogic:
    """Pure decision logic (unit-tested): feed it conditions, ask open(t)."""

    def __init__(self, settle_s):
        self.settle_s = settle_s
        self.bad_reason = None
        self.good_since = None

    def update(self, t, reason):
        """reason: None when everything is fine, else why the gate must be closed. -> (open, reason)."""
        if reason:
            self.bad_reason = reason
            self.good_since = None
            return False, reason
        if self.good_since is None:
            self.good_since = t
        if t - self.good_since >= self.settle_s:
            self.bad_reason = None
            return True, None
        return False, f'settling ({self.bad_reason or "start"})'


def in_zone(x, y, zones):
    for z in zones:
        if z['x_min'] <= x <= z['x_max'] and z['y_min'] <= y <= z['y_max']:
            return z.get('name', 'ramp')
    return None


class ScanGate(Node):
    def __init__(self):
        super().__init__('scan_gate')
        p = lambda n, v: self.declare_parameter(n, v).value
        self.tilt = math.radians(float(p('tilt_deg', 3.0)))
        self.max_rate = math.radians(float(p('max_yaw_rate_dps', 25.0)))
        geo = str(p('geometry_file', ''))
        if not geo:
            geo = os.path.join(get_package_share_directory('robot_slam'), 'config', 'course_geometry.yaml')
        self.zones = yaml.safe_load(open(geo)).get('ramp_zones', []) if bool(p('use_ramp_zones', True)) else []
        self.boot_n = int(p('boot_samples', 50))
        # 'accel': tilt from the low-passed /imu/data_raw gravity vector (no Madgwick; team 26 Oct: its orientation
        # drifted). 'orientation': the old /imu/data (Madgwick) quaternion.
        self.tilt_source = str(p('tilt_source', 'accel'))
        self.acc_tau = float(p('accel_tau_s', 0.3))   # low-pass time constant: ignores vibration and short bumps
        self.acc_lp = None
        self.acc_t = None
        self.logic = GateLogic(float(p('settle_s', 0.5)))
        self.boot, self.rp0 = [], None
        self.roll = self.pitch = 0.0
        self.yaw_rate = 0.0
        self.state = None
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.pub = self.create_publisher(LaserScan, str(p('out_topic', '/scan_mapping')), qos_profile_sensor_data)
        self.state_pub = self.create_publisher(String, '/scan_gate/state', 10)
        if self.tilt_source == 'orientation':
            self.create_subscription(Imu, '/imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu/data_raw', self.on_raw, qos_profile_sensor_data)
        self.create_subscription(LaserScan, str(p('in_topic', '/scan_filtered')), self.on_scan, qos_profile_sensor_data)
        self.n_pass = self.n_block = 0
        self.create_timer(10.0, self.report)

    def on_imu(self, m):
        if m.orientation_covariance[0] < 0:
            return
        self.feed_tilt(*rp_from_quat(m.orientation))

    def feed_tilt(self, r, pch):
        if self.rp0 is None:
            self.boot.append((r, pch))
            if len(self.boot) >= self.boot_n:
                self.rp0 = (sum(b[0] for b in self.boot) / len(self.boot), sum(b[1] for b in self.boot) / len(self.boot))
            return
        self.roll, self.pitch = r - self.rp0[0], pch - self.rp0[1]

    def on_raw(self, m):
        self.yaw_rate = m.angular_velocity.z
        if self.tilt_source == 'orientation':
            return
        a = m.linear_acceleration
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.acc_lp is None:
            self.acc_lp = [a.x, a.y, a.z]
        else:
            k = min(1.0, max(0.0, t - self.acc_t) / self.acc_tau) if self.acc_tau > 0 else 1.0
            self.acc_lp = [lp + k * (v - lp) for lp, v in zip(self.acc_lp, (a.x, a.y, a.z))]
        self.acc_t = t
        self.feed_tilt(*rp_from_accel(*self.acc_lp))

    def reason(self):
        if self.rp0 is None:
            return 'imu boot'
        if abs(self.pitch) > self.tilt or abs(self.roll) > self.tilt:
            return f'tilt {math.degrees(self.pitch):+.1f}/{math.degrees(self.roll):+.1f} deg'
        if abs(self.yaw_rate) > self.max_rate:
            return f'turning {math.degrees(self.yaw_rate):+.0f} deg/s'
        if self.zones:
            try:
                t = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time())
                z = in_zone(t.transform.translation.x, t.transform.translation.y, self.zones)
                if z:
                    return f'zone {z}'
            except TransformException:
                pass                                   # no map yet: zones cannot apply
        return None

    def on_scan(self, m):
        now = self.get_clock().now().nanoseconds * 1e-9
        ok, why = self.logic.update(now, self.reason())
        state = 'open' if ok else 'blocked:' + why.split(' ')[0]      # coarse state: log on changes only
        if state != self.state:
            self.state = state
            self.state_pub.publish(String(data=state if ok else f'blocked:{why}'))
            self.get_logger().info(f'scan gate {state} ({why or "level and steady"})')
        if ok:
            self.pub.publish(m)
            self.n_pass += 1
        else:
            self.n_block += 1

    def report(self):
        self.get_logger().info(f'scan gate: {self.n_pass} passed, {self.n_block} blocked (last 10 s)',
                               throttle_duration_sec=30.0)
        self.n_pass = self.n_block = 0


def main():
    rclpy.init()
    n = ScanGate()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

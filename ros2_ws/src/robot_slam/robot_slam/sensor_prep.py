#!/usr/bin/env python3
"""Sensor preparation: tilt-aware LiDAR gating and a bias-free, low-passed IMU.

/scan (+ /imu/data)         -> /scan_filtered  per-beam ground / overhead gate while on a ramp
/imu/data_raw (+ /imu/data) -> /imu/data_lpf   gyro bias removed at boot, low-passed, Madgwick orientation

Ramp mode: relative tilt (vs. the boot pose) above arm_deg turns it on, below release_deg turns it off.
On flat ground the scan is passed through unchanged. Dropped beams become NaN (ignored by Cartographer
and Nav2), never inf. Acceleration is only low-passed, never integrated.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan, Imu


def euler_from_quaternion(q):
    """(roll, pitch, yaw) in radians, REP-103 (pitch > 0 = nose down)."""
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


def gate_ranges(ranges, angles, pitch_rel, roll_rel, lidar_h, robot_h, margin, rmin, rmax):
    """Return a copy of ranges with ground hits and overhead hits set to NaN.

    pitch_rel / roll_rel are radians relative to the boot pose, REP-103 signs. For a beam at azimuth
    alpha the world-frame slope is dz = -sin(pitch)*cos(alpha) + sin(roll)*sin(alpha); a return at range r
    is then at height lidar_h + r*dz. Below `margin` it is the ground, above `robot_h` it is overhead.
    """
    r = np.asarray(ranges, dtype=np.float64)
    dz = -math.sin(pitch_rel) * np.cos(angles) + math.sin(roll_rel) * np.sin(angles)
    with np.errstate(invalid="ignore"):
        z = lidar_h + r * dz
    with np.errstate(invalid='ignore'):
        bad = ~np.isfinite(r) | (r < rmin) | (r > rmax) | (z < margin) | (z > robot_h)
    out = r.copy()
    out[bad] = np.nan
    return out.astype(np.float32)


class Hysteresis:
    def __init__(self, arm, release):
        self.arm, self.release, self.on = arm, release, False

    def update(self, value):
        if self.on and value < self.release:
            self.on = False
        elif not self.on and value > self.arm:
            self.on = True
        return self.on


class SensorPrep(Node):
    def __init__(self):
        super().__init__('sensor_prep')
        p = self.declare_parameter
        p('scan_in', '/scan'); p('scan_out', '/scan_filtered')
        p('imu_raw', '/imu/data_raw'); p('imu_fused', '/imu/data'); p('imu_out', '/imu/data_lpf')
        p('lidar_height', 0.15)        # m, laser above the ground (base_to_laser z)
        p('robot_height', 0.35)        # m, returns above this are overhead / sky
        p('ground_margin', 0.03)       # m, returns below this are ground
        p('arm_deg', 2.0); p('release_deg', 1.0)
        p('range_min', 0.18); p('range_max', 5.5)
        p('boot_samples', 200)         # still IMU samples used for the gyro bias and the boot pitch/roll
        p('boot_max_gyro_std', 0.05)   # rad/s; a noisier window means the robot moved, so it restarts
        p('lpf_alpha', 0.2)            # 0..1, higher = less smoothing (0.2 at 100 Hz = about 4 Hz cutoff)
        g = lambda n: self.get_parameter(n).value
        self.lidar_h = float(g('lidar_height')); self.robot_h = float(g('robot_height'))
        self.margin = float(g('ground_margin'))
        self.rmin = float(g('range_min')); self.rmax = float(g('range_max'))
        self.boot_n = int(g('boot_samples')); self.boot_std = float(g('boot_max_gyro_std'))
        self.alpha = float(g('lpf_alpha'))
        self.hyst = Hysteresis(math.radians(float(g('arm_deg'))), math.radians(float(g('release_deg'))))

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        self.pitch = self.roll = 0.0
        self.have_orientation = False
        self.fused = None
        self.boot_gyro, self.boot_tilt = [], []
        self.bias = None
        self.pitch0 = self.roll0 = 0.0
        self.lp_gyro = self.lp_acc = None
        self.ramp_mode = False

        self.create_subscription(Imu, g('imu_fused'), self.on_fused, qos)
        self.create_subscription(Imu, g('imu_raw'), self.on_raw, qos)
        self.create_subscription(LaserScan, g('scan_in'), self.on_scan, qos)
        self.pub_scan = self.create_publisher(LaserScan, g('scan_out'), qos)
        self.pub_imu = self.create_publisher(Imu, g('imu_out'), qos)
        self.get_logger().info('sensor_prep waiting for a still IMU to capture the gyro bias.')

    def on_fused(self, msg):
        self.fused = msg
        self.roll, self.pitch, _ = euler_from_quaternion(msg.orientation)
        self.have_orientation = True

    def on_raw(self, msg):
        w = msg.angular_velocity
        gyro = np.array([w.x, w.y, w.z])
        acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        if self.bias is None:
            self.boot_gyro.append(gyro)
            if self.have_orientation:
                self.boot_tilt.append((self.pitch, self.roll))
            if len(self.boot_gyro) >= self.boot_n:
                arr = np.array(self.boot_gyro)
                if arr.std(axis=0).max() > self.boot_std or not self.boot_tilt:
                    self.get_logger().warn('IMU not still (or no orientation yet) during bias capture; retrying.',
                                           throttle_duration_sec=5.0)
                    self.boot_gyro, self.boot_tilt = [], []
                    return
                self.bias = arr.mean(axis=0)
                self.pitch0, self.roll0 = np.mean(self.boot_tilt, axis=0)
                self.lp_gyro = np.zeros(3)
                self.lp_acc = acc.copy()
                self.get_logger().info(
                    f'Gyro bias captured: {np.degrees(self.bias).round(2)} deg/s; boot pitch '
                    f'{math.degrees(self.pitch0):.2f} deg, roll {math.degrees(self.roll0):.2f} deg.')
            return
        a = self.alpha
        self.lp_gyro = (1 - a) * self.lp_gyro + a * (gyro - self.bias)
        self.lp_acc = (1 - a) * self.lp_acc + a * acc
        out = Imu()
        out.header = msg.header
        if self.fused is not None:
            out.orientation = self.fused.orientation
            out.orientation_covariance = self.fused.orientation_covariance
        else:
            out.orientation_covariance[0] = -1.0      # orientation not available
        out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = map(float, self.lp_gyro)
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration.x, out.linear_acceleration.y, out.linear_acceleration.z = map(float, self.lp_acc)
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self.pub_imu.publish(out)

    def on_scan(self, scan):
        if self.bias is None:                         # no reference pose yet: pass the scan through
            self.pub_scan.publish(scan)
            return
        pr, rr = self.pitch - self.pitch0, self.roll - self.roll0
        was = self.ramp_mode
        self.ramp_mode = self.hyst.update(max(abs(pr), abs(rr)))
        if self.ramp_mode != was:
            self.get_logger().info(f'Ramp mode {"ON" if self.ramp_mode else "OFF"} '
                                   f'(pitch {math.degrees(pr):.1f} deg, roll {math.degrees(rr):.1f} deg).')
        if not self.ramp_mode:
            self.pub_scan.publish(scan)               # flat ground: unchanged
            return
        angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
        out = LaserScan()
        out.header = scan.header
        out.angle_min, out.angle_max = scan.angle_min, scan.angle_max
        out.angle_increment, out.time_increment, out.scan_time = scan.angle_increment, scan.time_increment, scan.scan_time
        out.range_min, out.range_max = float(self.rmin), float(self.rmax)
        out.ranges = gate_ranges(scan.ranges, angles, pr, rr, self.lidar_h, self.robot_h,
                                 self.margin, self.rmin, self.rmax).tolist()
        out.intensities = scan.intensities
        self.pub_scan.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SensorPrep()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()

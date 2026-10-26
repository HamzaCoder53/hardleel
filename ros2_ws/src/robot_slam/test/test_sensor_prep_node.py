import pytest
"""Run sensor_prep as a real ROS node against synthetic IMU and scan data (no robot).

    source /opt/ros/<distro>/setup.bash
    python3 ros2_ws/src/robot_slam/test/test_sensor_prep_node.py
Checks: the -12.4 deg/s gyro bias is captured and removed (/imu/data_lpf z rate about 0), flat scans pass
unchanged, and an 8 degree pitch arms ramp mode and drops far ground hits (with hysteresis on release).
"""
import math
import os
import sys

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, LaserScan

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from robot_slam.sensor_prep import SensorPrep  # noqa: E402

BIAS = math.radians(-12.4)
QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=50)


def fused(pitch_down_deg):
    """Madgwick-style orientation message: rotation about +y by the REP-103 pitch (nose down positive)."""
    m = Imu()
    h = math.radians(pitch_down_deg) / 2.0
    m.orientation.w, m.orientation.y = math.cos(h), math.sin(h)
    m.orientation_covariance[0] = 0.01
    return m


def raw():
    m = Imu()
    m.angular_velocity.z = BIAS
    m.linear_acceleration.z = 9.81
    return m


def scan(r):
    s = LaserScan()
    s.angle_min, s.angle_increment = -math.pi, 2 * math.pi / 360
    s.range_min, s.range_max = 0.15, 12.0
    s.ranges = [float(r)] * 360
    return s


def main():
    rclpy.init()
    node = SensorPrep()
    helper = rclpy.create_node('feeder')
    pub_raw = helper.create_publisher(Imu, '/imu/data_raw', QOS)
    pub_fused = helper.create_publisher(Imu, '/imu/data', QOS)
    pub_scan = helper.create_publisher(LaserScan, '/scan', QOS)
    got = {'lpf': [], 'scan': []}
    helper.create_subscription(Imu, '/imu/data_lpf', lambda m: got['lpf'].append(m), QOS)
    helper.create_subscription(LaserScan, '/scan_filtered', lambda m: got['scan'].append(m), QOS)

    def spin(n=20):
        for _ in range(n):
            rclpy.spin_once(node, timeout_sec=0.005)
            rclpy.spin_once(helper, timeout_sec=0.005)

    for _ in range(40):                                   # let discovery finish
        spin(5)

    def feed(pitch_deg, n_imu, scans=0, r=1.3):
        got['scan'].clear()
        for i in range(n_imu):
            pub_fused.publish(fused(pitch_deg))
            pub_raw.publish(raw())
            spin(4)
        for _ in range(scans):
            pub_scan.publish(scan(r))
            spin(10)

    feed(0.0, 260)                                        # boot: still, flat
    assert node.bias is not None, 'bias was not captured'
    assert abs(node.bias[2] - BIAS) < 1e-6, node.bias
    feed(0.0, 60, scans=1)
    z_rate = got['lpf'][-1].angular_velocity.z
    print(f'bias captured {math.degrees(node.bias[2]):.2f} deg/s -> /imu/data_lpf z rate {math.degrees(z_rate):.4f} deg/s')
    assert abs(z_rate) < 1e-3, z_rate
    assert got['scan'] and np.allclose(got['scan'][-1].ranges, 1.3), 'flat scan must pass unchanged'
    print('flat scan unchanged: OK')

    feed(8.0, 30, scans=1)                                # 8 deg nose-down: ramp mode on, ground hits dropped
    assert node.ramp_mode, 'ramp mode should be on at 8 deg'
    ranges = np.array(got['scan'][-1].ranges)
    angles = -math.pi + np.arange(360) * 2 * math.pi / 360
    fwd = np.abs(angles) < math.radians(5)
    assert np.isnan(ranges[fwd]).all() and not np.isnan(ranges[np.abs(angles - math.pi / 2) < 0.02]).any()
    print(f'8 deg pitch: ramp mode ON, {int(np.isnan(ranges).sum())} of 360 beams dropped (forward ground hits)')

    feed(1.5, 30, scans=1)                                # between release (1.0) and arm (2.0): stays on
    assert node.ramp_mode, 'hysteresis: still on at 1.5 deg'
    feed(0.2, 30, scans=1)                                # below release: off, scan unchanged again
    assert not node.ramp_mode and np.allclose(got['scan'][-1].ranges, 1.3)
    print('hysteresis release: OK')
    print('ALL OK')
    node.destroy_node()
    helper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


def test_footprint_filter_drops_body_near_far():
    import numpy as np
    sp = pytest.importorskip('robot_slam.sensor_prep')
    ang = np.radians([0.0, 180.0, 90.0, 45.0, 0.0])
    r = [0.05, 0.20, 0.5, 8.0, 1.0]           # too near / behind = inside the body box / ok / too far / ok
    out = sp.footprint_filter(r, ang, 0.15, 7.0, (0.10, 0.0), [[-0.14, 0.14, -0.135, 0.135]])
    assert np.isnan(out[0]) and np.isnan(out[1]) and np.isnan(out[3])
    assert out[2] == pytest.approx(0.5) and out[4] == pytest.approx(1.0)

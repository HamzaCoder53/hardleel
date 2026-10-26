"""CalibStats: the startup gyro calibration notices a robot that moved."""
import random

import pytest

imu = pytest.importorskip('robot_slam.imu_serial_node')


def feed(st, n=300, gyro_noise=0.02, bias_z=-12.4, accel_noise=0.002, push=None):
    rnd = random.Random(1)
    for i in range(n):
        gz = bias_z + rnd.gauss(0, gyro_noise) + (push(i) if push else 0.0)
        st.add(0.0, 0.0, 1.0 + rnd.gauss(0, accel_noise), rnd.gauss(0, gyro_noise), rnd.gauss(0, gyro_noise), gz)


def test_still_robot_passes_and_bias_is_mean():
    st = imu.CalibStats()
    feed(st)
    assert not st.moved(0.3, 0.02)
    assert st.bias()[2] == pytest.approx(-12.4, abs=0.01)


def test_turn_during_calibration_is_flagged():
    st = imu.CalibStats()
    feed(st, push=lambda i: 20.0 if 100 < i < 160 else 0.0)     # someone rotates the robot for 0.6 s
    assert st.moved(0.3, 0.02)


def test_bump_is_flagged_by_accel():
    st = imu.CalibStats()
    feed(st, accel_noise=0.05)
    assert st.moved(0.3, 0.02)

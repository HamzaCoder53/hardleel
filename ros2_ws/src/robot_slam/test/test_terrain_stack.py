"""Logic tests for the terrain stack (no robot, no motors).

Run from the repo root with a ROS 2 environment sourced:
    python3 ros2_ws/src/robot_slam/test/test_terrain_stack.py
Modules that need hardware or Nav2 messages (serial, nav2_msgs) are replaced by stubs when missing.
"""
import math
import os
import sys
import types
import unittest
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.abspath(os.path.join(HERE, '..', '..'))
sys.path[:0] = [os.path.join(SRC, 'robot_slam'), os.path.join(SRC, 'motor_teleop')]

import numpy as np  # noqa: E402

if 'serial' not in sys.modules:
    try:
        import serial  # noqa: F401
    except ImportError:
        stub = types.ModuleType('serial')
        stub.SerialException = OSError
        stub.Serial = mock.MagicMock()
        sys.modules['serial'] = stub
for name in ('nav2_msgs', 'nav2_msgs.action', 'nav2_msgs.msg', 'nav2_msgs.srv'):
    try:
        __import__(name)
    except ImportError:
        sys.modules[name] = mock.MagicMock()

from robot_slam.sensor_prep import gate_ranges, Hysteresis  # noqa: E402
from robot_slam.coverage_mission import generate_boustrophedon, drop_keepout  # noqa: E402
from motor_teleop.motor_bridge import MotorBridge  # noqa: E402

D = math.radians


class GateTests(unittest.TestCase):
    ang = np.array([0.0, math.pi / 2, math.pi])      # forward, left, backward
    args = dict(lidar_h=0.15, robot_h=0.35, margin=0.03, rmin=0.18, rmax=5.5)

    def test_flat_is_unchanged(self):
        r = np.array([1.0, 2.0, 3.0])
        out = gate_ranges(r, self.ang, 0.0, 0.0, **self.args)
        np.testing.assert_allclose(out, r)

    def test_nose_down_drops_far_ground_hits_forward_only(self):
        r = np.array([1.3, 1.3, 1.3])                # forward beam reaches the ground at 0.12 / sin(8 deg) = 0.86 m
        out = gate_ranges(r, self.ang, D(8), 0.0, **self.args)
        self.assertTrue(np.isnan(out[0]))
        self.assertEqual(out[1], np.float32(1.3))    # side beams stay level
        self.assertEqual(out[2], np.float32(1.3))    # the rear beam points up: 0.15 + 0.18 = 0.33 < 0.35 stays

    def test_near_hit_is_kept_on_a_slope(self):
        out = gate_ranges(np.array([0.5]), np.array([0.0]), D(8), 0.0, **self.args)
        self.assertFalse(np.isnan(out[0]))           # a wall 0.5 m ahead: z = 0.15 - 0.07 = 0.08

    def test_nose_up_drops_overhead_hits(self):
        out = gate_ranges(np.array([2.0]), np.array([0.0]), D(-8), 0.0, **self.args)
        self.assertTrue(np.isnan(out[0]))            # 0.15 + 2 * 0.139 = 0.43 m, above the robot

    def test_invalid_ranges_become_nan(self):
        out = gate_ranges(np.array([np.inf, 0.05, 9.0]), np.zeros(3), 0.0, 0.0, **self.args)
        self.assertTrue(np.isnan(out).all())


class HysteresisTests(unittest.TestCase):
    def test_arm_release(self):
        h = Hysteresis(D(2.0), D(1.0))
        seq = [(1.5, False), (2.5, True), (1.5, True), (0.9, False), (1.5, False)]
        for deg, expect in seq:
            self.assertEqual(h.update(D(deg)), expect, deg)


class CoverageTests(unittest.TestCase):
    def test_sweep_stays_inside_and_alternates(self):
        pts = generate_boustrophedon(4.9, 2.4, 0.6, 0.5)
        self.assertEqual(len(pts), 6)
        for x, y, _ in pts:
            self.assertTrue(0.5 - 1e-9 <= x <= 4.4 + 1e-9 and 0.5 - 1e-9 <= y <= 1.9 + 1e-9)
        self.assertEqual([round(p[0], 2) for p in pts], [0.5, 4.4, 4.4, 0.5, 0.5, 4.4])
        self.assertEqual([p[2] for p in pts], [0.0, 0.0, math.pi, math.pi, 0.0, 0.0])

    def test_keepout_drops_nearby_waypoints(self):
        pts = generate_boustrophedon(4.9, 2.4, 0.6, 0.5)
        kept = drop_keepout(pts, [3.0, 0.0, 4.0, 1.0], standoff=0.5)    # grows to x 2.5..4.5, y -0.5..1.5
        self.assertTrue(all(not (2.5 <= p[0] <= 4.5 and -0.5 <= p[1] <= 1.5) for p in kept))
        self.assertLess(len(kept), len(pts))

    def test_bad_margin_raises(self):
        with self.assertRaises(ValueError):
            generate_boustrophedon(1.0, 1.0, 0.5, 0.6)


class FakeClock:
    def now(self):
        return self

    def __sub__(self, other):
        return types.SimpleNamespace(nanoseconds=0)


class MotorLogicTests(unittest.TestCase):
    def bridge(self, **kw):
        b = object.__new__(MotorBridge)
        b.ramp_armed, b.dir_l, b.dir_r = True, 1, 1
        b.pitch0, b.pitch, b.pitch_up_sign = 0.0, -0.2, -1.0          # nose up by 0.2 rad (about 11.5 deg)
        b.gravity_ff, b.gravity_ff_max = 450.0, 90.0
        b.imu_timeout, b.last_imu = 0.5, object()
        b.imu_fresh = lambda: True
        for k, v in kw.items():
            setattr(b, k, v)
        return b

    def test_uphill_forward_adds_power_clamped(self):
        self.assertAlmostEqual(self.bridge(pitch=-0.5).feedforward(), 90.0)   # 450 * sin(0.5) = 216 -> clamped to 90
        b = self.bridge(pitch=-0.1)
        self.assertAlmostEqual(b.feedforward(), 450 * math.sin(0.1), places=6)

    def test_downhill_forward_removes_power(self):
        self.assertLess(self.bridge(pitch=0.1).feedforward(), 0.0)

    def test_reverse_uphill_is_downhill_for_the_robot(self):
        self.assertLess(self.bridge(dir_l=-1, dir_r=-1).feedforward(), 0.0)

    def test_no_feedforward_when_turning_in_place_stopped_or_not_armed(self):
        self.assertEqual(self.bridge(dir_l=-1, dir_r=1).feedforward(), 0.0)
        self.assertEqual(self.bridge(dir_l=0, dir_r=0).feedforward(), 0.0)
        self.assertEqual(self.bridge(ramp_armed=False).feedforward(), 0.0)
        self.assertEqual(self.bridge(imu_fresh=lambda: False).feedforward(), 0.0)

    def test_shape_caps_and_never_flips_sign(self):
        b = self.bridge()
        self.assertEqual(b.shape(200, 40.0, 180), 180)                   # cap wins
        self.assertEqual(b.shape(-200, 40.0, 220), -220)
        self.assertEqual(b.shape(30, -90.0, 220), 0)                     # reduced to zero, not reversed
        self.assertEqual(b.shape(0, 90.0, 220), 0)                       # nothing at standstill

    def test_ramp_uses_the_surface_slew(self):
        b = self.bridge(profiles={'FLAT': (220, 600.0), 'SAND': (150, 250.0), 'GRAVEL': (180, 300.0)},
                        surface='SAND', ramp_decel=1500.0, dt=0.02)
        self.assertAlmostEqual(b.ramp(0.0, 100), 250.0 * 0.02)           # 5 PWM per 20 ms tick
        self.assertAlmostEqual(b.ramp(100.0, 0), 100.0 - 1500.0 * 0.02)  # braking uses ramp_decel


if __name__ == '__main__':
    unittest.main(verbosity=2)

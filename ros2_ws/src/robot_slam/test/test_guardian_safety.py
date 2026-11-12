"""The guardian must never move the robot on its own in manual mode.

    source /opt/ros/<distro>/setup.bash
    python3 ros2_ws/src/robot_slam/test/test_guardian_safety.py
Regression for 22 Oct 2026: with an object within 0.28 m of the LiDAR the old bumper pivoted the robot at 85 % of
the turn limit although nobody was driving; repulsion and gyro damping also turned it, and the last keyboard
command never expired.
"""
import math
import os
import sys
import unittest

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from robot_slam.hardleel_guardian_node import HardleelGuardianNode  # noqa: E402


def scan(front=3.0, side=3.0):
    s = LaserScan()
    s.angle_min, s.angle_increment = -math.pi, 2 * math.pi / 360
    s.range_min, s.range_max = 0.15, 12.0
    s.ranges = [float(front) if abs(-math.pi + i * s.angle_increment) < math.radians(25) else float(side)   # symmetric cone, no boundary beam
                for i in range(360)]
    return s


class GuardianSafety(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def node(self, auto=False):
        n = HardleelGuardianNode()
        n.set_parameters([rclpy.parameter.Parameter('autonomous_explore', rclpy.Parameter.Type.BOOL, auto)])
        self.out = []
        n.pub_cmd_safe.publish = self.out.append
        self.addCleanup(n.destroy_node)
        return n

    def command(self, n, lin=0.0, ang=0.0):
        t = Twist()
        t.linear.x, t.angular.z = lin, ang
        n.cmd_cb(t)

    def test_no_motion_with_close_object_and_no_operator(self):
        n = self.node()
        n.latest_scan = scan(front=0.20)
        n.gyro_z = 0.15                                   # gyro noise must not turn the robot either
        n.control_loop()
        self.assertEqual((self.out[-1].linear.x, self.out[-1].angular.z), (0.0, 0.0))

    def test_close_object_stops_forward_but_does_not_turn(self):
        n = self.node()
        n.latest_scan = scan(front=0.20)
        self.command(n, lin=0.15)
        n.control_loop()
        self.assertEqual(self.out[-1].linear.x, 0.0)
        self.assertEqual(self.out[-1].angular.z, 0.0)

    def test_old_operator_command_expires(self):
        n = self.node()
        n.latest_scan = scan(front=3.0)
        self.command(n, lin=0.15)
        n.control_loop()
        self.assertGreater(self.out[-1].linear.x, 0.0)    # fresh command is passed through
        n.raw_stamp = n.get_clock().now() - Duration(seconds=2)
        n.control_loop()
        self.assertEqual((self.out[-1].linear.x, self.out[-1].angular.z), (0.0, 0.0))

    def test_commanded_turn_still_works(self):
        n = self.node()
        n.latest_scan = scan(front=3.0)
        self.command(n, ang=0.3)
        n.control_loop()
        self.assertGreater(self.out[-1].angular.z, 0.2)

    def test_autonomous_mode_still_pivots_away_from_a_close_object(self):
        n = self.node(auto=True)
        n.latest_scan = scan(front=0.20, side=2.0)
        n.control_loop()
        self.assertEqual(self.out[-1].linear.x, 0.0)
        self.assertNotEqual(self.out[-1].angular.z, 0.0)

    def test_publish_cmd_vel_false_never_publishes(self):
        n = self.node(auto=True)
        n.set_parameters([rclpy.parameter.Parameter('publish_cmd_vel', rclpy.Parameter.Type.BOOL, False)])
        n.latest_scan = scan(front=0.20, side=2.0)
        self.command(n, lin=0.15)
        n.control_loop()
        self.assertEqual(self.out, [])                    # the keyboard drives /cmd_vel directly in this mode


if __name__ == '__main__':
    unittest.main(verbosity=2)

#!/usr/bin/env python3
"""Time-based barrier for the RDJ 2026 simulation.

Raises / lowers the 'gate' model in Gazebo on a timer (random by default, like the
real field) and publishes the state on /gate_open (std_msgs/Bool) for logging.
Your robot should detect the gate with its own sensors - /gate_open is for checking.

Parameters:
  open_s, closed_s   mean seconds open / closed (default from arena_layout.yaml values)
  randomize          vary each period by +-50 % (default true)
  world              Gazebo world name (default rdj2026)
"""
import random
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class GateController(Node):
    def __init__(self):
        super().__init__('gate_controller')
        self.open_s = float(self.declare_parameter('open_s', 20.0).value)
        self.closed_s = float(self.declare_parameter('closed_s', 20.0).value)
        self.randomize = bool(self.declare_parameter('randomize', True).value)
        self.world = self.declare_parameter('world', 'rdj2026').value
        self.x = float(self.declare_parameter('x', 5.627).value)
        self.y = float(self.declare_parameter('y', 2.875).value)
        self.height = float(self.declare_parameter('height', 0.30).value)
        self.pub = self.create_publisher(Bool, 'gate_open', 1)
        self.is_open = random.random() < 0.5
        self.timer = None
        self.apply()

    def period(self):
        base = self.open_s if self.is_open else self.closed_s
        return max(1.0, base * random.uniform(0.5, 1.5)) if self.randomize else base

    def apply(self):
        z = -self.height if self.is_open else self.height / 2   # open = sunk below the floor
        req = f'name: "gate", position: {{x: {self.x}, y: {self.y}, z: {z}}}'
        try:
            subprocess.run(['ign', 'service', '-s', f'/world/{self.world}/set_pose',
                            '--reqtype', 'ignition.msgs.Pose', '--reptype', 'ignition.msgs.Boolean',
                            '--timeout', '2000', '--req', req],
                           check=False, capture_output=True, timeout=5)
        except (OSError, subprocess.TimeoutExpired) as e:
            self.get_logger().warn(f'set_pose failed: {e}')
        self.pub.publish(Bool(data=self.is_open))
        p = self.period()
        self.get_logger().info(f'gate {"OPEN" if self.is_open else "CLOSED"} for {p:.1f} s')
        if self.timer:
            self.timer.cancel()
        self.timer = self.create_timer(p, self.toggle)

    def toggle(self):
        self.is_open = not self.is_open
        self.apply()


def main():
    rclpy.init()
    node = GateController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

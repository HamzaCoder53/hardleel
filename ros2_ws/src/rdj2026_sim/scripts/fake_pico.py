#!/usr/bin/env python3
"""Fake Pico for the sim: lets the REAL motor_bridge drive the simulated robot through a motor model.

motor_bridge opens the pty this node creates (link: `port`, default /tmp/fake_pico) and sends its usual
`STBY,0,1` / `M,<id>,<pwm>` lines. Each 20 ms the node turns the last PWMs into a body twist on
`sim_cmd_vel` (the Gazebo diff-drive bridge in sim.launch.py hardware:=true) with a crude DC-motor model:

    wheel speed = sign(pwm) * max(0, |pwm| - deadband) / (255 - deadband) * v_max * terrain_factor
    v = (vr + vl) / 2,  w = (vr - vl) / track

Channel mapping is the real robot's (see motor_bridge.to_sent): with invert_linear the Pico channels 0/1
drive the physical RIGHT side reversed and 2/3 the LEFT side reversed. terrain_factor < 1 inside
`slow_zone` [x0, y0, x1, y1] of the sim world (needs /ground_truth) imitates sand/gravel for the speed
feedback tests. Params: port, deadband_left, deadband_right, v_max, track, slow_zone, slow_factor, invert_linear.
"""
import os
import pty
import tty

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class FakePico(Node):
    def __init__(self):
        super().__init__('fake_pico')
        p = self.declare_parameter
        self.link = p('port', '/tmp/fake_pico').value
        self.db_l, self.db_r = float(p('deadband_left', 60.0).value), float(p('deadband_right', 70.0).value)
        self.v_max = float(p('v_max', 0.9).value)          # m/s at PWM 255
        self.track = float(p('track', 0.50).value)          # EFFECTIVE track: skid steer turns much less than the 0.2 m geometry
        self.zone = list(p('slow_zone', [-1.0, -1.0, -1.0, -1.0]).value)   # world x0 y0 x1 y1 (sim frame)
        self.slow = float(p('slow_factor', 0.6).value)
        self.invert_linear = bool(p('invert_linear', True).value)
        self.master, slave = pty.openpty()
        tty.setraw(self.master)
        os.set_blocking(self.master, False)
        try:
            os.remove(self.link)
        except FileNotFoundError:
            pass
        os.symlink(os.ttyname(slave), self.link)
        self.pwm = [0, 0, 0, 0]
        self.buf = b''
        self.xy = None
        self.pub = self.create_publisher(Twist, 'sim_cmd_vel', 10)
        self.create_subscription(Odometry, '/ground_truth', self.on_truth, 10)
        self.create_timer(0.02, self.tick)
        self.get_logger().info(f'fake Pico on {self.link} -> {os.ttyname(slave)}; deadband L/R {self.db_l}/{self.db_r}, '
                               f'v_max {self.v_max}, slow zone {self.zone} x{self.slow}')

    def on_truth(self, m):
        self.xy = (m.pose.pose.position.x, m.pose.pose.position.y)

    def factor(self):
        x0, y0, x1, y1 = self.zone
        if self.xy and x0 <= self.xy[0] <= x1 and y0 <= self.xy[1] <= y1:
            return self.slow
        return 1.0

    def wheel(self, pwm, db):
        mag = max(0.0, abs(pwm) - db) / (255.0 - db) * self.v_max * self.factor()
        return mag if pwm > 0 else -mag

    def tick(self):
        try:
            self.buf += os.read(self.master, 4096)
        except (BlockingIOError, OSError):
            pass
        while b'\n' in self.buf:
            line, self.buf = self.buf.split(b'\n', 1)
            parts = line.decode(errors='ignore').strip().split(',')
            if parts[0] == 'M' and len(parts) == 3:
                try:
                    self.pwm[int(parts[1])] = int(parts[2])
                except (ValueError, IndexError):
                    pass
            elif parts[0] == 'STOP':
                self.pwm = [0, 0, 0, 0]
        sl, sr = self.pwm[0], self.pwm[2]                # sent left / right channel
        if self.invert_linear:
            pr, pl = -sl, -sr                            # inverse of motor_bridge.to_sent
        else:
            pl, pr = sl, sr
        vl, vr = self.wheel(pl, self.db_l), self.wheel(pr, self.db_r)
        t = Twist()
        t.linear.x = (vl + vr) / 2.0
        t.angular.z = (vr - vl) / self.track
        self.pub.publish(t)


def main():
    rclpy.init()
    n = FakePico()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            os.remove(n.link)
        except OSError:
            pass
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

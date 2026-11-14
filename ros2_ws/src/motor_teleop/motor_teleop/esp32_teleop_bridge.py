#!/usr/bin/env python3
"""cmd_vel -> ESP32 running the team's motor_code.ino (BTS7960 drivers), for teleop.

motor_code.ino (115200 baud, newline):  m <left> <right> [ms]   speeds -255..255, runs for ms then stops
                                        x                       stop
The sketch has no watchdog of its own: every drive command is a timed run (hold_ms), so while a
non-zero cmd_vel is fresh this node resends 'm l r hold_ms' at send_rate; if cmd_vel stops for
cmd_timeout it sends 'x', and if this node or the link dies the ESP32 stops by itself after hold_ms.
Values from the team's 26 Oct tuning (feat/esp32-motors): max_pwm 75, hold 200 ms (400 overshot
turns), LEFT side reversed. max_speed 0.5 so teleop_twist_keyboard's default 0.5 m/s = max_pwm.
The port is opened with DTR/RTS released: asserted, they hold the ESP32 in reset (13-14 Nov).
"""
import glob
import threading
import time

import rclpy
import serial
from geometry_msgs.msg import Twist
from rclpy.node import Node


def default_port():
    found = sorted(glob.glob('/dev/serial/by-path/*usb-0:1.1:1.0-port0'))
    return found[0] if found else '/dev/ttyUSB1'


class Esp32TeleopBridge(Node):
    def __init__(self):
        super().__init__('esp32_teleop_bridge')
        p = self.declare_parameter
        p('port', default_port())
        p('max_speed', 0.5)      # m/s that maps to max_pwm
        p('turn_gain', 0.5)      # wheel speed (m/s) per rad/s of angular.z
        p('max_pwm', 75)         # hard cap (team 26 Oct)
        p('min_pwm', 70)         # stall floor: team calibration 26 Oct (~/hardleel_ws/motor_calib.yaml: 70 both
                                 # sides). Below it Nav2's 0.27-0.35 m/s (PWM 40-52) did not move the robot.
        p('zero_speed', 0.02)    # m/s: smaller wheel speeds count as 0 (no 70-PWM jerk at the end of a ramp)
        p('invert_left', True)   # ESP32 + BTS7960 wiring: left side runs reversed (26 Oct)
        p('invert_right', False)
        p('hold_ms', 200)        # each 'm' runs this long on the ESP32 (its only timeout)
        p('send_rate', 10.0)     # Hz resend while driving
        p('cmd_timeout', 0.6)    # s without cmd_vel -> 'x'. teleop_twist_keyboard sends one Twist per key
                                 # event and key repeat starts after ~0.5 s: 0.3 made driving stutter
        g = lambda n: self.get_parameter(n).value
        self.max_speed, self.turn_gain = float(g('max_speed')), float(g('turn_gain'))
        self.max_pwm, self.min_pwm = int(g('max_pwm')), int(g('min_pwm'))
        self.zero_speed = float(g('zero_speed'))
        self.inv_l, self.inv_r = bool(g('invert_left')), bool(g('invert_right'))
        self.hold_ms, self.cmd_timeout = int(g('hold_ms')), float(g('cmd_timeout'))

        self.ser = serial.Serial()
        self.ser.port, self.ser.baudrate, self.ser.timeout = g('port'), 115200, 0.1
        self.ser.dtr = False
        self.ser.rts = False
        self.ser.open()
        time.sleep(1.5)                      # in case opening reset the board
        self.ser.reset_input_buffer()
        self.lock = threading.Lock()
        self.target = (0, 0)
        self.last_cmd = 0.0
        self.driving = False
        self.send('x')
        self.get_logger().info(f'ESP32 on {g("port")}: pwm {self.min_pwm}..{self.max_pwm}, '
                               f'max_speed {self.max_speed} m/s, hold {self.hold_ms} ms')

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.create_timer(1.0 / float(g('send_rate')), self.tick)
        threading.Thread(target=self.reader, daemon=True).start()

    def to_pwm(self, wheel_speed):
        """Wheel speed (m/s) -> signed PWM. Deadband offset like the team's bridge (26 Oct):
        0 stays 0, otherwise min_pwm + |p| * (max_pwm - min_pwm) / max_pwm."""
        if abs(wheel_speed) < self.zero_speed:
            return 0
        p = max(-1.0, min(1.0, wheel_speed / self.max_speed)) * self.max_pwm
        if self.min_pwm > 0:
            p = (self.min_pwm + abs(p) * (self.max_pwm - self.min_pwm) / self.max_pwm) * (1 if p > 0 else -1)
        return int(round(p))

    def on_cmd(self, msg):
        v, w = msg.linear.x, msg.angular.z
        left = self.to_pwm(v - w * self.turn_gain)
        right = self.to_pwm(v + w * self.turn_gain)
        if self.inv_l:
            left = -left
        if self.inv_r:
            right = -right
        self.target = (left, right)
        self.last_cmd = time.monotonic()

    def tick(self):
        fresh = time.monotonic() - self.last_cmd < self.cmd_timeout
        left, right = self.target if fresh else (0, 0)
        if left == 0 and right == 0:
            if self.driving:
                self.send('x')
                self.driving = False
            return
        self.send(f'm {left} {right} {self.hold_ms}')
        self.driving = True

    def send(self, line):
        with self.lock:
            try:
                self.ser.write((line + '\n').encode())
            except serial.SerialException as e:
                self.get_logger().error(f'serial write failed: {e}')

    def reader(self):
        while rclpy.ok():
            try:
                text = self.ser.readline().decode(errors='replace').strip()
            except (serial.SerialException, OSError):
                return
            if text and not text.startswith(('Run', 'Left', 'Right', 'm ', 'Motors')):
                self.get_logger().info(f'ESP32: {text}', throttle_duration_sec=2.0)

    def stop(self):
        for _ in range(3):
            self.send('x')
            time.sleep(0.05)


def main():
    rclpy.init()
    node = Esp32TeleopBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

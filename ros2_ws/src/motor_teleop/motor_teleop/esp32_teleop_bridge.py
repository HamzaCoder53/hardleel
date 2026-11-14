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
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node

# name: (default, description, (min, max) or None)
PARAMS = {
    'port': ('', 'ESP32 serial port (start only)', None),
    'max_speed': (0.5, 'wheel speed m/s that maps to max_pwm', (0.05, 3.0)),
    'turn_gain': (0.5, 'wheel speed m/s per rad/s of angular.z', (0.05, 3.0)),
    'max_pwm': (75, 'PWM cap (team 26 Oct: 75)', (0, 255)),
    'min_pwm': (70, 'stall floor for non-zero commands (team calibration 26 Oct: 70)', (0, 255)),
    'zero_speed': (0.02, 'wheel speeds below this m/s count as 0', (0.0, 0.5)),
    'invert_left': (True, 'left side runs reversed (ESP32 + BTS7960 wiring)', None),
    'invert_right': (False, 'right side runs reversed', None),
    'hold_ms': (200, 'ESP32 watchdog: each drive command runs this long (ms)', (50, 2000)),
    'send_rate': (10.0, 'Hz the drive command is resent while driving', (1.0, 50.0)),
    'cmd_timeout': (0.6, 'Pi watchdog: s without cmd_vel before stop', (0.1, 5.0)),
}


def default_port():
    found = sorted(glob.glob('/dev/serial/by-path/*usb-0:1.1:1.0-port0'))
    return found[0] if found else '/dev/ttyUSB1'


class Esp32TeleopBridge(Node):
    def __init__(self):
        super().__init__('esp32_teleop_bridge')
        # name: (default, description, (min, max) or None). All but 'port' can be changed live
        # (tuning GUI / ros2 param set); a node restart returns to these defaults unless saved.
        for name, (default, desc, rng) in PARAMS.items():
            d = ParameterDescriptor(description=desc + (f'  [{rng[0]}..{rng[1]}]' if rng else ''))
            self.declare_parameter(name, default_port() if name == 'port' else default, d)
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
        self.timer = self.create_timer(1.0 / float(g('send_rate')), self.tick)
        self.add_on_set_parameters_callback(self.on_params)
        threading.Thread(target=self.reader, daemon=True).start()

    def on_params(self, params):
        """Live tuning: check every value first, then apply all of them (all or nothing)."""
        new = {p.name: p.value for p in params}
        for name, value in new.items():
            if name == 'port':
                return SetParametersResult(successful=False, reason='port is read at start only: Save, then restart the bridge')
            if name not in PARAMS:
                continue
            rng = PARAMS[name][2]
            if rng and not rng[0] <= value <= rng[1]:
                return SetParametersResult(successful=False, reason=f'{name} must be {rng[0]}..{rng[1]}')
        lo = new.get('min_pwm', self.min_pwm)
        hi = new.get('max_pwm', self.max_pwm)
        if lo > hi:
            return SetParametersResult(successful=False, reason=f'min_pwm ({lo}) must be <= max_pwm ({hi})')
        for name, value in new.items():
            if name == 'max_speed':
                self.max_speed = float(value)
            elif name == 'turn_gain':
                self.turn_gain = float(value)
            elif name == 'max_pwm':
                self.max_pwm = int(value)
            elif name == 'min_pwm':
                self.min_pwm = int(value)
            elif name == 'zero_speed':
                self.zero_speed = float(value)
            elif name == 'invert_left':
                self.inv_l = bool(value)
            elif name == 'invert_right':
                self.inv_r = bool(value)
            elif name == 'hold_ms':
                self.hold_ms = int(value)
            elif name == 'cmd_timeout':
                self.cmd_timeout = float(value)
            elif name == 'send_rate':
                self.timer.cancel()
                self.destroy_timer(self.timer)
                self.timer = self.create_timer(1.0 / float(value), self.tick)
        self.get_logger().info('tuned: ' + ', '.join(f'{k}={v}' for k, v in new.items()))
        return SetParametersResult(successful=True)

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

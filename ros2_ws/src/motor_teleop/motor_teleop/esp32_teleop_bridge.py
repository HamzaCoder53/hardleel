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

DIY wheel encoders (16 Nov, docs/WHEEL_ENCODERS.md): the sketch streams 'E <ms> <lf> <lr> <rf> <rr>'.
With encoders:=true this node publishes /wheel/odom (nav_msgs/Odometry, frame odom -> base_link,
NO TF) and /wheel/speeds (Float32MultiArray [lf, lr, rf, rr] m/s). Off by default.

PWM odometry (16 Nov, stop-gap until the encoders work): with pwm_odom:=true this node publishes
/pwm/odom = the speed the SENT PWM should give (two-point calibration pwm_cal_*/pwm_v_*, from
'ros2 run motor_teleop pwm_calibrate'), lagged by pwm_tau, 0 below pwm_stall. It cannot see a
stall or a wall push, so its covariance is large: rf2o + IMU must win. Its best part is the
exact zero while stopped. Off by default; no TF.
"""
import glob
import math
import threading
import time

import rclpy
import serial
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node

from motor_teleop.wheel_odom import PwmOdom, WheelOdom, parse_line

# name: (default, description, (min, max) or None)
PARAMS = {
    'port': ('', 'ESP32 serial port (start only)', None),
    'max_speed': (0.5, 'wheel speed m/s that maps to max_pwm', (0.05, 3.0)),
    'turn_gain': (0.5, 'wheel speed m/s per rad/s of angular.z', (0.05, 3.0)),
    'max_pwm': (75, 'PWM cap (team 26 Oct: 75)', (0, 255)),
    'min_pwm': (70, 'stall floor for non-zero commands (team calibration 26 Oct: 70)', (0, 255)),
    # Turning: skid-steer wheels scrub sideways and need far more torque than driving straight; at 70-75 the
    # wheels did not get enough power to turn (user, 19 Nov). Used when the sides turn opposite ways (spin)
    # or one side is stopped (pivot); straight driving keeps min_pwm..max_pwm.
    'spin_min_pwm': (85, 'PWM floor when turning on the spot / pivoting', (0, 255)),
    'spin_max_pwm': (100, 'PWM cap when turning on the spot / pivoting (watch the motor drivers)', (0, 255)),
    'spin_timeout': (0.25, 'watchdog for TURN commands (s): one teleop key press turned too far with the 0.6 s '
                           'cmd_timeout at spin PWM (user 19 Nov)', (0.05, 5.0)),
    'zero_speed': (0.02, 'wheel speeds below this m/s count as 0', (0.0, 0.5)),
    'invert_left': (True, 'left side runs reversed (ESP32 + BTS7960 wiring)', None),
    'invert_right': (False, 'right side runs reversed', None),
    'hold_ms': (200, 'ESP32 watchdog: each drive command runs this long (ms)', (50, 2000)),
    'send_rate': (10.0, 'Hz the drive command is resent while driving', (1.0, 50.0)),
    'cmd_timeout': (0.6, 'Pi watchdog: s without cmd_vel before stop', (0.1, 5.0)),
    'accel': (1.0, 'wheel speed ramp UP, m/s^2 (0 = jump at once)', (0.0, 20.0)),
    'decel': (2.0, 'wheel speed ramp DOWN, m/s^2 (0 = jump); watchdog stops are always instant', (0.0, 20.0)),
    'encoders': (False, 'publish /wheel/odom + /wheel/speeds from the ESP32 E lines', None),
    'ticks_per_rev': (40.0, 'encoder counts per WHEEL turn (20-slot disc, both edges = 40)', (1.0, 100000.0)),
    'wheel_diameter': (0.065, 'wheel diameter m (measure! calibrate with a 2 m straight run)', (0.01, 0.5)),
    'track_width': (0.30, 'EFFECTIVE track m for encoder yaw (skid steer: calibrate by spinning)', (0.05, 2.0)),
    'enc_window': (0.2, 'speed averaging window s', (0.05, 2.0)),
    'pwm_odom': (False, 'publish /pwm/odom: speed estimated from the sent PWM (no encoders)', None),
    'pwm_stall': (70, 'PWM below which the wheels are assumed NOT to turn', (0, 255)),
    'pwm_cal_lo': (70, 'calibration point 1: PWM', (0, 255)),
    'pwm_v_lo': (0.10, 'calibration point 1: measured m/s at pwm_cal_lo (PLACEHOLDER - run pwm_calibrate)', (0.0, 3.0)),
    'pwm_cal_hi': (75, 'calibration point 2: PWM', (0, 255)),
    'pwm_v_hi': (0.15, 'calibration point 2: measured m/s at pwm_cal_hi (PLACEHOLDER - run pwm_calibrate)', (0.0, 3.0)),
    'pwm_tau': (0.15, 'motor lag s (first order) between PWM and speed', (0.0, 2.0)),
    'pwm_vx_std': (0.08, 'm/s 1-sigma for /pwm/odom vx (large: it is a guess)', (0.001, 5.0)),
}
ENC_PARAMS = ('ticks_per_rev', 'wheel_diameter', 'track_width', 'enc_window')
PWM_PARAMS = ('pwm_stall', 'pwm_cal_lo', 'pwm_v_lo', 'pwm_cal_hi', 'pwm_v_hi', 'pwm_tau', 'pwm_vx_std', 'track_width')



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
        self.spin_min_pwm, self.spin_max_pwm = int(g('spin_min_pwm')), int(g('spin_max_pwm'))
        self.spin_timeout = float(g('spin_timeout'))
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
        self.accel, self.decel = float(g('accel')), float(g('decel'))
        self.target = (0.0, 0.0)          # wheel speeds (m/s) asked for by cmd_vel
        self.cur = [0.0, 0.0]             # wheel speeds after the accel/decel ramp
        self.last_tick = time.monotonic()
        self.last_cmd = 0.0
        self.driving = False
        self.send('x')
        self.get_logger().info(f'ESP32 on {g("port")}: pwm {self.min_pwm}..{self.max_pwm}, '
                               f'max_speed {self.max_speed} m/s, hold {self.hold_ms} ms')

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.timer = self.create_timer(1.0 / float(g('send_rate')), self.tick)
        self.encoders = bool(g('encoders'))
        self.odom = WheelOdom()
        self.apply_enc_params({n: g(n) for n in ENC_PARAMS})
        self.odom_pub = self.create_publisher(Odometry, 'wheel/odom', 10)
        self.speed_pub = self.create_publisher(Float32MultiArray, 'wheel/speeds', 10)
        self.pwm_odom_on = bool(g('pwm_odom'))
        self.pwm_odom = PwmOdom()
        self.pwm_vx_std = 0.08
        self.apply_pwm_params({n: g(n) for n in PWM_PARAMS})
        self.pwm_pub = self.create_publisher(Odometry, 'pwm/odom', 10)
        self.last_pwm_t = time.monotonic()
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
        slo = new.get('spin_min_pwm', self.spin_min_pwm)
        shi = new.get('spin_max_pwm', self.spin_max_pwm)
        if slo > shi:
            return SetParametersResult(successful=False, reason=f'spin_min_pwm ({slo}) must be <= spin_max_pwm ({shi})')
        for name, value in new.items():
            if name == 'pwm_odom':
                self.pwm_odom_on = bool(value)
            if name in PWM_PARAMS:
                self.apply_pwm_params({name: value})
            if name == 'max_speed':
                self.max_speed = float(value)
            elif name == 'turn_gain':
                self.turn_gain = float(value)
            elif name == 'max_pwm':
                self.max_pwm = int(value)
            elif name == 'min_pwm':
                self.min_pwm = int(value)
            elif name == 'spin_min_pwm':
                self.spin_min_pwm = int(value)
            elif name == 'spin_max_pwm':
                self.spin_max_pwm = int(value)
            elif name == 'spin_timeout':
                self.spin_timeout = float(value)
            elif name == 'zero_speed':
                self.zero_speed = float(value)
            elif name == 'invert_left':
                self.inv_l = bool(value)
                self.apply_enc_params({})
            elif name == 'invert_right':
                self.inv_r = bool(value)
                self.apply_enc_params({})
            elif name == 'hold_ms':
                self.hold_ms = int(value)
            elif name == 'cmd_timeout':
                self.cmd_timeout = float(value)
            elif name == 'accel':
                self.accel = float(value)
            elif name == 'decel':
                self.decel = float(value)
            elif name == 'encoders':
                self.encoders = bool(value)
            elif name in ENC_PARAMS:
                self.apply_enc_params({name: value})
            elif name == 'send_rate':
                self.timer.cancel()
                self.destroy_timer(self.timer)
                self.timer = self.create_timer(1.0 / float(value), self.tick)
        self.get_logger().info('tuned: ' + ', '.join(f'{k}={v}' for k, v in new.items()))
        return SetParametersResult(successful=True)

    def apply_enc_params(self, p):
        o = self.odom
        o.ticks_per_rev = float(p.get('ticks_per_rev', o.ticks_per_rev))
        o.wheel_diameter = float(p.get('wheel_diameter', o.wheel_diameter))
        o.track_width = float(p.get('track_width', o.track_width))
        o.window = float(p.get('enc_window', o.window))
        # counts carry the sketch's sign; flip them the same way the PWM is flipped
        o.sign_left, o.sign_right = (-1 if self.inv_l else 1), (-1 if self.inv_r else 1)

    def apply_pwm_params(self, p):
        o = self.pwm_odom
        o.stall_pwm = int(p.get('pwm_stall', o.stall_pwm))
        o.pwm_lo = int(p.get('pwm_cal_lo', o.pwm_lo))
        o.v_lo = float(p.get('pwm_v_lo', o.v_lo))
        o.pwm_hi = int(p.get('pwm_cal_hi', o.pwm_hi))
        o.v_hi = float(p.get('pwm_v_hi', o.v_hi))
        o.tau = float(p.get('pwm_tau', o.tau))
        o.track_width = float(p.get('track_width', o.track_width))
        self.pwm_vx_std = float(p.get('pwm_vx_std', self.pwm_vx_std))

    def publish_pwm_odom(self, pwm_left, pwm_right):
        """pwm_*: forward-positive PWM just sent (0 when stopped)."""
        now = time.monotonic()
        dt, self.last_pwm_t = now - self.last_pwm_t, now
        r = self.pwm_odom.update(dt, pwm_left, pwm_right)
        if not self.pwm_odom_on:
            return
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
        msg.pose.pose.position.x, msg.pose.pose.position.y = r['x'], r['y']
        msg.pose.pose.orientation.z = math.sin(r['yaw'] / 2.0)
        msg.pose.pose.orientation.w = math.cos(r['yaw'] / 2.0)
        pc = [0.0] * 36
        for i in (0, 7, 14, 21, 28, 35):
            pc[i] = 1e3               # dead reckoning from commands: never fuse the pose
        msg.pose.covariance = pc
        msg.twist.twist.linear.x, msg.twist.twist.angular.z = r['vx'], r['wz']
        tc = [0.0] * 36
        # stopped -> confidently 0; driving -> a loose guess (stall / wall / battery / floor)
        tc[0] = 0.01 ** 2 if r['vx'] == 0.0 and pwm_left == 0 and pwm_right == 0 else self.pwm_vx_std ** 2
        tc[7] = 1e-4                  # vy = 0 (non-holonomic)
        tc[14] = tc[21] = tc[28] = 1e3
        tc[35] = 1.0                  # wz from PWM: skid-steer slip, the IMU owns yaw
        msg.twist.covariance = tc
        self.pwm_pub.publish(msg)

    def on_encoders(self, millis, counts):
        r = self.odom.update(millis, counts)
        if r is None or not self.encoders:
            return
        if r['faults']:
            self.get_logger().warn(f'encoder looks dead: {", ".join(r["faults"])} '
                                   '(using the other wheel of that side)', throttle_duration_sec=5.0)
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
        msg.pose.pose.position.x, msg.pose.pose.position.y = r['x'], r['y']
        msg.pose.pose.orientation.z = math.sin(r['yaw'] / 2.0)
        msg.pose.pose.orientation.w = math.cos(r['yaw'] / 2.0)
        pc = [0.0] * 36
        for i in (0, 7, 14, 21, 28, 35):
            pc[i] = 1e3               # pose is dead reckoning only: fuse the twist, not this
        msg.pose.covariance = pc
        msg.twist.twist.linear.x, msg.twist.twist.angular.z = r['vx'], r['wz']
        tc = [0.0] * 36
        tc[0] = 0.02 ** 2             # vx: good while the wheels grip
        tc[7] = 1e-4                  # vy = 0 (non-holonomic), fairly sure
        tc[14] = tc[21] = tc[28] = 1e3
        tc[35] = 0.5                  # wz: skid-steer slip, let the IMU own yaw
        msg.twist.covariance = tc
        self.odom_pub.publish(msg)
        self.speed_pub.publish(Float32MultiArray(data=[float(v) for v in r['wheel']]))

    def to_pwm(self, wheel_speed, lo=None, hi=None):
        """Wheel speed (m/s) -> signed PWM. Deadband offset like the team's bridge (26 Oct):
        0 stays 0, otherwise lo + |speed/max_speed| * (hi - lo). lo/hi default to min_pwm/max_pwm."""
        lo = self.min_pwm if lo is None else lo
        hi = self.max_pwm if hi is None else hi
        if abs(wheel_speed) < self.zero_speed:
            return 0
        f = max(-1.0, min(1.0, wheel_speed / self.max_speed))
        p = (lo + abs(f) * (hi - lo)) if lo > 0 else abs(f) * hi
        return int(round(p)) * (1 if f > 0 else -1)

    def is_turning(self, left, right):
        """Spin (sides opposite) or pivot (one side stopped, the other moving)."""
        zl, zr = abs(left) < self.zero_speed, abs(right) < self.zero_speed
        return (not zl and not zr and (left > 0) != (right > 0)) or (zl != zr)

    def on_cmd(self, msg):
        v, w = msg.linear.x, msg.angular.z
        self.target = (v - w * self.turn_gain, v + w * self.turn_gain)
        self.last_cmd = time.monotonic()

    def ramp(self, cur, tgt, dt):
        """Move one wheel speed toward its target at accel (speeding up) or decel (slowing/reversing)."""
        speeding_up = abs(tgt) > abs(cur) and (cur == 0.0 or (tgt > 0) == (cur > 0))
        rate = self.accel if speeding_up else self.decel
        step = rate * dt
        if rate <= 0.0 or abs(tgt - cur) <= step:
            return tgt
        return cur + (step if tgt > cur else -step)

    def tick(self):
        now = time.monotonic()
        dt, self.last_tick = min(now - self.last_tick, 0.5), now
        timeout = self.spin_timeout if self.is_turning(*self.target) else self.cmd_timeout
        if now - self.last_cmd < timeout:
            self.cur = [self.ramp(c, t, dt) for c, t in zip(self.cur, self.target)]
        else:
            self.cur = [0.0, 0.0]         # watchdog: stop at once, no ramp
        if self.is_turning(*self.cur):       # turning needs more torque than driving straight
            left = self.to_pwm(self.cur[0], self.spin_min_pwm, self.spin_max_pwm)
            right = self.to_pwm(self.cur[1], self.spin_min_pwm, self.spin_max_pwm)
        else:
            left, right = self.to_pwm(self.cur[0]), self.to_pwm(self.cur[1])
        self.publish_pwm_odom(left, right)   # forward-positive, before the wiring inversion
        if self.inv_l:
            left = -left
        if self.inv_r:
            right = -right
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
            if text.startswith('E '):
                parsed = parse_line(text)
                if parsed:
                    self.on_encoders(*parsed)
                continue
            if text and not text.startswith(('Run', 'Left', 'Right', 'm ', 'Motors', 'Encoder')):
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

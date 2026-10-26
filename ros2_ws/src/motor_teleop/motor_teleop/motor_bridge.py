#!/usr/bin/env python3
"""cmd_vel -> ESP32 serial motor bridge (BTS7960 + Dual Servos).

Subscribes : cmd_vel (geometry_msgs/Twist)   e.g. from teleop_twist_keyboard
Publishes  : encoders (std_msgs/Int32MultiArray)
             data = [lf, lr, rf, rr, v_lf, v_lr, v_rf, v_rr]   (ticks, ticks/s)

ESP32 protocol used (newline terminated; firmware esp32_pi_bridge.ino):
  M <left> <right>  side speeds (-255..255, positive = forward)
  S <angle>         both drop servos to angle (0..180)
  X                 stop motors
  P                 ping -> replies PONG
ESP32 replies read here: READY, PONG, ERR <reason>, WATCHDOG STOP.
The ESP32 firmware watchdog (500 ms) brakes the motors if no M command arrives.
While driving, this bridge resends M at least every 100 ms (10 Hz).
The Pi safety watchdog stops the motors if no cmd_vel arrives for 300 ms.
Servos: /motor/servo_cmd (Int32MultiArray [deg] or [deg, dps]) -> S <angle>;
republished on /motor/servo_state (String "moving,<deg>" / "done,<deg>") with timed confirmation.
"""
import glob
import math
import os
import threading

import serial
import rclpy
import yaml
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Int32MultiArray, String

from motor_teleop.esp32_link import Esp32Motors

LIVE = ('min_pwm_left', 'min_pwm_right', 'min_turn_pwm', 'max_speed', 'allow_pwm_test', 'yaw_feedback', 'yaw_kp',
        'yaw_ki', 'speed_scale_enabled', 'dash_input', 'turn_gain')
CALIB_KEYS = ('min_pwm_left', 'min_pwm_right', 'min_turn_pwm', 'max_speed')


def deadband(p, min_pwm, max_pwm):
    """Deadband offset on a signed PWM: 0 stays 0, otherwise min + |p| * (max - min) / max."""
    if p == 0 or min_pwm <= 0:
        return p
    mag = min_pwm + abs(p) * (max_pwm - min_pwm) / float(max_pwm)
    return int(round(math.copysign(min(mag, max_pwm), p)))

SURFACES = ('FLAT', 'SAND', 'GRAVEL')


class MotorBridge(Node):
    def __init__(self):
        super().__init__('motor_bridge')
        self.declare_parameter('port', '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0')
        # which ESP32 sketch is flashed: motor_code = the team's firmware/esp32_motor_code/motor_code.ino (26 Oct,
        # lowercase m/s/x, timed runs); bridge = firmware/esp32_pi_bridge (M/S/X/P + 500 ms watchdog)
        self.declare_parameter('esp32_protocol', 'motor_code')
        self.declare_parameter('esp32_hold_ms', 400)   # motor_code: each drive command runs this long (the timeout)
        self.declare_parameter('max_speed', 1.0)     # Twist value (m/s) that maps to max_pwm
        self.declare_parameter('turn_gain', 0.5)     # wheel speed per rad/s of angular.z
        self.declare_parameter('max_pwm', 255)       # hard cap, 0..255
        self.declare_parameter('min_pwm', 0)         # lift small non-zero commands above stall (0 = allow full fine control down to 0)
        self.declare_parameter('min_turn', 0.0)      # rad/s: smallest in-place spin threshold (0 = no forced floor)
        # ESP32 + BTS7960 wiring, checked on the robot 26 Oct (wheels up, teleop): the LEFT side runs reversed.
        # invert_linear was the Pico/TB6612 mapping (swapped + negated sides) - off for the ESP32.
        self.declare_parameter('invert_left', True)
        self.declare_parameter('invert_right', False)
        self.declare_parameter('invert_linear', False)
        self.declare_parameter('send_rate', 10.0)    # Hz: resends M every 100 ms to feed ESP32 500ms watchdog
        self.declare_parameter('cmd_timeout', 0.3)   # s without cmd_vel -> stop (300 ms Pi safety watchdog)
        self.declare_parameter('flat_max_pwm', 220)    # per-surface PWM cap
        self.declare_parameter('flat_ramp', 600.0)     # per-surface speed-up slew, PWM units/s (0 = no limit)
        self.declare_parameter('sand_max_pwm', 150)
        self.declare_parameter('sand_ramp', 250.0)
        self.declare_parameter('gravel_max_pwm', 180)
        self.declare_parameter('gravel_ramp', 300.0)
        self.declare_parameter('ramp_surface', 'GRAVEL')  # surface assumed while the pitch ramp-arm is on
        self.declare_parameter('imu_topic', '/imu/data_lpf')
        self.declare_parameter('imu_timeout', 0.5)     # s; a stale IMU disables the feedforward and ramp mode
        self.declare_parameter('arm_deg', 2.0)         # |pitch - pitch_boot| above this arms ramp mode
        self.declare_parameter('release_deg', 1.0)     # ... and below this releases it
        self.declare_parameter('pitch_up_sign', -1.0)  # REP-103 pitch is + nose-down, so nose-up = -pitch; flip if the IMU is mounted otherwise
        self.declare_parameter('gravity_ff', 450.0)    # PWM per unit sin(slope)
        self.declare_parameter('gravity_ff_max', 90.0) # PWM clamp on the feedforward (conservative)
        self.declare_parameter('vibration_enabled', False)
        self.declare_parameter('vibration_gyro_std', 0.25)  # rad/s std of roll/pitch rate over 1 s that counts as gravel
        self.declare_parameter('ramp_decel', 1500.0) # PWM units/s when slowing down (0 = no limit)
        # deadband / calibration (see module docstring); calib_file overrides the defaults when it exists
        self.declare_parameter('min_pwm_left', -1)     # -1 = use min_pwm (legacy) for this side
        self.declare_parameter('min_pwm_right', -1)
        self.declare_parameter('min_turn_pwm', 0)      # in-place spin floor per wheel, 0 = off
        self.declare_parameter('calib_file', '~/hardleel_ws/motor_calib.yaml')
        self.declare_parameter('allow_pwm_test', False)
        # feedback, OFF by default (only what was tested on the robot gets switched on)
        self.declare_parameter('yaw_feedback', False)
        self.declare_parameter('yaw_kp', 0.3)          # extra rad/s command per rad/s of yaw-rate error (sim-tuned)
        self.declare_parameter('yaw_ki', 0.4)
        self.declare_parameter('yaw_i_max', 1.5)       # rad/s clamp on the integral term
        self.declare_parameter('gyro_topic', '/imu/data_raw')
        self.declare_parameter('speed_scale_enabled', False)
        # ramp dash (final stack, OFF): /motor/dash_pwm [left, right] physical PWM straight to the Pico, top priority,
        # no surface cap / speed scale / deadband / slew; written as it arrives (max dash_max_hz), resent each tick
        self.declare_parameter('dash_input', False)
        self.declare_parameter('dash_max_hz', 50.0)

        g = lambda n: self.get_parameter(n).value
        self.max_speed = float(g('max_speed'))
        self.turn_gain = float(g('turn_gain'))
        self.max_pwm = max(0, min(255, int(g('max_pwm'))))
        self.min_pwm = max(0, min(255, int(g('min_pwm'))))
        legacy = self.min_pwm
        self.min_l = int(g('min_pwm_left')) if int(g('min_pwm_left')) >= 0 else legacy
        self.min_r = int(g('min_pwm_right')) if int(g('min_pwm_right')) >= 0 else legacy
        self.min_turn_pwm = int(g('min_turn_pwm'))
        self.allow_pwm_test = bool(g('allow_pwm_test'))
        self.yaw_feedback = bool(g('yaw_feedback'))
        self.yaw_kp, self.yaw_ki, self.yaw_i_max = float(g('yaw_kp')), float(g('yaw_ki')), float(g('yaw_i_max'))
        self.speed_scale_enabled = bool(g('speed_scale_enabled'))
        self.dash_input = bool(g('dash_input'))
        self.dash_min_dt = 1.0 / max(float(g('dash_max_hz')), 1.0)
        self.dash = None                    # (left, right, stamp) physical PWM from /motor/dash_pwm
        self.dash_written = 0.0
        self.load_calib(os.path.expanduser(str(g('calib_file'))))
        self.invert_left = bool(g('invert_left'))
        self.min_turn = float(g('min_turn'))
        self.invert_right = bool(g('invert_right'))
        self.invert_linear = bool(g('invert_linear'))
        self.cmd_timeout = float(g('cmd_timeout'))
        self.profiles = {'FLAT': (int(g('flat_max_pwm')), float(g('flat_ramp'))),
                         'SAND': (int(g('sand_max_pwm')), float(g('sand_ramp'))),
                         'GRAVEL': (int(g('gravel_max_pwm')), float(g('gravel_ramp')))}
        self.ramp_surface = str(g('ramp_surface')).upper()
        if self.ramp_surface not in SURFACES:
            self.ramp_surface = 'GRAVEL'
        self.imu_timeout = float(g('imu_timeout'))
        self.arm = math.radians(float(g('arm_deg')))
        self.release = math.radians(float(g('release_deg')))
        self.pitch_up_sign = float(g('pitch_up_sign'))
        self.gravity_ff = float(g('gravity_ff'))
        self.gravity_ff_max = float(g('gravity_ff_max'))
        self.vib_enabled = bool(g('vibration_enabled'))
        self.vib_std = float(g('vibration_gyro_std'))
        self.ramp_decel = float(g('ramp_decel'))
        send_rate = float(g('send_rate'))
        self.dt = 1.0 / send_rate
        port = g('port')
        if not os.path.exists(port):
            for pattern in ('/dev/serial/by-id/*CP210*', '/dev/serial/by-id/*CH340*',
                            '/dev/serial/by-id/*1a86_USB_Serial*', '/dev/serial/by-id/*Silicon_Labs*',
                            '/dev/serial/by-id/*Espressif*', '/dev/serial/by-id/*USB-Serial*',
                            '/dev/esp32'):
                matches = sorted(glob.glob(pattern))
                if matches:
                    self.get_logger().info(f'Port {port} not found; auto-detected ESP32 at {matches[0]}')
                    port = matches[0]
                    break

        self.last_servo_deg = 90
        self.lock = threading.Lock()
        self.running = True
        self.esp32 = Esp32Motors(
            port=port,
            baudrate=115200,
            timeout=0.1,
            auto_connect=True,
            on_error=self.on_esp32_error,
            on_watchdog=self.on_esp32_watchdog,
            on_connect=self.on_esp32_connect,
            on_disconnect=self.on_esp32_disconnect,
            protocol=str(g('esp32_protocol')),
            hold_ms=int(g('esp32_hold_ms')),
        )
        self.get_logger().info(f'ESP32 protocol {self.esp32.protocol} (hold {self.esp32.hold_ms} ms)')
        self.left_pwm = 0          # target
        self.right_pwm = 0         # target
        self.cur_l = 0.0           # ramped value actually sent
        self.cur_r = 0.0
        self.last_cmd = self.get_clock().now()
        self.last_sent = None
        self.dir_l = self.dir_r = 0         # physical drive direction of each side at the last command (+1 fwd, -1 back)
        self.surface = 'FLAT'
        self.override = None                # None = automatic
        self.pitch = 0.0
        self.pitch0 = None
        self.pitch_boot = []
        self.ramp_armed = False
        self.last_imu = None
        self.vib_buf = []
        self.cmd_lin = self.cmd_ang = 0.0   # last cmd_vel (wheel PWM is computed in tick so feedback can act)
        self.pwm_test = None                # (left, right, stamp) from /motor/pwm_test
        self.gyro_z, self.last_gyro = 0.0, None
        self.yaw_i = 0.0
        self.speed_scale = 1.0

        self.enc_pub = self.create_publisher(Int32MultiArray, 'encoders', 10)
        self.pwm_pub = self.create_publisher(Int32MultiArray, '/motor/pwm_state', 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(Int32MultiArray, '/motor/pwm_test', self.on_pwm_test, 10)
        self.create_subscription(Int32MultiArray, '/motor/dash_pwm', self.on_dash, 10)
        self.create_subscription(Float32, '/motor/speed_scale', self.on_speed_scale, 10)
        self.create_subscription(Imu, g('gyro_topic'), self.on_gyro,
                                 QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                            history=HistoryPolicy.KEEP_LAST, depth=5))
        self.add_on_set_parameters_callback(self.on_params)
        imu_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(Imu, g('imu_topic'), self.on_imu, imu_qos)
        self.create_subscription(String, 'surface_override', self.on_override, 10)
        self.surface_pub = self.create_publisher(String, 'surface_state', 1)
        self.servo_pub = self.create_publisher(String, '/motor/servo_state', 10)
        self.create_subscription(Int32MultiArray, '/motor/servo_cmd', self.on_servo_cmd, 10)
        self.create_timer(self.dt, self.tick)

        self.get_logger().info(
            f'Bridge ready. port={port}, max_pwm={self.max_pwm}, cmd_timeout={self.cmd_timeout}s, '
            f'profiles={self.profiles}, ramp_decel={self.ramp_decel}, send_rate={send_rate}Hz, '
            f'deadband L/R={self.min_l}/{self.min_r}, min_turn_pwm={self.min_turn_pwm}, max_speed={self.max_speed}, '
            f'yaw_feedback={self.yaw_feedback}, speed_scale={self.speed_scale_enabled}')

    def on_esp32_error(self, err_msg):
        self.get_logger().warn(f'ESP32 {err_msg}', throttle_duration_sec=1.0)

    def on_esp32_watchdog(self):
        self.get_logger().warn('ESP32 WATCHDOG STOP: firmware cut motor power', throttle_duration_sec=1.0)

    def esp32_port(self):
        # the first connect happens inside Esp32Motors(...), before self.esp32 exists
        return self.esp32.port if hasattr(self, 'esp32') else self.get_parameter('port').value

    def on_esp32_connect(self):
        self.get_logger().info(f'ESP32 connected successfully on {self.esp32_port()}')

    def on_esp32_disconnect(self):
        self.get_logger().error(f'ESP32 disconnected from {self.esp32_port()}; reconnecting every 1s...',
                                throttle_duration_sec=2.0)

    # ------------------------------------------------------------- calibration / live params
    def load_calib(self, path):
        if not os.path.isfile(path):
            self.get_logger().info(f'no calib file {path}: using params')
            return
        try:
            c = yaml.safe_load(open(path)) or {}
            c = c.get('calib', c)
            v = lambda k: c[k]['value'] if isinstance(c.get(k), dict) else c.get(k)
            if v('min_pwm_left') is not None:
                self.min_l = int(v('min_pwm_left'))
            if v('min_pwm_right') is not None:
                self.min_r = int(v('min_pwm_right'))
            if v('min_turn_pwm') is not None:
                self.min_turn_pwm = int(v('min_turn_pwm'))
            if v('max_speed') is not None:
                self.max_speed = float(v('max_speed'))
            if v('turn_gain') is not None:
                self.turn_gain = float(v('turn_gain'))
            self.get_logger().info(f'calib {path}: L/R {self.min_l}/{self.min_r}, turn {self.min_turn_pwm}, '
                                   f'max_speed {self.max_speed}, turn_gain {self.turn_gain}')
        except Exception as e:                       # a broken file must never stop the motors working
            self.get_logger().error(f'calib file {path} unreadable ({e}): using params')

    def on_params(self, params):
        for p in params:
            if p.name == 'min_pwm_left':
                self.min_l = max(0, min(200, int(p.value)))
            elif p.name == 'min_pwm_right':
                self.min_r = max(0, min(200, int(p.value)))
            elif p.name == 'min_turn_pwm':
                self.min_turn_pwm = max(0, min(255, int(p.value)))
            elif p.name == 'max_speed' and float(p.value) > 0.05:
                self.max_speed = float(p.value)
            elif p.name == 'turn_gain' and float(p.value) > 0.01:   # wheel m/s per rad/s: tune turning live
                self.turn_gain = float(p.value)
            elif p.name == 'allow_pwm_test':
                self.allow_pwm_test = bool(p.value)
                self.pwm_test = None
            elif p.name == 'yaw_feedback':
                self.yaw_feedback, self.yaw_i = bool(p.value), 0.0
            elif p.name == 'yaw_kp':
                self.yaw_kp = float(p.value)
            elif p.name == 'yaw_ki':
                self.yaw_ki = float(p.value)
            elif p.name == 'speed_scale_enabled':
                self.speed_scale_enabled, self.speed_scale = bool(p.value), 1.0
            elif p.name == 'dash_input':
                self.dash_input, self.dash = bool(p.value), None
            else:
                continue
            self.get_logger().info(f'param {p.name} = {p.value}')
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------- conversions
    def to_pwm(self, v, invert, min_pwm=0):
        p = v / self.max_speed * self.max_pwm
        p = max(-self.max_pwm, min(self.max_pwm, p))
        p = deadband(int(round(p)), min_pwm, self.max_pwm)
        return -p if invert else p

    def ramp(self, cur, target):
        # Reversing direction: go through zero first
        if cur != 0 and target != 0 and (cur > 0) != (target > 0):
            target = 0
        slowing = abs(target) < abs(cur)
        rate = self.ramp_decel if slowing else self.profiles[self.surface][1]
        if rate <= 0:                      # 0 = no limit
            return float(target)
        step = rate * self.dt
        return cur + max(-step, min(step, target - cur))

    def on_cmd_vel(self, m):
        self.cmd_lin, self.cmd_ang = m.linear.x, m.angular.z
        self.last_cmd = self.get_clock().now()
        self.pwm_test = None                    # a real command always wins over a test PWM

    def on_pwm_test(self, m):
        if not self.allow_pwm_test or len(m.data) != 2:
            return
        l, r = (max(-self.max_pwm, min(self.max_pwm, int(v))) for v in m.data)
        self.pwm_test = (l, r, self.get_clock().now())

    def on_dash(self, m):
        """Ramp dash PWM: written to the Pico at once (rate-limited), bypassing every cap and filter."""
        if not self.dash_input or len(m.data) != 2:
            if len(m.data) == 2 and any(m.data):
                self.get_logger().warn('dash PWM ignored: dash_input is false', throttle_duration_sec=5.0)
            return
        l, r = (max(-self.max_pwm, min(self.max_pwm, int(v))) for v in m.data)
        now = self.get_clock().now()
        self.dash = (l, r, now)
        t = now.nanoseconds * 1e-9
        if t - self.dash_written >= self.dash_min_dt or (l, r) == (0, 0):
            self.dash_written = t
            self.send_dash(l, r)

    def dash_fresh(self):
        return self.dash is not None and \
            (self.get_clock().now() - self.dash[2]).nanoseconds * 1e-9 <= self.cmd_timeout

    def send_dash(self, l, r):
        sl, sr = self.to_sent(l, r)
        self.dir_l, self.dir_r = (l > 0) - (l < 0), (r > 0) - (r < 0)
        self.cur_l, self.cur_r = float(sl), float(sr)      # the normal ramp continues from here after the dash
        self.esp32.drive(sl, sr)
        self.pwm_pub.publish(Int32MultiArray(data=[l, r]))

    def on_speed_scale(self, m):
        self.speed_scale = max(0.5, min(2.0, float(m.data)))

    def on_gyro(self, m):
        self.gyro_z, self.last_gyro = m.angular_velocity.z, self.get_clock().now()

    def gyro_fresh(self):
        return self.last_gyro is not None and \
            (self.get_clock().now() - self.last_gyro).nanoseconds * 1e-9 < self.imu_timeout

    def wheel_targets(self):
        """cmd_vel -> signed PWM targets (left, right) as sent to the Pico; sets dir_l/dir_r (+1 fwd)."""
        lin, ang = self.cmd_lin, self.cmd_ang
        if self.speed_scale_enabled:
            lin *= self.speed_scale
        if abs(lin) < 0.03 and 0.05 < abs(ang) < self.min_turn:   # in-place spin below the turning threshold
            ang = math.copysign(self.min_turn, ang)
        if self.yaw_feedback and self.gyro_fresh() and (abs(ang) > 0.02 or abs(lin) > 0.03):
            err = ang - self.gyro_z                                 # + = turning too slowly to the left
            self.yaw_i = max(-self.yaw_i_max, min(self.yaw_i_max, self.yaw_i + self.yaw_ki * err * self.dt))
            ang += self.yaw_kp * err + self.yaw_i
        else:
            self.yaw_i = 0.0
        pl, pr = lin - ang * self.turn_gain, lin + ang * self.turn_gain     # physical wheel speeds, + = forward
        self.dir_l = (pl > 1e-6) - (pl < -1e-6)
        self.dir_r = (pr > 1e-6) - (pr < -1e-6)
        lp = self.to_pwm(pl, False, self.min_l)        # deadband per PHYSICAL side (what calibration measures)
        rp = self.to_pwm(pr, False, self.min_r)
        if self.min_turn_pwm > 0 and abs(self.cmd_lin) < 0.03 and abs(self.cmd_ang) > 0.05:
            lp = int(math.copysign(max(abs(lp), self.min_turn_pwm), lp)) if lp else 0
            rp = int(math.copysign(max(abs(rp), self.min_turn_pwm), rp)) if rp else 0
        return self.to_sent(lp, rp)

    def to_sent(self, pl, pr):
        """Physical PWM (+ = forward) -> Pico channels. Same as the original mapping: invert_linear flipped only
        the linear part, i.e. the channels are the swapped, negated physical sides; then per-side inverts."""
        sl, sr = (-pr, -pl) if self.invert_linear else (pl, pr)
        return (-sl if self.invert_left else sl), (-sr if self.invert_right else sr)

    def physical(self, sl, sr):
        """Inverse of to_sent, for /motor/pwm_state ([left, right], + = forward)."""
        sl, sr = (-sl if self.invert_left else sl), (-sr if self.invert_right else sr)
        return [-sr, -sl] if self.invert_linear else [sl, sr]

    def test_targets(self):
        """/motor/pwm_test [left, right] (+ = forward, raw, no deadband) -> Pico channels."""
        l, r, _ = self.pwm_test
        self.dir_l, self.dir_r = (l > 0) - (l < 0), (r > 0) - (r < 0)
        return self.to_sent(l, r)

    # ------------------------------------------------------- surface and slope
    def on_override(self, m):
        v = m.data.strip().upper()
        if v == 'AUTO':
            self.override = None
        elif v in SURFACES:
            self.override = v
        else:
            self.get_logger().warn(f'Ignoring surface_override {m.data!r}')

    def on_imu(self, m):
        q = m.orientation
        if m.orientation_covariance[0] < 0:
            return
        self.pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
        self.last_imu = self.get_clock().now()
        if self.pitch0 is None:                       # the first 50 samples define "flat"
            self.pitch_boot.append(self.pitch)
            if len(self.pitch_boot) >= 50:
                self.pitch0 = sum(self.pitch_boot) / len(self.pitch_boot)
            return
        rel = abs(self.pitch - self.pitch0)
        if self.ramp_armed and rel < self.release:
            self.ramp_armed = False
        elif not self.ramp_armed and rel > self.arm:
            self.ramp_armed = True
        if self.vib_enabled:
            self.vib_buf.append(math.hypot(m.angular_velocity.x, m.angular_velocity.y))
            self.vib_buf = self.vib_buf[-100:]

    def imu_fresh(self):
        return (self.pitch0 is not None and self.last_imu is not None and
                (self.get_clock().now() - self.last_imu).nanoseconds * 1e-9 < self.imu_timeout)

    def update_surface(self):
        if self.override:
            new = self.override
        elif not self.imu_fresh():
            new = 'FLAT'
        elif self.ramp_armed:
            new = self.ramp_surface
        elif self.vib_enabled and len(self.vib_buf) >= 50 and self.vib_rms() > self.vib_std:
            new = 'GRAVEL'
        else:
            new = 'FLAT'
        if new != self.surface:
            self.get_logger().info(f'Surface {self.surface} -> {new}')
            self.surface = new
            self.surface_pub.publish(String(data=new))

    def vib_rms(self):
        mean = sum(self.vib_buf) / len(self.vib_buf)
        return math.sqrt(sum((x - mean) ** 2 for x in self.vib_buf) / len(self.vib_buf))

    def feedforward(self):
        """PWM magnitude change for straight driving on a slope (0 if not armed, turning in place or stopped)."""
        if not (self.ramp_armed and self.imu_fresh()) or self.dir_l == 0 or self.dir_l != self.dir_r:
            return 0.0
        slope_up = self.pitch_up_sign * (self.pitch - self.pitch0)       # + when the nose is up
        ff = self.dir_l * self.gravity_ff * math.sin(slope_up)           # more power forward-uphill, less downhill
        return max(-self.gravity_ff_max, min(self.gravity_ff_max, ff))

    def shape(self, target, ff, cap):
        """Apply the feedforward to a wheel's PWM magnitude, then the surface cap; never flips the sign."""
        if target == 0:
            return 0
        mag = max(0.0, min(float(cap), abs(target) + ff))
        return int(math.copysign(round(mag), target))

    # ----------------------------------------------------------------- serial
    def write(self, text):
        """Legacy helper for direct text sending."""
        if hasattr(self, 'esp32') and self.esp32:
            self.esp32._send_command(text.strip())

    def tick(self):
        self.update_surface()
        if self.dash_input and self.dash_fresh():       # dash has top priority; resend keeps the watchdog fed
            self.send_dash(self.dash[0], self.dash[1])
            return
        now = self.get_clock().now()
        age = (now - self.last_cmd).nanoseconds * 1e-9
        if self.pwm_test is not None and self.allow_pwm_test and \
                (now - self.pwm_test[2]).nanoseconds * 1e-9 <= self.cmd_timeout:
            tl, tr = self.test_targets()             # calibration: raw PWM, no feedforward/cap/feedback
        elif age > self.cmd_timeout:
            tl = tr = 0
            self.yaw_i = 0.0
        else:
            self.left_pwm, self.right_pwm = self.wheel_targets()
            ff = self.feedforward()
            cap = self.profiles[self.surface][0]
            tl, tr = self.shape(self.left_pwm, ff, cap), self.shape(self.right_pwm, ff, cap)

        self.cur_l = self.ramp(self.cur_l, tl)
        self.cur_r = self.ramp(self.cur_r, tr)
        l = int(round(self.cur_l))
        r = int(round(self.cur_r))

        if (l, r) != self.last_sent:
            self.get_logger().info(f'left={l} right={r}')
            self.last_sent = (l, r)

        if l == 0 and r == 0:
            self.esp32.stop()
        else:
            self.esp32.drive(l, r)

        self.pwm_pub.publish(Int32MultiArray(data=self.physical(l, r)))

    def on_servo_cmd(self, m):
        """[deg] or [deg, dps] -> 'S <deg>' (firmware: deg 0..180)."""
        if not m.data:
            return
        deg = max(0, min(180, int(m.data[0])))
        dps = int(m.data[1]) if len(m.data) >= 2 and m.data[1] > 0 else 240
        self.esp32.servos(deg)
        delta = abs(deg - self.last_servo_deg)
        slew_s = (delta / float(dps) if dps > 0 else 0.0) + 0.15
        self.last_servo_deg = deg
        self.servo_pub.publish(String(data=f'moving,{deg}'))
        threading.Timer(slew_s, lambda: self.servo_pub.publish(String(data=f'done,{deg}'))).start()
        self.get_logger().info(f'servo -> S {deg} (slew {slew_s:.2f}s)')

    def shutdown(self):
        self.running = False
        try:
            self.esp32.stop()
            self.esp32.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = MotorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

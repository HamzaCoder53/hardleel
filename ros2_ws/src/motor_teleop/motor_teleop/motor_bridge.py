#!/usr/bin/env python3
"""cmd_vel -> Pico serial motor bridge (for the open-loop motor test firmware).

Subscribes : cmd_vel (geometry_msgs/Twist)   e.g. from teleop_twist_keyboard
Publishes  : encoders (std_msgs/Int32MultiArray)
             data = [lf, lr, rf, rr, v_lf, v_lr, v_rf, v_rr]   (ticks, ticks/s)

Pico protocol used (newline terminated):
  STBY,0,1        enable both TB6612 drivers
  M,<id>,<pwm>    id 0=LF 1=LR 2=RF 3=RR, pwm -255..255
  STOP            all motors off
Pico replies read here: ENC,... / STAT,... / ERR,... / WD,stopped

Ramping: the PWM sent to the Pico changes at most <surface>_ramp (speeding up) or
ramp_decel (slowing down) PWM units per second. A direction reversal passes
through zero first. Set a ramp parameter to 0 to disable that limit.

Surface profiles (max PWM cap / speed-up ramp in PWM per second): FLAT 220/600, SAND 150/250,
GRAVEL 180/300. The profile only caps and slews the PWM; the m/s -> PWM scale stays max_speed/max_pwm.
Surface state: FLAT, or ramp_surface (default GRAVEL) while the relative pitch from /imu/data_lpf is armed
(> arm_deg on, < release_deg off); optional IMU vibration also selects GRAVEL. /surface_override
(std_msgs/String: FLAT | SAND | GRAVEL | AUTO) lets the mission set the surface by map zone.
Gravity feedforward: while armed and both wheels drive the same physical way, the PWM magnitude is raised
by gravity_ff * sin(uphill slope) (lowered when going downhill), clamped to +/-gravity_ff_max, never
reversing a wheel and never applied at standstill.
"""
import math
import threading

import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32MultiArray, String

SURFACES = ('FLAT', 'SAND', 'GRAVEL')


class MotorBridge(Node):
    def __init__(self):
        super().__init__('motor_bridge')
        self.declare_parameter('port', '/dev/pico')
        self.declare_parameter('max_speed', 1.0)     # Twist value (m/s) that maps to max_pwm
        self.declare_parameter('turn_gain', 0.5)     # wheel speed per rad/s of angular.z
        self.declare_parameter('max_pwm', 255)       # hard cap, 0..255
        self.declare_parameter('min_pwm', 0)         # lift small non-zero commands above stall (0 = allow full fine control down to 0)
        self.declare_parameter('min_turn', 0.0)      # rad/s: smallest in-place spin threshold (0 = no forced floor)
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)
        self.declare_parameter('invert_linear', True)   # flips forward/backward only; turning is unchanged
        self.declare_parameter('send_rate', 50.0)    # Hz, also keeps the Pico watchdog fed
        self.declare_parameter('cmd_timeout', 0.4)   # s without cmd_vel -> stop
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

        g = lambda n: self.get_parameter(n).value
        self.max_speed = float(g('max_speed'))
        self.turn_gain = float(g('turn_gain'))
        self.max_pwm = max(0, min(255, int(g('max_pwm'))))
        self.min_pwm = max(0, min(255, int(g('min_pwm'))))
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

        try:
            self.ser = serial.Serial(port, 115200, timeout=0.1)   # baud ignored on USB-CDC
        except serial.SerialException as e:
            self.get_logger().fatal(
                f'Cannot open {port}: {e}. Check the cable, that the port is not '
                f'open in screen/minicom, and that you are in the "dialout" group.')
            raise

        self.lock = threading.Lock()
        self.running = True
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

        self.enc_pub = self.create_publisher(Int32MultiArray, 'encoders', 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        imu_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(Imu, g('imu_topic'), self.on_imu, imu_qos)
        self.create_subscription(String, 'surface_override', self.on_override, 10)
        self.surface_pub = self.create_publisher(String, 'surface_state', 1)
        self.create_timer(self.dt, self.tick)

        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()
        self.get_logger().info(
            f'Connected to {port}. max_pwm={self.max_pwm}, cmd_timeout={self.cmd_timeout}s, '
            f'profiles={self.profiles}, ramp_decel={self.ramp_decel}, send_rate={send_rate}Hz')

    # ------------------------------------------------------------- conversions
    def to_pwm(self, v, invert):
        p = v / self.max_speed * self.max_pwm
        p = max(-self.max_pwm, min(self.max_pwm, p))
        p = int(round(p))
        if p != 0 and abs(p) < self.min_pwm:
            p = int(math.copysign(self.min_pwm, p))
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
        lin, ang = m.linear.x, m.angular.z
        if abs(lin) < 0.03 and 0.05 < abs(ang) < self.min_turn:   # in-place spin below the turning threshold
            ang = math.copysign(self.min_turn, ang)
        pl, pr = lin - ang * self.turn_gain, lin + ang * self.turn_gain     # physical wheel speeds, + = forward
        self.dir_l = (pl > 1e-6) - (pl < -1e-6)
        self.dir_r = (pr > 1e-6) - (pr < -1e-6)
        if self.invert_linear:
            lin = -lin
        left = lin - ang * self.turn_gain       # +angular.z = turn left (CCW)
        right = lin + ang * self.turn_gain
        self.left_pwm = self.to_pwm(left, self.invert_left)
        self.right_pwm = self.to_pwm(right, self.invert_right)
        self.last_cmd = self.get_clock().now()

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
        try:
            with self.lock:
                self.ser.write(text.encode())
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Serial write failed: {e}', throttle_duration_sec=2.0)

    def tick(self):
        self.update_surface()
        age = (self.get_clock().now() - self.last_cmd).nanoseconds * 1e-9
        if age > self.cmd_timeout:
            tl = tr = 0
        else:
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

        # STBY is resent every tick so a rebooted Pico recovers by itself
        self.write(f'STBY,0,1\nM,0,{l}\nM,1,{l}\nM,2,{r}\nM,3,{r}\n')

    def read_loop(self):
        while self.running and rclpy.ok():
            try:
                line = self.ser.readline().decode(errors='ignore').strip()
            except (serial.SerialException, OSError) as e:
                if self.running:
                    self.get_logger().error(f'Serial read failed: {e}')
                break
            if line.startswith('ENC,'):
                try:
                    v = [int(x) for x in line.split(',')[1:]]
                except ValueError:
                    continue
                if len(v) == 9:                      # t_ms + 4 counts + 4 velocities
                    self.enc_pub.publish(Int32MultiArray(data=v[1:]))
            elif line.startswith('ERR,'):
                self.get_logger().warn(f'Pico: {line}', throttle_duration_sec=1.0)
            elif line.startswith('WD,'):
                self.get_logger().warn('Pico watchdog stopped the motors', throttle_duration_sec=1.0)

    def shutdown(self):
        self.running = False
        self.write('STOP\nSTBY,0,0\n')
        self.thread.join(timeout=0.5)
        try:
            self.ser.close()
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

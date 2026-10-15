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

Ramping: the PWM sent to the Pico changes at most ramp_accel (speeding up) or
ramp_decel (slowing down) PWM units per second. A direction reversal passes
through zero first. Set a ramp parameter to 0 to disable that limit.
"""
import math
import threading

import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray


class MotorBridge(Node):
    def __init__(self):
        super().__init__('motor_bridge')
        self.declare_parameter('port', '/dev/pico')
        self.declare_parameter('max_speed', 1.0)     # Twist value (m/s) that maps to max_pwm
        self.declare_parameter('turn_gain', 0.5)     # wheel speed per rad/s of angular.z
        self.declare_parameter('max_pwm', 255)       # hard cap, 0..255
        self.declare_parameter('min_pwm', 70)         # lift small non-zero commands above stall
        self.declare_parameter('min_turn', 1.2)      # rad/s: smallest in-place spin that actually turns the robot
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)
        self.declare_parameter('invert_linear', True)   # flips forward/backward only; turning is unchanged
        self.declare_parameter('send_rate', 10.0)    # Hz, also keeps the Pico watchdog fed
        self.declare_parameter('cmd_timeout', 0.4)   # s without cmd_vel -> stop
        self.declare_parameter('ramp_accel', 600.0)  # PWM units/s when speeding up (0 = no limit)
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
        self.ramp_accel = float(g('ramp_accel'))
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

        self.enc_pub = self.create_publisher(Int32MultiArray, 'encoders', 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_timer(self.dt, self.tick)

        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()
        self.get_logger().info(
            f'Connected to {port}. max_pwm={self.max_pwm}, cmd_timeout={self.cmd_timeout}s, '
            f'ramp_accel={self.ramp_accel}, ramp_decel={self.ramp_decel}, send_rate={send_rate}Hz')

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
        rate = self.ramp_decel if slowing else self.ramp_accel
        if rate <= 0:                      # 0 = no limit
            return float(target)
        step = rate * self.dt
        return cur + max(-step, min(step, target - cur))

    def on_cmd_vel(self, m):
        lin, ang = m.linear.x, m.angular.z
        if abs(lin) < 0.03 and 0.05 < abs(ang) < self.min_turn:   # in-place spin below the turning threshold
            ang = math.copysign(self.min_turn, ang)
        if self.invert_linear:
            lin = -lin
        left = lin - ang * self.turn_gain       # +angular.z = turn left (CCW)
        right = lin + ang * self.turn_gain
        self.left_pwm = self.to_pwm(left, self.invert_left)
        self.right_pwm = self.to_pwm(right, self.invert_right)
        self.last_cmd = self.get_clock().now()

    # ----------------------------------------------------------------- serial
    def write(self, text):
        try:
            with self.lock:
                self.ser.write(text.encode())
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Serial write failed: {e}', throttle_duration_sec=2.0)

    def tick(self):
        age = (self.get_clock().now() - self.last_cmd).nanoseconds * 1e-9
        if age > self.cmd_timeout:
            tl = tr = 0
        else:
            tl, tr = self.left_pwm, self.right_pwm

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

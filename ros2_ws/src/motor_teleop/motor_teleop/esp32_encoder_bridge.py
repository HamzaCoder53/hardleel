#!/usr/bin/env python3
"""cmd_vel -> ESP32 encoder motor controller (firmware/esp32_encoder_motors), closed-loop on the encoders.

Hardware (22 Nov): 4x 6 V 210 RPM gear motors with quadrature encoders, 2S battery, 2x BTS7960. The firmware
caps the motor voltage at 6 V, ramps, and stops by itself 500 ms after the last command.
  Pi -> ESP32   M,<left>,<right>  (-255..255, 255 = the 6 V ceiling)   S = stop   R = reset counts   I = info
  ESP32 -> Pi   E,<ms>,<cFL>,<cRL>,<cFR>,<cRR>,<rpmFL>,<rpmRL>,<rpmFR>,<rpmRR>,<battV>,<flags>  at 20 Hz
                '#' lines = info

Per side: command = feedforward(target speed) + PI(target - measured speed), sent at send_rate. With the
geared encoder motors slow speeds work - no PWM-70 floor or turning boost (those were for the old 1000 RPM
motors). Odometry from the encoder counts on /wheel/odom (no TF: the EKF owns odom->base_link): forward
speed trusted, yaw rate loose (skid steer slips; the IMU owns yaw). All parameters except port are live.
"""
import math
import threading
import time

import rclpy
import serial
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray

# name: (default, description, (min, max) or None)
PARAMS = {
    'port': ('', 'ESP32 serial port (start only; pi_mapping.launch.py passes the detected one)', None),
    'wheel_diameter': (0.0804, 'wheel diameter m (24 Nov: 2 m straight run, real 2.02 m vs encoders 2.009 m)', (0.01, 0.5)),
    'track_width': (0.52, 'EFFECTIVE left-right wheel distance m (24 Nov: skid steer slip depends on speed - 0.52 gives 77-116% of the asked turn rate at 0.6-1.2 rad/s)',
                    (0.05, 1.0)),
    'counts_per_rev': (748.0, 'encoder counts per WHEEL turn (firmware: 11 PPR x 34 gear x 2 edges)',
                       (1.0, 100000.0)),
    'max_rpm': (210.0, 'wheel RPM at command 255 (6 V, no load) - feedforward scale', (10.0, 2000.0)),
    'max_speed': (0.45, 'wheel speed limit m/s', (0.02, 3.0)),
    'max_accel': (0.0, 'wheel speed ramp when speeding up m/s^2 (0 = off: only the firmware ramp, user 22 Nov)', (0.0, 20.0)),
    'swap_motor_sides': (True, 'send the left command to the firmware RIGHT output and vice versa (22 Nov: motor outputs wired to the opposite sides of the encoders; turns went the wrong way)', None),
    'max_decel': (0.0, 'wheel speed ramp when slowing down / stopping m/s^2 (0 = off: only the firmware ramp)', (0.0, 20.0)),
    'kp': (150.0, 'PI: command per m/s of speed error', (0.0, 5000.0)),
    'ki': (400.0, 'PI: command per (m/s * s) of accumulated error', (0.0, 20000.0)),
    'i_limit': (175.0, 'PI: integral term limit (command units; 24 Nov 80 -> 175: turns on the spot reached only 25-50%)', (0.0, 255.0)),
    'min_cmd': (0.0, 'smallest non-zero command (static friction kick); 0 = off', (0.0, 255.0)),
    'cmd_timeout': (0.6, 's without cmd_vel before the target goes to 0 (just over the ~0.5 s keyboard repeat delay)', (0.05, 5.0)),
    'send_rate': (20.0, 'Hz control loop / M commands (firmware stops after 500 ms of silence)', (5.0, 50.0)),
    # encoder-only stack (pi_encoder_mapping.launch.py, no IMU/EKF): publish odom->base_link + /odom for Nav2
    'publish_tf': (False, 'broadcast odom->base_link from the encoders (start only; only without an EKF)', None),
    'odom_topic': ('/wheel/odom', 'odometry topic (start only; /odom when this bridge is the only odometry)', None),
    'battery_voltage': (0.0, 'battery V measured at the start of the run -> ESP32 PWM ceiling exactly 6 V '
                             '(needs firmware with the V command). 0 = not set: firmware assumes 8.4 V', (0.0, 8.6)),
    'encoder_mode': ('max', 'per side from the front/rear encoder: max (larger reading - ignores a loose encoder), '
                            'mean, front, rear', None),
    'stall_cmd': (180.0, 'stall warning: |command| above this ...', (0.0, 255.0)),
    'stall_time': (1.0, '... for this long (s) with the side not turning', (0.2, 10.0)),
}


def parse_telemetry(line):
    """'E,ms,cFL,cRL,cFR,cRR,rFL,rRL,rFR,rRR,batt,flags' -> dict, or None."""
    p = line.strip().split(',')
    if len(p) != 12 or p[0] != 'E':
        return None
    try:
        return {'ms': int(p[1]), 'counts': [int(x) for x in p[2:6]], 'rpm': [float(x) for x in p[6:10]],
                'batt': float(p[10]), 'flags': int(p[11])}
    except ValueError:
        return None


def side_targets(v, w, track):
    """Body speed (m/s, rad/s) -> (left, right) wheel speeds m/s."""
    return v - w * track / 2.0, v + w * track / 2.0


ENCODER_MODES = ('max', 'mean', 'front', 'rear')


def side_value(a, b, mode='max'):
    """One side's two encoders (front a, rear b) -> one value.
    max   (default, 23 Nov): the larger reading. A loose encoder only MISSES pulses (reads low, never high) and
          both wheels of a side get the same command, so the larger one is the healthy one - also for partial
          dropouts (23 Nov test: RR 35 rpm while FR 88). Opposite signs (noise around 0): the mean.
    mean  the average, except a reading near 0 while the other clearly moves (dead encoder): the moving one.
    front / rear  use only that encoder."""
    if mode == 'front':
        return a
    if mode == 'rear':
        return b
    if mode == 'max':
        if a * b >= 0 or min(abs(a), abs(b)) < 0.25 * max(abs(a), abs(b)):   # same sign, or one ~0 (dead)
            return a if abs(a) >= abs(b) else b
        return (a + b) / 2.0
    if abs(a) < 0.25 * abs(b):
        return b
    if abs(b) < 0.25 * abs(a):
        return a
    return (a + b) / 2.0


class OdomIntegrator:
    """Encoder counts -> 2D pose (per side: mean of front/rear, or the one still working - side_value)."""
    def __init__(self):
        self.x = self.y = self.th = 0.0
        self.last = None

    def update(self, counts, m_per_count, track, mode='max'):
        if self.last is None:
            self.last = list(counts)
            return 0.0, 0.0
        d = [c - p for c, p in zip(counts, self.last)]
        self.last = list(counts)
        dl = side_value(d[0], d[1], mode) * m_per_count
        dr = side_value(d[2], d[3], mode) * m_per_count
        ds, dth = (dl + dr) / 2.0, (dr - dl) / track
        self.x += ds * math.cos(self.th + dth / 2.0)
        self.y += ds * math.sin(self.th + dth / 2.0)
        self.th = math.atan2(math.sin(self.th + dth), math.cos(self.th + dth))
        return ds, dth


class EncoderBridge(Node):
    def __init__(self):
        super().__init__('esp32_encoder_bridge')
        for name, (default, desc, rng) in PARAMS.items():
            self.declare_parameter(name, default, ParameterDescriptor(
                description=desc + (f'  [{rng[0]}..{rng[1]}]' if rng else '')))
        self.p = {k: self.get_parameter(k).value for k in PARAMS}
        self.lock = threading.Lock()
        self.target = [0.0, 0.0]      # wanted wheel speeds (m/s) from cmd_vel
        self.ramped = [0.0, 0.0]      # after the accel ramp
        self.integ = [0.0, 0.0]
        self.meas = [0.0, 0.0]        # measured wheel speeds (m/s)
        self.last_cmd_vel = 0.0
        self.last_tele = 0.0
        self.stall_since = [None, None]
        self.flags = 0
        self.sent_stop = True
        self.odom = OdomIntegrator()
        self.last_odom_t = None

        port = self.p['port'] or '/dev/ttyUSB1'
        self.ser = serial.Serial()
        self.ser.port, self.ser.baudrate, self.ser.timeout = port, 115200, 0.2
        self.ser.dtr = False          # asserted DTR/RTS reset or hold the ESP32
        self.ser.rts = False
        self.ser.open()
        time.sleep(1.0)
        self.send('S')
        self.send_battery()
        self.send('I')
        self.get_logger().info(f'ESP32 encoder controller on {port}' + (
            f', battery {self.p["battery_voltage"]:.2f} V -> 6 V ceiling = {600 / self.p["battery_voltage"]:.0f}% PWM'
            if self.p['battery_voltage'] > 0 else ', battery voltage not set: firmware assumes 8.4 V (71% PWM)'))
        self.create_timer(2.0, self.send_battery)  # resend: survives an ESP32 reset (it forgets the value)

        self.odom_pub = self.create_publisher(Odometry, self.p['odom_topic'], 10)
        self.tf_pub = TransformBroadcaster(self) if self.p['publish_tf'] else None
        self.speed_pub = self.create_publisher(Float32MultiArray, '/wheel/speeds', 10)
        self.batt_pub = self.create_publisher(Float32, '/wheel/battery', 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.add_on_set_parameters_callback(self.on_params)
        self.timer = self.create_timer(1.0 / self.p['send_rate'], self.control)
        threading.Thread(target=self.reader, daemon=True).start()

    # ---- parameters ----------------------------------------------------------------------------
    def on_params(self, params):
        new = {x.name: x.value for x in params}
        for name, value in new.items():
            if name in ('port', 'publish_tf', 'odom_topic'):
                return SetParametersResult(successful=False, reason=f'{name} is read at start only')
            if name == 'battery_voltage' and value != 0.0 and not 6.4 <= value <= 8.6:
                return SetParametersResult(successful=False, reason='battery_voltage must be 0 (not set) or 6.4..8.6')
            if name == 'encoder_mode' and value not in ENCODER_MODES:
                return SetParametersResult(successful=False, reason=f'encoder_mode must be one of {ENCODER_MODES}')
            rng = PARAMS.get(name, (None, None, None))[2]
            if rng and not rng[0] <= value <= rng[1]:
                return SetParametersResult(successful=False, reason=f'{name} must be {rng[0]}..{rng[1]}')
        with self.lock:
            self.p.update({k: v for k, v in new.items() if k in PARAMS})
        if 'battery_voltage' in new:
            self.send_battery()
        if 'send_rate' in new:
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.timer = self.create_timer(1.0 / new['send_rate'], self.control)
        self.get_logger().info('tuned: ' + ', '.join(f'{k}={v}' for k, v in new.items()))
        return SetParametersResult(successful=True)

    def send_battery(self):
        """Tell the ESP32 the battery voltage so its PWM ceiling is exactly 6 V (firmware V command)."""
        v = self.p['battery_voltage']
        if v > 0:
            self.send(f'V,{v:.2f}')

    def m_per_count(self):
        return math.pi * self.p['wheel_diameter'] / self.p['counts_per_rev']

    def v_max(self):
        """Wheel speed m/s at command 255 (feedforward scale)."""
        return self.p['max_rpm'] / 60.0 * math.pi * self.p['wheel_diameter']

    # ---- serial ----------------------------------------------------------------------------------
    def send(self, line):
        try:
            self.ser.write((line + '\n').encode())
        except serial.SerialException as e:
            self.get_logger().error(f'serial write failed: {e}', throttle_duration_sec=2.0)

    def reader(self):
        buf = b''
        while rclpy.ok():
            try:
                buf += self.ser.read(max(1, self.ser.in_waiting))
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f'serial read failed: {e}', throttle_duration_sec=2.0)
                time.sleep(0.5)
                continue
            *lines, buf = buf.split(b'\n')
            for raw in lines:
                line = raw.decode(errors='replace').strip()
                if line.startswith('E,'):
                    self.on_telemetry(line)
                elif line.startswith('#'):
                    self.get_logger().info(f'ESP32 {line}', throttle_duration_sec=1.0)
            if len(buf) > 512:
                buf = b''

    def on_telemetry(self, line):
        t = parse_telemetry(line)
        if t is None:
            return
        now = time.monotonic()
        k = math.pi * self.p['wheel_diameter'] / 60.0
        with self.lock:
            mode = self.p['encoder_mode']
            self.meas = [k * side_value(t['rpm'][0], t['rpm'][1], mode), k * side_value(t['rpm'][2], t['rpm'][3], mode)]
            self.last_tele = now
            if t['flags'] & 1 and not self.flags & 1:
                self.get_logger().error(f'ESP32 LOW BATTERY ({t["batt"]:.2f} V): motors disabled')
            self.flags = t['flags']
            ds, dth = self.odom.update(t['counts'], self.m_per_count(), self.p['track_width'], mode)
            meas = list(self.meas)
        dt = (now - self.last_odom_t) if self.last_odom_t else 0.05
        self.last_odom_t = now
        self.publish_odom(ds / max(dt, 1e-3), dth / max(dt, 1e-3))
        self.speed_pub.publish(Float32MultiArray(data=[float(x) for x in meas + t['rpm']]))
        self.batt_pub.publish(Float32(data=t['batt']))

    def publish_odom(self, vx, wz):
        m = Odometry()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id, m.child_frame_id = 'odom', 'base_link'
        m.pose.pose.position.x, m.pose.pose.position.y = self.odom.x, self.odom.y
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(self.odom.th / 2), math.cos(self.odom.th / 2)
        m.twist.twist.linear.x, m.twist.twist.angular.z = vx, wz
        pc = [0.0] * 36
        pc[0] = pc[7] = 0.05
        pc[35] = 0.5
        pc[14] = pc[21] = pc[28] = 1e6
        m.pose.covariance = pc
        tc = [0.0] * 36
        tc[0] = 0.01 if abs(vx) > 1e-3 else 0.0005   # forward speed: trusted (confident 0 while still)
        tc[7] = 0.001                                # no sideways motion
        tc[35] = 0.5                                 # yaw rate: skid steer slips, the IMU owns yaw
        tc[14] = tc[21] = tc[28] = 1e6
        m.twist.covariance = tc
        self.odom_pub.publish(m)
        if self.tf_pub is not None:                  # encoder-only stack: odom->base_link
            tf = TransformStamped()
            tf.header = m.header
            tf.child_frame_id = 'base_link'
            tf.transform.translation.x, tf.transform.translation.y = self.odom.x, self.odom.y
            tf.transform.rotation = m.pose.pose.orientation
            self.tf_pub.sendTransform(tf)

    # ---- control ---------------------------------------------------------------------------------
    def on_cmd(self, msg):
        vl, vr = side_targets(msg.linear.x, msg.angular.z, self.p['track_width'])
        lim = self.p['max_speed']
        with self.lock:
            self.target = [max(-lim, min(lim, vl)), max(-lim, min(lim, vr))]
            self.last_cmd_vel = time.monotonic()

    def control(self):
        now = time.monotonic()
        dt = 1.0 / self.p['send_rate']
        with self.lock:
            p = dict(self.p)
            fresh = now - self.last_cmd_vel < p['cmd_timeout']
            target = list(self.target) if fresh else [0.0, 0.0]
            meas, tele_ok = list(self.meas), now - self.last_tele < 0.5
        if not tele_ok:
            self.get_logger().warn('no encoder telemetry from the ESP32 (E lines) - closed loop off',
                                   throttle_duration_sec=3.0)
        cmds = []
        for i in range(2):
            r = self.ramped[i]
            slowing = abs(target[i]) < abs(r) or target[i] * r < 0
            a = p['max_decel'] if slowing else p['max_accel']
            step = a * dt
            r = target[i] if a <= 0 or abs(target[i] - r) <= step else r + math.copysign(step, target[i] - r)
            self.ramped[i] = r
            if abs(r) < 1e-4 and abs(target[i]) < 1e-4:
                self.integ[i] = 0.0
                cmds.append(0)
                self.stall_since[i] = None
                continue
            ff = r / self.v_max() * 255.0
            err = (r - meas[i]) if tele_ok else 0.0
            self.integ[i] = max(-p['i_limit'], min(p['i_limit'], self.integ[i] + p['ki'] * err * dt))
            c = ff + p['kp'] * err + self.integ[i]
            if p['min_cmd'] > 0 and 0 < abs(c) < p['min_cmd']:
                c = math.copysign(p['min_cmd'], c)
            c = int(round(max(-255.0, min(255.0, c))))
            cmds.append(c)
            # stall: driven hard but the side does not turn (motor blocked / encoder dead)
            if tele_ok and abs(c) >= p['stall_cmd'] and abs(meas[i]) < 0.02:
                self.stall_since[i] = self.stall_since[i] or now
                if now - self.stall_since[i] > p['stall_time']:
                    self.get_logger().warn(f'{"left" if i == 0 else "right"} side STALLED: command {c} but '
                                           f'{meas[i]:.3f} m/s (blocked motor or no encoder signal?)',
                                           throttle_duration_sec=2.0)
            else:
                self.stall_since[i] = None
        if cmds == [0, 0]:
            if not self.sent_stop:
                self.send('S')
                self.sent_stop = True
            return
        self.sent_stop = False
        if self.p['swap_motor_sides']:
            cmds = cmds[::-1]
        self.send(f'M,{cmds[0]},{cmds[1]}')

    def stop(self):
        for _ in range(3):
            self.send('S')
            time.sleep(0.05)


def main():
    rclpy.init()
    node = EncoderBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop()
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

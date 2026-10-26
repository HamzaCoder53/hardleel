#!/usr/bin/env python3
"""ramp_dash (final stack item 5) - RUNS ON THE PI so the control loop never crosses Wi-Fi. OFF unless launched
(pi_bringup ramp_dash:=true, which also turns on motor_bridge dash_input). Logic: ramp_dash_core.Dash.

Start:  /ramp_dash/start (String: '' or JSON param overrides, e.g. {"profile": "up_only"}) - the mission sends it at
        the foot of the ramp. Abort: /ramp_dash/abort (Empty) -> motors 0 at once.
Motors: /motor/dash_pwm (Int32MultiArray [left, right], physical, + = forward) at IMU rate while a dash runs; the
        bridge writes it straight to the Pico (no Nav2, velocity smoother, collision monitor, surface cap or slew).
Inputs: /scan (raw; side sectors only while tilted), /imu/data_raw (attitude + gyro heading at IMU rate),
        /ramp_dash/camera_clear (Bool, optional low-object check from the laptop), /battery_state (optional).
Output: /ramp_dash/state (String phase), /ramp_dash/result (String JSON when done), /ramp_dash/log
        (Float32MultiArray, Dash.LOG_FIELDS, every IMU sample while active) + ~/logs/ramp_dash_<time>.csv.
Params: every key of ramp_dash_core.DEFAULTS (config/ramp_dash.yaml), scan_topic, imu_topic, log_dir.
"""
import csv
import json
import math
import os
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Imu, LaserScan
from std_msgs.msg import Bool, Empty, Float32MultiArray, Int32MultiArray, String

from robot_slam import ramp_dash_core as core


class RampDash(Node):
    def __init__(self):
        super().__init__('ramp_dash')
        self.cfg = {}
        for k, v in core.DEFAULTS.items():
            val = self.declare_parameter(k, list(v) if isinstance(v, tuple) else v).value
            self.cfg[k] = tuple(val) if isinstance(v, tuple) else val
        self.log_dir = os.path.expanduser(self.declare_parameter('log_dir', '~/logs').value)
        self.dash = core.Dash(self.cfg)
        self.att = core.Attitude(self.dash.c)
        self.geo = core.ScanGeometry(self.dash.c)
        self.battery = None
        self.csv = None
        self.last_phase = 'IDLE'
        self.pub_pwm = self.create_publisher(Int32MultiArray, '/motor/dash_pwm', 10)
        self.pub_state = self.create_publisher(String, '/ramp_dash/state', 10)
        self.pub_result = self.create_publisher(String, '/ramp_dash/result', 10)
        self.pub_log = self.create_publisher(Float32MultiArray, '/ramp_dash/log', 50)
        self.create_subscription(LaserScan, self.declare_parameter('scan_topic', '/scan').value, self.on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Imu, self.declare_parameter('imu_topic', '/imu/data_raw').value, self.on_imu,
                                 qos_profile_sensor_data)
        self.create_subscription(String, '/ramp_dash/start', self.on_start, 10)
        self.create_subscription(Empty, '/ramp_dash/abort', self.on_abort, 10)
        self.create_subscription(Bool, '/ramp_dash/camera_clear', lambda m: setattr(self.dash, 'camera_clear', m.data), 10)
        self.create_subscription(BatteryState, '/battery_state', lambda m: setattr(self, 'battery', m.voltage), 10)
        self.create_timer(1.0, lambda: self.pub_state.publish(String(data=self.dash.phase)))
        self.get_logger().info('ramp_dash ready (idle): /ramp_dash/start to run')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def active(self):
        return self.dash.phase not in ('IDLE', 'DONE', 'FAILED')

    def on_scan(self, m):
        tilted = self.att.pitch is not None and abs(self.att.pitch) > math.radians(self.dash.c['tilt_deg'])
        self.dash.scan(self.geo(m.ranges, m.angle_min, m.angle_increment, tilted))
        if self.active():
            self.dash.scan_speed(self.now())

    def on_imu(self, m):
        t = self.now()
        a, w = m.linear_acceleration, m.angular_velocity
        self.att.update(t, a.x, a.y, a.z, w.y, w.z)
        if not self.active():
            return
        left, right = self.dash.step(t, self.att)
        self.pub_pwm.publish(Int32MultiArray(data=[int(left), int(right)]))
        row = self.dash.log_row(t, self.att)
        self.pub_log.publish(Float32MultiArray(data=[float(v) for v in row]))
        if self.csv:
            self.csv[1].writerow([f'{v:.4f}' for v in row])
        self.flush_events()
        if not self.active():
            self.finish()

    def flush_events(self):
        for t, text in self.dash.events:
            self.get_logger().info(f'[dash {t:.2f}] {text}')
        self.dash.events.clear()
        if self.dash.phase != self.last_phase:
            self.last_phase = self.dash.phase
            self.pub_state.publish(String(data=self.dash.phase))

    def on_start(self, m):
        if self.active():
            self.get_logger().warn('dash already running')
            return
        over = {}
        if m.data.strip():
            try:
                over = json.loads(m.data)
            except ValueError:
                self.get_logger().error(f'bad /ramp_dash/start overrides {m.data!r}: ignored')
        self.dash = core.Dash({**self.cfg, **over})
        self.att.c = self.dash.c
        self.geo.c = self.dash.c
        if self.att.pitch is None:
            self.get_logger().error('no IMU yet: dash refused')
            self.pub_result.publish(String(data=json.dumps({'ok': False, 'reason': 'no imu'})))
            return
        os.makedirs(self.log_dir, exist_ok=True)
        path = os.path.join(self.log_dir, time.strftime('ramp_dash_%Y%m%d_%H%M%S.csv'))
        f = open(path, 'w', newline='')
        w = csv.writer(f)
        w.writerow(core.Dash.LOG_FIELDS)
        self.csv = (f, w)
        thr = None
        try:
            thr = subprocess.run(['vcgencmd', 'get_throttled'], capture_output=True, text=True, timeout=1).stdout.strip()
        except (OSError, subprocess.TimeoutExpired):
            pass
        self.get_logger().info(f'DASH START: battery {self.battery if self.battery is not None else "unavailable"} V, '
                               f'{thr or "throttle n/a"}, overrides {over}, log {path}')
        self.dash.start(self.now(), self.att)
        self.flush_events()

    def on_abort(self, _):
        if self.active():
            self.dash.finish(self.now(), False, 'aborted')
            self.finish()

    def finish(self):
        for _ in range(3):
            self.pub_pwm.publish(Int32MultiArray(data=[0, 0]))
        res = self.dash.result or {'ok': False, 'reason': 'stopped'}
        res['battery_v'] = self.battery
        self.pub_result.publish(String(data=json.dumps(res)))
        self.get_logger().info(f'DASH END: {res}')
        self.flush_events()
        if self.csv:
            self.csv[0].close()
            self.csv = None


def main():
    rclpy.init()
    n = RampDash()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        if n.active():
            n.pub_pwm.publish(Int32MultiArray(data=[0, 0]))
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Measure real speed per PWM with the lidar -> pwm_v_lo / pwm_v_hi for /pwm/odom.

SETUP: stop esp32_teleop_bridge (this script needs the ESP32 serial port), keep the lidar running
(/scan). Put the robot on the COMPETITION floor, square to a flat wall 0.8-2.0 m straight ahead,
nothing else in front, and stay with it. Same battery state as in the run if you can.

    ros2 run motor_teleop pwm_calibrate --ros-args -p pwms:=[70,72,75] -p run_s:=1.5

For each PWM it drives FORWARD for run_s (a timed 'm l r ms' command: the ESP32 stops by itself
even if this script dies), fits the speed from the shrinking front distance (first 0.4 s skipped =
spin-up), then drives BACKWARD the same way to return. Abort: any front reading < min_front (0.30 m)
-> 'x'. Prints the table and the parameters to put in the tuning GUI / saved params.
"""
import math
import statistics
import threading
import time

import rclpy
import serial
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from motor_teleop.esp32_teleop_bridge import default_port


def front_range(scan, half_width_deg=5.0):
    """Median range within +-half_width of the lidar's 0 deg (= robot front, yaw 0 confirmed 15 Nov)."""
    vals = []
    for i, r in enumerate(scan.ranges):
        a = scan.angle_min + i * scan.angle_increment
        a = math.atan2(math.sin(a), math.cos(a))
        if abs(math.degrees(a)) <= half_width_deg and scan.range_min < r < scan.range_max and math.isfinite(r):
            vals.append(r)
    return statistics.median(vals) if len(vals) >= 2 else None


def fit_slope(samples):
    """Least-squares d(range)/dt of [(t, r)]."""
    n = len(samples)
    if n < 3:
        return None
    mt = sum(t for t, _ in samples) / n
    mr = sum(r for _, r in samples) / n
    den = sum((t - mt) ** 2 for t, _ in samples)
    return None if den == 0 else sum((t - mt) * (r - mr) for t, r in samples) / den


class Calibrator(Node):
    def __init__(self):
        super().__init__('pwm_calibrate')
        self.declare_parameter('port', default_port())
        self.declare_parameter('pwms', [70, 72, 75])
        self.declare_parameter('run_s', 1.5)
        self.declare_parameter('skip_s', 0.4)
        self.declare_parameter('min_front', 0.30)
        self.declare_parameter('invert_left', True)
        self.declare_parameter('invert_right', False)
        self.samples = []
        self.lock = threading.Lock()
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, scan):
        r = front_range(scan)
        if r is not None:
            with self.lock:
                self.samples.append((time.monotonic(), r))

    def window(self, t0, t1):
        with self.lock:
            return [(t, r) for t, r in self.samples if t0 <= t <= t1]


def main():
    rclpy.init()
    node = Calibrator()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    g = lambda n: node.get_parameter(n).value
    log = node.get_logger()
    pwms = [int(p) for p in g('pwms')]
    run_s, skip_s, min_front = float(g('run_s')), float(g('skip_s')), float(g('min_front'))
    sl, sr = (-1 if g('invert_left') else 1), (-1 if g('invert_right') else 1)

    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = g('port'), 115200, 0.1
    ser.dtr = ser.rts = False
    ser.open()
    time.sleep(1.5)
    send = lambda line: ser.write((line + '\n').encode())
    send('x')

    def drive(pwm):
        """Timed run; returns (speed m/s along the forward axis, aborted)."""
        t0 = time.monotonic()
        send(f'm {sl * pwm} {sr * pwm} {int(run_s * 1000)}')
        aborted = False
        while time.monotonic() - t0 < run_s + 0.1:
            w = node.window(time.monotonic() - 0.3, time.monotonic())
            if pwm > 0 and w and min(r for _, r in w) < min_front:
                send('x')
                aborted = True
                break
            time.sleep(0.02)
        slope = fit_slope(node.window(t0 + skip_s, t0 + run_s))
        time.sleep(1.0)                                  # settle before the next run
        return (None if slope is None else -slope), aborted

    results = []
    try:
        time.sleep(2.0)
        start = node.window(time.monotonic() - 1.0, time.monotonic())
        if not start:
            log.error('no /scan front readings - is the lidar running and a wall straight ahead?')
            return
        d0 = statistics.median(r for _, r in start)
        log.info(f'front wall at {d0:.2f} m')
        if d0 < min_front + 0.4:
            log.error(f'need at least {min_front + 0.4:.2f} m free in front')
            return
        for pwm in pwms:
            vf, ab = drive(pwm)
            if ab:
                log.error('too close to the wall - stopped. Move the robot back and run again.')
                break
            vb, _ = drive(-pwm)
            vb = None if vb is None else -vb
            results.append((pwm, vf, vb))
            log.info(f'PWM {pwm}: forward {vf if vf is None else round(vf, 3)} m/s, '
                     f'backward {vb if vb is None else round(vb, 3)} m/s')
    finally:
        for _ in range(3):
            send('x')
            time.sleep(0.05)
        ser.close()

    if results:
        print('\n PWM | forward m/s | backward m/s | mean')
        good = []
        for pwm, vf, vb in results:
            vs = [v for v in (vf, vb) if v is not None]
            mean = sum(vs) / len(vs) if vs else None
            print(f' {pwm:3d} | {vf if vf is None else f"{vf:.3f}":>11} | {vb if vb is None else f"{vb:.3f}":>12} | '
                  f'{"-" if mean is None else f"{mean:.3f}"}')
            if mean is not None:
                good.append((pwm, mean))
        stalled = [p for p, v in good if v < 0.02]
        moving = [(p, v) for p, v in good if v >= 0.02]
        if len(moving) >= 1:
            lo, hi = moving[0], moving[-1]
            print('\nSet on esp32_teleop_bridge (tuning GUI, then Save):')
            print(f'  pwm_cal_lo: {lo[0]}   pwm_v_lo: {lo[1]:.3f}')
            print(f'  pwm_cal_hi: {hi[0]}   pwm_v_hi: {hi[1]:.3f}')
            print(f'  pwm_stall:  {max(stalled) + 1 if stalled else lo[0]}')
            if len(moving) == 1:
                print('  (only one PWM moved the robot: both points are the same, speed is constant)')
        else:
            print('\nThe robot did not move at any PWM tested (stall) - try higher PWMs / check the battery.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

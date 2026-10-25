#!/usr/bin/env python3
"""motor_calibrator: floor calibration services (robot ON THE FLOOR, clear space around it).

  /motor/calibrate_turn  (std_srvs/Trigger)  in-place spin test -> min_turn_pwm (+ yaw rate report)
  /motor/calibrate_speed (std_srvs/Trigger)  short straight drives + TF -> max_speed

Drives the motors through /motor/pwm_test (allow_pwm_test enabled on /motor_bridge only while a
test runs). All numeric defaults are PLACEHOLDERS to be tuned on the real robot.
"""
import math
import threading
import time
from collections import deque

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import Trigger
import tf2_ros

from motor_teleop import calib_math
from motor_teleop.bridge_params import set_bridge_params
from motor_teleop.calib_store import CalibStore, DEFAULT_PATH


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MotorCalibrator(Node):
    def __init__(self):
        super().__init__('motor_calibrator')
        d = self.declare_parameter
        d('calib_file', DEFAULT_PATH)                 # PLACEHOLDER-free: path
        d('bridge_node', '/motor_bridge')
        d('imu_topic', '/imu/data_raw')
        d('scan_topic', '/scan')
        d('map_frame', 'map')
        d('base_frame', 'base_link')
        d('max_pwm', 255)
        # turn test (PLACEHOLDER defaults)
        d('turn_start_pwm', 30)
        d('turn_step', 5)
        d('turn_hold_s', 0.5)
        d('turn_detect_rad_s', 0.25)
        d('turn_max_pwm', 200)
        d('turn_margin', 5)
        d('turn_ref_offset', 30)          # reference PWM = threshold + this
        d('turn_ref_hold_s', 0.8)
        d('turn_max_angle_deg', 180.0)    # per direction
        d('turn_window_s', 0.3)
        d('turn_consecutive', 2)
        # speed test (PLACEHOLDER defaults)
        d('speed_levels', [100, 160, 220])
        d('speed_drive_s', 1.5)
        d('speed_skip_s', 0.4)
        d('speed_clear_m', 1.0)
        d('speed_cone_deg', 20.0)
        d('speed_max_dist_m', 0.8)
        d('speed_pause_s', 1.0)
        d('speed_min_pwm_fallback', 0)    # used if min_pwm_left/right not in the calib file
        d('lidar_forward_yaw_deg', 0.0)   # direction of robot-forward in the scan frame
        d('force_save', False)

        g = lambda n: self.get_parameter(n).value
        self.store = CalibStore(g('calib_file'))
        self.bridge = g('bridge_node')
        self.lock = threading.Lock()
        self.gyro = deque(maxlen=2000)     # (t_mono, z)
        self.angle = 0.0
        self._last_gt = None
        self.integrating = False
        self.scan = None
        self.busy = threading.Lock()

        cb_sub = MutuallyExclusiveCallbackGroup()
        cb_srv = ReentrantCallbackGroup()
        self.create_subscription(Imu, g('imu_topic'), self.on_imu, qos_profile_sensor_data,
                                 callback_group=cb_sub)
        self.create_subscription(LaserScan, g('scan_topic'), self.on_scan, qos_profile_sensor_data,
                                 callback_group=cb_sub)
        self.pub = self.create_publisher(Int32MultiArray, '/motor/pwm_test', 10)
        self.tfbuf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfbuf, self)
        self.create_service(Trigger, '/motor/calibrate_turn', self.srv_turn, callback_group=cb_srv)
        self.create_service(Trigger, '/motor/calibrate_speed', self.srv_speed, callback_group=cb_srv)
        self.get_logger().info('motor_calibrator ready: /motor/calibrate_turn, /motor/calibrate_speed')

    # ---- subscriptions ----
    def on_imu(self, m):
        t = time.monotonic()
        z = m.angular_velocity.z
        with self.lock:
            self.gyro.append((t, z))
            if self.integrating and self._last_gt is not None:
                self.angle += z * min(t - self._last_gt, 0.1)
            self._last_gt = t

    def on_scan(self, m):
        self.scan = m

    # ---- helpers ----
    def send(self, l, r):
        m = Int32MultiArray()
        m.data = [int(l), int(r)]
        self.pub.publish(m)

    def stop_motors(self):
        for _ in range(5):
            self.send(0, 0)
            time.sleep(0.03)

    def hold(self, l, r, seconds, check=None):
        """Publish [l, r] at 10 Hz for `seconds`; abort early if check() is true. Returns True if completed."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.send(l, r)
            if check is not None and check():
                return False
            time.sleep(0.1)
        return True

    def gyro_mean(self, window):
        with self.lock:
            return calib_math.window_mean(list(self.gyro), time.monotonic(), window)

    def clear_ahead(self):
        """Min range in the forward cone, or None if no scan."""
        s = self.scan
        if s is None:
            return None
        half = math.radians(float(self.get_parameter('speed_cone_deg').value))
        fwd = math.radians(float(self.get_parameter('lidar_forward_yaw_deg').value))
        best = float('inf')
        for i, r in enumerate(s.ranges):
            a = s.angle_min + i * s.angle_increment - fwd
            a = math.atan2(math.sin(a), math.cos(a))
            if abs(a) <= half and math.isfinite(r) and s.range_min <= r <= s.range_max:
                best = min(best, r)
        return best

    def set_test_mode(self, on):
        return set_bridge_params(self, {'allow_pwm_test': bool(on)}, target=self.bridge)

    def _save(self, key, value):
        force = bool(self.get_parameter('force_save').value)
        use, ok, reason = self.store.effective(key, value, force=force, source='motor_calibrator')
        live_msg = ''
        if use is not None:
            v = int(round(use)) if key != 'max_speed' else float(use)
            ok2, m2 = set_bridge_params(self, {key: v}, target=self.bridge)
            live_msg = f'; live {key}={v} ({m2})'
        return ok, reason + live_msg

    # ---- turn calibration ----
    def srv_turn(self, req, resp):
        if not self.busy.acquire(blocking=False):
            resp.success, resp.message = False, 'another calibration is running'
            return resp
        try:
            resp.success, resp.message = self._do_turn()
        except Exception as e:  # noqa: BLE001
            resp.success, resp.message = False, f'turn calibration failed: {e}'
            self.get_logger().error(resp.message)
        finally:
            try:
                self.stop_motors()
                self.set_test_mode(False)
                self.stop_motors()
            finally:
                with self.lock:
                    self.integrating = False
                self.busy.release()
        return resp

    def _do_turn(self):
        g = lambda n: self.get_parameter(n).value
        if self.gyro_mean(1.0) is None:
            return False, f'no gyro data on {g("imu_topic")}'
        ok, msg = self.set_test_mode(True)
        if not ok:
            return False, f'cannot enable allow_pwm_test: {msg}'
        thr = {}
        steps_all = {}
        max_ang = math.radians(float(g('turn_max_angle_deg')))
        for name, sgn in (('left', 1), ('right', -1)):
            # left spin (CCW): [-p, +p]; right spin (CW): [+p, -p]
            with self.lock:
                self.angle = 0.0
                self._last_gt = None
                self.integrating = True
            steps = []
            found = None
            p = int(g('turn_start_pwm'))
            over = lambda: abs(self.angle) >= max_ang
            while p <= int(g('turn_max_pwm')):
                done = self.hold(-sgn * p, sgn * p, float(g('turn_hold_s')), over)
                gz = self.gyro_mean(float(g('turn_window_s')))
                steps.append((p, gz if gz is not None else 0.0))
                found = calib_math.detect_turn_threshold(
                    steps, float(g('turn_detect_rad_s')), int(g('turn_consecutive')))
                if found is not None or not done:
                    break
                p += int(g('turn_step'))
            steps_all[name] = steps
            if found is None:
                self.stop_motors()
                with self.lock:
                    ang = math.degrees(self.angle)
                return False, (f'{name}: no turn threshold found (tested up to PWM {steps[-1][0] if steps else "-"}, '
                               f'angle {ang:.0f} deg, limit {g("turn_max_angle_deg")}); nothing saved')
            thr[name] = found
            # reference yaw rate
            ref = min(found + int(g('turn_ref_offset')), 255)
            self.hold(-sgn * ref, sgn * ref, float(g('turn_ref_hold_s')), over)
            rate = self.gyro_mean(0.4)
            thr[name + '_ref'] = (ref, abs(rate) if rate is not None else float('nan'))
            self.stop_motors()
            with self.lock:
                self.integrating = False
            time.sleep(1.0)     # let it settle
        value = max(thr['left'], thr['right']) + int(g('turn_margin'))
        saved_ok, smsg = self._save('min_turn_pwm', value)
        summary = (f'turn thresholds: left {thr["left"]}, right {thr["right"]} -> min_turn_pwm {value} '
                   f'({"saved" if saved_ok else "REJECTED, last good kept"}: {smsg}); '
                   f'yaw rate at PWM {thr["left_ref"][0]} (left): {thr["left_ref"][1]:.2f} rad/s, '
                   f'at PWM {thr["right_ref"][0]} (right): {thr["right_ref"][1]:.2f} rad/s')
        self.get_logger().info(summary)
        return saved_ok, summary

    # ---- speed calibration ----
    def _pose(self):
        try:
            tf = self.tfbuf.lookup_transform(self.get_parameter('map_frame').value,
                                             self.get_parameter('base_frame').value,
                                             rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return None
        s = tf.header.stamp
        tr = tf.transform.translation
        return (s.sec + s.nanosec * 1e-9, tr.x, tr.y, _yaw(tf.transform.rotation))

    def srv_speed(self, req, resp):
        if not self.busy.acquire(blocking=False):
            resp.success, resp.message = False, 'another calibration is running'
            return resp
        try:
            resp.success, resp.message = self._do_speed()
        except Exception as e:  # noqa: BLE001
            resp.success, resp.message = False, f'speed calibration failed: {e}'
            self.get_logger().error(resp.message)
        finally:
            try:
                self.stop_motors()
                self.set_test_mode(False)
                self.stop_motors()
            finally:
                self.busy.release()
        return resp

    def _do_speed(self):
        g = lambda n: self.get_parameter(n).value
        if self._pose() is None:
            return False, f'no TF {g("map_frame")}->{g("base_frame")}'
        clear = self.clear_ahead()
        need = float(g('speed_clear_m'))
        if clear is None:
            return False, f'no /scan on {g("scan_topic")}; refusing to drive'
        if clear < need:
            return False, f'obstacle {clear:.2f} m ahead (< {need} m); refusing to drive'
        cal = self.store.load()
        mins = [cal[k] for k in ('min_pwm_left', 'min_pwm_right') if k in cal]
        min_pwm = sum(mins) / len(mins) if mins else float(g('speed_min_pwm_fallback'))
        ok, msg = self.set_test_mode(True)
        if not ok:
            return False, f'cannot enable allow_pwm_test: {msg}'
        samples, rows = [], []
        for lvl in [int(x) for x in g('speed_levels')]:
            clear = self.clear_ahead()
            if clear is None or clear < need:
                rows.append(f'PWM {lvl}: skipped (obstacle)')
                break
            poses = []
            start = self._pose()
            if start is None:
                rows.append(f'PWM {lvl}: skipped (no TF)')
                break

            def check():
                p = self._pose()
                if p is not None and (not poses or p[0] != poses[-1][0]):
                    poses.append(p)
                if p is not None and math.hypot(p[1] - start[1], p[2] - start[2]) >= float(g('speed_max_dist_m')):
                    return True
                c = self.clear_ahead()
                return c is not None and c < 0.3

            self.hold(lvl, lvl, float(g('speed_drive_s')), check)
            self.stop_motors()
            v = calib_math.speed_from_poses(
                [(t - 0.0, x, y, th) for t, x, y, th in poses], float(g('speed_skip_s')))
            rows.append(f'PWM {lvl}: {len(poses)} poses, v={v if v is None else round(v, 3)} m/s')
            if v is not None:
                samples.append((lvl, v))
            time.sleep(float(g('speed_pause_s')))
        self.stop_motors()
        fit = calib_math.fit_speed(samples, min_pwm, int(g('max_pwm')))
        if fit is None:
            return False, 'speed fit failed (' + '; '.join(rows) + ')'
        ok, smsg = self._save('max_speed', fit['max_speed'])
        summary = (f'max_speed {fit["max_speed"]:.3f} m/s (k={fit["k"]:.5f} m/s per PWM, min_pwm {min_pwm:.0f}, '
                   f'rms {fit["rms"]:.3f}, n={fit["n"]}) '
                   f'({"saved" if ok else "REJECTED, last good kept"}: {smsg}) | ' + '; '.join(rows))
        self.get_logger().info(summary)
        return ok, summary


def main(args=None):
    rclpy.init(args=args)
    node = MotorCalibrator()
    ex = MultiThreadedExecutor(num_threads=6)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop_motors()
            node.set_test_mode(False)
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

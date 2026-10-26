#!/usr/bin/env python3
"""loc_monitor: how much to trust the pose, and what to do when it is wrong (final stack, item 1). Laptop only.

Confidence (/localization/confidence Float32 0..1, /localization/state String JSON, 2 Hz):
    0.6 * fraction of scan points on course_reference walls (within 5 cm)
  + 0.25 * agreement of rf2o and Cartographer motion over the last 2 s
  + 0.15 * pose inside the field rectangle;   outside the rectangle the confidence is capped at 0.2.
Checkpoint at every stop (rf2o speed < 2 cm/s and gyro < 2 deg/s for stop_s, not in a ramp zone): still scans are
merged and a local correlative search on course_reference finds the pose error.
    error <= ok_m / ok_deg            -> ok
    error <= correct_m / correct_deg  -> 'correct': relocalize Cartographer at the corrected pose (apply_corrections)
    otherwise, or a poor match        -> RECOVERY
Recovery: /localization/recovering = true (the mission pauses its goals when loc_pause is on; the dash never asks),
still scans, wide local search, then a global search over the free map (headings from the square walls). Ties are
broken by the last known pose and the gyro heading. Relocalize = /finish_trajectory + /start_trajectory with the
pose (relative to trajectory 0). Ramp exit: pitch up -> (top) -> down -> level marks the end of a ramp; the first
stop after it is a WIDE checkpoint (course_geometry ramp_profile, when measured, also gives the along-strip
position). Start check: /localization/verify_start (Trigger) matches still scans around (0, 0, 0).
Everything is logged on /localization/event (String JSON). Params: see declare_parameter below.
"""
import json
import math
import os
import time
from collections import deque

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from robot_slam import loc_core as lc
from robot_slam.scan_gate import in_zone, rp_from_quat


def repo_path(rel):
    """A repo-relative path such as maps/course/course_reference.yaml: as given (absolute / cwd), then under
    $HARDLEEL_DIR, then ~/hardleel (the laptop's link to the checkout)."""
    rel = os.path.expanduser(rel)
    for base in ('', os.environ.get('HARDLEEL_DIR', ''), os.path.expanduser('~/hardleel')):
        p = os.path.normpath(os.path.join(base, rel)) if base else rel
        if os.path.isfile(p):
            return p
    raise FileNotFoundError(f'{rel} (looked in cwd, $HARDLEEL_DIR, ~/hardleel)')


class Confidence:
    """Pure scoring (unit-tested)."""

    @staticmethod
    def agreement(d_rf2o, d_carto):
        """d_* = (dist m, dyaw rad) over the same window -> 0..1."""
        e = abs(d_rf2o[0] - d_carto[0]) + 0.3 * abs(lc.wrap(d_rf2o[1] - d_carto[1]))
        return math.exp(-e / 0.10)

    @staticmethod
    def combine(on_wall, agree, inside):
        c = 0.6 * on_wall + 0.25 * agree + 0.15 * (1.0 if inside else 0.0)
        return min(c, 0.2) if not inside else c


def classify_error(dxy, dyaw, score, p):
    """-> 'ok' | 'correct' | 'recover'."""
    if score < p['min_match_score']:
        return 'recover'
    if dxy <= p['ok_m'] and abs(dyaw) <= math.radians(p['ok_deg']):
        return 'ok'
    if dxy <= p['correct_m'] and abs(dyaw) <= math.radians(p['correct_deg']):
        return 'correct'
    return 'recover'


class RampWatch:
    """Pitch transitions: flat -> up -> (top) -> down -> flat = one ramp crossed. update() -> event or None."""

    def __init__(self, arm=math.radians(5), release=math.radians(2)):
        self.arm, self.release = arm, release
        self.seen_up = self.seen_down = False

    def update(self, pitch_up):
        ev = None
        if pitch_up > self.arm and not self.seen_up:
            self.seen_up, ev = True, 'climb_start'
        elif self.seen_up and pitch_up < -self.arm and not self.seen_down:
            self.seen_down, ev = True, 'descent_start'
        elif self.seen_up and abs(pitch_up) < self.release and self.seen_down:
            self.seen_up = self.seen_down = False
            ev = 'ramp_exit'
        return ev


class LocMonitor(Node):
    def __init__(self):
        super().__init__('loc_monitor')
        d = self.declare_parameter
        self.P = {
            'reference_map': d('reference_map', 'maps/course/course_reference.yaml').value,
            'geometry_file': d('geometry_file', '').value,
            'ok_m': d('ok_m', 0.03).value, 'ok_deg': d('ok_deg', 1.5).value,
            'correct_m': d('correct_m', 0.20).value, 'correct_deg': d('correct_deg', 8.0).value,
            'min_match_score': d('min_match_score', 0.45).value,
            'global_min_score': d('global_min_score', 0.55).value,
            'stop_s': d('stop_s', 0.7).value, 'still_scans': d('still_scans', 3).value,
            'apply_corrections': d('apply_corrections', False).value,
            'checkpoints': d('checkpoints', True).value,
            'carto_config_dir': d('carto_config_dir', '').value,
            'carto_config_basename': d('carto_config_basename', 'cartographer_2d.lua').value,
            'scan_topic': d('scan_topic', '/scan_filtered').value,
        }
        self.cb = ReentrantCallbackGroup()
        self.work = MutuallyExclusiveCallbackGroup()
        geo = self.P['geometry_file'] or os.path.join(get_package_share_directory('robot_slam'), 'config',
                                                      'course_geometry.yaml')
        self.geo = yaml.safe_load(open(geo))
        ref = repo_path(self.P['reference_map'])
        self.grid = lc.MapGrid.from_yaml(ref)
        self.field = lc.LikelihoodField(self.grid)
        self.free = lc.free_cells(self.grid, 0.10, 0.15)
        self.get_logger().info(f'reference map {ref}: {self.grid.w}x{self.grid.h}, {len(self.free)} free cells')
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.scans = deque(maxlen=8)
        self.rf2o = deque(maxlen=60)          # (t, x, y, yaw, speed)
        self.carto = deque(maxlen=60)         # (t, x, y, yaw)
        self.gyro_z = 0.0
        self.gyro_yaw = None                  # integrated gyro (relative), aligned to the pose at each good fix
        self.t_gyro = None
        self.pitch_up = 0.0
        self.rp0, self.boot = None, []
        self.ramp = RampWatch()
        self.stop_since = None
        self.checked_this_stop = False
        self.wide_next = False
        self.recovering = False
        self.last_good = None
        self.conf = 0.0
        self.busy = False
        self.pub_conf = self.create_publisher(Float32, '/localization/confidence', 10)
        self.pub_state = self.create_publisher(String, '/localization/state', 10)
        self.pub_event = self.create_publisher(String, '/localization/event', 20)
        self.pub_rec = self.create_publisher(Bool, '/localization/recovering', 10)
        self.create_subscription(LaserScan, self.P['scan_topic'], self.on_scan, qos_profile_sensor_data,
                                 callback_group=self.cb)
        self.create_subscription(Odometry, '/odom_rf2o', self.on_rf2o, 20, callback_group=self.cb)
        self.create_subscription(Imu, '/imu/data_raw', self.on_raw, qos_profile_sensor_data, callback_group=self.cb)
        self.create_subscription(Imu, '/imu/data', self.on_imu, qos_profile_sensor_data, callback_group=self.cb)
        self.create_service(Trigger, '/localization/verify_start', self.verify_start, callback_group=self.work)
        self.create_timer(0.5, self.tick, callback_group=self.work)
        self.cli = {}

    # ----------------------------------------------------------------------------------------------- inputs
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def event(self, name, **kw):
        kw['event'] = name
        kw['t'] = round(self.now(), 2)
        self.pub_event.publish(String(data=json.dumps(kw)))
        self.get_logger().info(f'{name}: {kw}')

    def on_scan(self, m):
        self.scans.append(m)

    def on_rf2o(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.rf2o.append((self.now(), m.pose.pose.position.x, m.pose.pose.position.y, yaw,
                          math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)))

    def on_raw(self, m):
        t = self.now()
        self.gyro_z = m.angular_velocity.z
        if self.t_gyro is not None and self.gyro_yaw is not None:
            self.gyro_yaw = lc.wrap(self.gyro_yaw + self.gyro_z * min(max(t - self.t_gyro, 0.0), 0.1))
        self.t_gyro = t

    def on_imu(self, m):
        if m.orientation_covariance[0] < 0:
            return
        r, p = rp_from_quat(m.orientation)
        if self.rp0 is None:
            self.boot.append(p)
            if len(self.boot) >= 50:
                self.rp0 = sum(self.boot) / len(self.boot)
            return
        self.pitch_up = -(p - self.rp0)                     # REP-103 pitch + = nose down
        ev = self.ramp.update(self.pitch_up)
        if ev:
            self.event(ev, pitch_deg=round(math.degrees(self.pitch_up), 1))
            if ev == 'ramp_exit':
                self.wide_next = True                       # first stop after a ramp = wide checkpoint

    def pose(self):
        try:
            t = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time())
        except TransformException:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def laser_pts(self, scans):
        """Merged robot-frame points of the given scans (laser frame ~ base_link: no yaw offset on hardleel)."""
        if not scans:
            return np.zeros((0, 2))
        return np.vstack([lc.scan_points(s.ranges, s.angle_min, s.angle_increment, 0.15, 6.0) for s in scans])

    # ----------------------------------------------------------------------------------------------- loop
    def tick(self):
        pose = self.pose()
        if pose is None or not self.scans:
            return
        t = self.now()
        self.carto.append((t,) + pose)
        if self.gyro_yaw is None:
            self.gyro_yaw = pose[2]
        pts = self.laser_pts([self.scans[-1]])
        on_wall = self.field.on_wall_fraction(pose, pts)
        agree = self.agreement(2.0)
        inside = lc.inside_rect(pose[0], pose[1], self.geo['field'], 0.05)
        self.conf = Confidence.combine(on_wall, 1.0 if agree is None else agree, inside)
        self.pub_conf.publish(Float32(data=float(self.conf)))
        self.pub_state.publish(String(data=json.dumps({
            'confidence': round(self.conf, 3), 'on_wall': round(on_wall, 3),
            'agree': None if agree is None else round(agree, 3), 'inside': inside,
            'pose': [round(v, 3) for v in pose], 'recovering': self.recovering})))
        if self.conf > 0.7 and not self.recovering:
            self.last_good = pose
        if self.P['checkpoints']:
            self.stop_logic(t, pose)

    def agreement(self, window):
        if len(self.rf2o) < 2 or len(self.carto) < 2:
            return None
        t = self.now()

        def delta(buf, has_speed):
            old = [b for b in buf if b[0] >= t - window]
            if len(old) < 2:
                return None
            a, b = old[0], old[-1]
            return math.hypot(b[1] - a[1], b[2] - a[2]), lc.wrap(b[3] - a[3])
        dr, dc = delta(self.rf2o, True), delta(self.carto, False)
        return None if dr is None or dc is None else Confidence.agreement(dr, dc)

    def stopped(self):
        if not self.rf2o:
            return False
        return self.rf2o[-1][4] < 0.02 and abs(self.gyro_z) < math.radians(2.0)

    def stop_logic(self, t, pose):
        zone = in_zone(pose[0], pose[1], self.geo.get('ramp_zones', []))
        if zone or not self.stopped():
            self.stop_since, self.checked_this_stop = None, False
            return
        self.stop_since = self.stop_since or t
        if self.checked_this_stop or t - self.stop_since < self.P['stop_s']:
            return
        self.checked_this_stop = True
        self.checkpoint(pose, wide=self.wide_next)
        self.wide_next = False

    # ----------------------------------------------------------------------------------------------- checks
    def still_points(self, n):
        """Wait for n fresh scans while still; -> merged points."""
        first = len(self.scans)
        stamp0 = self.scans[-1].header.stamp if self.scans else None
        end = time.monotonic() + 0.4 * n + 1.0
        fresh = []
        while time.monotonic() < end and len(fresh) < n:
            time.sleep(0.05)
            if self.scans and (stamp0 is None or self.scans[-1].header.stamp != stamp0):
                stamp0 = self.scans[-1].header.stamp
                fresh.append(self.scans[-1])
        return self.laser_pts(fresh or list(self.scans)[-1:])

    def checkpoint(self, pose, wide=False):
        pts = self.still_points(int(self.P['still_scans']))
        if wide:
            p, s, s0 = lc.correlative_search(self.field, pts, pose, 0.5, math.radians(25), 0.05, math.radians(2.5))
            p, s, _ = lc.correlative_search(self.field, pts, p, 0.06, math.radians(3), 0.01, math.radians(0.5))
        else:
            p, s, s0 = lc.refine(self.field, pts, pose)
        dxy = math.hypot(p[0] - pose[0], p[1] - pose[1])
        dyaw = lc.wrap(p[2] - pose[2])
        verdict = classify_error(dxy, dyaw, s, self.P)
        self.event('checkpoint', verdict=verdict, wide=wide, err_m=round(dxy, 3), err_deg=round(math.degrees(dyaw), 2),
                   score=round(s, 3), score_before=round(s0, 3), pose=[round(v, 3) for v in pose],
                   corrected=[round(v, 3) for v in p])
        if verdict == 'ok':
            self.gyro_yaw = p[2]
        elif verdict == 'correct':
            if self.P['apply_corrections']:
                self.relocalize(p, 'checkpoint correction')
            self.gyro_yaw = p[2]
        else:
            self.recover(pose)

    def recover(self, pose):
        self.recovering = True
        self.pub_rec.publish(Bool(data=True))
        self.event('recovery_start', pose=[round(v, 3) for v in pose], confidence=round(self.conf, 3))
        try:
            time.sleep(1.0)                                    # the mission stops (loc_pause); scans settle
            pts = self.still_points(4)
            p, s, _ = lc.correlative_search(self.field, pts, pose, 0.6, math.radians(30), 0.05, math.radians(3))
            p, s, _ = lc.correlative_search(self.field, pts, p, 0.06, math.radians(3), 0.01, math.radians(0.5))
            how = 'local'
            if s < self.P['global_min_score']:
                a, strength, _ = lc.dominant_wall_angle(pts)
                gy = self.gyro_yaw if self.gyro_yaw is not None else pose[2]
                heads = ([lc.heading_from_walls(a, gy + k * math.pi / 2) for k in range(4)] if a is not None
                         else list(np.radians(np.arange(-180, 180, 10))))
                cands = lc.global_search(self.field, pts, self.free, heads, top_k=6)
                best = lc.pick_candidate(cands, self.last_good, gy, self.P['global_min_score'])
                how = 'global'
                if best is None:
                    self.event('recovery_failed', best_score=round(cands[0][0], 3) if cands else None)
                    return
                p, s = best, max(c[0] for c in cands if c[1] == best)
            self.event('recovery_done', method=how, score=round(s, 3), pose=[round(v, 3) for v in p])
            if self.P['apply_corrections']:
                self.relocalize(p, f'recovery ({how})')
            self.gyro_yaw = p[2]
        finally:
            self.recovering = False
            self.pub_rec.publish(Bool(data=False))

    def verify_start(self, req, res):
        pts = self.still_points(3)
        pose = self.pose() or (0.0, 0.0, 0.0)
        p, s, s0 = lc.refine(self.field, pts, (0.0, 0.0, 0.0))
        dxy, dyaw = math.hypot(p[0], p[1]), lc.wrap(p[2])
        res.success = dxy <= 0.05 and abs(dyaw) <= math.radians(2.0) and s >= self.P['min_match_score']
        res.message = json.dumps({'offset_m': round(dxy, 3), 'offset_deg': round(math.degrees(dyaw), 2),
                                  'score': round(s, 3), 'match': [round(v, 3) for v in p],
                                  'tf_pose': [round(v, 3) for v in pose]})
        self.event('verify_start', ok=res.success, **json.loads(res.message))
        return res

    # ----------------------------------------------------------------------------------------------- Cartographer
    def call(self, srv_type, name, req, timeout=5.0):
        if name not in self.cli:
            self.cli[name] = self.create_client(srv_type, name, callback_group=self.cb)
        c = self.cli[name]
        if not c.wait_for_service(timeout_sec=2.0):
            return None
        fut = c.call_async(req)
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end:
            time.sleep(0.02)
        return fut.result() if fut.done() else None

    def relocalize(self, pose, why):
        from cartographer_ros_msgs.srv import FinishTrajectory, GetTrajectoryStates, StartTrajectory
        st = self.call(GetTrajectoryStates, '/get_trajectory_states', GetTrajectoryStates.Request())
        if st is None:
            self.event('relocalize_failed', why=why, reason='no /get_trajectory_states')
            return False
        active = [tid for tid, s in zip(st.trajectory_states.trajectory_id, st.trajectory_states.trajectory_state)
                  if s == 0]                                                    # ACTIVE
        for tid in active:
            self.call(FinishTrajectory, '/finish_trajectory', FinishTrajectory.Request(trajectory_id=tid), 10.0)
        req = StartTrajectory.Request()
        req.configuration_directory = self.P['carto_config_dir'] or os.path.join(
            get_package_share_directory('robot_slam'), 'config')
        req.configuration_basename = self.P['carto_config_basename']
        req.use_initial_pose = True
        req.initial_pose.position.x, req.initial_pose.position.y = float(pose[0]), float(pose[1])
        req.initial_pose.orientation.z, req.initial_pose.orientation.w = math.sin(pose[2] / 2), math.cos(pose[2] / 2)
        req.relative_to_trajectory_id = 0
        r = self.call(StartTrajectory, '/start_trajectory', req, 10.0)
        ok = r is not None and r.status.code == 0
        self.event('relocalize', ok=ok, why=why, finished=active, pose=[round(v, 3) for v in pose],
                   new_trajectory=getattr(r, 'trajectory_id', None))
        return ok


def main():
    rclpy.init()
    n = LocMonitor()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(n)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

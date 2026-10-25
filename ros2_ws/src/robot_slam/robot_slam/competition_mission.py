#!/usr/bin/env python3
"""Autonomous competition mission for hardleel (Robotics Dojo 2026), an explicit state machine over Nav2.

    WAIT_START -> GO_SCREEN -> CLASSIFY -> GO_LOADING -> LOAD -> TERRAIN -> OBSTACLES -> JUNCTION
    -> GATE_DECISION -> GO_PATH (Path 2 or Path 1) -> DEPOSIT -> FINISH -> DONE

Decision logic lives in competition_core.py / gate_check.py / obstacle_watch.py (pure Python, unit tested);
this file only does the ROS I/O.

Waypoints: `waypoints_file` (waypoints.py format, map frame, origin = start pose). Stages use the poses
screen, loading_<class>, sequence terrain, sequence obstacles (optional), junction + gate_check, sequence
path2 / path1, bin_<class>, finish.

Topics: in  /mission/start, /mission/abort (std_msgs/Empty), /scan, /map, /collision_monitor_state
        out /mission/state (String), /mission/event (String JSON -> event_logger), /speed_limit, /cmd_vel (stop)
Services: /classify (std_srvs/Trigger 'class:conf:colour'), /dump (std_srvs/Trigger, only if has_drop_bin),
          Nav2 costmap clearing.
Log: ~/logs/mission_<timestamp>.csv, one row per waypoint/state (the event_logger writes the judges' file).

Rules of the run: every goal has a timeout; a failed goal is retried once after clearing both costmaps, then
skipped so one failure never ends the run. If the collision monitor holds the robot stopped for
obstacle_wait_s, the goal is cancelled and re-sent (a replan around the obstacle, max_replans per goal).
After mission_time_limit_s the mission jumps to FINISH; /mission/abort stops everything at once.
"""
import json
import math
import os
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CollisionMonitorState, SpeedLimit
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from robot_slam import competition_core as core
from robot_slam import gate_check
from robot_slam import obstacle_watch
from robot_slam import waypoints as wp
from robot_slam.loading_adapters import make_loading_adapter

MONITOR_ACTIONS = {1: 'stop', 2: 'slowdown', 3: 'approach', 4: 'limit'}


def yaw_to_quat(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class Aborted(Exception):
    pass


class DeadlineHit(Exception):
    pass


class SkipToDeposit(Exception):
    """Soft deadline: less than deposit_reserve_s left before DEPOSIT -> go and deposit now."""


class SwitchToPath1(Exception):
    """Blocked by an obstacle where an alternative exists (to the junction/gate check, or on path2)."""


class CompetitionMission(Node):
    def __init__(self):
        super().__init__('competition_mission')
        p = self.declare_parameter
        p('auto_start', False)
        p('waypoints_file', '')                    # '' or missing = <share>/config/waypoints_course.yaml
        p('mission_time_limit_s', 175.0)
        p('finish_grace_s', 10.0)
        p('goal_timeout_s', 45.0)
        p('max_retries', 1)
        p('classify_retries', 2)
        p('classify_timeout_s', 8.0)
        p('reposition_yaw_deg', 8.0)
        p('default_class', 'plastic')
        p('stub_class', '')                        # non-empty: skip /classify (sim / no camera)
        p('bay_plastic', 'blue'); p('bay_organic', 'white'); p('bay_metal', 'black')
        p('loading_adapter', 'stub')
        p('load_wait_s', 6.0)
        p('has_drop_bin', False)
        p('drop_service', '/dump')
        p('drop_timeout_s', 8.0)
        p('bin_pause_s', 2.0)                      # pause at the bin when there is nothing to drop
        p('gate_max_wait_s', 5.0)                  # PLACEHOLDER: rulebook "standard waiting time" unknown
        p('gate_reserve_s', 55.0)                  # time kept for path + deposit + finish after the gate decision
        p('deposit_reserve_s', 35.0)               # less than this left before DEPOSIT -> skip ahead to deposit
        p('gate_center_deg', 0.0)                  # sector centre in the laser frame (0 = straight ahead)
        p('gate_half_width_deg', 10.0)
        p('gate_open_threshold_m', 1.0)            # median range above this = open
        p('gate_open_confirm_n', 3)
        p('gate_poll_hz', 5.0)
        p('obstacle_wait_s', 3.0)                  # held stopped this long -> cancel + re-send (replan)
        p('max_replans', 2)
        p('obstacle_range_m', 1.0)                 # cone detector: range and half angle straight ahead
        p('obstacle_half_angle_deg', 35.0)
        p('obstacle_min_points', 4)
        p('obstacle_route_len_m', 2.5)             # route detector: unexpected points near the first metres of
        p('obstacle_route_tol_m', 0.30)            # Nav2's plan (/plan) ...
        p('obstacle_route_max_range_m', 4.0)       # ... up to this lidar range
        p('obstacle_max_wait_s', 10.0)             # obstacle seen + no progress this long -> switch path or wait on
        p('obstacle_min_move_m', 0.10)
        p('obstacle_clear_s', 3.0)                 # this long without detections ends an obstacle episode
        p('obstacle_map_topic', '/map_nav')        # walls-only map frozen at the start (map_freezer)
        p('obstacle_switch_path', True)            # blocked to the junction/gate check or on path2 -> path1
        p('mission_speed', 0.30)
        p('terrain_speed', 0.15)
        p('terrain_surface', '')
        p('cmd_vel_topic', '/cmd_vel')
        p('log_dir', '~/logs')
        p('collision_monitor', '/collision_monitor')   # node whose stop box is switched to the mission one
        p('turn_calibration', False)               # NEW, off: /motor/calibrate_turn right after the start
        p('turn_calibration_timeout_s', 40.0)
        p('start_nudge_m', 0.15)                   # creep forward before the first goal: the robot starts with its
        p('start_nudge_speed', 0.10)               # back ~0.13 m from the start-box wall (Nav2 sees it 'in' the wall)
        g = lambda n: self.get_parameter(n).value
        self.g = g

        path = os.path.expanduser(str(g('waypoints_file')))
        if not path or not os.path.isfile(path):
            fallback = os.path.join(get_package_share_directory('robot_slam'), 'config', 'waypoints_course.yaml')
            self.get_logger().warn(f'waypoints file "{path}" not found, using packaged draft {fallback}')
            path = fallback
        self.poses, self.seqs, _meta = wp.load_waypoints(path)
        for problem in wp.check_waypoints(self.poses, self.seqs):
            self.get_logger().error(f'waypoints: {problem}')
        self.bay_map = {'plastic': str(g('bay_plastic')), 'organic': str(g('bay_organic')),
                        'metal': str(g('bay_metal'))}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.classify_cli = self.create_client(Trigger, 'classify')
        self.drop_cli = self.create_client(Trigger, str(g('drop_service')))
        self.clear_srvs = [self.create_client(ClearEntireCostmap, n) for n in
                           ('/global_costmap/clear_entirely_global_costmap',
                            '/local_costmap/clear_entirely_local_costmap')]
        self.pub_state = self.create_publisher(String, '/mission/state', 10)
        self.pub_event = self.create_publisher(String, '/mission/event', 50)
        self.pub_limit = self.create_publisher(SpeedLimit, 'speed_limit', 1)
        self.pub_surface = self.create_publisher(String, '/surface_override', 1)
        self.pub_cmd = self.create_publisher(Twist, str(g('cmd_vel_topic')), 1)
        self.create_subscription(Empty, '/mission/start', lambda m: setattr(self, 'start_req', True), 1)
        self.create_subscription(Empty, '/mission/abort', self.on_abort, 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, str(g('obstacle_map_topic')), self.on_map,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(NavPath, '/plan', self.on_plan, 1)
        self.create_subscription(CollisionMonitorState, '/collision_monitor_state', self.on_monitor, 10)
        self.create_timer(1.0, lambda: self.publish_state())
        self.loader = make_loading_adapter(str(g('loading_adapter')), self, {'load_wait_s': g('load_wait_s')})

        self.start_req = False
        self.abort_req = False
        self.scan = None
        self.grid = None
        self.state = 'WAIT_START'
        self.endgame = False
        self.deadline = None
        self.waste_class, self.waste_conf, self.bay_colour = 'unknown', 0.0, ''
        self.gate_decision = ''
        # obstacle bookkeeping: every episode logs obstacle_detected + obstacle_response; the event logger keeps
        # the first of each as the rulebook timestamp and stores the rest
        self.monitor_action, self.stop_since = 0, None
        self.episodes = core.ObstacleEpisodes(float(g('obstacle_clear_s')))
        self.obst_seen_t = None                    # last time either detector saw something
        self.plan_xy = []                          # current Nav2 plan (map frame)
        self.current_path = ''                     # path1 / path2 while GO_PATH runs
        self.skipped_to_deposit = False

        log_dir = os.path.expanduser(str(g('log_dir')))
        os.makedirs(log_dir, exist_ok=True)
        self.log_path = os.path.join(log_dir, time.strftime('mission_%Y%m%d_%H%M%S.csv'))
        with open(self.log_path, 'w') as f:
            f.write('wall_time,ros_time_s,mission_t_s,state,result,x,y,yaw_deg,detail\n')
        self.get_logger().info(f'{len(self.poses)} poses from {path}; log {self.log_path}')

    # ----------------------------------------------------------------- helpers
    def now(self):
        return time.monotonic()

    def running(self):
        return self.deadline is not None and not self.endgame

    def event(self, name, **detail):
        self.pub_event.publish(String(data=json.dumps({'event': name, 'detail': detail})))
        self.log_row(f'EVENT:{name}', 'event', json.dumps(detail).replace(',', ';'))

    def on_abort(self, _):
        self.get_logger().error('ABORT received')
        self.abort_req = True

    def on_map(self, m):
        # Obstacle detection compares the scan with /map_nav: the map frozen at the start with free-standing blobs
        # (workers/cylinders from the scan) removed by map_freezer - walls only, so any worker counts as unexpected.
        self.grid = {'data': np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width),
                     'resolution': m.info.resolution,
                     'origin_x': m.info.origin.position.x, 'origin_y': m.info.origin.position.y}

    def ensure_nav2_active(self, tries=3):
        """Before the start: Nav2 must be active. A lifecycle bring-up that timed out (seen in the sim under load)
        leaves every goal 'rejected'; ask the lifecycle manager to STARTUP again. Loud log either way."""
        from nav2_msgs.srv import ManageLifecycleNodes
        mgr = '/lifecycle_manager_navigation'
        is_active = self.create_client(Trigger, mgr + '/is_active')
        manage = self.create_client(ManageLifecycleNodes, mgr + '/manage_nodes')

        def call(cli, req, timeout):
            if not cli.wait_for_service(timeout_sec=5.0):
                return None
            fut = cli.call_async(req)
            end = self.now() + timeout
            while not fut.done() and self.now() < end:
                rclpy.spin_once(self, timeout_sec=0.05)
            return fut.result() if fut.done() else None

        for k in range(tries):
            r = call(is_active, Trigger.Request(), 5.0)
            if r is not None and r.success:
                self.get_logger().info('NAV2 READY')
                return True
            self.get_logger().error(f'NAV2 NOT ACTIVE (check {k + 1}/{tries}): asking {mgr} to start it again')
            req = ManageLifecycleNodes.Request()
            req.command = ManageLifecycleNodes.Request.RESET
            call(manage, req, 30.0)
            req.command = ManageLifecycleNodes.Request.STARTUP
            call(manage, req, 60.0)
        self.get_logger().error('!' * 60 + '\nNAV2 STILL NOT ACTIVE - restart laptop_mission before the start\n' + '!' * 60)
        return False

    def set_monitor_profile(self, mission=True):
        """Collision monitor: mapping stop box -> mission stop box (or back). Logged, never fatal."""
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
        from rcl_interfaces.srv import SetParameters
        cli = self.create_client(SetParameters, str(self.g('collision_monitor')) + '/set_parameters')
        req = SetParameters.Request()
        for name, on in (('StopBox.enabled', mission), ('StopBoxMapping.enabled', not mission)):
            req.parameters.append(Parameter(name=name, value=ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                                                            bool_value=on)))
        for _ in range(3):
            if cli.wait_for_service(timeout_sec=3.0):
                fut = cli.call_async(req)
                end = self.now() + 3.0
                while not fut.done() and self.now() < end:
                    rclpy.spin_once(self, timeout_sec=0.05)
                if fut.done() and fut.result() and all(r.successful for r in fut.result().results):
                    self.get_logger().info(f'collision monitor: {"mission" if mission else "mapping"} stop box')
                    return True
        self.get_logger().error('collision monitor profile switch FAILED (mapping stop box stays active)')
        return False

    def on_monitor(self, m):
        self.monitor_action = m.action_type
        if m.action_type == 1:
            if self.stop_since is None:
                self.stop_since = self.now()
        else:
            self.stop_since = None
        if m.action_type in MONITOR_ACTIONS and self.episodes.active:
            self.obstacle_response(MONITOR_ACTIONS[m.action_type])

    def on_plan(self, m):
        self.plan_xy = [(ps.pose.position.x, ps.pose.position.y) for ps in m.poses]

    def on_scan(self, m):
        self.scan = m
        if not self.running() or self.grid is None:
            return
        pose = self.pose_in_map(m.header.frame_id or 'laser')
        if pose is None:
            return
        g = self.g
        n_cone = obstacle_watch.unexpected_points(
            m.ranges, m.angle_min, m.angle_increment, pose, self.grid, float(g('obstacle_range_m')),
            math.radians(float(g('obstacle_half_angle_deg'))))
        n_route = obstacle_watch.unexpected_near_path(
            m.ranges, m.angle_min, m.angle_increment, pose, self.grid, self.plan_xy, float(g('obstacle_route_len_m')),
            float(g('obstacle_route_tol_m')), float(g('obstacle_route_max_range_m'))) if self.plan_xy else 0
        need = int(g('obstacle_min_points'))
        seen = n_cone >= need or n_route >= need
        if seen:
            self.obst_seen_t = self.now()
        change = self.episodes.update(self.now(), seen)
        if change == 'start':
            self.event('obstacle_detected', detector='route' if n_route >= need else 'cone', points=max(n_cone, n_route),
                       state=self.state)
        elif change == 'cleared' and not self.episodes.responded:
            self.obstacle_response('avoided')              # gone (walked away / driven around) before any action

    def obstacle_seen_recently(self, hold_s=1.5):
        return self.obst_seen_t is not None and self.now() - self.obst_seen_t <= hold_s

    def obstacle_response(self, action):
        """First reaction of the robot to the current obstacle episode (one event per episode)."""
        if self.episodes.start_t is None or self.episodes.responded:
            return
        self.episodes.responded = True
        self.event('obstacle_response', action=action, response_s=round(self.now() - self.episodes.start_t, 2))

    def publish_state(self):
        self.pub_state.publish(String(data=self.state))

    def pose_in_map(self, frame='base_link'):
        try:
            t = self.tf_buffer.lookup_transform('map', frame, rclpy.time.Time())
        except TransformException:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def log_row(self, state, result, detail=''):
        x, y, yaw = self.pose_in_map() or (float('nan'),) * 3
        mt = self.deadline.elapsed(self.now()) if self.deadline else 0.0
        with open(self.log_path, 'a') as f:
            f.write(f'{time.strftime("%Y-%m-%d %H:%M:%S")},{self.get_clock().now().nanoseconds / 1e9:.3f},'
                    f'{mt:.1f},{state},{result},{x:.3f},{y:.3f},{math.degrees(yaw):.1f},{detail}\n')
        self.get_logger().info(f'[{mt:6.1f}s] {state}: {result} {detail}')

    def enter(self, state):
        self.state = state
        self.publish_state()

    def stop_robot(self):
        for _ in range(5):
            self.pub_cmd.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait(self, pred, timeout=None):
        """Spin until pred() is true (-> True) or timeout (-> False). Raises Aborted / DeadlineHit / SkipToDeposit."""
        t_end = self.now() + timeout if timeout is not None else None
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.abort_req:
                raise Aborted()
            if not self.endgame and self.deadline and self.deadline.expired(self.now()):
                raise DeadlineHit()
            if (self.deadline and not self.skipped_to_deposit and
                    core.soft_deadline_due(self.state, self.deadline.remaining(self.now()),
                                           float(self.g('deposit_reserve_s')))):
                self.skipped_to_deposit = True
                raise SkipToDeposit()
            if pred():
                return True
            if t_end is not None and self.now() >= t_end:
                return False
        raise Aborted()

    def pause(self, seconds):
        self.wait(lambda: False, seconds)

    def set_speed(self, speed):
        m = SpeedLimit()
        m.header.stamp = self.get_clock().now().to_msg()
        m.percentage = False
        m.speed_limit = float(speed)
        self.pub_limit.publish(m)

    def set_surface(self, name):
        if name:
            self.pub_surface.publish(String(data=name))

    def clear_costmaps(self):
        for c in self.clear_srvs:
            if c.wait_for_service(timeout_sec=0.5):
                fut = c.call_async(ClearEntireCostmap.Request())
                try:
                    self.wait(fut.done, 2.0)
                except DeadlineHit:
                    pass

    def call_trigger(self, client, timeout):
        """-> (ok, message); ok None when the service is missing or timed out."""
        if not client.wait_for_service(timeout_sec=1.0):
            return None, 'service missing'
        fut = client.call_async(Trigger.Request())
        if not self.wait(fut.done, timeout):
            return None, 'timeout'
        r = fut.result()
        return r.success, r.message

    # ----------------------------------------------------------------- navigation
    def send_goal(self, pose, timeout):
        """One NavigateToPose attempt -> 'ok' | 'failed' | 'timeout' | 'rejected' | 'blocked' | 'obstacle'.
        'blocked': the collision monitor held the robot stopped for obstacle_wait_s (-> re-send = replan).
        'obstacle': an obstacle is on the way and the robot made no progress for obstacle_max_wait_s."""
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(pose[0]), float(pose[1])
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = yaw_to_quat(pose[2])
        handle = None
        wait_s = float(self.g('obstacle_wait_s'))
        try:
            if not self.wait(self.nav.server_is_ready, 5.0):
                return 'rejected'
            send = self.nav.send_goal_async(goal)
            if not self.wait(send.done, 10.0):
                return 'rejected'
            handle = send.result()
            if handle is None or not handle.accepted:
                return 'rejected'
            res = handle.get_result_async()
            self.plan_xy = []                                       # the old plan belongs to the previous goal
            start = self.pose_in_map()
            watch = core.ProgressWatch(self.now(), start[:2] if start else None, float(self.g('obstacle_max_wait_s')),
                                       float(self.g('obstacle_min_move_m')))
            stuck = [False]

            def obstacle_stuck():
                p = self.pose_in_map()
                seen = self.obstacle_seen_recently()
                stuck[0] = watch.update(self.now(), p[:2] if p else None, seen)
                if seen and self.episodes.active and self.now() - watch.anchor_t >= 1.0:
                    self.obstacle_response('wait')      # Nav2 stopped short of it (no collision-monitor action)
                return stuck[0]

            blocked = lambda: self.stop_since is not None and self.now() - self.stop_since > wait_s
            if not self.wait(lambda: res.done() or blocked() or obstacle_stuck(), timeout):
                return 'timeout'
            if not res.done():
                return 'obstacle' if stuck[0] else 'blocked'
            handle = None                                           # finished by itself
            return 'ok' if res.result().status == GoalStatus.STATUS_SUCCEEDED else 'failed'
        finally:
            if handle is not None:                                  # timeout, block, abort or deadline: cancel
                handle.cancel_goal_async()
                end = self.now() + 0.4
                while self.now() < end and rclpy.ok():
                    rclpy.spin_once(self, timeout_sec=0.05)

    def navigate(self, pose, label, timeout=None):
        """Goal with obstacle replans, then retry (after clearing costmaps) and skip -> 'ok' | 'skipped'."""
        policy = core.GoalPolicy(int(self.g('max_retries')))
        timeout = float(self.g('goal_timeout_s')) if timeout is None else timeout
        replans = 0
        while True:
            res = self.send_goal(pose, timeout)
            if res == 'obstacle' or (res in ('timeout', 'failed') and self.obstacle_seen_recently(5.0)):
                # blocked for obstacle_max_wait_s (or the goal ran out while an obstacle was in the way)
                if self.can_switch_path():
                    self.obstacle_response('switch_to_path1')
                    self.log_row(f'wp:{label}', 'obstacle', 'blocked: switching to path1 (alternative available)')
                    raise SwitchToPath1()
                self.obstacle_response('wait')
                self.log_row(f'wp:{label}', 'obstacle', 'blocked, no alternative here: waiting and re-checking')
                continue                             # keep waiting (soft deadline -> deposit if time runs out)
            if res == 'blocked' and replans < int(self.g('max_replans')):
                replans += 1
                self.stop_since = None
                self.obstacle_response('wait_then_replan')
                self.log_row(f'wp:{label}', 'blocked', f'waited {self.g("obstacle_wait_s")} s, replan {replans}')
                continue
            action = policy.on_result('failed' if res == 'blocked' else res)
            if action == 'done':
                return 'ok'
            self.get_logger().warn(f'{label}: {res} (attempt {policy.attempts}) -> {action}')
            if action == 'skip':
                return 'skipped'
            self.clear_costmaps()

    def can_switch_path(self):
        """An alternative route exists: still on the way to the junction / gate check, or on path2."""
        if not self.g('obstacle_switch_path'):
            return False
        return core.path_switch_possible(self.state, self.current_path)

    def run_names(self, names, timeout=None):
        results = []
        for name in names:
            if name not in self.poses:
                self.log_row(f'wp:{name}', 'missing')
                results.append('skipped')
                continue
            r = self.navigate(self.poses[name], name, timeout)
            results.append(r)
            self.log_row(f'wp:{name}', r)
        return core.stage_result(results)

    # ----------------------------------------------------------------- state handlers
    def do_wait_start(self):
        self.get_logger().info('Waiting for /mission/start' + (' (auto_start)' if self.g('auto_start') else ''))
        self.log_row('WAIT_START', 'waiting')
        self.set_monitor_profile(mission=True)
        self.ensure_nav2_active()
        if not self.g('auto_start'):
            self.wait(lambda: self.start_req)
        self.deadline = core.Deadline(self.now(), float(self.g('mission_time_limit_s')))
        self.event('autonomous_start')
        if self.g('turn_calibration'):
            ok, msg = self.call_trigger(self.create_client(Trigger, '/motor/calibrate_turn'),
                                        float(self.g('turn_calibration_timeout_s')))
            self.log_row('TURN_CALIBRATION', 'ok' if ok else 'failed', str(msg).replace(',', ';'))
        self.start_nudge()
        return 'started'

    def start_nudge(self):
        """Drive start_nudge_m straight ahead through twist_mux (navigation input) -> velocity smoother ->
        collision monitor, so the stop box still protects it. Nav2 cannot plan while the footprint touches the
        start-box wall behind the robot."""
        dist, v = float(self.g('start_nudge_m')), float(self.g('start_nudge_speed'))
        if dist <= 0 or v <= 0:
            return
        pub = self.create_publisher(Twist, '/cmd_vel_nav', 1)
        start = self.pose_in_map()
        end = self.now() + dist / v + 1.0                     # +1 s for the velocity smoother ramp
        while self.now() < end:
            p = self.pose_in_map()
            if start and p and math.hypot(p[0] - start[0], p[1] - start[1]) >= dist:
                break
            t = Twist()
            t.linear.x = v
            pub.publish(t)
            self.pause(0.1)
        for _ in range(3):
            pub.publish(Twist())
            self.pause(0.05)
        p = self.pose_in_map()
        moved = math.hypot(p[0] - start[0], p[1] - start[1]) if start and p else float('nan')
        self.log_row('START_NUDGE', 'done', f'moved {moved:.2f} m')

    def do_go_screen(self):
        self.set_speed(self.g('mission_speed'))
        return self.run_names(['screen'])

    def do_classify(self):
        stub = str(self.g('stub_class'))
        if stub:
            self.waste_class, self.waste_conf = stub, 1.0
            self.event('waste_detected', **{'class': stub, 'confidence': 1.0, 'source': 'stub'})
            return 'stub'
        n = int(self.g('classify_retries'))
        base = self.poses['screen']
        for k in range(n + 1):
            ok, msg = self.call_trigger(self.classify_cli, float(self.g('classify_timeout_s')))
            cls, conf, colour = core.parse_classify(msg) if ok else ('unknown', 0.0, '')
            if core.usable_class(cls):
                self.waste_class, self.waste_conf = cls, conf
                self.event('waste_detected', **{'class': cls, 'confidence': round(conf, 3), 'attempt': k + 1})
                return 'ok'
            self.log_row('CLASSIFY_TRY', 'unusable', f'{k + 1}/{n + 1} {msg}')
            if k < n:                                              # small reposition: wiggle the heading
                sign = 1 if k % 2 == 0 else -1
                dyaw = math.radians(float(self.g('reposition_yaw_deg'))) * sign
                self.navigate((base[0], base[1], base[2] + dyaw), 'reposition', 15.0)
        self.waste_class, self.waste_conf = str(self.g('default_class')), 0.0
        self.event('waste_detected', **{'class': self.waste_class, 'confidence': 0.0, 'source': 'default'})
        return 'default_class'

    def do_go_loading(self):
        return self.run_names([f'loading_{self.waste_class}'])

    def do_load(self):
        result, detail = self.loader.run()
        self.event('loading_complete', result=result, detail=detail)
        return result

    def do_terrain(self):
        self.set_speed(self.g('terrain_speed'))
        self.set_surface(str(self.g('terrain_surface')))
        r = self.run_names(self.seqs.get('terrain', []))
        self.event('terrain_complete', result=r)
        return r

    def do_obstacles(self):
        self.set_speed(self.g('mission_speed'))
        if self.g('terrain_surface'):
            self.set_surface('AUTO')
        return self.run_names(self.seqs.get('obstacles', []))

    def do_junction(self):
        return self.run_names(['junction', 'gate_check'])

    def gate_median(self):
        s = self.scan
        if s is None:
            return None
        return gate_check.sector_median(s.ranges, s.angle_min, s.angle_increment, float(self.g('gate_center_deg')),
                                        float(self.g('gate_half_width_deg')), max(s.range_min, 0.12),
                                        min(s.range_max, 12.0))

    def do_gate_decision(self):
        """Wait for the gate up to gate_max_wait_s, but never into the time the rest of the run needs
        (gate_reserve_s). Budget 0 = take whatever is open now: an open gate -> path2, else path1 (always open)."""
        remaining = self.deadline.remaining(self.now()) if self.deadline else float('inf')
        budget = core.gate_wait_budget(remaining, float(self.g('gate_max_wait_s')), float(self.g('gate_reserve_s')))
        confirm = int(self.g('gate_open_confirm_n'))
        if budget <= 0.0:
            confirm = 1                                             # no time: one reading decides
        poller = gate_check.GatePoller(self.now(), budget, float(self.g('gate_open_threshold_m')), confirm)
        period = 1.0 / max(float(self.g('gate_poll_hz')), 0.5)
        decision = poller.update(self.now(), self.gate_median())   # budget 0: decided by this reading
        while decision is None:
            self.pause(period)
            decision = poller.update(self.now(), self.gate_median())
        self.gate_decision, wait_s = decision
        self.event('gate_decision', decision=self.gate_decision, wait_s=round(wait_s, 2), wait_budget_s=round(budget, 1),
                   remaining_s=round(remaining, 1),
                   last_median_m=None if poller.last_median is None else round(poller.last_median, 2))
        return self.gate_decision

    def do_go_path(self):
        first = 'path2' if self.gate_decision == 'path2' else 'path1'
        self.current_path = first
        r = self.run_names(self.seqs.get(first, []))
        if first == 'path2' and r == 'skipped':                     # Path 2 unusable: take Path 1 (always open)
            self.log_row('GO_PATH', 'path2 failed, switching to path1')
            self.current_path = 'path1'
            r = self.run_names(self.seqs.get('path1', []))
            return f'path2->path1:{r}'
        return f'{first}:{r}'

    def do_deposit(self):
        self.bay_colour = core.bay_for_class(self.waste_class, self.bay_map, str(self.g('default_class')))
        name = f'bin_{self.waste_class}' if f'bin_{self.waste_class}' in self.poses else \
            f'bin_{self.g("default_class")}'
        r = self.run_names([name])
        dropped, msg = False, 'no drop bin'
        if self.g('has_drop_bin'):
            ok, msg = self.call_trigger(self.drop_cli, float(self.g('drop_timeout_s')))
            dropped = bool(ok)
        else:
            self.pause(float(self.g('bin_pause_s')))
        self.event('deposit_complete', bin=name, colour=self.bay_colour, reached=r, dropped=dropped, detail=msg)
        return f'{name}:{r}:{"dropped" if dropped else msg}'

    def do_finish(self):
        self.endgame = True
        grace = float(self.g('finish_grace_s'))
        remaining = self.deadline.remaining(self.now()) if self.deadline else grace
        timeout = min(float(self.g('goal_timeout_s')), max(remaining, 0.0) + grace)
        r = self.run_names(['finish'], timeout=timeout)
        self.stop_robot()
        self.event('finish', result=r)
        return r

    # ----------------------------------------------------------------- main loop
    def run(self):
        handlers = {'WAIT_START': self.do_wait_start, 'GO_SCREEN': self.do_go_screen, 'CLASSIFY': self.do_classify,
                    'GO_LOADING': self.do_go_loading, 'LOAD': self.do_load, 'TERRAIN': self.do_terrain,
                    'OBSTACLES': self.do_obstacles, 'JUNCTION': self.do_junction,
                    'GATE_DECISION': self.do_gate_decision, 'GO_PATH': self.do_go_path,
                    'DEPOSIT': self.do_deposit, 'FINISH': self.do_finish}
        state = 'WAIT_START'
        try:
            while rclpy.ok() and state not in core.TERMINAL:
                self.enter(state)
                try:
                    result = handlers[state]()
                    self.log_row(state, str(result), self.detail())
                    state = core.next_state(state)
                except DeadlineHit:
                    self.log_row(state, 'deadline', f'limit {self.g("mission_time_limit_s")}s')
                    state = core.state_after_deadline(state)
                except SwitchToPath1:
                    if state in ('JUNCTION', 'GATE_DECISION') and not self.gate_decision:
                        self.event('gate_decision', decision='path1', wait_s=0.0, reason='obstacle before the gate')
                    self.gate_decision = 'path1'
                    self.log_row(state, 'switch_to_path1', 'obstacle')
                    state = 'GO_PATH'
                except SkipToDeposit:
                    left = self.deadline.remaining(self.now())
                    self.log_row(state, 'skip_to_deposit', f'{left:.0f} s left < deposit_reserve_s '
                                                           f'{self.g("deposit_reserve_s")}')
                    state = core.state_after_soft_deadline(state)
        except Aborted:
            self.state = 'ABORTED'
            self.publish_state()
            self.stop_robot()
            self.log_row('ABORTED', 'abort', f'during {state}')
            return
        self.enter('DONE')
        self.stop_robot()
        self.log_row('DONE', 'mission finished', self.detail())

    def detail(self):
        return f'class={self.waste_class} bay={self.bay_colour} gate={self.gate_decision}'.strip()


def main(args=None):
    rclpy.init(args=args)
    node = CompetitionMission()
    try:
        node.run()
    except KeyboardInterrupt:
        node.stop_robot()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()

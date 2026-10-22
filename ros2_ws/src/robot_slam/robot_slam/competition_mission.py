#!/usr/bin/env python3
"""Autonomous competition mission for hardleel (Robotics Dojo 2026), an explicit state machine over Nav2.

    WAIT_START -> GO_SCREEN -> CLASSIFY -> GO_LOADING -> LOAD -> TERRAIN -> OBSTACLES -> JUNCTION
    -> GATE_DECISION -> GO_PATH (Path 2 or Path 1) -> DEPOSIT -> FINISH -> DONE

All decision logic lives in competition_core.py (pure Python, unit tested); this file only does the ROS I/O.

Topics: in  /mission/start, /mission/abort, /loading/done (std_msgs/Empty), /scan
        out /mission/state (String), /deposit/trigger (Empty), /speed_limit (nav2 SpeedLimit),
            /surface_override (String, optional), /cmd_vel (zero on stop)
Services: /classify (std_srvs/Trigger, answers 'class:conf:colour'); costmap clear services of Nav2.
Log: ~/logs/mission_<timestamp>.csv, one row per finished state (rulebook section 8 event timestamps).

Rules of the run: a goal is retried once after clearing both costmaps, then skipped so that one failure does not
end the run; after `mission_time_limit_s` the mission jumps to FINISH; /mission/abort stops everything at once.
"""
import math
import os
import time

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from robot_slam import competition_core as core
from robot_slam.coverage_mission import yaw_to_quat


class Aborted(Exception):
    pass


class DeadlineHit(Exception):
    pass


class CompetitionMission(Node):
    def __init__(self):
        super().__init__('competition_mission')
        p = self.declare_parameter
        p('auto_start', False)
        p('course_file', '')                       # '' = <share>/config/competition_course.yaml
        p('mission_time_limit_s', 175.0)
        p('finish_grace_s', 10.0)                  # time allowed for the finish goal after the deadline
        p('goal_timeout_s', 45.0)
        p('max_retries', 1)                        # retries per goal after ClearEntireCostmap
        p('classify_retries', 2)
        p('classify_timeout_s', 8.0)
        p('reposition_yaw_deg', 8.0)               # yaw wiggle at the screen between classify attempts
        p('default_class', 'plastic')              # used when /classify never gives a usable answer
        p('bay_plastic', 'blue'); p('bay_organic', 'white'); p('bay_metal', 'black')
        p('load_wait_s', 6.0)                      # PLACEHOLDER loading protocol
        p('deposit_wait_s', 3.0)                   # PLACEHOLDER deposit actuator
        p('gate_wait_s', 15.0)                     # PLACEHOLDER: the rulebook "standard waiting time" is unknown
        p('gate_stable_s', 0.8)
        p('gate_max_range_m', 0.9)
        p('gate_half_angle_deg', 20.0)
        p('mission_speed', 0.30)
        p('terrain_speed', 0.15)
        p('terrain_surface', '')                   # e.g. SAND; '' = do not publish /surface_override
        p('cmd_vel_topic', '/cmd_vel')
        p('log_dir', '~/logs')
        g = lambda n: self.get_parameter(n).value
        self.g = g

        course = str(g('course_file')) or os.path.join(
            get_package_share_directory('robot_slam'), 'config', 'competition_course.yaml')
        self.wps, self.stages = core.load_course(course)
        self.bay_map = {'plastic': str(g('bay_plastic')), 'organic': str(g('bay_organic')),
                        'metal': str(g('bay_metal'))}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.classify_cli = self.create_client(Trigger, 'classify')
        self.clear_srvs = [self.create_client(ClearEntireCostmap, n) for n in
                           ('/global_costmap/clear_entirely_global_costmap',
                            '/local_costmap/clear_entirely_local_costmap')]
        self.pub_state = self.create_publisher(String, '/mission/state', 10)
        self.pub_limit = self.create_publisher(SpeedLimit, 'speed_limit', 1)
        self.pub_surface = self.create_publisher(String, '/surface_override', 1)
        self.pub_deposit = self.create_publisher(Empty, '/deposit/trigger', 1)
        self.pub_cmd = self.create_publisher(Twist, str(g('cmd_vel_topic')), 1)
        self.create_subscription(Empty, '/mission/start', lambda m: setattr(self, 'start_req', True), 1)
        self.create_subscription(Empty, '/mission/abort', self.on_abort, 10)
        self.create_subscription(Empty, '/loading/done', lambda m: setattr(self, 'loading_done', True), 1)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(1.0, lambda: self.publish_state())

        self.start_req = False
        self.abort_req = False
        self.loading_done = False
        self.scan = None
        self.state = 'WAIT_START'
        self.endgame = False
        self.deadline = None
        self.waste_class, self.bay_colour = 'unknown', ''
        self.gate_decision = ''
        self.active_goal = None

        log_dir = os.path.expanduser(str(g('log_dir')))
        os.makedirs(log_dir, exist_ok=True)
        self.log_path = os.path.join(log_dir, time.strftime('mission_%Y%m%d_%H%M%S.csv'))
        with open(self.log_path, 'w') as f:
            f.write('wall_time,ros_time_s,mission_t_s,state,result,x,y,yaw_deg,detail\n')
        self.get_logger().info(f'{len(self.wps)} waypoints from {course}; log {self.log_path}')

    # ----------------------------------------------------------------- helpers
    def now(self):
        return time.monotonic()

    def on_abort(self, _):
        self.get_logger().error('ABORT received')
        self.abort_req = True

    def on_scan(self, m):
        self.scan = m

    def publish_state(self):
        msg = String()
        msg.data = self.state
        self.pub_state.publish(msg)

    def pose_now(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except TransformException:
            return (float('nan'),) * 3
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, math.degrees(yaw)

    def log_row(self, state, result, detail=''):
        x, y, yaw = self.pose_now()
        mt = self.deadline.elapsed(self.now()) if self.deadline else 0.0
        with open(self.log_path, 'a') as f:
            f.write(f'{time.strftime("%Y-%m-%d %H:%M:%S")},{self.get_clock().now().nanoseconds / 1e9:.3f},'
                    f'{mt:.1f},{state},{result},{x:.3f},{y:.3f},{yaw:.1f},{detail}\n')
        self.get_logger().info(f'[{mt:6.1f}s] {state}: {result} {detail}')

    def enter(self, state):
        self.state = state
        self.publish_state()

    def stop_robot(self):
        for _ in range(5):
            self.pub_cmd.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait(self, pred, timeout=None):
        """Spin until pred() is true (-> True) or timeout (-> False). Raises Aborted / DeadlineHit."""
        t_end = self.now() + timeout if timeout is not None else None
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.abort_req:
                raise Aborted()
            if not self.endgame and self.deadline and self.deadline.expired(self.now()):
                raise DeadlineHit()
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

    # ----------------------------------------------------------------- navigation
    def send_goal(self, pose, timeout):
        """One NavigateToPose attempt -> 'ok' | 'failed' | 'timeout' | 'rejected'."""
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(pose[0]), float(pose[1])
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = yaw_to_quat(pose[2])
        handle = None
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
            if not self.wait(res.done, timeout):
                return 'timeout'
            handle = None                                           # finished by itself
            return 'ok' if res.result().status == GoalStatus.STATUS_SUCCEEDED else 'failed'
        finally:
            if handle is not None:                                  # timeout, abort or deadline: cancel
                handle.cancel_goal_async()
                end = self.now() + 0.4
                while self.now() < end and rclpy.ok():
                    rclpy.spin_once(self, timeout_sec=0.05)

    def navigate(self, pose, label, timeout=None):
        """Goal with retry (after clearing costmaps) and skip -> 'ok' | 'skipped'."""
        policy = core.GoalPolicy(int(self.g('max_retries')))
        timeout = float(self.g('goal_timeout_s')) if timeout is None else timeout
        while True:
            res = self.send_goal(pose, timeout)
            action = policy.on_result(res)
            if action == 'done':
                return 'ok'
            self.get_logger().warn(f'{label}: {res} (attempt {policy.attempts}) -> {action}')
            if action == 'skip':
                return 'skipped'
            self.clear_costmaps()

    def run_stage(self, stage, timeout=None):
        results = []
        for name in self.stages.get(stage, []):
            r = self.navigate(self.wps[name], name, timeout)
            results.append(r)
            self.log_row(f'wp:{name}', r)
        return core.stage_result(results)

    # ----------------------------------------------------------------- state handlers
    def do_wait_start(self):
        self.get_logger().info('Waiting for /mission/start' + (' (auto_start)' if self.g('auto_start') else ''))
        self.log_row('WAIT_START', 'waiting')
        if not self.g('auto_start'):
            self.wait(lambda: self.start_req)
        self.deadline = core.Deadline(self.now(), float(self.g('mission_time_limit_s')))
        self.log_row('START', 'autonomous start')
        return 'started'

    def do_go_screen(self):
        self.set_speed(self.g('mission_speed'))
        return self.run_stage('screen')

    def do_classify(self):
        n = int(self.g('classify_retries'))
        base = self.wps[self.stages['screen'][-1]]
        for k in range(n + 1):
            if not self.classify_cli.wait_for_service(timeout_sec=1.0):
                detail = 'service missing'
            else:
                fut = self.classify_cli.call_async(Trigger.Request())
                if self.wait(fut.done, float(self.g('classify_timeout_s'))):
                    cls, conf, colour = core.parse_classify(fut.result().message)
                    detail = f'{cls}:{conf:.2f}:{colour}'
                    if core.usable_class(cls):
                        self.waste_class = cls
                        self.log_row('CLASSIFY_TRY', 'ok', detail)
                        return 'ok'
                else:
                    detail = 'timeout'
            self.log_row('CLASSIFY_TRY', 'unusable', f'{k + 1}/{n + 1} {detail}')
            if k < n:                                              # small reposition: wiggle the heading
                sign = 1 if k % 2 == 0 else -1
                dyaw = math.radians(float(self.g('reposition_yaw_deg'))) * sign
                self.navigate((base[0], base[1], base[2] + dyaw), 'reposition', 15.0)
        self.waste_class = str(self.g('default_class'))
        return 'default_class'

    def do_go_loading(self):
        return self.run_stage('loading')

    def do_load(self):
        """PLACEHOLDER loading protocol: wait load_wait_s or until /loading/done."""
        self.loading_done = False
        done = self.wait(lambda: self.loading_done, float(self.g('load_wait_s')))
        return 'done' if done else 'waited'

    def do_terrain(self):
        self.set_speed(self.g('terrain_speed'))
        self.set_surface(str(self.g('terrain_surface')))
        return self.run_stage('terrain')

    def do_obstacles(self):
        self.set_speed(self.g('mission_speed'))
        if self.g('terrain_surface'):
            self.set_surface('AUTO')
        return self.run_stage('obstacles')

    def do_junction(self):
        return self.run_stage('junction')

    def gate_free_now(self):
        s = self.scan
        if s is None:
            return None
        return core.gate_region_free(s.ranges, s.angle_min, s.angle_increment, float(self.g('gate_max_range_m')),
                                     math.radians(float(self.g('gate_half_angle_deg'))), max(s.range_min, 0.12))

    def do_gate_decision(self):
        d = core.GateDecider(self.now(), float(self.g('gate_wait_s')), float(self.g('gate_stable_s')))
        self.wait(lambda: d.update(self.now(), self.gate_free_now()) is not None)
        self.gate_decision = d.decision
        return d.decision

    def do_go_path(self):
        first = 'gate_path2' if self.gate_decision == 'path2' else 'path1_detour'
        r = self.run_stage(first)
        if r == 'skipped' and first == 'gate_path2':               # Path 2 unusable: fall back to the open path
            self.log_row('GO_PATH', 'path2 failed, trying path1')
            r = self.run_stage('path1_detour')
        return f'{first}:{r}'

    def do_deposit(self):
        bay = core.bay_for_class(self.waste_class, self.bay_map, str(self.g('default_class')))
        self.bay_colour = bay
        r = self.run_stage(core.bay_stage(bay))
        if r == 'skipped':
            return f'bay_{bay}:not_reached'
        self.pub_deposit.publish(Empty())                           # PLACEHOLDER deposit actuator hook
        self.pause(float(self.g('deposit_wait_s')))
        return f'bay_{bay}:{r}'

    def do_finish(self):
        self.endgame = True
        grace = float(self.g('finish_grace_s'))
        remaining = self.deadline.remaining(self.now()) if self.deadline else grace
        timeout = min(float(self.g('goal_timeout_s')), max(remaining, 0.0) + grace)
        r = self.run_stage('finish', timeout=timeout)
        self.stop_robot()
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

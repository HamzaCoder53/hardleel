#!/usr/bin/env python3
"""Autonomous frontier exploration for the 3-minute mapping phase (mapping_mode:=auto), over Nav2.

Loop: wait for /map + Nav2 action server + start_delay_s -> find frontiers (frontier_core.py) -> send the best
as a NavigateToPose goal -> every replan_period_s re-check (frontier gone or clearly better one -> new goal)
-> stop when no frontier is left, time_limit_s passes, or /exploration/stop is called.
Decision logic is in frontier_core.py (pure, unit tested); this file only does the ROS I/O.

Topics: in /map (OccupancyGrid), TF map->base_link; out /exploration/state (String: waiting/exploring/done),
        zero Twist on cmd_vel_topic (default /cmd_vel_nav) at the end.
Services: /exploration/stop (std_srvs/Trigger) ends it early.
Params: time_limit_s 165, goal_timeout_s 20, min_frontier_size_cells 8, replan_period_s 3, return_to_start false,
start_delay_s 2, min_goal_dist_m 0.5, clearance_m 0.25, blacklist_radius_m 0.4, switch_margin 25,
done_confirm_n 3, cmd_vel_topic, robot_frame (see config/frontier_explorer.yaml).
A failed/timed-out/rejected goal is blacklisted; reached goals go to the visited list.
"""
import math
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from robot_slam import frontier_core as fc


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        p = self.declare_parameter
        p('time_limit_s', 165.0)
        p('goal_timeout_s', 20.0)
        p('min_frontier_size_cells', 8)
        p('replan_period_s', 3.0)
        p('return_to_start', False)
        p('start_delay_s', 2.0)
        p('min_goal_dist_m', 0.5)
        p('clearance_m', 0.25)
        p('blacklist_radius_m', 0.4)
        p('switch_margin', 25.0)
        p('done_confirm_n', 3)
        p('cmd_vel_topic', '/cmd_vel_nav')
        p('robot_frame', 'base_link')
        self.g = lambda n: self.get_parameter(n).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.pub_state = self.create_publisher(String, '/exploration/state', 10)
        self.pub_cmd = self.create_publisher(Twist, str(self.g('cmd_vel_topic')), 1)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, 1)
        self.create_service(Trigger, '/exploration/stop', self.on_stop)
        self.create_timer(1.0, lambda: self.publish_state())
        self.grid = None
        self.state = 'waiting'
        self.stop_req = False
        self.blacklist, self.visited = [], []

    # ----------------------------------------------------------------- helpers
    def now(self):
        return time.monotonic()

    def publish_state(self):
        self.pub_state.publish(String(data=self.state))

    def on_map(self, m):
        self.grid = {'data': np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width),
                     'resolution': m.info.resolution,
                     'origin': (m.info.origin.position.x, m.info.origin.position.y)}

    def on_stop(self, _req, resp):
        self.stop_req = True
        resp.success, resp.message = True, 'stopping exploration'
        return resp

    def pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', str(self.g('robot_frame')), rclpy.time.Time())
        except TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y

    def spin(self, seconds):
        end = self.now() + seconds
        while self.now() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def stop_robot(self):
        for _ in range(5):
            self.pub_cmd.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)

    def plan(self, robot):
        """-> (frontiers, best Goal or None) from the latest map."""
        gr = self.grid
        fr = fc.find_frontiers(gr['data'], gr['resolution'], gr['origin'], int(self.g('min_frontier_size_cells')))
        goal = fc.choose_goal(fr, robot, self.visited, self.blacklist, float(self.g('min_goal_dist_m')),
                              grid=gr['data'], resolution=gr['resolution'], origin_xy=gr['origin'],
                              clearance_m=float(self.g('clearance_m')),
                              blacklist_radius=float(self.g('blacklist_radius_m')))
        return fr, goal

    def send(self, x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        return self.nav.send_goal_async(goal)

    def cancel(self, handle):
        if handle is not None:
            handle.cancel_goal_async()
            self.spin(0.4)

    # ----------------------------------------------------------------- main loop
    def run(self):
        log = self.get_logger()
        log.info('waiting for /map, TF and navigate_to_pose')
        while rclpy.ok() and not self.stop_req and (self.grid is None or self.pose() is None
                                                    or not self.nav.wait_for_server(timeout_sec=0.0)):
            rclpy.spin_once(self, timeout_sec=0.1)
        self.spin(float(self.g('start_delay_s')))
        t0 = self.now()
        limit = float(self.g('time_limit_s'))
        self.state = 'exploring'
        self.publish_state()
        log.info(f'exploration started (limit {limit:.0f} s)')

        send_fut = res_fut = handle = None
        goal = None                                   # current frontier_core.Goal
        goal_t = last_plan = 0.0
        empty_n, returning, reason = 0, False, 'done'
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            now = self.now()
            if self.stop_req:
                reason = 'stop requested'
                break
            if now - t0 >= limit:
                reason = 'time limit'
                break
            pose = self.pose()
            if pose is None or self.grid is None:
                continue

            if send_fut is not None and send_fut.done():           # goal accepted / rejected
                handle = send_fut.result()
                send_fut = None
                if handle is None or not handle.accepted:
                    log.warn('goal rejected, blacklisting')
                    self.blacklist.append((goal.x, goal.y))
                    if returning:
                        reason = 'return rejected'
                        break
                    handle = goal = None
                    last_plan = 0.0
                else:
                    res_fut = handle.get_result_async()
            if res_fut is not None and res_fut.done():             # finished by itself
                ok = res_fut.result().status == GoalStatus.STATUS_SUCCEEDED
                log.info(f'goal ({goal.x:.2f},{goal.y:.2f}) {"reached" if ok else "failed"}')
                (self.visited if ok else self.blacklist).append((goal.x, goal.y))
                if returning:
                    reason = 'returned to start'
                    break
                res_fut = handle = goal = None
                last_plan = 0.0                                    # replan immediately
            elif res_fut is not None and now - goal_t > float(self.g('goal_timeout_s')):
                log.warn(f'goal ({goal.x:.2f},{goal.y:.2f}) timeout, blacklisting')
                self.blacklist.append((goal.x, goal.y))
                self.cancel(handle)
                if returning:
                    reason = 'return timeout'
                    break
                res_fut = handle = goal = None
                last_plan = 0.0

            if returning or send_fut is not None:
                continue
            if last_plan > 0.0 and now - last_plan < float(self.g('replan_period_s')):
                continue
            last_plan = now
            frontiers, best = self.plan(pose)
            if goal is None and best is None:
                empty_n += 1
            elif fc.exploration_done(frontiers, int(self.g('min_frontier_size_cells'))):
                empty_n += 1
            else:
                empty_n = 0
            if empty_n >= int(self.g('done_confirm_n')):
                self.cancel(handle)
                if bool(self.g('return_to_start')) and math.hypot(*pose) > 0.5:
                    log.info('exploration complete, returning to start')
                    returning = True
                    goal = fc.Goal(0.0, 0.0, math.atan2(-pose[1], -pose[0]))
                    send_fut, res_fut, handle, goal_t = self.send(goal.x, goal.y, goal.yaw), None, None, now
                    continue
                reason = 'no frontiers left'
                break
            if best is None:
                continue
            if goal is not None:                                   # keep it unless gone or clearly worse
                gr = self.grid
                if fc.frontier_near(frontiers, goal.x, goal.y, gr['resolution'], gr['origin']):
                    cur = min(goal.frontier.size, 60) - 8.0 * math.hypot(goal.x - pose[0], goal.y - pose[1])
                    if best.score - cur < float(self.g('switch_margin')) or \
                            math.hypot(best.x - goal.x, best.y - goal.y) < 0.5:
                        continue
                self.cancel(handle)
                res_fut = handle = None
            goal = best
            goal_t = now
            log.info(f'goal ({goal.x:.2f},{goal.y:.2f}) size {goal.frontier.size} score {goal.score:.0f}; '
                     f'{len(frontiers)} frontiers')
            send_fut = self.send(goal.x, goal.y, goal.yaw)

        # ---- finish: cancel, stop, report
        if handle is None and send_fut is not None and send_fut.done():
            handle = send_fut.result()
        self.cancel(handle)
        self.stop_robot()
        self.state = 'done'
        self.publish_state()
        log.info(f'exploration finished ({reason}) after {self.now() - t0:.0f} s')


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        node.run()
        node.spin(1.0)                                # let /exploration/state go out
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

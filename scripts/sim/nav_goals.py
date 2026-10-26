#!/usr/bin/env python3
"""Virtual SLAM step 3: drive Nav2 through named waypoints and score the result against the simulator's truth.

    nav_goals.py GOAL [GOAL ...] [--waypoints FILE] [--out JSON] [--timeout S] [--start 0.06,0] [--teleport-after N]

Goals are pose names from the real mission waypoints file (frame map = start pose). For each goal: result status,
time, final distance to the goal (true pose), and the localization error (Cartographer map->base_link vs truth).
Truth = /ground_truth (world frame); map = world - start offset (the robot starts at world --start, facing east).
"""
import argparse
import json
import math
import os
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_WP = os.path.join(HERE, '../../ros2_ws/src/robot_slam/config/waypoints_course.yaml')


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Runner(Node):
    def __init__(self, start):
        super().__init__('nav_goals', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.start = start
        self.truth = None
        self.create_subscription(Odometry, '/ground_truth', self.on_truth, 10)
        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self)
        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def on_truth(self, m):
        p = m.pose.pose
        self.truth = (p.position.x - self.start[0], p.position.y - self.start[1], yaw_of(p.orientation))

    def spin(self, s):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def est(self):
        try:
            t = self.buf.lookup_transform('map', 'base_link', rclpy.time.Time()).transform
            return (t.translation.x, t.translation.y, yaw_of(t.rotation))
        except Exception:
            return None

    def loc_error(self):
        e = self.est()
        if e is None or self.truth is None:
            return None
        return math.hypot(e[0] - self.truth[0], e[1] - self.truth[1])

    def go(self, name, pose, timeout):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = pose['x'], pose['y']
        yaw = math.radians(pose.get('yaw_deg', 0.0))
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        t0 = time.time()
        fut = self.ac.send_goal_async(goal)
        while not fut.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        handle = fut.result()
        if not handle.accepted:
            return dict(goal=name, status='REJECTED')
        res = handle.get_result_async()
        worst = 0.0
        while not res.done() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            le = self.loc_error()
            worst = max(worst, le or 0.0)
        if not res.done():
            handle.cancel_goal_async()
            self.spin(1.0)
            status = 'TIMEOUT'
        else:
            status = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(res.result().status, str(res.result().status))
        self.spin(0.5)
        d = math.hypot(self.truth[0] - pose['x'], self.truth[1] - pose['y']) if self.truth else None
        return dict(goal=name, status=status, seconds=round(time.time() - t0, 1),
                    true_dist_to_goal_m=None if d is None else round(d, 3),
                    loc_error_end_m=None if self.loc_error() is None else round(self.loc_error(), 3),
                    loc_error_worst_m=round(worst, 3))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('goals', nargs='+')
    ap.add_argument('--waypoints', default=DEFAULT_WP)
    ap.add_argument('--out', default=None)
    ap.add_argument('--timeout', type=float, default=120.0)
    ap.add_argument('--start', default='0.06,0.0')
    ap.add_argument('--teleport-after', type=int, default=0, help='after goal N: teleport the robot (recovery test)')
    ap.add_argument('--teleport-to', default='-2.0,-2.07,180', help='x,y,yaw_deg in the MAP frame')
    a = ap.parse_args()
    poses = yaml.safe_load(open(a.waypoints))['poses']
    start = [float(v) for v in a.start.split(',')]
    rclpy.init()
    n = Runner(start)
    print('waiting for navigate_to_pose ...', flush=True)
    n.ac.wait_for_server()
    n.spin(3.0)
    results = []
    for i, g in enumerate(a.goals, 1):
        r = n.go(g, poses[g], a.timeout)
        print(json.dumps(r), flush=True)
        results.append(r)
        if a.teleport_after and i == a.teleport_after:
            from teleport import teleport
            tx, ty, tyaw = [float(v) for v in a.teleport_to.split(',')]
            teleport(tx + start[0], ty + start[1], math.radians(tyaw))
            n.spin(1.0)
            print('TELEPORTED to', a.teleport_to, 'est', n.est(), 'truth', n.truth, flush=True)
    ok = sum(r['status'] == 'SUCCEEDED' for r in results)
    summary = dict(succeeded=ok, total=len(results), results=results)
    if a.out:
        json.dump(summary, open(a.out, 'w'), indent=2)
    print(f'NAV SUMMARY {ok}/{len(results)} succeeded')
    return 0 if ok == len(results) else 1


if __name__ == '__main__':
    raise SystemExit(main())

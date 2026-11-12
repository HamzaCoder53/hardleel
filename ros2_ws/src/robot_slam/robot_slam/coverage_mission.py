#!/usr/bin/env python3
"""Deterministic coverage mission (Mode C): drive a fixed waypoint list through Nav2.

Waypoints come from the `waypoints` parameter (flat [x1, y1, x2, y2, ...] in the map frame) or, when that
is empty, from a boustrophedon sweep of the arena rectangle. Rectangles in `keepout` (flat
[xmin, ymin, xmax, ymax, ...], e.g. ramps and the gate) are grown by `standoff` and any waypoint inside
one is dropped, so the robot never targets a point closer than the standoff to them.

Speed regimes: 'mapping' (0.18 m/s) and 'mission' (0.30 m/s), applied through Nav2's speed_limit topic.
A failed or aborted goal is retried (Nav2 replans on every request) after clearing both costmaps; a goal
that keeps failing is skipped. The map-frame pose is logged at every waypoint to a CSV file.
"""
import math
import os
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit
from nav2_msgs.srv import ClearEntireCostmap
from tf2_ros import Buffer, TransformListener, TransformException


def generate_boustrophedon(width, height, lane_spacing, margin, origin=(0.0, 0.0)):
    """Lane end points of a sweep over a width x height rectangle: [(x, y, yaw), ...]."""
    ox, oy = origin
    x0, x1 = ox + margin, ox + width - margin
    y0, y1 = oy + margin, oy + height - margin
    if x1 <= x0 or y1 <= y0:
        raise ValueError('margin leaves no room inside the arena')
    n_lanes = max(1, int(math.floor((y1 - y0) / lane_spacing)) + 1)
    pts = []
    for i in range(n_lanes):
        y = y0 + i * (y1 - y0) / max(1, n_lanes - 1) if n_lanes > 1 else (y0 + y1) / 2.0
        forward = (i % 2 == 0)
        xa, xb = (x0, x1) if forward else (x1, x0)
        yaw = 0.0 if forward else math.pi
        pts.append((xa, y, yaw))
        pts.append((xb, y, yaw))
    return pts


def drop_keepout(points, keepout, standoff):
    """Remove waypoints inside any keepout rectangle grown by standoff."""
    boxes = [(keepout[i] - standoff, keepout[i + 1] - standoff, keepout[i + 2] + standoff, keepout[i + 3] + standoff)
             for i in range(0, len(keepout) - 3, 4)]
    return [p for p in points
            if not any(b[0] <= p[0] <= b[2] and b[1] <= p[1] <= b[3] for b in boxes)]


def yaw_to_quat(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)       # (z, w)


class CoverageMission(Node):
    def __init__(self):
        super().__init__('coverage_mission')
        p = self.declare_parameter
        p('regime', 'mapping')                 # 'mapping' | 'mission'
        p('mapping_speed', 0.18); p('mission_speed', 0.30)
        p('waypoints', [0.0]); p('keepout', [0.0])    # [0.0] = unset (an empty array cannot be typed)
        p('arena_width', 4.9); p('arena_height', 2.4)
        p('arena_origin_x', 0.0); p('arena_origin_y', 0.0)
        p('lane_spacing', 0.6)
        p('margin', 0.5)
        p('standoff', 0.5)                     # must stay above the inflation radius (0.35 / 0.45)
        p('max_retries', 2)
        p('goal_timeout_s', 120.0)
        p('log_dir', '~/logs')
        g = lambda n: self.get_parameter(n).value
        self.regime = str(g('regime'))
        self.speed = float(g('mission_speed' if self.regime == 'mission' else 'mapping_speed'))
        self.retries = int(g('max_retries'))
        self.timeout = float(g('goal_timeout_s'))

        wp = list(g('waypoints'))
        if len(wp) >= 2 and len(wp) % 2 == 0:
            pts = [(wp[i], wp[i + 1], None) for i in range(0, len(wp), 2)]
        else:
            pts = generate_boustrophedon(float(g('arena_width')), float(g('arena_height')),
                                         float(g('lane_spacing')), float(g('margin')),
                                         (float(g('arena_origin_x')), float(g('arena_origin_y'))))
        keep = list(g('keepout'))
        self.points = drop_keepout(pts, keep, float(g('standoff'))) if len(keep) >= 4 else pts
        # waypoints given without a heading face the next waypoint
        self.points = [self.with_heading(i) for i in range(len(self.points))]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.pub_limit = self.create_publisher(SpeedLimit, 'speed_limit', 1)
        self.clear_srvs = [self.create_client(ClearEntireCostmap, n) for n in
                           ('/global_costmap/clear_entirely_global_costmap',
                            '/local_costmap/clear_entirely_local_costmap')]
        log_dir = os.path.expanduser(str(g('log_dir')))
        os.makedirs(log_dir, exist_ok=True)
        self.log_path = os.path.join(log_dir, time.strftime('coverage_%Y%m%d_%H%M%S.csv'))
        with open(self.log_path, 'w') as f:
            f.write('t_s,index,target_x,target_y,status,retries,x,y,yaw_deg\n')
        self.t0 = time.time()
        self.get_logger().info(
            f'{len(self.points)} waypoints, regime={self.regime} ({self.speed} m/s), log {self.log_path}')

    def with_heading(self, i):
        x, y, yaw = self.points[i]
        if yaw is None:
            nxt = self.points[i + 1] if i + 1 < len(self.points) else self.points[i - 1] if i > 0 else None
            yaw = math.atan2(nxt[1] - y, nxt[0] - x) if nxt and (nxt[0], nxt[1]) != (x, y) else 0.0
            if i + 1 >= len(self.points) and nxt:
                yaw = math.atan2(y - nxt[1], x - nxt[0])
        return (x, y, yaw)

    def pose_now(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except TransformException:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, math.degrees(yaw)

    def log_row(self, idx, wp, status, retries):
        pose = self.pose_now() or (float('nan'),) * 3
        with open(self.log_path, 'a') as f:
            f.write(f'{time.time() - self.t0:.1f},{idx},{wp[0]:.3f},{wp[1]:.3f},{status},{retries},'
                    f'{pose[0]:.3f},{pose[1]:.3f},{pose[2]:.1f}\n')

    def set_speed_limit(self):
        msg = SpeedLimit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = False
        msg.speed_limit = self.speed
        self.pub_limit.publish(msg)

    def clear_costmaps(self):
        for c in self.clear_srvs:
            if c.wait_for_service(timeout_sec=1.0):
                rclpy.spin_until_future_complete(self, c.call_async(ClearEntireCostmap.Request()), timeout_sec=2.0)

    def go(self, wp):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(wp[0]), float(wp[1])
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = yaw_to_quat(wp[2])
        send = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            return 'rejected'
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=self.timeout)
        if not res.done():
            handle.cancel_goal_async()
            return 'timeout'
        return 'ok' if res.result().status == GoalStatus.STATUS_SUCCEEDED else 'aborted'

    def run(self):
        self.get_logger().info('Waiting for Nav2 (navigate_to_pose)...')
        while rclpy.ok() and not self.client.wait_for_server(timeout_sec=2.0):
            pass
        for i, wp in enumerate(self.points):
            if not rclpy.ok():
                return
            status, used = 'failed', 0
            for attempt in range(self.retries + 1):
                self.set_speed_limit()
                status = self.go(wp)
                used = attempt
                if status == 'ok':
                    break
                self.get_logger().warn(f'Waypoint {i} {status}; replanning ({attempt + 1}/{self.retries}).')
                self.clear_costmaps()
            self.log_row(i, wp, status, used)
            self.get_logger().info(f'Waypoint {i + 1}/{len(self.points)} {status}')
        self.get_logger().info('Coverage mission finished.')


def main(args=None):
    rclpy.init(args=args)
    node = CoverageMission()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()

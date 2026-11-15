#!/usr/bin/env python3
"""Frontier exploration for auto mapping (laptop, with Nav2 navigation on Cartographer's live /map).

A frontier is a free map cell next to unknown space. Every plan_period the explorer clusters frontier
cells, drops clusters that are too small / too close / near a goal that failed before, scores the rest
(size_weight * size - distance_weight * distance) and sends the best one to Nav2 (NavigateToPose).
A goal is cancelled when it times out or when its area has become known on the way; failed goals are
blacklisted. With no frontiers left: status 'complete' and, if return_home, a goal back to the start.

Starts DISABLED: start/stop with the tuning GUI's Auto tab, `ros2 param set /frontier_explorer enabled true`
or a String 'start' / 'stop' / 'home' on /explore/command. All parameters can be changed live.
Publishes /explore/status (String) and /explore/frontiers (MarkerArray for RViz).
"""
import math
import time
from collections import deque

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

# name: (default, description, (min, max) or None)
PARAMS = {
    'enabled': (False, 'explore now (start/stop); starts OFF for safety', None),
    'plan_period': (2.0, 's between frontier searches', (0.5, 30.0)),
    'min_frontier_size': (8, 'smallest frontier cluster (cells) worth a goal', (1, 500)),
    'min_goal_distance': (0.4, 'ignore frontiers closer than this to the robot (m)', (0.0, 5.0)),
    'size_weight': (1.0, 'score += size_weight * cluster size (cells)', (0.0, 100.0)),
    'distance_weight': (20.0, 'score -= distance_weight * distance (m): higher = prefer near frontiers', (0.0, 1000.0)),
    'goal_timeout': (45.0, 's before a goal is given up and blacklisted', (5.0, 600.0)),
    'blacklist_radius': (0.4, 'frontiers this close to a failed goal are skipped (m)', (0.05, 3.0)),
    'explored_radius': (0.25, 'cancel the goal early when no unknown cells are left this close to it (m)', (0.0, 2.0)),
    'free_threshold': (25, 'map cells below this occupancy (0-100) count as free', (0, 100)),
    'return_home': (True, 'drive back to the start (map 0,0) when exploration is complete', None),
}
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        for name, (default, desc, rng) in PARAMS.items():
            self.declare_parameter(name, default, ParameterDescriptor(
                description=desc + (f'  [{rng[0]}..{rng[1]}]' if rng else '')))
        if self.p('enabled'):   # never start driving on launch, even if 'enabled: true' was saved
            self.set_parameters([Parameter('enabled', value=False)])
            self.get_logger().warn('enabled was true at start - forced OFF; press Start to explore')
        self.add_on_set_parameters_callback(self.on_params)
        self.map = None
        self.goal_handle = None     # None, 'pending' (sent, not accepted yet) or the Nav2 goal handle
        self.token = None           # identifies the current goal; callbacks of older goals are ignored
        self.goal_xy = None
        self.goal_sent = 0.0
        self.going_home = False
        self.blacklist = []
        self.last_plan = 0.0
        self.state = 'idle (disabled)'
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # VOLATILE + RELIABLE matches both volatile and transient-local /map publishers
        self.create_subscription(OccupancyGrid, '/map', self.on_map,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(String, '/explore/command', self.on_command, 10)
        self.status_pub = self.create_publisher(String, '/explore/status', LATCHED)
        self.marker_pub = self.create_publisher(MarkerArray, '/explore/frontiers', 1)
        self.create_timer(0.5, self.loop)
        self.set_state('idle (disabled) - press Start in the tuning GUI Auto tab')

    # ---- parameters / commands -----------------------------------------------------------------
    def p(self, name):
        return self.get_parameter(name).value

    def on_params(self, params):
        for prm in params:
            spec = PARAMS.get(prm.name)
            if spec and spec[2] and not spec[2][0] <= prm.value <= spec[2][1]:
                return SetParametersResult(successful=False, reason=f'{prm.name} must be {spec[2][0]}..{spec[2][1]}')
        for prm in params:
            if prm.name == 'enabled':
                if prm.value:
                    self.blacklist.clear()
                    self.going_home = False
                    self.last_plan = 0.0
                    self.set_state('exploring: looking for frontiers')
                else:
                    self.cancel_goal()
                    self.set_state('stopped')
        return SetParametersResult(successful=True)

    def on_command(self, msg):
        cmd = msg.data.strip().lower()
        if cmd in ('start', 'stop'):
            self.set_parameters([Parameter('enabled', value=cmd == 'start')])
        elif cmd == 'home':
            self.set_parameters([Parameter('enabled', value=False)])
            self.go_home()

    def set_state(self, text):
        if text != self.state:
            self.get_logger().info(text)
        self.state = text
        self.status_pub.publish(String(data=text))

    # ---- map / pose ----------------------------------------------------------------------------
    def on_map(self, msg):
        self.map = msg

    def robot_xy(self):
        try:
            t = self.tf.lookup_transform('map', 'base_link', Time())
            return t.transform.translation.x, t.transform.translation.y
        except Exception:  # noqa: BLE001 - TF not there yet
            return None

    def grid(self):
        m = self.map
        return np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width), m.info

    @staticmethod
    def cell_to_xy(info, r, c):
        return (info.origin.position.x + (c + 0.5) * info.resolution,
                info.origin.position.y + (r + 0.5) * info.resolution)

    def frontier_clusters(self, g):
        free = (g >= 0) & (g < self.p('free_threshold'))
        unknown = g < 0
        near_unknown = np.zeros_like(unknown)
        near_unknown[1:, :] |= unknown[:-1, :]
        near_unknown[:-1, :] |= unknown[1:, :]
        near_unknown[:, 1:] |= unknown[:, :-1]
        near_unknown[:, :-1] |= unknown[:, 1:]
        frontier = free & near_unknown
        seen = np.zeros_like(frontier)
        clusters = []
        h, w = frontier.shape
        for r0, c0 in zip(*np.nonzero(frontier)):
            if seen[r0, c0]:
                continue
            seen[r0, c0] = True
            cells, q = [], deque([(r0, c0)])
            while q:
                r, c = q.popleft()
                cells.append((r, c))
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        rr, cc = r + dr, c + dc
                        if 0 <= rr < h and 0 <= cc < w and frontier[rr, cc] and not seen[rr, cc]:
                            seen[rr, cc] = True
                            q.append((rr, cc))
            clusters.append(cells)
        return clusters

    def unknown_near(self, g, info, xy, radius):
        if radius <= 0:
            return True
        c = int((xy[0] - info.origin.position.x) / info.resolution)
        r = int((xy[1] - info.origin.position.y) / info.resolution)
        k = max(1, int(radius / info.resolution))
        win = g[max(0, r - k):r + k + 1, max(0, c - k):c + k + 1]
        return bool((win < 0).any())

    # ---- main loop -----------------------------------------------------------------------------
    def loop(self):
        if self.goal_handle is not None:
            self.check_goal()
            return
        if not self.p('enabled') or self.map is None:
            return
        if time.monotonic() - self.last_plan < self.p('plan_period'):
            return
        self.last_plan = time.monotonic()
        pos = self.robot_xy()
        if pos is None:
            self.set_state('exploring: waiting for map -> base_link (Cartographer running?)')
            return
        if not self.nav.server_is_ready():
            self.set_state('exploring: waiting for Nav2 (navigate_to_pose)')
            return
        g, info = self.grid()
        best, best_score, shown = None, -math.inf, []
        for cells in self.frontier_clusters(g):
            if len(cells) < self.p('min_frontier_size'):
                continue
            rows, cols = zip(*cells)
            cr, cc = sum(rows) / len(rows), sum(cols) / len(cols)
            r, c = min(cells, key=lambda rc: (rc[0] - cr) ** 2 + (rc[1] - cc) ** 2)  # cell nearest the centroid
            xy = self.cell_to_xy(info, r, c)
            dist = math.hypot(xy[0] - pos[0], xy[1] - pos[1])
            if dist < self.p('min_goal_distance'):
                continue
            if any(math.hypot(xy[0] - bx, xy[1] - by) < self.p('blacklist_radius') for bx, by in self.blacklist):
                continue
            score = self.p('size_weight') * len(cells) - self.p('distance_weight') * dist
            shown.append((xy, len(cells)))
            if score > best_score:
                best, best_score = xy, score
        self.publish_markers(shown, best)
        if best is None:
            self.set_parameters([Parameter('enabled', value=False)])
            if self.p('return_home'):
                self.set_state('exploration COMPLETE - returning to the start')
                self.go_home()
            else:
                self.set_state('exploration COMPLETE - save the map now')
            return
        self.send_goal(best, pos, f'exploring: going to frontier at ({best[0]:.2f}, {best[1]:.2f}), '
                                  f'{len(shown)} frontier(s) left')

    # ---- goals ---------------------------------------------------------------------------------
    def send_goal(self, xy, pos, state):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = xy
        yaw = math.atan2(xy[1] - pos[1], xy[0] - pos[0]) if pos else 0.0
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.goal_xy, self.goal_sent = xy, time.monotonic()
        token = object()
        self.token, self.goal_handle = token, 'pending'
        self.nav.send_goal_async(goal).add_done_callback(lambda f: self.on_goal_accepted(f, token))
        self.set_state(state)

    def on_goal_accepted(self, fut, token):
        handle = fut.result()
        if token is not self.token:           # stopped/replaced while Nav2 was accepting it
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.goal_failed('Nav2 rejected the goal')
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self.on_result(f, token))

    def on_result(self, fut, token):
        if token is not self.token:           # result of an older, cancelled goal
            return
        status = fut.result().status
        was_home = self.going_home
        self.token, self.goal_handle = None, None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.going_home = False
            self.set_state('back at the start - save the map now' if was_home else 'reached frontier')
        elif status == GoalStatus.STATUS_CANCELED:
            pass  # cancelled on purpose (explored / timeout / stop) - state already set
        else:
            self.goal_failed(f'goal failed (status {status})')

    def goal_failed(self, why):
        self.token, self.goal_handle = None, None
        if self.going_home:
            self.going_home = False
            self.set_state(f'return home failed: {why}')
            return
        if self.goal_xy:
            self.blacklist.append(self.goal_xy)
        self.set_state(f'exploring: {why} - blacklisted, picking another frontier')

    def check_goal(self):
        if self.goal_handle == 'pending':
            if time.monotonic() - self.goal_sent > 10.0:
                self.goal_failed('Nav2 did not accept the goal within 10 s')
            return
        if not self.going_home and not self.p('enabled'):
            self.cancel_goal()
            return
        if time.monotonic() - self.goal_sent > self.p('goal_timeout'):
            if self.goal_xy and not self.going_home:
                self.blacklist.append(self.goal_xy)
            self.cancel_goal()
            self.set_state('exploring: goal timed out - blacklisted')
            return
        if not self.going_home and self.map is not None and self.goal_xy:
            g, info = self.grid()
            if not self.unknown_near(g, info, self.goal_xy, self.p('explored_radius')):
                self.cancel_goal()
                self.last_plan = 0.0
                self.set_state('exploring: goal area already mapped - next frontier')

    def cancel_goal(self):
        h = self.goal_handle
        self.token, self.goal_handle = None, None   # a still-pending goal is cancelled once accepted
        if h is not None and h != 'pending':
            h.cancel_goal_async()

    def go_home(self):
        if not self.nav.server_is_ready():
            self.set_state('cannot return home: Nav2 not ready')
            return
        self.cancel_goal()
        self.going_home = True
        self.send_goal((0.0, 0.0), self.robot_xy(), 'returning to the start (map 0, 0)')

    def publish_markers(self, frontiers, best):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for i, (xy, n) in enumerate(frontiers):
            m = Marker()
            m.header.frame_id = 'map'
            m.ns, m.id, m.type, m.action = 'frontiers', i + 1, Marker.SPHERE, Marker.ADD
            m.pose.position.x, m.pose.position.y = xy
            m.pose.orientation.w = 1.0
            s = min(0.4, 0.08 + 0.01 * n)
            m.scale.x = m.scale.y = m.scale.z = s
            is_best = best is not None and xy == best
            m.color.r, m.color.g, m.color.b, m.color.a = (0.1, 0.9, 0.1, 1.0) if is_best else (0.2, 0.5, 1.0, 0.8)
            arr.markers.append(m)
        self.marker_pub.publish(arr)


def main():
    rclpy.init()
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cancel_goal()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

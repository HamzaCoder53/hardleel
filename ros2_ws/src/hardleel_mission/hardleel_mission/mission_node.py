#!/usr/bin/env python3
"""hardleel mission: behaviour tree for waste sorting (23 Nov 2026).

  Mission (Sequence)
    WaitForStart                       auto_start, or a True on /mission/start
    ForEach waste item (pickups)       a failed item is skipped, the mission goes on
      HandleItem (Fallback)
        DeliverItem (Sequence)
          BatteryOk                    /battery/voltage (Float32) above min_battery_v (no sensor yet = OK)
          Retry  NavigateTo item       Nav2 NavigateToPose (map frame)
          Retry  Classify              /classify_waste (std_srvs/Trigger) -> "blue" | "white" | "black"
          Servo  grab                  /servo/cmd (std_msgs/String) - the Nano servo firmware listens
          Retry  NavigateTo bin[colour]
          Servo  release
        GiveUp (Sequence)              release whatever is held, log it, continue with the next item
    Retry  NavigateTo home

Positions, bins, retries and timings: config/mission.yaml (parameter mission_file). dry_run:=true simulates
Nav2, the camera and the servo (test the logic on the laptop without the robot).
Topics: /mission/status (String, one line), /mission/tree (String, whole tree with states, 1 Hz).

usage:  ros2 run hardleel_mission mission --ros-args -p mission_file:=<yaml> [-p dry_run:=true]
        ros2 topic pub --once /mission/start std_msgs/msg/Bool "{data: true}"     (when auto_start is false)
        ros2 run hardleel_mission where      prints the robot's current map pose as a YAML line for mission.yaml
"""
import math
import os
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger

from hardleel_mission.bt import (FAILURE, RUNNING, SUCCESS, Action, Blackboard, Condition, Fallback,
                                 ForEach, Retry, Sequence, Timeout)


# ---------------------------------------------------------------------------------------------- leaves
class NavigateTo(Action):
    """Drive to a pose with Nav2. target(bb) -> {'x', 'y', 'yaw'} (map frame) or None (= failure)."""
    def __init__(self, name, node, target):
        super().__init__(name)
        self.node, self.target = node, target
        self.goal_handle = self.result = self.t0 = None

    def start(self, bb):
        pose = self.target(bb)
        if pose is None:
            bb.log(f'{self.name}: no target pose')
            return FAILURE
        self.result, self.t0, self.goal_handle = None, time.monotonic(), None
        bb.log(f'{self.name}: -> x {pose["x"]:.2f} y {pose["y"]:.2f} yaw {pose.get("yaw", 0.0):.2f}')
        if self.node.dry_run:
            return RUNNING
        if not self.node.nav.wait_for_server(timeout_sec=2.0):
            bb.log(f'{self.name}: Nav2 navigate_to_pose server not available')
            return FAILURE
        goal = NavigateToPose.Goal()
        goal.pose = self.node.pose_msg(pose)
        fut = self.node.nav.send_goal_async(goal)
        fut.add_done_callback(self._accepted)
        return RUNNING

    def _accepted(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.result = GoalStatus.STATUS_ABORTED
            return
        self.goal_handle = gh
        gh.get_result_async().add_done_callback(lambda f: setattr(self, 'result', f.result().status))

    def update(self, bb):
        if self.node.dry_run:
            return SUCCESS if time.monotonic() - self.t0 > self.node.cfg['dry_run_nav_s'] else RUNNING
        if self.result is None:
            return RUNNING
        ok = self.result == GoalStatus.STATUS_SUCCEEDED
        bb.log(f'{self.name}: {"arrived" if ok else "FAILED (status %d)" % self.result} '
               f'after {time.monotonic() - self.t0:.1f} s')
        return SUCCESS if ok else FAILURE

    def stop(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()


class Classify(Action):
    """Ask the AI camera service which bin the item in front of the robot belongs to -> bb['colour']."""
    def __init__(self, name, node):
        super().__init__(name)
        self.node, self.fut, self.t0 = node, None, None

    def start(self, bb):
        self.t0 = time.monotonic()
        if self.node.dry_run:
            return RUNNING
        if not self.node.classify.wait_for_service(timeout_sec=2.0):
            bb.log(f'{self.name}: /classify_waste service not available')
            return FAILURE
        self.fut = self.node.classify.call_async(Trigger.Request())
        return RUNNING

    def update(self, bb):
        if self.node.dry_run:
            if time.monotonic() - self.t0 < 1.0:
                return RUNNING
            fake = self.node.cfg['dry_run_colours']
            colour = fake[self.node.fake_i % len(fake)]
            self.node.fake_i += 1
        else:
            if not self.fut.done():
                return RUNNING
            colour = (self.fut.result().message or '').strip().lower()
        bins = self.node.cfg['bins']
        bb.log(f'{self.name}: camera says "{colour}"')
        if colour not in bins:
            return FAILURE                     # "none" / "unknown": retried by the parent Retry
        bb['colour'] = colour
        return SUCCESS


class Servo(Action):
    """Send a servo command (e.g. "grab" / "release") and give the mechanism time to move."""
    def __init__(self, name, node, command):
        super().__init__(name)
        self.node, self.command, self.t0 = node, command, None

    def start(self, bb):
        self.t0 = time.monotonic()
        self.node.servo_pub.publish(String(data=self.command))
        bb.log(f'{self.name}: servo "{self.command}"')
        return RUNNING

    def update(self, bb):
        return SUCCESS if time.monotonic() - self.t0 >= self.node.cfg['servo_wait_s'] else RUNNING


class WaitForStart(Action):
    def __init__(self, node):
        super().__init__('WaitForStart')
        self.node = node

    def start(self, bb):
        if self.node.cfg['auto_start'] or self.node.start_requested:
            return SUCCESS
        bb.log('waiting for the start signal: ros2 topic pub --once /mission/start std_msgs/msg/Bool "{data: true}"')
        return RUNNING

    def update(self, bb):
        return SUCCESS if self.node.start_requested else RUNNING


class Log(Action):
    """Log a line; returns `result` (FAILURE for GiveUp so the item counts as not delivered)."""
    def __init__(self, name, fn, result=SUCCESS):
        super().__init__(name)
        self.fn, self.result = fn, result

    def start(self, bb):
        bb.log(self.fn(bb))
        return self.result


# ------------------------------------------------------------------------------------------------ node
DEFAULTS = {
    'auto_start': False,
    'tick_hz': 10.0,
    'nav_retries': 2,
    'nav_timeout_s': 90.0,
    'classify_retries': 3,
    'servo_wait_s': 1.0,
    'min_battery_v': 6.8,
    'dry_run_nav_s': 2.0,
    'dry_run_colours': ['blue', 'none', 'white', 'black'],
    'home': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
    'pickups': [],
    'bins': {},
}


class MissionNode(Node):
    def __init__(self):
        super().__init__('mission')
        share = get_package_share_directory('hardleel_mission')
        self.declare_parameter('mission_file', os.path.join(share, 'config', 'mission.yaml'))
        self.declare_parameter('dry_run', False)
        self.dry_run = self.get_parameter('dry_run').value
        path = self.get_parameter('mission_file').value
        with open(path) as f:
            self.cfg = {**DEFAULTS, **(yaml.safe_load(f) or {})}
        self.get_logger().info(f'mission file {path}: {len(self.cfg["pickups"])} item(s), bins '
                               f'{sorted(self.cfg["bins"])}{"  [DRY RUN]" if self.dry_run else ""}')

        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.classify = self.create_client(Trigger, '/classify_waste')
        self.servo_pub = self.create_publisher(String, '/servo/cmd', 10)
        self.status_pub = self.create_publisher(String, '/mission/status', 10)
        self.tree_pub = self.create_publisher(String, '/mission/tree', 10)
        self.battery_v, self.start_requested, self.fake_i = None, False, 0
        self.create_subscription(Float32, '/battery/voltage', lambda m: setattr(self, 'battery_v', m.data), 10)
        self.create_subscription(Bool, '/mission/start', self.on_start, 10)

        self.bb = Blackboard(logger=self.get_logger().info, pickups=self.cfg['pickups'])
        self.tree = self.build_tree()
        self.done, self.last_tree_pub = False, 0.0
        self.create_timer(1.0 / self.cfg['tick_hz'], self.tick)

    def on_start(self, msg):
        if msg.data and not self.start_requested:
            self.get_logger().info('start signal received')
        self.start_requested = self.start_requested or msg.data

    def pose_msg(self, p):
        m = PoseStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y = float(p['x']), float(p['y'])
        yaw = float(p.get('yaw', 0.0))
        m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return m

    def battery_ok(self, bb):
        if self.battery_v is None:             # battery sensor not wired / not publishing yet
            return True
        if self.battery_v < self.cfg['min_battery_v']:
            bb.log(f'battery LOW {self.battery_v:.2f} V < {self.cfg["min_battery_v"]} V - not starting this item')
            return False
        return True

    def build_tree(self):
        c = self.cfg

        def nav(name, target):
            return Retry(f'retry {name}', c['nav_retries'],
                         Timeout(f'timeout {name}', c['nav_timeout_s'], NavigateTo(name, self, target)))

        deliver = Sequence('DeliverItem', [
            Condition('BatteryOk', self.battery_ok),
            nav('NavigateTo item', lambda bb: bb['item']),
            Retry('retry Classify', c['classify_retries'], Classify('Classify', self)),
            Servo('Servo grab', self, 'grab'),
            nav('NavigateTo bin', lambda bb: c['bins'].get(bb.get('colour'))),
            Servo('Servo release', self, 'release'),
            Log('Delivered', lambda bb: f'item {bb["item"].get("name", "?")} delivered to the {bb["colour"]} bin'),
        ])
        give_up = Sequence('GiveUp', [
            Servo('Servo release (give up)', self, 'release'),
            Log('Skipped', lambda bb: f'item {bb["item"].get("name", "?")} SKIPPED - continuing', FAILURE),
        ])
        return Sequence('Mission', [
            WaitForStart(self),
            ForEach('ForEach waste item', 'pickups', 'item',
                    Fallback('HandleItem', [deliver, give_up])),
            nav('NavigateTo home', lambda bb: c['home']),
            Log('Finished', lambda bb: 'MISSION COMPLETE: ' + ', '.join(
                f'{i.get("name", "?")}={s}' for i, s in bb.get('results', []))),
        ])

    def tick(self):
        if self.done:
            return
        s = self.tree.tick(self.bb)
        running = [n for n in self.walk(self.tree) if n.status == RUNNING and not n.children()]
        self.status_pub.publish(String(data=f'{s}: ' + (running[0].name if running else '-')))
        now = time.monotonic()
        if now - self.last_tree_pub > 1.0 or s != RUNNING:
            self.last_tree_pub = now
            self.tree_pub.publish(String(data='\n'.join(self.tree.describe())))
        if s != RUNNING:
            self.done = True
            self.get_logger().info(f'mission ended: {s}\n' + '\n'.join(self.tree.describe()))

    def walk(self, n):
        yield n
        for c in n.children():
            yield from self.walk(c)


def main():
    rclpy.init()
    node = MissionNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.3)                        # let the last status messages go out
    except KeyboardInterrupt:
        pass
    finally:
        node.tree.reset()                      # cancels a running Nav2 goal
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def where():
    """Print the robot's current pose in the map as a line to paste into mission.yaml."""
    import tf2_ros
    rclpy.init()
    node = rclpy.create_node('mission_where')
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    end = time.monotonic() + 5.0
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            t = buf.lookup_transform('map', 'base_link', rclpy.time.Time()).transform
        except Exception:  # noqa: BLE001 - not available yet
            continue
        q = t.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        print(f'{{x: {t.translation.x:.2f}, y: {t.translation.y:.2f}, yaw: {yaw:.2f}}}')
        break
    else:
        print('no map -> base_link transform (is Nav2/AMCL or SLAM running?)')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

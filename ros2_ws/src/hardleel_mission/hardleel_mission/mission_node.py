#!/usr/bin/env python3
"""hardleel mission: behaviour tree for the waste-sorting run (23 Nov 2026, user's mission description).

  Mission (Sequence)
    WaitForStart                       auto_start, or a True on /mission/start
    ForEach round (rounds)             a failed round releases what it holds and the mission goes on
      Round (Fallback)
        DoRound (Sequence)
          BatteryOk                    /battery/voltage (Float32) above min_battery_v (no sensor yet = OK)
          Retry  NavigateTo camera     Nav2 NavigateToPose to the marked place
          Retry  Classify              AI camera /classify_waste (std_srvs/Trigger) -> "blue" | "white" | "black"
          Retry  NavigateTo collect
          Servo  grab                  /servo/cmd (std_msgs/String) - the Nano servo firmware listens
          Retry  NavigateTo gate       wait spot, robot FACING the timed gate
          WaitForGate                  lidar straight ahead clear (> gate_open_m) for gate_confirm_s
          Retry  NavigateTo <colour>   dump place of the camera's colour
          Servo  release
        GiveUp (Sequence)              release, log, next round
    NavigateTo home                    only if a "home" place is marked

Places (checkpoints): ~/.hardleel/mission_places.yaml, written by  ros2 run hardleel_mission mark <name>
(see places.py). Settings: config/mission.yaml (parameter mission_file). dry_run:=true simulates Nav2, the
camera, the servo and the gate (test the logic without the robot).
Topics: /mission/status (String, one line), /mission/tree (String, whole tree with states, 1 Hz).

usage:  ros2 launch hardleel_mission mission.launch.py [dry_run:=true]
        ros2 topic pub --once /mission/start std_msgs/msg/Bool "{data: true}"     (when auto_start is false)
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
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger

from hardleel_mission import places as places_mod
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
        bb.log(f'{self.name}: camera says "{colour}"')
        if colour not in self.node.cfg['colours']:
            return FAILURE                     # "none" / "unknown": retried by the parent Retry
        if colour not in self.node.places:
            bb.log(f'{self.name}: no dump place marked for "{colour}" (ros2 run hardleel_mission mark {colour})')
            return FAILURE
        bb['colour'] = colour
        return SUCCESS


class WaitForGate(Action):
    """At the gate place (robot facing the gate): wait until the lidar sees free space straight ahead
    (median range in +-gate_half_angle_deg > gate_open_m) for gate_confirm_s. Fails after gate_max_wait_s.
    The lidar faces forward (yaw 0 in base_link), so angle 0 of /scan = straight ahead."""
    def __init__(self, name, node):
        super().__init__(name)
        self.node, self.t0, self.open_since, self.last_log = node, None, None, 0.0

    def start(self, bb):
        self.t0, self.open_since, self.last_log = time.monotonic(), None, 0.0
        bb.log(f'{self.name}: waiting for the gate to open (ahead > {self.node.cfg["gate_open_m"]} m)')
        return RUNNING

    def update(self, bb):
        c, now = self.node.cfg, time.monotonic()
        if self.node.dry_run:
            return SUCCESS if now - self.t0 > c['dry_run_gate_s'] else RUNNING
        if now - self.t0 > c['gate_max_wait_s']:
            bb.log(f'{self.name}: gate still closed after {c["gate_max_wait_s"]:.0f} s')
            return FAILURE
        ahead = self.node.range_ahead()
        if ahead is not None and ahead > c['gate_open_m']:
            self.open_since = self.open_since or now
            if now - self.open_since >= c['gate_confirm_s']:
                bb.log(f'{self.name}: gate OPEN ({ahead:.2f} m ahead) after {now - self.t0:.1f} s')
                return SUCCESS
        else:
            self.open_since = None
            if now - self.last_log > 5.0:
                self.last_log = now
                bb.log(f'{self.name}: closed ({"no scan" if ahead is None else f"{ahead:.2f} m ahead"})')
        return RUNNING


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
    'rounds': 1,
    'colours': ['blue', 'white', 'black'],
    'gate_open_m': 0.6,
    'gate_half_angle_deg': 10.0,
    'gate_confirm_s': 0.5,
    'gate_max_wait_s': 120.0,
    'dry_run_nav_s': 2.0,
    'dry_run_gate_s': 3.0,
    'dry_run_colours': ['blue', 'none', 'white', 'black'],
}


class MissionNode(Node):
    def __init__(self):
        super().__init__('mission')
        share = get_package_share_directory('hardleel_mission')
        self.declare_parameter('mission_file', os.path.join(share, 'config', 'mission.yaml'))
        self.declare_parameter('dry_run', False)
        self.declare_parameter('places_file', places_mod.PLACES_FILE)
        self.dry_run = self.get_parameter('dry_run').value
        path = self.get_parameter('mission_file').value
        with open(path) as f:
            self.cfg = {**DEFAULTS, **(yaml.safe_load(f) or {})}
        ppath = self.get_parameter('places_file').value
        self.places = places_mod.load(ppath)
        need = places_mod.REQUIRED + self.cfg['colours']
        missing = [n for n in need if n not in self.places]
        self.get_logger().info(f'settings {path}; places {ppath}: {sorted(self.places)}'
                               f'{"  [DRY RUN]" if self.dry_run else ""}')
        if missing:
            self.get_logger().warn(f'places NOT marked: {missing} - mark them with: ros2 run hardleel_mission mark <name>'
                                   + ('' if self.dry_run else ' (the mission fails at the first missing one)'))
        if self.dry_run:                       # dry run: fill missing places with dummies to test the logic
            for i, n in enumerate(missing):
                self.places[n] = {'x': float(i), 'y': 0.0, 'yaw': 0.0}
        self.scan = None
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan', m), qos_profile_sensor_data)

        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.classify = self.create_client(Trigger, '/classify_waste')
        self.servo_pub = self.create_publisher(String, '/servo/cmd', 10)
        self.status_pub = self.create_publisher(String, '/mission/status', 10)
        self.tree_pub = self.create_publisher(String, '/mission/tree', 10)
        self.battery_v, self.start_requested, self.fake_i = None, False, 0
        self.create_subscription(Float32, '/battery/voltage', lambda m: setattr(self, 'battery_v', m.data), 10)
        self.create_subscription(Bool, '/mission/start', self.on_start, 10)

        self.bb = Blackboard(logger=self.get_logger().info, rounds=list(range(1, int(self.cfg['rounds']) + 1)))
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

    def range_ahead(self):
        """Median lidar range straight ahead (+-gate_half_angle_deg), or None without a fresh scan."""
        s = self.scan
        if s is None or s.angle_increment == 0.0:
            return None
        half = math.radians(self.cfg['gate_half_angle_deg'])
        vals = []
        for i, r in enumerate(s.ranges):
            a = math.atan2(math.sin(s.angle_min + i * s.angle_increment), math.cos(s.angle_min + i * s.angle_increment))
            if abs(a) <= half and math.isfinite(r) and s.range_min < r:
                vals.append(min(r, s.range_max))
            elif abs(a) <= half and math.isinf(r) and r > 0:
                vals.append(s.range_max)       # nothing in range = open
        if not vals:
            return None
        vals.sort()
        return vals[len(vals) // 2]

    def place(self, name):
        return self.places.get(name)

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

        do_round = Sequence('DoRound', [
            Condition('BatteryOk', self.battery_ok),
            nav('NavigateTo camera', lambda bb: self.place('camera')),
            Retry('retry Classify', c['classify_retries'], Classify('Classify', self)),
            nav('NavigateTo collect', lambda bb: self.place('collect')),
            Servo('Servo grab', self, 'grab'),
            nav('NavigateTo gate', lambda bb: self.place('gate')),
            WaitForGate('WaitForGate', self),
            nav('NavigateTo dump', lambda bb: self.place(bb.get('colour'))),
            Servo('Servo release', self, 'release'),
            Log('Delivered', lambda bb: f'round {bb["round"]}: waste dumped at {bb["colour"]}'),
        ])
        give_up = Sequence('GiveUp', [
            Servo('Servo release (give up)', self, 'release'),
            Log('Failed', lambda bb: f'round {bb["round"]} FAILED - released, continuing', FAILURE),
        ])
        home = Fallback('GoHome', [
            Condition('no home place', lambda bb: 'home' not in self.places),
            nav('NavigateTo home', lambda bb: self.place('home')),
        ])
        return Sequence('Mission', [
            WaitForStart(self),
            ForEach('ForEach round', 'rounds', 'round', Fallback('Round', [do_round, give_up])),
            home,
            Log('Finished', lambda bb: 'MISSION COMPLETE: ' + ', '.join(
                f'round {r}={s}' for r, s in bb.get('results', []))),
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

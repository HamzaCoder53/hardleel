#!/usr/bin/env python3
"""Maze explorer for auto mapping (runs ON THE PI): right-hand wall follower straight from the lidar.

Why not Nav2 here: corridors are 0.55-0.60 m, the motors only work at PWM 70-75 (no gentle speeds) and
Nav2 ran on the laptop over Wi-Fi at ~8 Hz - its spin/back-up recoveries hit the walls (15 Nov). This node
reacts on the Pi from the raw scan, needs no map or path planner, and only turns on the spot.

Behaviour (right-hand rule - visits every corridor of a connected maze and comes back to the start):
  FORWARD   drive straight; stay centred with short pivot pulses (one wheel stopped for a tick: with PWM
            70-75 a small steering command makes no difference between the wheels)
            right side opens  -> ADVANCE into the opening, then TURN right 90 deg
            front blocked     -> TURN right if open, else left, else turn around (180 deg)
  TURN      rotate on the spot by the target angle (heading from /odom = EKF with the IMU gyro)
Starts DISABLED. Same interface as frontier_explorer: tuning GUI Auto tab (Start / STOP), /explore/command
('start' | 'stop'), status on /explore/status. Publishes /cmd_vel for the motor bridge - do NOT run Nav2
(hauto) at the same time. Angles: lidar 0 deg = robot front (yaw 0, confirmed 15 Nov). All params live.
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

# name: (default, description, (min, max) or None)
PARAMS = {
    'enabled': (False, 'explore now (Start/STOP in the tuning GUI Auto tab); starts OFF', None),
    'forward_speed': (0.30, 'forward speed m/s sent to the motor bridge (0.3 ~ PWM 73)', (0.05, 1.0)),
    'turn_speed': (1.0, 'turn-on-the-spot speed rad/s', (0.2, 3.0)),
    'front_stop': (0.30, 'wall this close ahead (m, from the lidar) -> stop and choose a turn', (0.10, 1.0)),
    'emergency_stop': (0.20, 'anything this close ahead (m, from the lidar) -> stop at once; robot front edge is '
                             '0.14 m ahead of the lidar (28 x 30.5 cm robot)', (0.05, 0.5)),
    'side_open': (0.55, 'side distance (m) above which there is an opening (corridors are 0.55-0.60 m wide)',
                  (0.2, 2.0)),
    'wall_target': (0.27, 'distance to keep from a single wall (m) - half the corridor', (0.1, 0.6)),
    'steer_gain': (4.0, 'centring: pivot-pulse duty per metre off-centre', (0.0, 50.0)),
    'opening_advance': (0.22, 'drive this far into a right opening before turning (m)', (0.0, 1.0)),
    'commit_distance': (0.35, 'after a turn, ignore side openings for this distance (m)', (0.0, 2.0)),
    'turn_overshoot_deg': (8.0, 'stop turning this many degrees early (motors coast / control delay)', (0.0, 45.0)),
    'bridge_turn_gain': (0.5, 'must match the motor bridge turn_gain (used to stop one wheel for a pivot)',
                         (0.05, 3.0)),
    'control_rate': (10.0, 'Hz', (2.0, 30.0)),
    'min_explore_distance': (3.0, 'only finish at the start after driving at least this far (m)', (0.0, 100.0)),
    'home_radius': (0.30, 'back within this distance of the start (odom) -> maze complete', (0.05, 2.0)),
    'max_minutes': (10.0, 'stop after this long no matter what', (0.5, 60.0)),
    # --- added after the robot turned into a wall and kept pushing for 87 s (15 Nov) ---
    'turn_clearance': (0.23, 'turn on the spot only when nothing is closer than this all around (m); robot corners '
                             'sweep 0.207 m (28 x 30.5 cm)', (0.0, 0.6)),
    'max_extra_advance': (0.30, 'at a right opening, drive up to this much further to get turn clearance (m)',
                          (0.0, 1.0)),
    'stall_time': (2.0, 'stall check window (s)', (0.5, 10.0)),
    'stall_turn_deg': (5.0, 'a turn that gains less than this in stall_time is stuck', (0.5, 45.0)),
    'stall_move': (0.02, 'driving that moves less than this (m) in stall_time is stuck', (0.0, 0.3)),
    'backoff_distance': (0.08, 'after a stall, back up this far (m) if the space behind is clear', (0.0, 0.5)),
    'backoff_speed': (0.25, 'reverse speed for backing off (m/s)', (0.05, 1.0)),
    'max_recoveries': (3, 'stalls in a row before giving up with STUCK', (1, 20)),
}
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


def sector_min(scan, lo_deg, hi_deg):
    """Closest valid range (m) with the angle in [lo_deg, hi_deg] (lidar frame, 0 = front, +left)."""
    best = math.inf
    for i, r in enumerate(scan.ranges):
        if not (math.isfinite(r) and scan.range_min < r < scan.range_max):
            continue
        a = math.degrees(scan.angle_min + i * scan.angle_increment)
        a = (a + 180.0) % 360.0 - 180.0
        if lo_deg <= a <= hi_deg:
            best = min(best, r)
    return best


def sectors(scan):
    return {'front': sector_min(scan, -15, 15), 'front_left': sector_min(scan, 30, 60),
            'front_right': sector_min(scan, -60, -30), 'left': sector_min(scan, 75, 105),
            'right': sector_min(scan, -105, -75),
            'back': min(sector_min(scan, 165, 180), sector_min(scan, -180, -165)),
            'all': sector_min(scan, -180, 180)}


def lateral_offset(s, p):
    """How far (m) the robot should move LEFT to be centred (negative = move right)."""
    left, right = s['left'], s['right']
    left_wall, right_wall = left < p['side_open'], right < p['side_open']
    if left_wall and right_wall:
        return (left - right) / 2.0
    if right_wall:
        return p['wall_target'] - right
    if left_wall:
        return left - p['wall_target']
    return 0.0


def choose_turn(s, p):
    """At a blocked front: right if open, else left, else turn around. Returns degrees (+ = left)."""
    if s['right'] > p['side_open']:
        return -90.0
    if s['left'] > p['side_open']:
        return 90.0
    return 180.0


class MazeExplorer(Node):
    def __init__(self):
        super().__init__('maze_explorer')
        for name, (default, desc, rng) in PARAMS.items():
            self.declare_parameter(name, default, ParameterDescriptor(
                description=desc + (f'  [{rng[0]}..{rng[1]}]' if rng else '')))
        if self.p('enabled'):   # never start driving on launch
            self.set_parameters([Parameter('enabled', value=False)])
        self.add_on_set_parameters_callback(self.on_params)
        self.scan = None
        self.scan_time = 0.0
        self.yaw = None
        self.xy = None
        self.state = 'IDLE'
        self.turn_target = 0.0
        self.turned = 0.0
        self.last_yaw = None
        self.advance_from = None
        self.dist_since_turn = 0.0
        self.travelled = 0.0
        self.start_xy = None
        self.started = 0.0
        self.last_xy = None
        self.acc = 0.0
        self.status_text = ''
        self.watch_t, self.watch_turned, self.watch_xy = 0.0, 0.0, None   # stall detection window
        self.recoveries = 0
        self.backoff_from, self.backoff_t, self.pending_turn = None, 0.0, None
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/explore/status', LATCHED)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(String, '/explore/command', self.on_command, 10)
        self.timer = self.create_timer(1.0 / self.p('control_rate'), self.tick)
        self.say('maze explorer idle (disabled) - press Start in the tuning GUI Auto tab')

    # ---- interface -----------------------------------------------------------------------------
    def p(self, name):
        return self.get_parameter(name).value

    def params(self):
        return {k: self.p(k) for k in PARAMS}

    def on_params(self, params):
        for prm in params:
            spec = PARAMS.get(prm.name)
            if spec and spec[2] and not spec[2][0] <= prm.value <= spec[2][1]:
                return SetParametersResult(successful=False, reason=f'{prm.name} must be {spec[2][0]}..{spec[2][1]}')
        for prm in params:
            if prm.name == 'enabled':
                if prm.value:
                    self.state, self.acc = 'FORWARD', 0.0
                    self.travelled, self.dist_since_turn = 0.0, self.p('commit_distance')
                    self.start_xy, self.started = self.xy, time.monotonic()   # xy None -> set on first odom
                    self.last_say, self.recoveries, self.pending_turn = 0.0, 0, None
                    self.turned = 0.0
                    self.watch()
                    self.say('exploring: following the right wall')
                else:
                    self.state = 'IDLE'
                    self.stop()
                    self.say('stopped')
            elif prm.name == 'control_rate':
                self.timer.cancel()
                self.destroy_timer(self.timer)
                self.timer = self.create_timer(1.0 / prm.value, self.tick)
        return SetParametersResult(successful=True)

    def on_command(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == 'start':
            self.set_parameters([Parameter('enabled', value=True)])
        elif cmd in ('stop', 'home'):     # no 'return home' in maze mode: stop
            self.set_parameters([Parameter('enabled', value=False)])

    def say(self, text, every=0.0):
        """Publish a status line when it changes (at most once per `every` seconds if given)."""
        now = time.monotonic()
        if every and now - getattr(self, 'last_say', 0.0) < every:
            return
        self.last_say = now
        if text != self.status_text:
            self.get_logger().info(text)
            self.status_text = text
            self.status_pub.publish(String(data=text))

    def on_scan(self, msg):
        self.scan, self.scan_time = msg, time.monotonic()

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.last_xy is not None and self.state in ('FORWARD', 'ADVANCE'):
            step = math.hypot(xy[0] - self.last_xy[0], xy[1] - self.last_xy[1])
            if step < 0.5:                       # ignore jumps
                self.travelled += step
                self.dist_since_turn += step
        self.last_xy = self.xy = xy
        if self.state != 'IDLE' and self.start_xy is None:
            self.start_xy = xy

    # ---- motion helpers --------------------------------------------------------------------------
    def send(self, v, w):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.cmd_pub.publish(t)

    def stop(self):
        self.send(0.0, 0.0)

    def watch(self):
        """(Re)start the stall-detection window."""
        self.watch_t, self.watch_turned, self.watch_xy = time.monotonic(), self.turned, self.xy

    def start_turn(self, deg, why):
        self.state, self.turn_target, self.turned, self.last_yaw = 'TURN', math.radians(deg), 0.0, self.yaw
        self.watch()
        self.say(f'exploring: {why} -> turning {deg:+.0f} deg')

    def request_turn(self, deg, why, s, p):
        """Turn now if there is room all around; otherwise back off first (once) when the space behind is clear."""
        if s['all'] >= p['turn_clearance'] or p['turn_clearance'] <= 0:
            self.start_turn(deg, why)
        elif s['back'] > p['backoff_distance'] + 0.15:
            self.start_backoff((deg, why), f'{why}: only {s["all"]:.2f} m clearance to turn')
        else:
            self.start_turn(deg, why + f' (tight: {s["all"]:.2f} m)')    # stall detection protects

    def start_backoff(self, pending, why):
        self.stop()
        self.state, self.backoff_from, self.backoff_t, self.pending_turn = 'BACKOFF', self.xy, time.monotonic(), pending
        self.say(f'exploring: {why} -> backing up')

    def stalled(self, why, s, p):
        """Robot is not moving although driven: stop, back off (if clear) and decide again; give up after a few."""
        self.stop()
        self.recoveries += 1
        if self.recoveries > p['max_recoveries']:
            self.set_parameters([Parameter('enabled', value=False)])
            self.say(f'STUCK - {why}; {self.recoveries - 1} recoveries failed. Move the robot free, then Start again')
            return
        pending = None
        if self.state == 'TURN':
            left = math.degrees(abs(self.turn_target) - abs(self.turned))
            pending = (math.copysign(max(left, 0.0), self.turn_target), 'finishing the turn')
        if s['back'] > p['backoff_distance'] + 0.15:
            self.start_backoff(pending, f'STALL {self.recoveries}/{p["max_recoveries"]}: {why}')
        else:
            self.set_parameters([Parameter('enabled', value=False)])
            self.say(f'STUCK - {why} and no room behind ({s["back"]:.2f} m). Move the robot free, then Start again')

    # ---- control loop ----------------------------------------------------------------------------
    def tick(self):
        if self.state == 'IDLE':
            return
        p = self.params()
        if self.scan is None or time.monotonic() - self.scan_time > 1.0 or self.yaw is None:
            self.stop()
            self.say('exploring: PAUSED - no fresh /scan or /odom (lidar / EKF running?)')
            return
        if time.monotonic() - self.started > p['max_minutes'] * 60:
            self.set_parameters([Parameter('enabled', value=False)])
            self.say(f'stopped: max_minutes ({p["max_minutes"]:.0f}) reached - save the map')
            return
        s = sectors(self.scan)
        now = time.monotonic()

        if self.state == 'BACKOFF':
            moved = math.hypot(self.xy[0] - self.backoff_from[0], self.xy[1] - self.backoff_from[1]) \
                if self.xy and self.backoff_from else 0.0
            if moved >= p['backoff_distance'] or now - self.backoff_t > 3.0 or s['back'] < 0.12:
                self.stop()
                pending, self.pending_turn = self.pending_turn, None
                if pending and abs(pending[0]) > p['turn_overshoot_deg']:
                    self.start_turn(pending[0], pending[1])        # no clearance check again: no back-off loops
                else:
                    self.state, self.acc = 'FORWARD', 0.0
                    self.watch()
            else:
                self.send(-p['backoff_speed'], 0.0)
            return

        if self.state == 'TURN':
            d = math.atan2(math.sin(self.yaw - self.last_yaw), math.cos(self.yaw - self.last_yaw))
            self.turned += d
            self.last_yaw = self.yaw
            remaining = abs(self.turn_target) - abs(self.turned)
            if remaining <= math.radians(p['turn_overshoot_deg']):
                self.stop()
                self.state, self.dist_since_turn, self.acc, self.recoveries = 'FORWARD', 0.0, 0.0, 0
                self.watch()
                self.say(f'exploring: turned {math.degrees(self.turned):+.0f} deg, following the right wall')
            elif now - self.watch_t >= p['stall_time']:
                if abs(self.turned - self.watch_turned) < math.radians(p['stall_turn_deg']):
                    self.stalled(f'turn blocked (only {math.degrees(abs(self.turned)):.0f} of '
                                 f'{math.degrees(abs(self.turn_target)):.0f} deg)', s, p)
                    return
                self.watch()
                self.send(0.0, math.copysign(p['turn_speed'], self.turn_target))
            else:
                self.send(0.0, math.copysign(p['turn_speed'], self.turn_target))
            return

        # FORWARD / ADVANCE: stall check (driven but not moving)
        if now - self.watch_t >= p['stall_time']:
            if self.watch_xy and self.xy and \
                    math.hypot(self.xy[0] - self.watch_xy[0], self.xy[1] - self.watch_xy[1]) < p['stall_move']:
                self.stalled('not moving forward', s, p)
                return
            if self.watch_xy is not None:
                self.recoveries = 0                    # moving again
            self.watch()
        if s['front'] < p['emergency_stop']:
            self.stop()
            self.request_turn(choose_turn(s, p), f'EMERGENCY: obstacle {s["front"]:.2f} m ahead', s, p)
            return
        if s['front'] < p['front_stop']:
            self.stop()
            self.request_turn(choose_turn(s, p), f'wall ahead ({s["front"]:.2f} m)', s, p)
            return
        if self.state == 'ADVANCE':
            advanced = self.dist_since_turn - self.advance_from
            if advanced >= p['opening_advance']:
                if s['all'] >= p['turn_clearance'] or advanced >= p['opening_advance'] + p['max_extra_advance']:
                    self.stop()
                    self.start_turn(-90.0, f'right opening (clearance {s["all"]:.2f} m)')
                    return
            self.send(p['forward_speed'], 0.0)        # keep going until the turn has room
            return
        # FORWARD
        if s['right'] > p['side_open'] and self.dist_since_turn >= p['commit_distance']:
            self.state, self.advance_from = 'ADVANCE', self.dist_since_turn
            self.say(f'exploring: opening on the right ({s["right"]:.2f} m) - driving into it')
            self.send(p['forward_speed'], 0.0)
            return
        if (self.start_xy and self.xy and self.travelled > p['min_explore_distance']
                and math.hypot(self.xy[0] - self.start_xy[0], self.xy[1] - self.start_xy[1]) < p['home_radius']):
            self.set_parameters([Parameter('enabled', value=False)])
            self.say(f'maze COMPLETE - back at the start after {self.travelled:.1f} m - save the map now')
            return
        # centring with pivot pulses (sigma-delta): average correction ~ steer duty
        steer = max(-1.0, min(1.0, p['steer_gain'] * lateral_offset(s, p)))
        self.acc += steer
        v = p['forward_speed']
        pivot_w = v / p['bridge_turn_gain']      # makes the inner wheel speed exactly 0 in the bridge
        if self.acc >= 1.0:
            self.acc -= 1.0
            self.send(v, +pivot_w)               # pivot left
        elif self.acc <= -1.0:
            self.acc += 1.0
            self.send(v, -pivot_w)               # pivot right
        else:
            self.send(v, 0.0)
        self.say(f'exploring: corridor (L {s["left"]:.2f} / R {s["right"]:.2f} / front {s["front"]:.2f} m), '
                 f'{self.travelled:.1f} m driven', every=1.0)


def main():
    rclpy.init()
    node = MazeExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop()
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

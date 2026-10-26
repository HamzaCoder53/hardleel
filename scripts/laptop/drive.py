#!/usr/bin/env python3
"""Car-style keyboard driving for mapping (laptop, WSL). Publishes /cmd_vel_teleop (twist_mux -> velocity
smoother -> collision monitor -> Pi), so every safety stop still applies.

    W / Up      faster forward (speed STAYS - cruise; no need to hold)
    S / Down    slower, through 0 into reverse
    A / Left    turn left  while pressed/held (straight again ~0.4 s after you let go; spins in place at 0 speed)
    D / Right   turn right
    Space       STOP now          Shift+W / Shift+S   speed step x2          Q / Esc   quit (stops)

A terminal cannot see a key being HELD, only its auto-repeat, so turning uses a short grace window that covers
the repeat delay. The screen shows the command and the collision monitor (OBSTACLE STOP / SLOW).
    scripts/laptop/drive.sh
"""
import curses
import sys
import time

STEP = 0.05          # m/s per W/S press
MAX_V = 0.40         # m/s
TURN = 1.0           # rad/s while A/D is held
TURN_GRACE = 0.45    # s after the last A/D key: covers the keyboard auto-repeat delay
RATE = 20.0          # Hz publish


class DriveState:
    """Pure key -> command logic (unit-tested without ROS / a terminal)."""

    def __init__(self):
        self.v = 0.0
        self.turn_dir = 0
        self.turn_t = -1e9

    def key(self, k, t):
        """k: 'up' | 'down' | 'up2' | 'down2' | 'left' | 'right' | 'stop'. -> False on quit."""
        if k in ('up', 'up2', 'down', 'down2'):
            step = STEP * (2 if k.endswith('2') else 1) * (1 if k.startswith('up') else -1)
            self.v = round(max(-MAX_V, min(MAX_V, self.v + step)), 3)
        elif k in ('left', 'right'):
            self.turn_dir, self.turn_t = (1 if k == 'left' else -1), t
        elif k == 'stop':
            self.v, self.turn_dir = 0.0, 0
        return k != 'quit'

    def command(self, t):
        """(linear m/s, angular rad/s) to publish now."""
        w = TURN * self.turn_dir if t - self.turn_t <= TURN_GRACE else 0.0
        if w == 0.0:
            self.turn_dir = 0
        return self.v, w                     # A always swings the nose LEFT, also when reversing


KEYS = {ord('w'): 'up', curses.KEY_UP: 'up', ord('W'): 'up2', ord('s'): 'down', curses.KEY_DOWN: 'down',
        ord('S'): 'down2', ord('a'): 'left', ord('A'): 'left', curses.KEY_LEFT: 'left', ord('d'): 'right',
        ord('D'): 'right', curses.KEY_RIGHT: 'right', ord(' '): 'stop', ord('x'): 'stop', ord('q'): 'quit',
        ord('Q'): 'quit', 27: 'quit'}
MONITOR = {0: '', 1: 'OBSTACLE STOP - forward blocked (turn or reverse)', 2: 'slow zone: obstacle near',
           3: 'approach: obstacle ahead'}


DIRECT = '--direct' in sys.argv      # motor test: straight to /cmd_vel, NO collision monitor / smoother


def run(scr):
    import rclpy
    from geometry_msgs.msg import Twist
    from nav2_msgs.msg import CollisionMonitorState
    rclpy.init()
    node = rclpy.create_node('drive_teleop')
    pub = node.create_publisher(Twist, '/cmd_vel' if DIRECT else '/cmd_vel_teleop', 10)
    mon = {'action': 0, 't': None}

    def on_mon(m):
        mon['action'], mon['t'] = m.action_type, time.monotonic()
    node.create_subscription(CollisionMonitorState, '/collision_monitor_state', on_mon, 10)

    curses.curs_set(0)
    scr.nodelay(True)
    scr.keypad(True)
    st = DriveState()
    try:
        while True:
            now = time.monotonic()
            while True:
                c = scr.getch()
                if c == -1:
                    break
                k = KEYS.get(c)
                if k and not st.key(k, now):
                    return
            v, w = st.command(now)
            msg = Twist()
            msg.linear.x, msg.angular.z = float(v), float(w)
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0)
            scr.erase()
            scr.addstr(0, 0, 'HARDLEEL DRIVE   W/S speed (stays)   A/D turn (hold)   SPACE stop   Q quit',
                       curses.A_BOLD)
            bar = int(abs(v) / MAX_V * 20)
            scr.addstr(2, 0, f'speed {v:+.2f} m/s  [{("<" if v < 0 else ">") * bar:<20}]')
            scr.addstr(3, 0, f'turn  {"LEFT " if w > 0 else "RIGHT" if w < 0 else "  -  "}  {w:+.1f} rad/s')
            warn = MONITOR.get(mon['action'], '')
            if DIRECT:
                scr.addstr(5, 0, 'MOTOR TEST MODE: direct to /cmd_vel - NO obstacle stop. SPACE stops.',
                           curses.A_REVERSE | curses.A_BOLD)
            elif mon['t'] is None:
                scr.addstr(5, 0, '(no collision monitor seen yet - is laptop_mapping running?)')
            elif warn:
                scr.addstr(5, 0, warn, curses.A_REVERSE | curses.A_BOLD)
            scr.refresh()
            time.sleep(1.0 / RATE)
    finally:
        for _ in range(3):
            pub.publish(Twist())
            time.sleep(0.05)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    curses.wrapper(run)

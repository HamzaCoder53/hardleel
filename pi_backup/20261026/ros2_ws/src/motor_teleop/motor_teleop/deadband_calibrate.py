#!/usr/bin/env python3
"""Interactive per-side deadband (minimum PWM) calibration. WHEELS MUST BE IN THE AIR.

  ros2 run motor_teleop deadband_calibrate [--side left|right|both] [--step 2] [--start 20] [--hold 0.6]

For each side and each direction the PWM is ramped up by --step every --hold seconds
(published on /motor/pwm_test at 10 Hz). Press ENTER the moment that wheel starts turning,
'r' to redo this ramp, 'q' to abort. min_pwm_<side> = max over the two directions.
Motors are ALWAYS stopped and allow_pwm_test set false on exit (also on Ctrl-C).
"""
import argparse
import queue
import select
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

from motor_teleop.bridge_params import set_bridge_params
from motor_teleop.calib_store import CalibStore, DEFAULT_PATH

try:
    import termios
    import tty
except ImportError:  # not a POSIX terminal
    termios = None


class KeyReader:
    """Single-key input: termios cbreak when on a TTY, else line-based input() fallback."""

    def __init__(self):
        self.fd = None
        self.old = None
        self.q = queue.Queue()
        if termios is not None and sys.stdin.isatty():
            try:
                self.fd = sys.stdin.fileno()
                self.old = termios.tcgetattr(self.fd)
                tty.setcbreak(self.fd)
            except Exception:
                self.fd = None
        if self.fd is None:
            threading.Thread(target=self._line_thread, daemon=True).start()

    def _line_thread(self):
        while True:
            try:
                line = input()
            except EOFError:
                self.q.put('q')
                return
            self.q.put(line.strip()[:1].lower() or '\n')

    def get(self, timeout=0.0):
        """Return one lowercase key ('\\n' for ENTER) or None."""
        if self.fd is not None:
            r, _, _ = select.select([sys.stdin], [], [], timeout)
            if r:
                c = sys.stdin.read(1)
                return '\n' if c in ('\r', '\n') else c.lower()
            return None
        try:
            return self.q.get(timeout=timeout) if timeout > 0 else self.q.get_nowait()
        except queue.Empty:
            return None

    def wait(self):
        while True:
            k = self.get(0.2)
            if k is not None:
                return k

    def restore(self):
        if self.fd is not None and self.old is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)


class Calibrator:
    def __init__(self, node, keys, args):
        self.node, self.keys, self.args = node, keys, args
        self.pub = node.create_publisher(Int32MultiArray, '/motor/pwm_test', 10)

    def send(self, l, r):
        m = Int32MultiArray()
        m.data = [int(l), int(r)]
        self.pub.publish(m)

    def stop(self):
        for _ in range(3):
            self.send(0, 0)
            time.sleep(0.03)

    def ramp(self, side, sign):
        """Returns ('ok', pwm) | ('redo', None) | ('abort', None) | ('none', None)."""
        a = self.args
        p = a.start
        t_next = time.monotonic() + a.hold
        while p <= 255:
            pair = (sign * p, 0) if side == 'left' else (0, sign * p)
            self.send(*pair)
            print(f'\r  {side} {"fwd" if sign > 0 else "back"}  PWM {p:3d} ', end='', flush=True)
            k = self.keys.get(0.1)
            if k == '\n':
                self.stop()
                print()
                return 'ok', p
            if k == 'r':
                self.stop()
                print()
                return 'redo', None
            if k == 'q':
                self.stop()
                print()
                return 'abort', None
            if time.monotonic() >= t_next:
                p += a.step
                t_next = time.monotonic() + a.hold
        self.stop()
        print('\n  reached 255 without a keypress')
        return 'none', None

    def measure_side(self, side):
        vals = {}
        for sign, name in ((1, 'forward'), (-1, 'backward')):
            while True:
                print(f'\n[{side} {name}] ENTER when the wheel starts turning, r=redo, q=abort. '
                      f'Starting in 1 s...')
                time.sleep(1.0)
                self.keys.get(0.0)
                res, p = self.ramp(side, sign)
                if res == 'abort':
                    return None
                if res == 'redo':
                    continue
                if res == 'none':
                    print('  no movement detected; r=redo, q=abort, s=skip (treat as unknown)')
                    k = self.keys.wait()
                    if k == 'q':
                        return None
                    if k == 's':
                        break
                    continue
                print(f'  recorded {p}. Accept? [Y]es / [r]edo / [q]uit')
                k = self.keys.wait()
                if k == 'q':
                    return None
                if k == 'r':
                    continue
                vals[name] = p
                break
        if not vals:
            return None
        return max(vals.values()), vals


def main(argv=None):
    ap = argparse.ArgumentParser(description='Interactive deadband calibration (wheels in the air)')
    ap.add_argument('--side', choices=['left', 'right', 'both'], default='both')
    ap.add_argument('--step', type=int, default=2)
    ap.add_argument('--start', type=int, default=20)
    ap.add_argument('--hold', type=float, default=0.6)
    ap.add_argument('--calib-file', default=DEFAULT_PATH)
    ap.add_argument('--force', action='store_true', help='skip the +-50%% change sanity check')
    args, ros_args = ap.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = Node('deadband_calibrate')
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    keys = KeyReader()
    cal = Calibrator(node, keys, args)
    store = CalibStore(args.calib_file)
    enabled = False
    try:
        print('WHEELS MUST BE IN THE AIR (robot on a box). Enabling allow_pwm_test on /motor_bridge...')
        ok, msg = set_bridge_params(node, {'allow_pwm_test': True})
        if not ok:
            print(f'Cannot enable allow_pwm_test: {msg}')
            return 1
        enabled = True
        results = {}
        for side in (['left', 'right'] if args.side == 'both' else [args.side]):
            r = cal.measure_side(side)
            if r is None:
                print(f'{side}: aborted/no result')
                if keys.get(0.0) == 'q':
                    break
                continue
            results[side] = r
            print(f'{side}: per-direction {r[1]} -> min_pwm_{side} = {r[0]}')
        cal.stop()
        live = {}
        for side, (val, _) in results.items():
            key = f'min_pwm_{side}'
            print(f'Save {key} = {val}? [Y/n]')
            if keys.wait() == 'n':
                continue
            use, ok, reason = store.effective(key, val, force=args.force, source='deadband_calibrate')
            print(f'  {key}: {"saved" if ok else "REJECTED (" + reason + ")"}; live value {use}')
            if use is not None:
                live[key] = int(round(use))
        if live:
            cal.stop()
            ok, msg = set_bridge_params(node, live)
            print(f'Set live on /motor_bridge: {live} -> {msg}')
        return 0
    except KeyboardInterrupt:
        print('\nInterrupted.')
        return 130
    finally:
        try:
            cal.stop()
            if enabled:
                set_bridge_params(node, {'allow_pwm_test': False})
            cal.stop()
        except Exception as e:  # noqa: BLE001
            print(f'cleanup error: {e}', file=sys.stderr)
        keys.restore()
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    sys.exit(main())

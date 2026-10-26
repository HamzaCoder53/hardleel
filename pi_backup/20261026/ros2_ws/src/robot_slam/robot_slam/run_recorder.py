#!/usr/bin/env python3
"""run_recorder: automatic recording into run folders (final stack item 2). One node per side:
    role laptop:  creates ~/runs/run_YYYYMMDD_HHMMSS/{bags,video,stills,maps,logs,config}, publishes /run/id
                  (latched) so the Pi uses the SAME run name, writes git_commit.txt, a config snapshot and NOTES.md.
    role pi:      waits up to run_id_wait_s for /run/id (else names the run itself), records the Pi-side topics at
                  the lowest priority (nice 19 + ionice idle) and STOPS on high temperature, a current
                  throttling/under-voltage flag or < min_free_gb free disk (logged with the reason).
Recording is split by phase: 'mapping' until /mission/state first leaves WAIT_START, then 'mission', then 'post'
after DONE/ABORTED (an explicit /run/phase String overrides). Each phase is its own MCAP bag
(bags/<role>_<phase>), zstd-compressed (config recording.yaml). Stops cleanly (SIGINT to ros2 bag) at shutdown.
/run/recorder_state (String JSON) every check_s: phase, bag, reason of a stop, disk/temp.
"""
import json
import os
import shutil
import signal
import subprocess
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
NOTES = """# {run}
- where: [ ] sim  [ ] real   |  [ ] office  [ ] field   |  [ ] practice  [ ] competition
- battery (V / % at start):
- robot changes since last run:
- what happened:
- data: bags/ (pi_*, laptop_*), video/, stills/, maps/, logs/ ; verify: scripts/laptop/verify_run.py {run}
"""


def phase_from_state(cur, state):
    """Mission state -> recording phase (never goes back)."""
    if state in ('DONE', 'ABORTED'):
        return 'post' if cur in ('mission', 'mapping') else cur
    if state and state != 'WAIT_START' and cur == 'mapping':
        return 'mission'
    return cur


def throttled_now(flags):
    """vcgencmd get_throttled bits 0 (under-voltage now) and 2 (throttled now)."""
    return bool(flags & 0x1) or bool(flags & 0x4)


def stop_reason(cfg, temp_c, flags, free_gb):
    if cfg.get('stop_temp_c', 0) and temp_c is not None and temp_c >= cfg['stop_temp_c']:
        return f'temperature {temp_c:.1f} C'
    if cfg.get('stop_on_throttle') and flags is not None and throttled_now(flags):
        return f'throttled/under-voltage (0x{flags:x})'
    if free_gb is not None and free_gb < cfg.get('min_free_gb', 3.0):
        return f'disk {free_gb:.1f} GB free'
    return None


class RunRecorder(Node):
    def __init__(self):
        super().__init__('run_recorder')
        d = self.declare_parameter
        self.role = d('role', 'laptop').value
        cfg_file = d('config_file', '').value or os.path.join(get_package_share_directory('robot_slam'), 'config',
                                                              'recording.yaml')
        self.all = yaml.safe_load(open(cfg_file))
        self.cfg = self.all[self.role]
        self.runs = os.path.expanduser(d('runs_dir', self.all.get('runs_dir', '~/runs')).value)
        self.repo = os.path.expanduser(d('repo_dir', '~/hardleel').value)
        self.phase = 'mapping'
        self.proc = None
        self.bag = None
        self.stopped_reason = None
        self.run = None
        self.t0 = time.monotonic()
        self.pub_id = self.create_publisher(String, '/run/id', LATCHED)
        self.pub_state = self.create_publisher(String, '/run/recorder_state', 10)
        self.create_subscription(String, '/run/id', self.on_run_id, LATCHED)
        self.create_subscription(String, '/mission/state', self.on_state, 10)
        self.create_subscription(String, '/run/phase', self.on_phase, 10)
        if self.role == 'laptop':
            self.start_run(time.strftime('run_%Y%m%d_%H%M%S'))
        self.create_timer(float(self.cfg.get('check_s', 5.0)), self.check)

    # ------------------------------------------------------------------ run folder
    def start_run(self, name):
        if self.run:
            return
        self.run = name
        root = os.path.join(self.runs, name)
        for sub in ('bags', 'video', 'stills', 'maps', 'logs', 'config'):
            os.makedirs(os.path.join(root, sub), exist_ok=True)
        if self.role == 'laptop':
            self.pub_id.publish(String(data=name))
            self.snapshot(root)
        self.get_logger().info(f'run {name} ({self.role}) -> {root}')
        self.record()

    def snapshot(self, root):
        try:
            h = subprocess.run(['git', '-C', self.repo, 'rev-parse', 'HEAD'], capture_output=True, text=True).stdout
            dirty = subprocess.run(['git', '-C', self.repo, 'status', '--porcelain'], capture_output=True,
                                   text=True).stdout
            open(os.path.join(root, 'git_commit.txt'), 'w').write(h + ('(uncommitted changes)\n' + dirty if dirty else ''))
        except OSError:
            pass
        cfg = os.path.join(get_package_share_directory('robot_slam'), 'config')
        for f in os.listdir(cfg):
            if f.endswith(('.yaml', '.lua', '.xml')):
                shutil.copy(os.path.join(cfg, f), os.path.join(root, 'config', f))
        wp = os.path.expanduser('~/hardleel_ws/waypoints')
        if os.path.isdir(wp):
            shutil.copytree(wp, os.path.join(root, 'config', 'waypoints'), dirs_exist_ok=True)
        notes = os.path.join(root, 'NOTES.md')
        if not os.path.exists(notes):
            open(notes, 'w').write(NOTES.format(run=self.run))

    def on_run_id(self, m):
        if self.role == 'pi' and not self.run and m.data:
            self.start_run(m.data)

    # ------------------------------------------------------------------ phases
    def on_state(self, m):
        self.set_phase(phase_from_state(self.phase, m.data))

    def on_phase(self, m):
        if m.data:
            self.set_phase(m.data)

    def set_phase(self, phase):
        if phase == self.phase:
            return
        self.get_logger().info(f'phase {self.phase} -> {phase}: new bag')
        self.phase = phase
        if self.run and not self.stopped_reason:
            self.stop_bag()
            self.record()

    # ------------------------------------------------------------------ ros2 bag
    def record(self):
        name = f'{self.role}_{self.phase}'
        out = os.path.join(self.runs, self.run, 'bags', name)
        k = 1
        while os.path.exists(out):
            k += 1
            out = os.path.join(self.runs, self.run, 'bags', f'{name}_{k}')
        cmd = ['ros2', 'bag', 'record', '-s', 'mcap', '--storage-preset-profile', self.all.get('storage_preset',
               'zstd_fast'), '--include-hidden-topics', '-o', out] + list(self.cfg['topics'])
        nice = int(self.cfg.get('nice', 10))
        pre = ['nice', '-n', str(nice)] + (['ionice', '-c3'] if self.role == 'pi' and shutil.which('ionice') else [])
        log = open(os.path.join(self.runs, self.run, 'logs', f'record_{os.path.basename(out)}.log'), 'w')
        self.proc = subprocess.Popen(pre + cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        self.bag = out
        self.get_logger().info(f'recording {out} ({len(self.cfg["topics"])} topics, pid {self.proc.pid})')

    def stop_bag(self):
        if self.proc and self.proc.poll() is None:
            os.killpg(self.proc.pid, signal.SIGINT)              # ros2 bag closes the MCAP and writes metadata
            try:
                self.proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid, signal.SIGKILL)
        self.proc = None

    # ------------------------------------------------------------------ watchdog
    def health(self):
        temp = flags = None
        try:
            temp = int(open('/sys/class/thermal/thermal_zone0/temp').read()) / 1000.0
        except (OSError, ValueError):
            pass
        if shutil.which('vcgencmd'):
            try:
                out = subprocess.run(['vcgencmd', 'get_throttled'], capture_output=True, text=True, timeout=2).stdout
                flags = int(out.strip().split('=')[1], 16)
            except (OSError, ValueError, IndexError, subprocess.TimeoutExpired):
                pass
        free = shutil.disk_usage(self.runs).free / 1e9 if os.path.isdir(self.runs) else None
        return temp, flags, free

    def check(self):
        if self.role == 'pi' and not self.run and time.monotonic() - self.t0 > float(self.all.get('run_id_wait_s', 15)):
            self.get_logger().warn('no /run/id from the laptop: naming the run on the Pi')
            self.start_run(time.strftime('run_%Y%m%d_%H%M%S_pi'))
        temp, flags, free = self.health()
        why = stop_reason(self.cfg, temp, flags, free)
        if why and not self.stopped_reason and self.proc:
            self.stopped_reason = why
            self.stop_bag()
            self.get_logger().error(f'RECORDING STOPPED: {why}')
        if self.proc and self.proc.poll() is not None and not self.stopped_reason:
            self.get_logger().error(f'ros2 bag exited (code {self.proc.returncode}): restarting')
            self.record()
        self.pub_state.publish(String(data=json.dumps({
            'role': self.role, 'run': self.run, 'phase': self.phase, 'bag': self.bag, 'stopped': self.stopped_reason,
            'temp_c': temp, 'throttled': None if flags is None else hex(flags),
            'free_gb': None if free is None else round(free, 1)})))

    def shutdown(self):
        self.stop_bag()


def main():
    rclpy.init()
    n = RunRecorder()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.shutdown()
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Pi side of the tuning GUI (runs in tmux 'robot', window 'tuning', started by start_robot.sh).

Topics (std_msgs/String, YAML text):
  /hardleel_tuning/carto_info    (out, latched) {'defaults': {...}, 'saved': {...}, 'session': {...}}
  /hardleel_tuning/saved_params  (out, latched) the Pi's saved params as {'/node': {param: value}}
  /hardleel_tuning/status        (out) one-line results for the GUI
  /hardleel_tuning/save          (in)  {'params': {'/node': {...}}, 'carto': {key: value}} -> Pi's saved files
  /hardleel_tuning/carto_apply   (in)  {key: value} this run only -> restart Cartographer (the map restarts)
Nothing is written unless the GUI sends it; start_robot.sh deletes the session file on every new run.
"""
import os
import subprocess
import threading

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from robot_slam import tuning_store as ts

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
CARTO_CMD = "ros2 launch robot_slam carto_mapping.launch.py use_lidar:=false 2>&1 | tee ~/carto.log"


class TuningHelper(Node):
    def __init__(self):
        super().__init__('tuning_helper')
        self.lua = os.path.join(get_package_share_directory('robot_slam'), 'config', 'cartographer_2d.lua')
        self.info_pub = self.create_publisher(String, '/hardleel_tuning/carto_info', LATCHED)
        self.saved_pub = self.create_publisher(String, '/hardleel_tuning/saved_params', LATCHED)
        self.status_pub = self.create_publisher(String, '/hardleel_tuning/status', 10)
        self.create_subscription(String, '/hardleel_tuning/save', self.on_save, 10)
        self.create_subscription(String, '/hardleel_tuning/carto_apply', self.on_apply, 10)
        self.create_subscription(String, '/hardleel_tuning/save_map', self.on_save_map, 10)
        self.publish_state()
        self.get_logger().info('tuning helper ready')

    def publish_state(self):
        with open(self.lua) as f:
            defaults = ts.lua_tunables(f.read())
        info = {'defaults': defaults, 'saved': ts.load(ts.SAVED_CARTO), 'session': ts.load(ts.SESSION_CARTO)}
        self.info_pub.publish(String(data=yaml.safe_dump(info)))
        self.saved_pub.publish(String(data=yaml.safe_dump(ts.load_saved_flat())))

    def status(self, text):
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    @staticmethod
    def clean_carto(d):
        bad = {k: v for k, v in (d or {}).items() if not ts.LUA_VALUE.match(str(v))}
        return {k: str(v) for k, v in (d or {}).items() if k not in bad}, bad

    def on_save(self, msg):
        try:
            data = yaml.safe_load(msg.data) or {}
            carto, bad = self.clean_carto(data.get('carto'))
            ts.save_flat(data.get('params') or {})
            ts.write(ts.SAVED_CARTO, carto)
            n = sum(len(v) for v in (data.get('params') or {}).values())
            self.status(f'Pi: saved {n} parameter(s) and {len(carto)} Cartographer value(s)'
                        + (f'; ignored invalid {list(bad)}' if bad else '') if (n or carto) else
                        'Pi: saved values cleared - next run uses the defaults')
        except Exception as e:  # noqa: BLE001 - report anything back to the GUI
            self.status(f'Pi: save failed: {e}')
        self.publish_state()

    def on_apply(self, msg):
        try:
            session, bad = self.clean_carto(yaml.safe_load(msg.data) or {})
            if bad:
                self.status(f'Pi: not applied, invalid lua values {bad} (use numbers, true/false, math.rad(x))')
                return
            ts.write(ts.SESSION_CARTO, session)
            ts.effective_carto_dir(os.path.dirname(self.lua))     # rewrite ~/.hardleel/carto/cartographer_2d.lua
            self.publish_state()
            tmux_carto = subprocess.run(['tmux', 'list-windows', '-t', 'robot', '-F', '#W'], capture_output=True,
                                        text=True).stdout.split()
            if 'carto' in tmux_carto:        # started by start_robot.sh: recreate the tmux window
                subprocess.run(['tmux', 'kill-window', '-t', 'robot:carto'], capture_output=True)
                subprocess.run(['tmux', 'new-window', '-d', '-t', 'robot', '-n', 'carto', f'bash -ic "{CARTO_CMD}"'],
                               check=True)
                how = 'tmux window restarted'
            elif subprocess.run(['pkill', '-INT', '-x', 'cartographer_no'], capture_output=True).returncode == 0:
                how = 'stopped, the launch file respawns it in ~2 s'   # pi_mapping.launch.py: respawn=True
            else:
                self.status('Pi: values stored for this run, but Cartographer is not running')
                return
            self.status(f'Pi: Cartographer restarting with {len(session)} session value(s) ({how}) - new map')
        except Exception as e:  # noqa: BLE001
            self.status(f'Pi: apply failed: {e}')


    def on_save_map(self, msg):
        """Save the live Cartographer map as ~/maps/<name>.{pbstream,pgm,yaml} without stopping mapping."""
        name = ''.join(ch for ch in msg.data.strip() if ch.isalnum() or ch in '_-')
        if not name:
            self.status('Pi: map not saved - empty name')
            return
        threading.Thread(target=self.save_map, args=(name,), daemon=True).start()

    def save_map(self, name):
        base = os.path.expanduser(f'~/maps/{name}')
        os.makedirs(os.path.dirname(base), exist_ok=True)
        self.status(f'Pi: saving map {name} ...')
        req = f'{{filename: {base}.pbstream, include_unfinished_submaps: true}}'
        ws = subprocess.run(['ros2', 'service', 'call', '/write_state', 'cartographer_ros_msgs/srv/WriteState', req],
                            capture_output=True, text=True, timeout=120)
        ms = subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', base,
                             '--ros-args', '-p', 'map_subscribe_transient_local:=true'],
                            capture_output=True, text=True, timeout=90)
        got = [ext for ext in ('pbstream', 'pgm', 'yaml') if os.path.exists(f'{base}.{ext}')]
        if len(got) == 3:
            self.status(f'Pi: map saved: ~/maps/{name}.pbstream/.pgm/.yaml')
        else:
            err = (ws.stdout + ws.stderr + ms.stderr)[-200:].replace('\n', ' ')
            self.status(f'Pi: map save incomplete (have {got}): {err}')


def main():
    rclpy.init()
    node = TuningHelper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

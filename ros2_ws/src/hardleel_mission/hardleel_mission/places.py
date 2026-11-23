#!/usr/bin/env python3
"""Named mission places (checkpoints) in the MAP frame, stored in ~/.hardleel/mission_places.yaml.

Mark them WHILE MAPPING (or later on the saved map with Nav2/AMCL running): drive the robot to the spot,
facing the way it must face there, then

  ros2 run hardleel_mission mark camera       AI camera picture spot (face the waste)
  ros2 run hardleel_mission mark collect      waste collection spot
  ros2 run hardleel_mission mark gate         wait spot in front of the timed gate (FACE THE GATE)
  ros2 run hardleel_mission mark blue         dump spot for plastic   (camera colour "blue")
  ros2 run hardleel_mission mark white        dump spot for organic   (camera colour "white")
  ros2 run hardleel_mission mark black        dump spot for metal     (camera colour "black")
  ros2 run hardleel_mission mark home         optional: where to end
  ros2 run hardleel_mission mark --list       show all places
  ros2 run hardleel_mission mark --remove gate

Marks made during mapping belong to the map saved at the END OF THAT SAME mapping run (same map frame).
A new mapping run = a new map frame: mark the places again.
"""
import math
import os
import sys
import time

import yaml

PLACES_FILE = os.path.expanduser('~/.hardleel/mission_places.yaml')
REQUIRED = ['camera', 'collect', 'gate']          # plus one dump place per camera colour


def load(path=PLACES_FILE):
    try:
        with open(path) as f:
            return (yaml.safe_load(f) or {}).get('places', {}) or {}
    except FileNotFoundError:
        return {}


def save(places, path=PLACES_FILE):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        f.write('# hardleel mission places (map frame). Written by: ros2 run hardleel_mission mark <name>\n')
        yaml.safe_dump({'places': places}, f, default_flow_style=None, sort_keys=True)
    os.replace(tmp, path)                          # never leave a half-written file


def current_pose(timeout_s=5.0):
    """Robot pose map -> base_link as {'x', 'y', 'yaw'}, or None."""
    import rclpy
    import tf2_ros
    rclpy.init()
    node = rclpy.create_node('mission_mark')
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    pose, end = None, time.monotonic() + timeout_s
    while time.monotonic() < end and pose is None:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            t = buf.lookup_transform('map', 'base_link', rclpy.time.Time()).transform
        except Exception:  # noqa: BLE001 - not available yet
            continue
        q = t.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        pose = {'x': round(t.translation.x, 3), 'y': round(t.translation.y, 3), 'yaw': round(yaw, 3)}
    node.destroy_node()
    rclpy.shutdown()
    return pose


def fmt(name, p):
    return f'  {name:8s} x {p["x"]:+7.2f}  y {p["y"]:+7.2f}  yaw {math.degrees(p.get("yaw", 0.0)):+6.0f} deg'


def mark_main(argv=None):
    args = (argv if argv is not None else sys.argv[1:])
    args = [a for a in args if not a.startswith('--ros-args')]
    places = load()
    if not args or args[0] in ('-h', '--help'):
        print(__doc__)
        return 0
    if args[0] in ('--list', '-l'):
        print(f'{PLACES_FILE}:')
        for n in sorted(places):
            print(fmt(n, places[n]))
        missing = [n for n in REQUIRED + ['blue', 'white', 'black'] if n not in places]
        print('missing: ' + (', '.join(missing) if missing else 'nothing'))
        return 0
    if args[0] in ('--remove', '-r'):
        for n in args[1:]:
            print(f'removed {n}' if places.pop(n, None) else f'{n}: not marked')
        save(places)
        return 0
    name = args[0].strip().lower()
    pose = current_pose()
    if pose is None:
        print('no map -> base_link transform: is mapping (Cartographer/SLAM) or Nav2/AMCL running?')
        return 1
    old = places.get(name)
    places[name] = pose
    save(places)
    print(('updated' if old else 'marked') + ':\n' + fmt(name, pose))
    return 0


def main():
    sys.exit(mark_main())


if __name__ == '__main__':
    main()

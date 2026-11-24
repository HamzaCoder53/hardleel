#!/usr/bin/env python3
"""Named mission places (checkpoints) in the MAP frame, stored in ~/.hardleel/mission_places.yaml.

Mark them WHILE MAPPING (or later on the saved map with Nav2/AMCL running): drive the robot to the spot,
facing the way it must face there, then

  ros2 run hardleel_mission mark camera       AI camera picture spot (face the waste)
  ros2 run hardleel_mission mark collect      waste collection spot
  ros2 run hardleel_mission mark gate         wait spot in front of the timed gate (FACE THE GATE)
  ros2 run hardleel_mission mark blue         dump spot for plastic   (camera colour "blue")
  ros2 run hardleel_mission mark green        drop-off for organic    (camera colour "green")
  ros2 run hardleel_mission mark red          drop-off for metal      (camera colour "red")
  ros2 run hardleel_mission mark home         optional: where to end
  ros2 run hardleel_mission mark --list       show all places
  ros2 run hardleel_mission mark --remove gate

WITHOUT THE ROBOT, by clicking on a saved map (laptop):
  ros2 launch hardleel_mission mark_on_map.launch.py map:=$HOME/maps/mymap.yaml     (map + RViz)
  ros2 run hardleel_mission mark --click             asks for camera, collect, gate, blue, white, black, home
  ros2 run hardleel_mission mark --click gate        only re-click one place
  RViz "2D Goal Pose" tool: click = position, drag = the way the robot faces. Places show as arrows.

Marks made during mapping belong to the map saved at the END OF THAT SAME mapping run (same map frame).
A new mapping run = a new map frame: mark the places again.
"""
import math
import os
import sys
import time

import yaml

PLACES_FILE = os.path.expanduser('~/.hardleel/mission_places.yaml')
REQUIRED = ['camera']                            # + per colour: a pickup (pickup_<c> or 'collect') and a drop-off (<c>)
COLOURS = ['blue', 'green', 'red']               # rule book 2026: plastic -> Blue, organic -> Green, metal -> Red
CLICK_ORDER = ['camera', 'pickup_blue', 'pickup_green', 'pickup_red', 'blue', 'green', 'red', 'gate']   # no home in the competition (user 24 Nov); 'mark home' still works


def missing(places, colours=COLOURS):
    """Places the mission still needs. 'gate' and 'home' are optional."""
    out = [n for n in REQUIRED if n not in places]
    for c in colours:
        if f'pickup_{c}' not in places and 'collect' not in places:
            out.append(f'pickup_{c}')
        if c not in places:
            out.append(c)
    return out


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


def markers(places):
    """visualization_msgs/MarkerArray: an arrow + a label per place (RViz: MarkerArray on /mission/places)."""
    from visualization_msgs.msg import Marker, MarkerArray
    colour = {'blue': (0.1, 0.3, 1.0), 'green': (0.1, 0.8, 0.2), 'red': (0.9, 0.1, 0.1),
              'gate': (1.0, 0.5, 0.0), 'camera': (0.8, 0.2, 0.9), 'collect': (0.1, 0.8, 0.2), 'home': (1.0, 1.0, 0.0)}
    arr = MarkerArray()
    clear = Marker()
    clear.action = Marker.DELETEALL
    arr.markers.append(clear)
    for i, (name, p) in enumerate(sorted(places.items())):
        r, g, b = colour.get(name, (1.0, 0.2, 0.2))
        for kind in ('arrow', 'text'):
            m = Marker()
            m.header.frame_id = 'map'
            m.ns, m.id = kind, i
            m.pose.position.x, m.pose.position.y = float(p['x']), float(p['y'])
            yaw = float(p.get('yaw', 0.0))
            m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
            if kind == 'arrow':
                m.type = Marker.ARROW
                m.scale.x, m.scale.y, m.scale.z = 0.30, 0.06, 0.06
            else:
                m.type = Marker.TEXT_VIEW_FACING
                m.text = name
                m.pose.position.z = 0.25
                m.pose.orientation.z, m.pose.orientation.w = 0.0, 1.0
                m.scale.z = 0.18
            arr.markers.append(m)
    return arr


def click_main(names, path=PLACES_FILE):
    """Mark places by clicking on the map in RViz ("2D Goal Pose" tool: click = position, drag = facing).
    Works without the robot (only the map + RViz). Publishes the places as markers on /mission/places."""
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from visualization_msgs.msg import MarkerArray
    rclpy.init()
    node = rclpy.create_node('mission_mark_click')
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(MarkerArray, '/mission/places', latched)
    places = load(path)
    pub.publish(markers(places))
    got = []
    node.create_subscription(PoseStamped, '/goal_pose', got.append, 10)
    todo = list(names)
    print(f'RViz: choose "2D Goal Pose", then for each place CLICK on the spot and DRAG towards where the robot '
          f'must face.\n(places file {path}; Ctrl+C to stop - every click is saved immediately)')
    try:
        while todo and rclpy.ok():
            print(f'\n>>> click the place: {todo[0]}   ({len(todo) - 1} more after this)')
            got.clear()
            while not got and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.2)
            q = got[-1].pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            p = got[-1].pose.position
            places[todo[0]] = {'x': round(p.x, 3), 'y': round(p.y, 3), 'yaw': round(yaw, 3)}
            save(places, path)
            pub.publish(markers(places))
            print('saved:' + fmt(todo[0], places[todo[0]])[1:])
            todo.pop(0)
        print('\nall done:')
        for n in sorted(places):
            print(fmt(n, places[n]))
        missing_ = missing(places)
        print('missing: ' + (', '.join(missing_) if missing_ else 'nothing') + '   (gate optional; no home in the competition)')
        print('\nThe arrows stay visible in RViz until you press Ctrl+C here.')
        while rclpy.ok():                         # keep the latched markers alive for RViz
            rclpy.spin_once(node, timeout_sec=0.5)
    except (KeyboardInterrupt, ExternalShutdownException):
        print('\nstopped (places clicked so far are saved)')
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


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
        missing_ = missing(places)
        print('missing: ' + (', '.join(missing_) if missing_ else 'nothing') + '   (gate optional; no home in the competition)')
        return 0
    if args[0] in ('--click', '-c'):
        return click_main(args[1:] or CLICK_ORDER)
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

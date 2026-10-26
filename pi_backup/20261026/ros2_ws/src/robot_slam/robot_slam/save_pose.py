"""Save the robot's current map->base_link pose as a named waypoint.

    ros2 run robot_slam save_pose <name> [--file PATH]     # drive the robot there first
    ros2 run robot_slam save_pose --list [--file PATH]     # print the file (no ROS needed)

File: --file, else $HARDLEEL_WAYPOINTS, else ~/hardleel_ws/waypoints/waypoints_course.yaml
(created from the packaged waypoints_course.yaml draft if missing).
"""
import argparse
import math
import os
import shutil
import sys

from robot_slam import waypoints as W

DEFAULT_FILE = '~/hardleel_ws/waypoints/waypoints_course.yaml'


def default_file():
    return os.path.expanduser(os.environ.get('HARDLEEL_WAYPOINTS') or DEFAULT_FILE)


def packaged_draft(name='waypoints_course.yaml'):
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory('robot_slam'), 'config', name)
        if os.path.exists(p):
            return p
    except Exception:
        pass
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', name)   # source tree


def ensure_file(path):
    """Create `path` from the packaged draft if it does not exist. Returns True when created."""
    if os.path.exists(path):
        return False
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    shutil.copyfile(packaged_draft(), path)
    return True


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def lookup_pose(target='map', source='base_link', timeout=3.0):
    """(x, y, yaw) of `source` in `target`, or None after `timeout` s."""
    import rclpy
    import tf2_ros
    from rclpy.duration import Duration
    from rclpy.time import Time
    rclpy.init()
    node = rclpy.create_node('save_pose')
    try:
        buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(buf, node)
        end = node.get_clock().now() + Duration(seconds=timeout)
        while node.get_clock().now() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                t = buf.lookup_transform(target, source, Time()).transform
                return t.translation.x, t.translation.y, yaw_of(t.rotation)
            except Exception:
                continue
        return None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def print_file(path):
    poses, seqs, meta = W.load_waypoints(path)
    print('%s' % path)
    for n, (x, y, yaw) in poses.items():
        print('  %-16s x=%7.3f y=%7.3f yaw=%7.1f  %-5s %s' % (n, x, y, math.degrees(yaw), meta[n].get('conf', ''), meta[n].get('note', '')[:60]))
    for s, names in seqs.items():
        print('  seq %-8s %s' % (s, ', '.join(names)))
    for p in W.check_waypoints(poses, seqs):
        print('  PROBLEM:', p)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('name', nargs='?', help='waypoint name, e.g. screen')
    ap.add_argument('--file', default=None)
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--frame', default='map')
    ap.add_argument('--base', default='base_link')
    a = ap.parse_args(argv)
    path = os.path.expanduser(a.file) if a.file else default_file()
    if ensure_file(path):
        print('created %s from the packaged draft' % path)
    if a.list or not a.name:
        print_file(path)
        return 0
    pose = lookup_pose(a.frame, a.base, 3.0)
    if pose is None:
        print('ERROR: no TF %s -> %s within 3 s (is SLAM / localization running?)' % (a.frame, a.base), file=sys.stderr)
        return 1
    known = W.load_waypoints(path)[0]
    if a.name not in known:
        print('note: %r is a new name (known: %s)' % (a.name, ', '.join(known)))
    x, y, yaw = W.set_pose(path, a.name, *pose)
    print('saved %s: x=%.3f y=%.3f yaw=%.1f deg -> %s' % (a.name, x, y, math.degrees(yaw), path))
    return 0


if __name__ == '__main__':
    sys.exit(main())

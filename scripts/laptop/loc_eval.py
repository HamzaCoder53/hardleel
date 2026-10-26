#!/usr/bin/env python3
"""SIM ONLY: localization error = Cartographer pose (TF map -> base_link) vs Gazebo /ground_truth.
The map origin is wherever the robot started, so the world->map offset is taken from the first sample.
    loc_eval.py <out.csv>        (runs until Ctrl-C / TERM, then prints a summary line)
Also logs /localization/confidence and counts /localization/event types."""
import json
import math
import signal
import sys

import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformException, TransformListener


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def main():
    out = open(sys.argv[1] if len(sys.argv) > 1 else '/tmp/loc_eval.csv', 'w')
    out.write('t,gt_x,gt_y,gt_yaw,est_x,est_y,est_yaw,err_m,err_deg,confidence\n')
    rclpy.init()
    n = rclpy.create_node('loc_eval', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    tf = Buffer()
    TransformListener(tf, n)
    st = {'gt': None, 'off': None, 'conf': float('nan'), 'errs': [], 'yerrs': [], 'events': {}}
    n.create_subscription(Odometry, '/ground_truth', lambda m: st.__setitem__('gt', m), 10)
    n.create_subscription(Float32, '/localization/confidence', lambda m: st.__setitem__('conf', m.data), 10)

    def on_ev(m):
        e = json.loads(m.data).get('event', '?')
        st['events'][e] = st['events'].get(e, 0) + 1
    n.create_subscription(String, '/localization/event', on_ev, 50)

    def tick():
        g = st['gt']
        if g is None:
            return
        try:
            t = tf.lookup_transform('map', 'base_link', rclpy.time.Time())
        except TransformException:
            return
        gx, gy, gth = g.pose.pose.position.x, g.pose.pose.position.y, yaw(g.pose.pose.orientation)
        ex, ey, eth = t.transform.translation.x, t.transform.translation.y, yaw(t.transform.rotation)
        if st['off'] is None:                               # world pose of the map origin
            oth = gth - eth
            ox = gx - (math.cos(oth) * ex - math.sin(oth) * ey)
            oy = gy - (math.sin(oth) * ex + math.cos(oth) * ey)
            st['off'] = (ox, oy, oth)
        ox, oy, oth = st['off']
        wx = ox + math.cos(oth) * ex - math.sin(oth) * ey
        wy = oy + math.sin(oth) * ex + math.cos(oth) * ey
        e, ye = math.hypot(wx - gx, wy - gy), math.degrees(wrap(oth + eth - gth))
        st['errs'].append(e)
        st['yerrs'].append(abs(ye))
        now = n.get_clock().now().nanoseconds * 1e-9
        out.write(f'{now:.2f},{gx:.3f},{gy:.3f},{gth:.3f},{wx:.3f},{wy:.3f},{oth + eth:.3f},{e:.3f},{ye:.2f},{st["conf"]:.3f}\n')

    n.create_timer(0.5, tick)
    stop = {'v': False}
    signal.signal(signal.SIGTERM, lambda *a: stop.__setitem__('v', True))
    try:
        while rclpy.ok() and not stop['v']:
            rclpy.spin_once(n, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    out.close()
    E, Y = st['errs'], st['yerrs']
    if E:
        s = sorted(E)
        print(f'LOC_EVAL n={len(E)} pos mean {sum(E) / len(E):.3f} p95 {s[int(0.95 * (len(s) - 1))]:.3f} '
              f'max {max(E):.3f} final {E[-1]:.3f} m | yaw mean {sum(Y) / len(Y):.2f} max {max(Y):.2f} deg | '
              f'events {st["events"]}')
    else:
        print('LOC_EVAL no samples (no /ground_truth or no map TF)')


if __name__ == '__main__':
    main()

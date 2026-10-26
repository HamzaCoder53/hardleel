#!/usr/bin/env python3
"""Start the mission and CONFIRM it started: publish /mission/start at 2 Hz until /mission/state leaves
WAIT_START (or timeout). A single `topic pub` can reach only some subscribers (map_freezer also listens), so the
mission could miss it (sim, 26 Oct). Exit 0 = started, 1 = no mission seen / never left WAIT_START.
    mission_go.py [timeout_s=20]"""
import sys
import time

import rclpy
from std_msgs.msg import Empty, String


def main():
    timeout = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    rclpy.init()
    n = rclpy.create_node('mission_go')
    pub = n.create_publisher(Empty, '/mission/start', 10)
    st = {'state': None}
    n.create_subscription(String, '/mission/state', lambda m: st.__setitem__('state', m.data), 10)
    end = time.monotonic() + timeout
    sent = 0
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
        s = st['state']
        if s is not None and s != 'WAIT_START':
            print(f'MISSION STARTED ({s}) after {sent} start message(s) {time.strftime("%H:%M:%S")}')
            return 0
        if s == 'WAIT_START':                    # the mission is up and listening: send (repeats are ignored)
            pub.publish(Empty())
            sent += 1
            for _ in range(10):
                rclpy.spin_once(n, timeout_sec=0.05)
    print(f'MISSION DID NOT START: state {st["state"]!r} after {timeout:.0f} s ({sent} sent) - is laptop_mission '
          f'running and idle?', file=sys.stderr)
    return 1


if __name__ == '__main__':
    sys.exit(main())

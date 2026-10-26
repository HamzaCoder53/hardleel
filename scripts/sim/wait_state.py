#!/usr/bin/env python3
"""Block until /mission/state equals STATE, then wait DELAY simulated seconds (/clock), then exit 0 (exit 1 on timeout).

    wait_state.py STATE [delay_sim_s=0] [timeout_wall_s=900]

Used by mission_demo.sh for the timed gate: the gate opens `delay` SIM seconds after the mission reaches GATE_DECISION
(sim time, because the mission and Nav2 run on sim time and the real-time factor is not always 1).
"""
import sys
import time

import rclpy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

state, delay = sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
timeout = float(sys.argv[3]) if len(sys.argv) > 3 else 900.0
rclpy.init()
n = rclpy.create_node('wait_state')
box = {'seen': False, 'clock': 0.0, 't0': None}
n.create_subscription(String, '/mission/state', lambda m: box.__setitem__('seen', box['seen'] or m.data == state), 10)
n.create_subscription(Clock, '/clock', lambda m: box.__setitem__('clock', m.clock.sec + m.clock.nanosec * 1e-9), 10)
end = time.time() + timeout
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.05)
    if box['seen'] and box['t0'] is None:
        box['t0'] = box['clock']
        print(f'state {state} reached at sim {box["t0"]:.1f} s', flush=True)
    if box['t0'] is not None and box['clock'] - box['t0'] >= delay:
        print(f'delay {delay} sim s over at sim {box["clock"]:.1f} s', flush=True)
        sys.exit(0)
sys.exit(1)

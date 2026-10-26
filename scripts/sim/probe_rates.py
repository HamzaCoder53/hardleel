#!/usr/bin/env python3
"""Print message rates of topics for a few seconds.  probe_rates.py SECONDS /topic:type ...   (type: scan|odom|imu|any)"""
import sys, time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import LaserScan
rclpy.init(); n = Node('probe_rates'); cnt = {}
secs = float(sys.argv[1])
for t in sys.argv[2:]:
    cnt[t] = 0
    topics = dict(n.get_topic_names_and_types())
    ty = topics.get(t, [None])[0]
    if ty is None:
        continue
    n.create_subscription(get_message(ty), t, lambda m, t=t: cnt.__setitem__(t, cnt[t] + 1), qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < secs: rclpy.spin_once(n, timeout_sec=0.1)
for t, c in cnt.items(): print(f'{t}: {c / secs:.1f} Hz')

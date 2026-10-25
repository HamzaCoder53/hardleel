"""Count messages per topic for N s and print publisher QoS (no ros2 daemon needed)."""
import sys, time
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from rosidl_runtime_py.utilities import get_message
rclpy.init(); n = rclpy.create_node('probe')
dur = float(sys.argv[1]); topics = sys.argv[2:]
time.sleep(2.0)
for _ in range(10): rclpy.spin_once(n, timeout_sec=0.1)
counts = {}
for t in topics:
    info = n.get_publishers_info_by_topic(t)
    if not info:
        print(f'{t}: NO PUBLISHER'); continue
    typ = info[0].topic_type
    rel = sorted({str(i.qos_profile.reliability).split('.')[-1] for i in info})
    nodes = sorted({i.node_name for i in info})
    counts[t] = 0
    n.create_subscription(get_message(typ), t, lambda m, t=t: counts.__setitem__(t, counts[t] + 1), qos_profile_sensor_data)
    print(f'{t}: {typ} pubs={nodes} reliability={rel}')
t0 = time.time()
while time.time() - t0 < dur: rclpy.spin_once(n, timeout_sec=0.1)
for t, c in counts.items(): print(f'{t}: {c / dur:.1f} Hz')
if '/tf' in topics:
    from tf2_ros import Buffer, TransformListener

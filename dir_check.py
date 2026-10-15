import math, time
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


rclpy.init()
node = Node('dir_check')
buf = Buffer()
TransformListener(buf, node, spin_thread=True)


def pose(parent):
    t = buf.lookup_transform(parent, 'base_link', rclpy.time.Time())
    tr = t.transform.translation
    return tr.x, tr.y, yaw_of(t.transform.rotation)


time.sleep(2.0)
frames = ['map', 'odom']
before = {f: pose(f) for f in frames}
input('Press Enter AFTER you have driven the robot straight FORWARD about 0.5 m with teleop (key i) and stopped... ')
after = {f: pose(f) for f in frames}

for f in frames:
    x0, y0, _ = before[f]
    x1, y1, yaw1 = after[f]
    dx, dy = x1 - x0, y1 - y0
    dist = math.hypot(dx, dy)
    if dist < 0.1:
        print(f'{f:5s}: moved only {dist:.2f} m, drive further and retry')
        continue
    ang = math.degrees(math.atan2(dy, dx) - yaw1)
    ang = (ang + 180) % 360 - 180
    if abs(ang) < 45:
        verdict = 'ALIGNED with the robot heading (good)'
    elif abs(ang) > 135:
        verdict = 'OPPOSITE to the robot heading (this frame is wrong)'
    else:
        verdict = 'sideways or unclear (did the robot turn while driving?)'
    print(f'{f:5s}: moved {dist:.2f} m, angle to heading {ang:6.1f} deg -> {verdict}')
rclpy.shutdown()

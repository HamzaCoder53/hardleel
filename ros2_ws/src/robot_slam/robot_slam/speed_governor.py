#!/usr/bin/env python3
"""speed_governor (laptop, OFF unless launched with speed_feedback:=true): closes the forward-speed loop that the
open-loop motors cannot. Sand, gravel and ramps need more PWM than the floor for the same speed.

While the robot drives roughly straight (|cmd v| > min_cmd, |cmd w| < max_turn) it compares the commanded speed
(/cmd_vel, what goes to the Pi) with the speed measured from the Cartographer pose (TF map->base_link) over
window_s, and nudges /motor/speed_scale (std_msgs/Float32, 0.5..2.0) towards cmd/measured. motor_bridge multiplies
linear speed by it when speed_scale_enabled (this node switches that on at start and off at exit).
The pose is localisation, not odometry: nothing here feeds commanded PWM back into localisation.
Params: min_cmd (0.08), max_turn (0.3), window_s (1.5), gain (0.3), scale_min/max (0.5/2.0), rate_hz (2),
        decay (0.05: drift back towards 1.0 per update while not driving straight), bridge (/motor_bridge).
"""
import math
import time
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformException, TransformListener


class Governor:
    """Pure logic: feed (t, cmd_v, cmd_w) and (t, x, y, yaw); update() -> new scale."""

    def __init__(self, min_cmd=0.08, max_turn=0.3, window_s=1.5, gain=0.3, smin=0.5, smax=2.0, decay=0.05):
        self.min_cmd, self.max_turn, self.window, self.gain = min_cmd, max_turn, window_s, gain
        self.smin, self.smax, self.decay = smin, smax, decay
        self.cmds, self.poses = deque(), deque()
        self.scale = 1.0
        self.last_ratio = None

    def add_cmd(self, t, v, w):
        self.cmds.append((t, v, w))

    def add_pose(self, t, x, y, yaw):
        self.poses.append((t, x, y, yaw))

    def _trim(self, now):
        for q in (self.cmds, self.poses):
            while q and q[0][0] < now - self.window:
                q.popleft()

    def update(self, now):
        self._trim(now)
        straight = (len(self.cmds) >= 3 and len(self.poses) >= 2 and
                    all(abs(v) > self.min_cmd and abs(w) < self.max_turn for _, v, w in self.cmds) and
                    all((v > 0) == (self.cmds[0][1] > 0) for _, v, _w in self.cmds))
        if not straight:
            self.scale += (1.0 - self.scale) * self.decay
            self.last_ratio = None
            return self.scale
        cmd = sum(v for _, v, _w in self.cmds) / len(self.cmds)
        a, b = self.poses[0], self.poses[-1]
        dt = b[0] - a[0]
        if dt < 0.5 * self.window:
            return self.scale
        meas = ((b[1] - a[1]) * math.cos(a[3]) + (b[2] - a[2]) * math.sin(a[3])) / dt
        # the scale in effect produced `meas` from `cmd`; the ideal scale is scale * cmd / meas
        ratio = abs(cmd) / max(abs(meas), 0.02)
        self.last_ratio = ratio
        target = min(self.smax, max(self.smin, self.scale * ratio))
        self.scale = min(self.smax, max(self.smin, self.scale * (target / self.scale) ** self.gain))
        return self.scale


class SpeedGovernor(Node):
    def __init__(self):
        super().__init__('speed_governor')
        p = self.declare_parameter
        g = Governor(p('min_cmd', 0.08).value, p('max_turn', 0.3).value, p('window_s', 1.5).value,
                     p('gain', 0.3).value, p('scale_min', 0.5).value, p('scale_max', 2.0).value, p('decay', 0.05).value)
        self.gov = g
        self.bridge = p('bridge', '/motor_bridge').value
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.pub = self.create_publisher(Float32, '/motor/speed_scale', 10)
        self.create_subscription(Twist, '/cmd_vel', lambda m: g.add_cmd(time.monotonic(), m.linear.x, m.angular.z), 10)
        self.create_timer(0.1, self.sample_pose)
        self.create_timer(1.0 / float(p('rate_hz', 2.0).value), self.tick)
        self.params = self.create_client(SetParameters, self.bridge + '/set_parameters')
        self.enabled = False

    def set_bridge(self, on):
        if not self.params.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f'{self.bridge} not reachable: speed_scale_enabled not set')
            return
        req = SetParameters.Request()
        req.parameters = [Parameter(name='speed_scale_enabled',
                                    value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=on))]
        self.params.call_async(req)
        self.enabled = on
        self.get_logger().info(f'motor_bridge speed_scale_enabled = {on}')

    def sample_pose(self):
        try:
            t = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time())
        except TransformException:
            return
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.gov.add_pose(time.monotonic(), t.transform.translation.x, t.transform.translation.y, yaw)

    def tick(self):
        if not self.enabled:
            self.set_bridge(True)
        s = self.gov.update(time.monotonic())
        self.pub.publish(Float32(data=float(s)))
        if self.gov.last_ratio is not None:
            self.get_logger().info(f'speed scale {s:.2f} (cmd/measured {self.gov.last_ratio:.2f})',
                                   throttle_duration_sec=2.0)


def main():
    rclpy.init()
    n = SpeedGovernor()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            n.set_bridge(False)
        except Exception:
            pass
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

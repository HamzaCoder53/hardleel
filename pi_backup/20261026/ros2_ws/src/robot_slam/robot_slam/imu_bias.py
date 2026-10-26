#!/usr/bin/env python3
"""imu_bias: online gyro bias estimation on top of the Pi's startup calibration (laptop side, nothing changes on
the Pi). /imu/data_raw -> /imu/data_raw_unbiased. While the robot is STILL (|gyro| < still_rate_dps on all axes and
the accel norm steady for still_s), the residual bias follows the measured rate with a slow EMA (tau_s); otherwise
it is frozen. Then a laptop Madgwick filter turns the output into /imu/data_v2 (see stack_common.imu_v2).
/imu/bias (Vector3Stamped, rad/s) is published for logging. Params: still_rate_dps, still_accel_std, still_s, tau_s,
max_bias_dps (clamp).
"""
import math
from collections import deque

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class BiasEstimator:
    """Pure logic (unit-tested). update(t, gyro xyz, accel xyz) -> corrected gyro xyz."""

    def __init__(self, still_rate=math.radians(1.5), still_accel_std=0.05, still_s=1.0, tau_s=8.0,
                 max_bias=math.radians(3.0)):
        self.still_rate, self.acc_std, self.still_s, self.tau = still_rate, still_accel_std, still_s, tau_s
        self.max_bias = max_bias
        self.bias = [0.0, 0.0, 0.0]
        self.still_since = None
        self.acc = deque(maxlen=50)
        self.t = None
        self.still = False

    def update(self, t, g, a):
        dt = 0.0 if self.t is None else min(max(t - self.t, 0.0), 0.1)
        self.t = t
        corr = [g[i] - self.bias[i] for i in range(3)]
        self.acc.append(math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2))
        n = len(self.acc)
        mean = sum(self.acc) / n
        std = math.sqrt(sum((v - mean) ** 2 for v in self.acc) / n)
        quiet = all(abs(c) < self.still_rate for c in corr) and n >= 10 and std < self.acc_std
        if quiet:
            self.still_since = self.still_since if self.still_since is not None else t
        else:
            self.still_since = None
        self.still = quiet and t - self.still_since >= self.still_s
        if self.still and dt > 0:
            k = dt / self.tau
            for i in range(3):
                b = self.bias[i] + k * (g[i] - self.bias[i])
                self.bias[i] = max(-self.max_bias, min(self.max_bias, b))
        return [g[i] - self.bias[i] for i in range(3)]


class ImuBias(Node):
    def __init__(self):
        super().__init__('imu_bias')
        p = lambda n, v: self.declare_parameter(n, v).value
        self.est = BiasEstimator(math.radians(float(p('still_rate_dps', 1.5))), float(p('still_accel_std', 0.05)),
                                 float(p('still_s', 1.0)), float(p('tau_s', 8.0)),
                                 math.radians(float(p('max_bias_dps', 3.0))))
        self.pub = self.create_publisher(Imu, '/imu/data_raw_unbiased', qos_profile_sensor_data)
        self.bias_pub = self.create_publisher(Vector3Stamped, '/imu/bias', 10)
        self.create_subscription(Imu, '/imu/data_raw', self.on_imu, qos_profile_sensor_data)
        self.n = 0

    def on_imu(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        g = (m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z)
        a = (m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z)
        c = self.est.update(t, g, a)
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = c
        self.pub.publish(m)
        self.n += 1
        if self.n % 50 == 0:
            b = Vector3Stamped()
            b.header = m.header
            b.vector.x, b.vector.y, b.vector.z = self.est.bias
            self.bias_pub.publish(b)


def main():
    rclpy.init()
    n = ImuBias()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

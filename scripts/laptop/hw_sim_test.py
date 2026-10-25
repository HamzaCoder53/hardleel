#!/usr/bin/env python3
"""Hardware-in-the-loop sim checks for motor_bridge (sim.launch.py hardware:=true: real motor_bridge -> fake_pico
motor model with deadband L/R 60/70, v_max 0.9, slow zone in the right corridor).

    python3 hw_sim_test.py deadband      # slow command without / with deadband compensation
    python3 hw_sim_test.py yaw           # turn-rate tracking in the slow zone, yaw_feedback off / on
    python3 hw_sim_test.py speed         # forward speed in the slow zone, speed_scale off / on (needs speed_governor)
Prints measured vs commanded values from /ground_truth; exit code 0 = all checks passed.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters


class T:
    def __init__(self):
        rclpy.init()
        self.n = rclpy.create_node('hw_sim_test')
        self.pub = self.n.create_publisher(Twist, '/cmd_vel', 10)
        self.pose = None
        self.n.create_subscription(Odometry, '/ground_truth', self.on_truth, 10)
        self.params = self.n.create_client(SetParameters, '/motor_bridge/set_parameters')
        self.wait(lambda: self.pose is not None, 20)

    def on_truth(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw, time.monotonic())

    def wait(self, pred, t):
        end = time.monotonic() + t
        while time.monotonic() < end and not pred():
            rclpy.spin_once(self.n, timeout_sec=0.05)
        return pred()

    def spin(self, t):
        self.wait(lambda: False, t)

    def set(self, **kv):
        req = SetParameters.Request()
        for k, v in kv.items():
            pv = ParameterValue()
            if isinstance(v, bool):
                pv.type, pv.bool_value = ParameterType.PARAMETER_BOOL, v
            elif isinstance(v, int):
                pv.type, pv.integer_value = ParameterType.PARAMETER_INTEGER, v
            else:
                pv.type, pv.double_value = ParameterType.PARAMETER_DOUBLE, float(v)
            req.parameters.append(Parameter(name=k, value=pv))
        self.params.wait_for_service(timeout_sec=10)
        fut = self.params.call_async(req)
        self.wait(fut.done, 5)
        ok = fut.done() and all(r.successful for r in fut.result().results)
        print(f'  set {kv}: {"ok" if ok else "FAILED"}')
        return ok

    def drive(self, lin, ang, secs):
        """Command (lin, ang) for secs; return mean (forward speed, yaw rate) over the last 60 %."""
        t0 = time.monotonic()
        samples = []
        while time.monotonic() - t0 < secs:
            tw = Twist()
            tw.linear.x, tw.angular.z = lin, ang
            self.pub.publish(tw)
            rclpy.spin_once(self.n, timeout_sec=0.05)
            if time.monotonic() - t0 > 0.4 * secs and self.pose:
                samples.append(self.pose)
        for _ in range(5):
            self.pub.publish(Twist())
            rclpy.spin_once(self.n, timeout_sec=0.05)
        self.spin(1.0)
        if len(samples) < 2:
            return 0.0, 0.0
        a, b = samples[0], samples[-1]
        dt = b[3] - a[3]
        d = (b[0] - a[0]) * math.cos(a[2]) + (b[1] - a[1]) * math.sin(a[2])
        dyaw = sum(math.atan2(math.sin(q[2] - p[2]), math.cos(q[2] - p[2]))   # unwrapped: spins pass +-pi
                   for p, q in zip(samples, samples[1:]))
        return d / dt, dyaw / dt


def main():
    what = sys.argv[1] if len(sys.argv) > 1 else 'deadband'
    t = T()
    ok = True
    if what == 'deadband':
        t.set(min_pwm_left=0, min_pwm_right=0)
        v0, _ = t.drive(0.10, 0.0, 3.0)
        t.drive(-0.10, 0.0, 3.0)                       # back to roughly where it was
        t.set(min_pwm_left=60, min_pwm_right=70)
        v1, w1 = t.drive(0.10, 0.0, 3.0)
        t.drive(-0.10, 0.0, 3.0)
        print(f'cmd 0.10 m/s: without compensation {v0:.3f} m/s, with 60/70 {v1:.3f} m/s (yaw drift {w1:.3f} rad/s)')
        ok = v0 < 0.03 and 0.07 < v1 < 0.13
    elif what == 'speed':                       # needs laptop_mapping (TF) + speed_governor started by the caller
        def zone_speed(secs):
            """Drive straight at 0.2 m/s; mean ground-truth speed while inside the slow zone (world y 1.3-2.4)."""
            t0, inside = time.monotonic(), []
            while time.monotonic() - t0 < secs:
                tw = Twist()
                tw.linear.x = 0.2
                t.pub.publish(tw)
                rclpy.spin_once(t.n, timeout_sec=0.05)
                if t.pose and 1.45 < t.pose[1] < 2.3:
                    inside.append(t.pose)
            if len(inside) < 2:
                return 0.0
            return math.hypot(inside[-1][0] - inside[0][0], inside[-1][1] - inside[0][1]) / (inside[-1][3] - inside[0][3])
        t.set(min_pwm_left=60, min_pwm_right=70, speed_scale_enabled=False)
        start_y = t.pose[1]
        v0 = zone_speed(12.0)
        t.drive(-0.25, 0.0, (t.pose[1] - start_y) / 0.25 + 1.0)       # back to the start
        t.set(speed_scale_enabled=True)                                  # what speed_governor does at start-up
        v1 = zone_speed(12.0)
        t.drive(-0.25, 0.0, (t.pose[1] - start_y) / 0.25 + 1.0)
        print(f'0.20 m/s in the slow zone (x0.6): governor off {v0:.3f}, on {v1:.3f} m/s')
        ok = v0 < 0.15 and v1 > 0.17
    elif what in ('cal_turn', 'cal_speed'):     # needs motor_calibrator (and laptop_mapping for cal_speed)
        from std_srvs.srv import Trigger
        srv = '/motor/calibrate_turn' if what == 'cal_turn' else '/motor/calibrate_speed'
        cli = t.n.create_client(Trigger, srv)
        ok = cli.wait_for_service(timeout_sec=15)
        if ok:
            fut = cli.call_async(Trigger.Request())
            t.wait(fut.done, 120)
            ok = fut.done() and fut.result().success
            print(f'{srv}: {fut.result().message if fut.done() else "TIMEOUT"}')
        else:
            print(f'{srv}: service not found')
    elif what == 'yaw':
        t.set(min_pwm_left=60, min_pwm_right=70, yaw_feedback=False)
        _, w0 = t.drive(0.0, 0.8, 5.0)
        t.set(yaw_feedback=True)
        _, w1 = t.drive(0.0, 0.8, 5.0)
        t.set(yaw_feedback=False)
        print(f'cmd 0.8 rad/s spin: feedback off {w0:.3f}, on {w1:.3f} rad/s')
        ok = abs(w1 - 0.8) < abs(w0 - 0.8) and abs(w1 - 0.8) < 0.15
    print('PASS' if ok else 'FAIL')
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()

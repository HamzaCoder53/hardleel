#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist

def euler_from_quaternion(q):
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

class HardleelGuardianNode(Node):
    def __init__(self):
        super().__init__('hardleel_guardian_node')

        # Parameters
        self.declare_parameter('autonomous_explore', True)
        self.declare_parameter('safe_dist_m', 0.50)
        self.declare_parameter('crit_dist_m', 0.28)
        self.declare_parameter('max_lin_vel', 0.18)
        self.declare_parameter('max_ang_vel', 0.35)
        self.declare_parameter('yaw_kp', 0.75)
        self.declare_parameter('cmd_timeout_s', 0.5)   # operator command older than this counts as stopped
        self.declare_parameter('publish_cmd_vel', True)  # False: only filter the scan, never publish motor commands

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # State variables
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.gyro_z = 0.0
        self.latest_raw_cmd = Twist()
        self.raw_stamp = None        # when /cmd_vel_raw last arrived; an old command counts as zero
        self.latest_scan = None

        # Subscribers & Publishers
        self.sub_imu = self.create_subscription(Imu, '/imu/data', self.imu_cb, sensor_qos)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_cb, sensor_qos)
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel_raw', self.cmd_cb, 10)

        self.pub_scan_filt = self.create_publisher(LaserScan, '/scan_filtered', sensor_qos)
        self.pub_cmd_safe = self.create_publisher(Twist, '/cmd_vel', 10)

        # 50 Hz control loop for reactive bumper + IMU yaw stabilization
        self.timer = self.create_timer(0.02, self.control_loop)
        self.get_logger().info('Hardleel Guardian Node started (Tilt Gate + Autonomous APF Bumper).')

    def imu_cb(self, msg: Imu):
        roll, pitch, _ = euler_from_quaternion(msg.orientation)
        self.roll_deg = math.degrees(roll)
        self.pitch_deg = math.degrees(pitch)
        self.gyro_z = msg.angular_velocity.z

    def cmd_cb(self, msg: Twist):
        self.latest_raw_cmd = msg
        self.raw_stamp = self.get_clock().now()

    def scan_cb(self, msg: LaserScan):
        self.latest_scan = msg
        abs_pitch = abs(self.pitch_deg)
        abs_roll = abs(self.roll_deg)

        # Stage 3: Severe ramp crest/tilt -> drop scan completely to protect submap
        if abs_pitch > 7.0 or abs_roll > 6.0:
            return

        filt_scan = LaserScan()
        filt_scan.header = msg.header
        filt_scan.angle_min = msg.angle_min
        filt_scan.angle_max = msg.angle_max
        filt_scan.angle_increment = msg.angle_increment
        filt_scan.time_increment = msg.time_increment
        filt_scan.scan_time = msg.scan_time
        filt_scan.range_min = 0.18
        filt_scan.range_max = 5.50

        ranges = list(msg.ranges)

        # Stage 2: Moderate ramp tilt -> mask front/rear cones with NaN, keep side walls
        if abs_pitch > 3.5 or abs_roll > 3.5:
            for i in range(len(ranges)):
                angle_rad = msg.angle_min + i * msg.angle_increment
                # Normalize angle to [-pi, pi]
                ang = math.atan2(math.sin(angle_rad), math.cos(angle_rad))
                ang_deg = abs(math.degrees(ang))
                # Mask forward (+/- 40 deg) and rear (>= 140 deg) beams hitting ramp/floor
                if ang_deg < 40.0 or ang_deg > 140.0:
                    ranges[i] = float('nan')

        filt_scan.ranges = ranges
        filt_scan.intensities = msg.intensities
        self.pub_scan_filt.publish(filt_scan)

    def control_loop(self):
        if self.latest_scan is None:
            return

        auto_mode = self.get_parameter('autonomous_explore').value
        if not self.get_parameter('publish_cmd_vel').value:
            return                                   # scan filtering runs in scan_cb; no motor commands from here
        raw = self.latest_raw_cmd
        fresh = (self.raw_stamp is not None and
                 (self.get_clock().now() - self.raw_stamp).nanoseconds * 1e-9 < self.get_parameter('cmd_timeout_s').value)
        if not auto_mode and not (fresh and (abs(raw.linear.x) > 1e-3 or abs(raw.angular.z) > 1e-3)):
            self.pub_cmd_safe.publish(Twist())       # manual mode, no fresh operator command: never move
            return
        safe_d = self.get_parameter('safe_dist_m').value
        crit_d = self.get_parameter('crit_dist_m').value
        max_v = self.get_parameter('max_lin_vel').value
        max_w = self.get_parameter('max_ang_vel').value
        yaw_kp = self.get_parameter('yaw_kp').value

        scan = self.latest_scan
        rep_x, rep_y = 0.0, 0.0
        min_front_dist = 10.0
        left_clearance, right_clearance = 0.0, 0.0
        left_count, right_count = 1, 1

        for i, r in enumerate(scan.ranges):
            if math.isnan(r) or math.isinf(r) or r < 0.15 or r > 5.5:
                continue
            ang = math.atan2(
                math.sin(scan.angle_min + i * scan.angle_increment),
                math.cos(scan.angle_min + i * scan.angle_increment)
            )
            # Track front cone minimum distance (+/- 55 deg)
            if abs(ang) < math.radians(55.0):
                min_front_dist = min(min_front_dist, r)

            # Accumulate left vs right clearance for autonomous steering
            if 0.1 < ang < math.radians(85.0):
                left_clearance += r
                left_count += 1
            elif -math.radians(85.0) < ang < -0.1:
                right_clearance += r
                right_count += 1

            # Repulsive Potential Field inside safe_dist_m
            if r < safe_d:
                gain = 0.08 * (1.0 / r - 1.0 / safe_d) / (r * r)
                rep_x -= gain * math.cos(ang)
                rep_y -= gain * math.sin(ang)

        cmd = Twist()
        if auto_mode:
            # Base forward speed scaled by front clearance
            cmd.linear.x = max_v * min(1.0, max(0.0, (min_front_dist - crit_d) / (safe_d - crit_d)))
            # Steer smoothly toward the side with greater average clearance
            avg_left = left_clearance / left_count
            avg_right = right_clearance / right_count
            cmd.angular.z = 0.45 * (avg_left - avg_right) + 1.2 * rep_y
        else:
            cmd.linear.x = max(0.0, raw.linear.x + rep_x)
            cmd.angular.z = raw.angular.z + 1.2 * rep_y

        # Critical Emergency Bumper: Zero forward velocity & pivot toward open space
        if min_front_dist < crit_d:
            cmd.linear.x = 0.0                       # never drive into something
            if auto_mode:                            # only an autonomous run turns the robot by itself
                turn_dir = 1.0 if (left_clearance / left_count) >= (right_clearance / right_count) else -1.0
                cmd.angular.z = turn_dir * (0.85 * max_w)

        # Ramp Grade Assist & Descent Braking via IMU Pitch
        if self.pitch_deg > 4.0 and cmd.linear.x > 0.02:
            cmd.linear.x = min(max_v * 1.15, cmd.linear.x * 1.20)
        elif self.pitch_deg < -4.0 and cmd.linear.x > 0.02:
            cmd.linear.x = min(0.11, cmd.linear.x)

        # Clamp velocities to protect 5.5 Hz encoderless Cartographer
        cmd.linear.x = max(-0.12, min(max_v, cmd.linear.x))
        target_w = max(-max_w, min(max_w, cmd.angular.z))

        # Closed-loop IMU Gyro Yaw-Rate Damping (prevents sand/gravel slip spinout)
        if abs(target_w) > 0.02:
            cmd.angular.z = max(-max_w, min(max_w, target_w + yaw_kp * (target_w - self.gyro_z)))
        else:
            cmd.angular.z = 0.0                      # no commanded turn: do not turn on gyro noise

        self.pub_cmd_safe.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = HardleelGuardianNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Read the Nano 33 BLE Sense IMU over USB serial and publish ROS 2 messages.

Startup gyro calibration: the gyro is averaged for gyro_calib_s while the robot stands still and the mean is
subtracted. If the gyro or accelerometer spread shows the robot moved, the node logs a loud error and starts
again (up to calib_max_attempts, then it keeps the stillest attempt). /imu/recalibrate (std_srvs/Trigger)
reruns it on demand; /imu/calib_status (String, latched) reports the result.
"""
import math
import threading
import time

import rclpy
import serial
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import String
from std_srvs.srv import Trigger

G = 9.80665
DEG2RAD = math.pi / 180.0


class CalibStats:
    """Running mean/std of the three gyro axes (deg/s) and of the accel norm (g)."""

    def __init__(self):
        self.n = 0
        self.g_sum = [0.0] * 3
        self.g_sq = [0.0] * 3
        self.a_sum = 0.0
        self.a_sq = 0.0

    def add(self, ax, ay, az, gx, gy, gz):
        self.n += 1
        for i, v in enumerate((gx, gy, gz)):
            self.g_sum[i] += v
            self.g_sq[i] += v * v
        a = math.sqrt(ax * ax + ay * ay + az * az)
        self.a_sum += a
        self.a_sq += a * a

    @staticmethod
    def _std(s, sq, n):
        return math.sqrt(max(sq / n - (s / n) ** 2, 0.0))

    def bias(self):
        return [s / self.n for s in self.g_sum]

    def gyro_std(self):
        return max(self._std(s, q, self.n) for s, q in zip(self.g_sum, self.g_sq))

    def accel_std(self):
        return self._std(self.a_sum, self.a_sq, self.n)

    def moved(self, gyro_std_max, accel_std_max):
        return self.gyro_std() > gyro_std_max or self.accel_std() > accel_std_max


class ImuSerialNode(Node):
    def __init__(self):
        super().__init__('imu_serial_node')
        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('gyro_calib_s', 3.0)   # keep the robot still this long at startup
        self.declare_parameter('gyro_deadband_dps', 0.3)  # ignore yaw rates smaller than this
        # still robot measured 22 Oct: gyro std ~0.02 deg/s; limits are generous PLACEHOLDERs
        self.declare_parameter('calib_gyro_std_max_dps', 1.0)   # still robot 25 Oct (lidar spinning): 0.39
        self.declare_parameter('calib_accel_std_max_g', 0.02)
        self.declare_parameter('calib_max_attempts', 5)

        self.frame_id = self.get_parameter('frame_id').value
        self.port, self.baud = self.get_parameter('port').value, self.get_parameter('baud').value
        # blocking reads in a thread: the old 5 ms polling timer with non-blocking reads used ~70 % of a Pi 4 core.
        # The port is opened by the reader thread and retried, so a missing / resetting Arduino is not fatal.
        self.ser = None

        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 20)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, 'imu/calib_status', latched)
        self.create_service(Trigger, 'imu/recalibrate', self.on_recalibrate)

        self.calib_s = self.get_parameter('gyro_calib_s').value
        self.deadband = self.get_parameter('gyro_deadband_dps').value
        self.gyro_std_max = float(self.get_parameter('calib_gyro_std_max_dps').value)
        self.accel_std_max = float(self.get_parameter('calib_accel_std_max_g').value)
        self.max_attempts = int(self.get_parameter('calib_max_attempts').value)
        self.bias = None
        self.start_calibration()

        self.running = True
        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()

    def start_calibration(self, attempt=1, best=None):
        self.calib_start = time.monotonic()
        self.stats = CalibStats()
        self.attempt = attempt
        self.best = best                          # stillest finished attempt so far
        self.calibrating = True                   # a recalibration keeps publishing with the old bias
        self.publish_status(f'CALIBRATING attempt {attempt}')
        self.get_logger().info(f'Calibrating gyro for {self.calib_s:.0f} s (attempt {attempt}): keep the robot still')

    def publish_status(self, text):
        self.status_pub.publish(String(data=text))

    def on_recalibrate(self, _req, res):
        self.start_calibration()
        res.success, res.message = True, f'recalibrating for {self.calib_s:.0f} s: keep the robot still'
        return res

    def finish_calibration(self):
        st = self.stats
        detail = f'gyro std {st.gyro_std():.3f} deg/s, accel std {st.accel_std():.4f} g, n={st.n}'
        if st.moved(self.gyro_std_max, self.accel_std_max):
            if self.best is None or st.gyro_std() < self.best.gyro_std():
                self.best = st
            self.get_logger().error('!' * 60)
            self.get_logger().error(f'ROBOT MOVED DURING GYRO CALIBRATION ({detail}). Hold it still!')
            self.get_logger().error('!' * 60)
            if self.attempt < self.max_attempts:
                self.start_calibration(self.attempt + 1, self.best)
                return
            st = self.best
            self.publish_status(f'WARN moved: using stillest of {self.attempt} attempts ({detail})')
            self.get_logger().error(f'Giving up after {self.attempt} attempts; using the stillest one.')
        else:
            self.publish_status(f'OK {detail}')
        self.bias = st.bias()
        self.calibrating = False
        self.get_logger().info(
            f'Gyro bias (deg/s): {self.bias[0]:.3f}, {self.bias[1]:.3f}, {self.bias[2]:.3f} ({detail}). Publishing.')

    def read_loop(self):
        """Blocking line reads (sleeps until the Arduino sends). After a USB reset of the Arduino (seen on the
        robot 26 Oct) the port is reopened instead of failing forever."""
        while self.running and rclpy.ok():
            try:
                if self.ser is None:
                    self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
                    self.get_logger().info(f'IMU serial open on {self.port}')
                line = self.ser.readline()
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f'Serial error: {e} - reopening', throttle_duration_sec=2.0)
                try:
                    if self.ser is not None:
                        self.ser.close()
                except Exception:
                    pass
                self.ser = None
                time.sleep(0.5)
                continue
            if line:
                self.handle_line(line.decode(errors='ignore').strip())

    def handle_line(self, line):
        parts = line.split(',')
        try:
            if parts[0] == 'I' and len(parts) == 7:
                ax, ay, az, gx, gy, gz = (float(v) for v in parts[1:])
                self.handle_imu(ax, ay, az, gx, gy, gz)
            elif parts[0] == 'M' and len(parts) == 4:
                mx, my, mz = (float(v) for v in parts[1:])
                self.handle_mag(mx, my, mz)
        except ValueError:
            pass  # partial line at startup, ignore

    def handle_imu(self, ax, ay, az, gx, gy, gz):
        if self.calibrating:
            self.stats.add(ax, ay, az, gx, gy, gz)
            if time.monotonic() - self.calib_start >= self.calib_s and self.stats.n > 1:
                self.finish_calibration()
        if self.bias is None:
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.orientation_covariance[0] = -1.0      # no orientation estimate
        msg.angular_velocity.x = (gx - self.bias[0]) * DEG2RAD
        msg.angular_velocity.y = (gy - self.bias[1]) * DEG2RAD
        gz_c = gz - self.bias[2]
        if abs(gz_c) < self.deadband:
            gz_c = 0.0
        msg.angular_velocity.z = gz_c * DEG2RAD
        msg.linear_acceleration.x = ax * G
        msg.linear_acceleration.y = ay * G
        msg.linear_acceleration.z = az * G
        for i in (0, 4, 8):
            msg.angular_velocity_covariance[i] = 0.001
            msg.linear_acceleration_covariance[i] = 0.05
        self.imu_pub.publish(msg)

    def handle_mag(self, mx, my, mz):
        msg = MagneticField()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.magnetic_field.x = mx * 1e-6          # uT -> T
        msg.magnetic_field.y = my * 1e-6
        msg.magnetic_field.z = mz * 1e-6
        self.mag_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.reader.join(timeout=0.5)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

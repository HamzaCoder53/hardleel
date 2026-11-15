#!/usr/bin/env python3
"""Read the Nano 33 BLE Sense IMU over USB serial and publish ROS 2 messages."""
import math
import threading
import time

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField

G = 9.80665
DEG2RAD = math.pi / 180.0


class ImuSerialNode(Node):
    def __init__(self):
        super().__init__('imu_serial_node')
        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('gyro_calib_s', 3.0)   # keep the robot still this long at startup
        self.declare_parameter('gyro_deadband_dps', 0.3)  # ignore yaw rates smaller than this

        self.frame_id = self.get_parameter('frame_id').value
        # Blocking reads in a thread (sleeps until a line arrives). The old 5 ms polling timer used ~58 % of
        # a Pi core for a 93 Hz sensor (cpu_profile.py, 15 Nov).
        self.ser = serial.Serial(self.get_parameter('port').value,
                                 self.get_parameter('baud').value, timeout=0.2)

        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 20)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)

        self.calib_s = self.get_parameter('gyro_calib_s').value
        self.deadband = self.get_parameter('gyro_deadband_dps').value
        self.calib_start = time.monotonic()
        self.bias_sum = [0.0, 0.0, 0.0]
        self.bias_n = 0
        self.bias = None
        self.get_logger().info(f'Calibrating gyro for {self.calib_s:.0f} s: keep the robot still')

        self.add_on_set_parameters_callback(self.on_params)
        threading.Thread(target=self.read_loop, daemon=True).start()

    def on_params(self, params):
        """Live tuning (tuning GUI): gyro_deadband_dps applies at once; the rest only at the next start."""
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'gyro_deadband_dps':
                if not 0.0 <= float(p.value) <= 10.0:
                    return SetParametersResult(successful=False, reason='gyro_deadband_dps must be 0..10')
            elif p.name in ('port', 'baud', 'frame_id', 'gyro_calib_s'):
                return SetParametersResult(successful=False,
                                           reason=f'{p.name} is read at start only: Save, then restart the IMU')
        for p in params:
            if p.name == 'gyro_deadband_dps':
                self.deadband = float(p.value)
        return SetParametersResult(successful=True)

    def read_loop(self):
        """Reader thread: wait for data, take everything that is waiting, split into lines, nap 10 ms.
        (pyserial's readline() reads ONE byte per system call: ~46 % of a core at 94 lines/s, 15 Nov.)"""
        buf = b''
        while rclpy.ok():
            try:
                chunk = self.ser.read(max(1, self.ser.in_waiting))   # blocks up to 0.2 s for the first byte
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f'Serial error: {e}', throttle_duration_sec=2.0)
                time.sleep(0.5)
                continue
            if chunk:
                buf += chunk
                *lines, buf = buf.split(b'\n')
                for line in lines:
                    self.handle_line(line.decode(errors='ignore').strip())
                if len(buf) > 4096:            # no newline for a long time: garbage, start over
                    buf = b''
            time.sleep(0.01)                   # let ~1 line collect: ~100 wake-ups/s instead of thousands

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
        if self.bias is None:
            self.bias_sum[0] += gx
            self.bias_sum[1] += gy
            self.bias_sum[2] += gz
            self.bias_n += 1
            if time.monotonic() - self.calib_start >= self.calib_s and self.bias_n > 0:
                self.bias = [s / self.bias_n for s in self.bias_sum]
                self.get_logger().info(
                    f'Gyro bias (deg/s): {self.bias[0]:.3f}, {self.bias[1]:.3f}, {self.bias[2]:.3f}. Publishing.')
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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

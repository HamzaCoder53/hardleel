#!/usr/bin/env python3
"""Minimal RPLIDAR A1 node (standard scan, 115200) -> /scan. Written because sllidar_node kept failing to start the scan
(SL_RESULT_OPERATION_TIMEOUT / 80008000). Same angle convention as sllidar_ros2: ros_angle = pi - raw_angle (wrapped to [-pi, pi)).
Re-opens and resets the lidar itself whenever it stops sending data.
"""
import math
import threading
import time

import numpy as np
import rclpy
import serial
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

PORT, BAUD, NBINS = '/dev/ttyUSB0', 115200, 360


class A1(Node):
    def __init__(self):
        super().__init__('a1_lidar')
        self.pub = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.ser = None
        self.pts = []
        self.revs = 0
        threading.Thread(target=self.run, daemon=True).start()

    def start_scan(self):
        if self.ser:
            self.ser.close()
        self.ser = serial.Serial(PORT, BAUD, timeout=0.5)
        self.ser.dtr = True                      # this adapter only answers with DTR asserted
        time.sleep(1.0)
        self.ser.write(bytes([0xA5, 0x40]))      # reset first; the A1 then prints its banner and accepts commands
        time.sleep(3.0)
        for attempt in range(4):
            self.ser.write(bytes([0xA5, 0x25]))
            time.sleep(0.3)
            self.ser.reset_input_buffer()
            self.ser.write(bytes([0xA5, 0x20]))
            hdr = self.ser.read(7)
            if hdr.startswith(b'\xa5\x5a\x05'):
                self.get_logger().info('scan started')
                return True
            self.get_logger().warn(f'no scan header ({hdr.hex()}), resetting lidar')
            self.ser.write(bytes([0xA5, 0x40]))
            time.sleep(2.5)
            self.ser.reset_input_buffer()
        return False

    def run(self):
        buf = b''
        last = time.time()
        while rclpy.ok():
            try:
                if self.ser is None or not self.ser.is_open or time.time() - last > 3.0:
                    if not self.start_scan():
                        time.sleep(1.0)
                        continue
                    buf = b''
                    last = time.time()
                data = self.ser.read(1000)
                if data:
                    last = time.time()
                buf += data
                while len(buf) >= 5:
                    b0, b1 = buf[0], buf[1]
                    s, ns = b0 & 1, (b0 >> 1) & 1
                    if s == ns or not (b1 & 1):          # lost sync: slide one byte
                        buf = buf[1:]
                        continue
                    ang = ((buf[2] << 8) | b1) >> 1
                    dist = (buf[4] << 8) | buf[3]
                    buf = buf[5:]
                    if s and self.pts:
                        self.publish()
                    self.pts.append((ang / 64.0, dist / 4.0 / 1000.0, b0 >> 2))
            except Exception as e:                       # serial hiccup: reopen
                self.get_logger().warn(f'serial error {e}')
                self.ser = None
                time.sleep(1.0)

    def publish(self):
        pts, self.pts = self.pts, []
        if len(pts) < 100:
            return
        rng = np.full(NBINS, np.inf, dtype=np.float32)
        inten = np.zeros(NBINS, dtype=np.float32)
        for deg, r, q in pts:
            if r < 0.15 or r > 12.0:
                continue
            a = math.pi - math.radians(deg)
            a = (a + math.pi) % (2 * math.pi) - math.pi
            i = int((a + math.pi) / (2 * math.pi) * NBINS) % NBINS
            if r < rng[i]:
                rng[i], inten[i] = r, q
        m = LaserScan()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'laser'
        m.angle_min, m.angle_max = -math.pi, math.pi - 2 * math.pi / NBINS
        m.angle_increment = 2 * math.pi / NBINS
        m.scan_time = 0.2
        m.time_increment = m.scan_time / NBINS
        m.range_min, m.range_max = 0.15, 12.0
        m.ranges, m.intensities = rng.tolist(), inten.tolist()
        self.pub.publish(m)
        self.revs += 1


def main():
    rclpy.init()
    n = A1()
    try:
        rclpy.spin(n)
    finally:
        rclpy.shutdown()


main()

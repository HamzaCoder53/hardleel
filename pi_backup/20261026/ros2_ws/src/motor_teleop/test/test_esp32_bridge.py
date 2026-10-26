"""Tests for ESP32 motor bridge against a virtual serial port (PTY).

Validates:
1. Serial connection handshake (P -> PONG).
2. cmd_vel produces M <left> <right> commands.
3. 10 Hz keep-alive continues resending while driving.
4. 300 ms Pi safety watchdog stops the motors when cmd_vel stops.
5. Drop-bin commands route to S <angle> and publish /motor/servo_state.
"""

import os
import pty
import select
import threading
import time
from typing import List

import pytest
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, String

from motor_teleop.esp32_link import Esp32Motors
from motor_teleop.motor_bridge import MotorBridge


class FakeEsp32Device:
    """Simulates the ESP32 firmware on a PTY slave endpoint."""

    def __init__(self, master_fd: int):
        self.master_fd = master_fd
        self.running = True
        self.received_lines: List[str] = []
        self.received_m_times: List[float] = []
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        buf = b''
        while self.running:
            r, _, _ = select.select([self.master_fd], [], [], 0.05)
            if not r:
                continue
            try:
                data = os.read(self.master_fd, 1024)
                if not data:
                    break
                buf += data
            except OSError:
                break

            while b'\n' in buf:
                raw_line, buf = buf.split(b'\n', 1)
                line = raw_line.decode('ascii', errors='ignore').strip()
                if not line:
                    continue
                with self.lock:
                    self.received_lines.append(line)
                    if line.startswith('M'):
                        self.received_m_times.append(time.monotonic())

                if line == 'P':
                    os.write(self.master_fd, b'PONG\n')
                elif line.startswith('M'):
                    # Acknowledge or stay silent like real firmware
                    pass
                elif line.startswith('S'):
                    pass
                elif line == 'X':
                    pass

    def stop(self):
        self.running = False
        self.thread.join(timeout=0.5)


@pytest.fixture
def pty_pair():
    master_fd, slave_fd = pty.openpty()
    slave_name = os.ttyname(slave_fd)
    device = FakeEsp32Device(master_fd)
    yield master_fd, slave_name, device
    device.stop()
    try:
        os.close(master_fd)
    except OSError:
        pass
    try:
        os.close(slave_fd)
    except OSError:
        pass


def test_esp32_motors_handshake_and_commands(pty_pair):
    _, slave_name, device = pty_pair

    motors = Esp32Motors(port=slave_name, baudrate=115200, timeout=0.1)
    time.sleep(0.1)
    assert motors.connected

    # Test M command
    motors.drive(120, -120)
    time.sleep(0.05)
    with device.lock:
        assert any(l.startswith('M 120 -120') for l in device.received_lines)

    # Test keepalive: driving should periodically re-send M commands
    time.sleep(0.35)
    with device.lock:
        m_cmds = [l for l in device.received_lines if l.startswith('M 120 -120')]
        assert len(m_cmds) >= 3, f'Expected at least 3 keepalive M commands, got {len(m_cmds)}'

    # Test S command
    motors.servos(45)
    time.sleep(0.05)
    with device.lock:
        assert 'S 45' in device.received_lines

    # Test X command
    motors.stop()
    time.sleep(0.05)
    with device.lock:
        assert 'X' in device.received_lines
        assert not motors.is_driving

    motors.close()


def test_motor_bridge_safety_watchdog_and_servos(pty_pair):
    _, slave_name, device = pty_pair

    rclpy.init()
    try:
        # Create MotorBridge on the fake pty
        node = MotorBridge()
        node.cmd_timeout = 0.3 # 300 ms watchdog
        # Re-point esp32 link to the test slave
        node.esp32.close()
        node.esp32 = Esp32Motors(port=slave_name, baudrate=115200, timeout=0.1)
        time.sleep(0.1)
        assert node.esp32.connected

        # Send cmd_vel (forward 0.2 m/s)
        t = Twist()
        t.linear.x = 0.2
        node.on_cmd_vel(t)

        # Tick the bridge
        node.tick()
        time.sleep(0.05)

        with device.lock:
            m_lines = [l for l in device.received_lines if l.startswith('M ')]
            assert len(m_lines) > 0, 'Bridge did not send M command on cmd_vel'

        # Wait past 300 ms safety timeout without sending new cmd_vel
        time.sleep(0.35)
        node.tick()
        time.sleep(0.05)

        # Bridge must have stopped motors
        with device.lock:
            assert 'X' in device.received_lines or any(l.startswith('M 0 0') for l in device.received_lines)
            assert not node.esp32.is_driving

        # Test servo routing
        servo_states = []
        node.create_subscription(String, '/motor/servo_state', lambda m: servo_states.append(m.data), 10)

        # Send servo command to 120 deg
        m_servo = Int32MultiArray(data=[120, 240])
        node.on_servo_cmd(m_servo)
        time.sleep(0.05)

        with device.lock:
            assert 'S 120' in device.received_lines

        # Wait for the timed confirmation (slew ~0.15s + settle 0.15s)
        for _ in range(8):
            rclpy.spin_once(node, timeout_sec=0.1)
        assert any('moving,120' in s for s in servo_states)
        assert any('done,120' in s for s in servo_states)

        node.shutdown()
        node.destroy_node()
    finally:
        rclpy.shutdown()

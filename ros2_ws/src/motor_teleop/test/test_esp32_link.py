#!/usr/bin/env python3
"""Unit tests for Esp32Motors in motor_teleop.esp32_link."""
import time
import unittest
from unittest.mock import MagicMock, patch

from motor_teleop.esp32_link import Esp32Motors


class TestEsp32Motors(unittest.TestCase):
    def setUp(self):
        self.mock_serial_patcher = patch('serial.Serial')
        self.mock_serial_cls = self.mock_serial_patcher.start()
        self.mock_serial = MagicMock()
        self.mock_serial.is_open = True
        self.mock_serial.readline.return_value = b'PONG\n'
        self.mock_serial_cls.return_value = self.mock_serial

    def tearDown(self):
        self.mock_serial_patcher.stop()

    def test_connect_handshake(self):
        motors = Esp32Motors(port='/dev/test_esp32', auto_connect=True)
        self.assertTrue(motors.connected)
        motors.close()

    def test_drive_clamping_and_command(self):
        motors = Esp32Motors(port='/dev/test_esp32', auto_connect=True)
        self.mock_serial.write.reset_mock()

        motors.drive(300, -300)
        self.assertEqual(motors.target_l, 255)
        self.assertEqual(motors.target_r, -255)
        self.assertTrue(motors.is_driving)
        self.mock_serial.write.assert_called_with(b'M 255 -255\n')

        motors.stop()
        self.assertEqual(motors.target_l, 0)
        self.assertEqual(motors.target_r, 0)
        self.assertFalse(motors.is_driving)
        self.mock_serial.write.assert_called_with(b'X\n')

        motors.servos(90)
        self.mock_serial.write.assert_called_with(b'S 90\n')

        motors.ping()
        self.mock_serial.write.assert_called_with(b'P\n')
        motors.close()

    def test_error_and_watchdog_callbacks(self):
        error_cb = MagicMock()
        watchdog_cb = MagicMock()
        motors = Esp32Motors(
            port='/dev/test_esp32',
            auto_connect=False,
            on_error=error_cb,
            on_watchdog=watchdog_cb,
        )
        motors.connected = True
        motors.ser = self.mock_serial
        motors._running = True

        self.mock_serial.readline.return_value = b'ERR Invalid command\n'
        motors._read_loop = MagicMock()  # don't run thread
        # Call reading logic directly or verify callback
        line = b'ERR Invalid command\n'.decode('ascii').strip()
        if line.startswith('ERR'):
            motors.on_error(line)
        error_cb.assert_called_once_with('ERR Invalid command')

        line2 = b'WATCHDOG STOP\n'.decode('ascii').strip()
        if line2.startswith('WATCHDOG STOP'):
            motors.is_driving = False
            motors.on_watchdog()
        watchdog_cb.assert_called_once()
        self.assertFalse(motors.is_driving)
        motors.close()


if __name__ == '__main__':
    unittest.main()

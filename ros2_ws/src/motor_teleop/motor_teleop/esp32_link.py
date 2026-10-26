#!/usr/bin/env python3
"""ESP32 Serial Link for Hardleel Motor Bridge (BTS7960 + Dual Servos).

Protocol (115200 baud, ASCII, newline terminated):
    M <left> <right>    - Set motor speeds (-255..255)
    S <angle>           - Set both servos (0..180)
    X                   - Emergency stop (brakes motors)
    P                   - Ping (expects PONG)

Incoming from ESP32:
    READY               - ESP32 initialized after boot
    PONG                - Reply to ping
    ERR <reason>        - Command error
    WATCHDOG STOP       - Firmware watchdog stopped motors after 500ms without M command
"""

import argparse
import logging
import sys
import threading
import time
from typing import Callable, Optional

import serial

log = logging.getLogger('esp32_link')


class Esp32Motors:
    """Manages connection, keepalive, and messaging with the ESP32 motor controller."""

    def __init__(
        self,
        port: str = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
        baudrate: int = 115200,
        timeout: float = 0.1,
        auto_connect: bool = True,
        on_error: Optional[Callable[[str], None]] = None,
        on_watchdog: Optional[Callable[[], None]] = None,
        on_connect: Optional[Callable[[], None]] = None,
        on_disconnect: Optional[Callable[[], None]] = None,
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.on_error = on_error
        self.on_watchdog = on_watchdog
        self.on_connect = on_connect
        self.on_disconnect = on_disconnect

        self.ser: Optional[serial.Serial] = None
        self.connected = False
        self._lock = threading.Lock()

        self.target_l = 0
        self.target_r = 0
        self.is_driving = False

        self._running = False
        self._keepalive_thread: Optional[threading.Thread] = None
        self._reader_thread: Optional[threading.Thread] = None
        self._reconnect_thread: Optional[threading.Thread] = None
        self._last_m_time = 0.0

        if auto_connect:
            self.start()

    def start(self):
        """Starts worker threads and begins connection loop."""
        self._running = True
        self._connect_serial()

        self._reader_thread = threading.Thread(target=self._read_loop, name='esp32_reader', daemon=True)
        self._reader_thread.start()

        self._keepalive_thread = threading.Thread(target=self._keepalive_loop, name='esp32_keepalive', daemon=True)
        self._keepalive_thread.start()

        self._reconnect_thread = threading.Thread(target=self._reconnect_loop, name='esp32_reconnect', daemon=True)
        self._reconnect_thread.start()

    def _connect_serial(self) -> bool:
        with self._lock:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.close()
                except Exception:
                    pass
            self.ser = None
            self.connected = False

            try:
                # Open serial port
                s = serial.Serial()
                s.port = self.port
                s.baudrate = self.baudrate
                s.timeout = self.timeout
                s.rtscts = False
                s.dsrdtr = False
                s.open()
                # Ensure DTR and RTS are cleared on real serial ports (ignore on pseudo-terminals)
                try:
                    s.dtr = False
                    s.rts = False
                except (OSError, IOError):
                    pass

                # Settle and flush any garbage/boot messages
                time.sleep(0.15)
                try:
                    s.reset_input_buffer()
                    s.reset_output_buffer()
                except (OSError, IOError):
                    pass

                # Handshake: ping ESP32 until PONG or READY
                handshake_ok = False
                t0 = time.monotonic()
                while time.monotonic() - t0 < 2.0:
                    s.write(b'P\n')
                    s.flush()
                    line = s.readline().decode('ascii', errors='ignore').strip()
                    if line in ('PONG', 'READY') or line.endswith('PONG') or line.endswith('READY'):
                        handshake_ok = True
                        break
                    time.sleep(0.1)

                if not handshake_ok:
                    log.warning(f'ESP32 opened on {self.port} but handshake timed out (no PONG/READY)')

                self.ser = s
                self.connected = True
                log.info(f'Connected to ESP32 on {self.port} at {self.baudrate} baud')
                if self.on_connect:
                    try:
                        self.on_connect()
                    except Exception as e:
                        log.error(f'Error in on_connect callback: {e}')
                return True

            except (serial.SerialException, OSError) as e:
                log.error(f'Failed to open ESP32 serial port {self.port}: {e}')
                self.connected = False
                return False

    def drive(self, left: int, right: int):
        """Sets target wheel speeds (-255..255). Sends M command immediately."""
        left = max(-255, min(255, int(left)))
        right = max(-255, min(255, int(right)))
        self.target_l = left
        self.target_r = right
        self.is_driving = (left != 0 or right != 0)
        self._send_command(f'M {left} {right}')

    def stop(self):
        """Emergency stop: stops both motors and sends X command."""
        self.target_l = 0
        self.target_r = 0
        self.is_driving = False
        self._send_command('X')

    def servos(self, angle: int):
        """Sets both drop servos to target angle (0..180)."""
        angle = max(0, min(180, int(angle)))
        self._send_command(f'S {angle}')

    def ping(self):
        """Sends ping 'P' command."""
        self._send_command('P')

    def _send_command(self, cmd: str) -> bool:
        if not self.connected or not self.ser or not self.ser.is_open:
            return False
        with self._lock:
            try:
                self.ser.write((cmd.strip() + '\n').encode('ascii'))
                self.ser.flush()
                if cmd.startswith('M'):
                    self._last_m_time = time.monotonic()
                return True
            except (serial.SerialException, OSError) as e:
                log.error(f'Serial write error on {self.port}: {e}')
                self._handle_disconnect()
                return False

    def _handle_disconnect(self):
        if self.connected:
            self.connected = False
            log.error(f'ESP32 disconnected from {self.port}')
            if self.on_disconnect:
                try:
                    self.on_disconnect()
                except Exception as e:
                    log.error(f'Error in on_disconnect callback: {e}')

    def _keepalive_loop(self):
        """Resends M command at least every 100 ms (10 Hz) while driving to prevent ESP32 500ms watchdog trip."""
        while self._running:
            time.sleep(0.1)  # 10 Hz
            if self.connected and self.is_driving:
                # If driving, periodically re-send M command to keep ESP32 watchdog fed
                self._send_command(f'M {self.target_l} {self.target_r}')

    def _read_loop(self):
        """Continuously reads incoming lines from the ESP32."""
        while self._running:
            s = self.ser
            if not self._running or not self.connected or s is None:
                time.sleep(0.05)
                continue
            try:
                line = s.readline().decode('ascii', errors='ignore').strip()
            except (serial.SerialException, OSError, TypeError) as e:
                if self._running:
                    log.error(f'Serial read exception: {e}')
                    self._handle_disconnect()
                continue

            if not line:
                continue

            if line.startswith('ERR'):
                log.warning(f'ESP32 ERR: {line}')
                if self.on_error:
                    self.on_error(line)
            elif line.startswith('WATCHDOG STOP'):
                log.warning('ESP32 WATCHDOG STOP: firmware cut motor power due to command gap >500ms')
                self.is_driving = False
                if self.on_watchdog:
                    self.on_watchdog()
            elif line == 'READY':
                log.info('ESP32 status: READY')
            elif line == 'PONG':
                log.debug('ESP32 received PONG')
            else:
                log.debug(f'ESP32 incoming: {line}')

    def _reconnect_loop(self):
        """Retries connection every 1.0 second if disconnected."""
        while self._running:
            time.sleep(1.0)
            if not self.connected:
                log.info(f'Attempting reconnect to ESP32 on {self.port}...')
                self._connect_serial()

    def close(self):
        """Stops motors and closes serial connection cleanly."""
        self._running = False
        try:
            self.stop()
        except Exception:
            pass
        with self._lock:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.close()
                except Exception:
                    pass
            self.connected = False
        if self._keepalive_thread and self._keepalive_thread.is_alive():
            self._keepalive_thread.join(timeout=0.3)
        if self._reader_thread and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=0.3)
        if self._reconnect_thread and self._reconnect_thread.is_alive():
            self._reconnect_thread.join(timeout=0.3)
        log.info('Esp32Motors closed.')


def main():
    logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(name)s: %(message)s')
    parser = argparse.ArgumentParser(description='Interactive test link for ESP32 motor controller')
    parser.add_argument('--port', default='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
                        help='Serial port path')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    args = parser.parse_args()

    print(f'Connecting to ESP32 on {args.port}...')
    motors = Esp32Motors(port=args.port, baudrate=args.baud)

    print('Commands: m <left> <right> | s <angle> | x (stop) | p (ping) | q (quit)')
    try:
        while True:
            cmd = input('> ').strip()
            if not cmd:
                continue
            if cmd == 'q':
                break
            elif cmd.startswith('m'):
                parts = cmd.split()
                if len(parts) == 3:
                    motors.drive(int(parts[1]), int(parts[2]))
                else:
                    print('Usage: m <left> <right>')
            elif cmd.startswith('s'):
                parts = cmd.split()
                if len(parts) == 2:
                    motors.servos(int(parts[1]))
                else:
                    print('Usage: s <angle>')
            elif cmd == 'x':
                motors.stop()
            elif cmd == 'p':
                motors.ping()
            else:
                print('Unknown command. Use m, s, x, p, or q.')
    except KeyboardInterrupt:
        pass
    finally:
        motors.close()


if __name__ == '__main__':
    main()

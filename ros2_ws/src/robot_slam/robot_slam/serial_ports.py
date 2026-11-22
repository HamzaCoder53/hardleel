#!/usr/bin/env python3
"""Find which USB-serial port is the LIDAR and which is the ESP32 motor board.

Both use identical CP2102 adapters (same USB serial number), so Linux cannot tell them apart and the
physical hub port changes when plugs are swapped (19 Nov: lidar and ESP32 swapped -> no scans, motor
commands sent to the lidar). So each /dev/ttyUSB* is asked what it is:
  ESP32  answers "h\n" with its help text ("this help" / "stop motors")   - harmless, nothing moves
  lidar  answers GET_INFO (A5 50) with A5 5A                              - harmless, only spins the lidar
If only the ESP32 answers and exactly one other port exists, that port is the lidar (by elimination:
the A1 sometimes stays silent until it has been reset).
Run ONCE before the nodes open the ports (pi_mapping.launch.py / start_robot.sh do that).

CLI:  python3 -m robot_slam.serial_ports            -> JSON
      python3 -m robot_slam.serial_ports --shell    -> LIDAR_PORT=... ESP32_PORT=... (for bash eval)
"""
import glob
import json
import sys
import time

FALLBACK = {'esp32': '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-port0',
            'lidar': '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0-port0'}


def _open(port, dtr):
    import serial
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = port, 115200, 0.2
    s.dtr, s.rts = dtr, False          # set before open: an asserted line can reset / hold the ESP32
    s.open()
    return s


def is_esp32(port):
    try:
        s = _open(port, False)
    except Exception:  # noqa: BLE001 - busy / gone
        return False
    try:
        # encoder firmware (22 Nov) streams 'E,...' telemetry at 20 Hz and prints '#' info lines
        data, t = b'', time.time()
        while time.time() - t < 1.5:   # also covers a reboot caused by opening the port
            data += s.read(256)
        if b'\nE,' in data or data.startswith(b'E,') or b'# ESP32' in data or b'# ceiling' in data:
            return True
        s.reset_input_buffer()
        s.write(b'I\n')                # encoder firmware: '# ceiling duty ...'
        s.write(b'h\n')                # old motor_code.ino: help text
        data, t = b'', time.time()
        while time.time() - t < 1.2:
            data += s.read(256)
        return any(k in data for k in (b'this help', b'stop motors', b'# ceiling', b'\nE,'))
    finally:
        s.close()


def is_lidar(port):
    for dtr in (False, True):          # the A1 answered with one or the other line state on different days
        try:
            s = _open(port, dtr)
        except Exception:  # noqa: BLE001
            return False
        try:
            time.sleep(0.5)
            for _ in range(2):
                s.write(b'\xa5\x25')   # stop scanning
                time.sleep(0.2)
                s.reset_input_buffer()
                s.write(b'\xa5\x50')   # GET_INFO
                time.sleep(0.4)
                if s.read(27).startswith(b'\xa5\x5a'):
                    return True
        finally:
            s.close()
    return False


def detect():
    """{'esp32': port|None, 'lidar': port|None, 'how': {...}, 'ports': [...]}"""
    ports = sorted(glob.glob('/dev/ttyUSB*'))
    res = {'esp32': None, 'lidar': None, 'how': {}, 'ports': ports}
    for p in ports:
        if is_esp32(p):
            res['esp32'], res['how']['esp32'] = p, 'answered h'
            break
    for p in ports:
        if p != res['esp32'] and is_lidar(p):
            res['lidar'], res['how']['lidar'] = p, 'answered GET_INFO'
            break
    others = [p for p in ports if p != res['esp32']]
    if res['lidar'] is None and res['esp32'] and len(others) == 1:
        res['lidar'], res['how']['lidar'] = others[0], 'by elimination (lidar silent)'
    return res


def ports_or_fallback():
    """detect() with the fixed hub-port defaults for anything not found."""
    try:
        res = detect()
    except Exception as e:  # noqa: BLE001
        res = {'esp32': None, 'lidar': None, 'how': {'error': str(e)}, 'ports': []}
    for k in ('esp32', 'lidar'):
        if not res[k]:
            res[k], res['how'][k] = FALLBACK[k], 'FALLBACK hub port (not detected)'
    return res


def main():
    res = ports_or_fallback()
    if '--shell' in sys.argv:
        print(f"LIDAR_PORT={res['lidar']} ESP32_PORT={res['esp32']}")
        print(f"# lidar: {res['how'].get('lidar')} | esp32: {res['how'].get('esp32')}", file=sys.stderr)
    else:
        print(json.dumps(res, indent=1))


if __name__ == '__main__':
    main()

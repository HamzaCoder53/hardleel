#!/usr/bin/env python3
"""Move the servo (Nano 33 BLE, pin D9) to any angle - directly over the Nano's USB port.

Use this ONLY when the robot stack is NOT running (the IMU node owns the Nano's port while it runs).
With the robot stack running use instead:   ros2 topic pub --once /servo/cmd std_msgs/msg/String "{data: '45'}"

  python3 ~/pi_tools/servo.py 45          move to 45 degrees (0..180)
  python3 ~/pi_tools/servo.py off         servo limp
  python3 ~/pi_tools/servo.py sweep       0 -> 180 -> 90 test
  python3 ~/pi_tools/servo.py             interactive: type an angle, Enter; q to quit
"""
import glob
import sys
import time

import serial


def find_port():
    p = sorted(glob.glob('/dev/serial/by-id/usb-Arduino*')) or sorted(glob.glob('/dev/ttyACM*'))
    if not p:
        sys.exit('Nano not found on USB (/dev/serial/by-id/usb-Arduino*)')
    return p[0]


class Nano:
    def __init__(self):
        try:
            self.ser = serial.Serial(find_port(), 115200, timeout=0)
        except serial.SerialException as e:
            sys.exit(f'cannot open the Nano ({e}) - is the robot stack running? Then use /servo/cmd instead.')
        self.buf = b''
        time.sleep(0.5)

    def cmd(self, text, timeout=1.5):
        self.ser.reset_input_buffer()
        self.buf = b''
        self.ser.write((text + '\n').encode())
        end = time.time() + timeout
        while time.time() < end:
            self.buf += self.ser.read(self.ser.in_waiting or 1)
            for line in self.buf.split(b'\n')[:-1]:
                s = line.decode(errors='ignore').strip()
                if s.startswith('S,'):
                    return s
            time.sleep(0.005)
        return None


def to_cmd(a):
    a = a.strip().lower()
    if a == 'off':
        return 'S,off'
    if a in ('?', ''):
        return 'S,?'
    return f'S,{max(0, min(180, int(float(a))))}'


def main():
    nano = Nano()
    args = sys.argv[1:]
    if args and args[0] == 'sweep':
        for a in (90, 0, 45, 90, 135, 180, 90):
            print(f'S,{a:<4d} -> {nano.cmd(f"S,{a}")}')
            time.sleep(0.6)
        return
    if args:
        c = to_cmd(args[0])
        print(f'{c} -> {nano.cmd(c)}')
        return
    print('type an angle 0..180, "off", "?" or q to quit')
    while True:
        try:
            a = input('angle> ')
        except (EOFError, KeyboardInterrupt):
            break
        if a.strip().lower() in ('q', 'quit', 'exit'):
            break
        try:
            c = to_cmd(a)
        except ValueError:
            print('  not a number')
            continue
        print(f'  {c} -> {nano.cmd(c)}')


if __name__ == '__main__':
    main()

"""Esp32Motors with protocol='motor_code' (the team's motor_code.ino, 26 Oct) against a fake sketch on a pty:
handshake via 'h', timed drive commands re-sent while driving, x / s, and the 'Timed run finished' report."""
import os
import pty
import threading
import time
import tty

import pytest

from motor_teleop.esp32_link import Esp32Motors


class FakeSketch(threading.Thread):
    """Answers like motor_code.ino: help on 'h', echo lines for m / s / x."""

    def __init__(self):
        super().__init__(daemon=True)
        self.master, slave = pty.openpty()
        tty.setraw(self.master)
        self.port = os.ttyname(slave)
        self.lines = []
        self.running = True

    def say(self, text):
        os.write(self.master, (text + '\n').encode())

    def run(self):
        buf = b''
        while self.running:
            try:
                buf += os.read(self.master, 256)
            except OSError:
                break
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                s = line.decode().strip()
                self.lines.append(s)
                if s == 'h':
                    self.say('Commands:')
                    self.say('  m <left> <right> [ms]  speeds -255..255')
                elif s.startswith('m '):
                    p = s.split()
                    self.say(f'Left {p[1]}, right {p[2]} for {p[3]} ms')
                elif s == 'x':
                    self.say('Stop')


@pytest.fixture
def sketch():
    s = FakeSketch()
    s.start()
    yield s
    s.running = False


def test_motor_code_protocol(sketch):
    m = Esp32Motors(port=sketch.port, protocol='motor_code', hold_ms=400)
    try:
        assert m.connected
        assert 'h' in sketch.lines                         # handshake used the help command
        m.drive(120, -80)
        time.sleep(0.35)                                   # keepalive re-sends every 100 ms
        drives = [l for l in sketch.lines if l == 'm 120 -80 400']
        assert len(drives) >= 3
        m.servos(30)
        m.stop()
        time.sleep(0.1)
        assert 's 30' in sketch.lines and 'x' in sketch.lines
        assert not any(l.startswith(('M ', 'S ', 'X', 'P')) for l in sketch.lines)
    finally:
        m.close()


def test_timed_run_finished_while_driving_reports_watchdog(sketch):
    hits = []
    m = Esp32Motors(port=sketch.port, protocol='motor_code', on_watchdog=lambda: hits.append(1))
    try:
        m.target_l, m.target_r, m.is_driving = 100, 100, True
        sketch.say('Timed run finished')
        time.sleep(0.3)
        assert hits == [1] and not m.is_driving
    finally:
        m.close()


def test_bad_protocol_rejected():
    with pytest.raises(ValueError):
        Esp32Motors(port='/dev/null', auto_connect=False, protocol='pico')

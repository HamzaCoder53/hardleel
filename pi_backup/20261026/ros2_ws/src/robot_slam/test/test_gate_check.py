"""cd ros2_ws/src/robot_slam && python3 -m pytest test/test_gate_check.py -q"""
import math
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from robot_slam.gate_check import GatePoller, sector_median  # noqa: E402

INC = math.radians(1.0)
AMIN = -math.pi


def scan(fill=2.0, n=360):
    return [fill] * n


def test_basic_median():
    assert sector_median(scan(2.0), AMIN, INC, 0, 10) == 2.0


def test_wraparound_at_180():
    r = scan(5.0)
    for i in range(360):                       # beams near +-180 deg = 1.0 m
        a = math.degrees(AMIN + i * INC)
        if abs(abs(a) - 180) < 5:
            r[i] = 1.0
    assert sector_median(r, AMIN, INC, 180, 8) == 1.0
    assert sector_median(r, AMIN, INC, -180, 8) == 1.0
    assert sector_median(r, AMIN, INC, 0, 8) == 5.0


def test_angle_min_zero_scan_wraps():
    r = [3.0] * 360                            # angles 0..359 deg, so 350 deg -> -10
    assert sector_median(r, 0.0, INC, -5, 3) == 3.0


def test_invalid_beams_ignored():
    r = [float('nan'), float('inf'), 0.05, 20.0, 1.0, 2.0, 3.0]
    assert sector_median(r, -3 * INC, INC, 0, 10) == 2.0


def test_empty_sector():
    r = [float('inf')] * 360
    assert sector_median(r, AMIN, INC, 0, 10) == float('inf')
    assert sector_median(r, AMIN, INC, 0, 10, treat_empty_as_open=False) is None
    assert sector_median(scan(2.0), AMIN, INC, 0, 1, min_valid=10) == float('inf')


def test_open_confirm():
    g = GatePoller(100.0, 15.0, 1.0, 3)
    assert g.update(101, 2.0) is None
    assert g.update(102, 2.0) is None
    assert g.update(103, 0.4) is None          # streak broken
    assert g.update(104, 2.0) is None
    assert g.update(105, 2.0) is None
    assert g.update(106, 2.0) == ('path2', 6)
    assert g.decision == 'path2' and g.wait_s == 6 and g.polls == 6 and g.last_median == 2.0


def test_timeout_path1():
    g = GatePoller(0.0, 10.0, 1.0)
    assert g.update(5, 0.3) is None
    assert g.update(10, 0.3) == ('path1', 10)
    assert g.decision == 'path1'


def test_none_does_not_count():
    g = GatePoller(0.0, 10.0, 1.0, 2)
    g.update(1, 2.0)
    assert g.update(2, None) is None           # neither resets nor counts
    assert g.polls == 1
    assert g.update(3, 2.0) == ('path2', 3)
    g2 = GatePoller(0.0, 4.0, 1.0)
    assert g2.update(4, None) == ('path1', 4)  # timeout still fires on None

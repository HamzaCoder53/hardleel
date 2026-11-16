"""Offline checks for wheel_odom (no ROS needed): python3 -m pytest test/test_wheel_odom.py"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from motor_teleop.wheel_odom import WheelOdom, parse_line  # noqa: E402


def make(**kw):
    # 1 count = 1 cm, track 0.3 m
    return WheelOdom(ticks_per_rev=100.0, wheel_diameter=1.0 / math.pi, track_width=0.3, window=0.2, **kw)


def feed(o, rate_l, rate_r, seconds, hz=20, faults=None):
    """Drive at constant counts/s per side; return the last result."""
    r = None
    c = getattr(o, '_c', [0.0] * 4)
    t = getattr(o, '_t', 0)
    for _ in range(int(seconds * hz)):
        t += 1000 // hz
        c = [c[0] + rate_l / hz, c[1] + rate_l / hz, c[2] + rate_r / hz, c[3] + rate_r / hz]
        raw = [int(round(v)) for v in c]
        for i in faults or ():
            raw[i] = 0
        r = o.update(t, raw)
    o._c, o._t = c, t
    return r


def test_parse():
    assert parse_line('E 1234 1 -2 3 4') == (1234, [1, -2, 3, 4])
    assert parse_line('Left 70, right 70 for 200 ms') is None
    assert parse_line('E 12 a b c d') is None


def test_straight():
    o = make()
    r = feed(o, 20, 20, 2.0)              # 20 cm/s for 2 s
    assert abs(r['vx'] - 0.20) < 0.01
    assert abs(r['wz']) < 1e-6
    assert abs(r['x'] - 0.40) < 0.02 and abs(r['y']) < 1e-6


def test_spin_in_place():
    o = make()
    r = feed(o, -15, 15, 1.0)             # wheels +-0.15 m/s, track 0.3 -> 1 rad/s
    assert abs(r['vx']) < 1e-6
    assert abs(r['wz'] - 1.0) < 0.05
    assert abs(r['yaw'] - 1.0) < 0.1      # first sample is only the baseline (robot starts at rest)


def test_left_inverted_sign():
    o = make(sign_left=-1)
    r = feed(o, -20, 20, 1.0)             # sketch reports left negative when it is wired reversed
    assert abs(r['vx'] - 0.20) < 0.01 and abs(r['wz']) < 1e-6


def test_dead_sensor_uses_other_wheel():
    o = make()
    r = feed(o, 20, 20, 1.0, faults=[1])  # left-rear sensor unplugged
    assert r['faults'] == ['left_rear']   # names the DEAD sensor
    assert abs(r['vx'] - 0.20) < 0.01


def test_esp32_reboot_keeps_pose():
    o = make()
    feed(o, 20, 20, 1.0)
    x = o.x
    o._t, o._c = 0, [0.0] * 4             # millis and counts start again
    assert o.update(10, [0, 0, 0, 0]) is None or abs(o.x - x) < 1e-9
    r = feed(o, 20, 20, 1.0)
    assert abs(r['x'] - (x + 0.2)) < 0.03

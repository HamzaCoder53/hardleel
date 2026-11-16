"""Wheel odometry from the 4 DIY single-channel encoders (no ROS in here: unit-testable).

Input: ESP32 lines 'E <millis> <lf> <lr> <rf> <rr>' = cumulative counts, already signed by the
sketch with each side's commanded direction (sketch convention, before the Pi's invert_left/right).

Skid steer: wheels slip sideways in every turn, so the encoder YAW is poor - trust vx, give wz a
large covariance and let the IMU own yaw. Speeds use a sliding window (single-channel discs give
only ~40 counts per turn, so one 50 ms sample is too coarse).
"""
import math
from collections import deque

LF, LR, RF, RR = range(4)


def parse_line(text):
    """'E 1234 1 2 3 4' -> (1234, [1, 2, 3, 4]) or None."""
    parts = text.split()
    if len(parts) != 6 or parts[0] != 'E':
        return None
    try:
        return int(parts[1]), [int(p) for p in parts[2:]]
    except ValueError:
        return None


def side_delta(front, rear, min_counts=3):
    """Mean of the two wheels of a side. If one sensor stays at 0 while the other clearly moves
    (sensor unplugged / disc slipped), use the moving one and flag it."""
    if front == 0 and abs(rear) >= min_counts:
        return float(rear), 'front'
    if rear == 0 and abs(front) >= min_counts:
        return float(front), 'rear'
    return (front + rear) / 2.0, None


class WheelOdom:
    def __init__(self, ticks_per_rev=40.0, wheel_diameter=0.065, track_width=0.30, window=0.2,
                 sign_left=1, sign_right=1):
        self.ticks_per_rev, self.wheel_diameter = ticks_per_rev, wheel_diameter
        self.track_width, self.window = track_width, window
        self.sign_left, self.sign_right = sign_left, sign_right
        self.hist = deque()           # (t_s, counts[4]) with the Pi-side signs applied
        self.x = self.y = self.yaw = 0.0
        self.faults = set()           # wheels whose sensor looked dead in the last update

    @property
    def m_per_count(self):
        return math.pi * self.wheel_diameter / self.ticks_per_rev

    def reset_pose(self):
        self.x = self.y = self.yaw = 0.0

    def update(self, millis, raw):
        """Feed one E sample. Returns dict(vx, wz, wheel=[4 m/s], x, y, yaw, faults) or None
        until there are two samples."""
        t = millis / 1000.0
        sl, sr = self.sign_left, self.sign_right
        c = [raw[LF] * sl, raw[LR] * sl, raw[RF] * sr, raw[RR] * sr]
        if self.hist and t <= self.hist[-1][0]:
            self.hist.clear()         # ESP32 rebooted (millis went back): start again, keep the pose
        prev = self.hist[-1] if self.hist else None
        self.hist.append((t, c))
        while len(self.hist) > 2 and self.hist[1][0] <= t - self.window:
            self.hist.popleft()
        if prev is None:
            return None

        m = self.m_per_count
        self.faults = set()
        # pose: integrate the step since the previous sample (midpoint heading)
        d = [a - b for a, b in zip(c, prev[1])]
        dl, _ = side_delta(d[LF], d[LR])
        dr, _ = side_delta(d[RF], d[RR])
        dl, dr = dl * m, dr * m
        ds, dyaw = (dl + dr) / 2.0, (dr - dl) / self.track_width
        mid = self.yaw + dyaw / 2.0
        self.x += ds * math.cos(mid)
        self.y += ds * math.sin(mid)
        self.yaw = math.atan2(math.sin(self.yaw + dyaw), math.cos(self.yaw + dyaw))

        # speeds: over the window
        t0, c0 = self.hist[0]
        dt = t - t0
        w = [(a - b) * m / dt for a, b in zip(c, c0)]
        vl, fl = side_delta(c[LF] - c0[LF], c[LR] - c0[LR])
        vr, fr = side_delta(c[RF] - c0[RF], c[RR] - c0[RR])
        vl, vr = vl * m / dt, vr * m / dt
        if fl:
            self.faults.add('left_' + fl)
        if fr:
            self.faults.add('right_' + fr)
        return {'vx': (vl + vr) / 2.0, 'wz': (vr - vl) / self.track_width, 'wheel': w,
                'x': self.x, 'y': self.y, 'yaw': self.yaw, 'faults': sorted(self.faults)}

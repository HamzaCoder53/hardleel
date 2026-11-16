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


# ---------------------------------------------------------------------------------------------
# PWM "odometry" (16 Nov): the speed the PWM SHOULD give, until real encoders are fitted.
# It is a guess, not a measurement: it keeps "moving" when the robot stalls or pushes a wall,
# and it changes with battery and floor. Publish it with a large covariance so rf2o + IMU win.
# ---------------------------------------------------------------------------------------------

def pwm_to_speed(pwm, stall_pwm, pwm_lo, v_lo, pwm_hi, v_hi):
    """Signed PWM -> signed wheel speed (m/s). Below stall_pwm = 0; linear through the two
    calibration points (pwm_lo, v_lo) and (pwm_hi, v_hi), extrapolated, never negative."""
    a = abs(pwm)
    if a == 0 or a < stall_pwm:
        return 0.0
    if pwm_hi == pwm_lo:
        v = v_hi
    else:
        v = v_lo + (a - pwm_lo) * (v_hi - v_lo) / (pwm_hi - pwm_lo)
    return max(0.0, v) * (1.0 if pwm > 0 else -1.0)


class PwmOdom:
    def __init__(self, stall_pwm=70, pwm_lo=70, v_lo=0.10, pwm_hi=75, v_hi=0.15, tau=0.15,
                 track_width=0.30):
        self.stall_pwm, self.pwm_lo, self.v_lo = stall_pwm, pwm_lo, v_lo
        self.pwm_hi, self.v_hi, self.tau, self.track_width = pwm_hi, v_hi, tau, track_width
        self.v = [0.0, 0.0]           # modelled left/right wheel speed (lags the command)
        self.x = self.y = self.yaw = 0.0

    def update(self, dt, pwm_left, pwm_right):
        """pwm_*: forward-positive PWM actually sent (before the wiring inversion)."""
        dt = max(0.0, min(dt, 0.5))
        k = 1.0 if self.tau <= 0 else min(1.0, dt / self.tau)
        for i, p in enumerate((pwm_left, pwm_right)):
            tgt = pwm_to_speed(p, self.stall_pwm, self.pwm_lo, self.v_lo, self.pwm_hi, self.v_hi)
            self.v[i] += (tgt - self.v[i]) * k
            if tgt == 0.0 and abs(self.v[i]) < 1e-3:
                self.v[i] = 0.0
        vl, vr = self.v
        vx, wz = (vl + vr) / 2.0, (vr - vl) / self.track_width
        mid = self.yaw + wz * dt / 2.0
        self.x += vx * dt * math.cos(mid)
        self.y += vx * dt * math.sin(mid)
        self.yaw = math.atan2(math.sin(self.yaw + wz * dt), math.cos(self.yaw + wz * dt))
        return {'vx': vx, 'wz': wz, 'x': self.x, 'y': self.y, 'yaw': self.yaw}

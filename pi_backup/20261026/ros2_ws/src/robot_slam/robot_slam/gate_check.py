"""Pure helpers for the gate decision (no ROS): median range in a scan sector and a confirm-N poller."""
import math


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def sector_median(ranges, angle_min, angle_increment, center_deg, half_width_deg,
                  range_min=0.12, range_max=12.0, min_valid=3, treat_empty_as_open=True):
    """Median of valid ranges whose beam angle (wrapped to [-pi, pi]) is within center +- half_width.

    Invalid: nan/inf/<range_min/>range_max. With fewer than min_valid valid beams the result is inf
    (treat_empty_as_open) or None. An A1 returns no echo through an open gap to far walls beyond range,
    so an empty sector usually means "open"; set treat_empty_as_open=False to get None instead.
    """
    c = math.radians(center_deg)
    hw = math.radians(half_width_deg)
    vals = []
    for i, r in enumerate(ranges):
        if r is None or not math.isfinite(r) or r < range_min or r > range_max:
            continue
        a = _wrap(angle_min + i * angle_increment)
        if abs(_wrap(a - c)) <= hw:
            vals.append(r)
    if len(vals) < min_valid:
        return float('inf') if treat_empty_as_open else None
    vals.sort()
    n = len(vals)
    return vals[n // 2] if n % 2 else 0.5 * (vals[n // 2 - 1] + vals[n // 2])


class GatePoller:
    """Feed (t, median) polls. Returns ('path2', wait_s) after open_confirm_n consecutive polls with
    median > threshold, ('path1', wait_s) once t - t_start >= max_wait_s, else None. None medians are ignored."""

    def __init__(self, t_start, max_wait_s, threshold_m, open_confirm_n=3):
        self.t_start = t_start
        self.max_wait_s = max_wait_s
        self.threshold_m = threshold_m
        self.open_confirm_n = open_confirm_n
        self.decision = None
        self.wait_s = 0.0
        self.polls = 0
        self.last_median = None
        self._streak = 0

    def update(self, t, median):
        if self.decision is not None:
            return (self.decision, self.wait_s)
        self.wait_s = t - self.t_start
        if median is not None:
            self.polls += 1
            self.last_median = median
            self._streak = self._streak + 1 if median > self.threshold_m else 0
            if self._streak >= self.open_confirm_n:
                self.decision = 'path2'
                return (self.decision, self.wait_s)
        if self.wait_s >= self.max_wait_s:
            self.decision = 'path1'
            return (self.decision, self.wait_s)
        return None

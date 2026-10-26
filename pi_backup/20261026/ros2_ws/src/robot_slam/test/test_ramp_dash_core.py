"""ramp_dash_core: steering never above 100 %, run-up, wall geometry, and whole simulated dashes (clean climb,
bog-down + one retry, two bog-downs -> FAILED, blocked ramp)."""
import math

import numpy as np
import pytest

rd = pytest.importorskip('robot_slam.ramp_dash_core')


def test_steer_reduces_only_the_faster_side():
    for u in np.linspace(-1.5, 1.5, 31):
        l, r = rd.steer_pwm(255, u, 0.6)
        assert max(l, r) == 255 and min(l, r) >= 255 * 0.4 - 1
    assert rd.steer_pwm(255, 0.3, 0.6)[0] < 255          # turn left: LEFT side slower
    assert rd.steer_pwm(-160, 0.0, 0.6) == (-160, -160)


def corridor_scan(lat=0.0, heading=0.0, half=0.36, rear=1.0, front=float('inf')):
    """Synthetic 720-beam scan in a straight corridor (walls at y = +/-half in the strip frame)."""
    a = np.radians(np.arange(-180, 180, 0.5))
    r = np.full(a.shape, np.inf)
    th = a + heading                                        # beam direction in the strip frame
    with np.errstate(divide='ignore', invalid='ignore'):
        dl = (half - lat) / np.sin(th)
        dr = (-half - lat) / np.sin(th)
    r = np.where((np.sin(th) > 0.05) & (dl > 0), dl, r)
    r = np.where((np.sin(th) < -0.05) & (dr > 0), dr, r)
    back = np.abs(np.abs(a) - math.pi) < math.radians(5)
    r[back] = rear
    fwd = np.abs(a) < math.radians(5)
    r[fwd] = front
    return r, a[0], a[1] - a[0]


def test_geometry_lat_and_heading():
    g = rd.ScanGeometry(dict(rd.DEFAULTS))
    r, a0, da = corridor_scan(lat=0.05, heading=math.radians(3))
    geo = g(r, a0, da, tilted=False)
    assert geo['lat'] == pytest.approx(0.05, abs=0.01)
    assert geo['heading'] == pytest.approx(math.radians(3), abs=math.radians(0.7))
    assert geo['clear_pts'] == 0


class World:
    """Tiny kinematic world: speed ~ PWM, a ramp along x (up 0.4 m, top 0.4 m, down 0.4 m from x_ramp)."""

    def __init__(self, x_ramp=0.5, bog_until=None):
        self.x, self.lat, self.th, self.t = 0.0, 0.0, 0.0, 0.0
        self.x_ramp, self.bog_until = x_ramp, bog_until      # bog: no progress on the slope for the first attempt(s)
        self.bogs = 0

    def pitch(self):
        s = self.x - self.x_ramp
        return math.radians(14) if 0 <= s < 0.4 else (math.radians(-14) if 0.8 <= s < 1.2 else 0.0)

    def step(self, l, r, dt):
        v = (l + r) / 2 / 255 * 0.8
        w = (r - l) / 255 * 0.8 / 0.25
        on_slope = 0 <= self.x - self.x_ramp < 0.4
        if on_slope and v > 0 and self.bog_until is not None and self.bogs < self.bog_until:
            v = 0.0                                          # stuck in the dirt
            self.stuck_t = getattr(self, 'stuck_t', 0) + dt
        self.x += v * math.cos(self.th) * dt
        self.lat += v * math.sin(self.th) * dt
        self.th += w * dt
        if getattr(self, 'stuck_t', 0) > 0 and self.x < self.x_ramp - 0.05:
            self.bogs, self.stuck_t = self.bogs + 1, 0        # backed off the slope: the next try works


def run(world, cfg=None, max_s=30.0):
    d = rd.Dash(cfg)
    att = rd.Attitude(d.c)
    geo = rd.ScanGeometry(d.c)
    dt = 0.01
    att.update(0.0, 0.0, 0.0, 9.81, 0.0, 0.0)
    d.start(0.0, att)
    max_pwm = 0
    for k in range(int(max_s / dt)):
        t = k * dt
        p = world.pitch()
        att.update(t, 9.81 * math.sin(p), 0.0, 9.81 * math.cos(p), 0.0, 0.0)
        att.pitch = p                                        # exact attitude in this test
        att.yaw, att.yaw_rate = world.th, 0.0
        if k % 18 == 0:
            r, a0, da = corridor_scan(world.lat, world.th, rear=1.0 + world.x)
            d.scan(geo(r, a0, da, tilted=abs(p) > math.radians(4)))
        l, r_ = d.step(t, att)
        max_pwm = max(max_pwm, abs(l), abs(r_))
        world.step(l, r_, dt)
        if d.phase in ('DONE', 'FAILED'):
            break
    return d, max_pwm


def test_clean_climb():
    w = World()
    d, mx = run(w)
    assert d.phase == 'DONE', d.events
    assert d.result['ok'] and d.result['attempts'] == 1
    assert mx <= 255
    assert abs(w.lat) < 0.05                                 # stayed centred


def test_bog_then_retry_succeeds():
    w = World(bog_until=1)
    d, _ = run(w)
    assert d.phase == 'DONE', d.events
    assert d.result['attempts'] == 2
    assert any('retry' in e[1] for e in d.events)


def test_two_bogs_fail():
    w = World(bog_until=5)
    d, _ = run(w)
    assert d.phase == 'FAILED' and 'bogged' in d.result['reason']


def test_blocked_ramp_never_launches():
    d = rd.Dash({'clear_wait_s': 1.0})
    att = rd.Attitude(d.c)
    geo = rd.ScanGeometry(d.c)
    att.update(0.0, 0.0, 0.0, 9.81, 0.0, 0.0)
    d.start(0.0, att)
    r, a0, da = corridor_scan(front=0.4)                    # something 0.4 m ahead in the corridor
    for k in range(300):
        d.scan(geo(r, a0, da, False))
        assert d.step(k * 0.01, att) == (0, 0) or d.phase == 'ALIGN'
    assert d.phase == 'FAILED' and 'blocked' in d.result['reason']

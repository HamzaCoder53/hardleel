"""speed_governor.Governor: converges to cmd/measured on a slow surface, holds while turning, stays clamped."""
import pytest

gov_mod = pytest.importorskip('robot_slam.speed_governor')


def run(plant_gain, steps=40, cmd=0.2, w=0.0):
    """Robot speed = cmd * scale * plant_gain; one update every 0.5 s."""
    g = gov_mod.Governor()
    t, x = 0.0, 0.0
    for _ in range(steps):
        for _ in range(5):                      # 5 cmd + pose samples per 0.5 s
            t += 0.1
            x += cmd * g.scale * plant_gain * 0.1
            g.add_cmd(t, cmd, w)
            g.add_pose(t, x, 0.0, 0.0)
        g.update(t)
    return g.scale


def test_sand_needs_more_pwm():
    assert run(0.6) == pytest.approx(1 / 0.6, rel=0.08)


def test_flat_floor_stays_near_one():
    assert run(1.0) == pytest.approx(1.0, abs=0.05)


def test_clamped():
    assert run(0.2) <= 2.0 + 1e-9


def test_turning_does_not_adapt():
    assert run(0.6, w=0.8) == pytest.approx(1.0, abs=1e-6)

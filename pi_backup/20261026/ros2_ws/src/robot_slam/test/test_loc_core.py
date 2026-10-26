"""loc_core on the REAL course reference map: correlative search, global search, wall heading, raycast."""
import math
import os

import numpy as np
import pytest

lc = pytest.importorskip('robot_slam.loc_core')
pytest.importorskip('scipy')
pytest.importorskip('PIL')

REF = os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'maps', 'course', 'course_reference.yaml')
ANG = np.radians(np.arange(-180, 180, 0.5))          # ~720 beams like the A1 in Boost mode (thinned)


@pytest.fixture(scope='module')
def world():
    if not os.path.isfile(REF):
        pytest.skip('maps/course not in this checkout')
    g = lc.MapGrid.from_yaml(REF)
    return g, lc.LikelihoodField(g)


def scan_at(g, pose, noise=0.0, seed=0):
    r = lc.raycast(g, pose, ANG, 7.0, step=0.004)
    if noise:
        r = r + np.random.default_rng(seed).normal(0, noise, r.shape)
    return lc.scan_points(r, ANG[0], ANG[1] - ANG[0], 0.15, 7.0)


def test_start_pose_scores_high_and_offset_scores_lower(world):
    g, f = world
    pts = scan_at(g, (0.0, 0.0, 0.0))
    assert len(pts) > 300
    assert f.score((0.0, 0.0, 0.0), pts) > 0.8
    assert f.on_wall_fraction((0.0, 0.0, 0.0), pts) > 0.9
    assert f.score((0.2, 0.1, 0.1), pts) < f.score((0.0, 0.0, 0.0), pts) - 0.2


@pytest.mark.parametrize('true', [(0.0, 0.0, 0.0), (-3.0, -2.07, 0.0), (-4.25, -1.0, math.pi / 2)])
def test_local_search_recovers_offset(world, true):
    g, f = world
    pts = scan_at(g, true, noise=0.01)
    guess = (true[0] + 0.15, true[1] - 0.12, true[2] + math.radians(6))
    p, s, s0 = lc.refine(f, pts, guess)
    assert s > s0
    assert math.hypot(p[0] - true[0], p[1] - true[1]) < 0.04
    assert abs(lc.wrap(p[2] - true[2])) < math.radians(1.5)


def test_global_search_with_square_wall_headings(world):
    g, f = world
    true = (-1.0, -2.07, math.pi)
    pts = scan_at(g, true, noise=0.01, seed=3)
    a, strength, n = lc.dominant_wall_angle(pts)
    assert strength > 0.5 and n > 10
    heads = [lc.heading_from_walls(a, k * math.pi / 2) for k in range(4)]
    free = lc.free_cells(g, 0.10, 0.15)
    cands = lc.global_search(f, pts, free, heads, top_k=6)
    best = lc.pick_candidate(cands, last_pose=(-0.8, -2.0, math.pi), gyro_yaw=math.pi, min_score=0.5)
    assert best is not None
    assert math.hypot(best[0] - true[0], best[1] - true[1]) < 0.06
    assert abs(lc.wrap(best[2] - true[2])) < math.radians(2)


@pytest.mark.parametrize('yaw_deg', [0.0, 3.0, -7.5, 30.0, 90.0, 133.0, -100.0])
def test_wall_heading_mod_90(world, yaw_deg):
    g, _ = world
    yaw = math.radians(yaw_deg)
    pts = scan_at(g, (-0.5, -1.0, yaw), noise=0.005)
    a, strength, _ = lc.dominant_wall_angle(pts)
    est = lc.heading_from_walls(a, yaw + math.radians(20))     # gyro estimate 20 deg off still resolves k
    assert abs(lc.wrap(est - yaw)) < math.radians(1.0)


def test_pick_candidate_tie_break_uses_last_pose():
    cands = [(0.80, (1.0, 0.0, 0.0)), (0.79, (-2.0, 0.0, 0.0))]
    assert lc.pick_candidate(cands, (-1.9, 0.0, 0.0), 0.0, 0.5) == (-2.0, 0.0, 0.0)
    assert lc.pick_candidate(cands, None, None, 0.9) is None


def test_inside_rect():
    r = {'x_min': -4.65, 'x_max': 1.10, 'y_min': -2.65, 'y_max': 0.40}
    assert lc.inside_rect(0.0, 0.0, r) and not lc.inside_rect(1.3, 0.0, r) and lc.inside_rect(1.12, 0, r, 0.05)

"""Tests of robot_slam.waypoints and the draft waypoint files.

    cd ros2_ws/src/robot_slam && python3 -m pytest test/test_waypoints.py -q
"""
import math
import os
import sys

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, '..')))
from robot_slam import waypoints as W  # noqa: E402

CFG = os.path.join(HERE, '..', 'config')
COURSE = os.path.join(CFG, 'waypoints_course_fig1.yaml')     # Figure-1 draft (arena-conversion tests below)
REAL_COURSE = os.path.join(CFG, 'waypoints_course.yaml')     # from the team's course map (25 Oct)
OFFICE = os.path.join(CFG, 'waypoints_office.yaml')
ARENA = os.path.join(HERE, '..', '..', 'rdj2026_sim', 'arena', 'arena_layout.yaml')
CLEARANCE_M = 0.2


@pytest.mark.parametrize('path', [COURSE, OFFICE, REAL_COURSE])
def test_shipped_files_are_valid(path):
    poses, seqs, meta = W.load_waypoints(path)
    assert W.check_waypoints(poses, seqs) == []
    assert all(m.get('conf') in ('low', 'med', 'high') and m.get('note') for m in meta.values())


def test_course_matches_arena_conversion():
    poses, _, _ = W.load_waypoints(COURSE)
    x, y, yaw = poses['bin_plastic']          # arena (3029, 3876, 90)
    assert (x, y) == pytest.approx(((3876 - 300) / 1000, -(3029 - 6215) / 1000), abs=1e-3)
    assert yaw == pytest.approx(0.0)


def test_check_reports_problems():
    poses = {'screen': (0, 0, 0)}
    bad = W.check_waypoints(poses, {'terrain': ['nope']})
    assert any('loading_metal' in b for b in bad)
    assert any('undefined pose nope' in b for b in bad)
    assert any('missing sequence: path1' in b for b in bad)


def test_roundtrip_order_header_bak(tmp_path):
    p = str(tmp_path / 'w.yaml')
    poses, seqs, meta = W.load_waypoints(COURSE)
    W.save_waypoints(p, poses, seqs, meta, header='# my header\n')
    W.save_waypoints(p, poses, seqs, meta)                 # second save: header kept, .bak made
    assert os.path.exists(p + '.bak')
    assert open(p).read().startswith('# my header\n')
    p2, s2, m2 = W.load_waypoints(p)
    assert list(p2) == list(poses) and s2 == seqs and m2 == meta
    for n in poses:
        assert p2[n][:2] == pytest.approx(poses[n][:2], abs=5e-4)
        assert math.degrees(p2[n][2]) == pytest.approx(math.degrees(poses[n][2]), abs=0.06)


def test_rounding_in_file(tmp_path):
    p = str(tmp_path / 'w.yaml')
    W.save_waypoints(p, {'a': (1.23456, -0.00049, math.radians(33.349))}, {}, {'a': {'conf': 'high', 'note': "it's: x"}})
    d = yaml.safe_load(open(p))['poses']['a']
    assert d['x'] == 1.235 and d['yaw_deg'] == 33.3 and d['conf'] == 'high' and d['note'] == "it's: x"
    assert abs(d['y']) < 1e-9


def test_set_pose_update_and_insert(tmp_path):
    p = str(tmp_path / 'w.yaml')
    open(p, 'w').write(open(OFFICE).read())
    before, seqs, _ = W.load_waypoints(p)
    W.set_pose(p, 'screen', 9.0, 8.0, math.pi / 2)
    W.set_pose(p, 'extra', 1, 2, 0)
    after, seqs2, meta = W.load_waypoints(p)
    assert after['screen'] == pytest.approx((9.0, 8.0, math.pi / 2), abs=1e-3)
    assert list(after) == list(before) + ['extra'] and seqs2 == seqs
    assert meta['screen']['conf'] == 'high'
    assert open(p).read().startswith('# Office test template')
    assert after['terrain_1'] == pytest.approx(before['terrain_1'])


# ---- clearance of the Figure 1 draft against the arena walls
def _segments():
    a = yaml.safe_load(open(ARENA))
    L, Wd = a['field']['length'], a['field']['width']
    segs = [tuple(w) for w in a['walls']] + [tuple(b) for b in a['baffles']['boards']]
    segs += [(0, 0, L, 0), (L, 0, L, Wd), (L, Wd, 0, Wd), (0, Wd, 0, 0)]       # outer walls
    cx, cy = a['loading_station']['centre']
    hx, hy = a['loading_station']['size'][0] / 2, a['loading_station']['size'][1] / 2
    r = [(cx - hx, cy - hy), (cx + hx, cy - hy), (cx + hx, cy + hy), (cx - hx, cy + hy)]
    segs += [r[i] + r[(i + 1) % 4] for i in range(4)]
    return segs


def _dist(px, py, s):
    x1, y1, x2, y2 = s
    dx, dy = x2 - x1, y2 - y1
    t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)))
    return math.hypot(px - x1 - t * dx, py - y1 - t * dy)


def test_course_draft_clear_of_walls():
    poses, _, _ = W.load_waypoints(COURSE)
    segs = _segments()
    for n, (mx, my, _) in poses.items():
        X, Y = 6215 - 1000 * my, 300 + 1000 * mx                                # map -> arena mm
        d = min(_dist(X, Y, s) for s in segs) / 1000
        assert d >= CLEARANCE_M, '%s at arena (%.0f, %.0f) is %.3f m from a wall' % (n, X, Y, d)

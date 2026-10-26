"""frontier_core tests on synthetic grids (res 0.05 m, origin (-1, -1))."""
import math

import numpy as np

from robot_slam import frontier_core as fc

RES, ORIGIN = 0.05, (-1.0, -1.0)


def room(doorway=False, size=60):
    """size x size cells: unknown outside, walls on the border, free inside; optional door in the top wall."""
    g = np.full((size + 20, size + 20), -1, dtype=np.int16)
    g[10:10 + size, 10:10 + size] = 100
    g[11:9 + size, 11:9 + size] = 0
    if doorway:
        g[9 + size, 30:42] = 0                     # door cells at the top wall: free next to unknown beyond
    return g


def test_closed_room_has_no_frontier():
    fr = fc.find_frontiers(room(), RES, ORIGIN)
    assert fr == [] and fc.exploration_done(fr)


def test_doorway_gives_one_frontier_there():
    fr = fc.find_frontiers(room(doorway=True), RES, ORIGIN)
    assert len(fr) == 1
    cx, cy = fr[0].centroid
    assert abs(cx - (-1.0 + 36.5 * RES)) < 0.2 and cy > -1.0 + 65 * RES * 0.9
    assert not fc.exploration_done(fr)


def test_small_noise_ignored():
    g = room()
    g[30:32, 30:32] = -1                           # 2x2 unknown speck inside: ring of ~12 cells
    assert fc.find_frontiers(g, RES, ORIGIN, min_size_cells=20) == []
    g2 = room()
    g2[30, 30] = -1                                # single speck: 8 frontier cells, below 9
    assert fc.find_frontiers(g2, RES, ORIGIN, min_size_cells=9) == []


def test_clearance_keeps_goal_off_walls():
    g = room(doorway=True)
    fr = fc.find_frontiers(g, RES, ORIGIN)
    goal = fc.choose_goal(fr, (0.0, 0.0), grid=g, resolution=RES, origin_xy=ORIGIN, clearance_m=0.25)
    assert goal is not None
    clear = fc.clearance_map(g, RES)
    col = int((goal.x - ORIGIN[0]) / RES)
    row = int((goal.y - ORIGIN[1]) / RES)
    assert clear[row, col] >= 0.25 - 1e-6
    assert g[row, col] == 0


def test_yaw_faces_frontier():
    g = room(doorway=True)
    fr = fc.find_frontiers(g, RES, ORIGIN)
    goal = fc.choose_goal(fr, (0.0, 0.0), grid=g, resolution=RES, origin_xy=ORIGIN)
    assert abs(goal.yaw - math.atan2(goal.y, goal.x)) < 1e-6


def test_blacklist_and_visited_respected():
    g = room(doorway=True)
    fr = fc.find_frontiers(g, RES, ORIGIN)
    goal = fc.choose_goal(fr, (0.0, 0.0), grid=g, resolution=RES, origin_xy=ORIGIN)
    assert goal is not None
    assert fc.choose_goal(fr, (0.0, 0.0), blacklist=[(goal.x + 0.1, goal.y)], grid=g,
                          resolution=RES, origin_xy=ORIGIN) is None
    assert fc.choose_goal(fr, (0.0, 0.0), visited_goals=[(goal.x, goal.y)], grid=g,
                          resolution=RES, origin_xy=ORIGIN) is None


def test_min_dist_and_preference():
    g = room(doorway=True)
    g[9 + 60, 12:24] = 0                           # second, farther-from-robot-x door on the same wall
    fr = fc.find_frontiers(g, RES, ORIGIN)
    assert len(fr) == 2
    near_second = (-0.075, 1.5)
    goal = fc.choose_goal(fr, near_second, grid=g, resolution=RES, origin_xy=ORIGIN, min_dist=0.3)
    assert goal is not None and goal.x < 0                  # nearer, similar-size door wins
    # robot standing on the goal: too close -> the other one is chosen
    goal2 = fc.choose_goal(fr, (goal.x, goal.y), grid=g, resolution=RES, origin_xy=ORIGIN, min_dist=0.5)
    assert goal2 is not None and math.hypot(goal2.x - goal.x, goal2.y - goal.y) > 0.5


def test_unreachable_clearance_skips_cluster():
    g = room(doorway=True)
    fr = fc.find_frontiers(g, RES, ORIGIN)
    assert fc.choose_goal(fr, (0.0, 0.0), grid=g, resolution=RES, origin_xy=ORIGIN, clearance_m=5.0) is None

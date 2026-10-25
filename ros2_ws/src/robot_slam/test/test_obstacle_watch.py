"""obstacle_watch: walls in the map are expected, a new object ahead is not."""
import math

import numpy as np

from robot_slam.obstacle_watch import unexpected_points

RES = 0.05


def grid_with_wall_at_x(x_wall):
    data = np.zeros((80, 80), dtype=np.int16)          # 4 x 4 m free, origin (-2, -2)
    data[:, int((x_wall + 2) / RES)] = 100
    return {'data': data, 'resolution': RES, 'origin_x': -2.0, 'origin_y': -2.0}


def scan_ahead(dist, n=360, width_deg=10.0):
    inc = 2 * math.pi / n
    ranges = [float('inf')] * n
    for i in range(n):
        a = -math.pi + i * inc
        if abs(math.degrees(a)) <= width_deg:
            ranges[i] = dist / math.cos(a)                 # flat surface `dist` ahead
    return ranges, -math.pi, inc


def test_wall_in_map_is_not_an_obstacle():
    r, a0, inc = scan_ahead(0.6)
    assert unexpected_points(r, a0, inc, (0.0, 0.0, 0.0), grid_with_wall_at_x(0.6)) == 0


def test_object_not_in_map_is_counted():
    r, a0, inc = scan_ahead(0.5)
    assert unexpected_points(r, a0, inc, (0.0, 0.0, 0.0), grid_with_wall_at_x(1.5)) >= 4


def test_far_and_behind_points_are_ignored():
    r, a0, inc = scan_ahead(1.2)                          # beyond max_range 0.8
    assert unexpected_points(r, a0, inc, (0.0, 0.0, 0.0), grid_with_wall_at_x(1.9)) == 0
    r, a0, inc = scan_ahead(0.5)
    assert unexpected_points(r, a0, inc, (0.0, 0.0, math.pi), grid_with_wall_at_x(1.5)) >= 4  # robot turned:
    # the object is now behind in the map frame (x = -0.5), still unexpected, still counted (it is ahead of the laser)


def test_unknown_cells_count_as_expected():
    g = grid_with_wall_at_x(1.5)
    g['data'][:, :] = -1
    r, a0, inc = scan_ahead(0.5)
    assert unexpected_points(r, a0, inc, (0.0, 0.0, 0.0), g) == 0

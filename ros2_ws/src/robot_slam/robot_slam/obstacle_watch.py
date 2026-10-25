"""Pure helpers for "is something in front of me that is not in the map?" (no ROS).

The mission logs the rulebook's obstacle detection/response timestamps from this: a lidar point ahead of the
robot that has no occupied map cell near it is an unexpected obstacle (a "worker" or a judge's cylinder),
whereas the walls the robot drives past every second are in the map and do not count.
"""
import math

import numpy as np


def unexpected_points(ranges, angle_min, angle_inc, laser_pose, grid, max_range=0.8, half_angle=math.radians(35),
                      margin_cells=3, occ_thresh=50):
    """Count beams within +-half_angle ahead and closer than max_range that land on map cells with no
    occupied cell within margin_cells. Unknown (-1) cells count as expected (fresh maps have holes).

    laser_pose: (x, y, yaw) of the laser in the map frame.
    grid: dict(data=2D int array [row=y, col=x], resolution, origin_x, origin_y).
    """
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_inc * np.arange(len(r))
    a = (a + math.pi) % (2 * math.pi) - math.pi
    ok = np.isfinite(r) & (r > 0.12) & (r < max_range) & (np.abs(a) <= half_angle)
    if not ok.any():
        return 0
    lx, ly, lyaw = laser_pose
    px = lx + r[ok] * np.cos(a[ok] + lyaw)
    py = ly + r[ok] * np.sin(a[ok] + lyaw)
    data, res = grid['data'], grid['resolution']
    cx = np.floor((px - grid['origin_x']) / res).astype(int)
    cy = np.floor((py - grid['origin_y']) / res).astype(int)
    h, w = data.shape
    count = 0
    for i, j in zip(cy, cx):
        if not (0 <= i < h and 0 <= j < w):
            continue                                   # outside the map: ignore
        win = data[max(i - margin_cells, 0):i + margin_cells + 1, max(j - margin_cells, 0):j + margin_cells + 1]
        if (win >= occ_thresh).any() or data[i, j] < 0:
            continue
        count += 1
    return count

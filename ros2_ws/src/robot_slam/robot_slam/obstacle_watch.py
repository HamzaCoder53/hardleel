"""Pure helpers for "is something on my way that is not in the map?" (no ROS).

The mission logs the rulebook's obstacle detection/response timestamps from this: a lidar point that has no
occupied map cell near it is an unexpected obstacle (a "worker" or a judge's cylinder), whereas the walls the
robot drives past every second are in the map and do not count. Two detectors:
  unexpected_points     points in a short cone straight ahead (close obstacles, any heading)
  unexpected_near_path  points near the first few metres of Nav2's current plan. Added 25 Oct: a worker that
                        blocks a corridor makes Nav2 stop planning metres away, so the robot never got within the
                        0.8 m cone and the worker was never logged (course sim).
"""
import math

import numpy as np


def _beams(ranges, angle_min, angle_inc, max_range, half_angle=math.pi):
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_inc * np.arange(len(r))
    a = (a + math.pi) % (2 * math.pi) - math.pi
    ok = np.isfinite(r) & (r > 0.12) & (r < max_range) & (np.abs(a) <= half_angle)
    return r[ok], a[ok]


def _unexpected_xy(r, a, laser_pose, grid, margin_cells=3, occ_thresh=50):
    """Map-frame (x, y) of the beam end points that land on known-free cells with no occupied cell within
    margin_cells. Unknown (-1) cells count as expected (fresh maps have holes); points off the map are ignored."""
    if len(r) == 0:
        return np.zeros((0, 2))
    lx, ly, lyaw = laser_pose
    px = lx + r * np.cos(a + lyaw)
    py = ly + r * np.sin(a + lyaw)
    data, res = grid['data'], grid['resolution']
    cx = np.floor((px - grid['origin_x']) / res).astype(int)
    cy = np.floor((py - grid['origin_y']) / res).astype(int)
    h, w = data.shape
    keep = []
    for k, (i, j) in enumerate(zip(cy, cx)):
        if not (0 <= i < h and 0 <= j < w):
            continue
        win = data[max(i - margin_cells, 0):i + margin_cells + 1, max(j - margin_cells, 0):j + margin_cells + 1]
        if (win >= occ_thresh).any() or data[i, j] < 0:
            continue
        keep.append(k)
    return np.stack([px[keep], py[keep]], axis=1) if keep else np.zeros((0, 2))


def unexpected_points(ranges, angle_min, angle_inc, laser_pose, grid, max_range=0.8, half_angle=math.radians(35),
                      margin_cells=3, occ_thresh=50):
    """Count beams within +-half_angle ahead and closer than max_range that are not explained by the map.

    laser_pose: (x, y, yaw) of the laser in the map frame.
    grid: dict(data=2D int array [row=y, col=x], resolution, origin_x, origin_y).
    """
    r, a = _beams(ranges, angle_min, angle_inc, max_range, half_angle)
    return len(_unexpected_xy(r, a, laser_pose, grid, margin_cells, occ_thresh))


def path_prefix(path_xy, start_xy, length_m):
    """The part of a planned path (list of (x, y)) from the point nearest start_xy onwards, length_m long."""
    if not path_xy:
        return []
    p = np.asarray(path_xy, dtype=float)
    k = int(np.argmin(np.hypot(p[:, 0] - start_xy[0], p[:, 1] - start_xy[1])))
    out, run = [tuple(p[k])], 0.0
    for a, b in zip(p[k:], p[k + 1:]):
        run += math.hypot(b[0] - a[0], b[1] - a[1])
        out.append(tuple(b))
        if run >= length_m:
            break
    return out


def unexpected_near_path(ranges, angle_min, angle_inc, laser_pose, grid, path_xy, path_len_m=2.5, tol_m=0.30,
                         max_range=4.0, margin_cells=3, occ_thresh=50):
    """Count unexpected points (see _unexpected_xy) lying within tol_m of the first path_len_m of the planned
    path (map frame), measured from the path point nearest the laser."""
    seg = path_prefix(path_xy, laser_pose[:2], path_len_m)
    if len(seg) < 1:
        return 0
    r, a = _beams(ranges, angle_min, angle_inc, max_range)
    pts = _unexpected_xy(r, a, laser_pose, grid, margin_cells, occ_thresh)
    if len(pts) == 0:
        return 0
    s = np.asarray(seg, dtype=float)
    if len(s) == 1:
        d = np.hypot(pts[:, 0] - s[0, 0], pts[:, 1] - s[0, 1])
        return int((d <= tol_m).sum())
    a0, b0 = s[:-1], s[1:]                            # distance from each point to each path segment
    ab = b0 - a0
    ab2 = np.maximum((ab ** 2).sum(axis=1), 1e-12)
    ap = pts[:, None, :] - a0[None, :, :]
    t = np.clip((ap * ab[None, :, :]).sum(axis=2) / ab2[None, :], 0.0, 1.0)
    proj = a0[None, :, :] + t[:, :, None] * ab[None, :, :]
    d = np.hypot(*(pts[:, None, :] - proj).transpose(2, 0, 1)).min(axis=1)
    return int((d <= tol_m).sum())

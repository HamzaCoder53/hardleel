"""Frontier exploration core for the mapping phase (pure numpy, no ROS; unit tested).

Grid convention = nav_msgs/OccupancyGrid reshaped to (height, width): grid[row, col], row = y, col = x,
values -1 unknown, 0..100 occupancy. World xy of cell centre = origin + (col + 0.5, row + 0.5) * resolution.

find_frontiers(grid, resolution, origin_xy, min_size_cells)   free cell (0 <= v < free_max) next to an unknown
    (-1) cell (8-neighbourhood), clustered with 8-connectivity; clusters under min_size_cells are noise.
choose_goal(...)   best cluster = size_weight*size - dist_weight*distance, with a goal cell that is free and
    at least clearance_m from any occupied (>= occ_min) cell; skips blacklisted / visited / too-close goals.
frontier_near(...) True if a frontier cell lies within radius of a point (is the current goal still a frontier).
exploration_done(frontiers, min_size)   no usable frontier left.

Params (all keyword defaults): min_size_cells 8, min_dist 0.5 m, clearance_m 0.25, blacklist_radius 0.4 m,
visited_radius 0.5 m, size_weight 1.0 per cell (size capped at size_cap), dist_weight 8.0 per m.
"""
import math
from collections import deque
from dataclasses import dataclass, field

import numpy as np

try:
    from scipy.ndimage import distance_transform_edt
except Exception:                                   # scipy missing: brute-force fallback
    distance_transform_edt = None

NEIGH8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


@dataclass
class Frontier:
    centroid: tuple                                 # world (x, y)
    size: int                                       # number of cells
    cells: list = field(default_factory=list)       # [(row, col), ...]


@dataclass
class Goal:
    x: float
    y: float
    yaw: float
    frontier: Frontier = None
    score: float = 0.0


def cell_to_world(row, col, resolution, origin_xy):
    return origin_xy[0] + (col + 0.5) * resolution, origin_xy[1] + (row + 0.5) * resolution


def frontier_mask(grid, free_max=25):
    g = np.asarray(grid)
    unknown = g < 0
    free = (g >= 0) & (g < free_max)
    pad = np.pad(unknown, 1, constant_values=False)
    h, w = g.shape
    near_unknown = np.zeros_like(unknown)
    for dr, dc in NEIGH8:
        near_unknown |= pad[1 + dr:1 + dr + h, 1 + dc:1 + dc + w]
    return free & near_unknown


def find_frontiers(grid, resolution, origin_xy, min_size_cells=8, free_max=25):
    mask = frontier_mask(grid, free_max)
    h, w = mask.shape
    seen = np.zeros_like(mask)
    out = []
    for r0, c0 in zip(*np.nonzero(mask)):
        if seen[r0, c0]:
            continue
        seen[r0, c0] = True
        cells, q = [], deque([(int(r0), int(c0))])
        while q:
            r, c = q.popleft()
            cells.append((r, c))
            for dr, dc in NEIGH8:
                rr, cc = r + dr, c + dc
                if 0 <= rr < h and 0 <= cc < w and mask[rr, cc] and not seen[rr, cc]:
                    seen[rr, cc] = True
                    q.append((rr, cc))
        if len(cells) < min_size_cells:
            continue
        xy = [cell_to_world(r, c, resolution, origin_xy) for r, c in cells]
        out.append(Frontier((float(np.mean([p[0] for p in xy])), float(np.mean([p[1] for p in xy]))),
                            len(cells), cells))
    return out


def clearance_map(grid, resolution, occ_min=65):
    """Distance (m) from every cell to the nearest occupied cell (inf when there is none)."""
    occ = np.asarray(grid) >= occ_min
    if not occ.any():
        return np.full(occ.shape, np.inf)
    if distance_transform_edt is not None:
        return distance_transform_edt(~occ) * resolution
    out = np.full(occ.shape, np.inf)                # fallback: brute force over the occupied cells
    rr, cc = np.indices(occ.shape)
    for r, c in zip(*np.nonzero(occ)):
        out = np.minimum(out, np.hypot(rr - r, cc - c) * resolution)
    return out


def goal_cell(frontier, clear, resolution, origin_xy, clearance_m=0.25):
    """Cluster cell nearest the centroid with enough clearance; None if no cell qualifies."""
    best, best_d = None, None
    for r, c in frontier.cells:
        if clear is not None and clear[r, c] < clearance_m:
            continue
        x, y = cell_to_world(r, c, resolution, origin_xy)
        d = math.hypot(x - frontier.centroid[0], y - frontier.centroid[1])
        if best_d is None or d < best_d:
            best, best_d = (x, y), d
    return best


def near_any(x, y, points, radius):
    return any(math.hypot(x - p[0], y - p[1]) < radius for p in points)


def frontier_near(frontiers, x, y, resolution, origin_xy, radius=0.6):
    for f in frontiers:
        for r, c in f.cells:
            cx, cy = cell_to_world(r, c, resolution, origin_xy)
            if math.hypot(cx - x, cy - y) < radius:
                return True
    return False


def choose_goal(frontiers, robot_xy, visited_goals=(), blacklist=(), min_dist=0.5, grid=None, resolution=0.05,
                origin_xy=(0.0, 0.0), clearance_m=0.25, blacklist_radius=0.4, visited_radius=0.5,
                size_weight=1.0, dist_weight=8.0, size_cap=60):
    """Best Goal or None. grid=None skips the clearance check."""
    clear = clearance_map(grid, resolution) if grid is not None else None
    best = None
    for f in frontiers:
        g = goal_cell(f, clear, resolution, origin_xy, clearance_m)
        if g is None:
            continue
        if near_any(g[0], g[1], blacklist, blacklist_radius) or \
                near_any(f.centroid[0], f.centroid[1], blacklist, blacklist_radius):
            continue
        if near_any(g[0], g[1], visited_goals, visited_radius):
            continue
        dist = math.hypot(g[0] - robot_xy[0], g[1] - robot_xy[1])
        if dist < min_dist:
            continue
        score = size_weight * min(f.size, size_cap) - dist_weight * dist
        if best is None or score > best.score:
            best = Goal(g[0], g[1], math.atan2(g[1] - robot_xy[1], g[0] - robot_xy[0]), f, score)
    return best


def exploration_done(frontiers, min_size=8):
    return not any(f.size >= min_size for f in frontiers)

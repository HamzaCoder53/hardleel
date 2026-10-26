"""Localization helpers (no ROS): likelihood field on a reference map, scan scoring, correlative pose search
(local + global), and the square-wall heading measurement.

All poses are (x, y, yaw) in the map frame; scan points are Nx2 in the robot (laser) frame. Maps are 2D arrays
indexed [row, col] with row 0 at the TOP (map_server .pgm convention); `MapGrid` hides that.
"""
import math

import numpy as np

try:
    from scipy.ndimage import distance_transform_edt
except ImportError:                                  # the Pi has no scipy; only the laptop builds fields
    distance_transform_edt = None


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class MapGrid:
    """Occupancy grid in map_server convention -> occupied mask + metric helpers."""

    def __init__(self, occupied, resolution, origin_xy):
        self.occ = np.asarray(occupied, dtype=bool)          # [row, col], row 0 = top (max y)
        self.res = float(resolution)
        self.ox, self.oy = float(origin_xy[0]), float(origin_xy[1])
        self.h, self.w = self.occ.shape

    @classmethod
    def from_yaml(cls, path):
        import os
        import yaml
        from PIL import Image
        m = yaml.safe_load(open(path))
        img = np.array(Image.open(os.path.join(os.path.dirname(path), m['image'])).convert('L'), dtype=float)
        if m.get('negate', 0):
            img = 255.0 - img
        occ = img < (1.0 - float(m.get('occupied_thresh', 0.65))) * 255.0
        return cls(occ, m['resolution'], m['origin'][:2])

    @classmethod
    def from_occupancy(cls, data, width, height, resolution, origin_xy, occupied=65):
        """nav_msgs/OccupancyGrid layout (row 0 = BOTTOM) -> MapGrid."""
        a = np.asarray(data, dtype=np.int16).reshape(height, width)[::-1]
        return cls(a >= occupied, resolution, origin_xy)

    def to_cell(self, x, y):
        """metric -> (row, col) float arrays (row 0 = top)."""
        col = (np.asarray(x) - self.ox) / self.res
        row = self.h - (np.asarray(y) - self.oy) / self.res
        return row, col


class LikelihoodField:
    """exp(-d^2 / 2 sigma^2) of the distance d to the nearest occupied cell; points off the map score 0."""

    def __init__(self, grid, sigma=0.05, max_dist=0.5):
        if distance_transform_edt is None:
            raise RuntimeError('scipy is needed to build a likelihood field')
        self.g = grid
        d = distance_transform_edt(~grid.occ) * grid.res
        self.dist = np.minimum(d, max_dist)
        self.lik = np.exp(-0.5 * (self.dist / sigma) ** 2)

    def _lookup(self, table, x, y, outside):
        row, col = self.g.to_cell(x, y)
        r, c = np.floor(row).astype(int), np.floor(col).astype(int)
        ok = (r >= 0) & (r < self.g.h) & (c >= 0) & (c < self.g.w)
        out = np.full(r.shape, outside, dtype=float)
        out[ok] = table[r[ok], c[ok]]
        return out

    def score(self, pose, pts):
        """Mean likelihood of the scan points at `pose` (0..1)."""
        if len(pts) == 0:
            return 0.0
        x, y = transform(pose, pts)
        return float(self._lookup(self.lik, x, y, 0.0).mean())

    def on_wall_fraction(self, pose, pts, tol=0.05):
        """Fraction of points within `tol` of a map wall (the confidence metric)."""
        if len(pts) == 0:
            return 0.0
        x, y = transform(pose, pts)
        return float((self._lookup(self.dist, x, y, 1e9) <= tol).mean())


def scan_points(ranges, angle_min, angle_inc, rmin=0.15, rmax=7.0, step=1):
    r = np.asarray(ranges, dtype=float)[::step]
    a = angle_min + angle_inc * np.arange(0, len(ranges), step)
    ok = np.isfinite(r) & (r >= rmin) & (r <= rmax)
    return np.column_stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])])


def transform(pose, pts):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return pose[0] + c * pts[:, 0] - s * pts[:, 1], pose[1] + s * pts[:, 0] + c * pts[:, 1]


def correlative_search(field, pts, init, lin_win=0.3, ang_win=math.radians(15), lin_step=0.025,
                       ang_step=math.radians(1.0), max_pts=180):
    """Brute-force search around `init` -> (best pose, best score, init score). Points are subsampled to
    max_pts; translations are evaluated for every rotation at once (vectorised)."""
    if len(pts) == 0:
        return tuple(init), 0.0, 0.0
    if len(pts) > max_pts:
        pts = pts[np.linspace(0, len(pts) - 1, max_pts).astype(int)]
    offs = np.arange(-lin_win, lin_win + 1e-9, lin_step)
    dx, dy = np.meshgrid(offs, offs, indexing='ij')
    dx, dy = dx.ravel(), dy.ravel()
    best = (tuple(init), -1.0)
    for dth in np.arange(-ang_win, ang_win + 1e-9, ang_step):
        th = init[2] + dth
        c, s = math.cos(th), math.sin(th)
        px = init[0] + c * pts[:, 0] - s * pts[:, 1]
        py = init[1] + s * pts[:, 0] + c * pts[:, 1]
        X = px[None, :] + dx[:, None]
        Y = py[None, :] + dy[:, None]
        sc = field._lookup(field.lik, X, Y, 0.0).mean(axis=1)
        k = int(np.argmax(sc))
        if sc[k] > best[1]:
            best = ((init[0] + dx[k], init[1] + dy[k], wrap(th)), float(sc[k]))
    return best[0], best[1], field.score(init, pts)


def refine(field, pts, pose):
    """Coarse-to-fine local search: 0.3 m / 15 deg at 5 cm / 2 deg, then 6 cm / 3 deg at 1 cm / 0.5 deg."""
    p, s, s0 = correlative_search(field, pts, pose, 0.3, math.radians(15), 0.05, math.radians(2.0))
    p, s, _ = correlative_search(field, pts, p, 0.06, math.radians(3), 0.01, math.radians(0.5))
    return p, s, s0


def global_search(field, pts, free_xy, headings, top_k=5, max_pts=90):
    """Score every (free cell, heading) pair; -> top_k [(score, pose)] after a local refine of each.
    free_xy: Mx2 candidate positions; headings: candidate yaws (e.g. the 4 square-wall solutions)."""
    if len(pts) > max_pts:
        pts = pts[np.linspace(0, len(pts) - 1, max_pts).astype(int)]
    cands = []
    for th in headings:
        c, s = math.cos(th), math.sin(th)
        rx = c * pts[:, 0] - s * pts[:, 1]
        ry = s * pts[:, 0] + c * pts[:, 1]
        X = free_xy[:, 0:1] + rx[None, :]
        Y = free_xy[:, 1:2] + ry[None, :]
        sc = field._lookup(field.lik, X, Y, 0.0).mean(axis=1)
        for k in np.argsort(sc)[-top_k:]:
            cands.append((float(sc[k]), (float(free_xy[k, 0]), float(free_xy[k, 1]), th)))
    cands.sort(key=lambda c: -c[0])
    out = []
    for _, pose in cands[:top_k]:
        p, s, _ = refine(field, pts, pose)
        out.append((s, p))
    out.sort(key=lambda c: -c[0])
    return out


def pick_candidate(cands, last_pose, gyro_yaw, min_score, tie_margin=0.03, w_dist=0.05, w_yaw=0.1):
    """Best candidate; candidates within tie_margin of the best score are ranked by closeness to the last known
    pose and to the gyro heading. -> pose or None if nothing scores min_score."""
    if not cands or cands[0][0] < min_score:
        return None
    top = cands[0][0]
    near = [c for c in cands if c[0] >= top - tie_margin]

    def cost(c):
        p = c[1]
        d = math.hypot(p[0] - last_pose[0], p[1] - last_pose[1]) if last_pose else 0.0
        dy = abs(wrap(p[2] - gyro_yaw)) if gyro_yaw is not None else 0.0
        return -c[0] + w_dist * d + w_yaw * dy
    return min(near, key=cost)[1]


def free_cells(grid, stride_m=0.10, clearance_m=0.15):
    """Candidate robot positions: cells at least clearance_m from any wall, every stride_m."""
    d = distance_transform_edt(~grid.occ) * grid.res
    step = max(1, int(round(stride_m / grid.res)))
    rows, cols = np.mgrid[0:grid.h:step, 0:grid.w:step]
    rows, cols = rows.ravel(), cols.ravel()
    ok = d[rows, cols] >= clearance_m
    x = grid.ox + (cols[ok] + 0.5) * grid.res
    y = grid.oy + (grid.h - rows[ok] - 0.5) * grid.res
    return np.column_stack([x, y])


def raycast(grid, pose, angles, max_range=7.0, step=None):
    """Expected range per beam (robot-frame angles) from `pose` on the map; inf when nothing is hit."""
    step = step or grid.res * 0.5
    t = np.arange(step, max_range + 1e-9, step)
    a = pose[2] + np.asarray(angles, dtype=float)
    X = pose[0] + np.cos(a)[:, None] * t[None, :]
    Y = pose[1] + np.sin(a)[:, None] * t[None, :]
    row, col = grid.to_cell(X, Y)
    r, c = np.floor(row).astype(int), np.floor(col).astype(int)
    inside = (r >= 0) & (r < grid.h) & (c >= 0) & (c < grid.w)
    hit = np.zeros(r.shape, dtype=bool)
    hit[inside] = grid.occ[r[inside], c[inside]]
    first = np.where(hit.any(axis=1), hit.argmax(axis=1), -1)
    return np.where(first >= 0, t[np.maximum(first, 0)], np.inf)


# --------------------------------------------------------------------------- square-wall heading
def dominant_wall_angle(pts, window=9, max_gap=0.10, min_len=0.15, max_rms=0.012):
    """Dominant wall direction in the ROBOT frame, modulo 90 deg -> (angle in [-45, 45] deg as radians,
    strength 0..1, number of straight segments). The scan is cut into windows of `window` consecutive points
    (half overlapping); a window counts when its points are contiguous (gaps < max_gap), at least min_len long and
    straight (line-fit rms < max_rms). Point-to-point directions are useless here (range noise / grid staircase).
    Each segment is weighted by its length; the 4-fold circular mean handles the 90 deg ambiguity."""
    n = len(pts)
    if n < window:
        return None, 0.0, 0
    C = S = W = 0.0
    nseg = 0
    for i in range(0, n - window + 1, max(1, window // 2)):
        q = pts[i:i + window]
        gaps = np.hypot(*np.diff(q, axis=0).T)
        if gaps.max() > max_gap:
            continue
        c = q - q.mean(axis=0)
        u, sv, vt = np.linalg.svd(c, full_matrices=False)
        length = float(np.hypot(*(q[-1] - q[0])))
        rms = sv[1] / math.sqrt(window)
        if length < min_len or rms > max_rms:
            continue
        th = math.atan2(vt[0, 1], vt[0, 0])
        C += length * math.cos(4 * th)
        S += length * math.sin(4 * th)
        W += length
        nseg += 1
    if nseg < 2 or W <= 0:
        return None, 0.0, nseg
    return math.atan2(S, C) / 4.0, math.hypot(C, S) / W, nseg


def heading_from_walls(wall_angle, yaw_ref):
    """Robot yaw that puts the dominant wall at 0/90 deg in the map: yaw = -a + k*90 deg, k chosen nearest yaw_ref
    (the gyro / current estimate resolves the multiple)."""
    base = -wall_angle
    k = round((yaw_ref - base) / (math.pi / 2))
    return wrap(base + k * math.pi / 2)


def inside_rect(x, y, rect, margin=0.0):
    return rect['x_min'] - margin <= x <= rect['x_max'] + margin and rect['y_min'] - margin <= y <= rect['y_max'] + margin

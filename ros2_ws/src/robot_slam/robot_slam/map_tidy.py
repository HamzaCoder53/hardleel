#!/usr/bin/env python3
"""map_tidy (final stack item 7): tidy a saved map (map_server .yaml/.pgm); the raw map is never changed.

    ros2 run robot_slam map_tidy <map.yaml> [--reference maps/course/course_reference.yaml] [--out <name>_tidy]
1. squareness: the dominant wall angle (all course walls are 0/90 deg); |angle| <= 2 deg -> the grid is rotated by
   it (nearest-cell resampling, no blur), bigger -> left alone (logged).
2. outer rectangle: bounding box of the occupied cells after removing small specks; size must match the course
   (5.75 x 3.05 m +/- size_tol) - otherwise the remaining steps are SKIPPED (something is wrong with the map).
3. outside the rectangle -> unknown.
4. gaps < 10 cm closed ONLY between wall ends the lidar saw: in each row/column, a run of free/unknown cells of at
   most max_gap cells with occupied cells on BOTH sides becomes occupied. Nothing else is ever added.
5. alignment against course_reference: correlative search (loc_core) of the map's wall cells -> transform + score.
Writes <out>.pgm/.yaml + <out>_report.json. Exit 2 when the size check fails.
"""
import argparse
import json
import math
import os
import sys

import numpy as np
import yaml
from PIL import Image

from robot_slam import loc_core as lc

COURSE = (5.75, 3.05)


def load(path):
    m = yaml.safe_load(open(path))
    img = np.array(Image.open(os.path.join(os.path.dirname(path), m['image'])).convert('L'))
    occ_t = (1.0 - float(m.get('occupied_thresh', 0.65))) * 255
    free_t = (1.0 - float(m.get('free_thresh', 0.196))) * 255
    grid = np.full(img.shape, -1, dtype=np.int8)               # -1 unknown, 0 free, 100 occupied
    grid[img <= occ_t] = 100
    grid[img >= free_t] = 0
    return m, grid


def save(grid, meta, out):
    img = np.full(grid.shape, 205, dtype=np.uint8)
    img[grid == 0] = 254
    img[grid == 100] = 0
    Image.fromarray(img).save(out + '.pgm')
    m = dict(meta)
    m['image'] = os.path.basename(out) + '.pgm'
    yaml.safe_dump(m, open(out + '.yaml', 'w'), sort_keys=False)


def wall_points(grid, res, origin):
    rows, cols = np.nonzero(grid == 100)
    h = grid.shape[0]
    return np.column_stack([origin[0] + (cols + 0.5) * res, origin[1] + (h - rows - 0.5) * res])


def dominant_angle(grid):
    """Dominant wall direction mod 90 deg from horizontal / vertical runs' structure tensor of the occupied mask."""
    occ = (grid == 100).astype(float)
    gy, gx = np.gradient(occ)
    w = np.hypot(gx, gy)
    if w.sum() < 1:
        return 0.0, 0.0
    th = np.arctan2(gy, gx)
    C, S = (w * np.cos(4 * th)).sum(), (w * np.sin(4 * th)).sum()
    return math.atan2(S, C) / 4.0, math.hypot(C, S) / w.sum()


def rotate_grid(grid, ang):
    """Rotate by ang (rad, CCW) about the centre, nearest cell (values stay -1/0/100)."""
    h, w = grid.shape
    cy, cx = (h - 1) / 2, (w - 1) / 2
    rr, cc = np.mgrid[0:h, 0:w]
    c, s = math.cos(ang), math.sin(ang)
    # inverse map: output (r, c) <- input; rows grow downward, so a CCW map rotation is CW in (row, col)
    x, y = cc - cx, cy - rr
    xs, ys = c * x + s * y, -s * x + c * y
    ci, ri = np.rint(xs + cx).astype(int), np.rint(cy - ys).astype(int)
    ok = (ri >= 0) & (ri < h) & (ci >= 0) & (ci < w)
    out = np.full_like(grid, -1)
    out[ok] = grid[ri[ok], ci[ok]]
    return out


def outer_rect(grid, min_cells=4):
    """Bounding box (r0, r1, c0, c1) of occupied cells in rows/cols with at least min_cells of them (specks out)."""
    occ = grid == 100
    rows = np.nonzero(occ.sum(axis=1) >= min_cells)[0]
    cols = np.nonzero(occ.sum(axis=0) >= min_cells)[0]
    if len(rows) < 2 or len(cols) < 2:
        return None
    return rows[0], rows[-1], cols[0], cols[-1]


def close_gaps(grid, max_gap):
    """Fill runs of <= max_gap non-occupied cells that have occupied cells on both ends (rows, then columns)."""
    out = grid.copy()
    occ = grid == 100
    filled = 0
    for axis in (0, 1):
        a = occ if axis == 0 else occ.T
        o = out if axis == 0 else out.T
        for i in range(a.shape[0]):
            idx = np.nonzero(a[i])[0]
            for p, q in zip(idx, idx[1:]):
                if 1 < q - p <= max_gap + 1:
                    o[i, p + 1:q] = 100
                    filled += q - p - 1
    return out, filled


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('map')
    ap.add_argument('--reference', default=os.path.expanduser('~/hardleel/maps/course/course_reference.yaml'))
    ap.add_argument('--out', default='')
    ap.add_argument('--size-tol', type=float, default=0.15)
    ap.add_argument('--max-gap-m', type=float, default=0.10)
    ap.add_argument('--max-square-deg', type=float, default=2.0)
    a = ap.parse_args(argv)
    meta, grid = load(a.map)
    res = float(meta['resolution'])
    origin = meta['origin'][:2]
    out = a.out or os.path.splitext(a.map)[0] + '_tidy'
    rep = {'map': a.map, 'out': out}
    ang, strength = dominant_angle(grid)
    rep['wall_angle_deg'], rep['wall_strength'] = round(math.degrees(ang), 2), round(strength, 3)
    if abs(ang) <= math.radians(a.max_square_deg) and abs(ang) > math.radians(0.2):
        grid = rotate_grid(grid, -ang)
        rep['squared_by_deg'] = round(-math.degrees(ang), 2)
    box = outer_rect(grid)
    if box is None:
        rep['skipped'] = 'no walls found'
        print(json.dumps(rep, indent=1, default=float))
        return 2
    r0, r1, c0, c1 = box
    size = ((c1 - c0 + 1) * res, (r1 - r0 + 1) * res)
    rep['rect_m'] = [round(size[0], 2), round(size[1], 2)]
    ok = any(abs(size[0] - A) <= a.size_tol + 0.1 and abs(size[1] - B) <= a.size_tol + 0.1
             for A, B in (COURSE, COURSE[::-1]))                 # +0.1: outer faces include the wall thickness
    if not ok:
        rep['skipped'] = f'rectangle {size[0]:.2f} x {size[1]:.2f} m is not the course {COURSE} +/- {a.size_tol}'
        save(grid, meta, out)
        json.dump(rep, open(out + '_report.json', 'w'), indent=1, default=float)
        print(json.dumps(rep, indent=1, default=float))
        return 2
    outside = np.ones(grid.shape, dtype=bool)
    outside[r0:r1 + 1, c0:c1 + 1] = False
    rep['cleared_outside'] = int(((grid != -1) & outside).sum())
    grid[outside] = -1
    grid, rep['gap_cells_filled'] = close_gaps(grid, int(round(a.max_gap_m / res)))
    save(grid, meta, out)
    if os.path.isfile(a.reference):
        ref = lc.MapGrid.from_yaml(a.reference)
        field = lc.LikelihoodField(ref)
        pts = wall_points(grid, res, origin)
        if len(pts) > 400:
            pts = pts[np.linspace(0, len(pts) - 1, 400).astype(int)]
        p, s, s0 = lc.correlative_search(field, pts, (0.0, 0.0, 0.0), 0.4, math.radians(10), 0.05, math.radians(1))
        p, s, _ = lc.correlative_search(field, pts, p, 0.06, math.radians(2), 0.01, math.radians(0.25))
        rep['reference_match'] = {'score': round(s, 3), 'score_unaligned': round(s0, 3),
                                  'offset_m': [round(p[0], 3), round(p[1], 3)], 'rot_deg': round(math.degrees(p[2]), 2)}
    json.dump(rep, open(out + '_report.json', 'w'), indent=1, default=float)
    print(json.dumps(rep, indent=1, default=float))
    return 0


if __name__ == '__main__':
    sys.exit(main())

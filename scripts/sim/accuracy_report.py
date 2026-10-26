#!/usr/bin/env python3
"""Virtual SLAM step 2: accuracy of a saved map against the ground-truth wall map.

    accuracy_report.py MAP.yaml [--truth maps/course/course_full_approx.yaml] [--start 0.06,0] [--out PREFIX]

Reports (JSON next to the overlay PNG):
  * wall-distance error: every occupied cell of the saved map -> distance to the nearest true wall
    (mean, median, 95th percentile). `raw` uses only the known start offset (map frame = start pose, as on the
    robot); `aligned` after the best rigid fit (shows how much of the error is a global shift/rotation).
  * coverage: share of true wall cells that have a mapped wall within 10 cm.
  * size: bounding box of the free space connected to the start (= inner wall faces) vs the truth and vs 5.75 x 3.05 m.
"""
import argparse
import json
import math
import os

import cv2
import numpy as np
import yaml
from scipy.optimize import minimize

INNER = (5.75, 3.05)


def load(yaml_path):
    meta = yaml.safe_load(open(yaml_path))
    img = cv2.imread(os.path.join(os.path.dirname(yaml_path), meta['image']), cv2.IMREAD_GRAYSCALE)
    if meta.get('negate', 0):
        img = 255 - img
    p = (255 - img.astype(float)) / 255.0
    occ = p > meta.get('occupied_thresh', 0.65)
    free = p < meta.get('free_thresh', 0.196)
    return dict(occ=occ, free=free, res=meta['resolution'], ox=meta['origin'][0], oy=meta['origin'][1], h=img.shape[0])


def cells_to_xy(m, mask):
    r, c = np.nonzero(mask)
    return np.column_stack([m['ox'] + (c + 0.5) * m['res'], m['oy'] + (m['h'] - 1 - r + 0.5) * m['res']])


def xy_to_cells(m, xy):
    return (np.floor((xy[:, 0] - m['ox']) / m['res']).astype(int),
            m['h'] - 1 - np.floor((xy[:, 1] - m['oy']) / m['res']).astype(int))


def dist_field(m):
    return cv2.distanceTransform((~m['occ']).astype(np.uint8), cv2.DIST_L2, 5) * m['res']


def dists(pts, truth, field, clip=1.0):
    c, r = xy_to_cells(truth, pts)
    ok = (c >= 0) & (c < field.shape[1]) & (r >= 0) & (r < field.shape[0])
    d = np.full(len(pts), clip)
    d[ok] = np.minimum(field[r[ok], c[ok]], clip)
    return d


def transform(pts, t):
    dx, dy, th = t
    ca, sa = math.cos(th), math.sin(th)
    return np.column_stack([ca * pts[:, 0] - sa * pts[:, 1] + dx, sa * pts[:, 0] + ca * pts[:, 1] + dy])


def free_bbox(m, start_xy):
    n, lab = cv2.connectedComponents(m['free'].astype(np.uint8), connectivity=4)
    c, r = xy_to_cells(m, np.array([start_xy]))
    c, r = int(np.clip(c[0], 0, lab.shape[1] - 1)), int(np.clip(r[0], 0, lab.shape[0] - 1))
    if lab[r, c] == 0:       # start cell not free: take the biggest free component
        counts = np.bincount(lab.ravel())[1:]
        comp = int(np.argmax(counts)) + 1
    else:
        comp = lab[r, c]
    pts = cells_to_xy(m, lab == comp)
    lo, hi = pts.min(0) - m['res'] / 2, pts.max(0) + m['res'] / 2
    return hi - lo


def stats(d):
    return dict(mean_m=float(d.mean()), median_m=float(np.median(d)), p95_m=float(np.percentile(d, 95)),
                max_m=float(d.max()), n=int(len(d)))


def overlay(truth, mapped_pts, path):
    s = 6
    h, w = truth['occ'].shape
    img = np.full((h * s, w * s, 3), 255, np.uint8)
    img[np.kron(truth['occ'], np.ones((s, s), bool))] = (170, 170, 170)
    for x, y in mapped_pts:
        c, r = xy_to_cells(truth, np.array([[x, y]]))
        if 0 <= c[0] < w and 0 <= r[0] < h:
            cv2.rectangle(img, (c[0] * s + 1, r[0] * s + 1), (c[0] * s + s - 2, r[0] * s + s - 2), (40, 40, 220), -1)
    cv2.putText(img, 'grey = true walls, red = mapped walls (after start offset)', (10, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.imwrite(path, img)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('map_yaml')
    ap.add_argument('--truth', default=os.path.join(os.path.dirname(__file__), '../../maps/course/course_full_approx.yaml'))
    ap.add_argument('--start', default='0.06,0.0', help='x,y of the robot start in the truth frame (map frame origin)')
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    out = a.out or os.path.splitext(a.map_yaml)[0] + '_accuracy'
    start = np.array([float(v) for v in a.start.split(',')])
    mp, tr = load(a.map_yaml), load(a.truth)
    field = dist_field(tr)
    pts = cells_to_xy(mp, mp['occ'])
    pts_w = pts + start                                  # map frame -> truth frame, known start pose only
    raw = dists(pts_w, tr, field)
    res = minimize(lambda t: dists(transform(pts_w, t), tr, field, 0.5).mean(), [0, 0, 0], method='Nelder-Mead',
                   options=dict(xatol=1e-3, fatol=1e-5,
                            initial_simplex=[[0, 0, 0], [0.05, 0, 0], [0, 0.05, 0], [0, 0, 0.02]]))
    fit = res.x
    aligned = dists(transform(pts_w, fit), tr, field)
    # coverage: truth wall cells with a mapped wall within 10 cm (in the aligned frame)
    inv = (~(np.zeros_like(mp['occ']))).astype(np.uint8)
    mp_aligned = transform(pts_w, fit)
    mfield_img = np.ones(tr['occ'].shape, np.uint8)
    c, r = xy_to_cells(tr, mp_aligned)
    ok = (c >= 0) & (c < mfield_img.shape[1]) & (r >= 0) & (r < mfield_img.shape[0])
    mfield_img[r[ok], c[ok]] = 0
    near = cv2.distanceTransform(mfield_img, cv2.DIST_L2, 5) * tr['res']
    tr_r, tr_c = np.nonzero(tr['occ'])
    coverage = float((near[tr_r, tr_c] < 0.10).mean())
    sz_m, sz_t = free_bbox(mp, (0.0, 0.0)), free_bbox(tr, tuple(start))
    rep = dict(map=a.map_yaml, truth=a.truth,
               wall_distance_raw=stats(raw), wall_distance_aligned=stats(aligned),
               alignment=dict(dx_m=float(fit[0]), dy_m=float(fit[1]), yaw_deg=float(math.degrees(fit[2]))),
               coverage_of_true_walls_within_10cm=coverage,
               size=dict(mapped_free_bbox_m=[round(float(v), 3) for v in sz_m],
                         truth_free_bbox_m=[round(float(v), 3) for v in sz_t], rulebook_inner_m=list(INNER),
                         error_vs_rulebook_m=[round(float(sz_m[0] - INNER[0]), 3), round(float(sz_m[1] - INNER[1]), 3)],
                         error_vs_truth_map_m=[round(float(sz_m[0] - sz_t[0]), 3), round(float(sz_m[1] - sz_t[1]), 3)],
                         note='the truth map itself is 1 cell (5 cm) per side smaller than the rulebook inner size'))
    json.dump(rep, open(out + '.json', 'w'), indent=2)
    overlay(tr, pts_w, out + '_overlay.png')
    print(json.dumps(rep, indent=2))


if __name__ == '__main__':
    main()

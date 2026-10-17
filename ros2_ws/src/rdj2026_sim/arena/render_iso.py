#!/usr/bin/env python3
"""Render arena_layout.yaml as an isometric 3D view from the same angle as Figure 2 of the rulebook,
so a layout can be checked against the official drawing by eye.

Usage:  python3 arena/render_iso.py [layout.yaml] [out.png]
"""
import math
import os
import sys

import yaml
from PIL import Image, ImageDraw

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
src = sys.argv[1] if len(sys.argv) > 1 else os.path.join(PKG, 'arena', 'arena_layout.yaml')
out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(PKG, 'arena', 'iso_view.png')
L = yaml.safe_load(open(src))
FL, FW = L['field']['length'], L['field']['width']
H = L['field']['wall_height']
T = L['field']['wall_thickness']

# Figure 2 view, calibrated on the drawing's four outer corners (true isometric, ~0.198 px/mm),
# in the pixel frame of Figure 2 rendered 1800 px wide - so the render can be overlaid on it.
# The calibration assumes the 4868 x 2440 field of Figure 2.
EX = (840.0 / 4868, 480.0 / 4868)       # +x (front long wall) on screen, px per mm
EY = (415.0 / 2440, -245.0 / 2440)      # +y (towards the back wall)
EZ = (0, -78.0 / 400)                   # up
OX, OY = 290, 498                       # left-front floor corner
CANVAS = (1800, 1039)


def proj(x, y, z=0):
    return (OX + x * EX[0] + y * EY[0] + z * EZ[0], OY + x * EX[1] + y * EY[1] + z * EZ[1])


def depth(x, y):            # larger = closer to the viewer (front corner is closest)
    return x * 0.6 - y * 1.0


def box(x0, y0, x1, y1, z0, z1, top, side):
    pts = {}
    for xx in (x0, x1):
        for yy in (y0, y1):
            for zz in (z0, z1):
                pts[(xx, yy, zz)] = proj(xx, yy, zz)
    faces = [
        ([(x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)], side),         # front (y0) face
        ([(x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1)], shade(side, 0.85)),  # +x face
        ([(x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)], top),          # top
    ]
    return depth((x0 + x1) / 2, (y0 + y1) / 2), [([pts[p] for p in f], c) for f, c in faces]


def shade(c, k):
    return tuple(int(v * k) for v in c)


def seg_box(x1, y1, x2, y2, h, top, side, t=T):
    if abs(x2 - x1) >= abs(y2 - y1):
        return box(min(x1, x2), min(y1, y2) - t / 2, max(x1, x2), max(y1, y2) + t / 2, 0, h, top, side)
    return box(min(x1, x2) - t / 2, min(y1, y2), max(x1, x2) + t / 2, max(y1, y2), 0, h, top, side)


WOOD_TOP, WOOD_SIDE = (222, 170, 80), (196, 140, 52)
items = []
img = Image.new('RGB', CANVAS, (245, 245, 245))
d = ImageDraw.Draw(img)
d.polygon([proj(0, 0), proj(FL, 0), proj(FL, FW), proj(0, FW)], fill=(214, 160, 72), outline=(120, 80, 30))

flat = []
for p in L.get('patches', []):
    cx, cy = p['centre']; sx, sy = p['size']
    col = {'grass': (60, 160, 60), 'soil': (70, 55, 40), 'gravel': (110, 110, 110), 'sand': (200, 180, 120)}.get(p['surface'], (150, 150, 150))
    flat.append(([proj(cx - sx / 2, cy - sy / 2), proj(cx + sx / 2, cy - sy / 2), proj(cx + sx / 2, cy + sy / 2), proj(cx - sx / 2, cy + sy / 2)], col))
for r in L.get('ramps', []):
    cx, cy = r['centre']; ln, wd = r['length'], r['width']; a = math.radians(r.get('yaw', 0))
    cs = [(-ln / 2, -wd / 2), (ln / 2, -wd / 2), (ln / 2, wd / 2), (-ln / 2, wd / 2)]
    col = {'soil': (70, 60, 50), 'sand': (90, 90, 95), 'gravel': (95, 95, 100), 'wood': (180, 140, 90)}.get(r['surface'], (100, 100, 100))
    flat.append(([proj(cx + u * math.cos(a) - v * math.sin(a), cy + u * math.sin(a) + v * math.cos(a)) for u, v in cs], col))
bays = L['offloading_bays']
BAYCOL = {'blue': (40, 80, 200), 'white': (240, 240, 240), 'black': (40, 40, 40), 'red': (170, 40, 40),
          'grey': (90, 90, 90), 'dark_grey': (75, 75, 80)}
xe = bays.get('x_edges', [])
for i, colname in enumerate(bays.get('colours', [])[:max(0, len(xe) - 1)]):
    flat.append(([proj(xe[i], FW - bays['depth']), proj(xe[i + 1], FW - bays['depth']), proj(xe[i + 1], FW), proj(xe[i], FW)],
                 BAYCOL.get(colname, (150, 150, 150))))
for b in bays.get('zones', []):
    (x0, y0, x1, y1), c = b['rect'], b['colour']
    flat.append(([proj(x0, y0), proj(x1, y0), proj(x1, y1), proj(x0, y1)],
                 {'blue': (40, 80, 200), 'white': (240, 240, 240), 'black': (40, 40, 40), 'red': (170, 40, 40), 'grey': (90, 90, 90)}[c]))
for poly, col in flat:
    d.polygon(poly, fill=col)

for x0, y0, x1, y1 in ([0, 0, FL, 0], [FL, 0, FL, FW], [FL, FW, 0, FW], [0, FW, 0, 0]):
    items.append(seg_box(x0, y0, x1, y1, H, WOOD_TOP, WOOD_SIDE))
for w in L['walls']:
    items.append(seg_box(*w, H, WOOD_TOP, WOOD_SIDE))
for w in L.get('baffles', {}).get('boards', []):
    items.append(seg_box(*w, L['baffles']['height'], WOOD_TOP, WOOD_SIDE))
ls = L['loading_station']
if not ls.get('hide'):
    (lx, ly), (sx, sy) = ls['centre'], ls['size']
    items.append(box(lx - sx / 2, ly - sy / 2, lx + sx / 2, ly + sy / 2, ls['height'] - 30, ls['height'], (60, 170, 80), (40, 120, 60)))
c = L['cylinders']
for x, y in c['positions']:
    r = c['diameter'] / 2
    items.append(box(x - r, y - r, x + r, y + r, 0, c['height'], (90, 90, 95), (180, 180, 185)))
s = L['start']
if not s.get('hide'):
    x, y, _ = s['pose']
    items.append(box(x - 60, y - 60, x + 60, y + 60, 0, 150, (60, 60, 65), (40, 40, 45)))
g = L['gate']
if not g.get('hide'):
    items.append(seg_box(*g['from'], *g['to'], 300, (120, 120, 130), (90, 90, 100), t=60))

for _, faces in sorted(items, key=lambda it: it[0]):
    for poly, col in faces:
        d.polygon(poly, fill=col, outline=(110, 75, 30))
d.text((20, 20), f'{os.path.basename(src)}  -  same view as rulebook Figure 2', fill=(0, 0, 0))
img.save(out)
print('wrote', out)

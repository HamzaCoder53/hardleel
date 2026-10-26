#!/usr/bin/env python3
"""Ground-truth occupancy map of a Gazebo world: every wall box of the SDF (links with a <collision> box that reach above
0.1 m, i.e. walls, not the floor) is rasterised. Independent of the map the world was generated from.

    export_truth.py WORLD.sdf OUT_PREFIX [--res 0.05] [--pad 0.5]      -> OUT_PREFIX.pgm / .yaml  (map_server format)
"""
import argparse
import math
import re

import numpy as np
from PIL import Image

BOX = re.compile(r'<link name="[^"]*"><pose>([^<]+)</pose><collision name="c"><geometry><box><size>([^<]+)</size>')


def boxes(path):
    out = []
    for pose, size in BOX.findall(open(path).read()):
        p, s = [float(v) for v in pose.split()], [float(v) for v in size.split()]
        if p[2] + s[2] / 2 > 0.1 and s[2] > 0.1:
            out.append((p[0], p[1], p[5], s[0], s[1]))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('world'); ap.add_argument('out')
    ap.add_argument('--res', type=float, default=0.05); ap.add_argument('--pad', type=float, default=0.5)
    a = ap.parse_args()
    bs = boxes(a.world)
    ext = [(x - abs(sx * math.cos(t)) / 2 - abs(sy * math.sin(t)) / 2, x + abs(sx * math.cos(t)) / 2 + abs(sy * math.sin(t)) / 2,
            y - abs(sx * math.sin(t)) / 2 - abs(sy * math.cos(t)) / 2, y + abs(sx * math.sin(t)) / 2 + abs(sy * math.cos(t)) / 2) for x, y, t, sx, sy in bs]
    x0, x1 = min(e[0] for e in ext) - a.pad, max(e[1] for e in ext) + a.pad
    y0, y1 = min(e[2] for e in ext) - a.pad, max(e[3] for e in ext) + a.pad
    w, h = int(math.ceil((x1 - x0) / a.res)), int(math.ceil((y1 - y0) / a.res))
    img = np.full((h, w), 254, np.uint8)
    xs = x0 + (np.arange(w) + 0.5) * a.res
    ys = y0 + (np.arange(h) + 0.5) * a.res
    X, Y = np.meshgrid(xs, ys)
    for x, y, t, sx, sy in bs:
        c, s = math.cos(-t), math.sin(-t)
        lx, ly = c * (X - x) - s * (Y - y), s * (X - x) + c * (Y - y)
        inside = (np.abs(lx) <= sx / 2) & (np.abs(ly) <= sy / 2)
        img[::-1][inside] = 0
    Image.fromarray(img).save(a.out + '.pgm')
    open(a.out + '.yaml', 'w').write(f'image: {a.out.split("/")[-1]}.pgm\nmode: trinary\nresolution: {a.res}\n'
                                     f'origin: [{x0:.3f}, {y0:.3f}, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    print(f'{a.out}: {len(bs)} wall boxes, {w}x{h} cells, origin ({x0:.2f}, {y0:.2f}), {int((img == 0).sum())} occupied cells')


if __name__ == '__main__':
    main()

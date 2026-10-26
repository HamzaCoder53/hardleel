#!/usr/bin/env python3
"""Unrelated test world: a small maze (6 x 4 m, corridors >= 0.9 m wide) to check that mapping + navigation do not
depend on the competition course.   python3 arena/make_maze.py   -> maps/maze.{pgm,yaml} (+ run world_from_map.py).
Frame: start (0, 0) facing east, x east, y north (same convention as the course; the robot spawns 6 cm in x)."""
import os

import numpy as np
from PIL import Image

RES, W_M, H_M, T = 0.05, 6.0, 4.0, 0.10          # resolution, size (m), wall thickness
ORIGIN = (-0.5, -0.5)                              # start (0,0) = 0.5 m in from the lower-left corner
WALLS = [  # (x0, y0, x1, y1) in maze metres (origin lower-left), axis-aligned
    (0, 0, 6, 0), (0, 4, 6, 4), (0, 0, 0, 4), (6, 0, 6, 4),                       # outer
    (2.0, 0.0, 2.0, 2.8), (4.0, 1.2, 4.0, 4.0), (2.0, 2.8, 3.1, 2.8), (4.0, 1.2, 5.0, 1.2),
]
h, w = int(H_M / RES), int(W_M / RES)
img = np.full((h, w), 254, np.uint8)
for x0, y0, x1, y1 in WALLS:
    c0, c1 = int((min(x0, x1) - T / 2) / RES), int((max(x0, x1) + T / 2) / RES)
    r0, r1 = int((min(y0, y1) - T / 2) / RES), int((max(y0, y1) + T / 2) / RES)
    img[max(h - r1, 0):max(h - r0, 0), max(c0, 0):min(c1, w)] = 0
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'maps')
Image.fromarray(img).save(os.path.join(out, 'maze.pgm'))
open(os.path.join(out, 'maze.yaml'), 'w').write(
    f'image: maze.pgm\nmode: trinary\nresolution: {RES}\norigin: [{ORIGIN[0]}, {ORIGIN[1]}, 0]\nnegate: 0\n'
    'occupied_thresh: 0.65\nfree_thresh: 0.196\n')
print('maze map', img.shape)

"""map_tidy on synthetic maps: squaring, size check, outside cleared, gaps closed only between seen wall ends."""
import math
import os

import numpy as np
import pytest
import yaml

mt = pytest.importorskip('robot_slam.map_tidy')
pytest.importorskip('PIL')

RES = 0.05


def course_grid(gap_cells=0, clutter=True, size=(5.75, 3.05)):
    w, h = int(round(size[0] / RES)) + 10, int(round(size[1] / RES)) + 10
    g = np.full((h, w), -1, dtype=np.int8)
    r0, r1, c0, c1 = 5, 5 + int(round(size[1] / RES)) - 1, 5, 5 + int(round(size[0] / RES)) - 1
    g[r0 + 1:r1, c0 + 1:c1] = 0
    g[r0, c0:c1 + 1] = g[r1, c0:c1 + 1] = 100
    g[r0:r1 + 1, c0] = g[r0:r1 + 1, c1] = 100
    if gap_cells:
        g[r0, 40:40 + gap_cells] = 0                     # door-like gap in the top wall
    if clutter:
        g[1, 2] = 100                                    # speck outside the field
    return g


def write(tmp, g, name='m'):
    img = np.full(g.shape, 205, dtype=np.uint8)
    img[g == 0], img[g == 100] = 254, 0
    from PIL import Image
    Image.fromarray(img).save(os.path.join(tmp, name + '.pgm'))
    yaml.safe_dump({'image': name + '.pgm', 'mode': 'trinary', 'resolution': RES, 'origin': [-5.0, -3.0, 0],
                    'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.196}, open(os.path.join(tmp, name + '.yaml'), 'w'))
    return os.path.join(tmp, name + '.yaml')


def test_close_gaps_only_between_seen_ends():
    g = np.zeros((3, 12), dtype=np.int8)
    g[1, [0, 1, 4, 5, 11]] = 100                         # gap of 2 (fill), gap of 5 (keep)
    out, n = mt.close_gaps(g, 2)
    assert (out[1, 2:4] == 100).all() and (out[1, 6:11] == 0).all() and n == 2


def test_rotate_keeps_values():
    g = course_grid(clutter=False)
    r = mt.rotate_grid(g, math.radians(1.5))
    assert set(np.unique(r)) <= {-1, 0, 100}


def test_tidy_course_map(tmp_path):
    path = write(str(tmp_path), course_grid(gap_cells=1))
    assert mt.main([path, '--reference', 'none']) == 0
    _, out = mt.load(str(tmp_path / 'm_tidy.yaml'))
    assert out[1, 2] == -1                               # outside speck cleared
    assert out[5, 40] == 100                             # 5 cm gap closed


def test_wrong_size_skips(tmp_path):
    path = write(str(tmp_path), course_grid(size=(4.0, 3.05)))
    assert mt.main([path, '--reference', 'none']) == 2

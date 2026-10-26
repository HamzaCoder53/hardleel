"""map_freezer.remove_free_standing_blobs: workers/cylinders from the scan go, walls and wall stubs stay."""
import numpy as np
import pytest

mf = pytest.importorskip('robot_slam.map_freezer')
RES = 0.05


def room():
    g = np.zeros((60, 60), dtype=np.int16)          # 3 x 3 m
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = 100    # outer walls
    return g


def test_free_standing_worker_is_removed():
    g = room()
    g[28:33, 28:33] = 100                            # 0.25 m worker in the middle
    out, n = mf.remove_free_standing_blobs(g, RES)
    assert n == 1 and (out[28:33, 28:33] == 0).all()
    assert (out[0, :] == 100).all()                  # walls untouched


def test_post_touching_a_wall_is_kept():
    g = room()
    g[1:5, 30:33] = 100                              # stub attached to the top wall
    out, n = mf.remove_free_standing_blobs(g, RES)
    assert n == 0 and (out[1:5, 30:33] == 100).all()


def test_long_interior_wall_is_kept():
    g = room()
    g[30, 10:50] = 100                               # 2 m interior wall, free-standing but too long
    out, n = mf.remove_free_standing_blobs(g, RES)
    assert n == 0 and (out[30, 10:50] == 100).all()


def test_merge_keeps_frozen_known_cells_and_adds_new_area():
    frozen = np.full((4, 4), -1, dtype=np.int16)
    frozen[1:3, 1:3] = 0                               # known free square
    frozen[1, 1] = 100                                 # a wall it saw
    live = np.zeros((6, 8), dtype=np.int16)            # bigger live map, origin 1 cell left/below
    live[2, 2] = 100                                   # a worker the live map now marks on a frozen FREE cell
    live[5, 7] = 100                                   # new wall far outside the frozen bounds
    out = mf.merge_frozen(frozen, (0.0, 0.0), live, (-0.05, -0.05), 0.05)
    assert out.shape == live.shape
    assert out[2, 2] == 100                            # frozen (1,1) wall -> live (2,2): frozen value kept
    assert out[3, 3] == 0                              # frozen free cell wins over anything live
    assert out[5, 7] == 100                            # new area comes from the live map
    live2 = live.copy()
    live2[3, 3] = 100                                  # worker on a cell the frozen map knows as free
    assert mf.merge_frozen(frozen, (0.0, 0.0), live2, (-0.05, -0.05), 0.05)[3, 3] == 0


def test_two_workers_and_unknown_cells():
    g = room()
    g[10:15, 10:15] = 100
    g[40:44, 40:44] = 100
    g[20:25, 50:55] = -1                             # unknown stays unknown
    out, n = mf.remove_free_standing_blobs(g, RES)
    assert n == 2 and (out[20:25, 50:55] == -1).all()

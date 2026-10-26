#!/usr/bin/env python3
"""map_freezer: /map -> /map_nav (transient local) for Nav2's global costmap and the mission's obstacle check.

Workers stand still during a run but are moved between runs/scans (team, 25 Oct). So the phase-1 map can contain a
worker that is no longer there in phase 2 (a phantom wall that could even block a lane for the planner), and a
worker standing on the same spot would not count as "unexpected" for the obstacle log. Therefore:
  - until /mission/start: relay /map unchanged (auto mapping / explorer needs the live map);
  - at /mission/start: freeze the map AND remove small free-standing blobs (occupied clusters whose bounding box is
    <= blob_max_m and that do not touch a bigger structure within blob_gap_m) - workers and cylinders. Walls stay.
Anything still really there is marked live by the costmaps' obstacle layers (lidar, clears itself) and is logged as
an obstacle. The frozen map is re-sent every republish_s.
Params: blob_max_m (0.45), blob_gap_m (0.15), occupied (65), republish_s (2.0), clean_blobs (true).
Topics: in /map (OccupancyGrid), /mission/start (Empty), /mission/state (String; anything but WAIT_START also
freezes, in case the start message went to another subscriber only); out /map_nav (OccupancyGrid, transient local).
"""
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String


def _label(mask):
    """8-connected components -> (labels array, count). scipy if available, else a BFS."""
    try:
        from scipy.ndimage import label
        return label(mask, structure=np.ones((3, 3), dtype=int))
    except ImportError:
        lab = np.zeros(mask.shape, dtype=int)
        n = 0
        for i, j in zip(*np.nonzero(mask)):
            if lab[i, j]:
                continue
            n += 1
            stack = [(i, j)]
            lab[i, j] = n
            while stack:
                a, b = stack.pop()
                for da in (-1, 0, 1):
                    for db in (-1, 0, 1):
                        x, y = a + da, b + db
                        if 0 <= x < mask.shape[0] and 0 <= y < mask.shape[1] and mask[x, y] and not lab[x, y]:
                            lab[x, y] = n
                            stack.append((x, y))
        return lab, n


def merge_frozen(frozen, f_origin, live, l_origin, resolution):
    """Live-map geometry, frozen values wherever the frozen map KNOWS the cell (!= -1). So areas phase 1 never mapped
    (outside the frozen bounds, or unknown in it) still come in from the live map: without this a short mapping run
    left mission goals 'outside bounds' for the whole mission (sim, 26 Oct). Same resolution assumed."""
    out = live.copy()
    dc = int(round((f_origin[0] - l_origin[0]) / resolution))       # frozen (0, 0) cell inside the live grid
    dr = int(round((f_origin[1] - l_origin[1]) / resolution))
    fh, fw = frozen.shape
    lh, lw = live.shape
    r0, c0 = max(0, dr), max(0, dc)
    r1, c1 = min(lh, dr + fh), min(lw, dc + fw)
    if r1 <= r0 or c1 <= c0:
        return out
    f = frozen[r0 - dr:r1 - dr, c0 - dc:c1 - dc]
    known = f != -1
    out[r0:r1, c0:c1][known] = f[known]
    return out


def remove_free_standing_blobs(data, resolution, blob_max_m=0.45, gap_m=0.15, occupied=65):
    """data: 2D int8/int16 occupancy (-1 unknown, 0..100). Returns (cleaned copy, number of blobs removed).
    A blob = occupied component with bbox <= blob_max_m in both axes that has no cell of a BIGGER component within
    gap_m (so wall stubs / posts attached to walls are kept). Removed cells become free (0)."""
    occ = data >= occupied
    lab, n = _label(occ)
    if n == 0:
        return data.copy(), 0
    max_cells = blob_max_m / resolution
    gap = int(np.ceil(gap_m / resolution))
    small = []
    for k in range(1, n + 1):
        ii, jj = np.nonzero(lab == k)
        if ii.max() - ii.min() + 1 <= max_cells and jj.max() - jj.min() + 1 <= max_cells:
            small.append((k, ii, jj))
    small_ids = {k for k, _, _ in small}
    out = data.copy()
    removed = 0
    for k, ii, jj in small:
        i0, i1 = max(ii.min() - gap, 0), min(ii.max() + gap + 1, lab.shape[0])
        j0, j1 = max(jj.min() - gap, 0), min(jj.max() + gap + 1, lab.shape[1])
        near = set(np.unique(lab[i0:i1, j0:j1])) - {0, k}
        if near - small_ids:                         # touches a big structure (wall): keep it
            continue
        out[ii, jj] = 0
        removed += 1
    return out, removed


class MapFreezer(Node):
    def __init__(self):
        super().__init__('map_freezer')
        p = self.declare_parameter
        self.blob_max = float(p('blob_max_m', 0.45).value)
        self.blob_gap = float(p('blob_gap_m', 0.15).value)
        self.occupied = int(p('occupied', 65).value)
        self.clean = bool(p('clean_blobs', True).value)
        self.frozen = False
        self.last = None
        self.live = None
        self.frozen_grid, self.frozen_origin = None, None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(OccupancyGrid, '/map_nav', latched)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, 1)
        self.create_subscription(Empty, '/mission/start', self.on_start, 1)
        # backup trigger: a single `topic pub --once` reaches only the first matched subscriber (sim 25 Oct: the
        # mission started, this node never froze -> the worker got mapped and was never "unexpected")
        self.create_subscription(String, '/mission/state', self.on_state, 10)
        self.create_timer(float(p('republish_s', 2.0).value), self.republish)

    def on_map(self, m):
        if not self.frozen:
            self.last = m
            self.pub.publish(m)
        else:
            self.live = m                         # merged in at the next republish (areas phase 1 never saw)

    def on_state(self, m):
        if m.data and m.data != 'WAIT_START':
            self.on_start(None)

    def on_start(self, _):
        if self.frozen:
            return
        self.frozen = True
        if self.last is None:
            self.get_logger().warn('mission started but no /map received yet: /map_nav stays empty')
            return
        removed = 0
        if self.clean:
            m = self.last
            grid = np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
            cleaned, removed = remove_free_standing_blobs(grid, m.info.resolution, self.blob_max, self.blob_gap,
                                                          self.occupied)
            m.data = cleaned.astype(np.int8).ravel().tolist()
        m = self.last
        self.frozen_grid = np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        self.frozen_origin = (m.info.origin.position.x, m.info.origin.position.y)
        self.republish()
        self.get_logger().info(f'mission started: /map_nav frozen, {removed} free-standing blob(s) removed '
                               f'(workers/cylinders from the scan; live ones come from the lidar)')

    def republish(self):
        if not self.frozen or self.last is None:
            return
        live = self.live
        if live is not None and abs(live.info.resolution - self.last.info.resolution) < 1e-6:
            self.live = None
            lg = np.asarray(live.data, dtype=np.int16).reshape(live.info.height, live.info.width)
            merged = merge_frozen(self.frozen_grid, self.frozen_origin, lg,
                                  (live.info.origin.position.x, live.info.origin.position.y), live.info.resolution)
            out = OccupancyGrid()
            out.header.frame_id = live.header.frame_id
            out.info = live.info
            out.data = merged.astype(np.int8).ravel().tolist()
            self.last = out
        self.last.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.last)


def main():
    rclpy.init()
    n = MapFreezer()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

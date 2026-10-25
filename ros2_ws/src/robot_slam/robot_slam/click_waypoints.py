"""Click every waypoint on a freshly built map (matplotlib GUI; works under WSLg).

    click_waypoints MAP.yaml [--waypoints FILE] [--only a,b]      # nav2 map_server yaml (pgm/png)
    click_waypoints --topic /map [--waypoints FILE]               # grab one OccupancyGrid (needs ROS)
    click_waypoints MAP.yaml --waypoints draft.yaml --selftest    # headless check, writes a temp copy only

Waypoints file default: $HARDLEEL_WAYPOINTS or ~/hardleel_ws/waypoints/waypoints_course.yaml (created from the draft).
Map frame is metres, origin = start pose (0,0, arrow +x). Names are asked in file order.
Keys: left-click = position (heading stays the draft's), or with heading mode (h) a 2nd click aims the heading;
Enter/space = keep the draft pose; s = skip; u/Backspace = undo; h = toggle heading-click mode; right-click =
cancel the half-done point; q = save and quit; Esc = quit WITHOUT saving (confirm in the terminal).
Clicked poses get conf 'high', kept ones 'med', skipped ones are untouched. The old file is saved as .bak.
"""
import argparse
import math
import os
import shutil
import sys
import tempfile
import time
from collections import namedtuple

import numpy as np
import yaml

from robot_slam import waypoints as W

MapData = namedtuple('MapData', 'rgb extent')   # rgb: HxWx3 uint8 (row 0 = top); extent: [x0, x1, y0, y1] metres


# ---------------------------------------------------------------- map loading
def _grid_rgb(occ, free):
    rgb = np.full(occ.shape + (3,), 205, np.uint8)     # unknown = grey
    rgb[free] = 254
    rgb[occ] = 0
    return rgb


def load_map_yaml(path):
    from PIL import Image
    with open(path) as f:
        m = yaml.safe_load(f)
    img = m['image']
    img = img if os.path.isabs(img) else os.path.join(os.path.dirname(os.path.abspath(path)), img)
    a = np.asarray(Image.open(img).convert('L'), dtype=float)
    p = a / 255.0 if m.get('negate', 0) else (255.0 - a) / 255.0
    res, (ox, oy) = float(m['resolution']), [float(v) for v in m['origin'][:2]]
    if len(m['origin']) > 2 and abs(float(m['origin'][2])) > 1e-6:
        print('warning: map origin yaw != 0 is ignored')
    rgb = _grid_rgb(p > float(m.get('occupied_thresh', 0.65)), p < float(m.get('free_thresh', 0.196)))
    h, w = a.shape
    return MapData(rgb, [ox, ox + w * res, oy, oy + h * res])


def load_map_topic(topic='/map', timeout=15.0):
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    rclpy.init()
    node = rclpy.create_node('click_waypoints_map')
    box = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(OccupancyGrid, topic, box.append, qos)
    end = time.time() + timeout
    while not box and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()
    if not box:
        raise RuntimeError('no OccupancyGrid on %s within %.0f s' % (topic, timeout))
    g = box[0]
    d = np.array(g.data, dtype=np.int16).reshape(g.info.height, g.info.width)
    rgb = np.flipud(_grid_rgb((d >= 65), (d >= 0) & (d < 20)))
    res, o = g.info.resolution, g.info.origin.position
    return MapData(rgb, [o.x, o.x + g.info.width * res, o.y, o.y + g.info.height * res])


# ---------------------------------------------------------------- the clicker
class Clicker:
    """All state + handlers; GUI events just call these (so --selftest can drive them)."""

    def __init__(self, mapdata, path, only=None, heading_mode=False):
        self.path = path
        self.map = mapdata
        self.poses, self.seqs, self.meta = W.load_waypoints(path)
        self.names = [n for n in self.poses if (not only or n in only)]
        if not self.names:
            raise SystemExit('no waypoint names to ask for (--only %s)' % only)
        self.idx = 0
        self.status = {}              # name -> 'clicked' | 'kept' | 'skipped'
        self.history = []             # (name, old pose, old meta, old status)
        self.pending = None           # first click (x, y) in heading mode
        self.heading_mode = heading_mode
        self.fig = self.ax = self.rubber = None
        self.finished = False
        self.saved = False

    # -- state
    @property
    def current(self):
        return self.names[self.idx] if self.idx < len(self.names) else None

    def _commit(self, status, x=None, y=None, yaw=None):
        n = self.current
        self.history.append((n, self.poses[n], dict(self.meta.get(n, {})), self.status.get(n)))
        if status != 'skipped':
            ox, oy, oyaw = self.poses[n]
            self.poses[n] = (ox if x is None else x, oy if y is None else y, oyaw if yaw is None else yaw)
            m = self.meta.setdefault(n, {})
            m['conf'] = 'high' if status == 'clicked' else 'med'
            m['note'] = '%s on map %s' % (status, time.strftime('%Y-%m-%d %H:%M'))
        self.status[n] = status
        self.pending = None
        self.idx += 1
        if self.idx >= len(self.names):
            self.finish()

    # -- handlers (also used by the selftest)
    def click(self, x, y, button=1):
        if self.finished:
            return
        if button == 3:
            return self.cancel()
        if button != 1:
            return
        if not self.heading_mode:
            self._commit('clicked', x, y)
        elif self.pending is None:
            self.pending = (x, y)
        else:
            px, py = self.pending
            yaw = math.atan2(y - py, x - px) if math.hypot(x - px, y - py) > 0.05 else None
            self._commit('clicked', px, py, yaw)
        self.redraw()

    def keep(self):
        if self.finished:
            return
        if self.pending is not None:
            self._commit('clicked', *self.pending)       # position clicked, heading = draft
        else:
            self._commit('kept')
        self.redraw()

    def skip(self):
        if not self.finished:
            self._commit('skipped')
            self.redraw()

    def undo(self):
        if self.pending is not None:
            self.pending = None
        elif self.history and not self.finished:
            n, pose, meta, st = self.history.pop()
            self.poses[n] = pose
            self.meta[n] = meta
            if st is None:
                self.status.pop(n, None)
            else:
                self.status[n] = st
            self.idx = self.names.index(n)
        self.redraw()

    def toggle_heading(self):
        self.heading_mode = not self.heading_mode
        self.pending = None
        self.redraw()

    def cancel(self):
        self.pending = None
        self.redraw()

    def _close(self):
        if self.fig is not None:
            import matplotlib.pyplot as plt
            plt.close(self.fig)

    def finish(self):
        """Save (with .bak), print the table, close the window."""
        if self.finished:
            return
        self.finished = True
        W.save_waypoints(self.path, self.poses, self.seqs, self.meta)
        self.saved = True
        print('\nsaved %s (previous: %s.bak)' % (self.path, os.path.basename(self.path)))
        print('%-16s %8s %8s %8s  %-8s %s' % ('name', 'x', 'y', 'yaw_deg', 'status', 'conf'))
        for n, (x, y, yaw) in self.poses.items():
            print('%-16s %8.3f %8.3f %8.1f  %-8s %s' % (n, x, y, math.degrees(yaw), self.status.get(n, '-'),
                                                         self.meta.get(n, {}).get('conf', '')))
        for p in W.check_waypoints(self.poses, self.seqs):
            print('PROBLEM:', p)
        self._close()

    def abort(self, answer=None):
        ans = answer if answer is not None else input('Quit WITHOUT saving? [y/N] ')
        if ans.strip().lower().startswith('y'):
            self.finished = True
            print('aborted, nothing saved')
            self._close()

    def key(self, k):
        if k in ('enter', ' ', 'space'):
            self.keep()
        elif k == 's':
            self.skip()
        elif k in ('u', 'backspace'):
            self.undo()
        elif k == 'h':
            self.toggle_heading()
        elif k == 'q':
            self.finish()
        elif k == 'escape':
            self.abort()

    # -- drawing
    def _arrow(self, x, y, yaw, color, size, lw):
        self.ax.arrow(x, y, size * math.cos(yaw), size * math.sin(yaw), head_width=size * 0.45,
                      head_length=size * 0.45, length_includes_head=True, color=color, lw=lw, zorder=5)

    def redraw(self):
        if self.fig is None or self.finished:
            return
        ax = self.ax
        lim = (ax.get_xlim(), ax.get_ylim()) if ax.images else None
        ax.clear()
        ax.imshow(self.map.rgb, extent=self.map.extent, origin='upper', interpolation='nearest', zorder=0)
        ax.plot([0], [0], 'bs', ms=7, zorder=4)
        self._arrow(0, 0, 0.0, 'blue', 0.4, 1.5)
        ax.text(0.05, -0.12, 'start', color='blue', fontsize=8, zorder=6)
        cur = self.current
        for n, (x, y, yaw) in self.poses.items():
            if n == cur:
                col, size, fs = 'red', 0.5, 11
            elif self.status.get(n) in ('clicked', 'kept'):
                col, size, fs = 'green', 0.3, 8
            else:
                col, size, fs = ('grey' if n in self.names else 'lightgrey'), 0.3, 8
            ax.plot([x], [y], 'o', color=col, ms=11 if n == cur else 5, zorder=5)
            self._arrow(x, y, yaw, col, size, 2.5 if n == cur else 1)
            ax.text(x + 0.06, y + 0.06, n, color=col, fontsize=fs, zorder=6,
                    fontweight='bold' if n == cur else 'normal')
        if self.pending is not None:
            ax.plot(*self.pending, 'x', color='red', ms=12, mew=2, zorder=7)
        self.rubber, = ax.plot([], [], '-', color='red', lw=1, zorder=7)
        ax.set_aspect('equal')
        if lim:
            ax.set_xlim(*lim[0])
            ax.set_ylim(*lim[1])
        else:
            self._fit()
        note = self.meta.get(cur, {}).get('note', '') if cur else ''
        note = note if len(note) < 90 else note[:87] + '...'
        ax.set_title('%d/%d: %s — %s\nclick=pos%s  Enter/space=keep  s=skip  u=undo  h=heading[%s]  '
                     'RMB=cancel  q=save+quit  Esc=abort' % (
                         min(self.idx + 1, len(self.names)), len(self.names), cur, note,
                         ' then click=aim' if self.heading_mode else '', 'ON' if self.heading_mode else 'off'),
                     fontsize=8)
        ax.set_xlabel('map x [m] (start heading)')
        ax.set_ylabel('map y [m]')
        self.fig.canvas.draw_idle()

    def _fit(self):
        e, rgb = self.map.extent, self.map.rgb
        known = np.argwhere(rgb[..., 0] != 205)
        xs, ys = [0.0], [0.0]
        if len(known):
            h, w = rgb.shape[:2]
            (r0, c0), (r1, c1) = known.min(0), known.max(0)
            sx, sy = (e[1] - e[0]) / w, (e[3] - e[2]) / h
            xs += [e[0] + c0 * sx, e[0] + (c1 + 1) * sx]
            ys += [e[3] - (r1 + 1) * sy, e[3] - r0 * sy]
        for x, y, _ in self.poses.values():
            xs.append(x)
            ys.append(y)
        self.ax.set_xlim(min(xs) - 0.5, max(xs) + 0.5)
        self.ax.set_ylim(min(ys) - 0.5, max(ys) + 0.5)

    def run(self):
        import matplotlib.pyplot as plt
        for k in list(plt.rcParams):                      # free our keys; keep pan/zoom
            if k.startswith('keymap.') and k not in ('keymap.pan', 'keymap.zoom'):
                plt.rcParams[k] = []
        self.fig, self.ax = plt.subplots(figsize=(11, 8))
        c = self.fig.canvas
        c.mpl_connect('button_press_event', self._on_button)
        c.mpl_connect('key_press_event', lambda ev: self.key(ev.key))
        c.mpl_connect('motion_notify_event', self._on_motion)
        self.redraw()
        plt.show()
        if not self.finished:                             # window closed by hand: do not lose the work
            print('window closed, saving')
            self.finish()

    def _on_button(self, ev):
        tb = getattr(self.fig.canvas, 'toolbar', None)
        if ev.inaxes is not self.ax or ev.xdata is None or (tb is not None and str(getattr(tb, 'mode', ''))):
            return
        b = ev.button
        self.click(ev.xdata, ev.ydata, b if isinstance(b, int) else getattr(b, 'value', 1))

    def _on_motion(self, ev):
        if self.pending is not None and ev.inaxes is self.ax and self.rubber is not None and ev.xdata is not None:
            self.rubber.set_data([self.pending[0], ev.xdata], [self.pending[1], ev.ydata])
            self.fig.canvas.draw_idle()


# ---------------------------------------------------------------- selftest
def selftest(mapdata, draft):
    """Drive the handlers without a window; works on a temp copy of the draft. Raises AssertionError on failure."""
    tmp = tempfile.mkdtemp(prefix='clickwp_')
    path = os.path.join(tmp, 'wp.yaml')
    shutil.copyfile(draft, path)
    orig = W.load_waypoints(path)[0]
    c = Clicker(mapdata, path)
    names = list(c.names)
    c.keep()                                           # 1: keep draft
    c.click(1.0, 2.0)                                  # 2: click, heading stays draft
    assert c.poses[names[1]][:2] == (1.0, 2.0) and c.poses[names[1]][2] == orig[names[1]][2]
    c.toggle_heading()
    c.click(3.0, 1.0)                                  # 3: heading mode, first click
    c.click(3.0, 2.0)                                  #    second click -> yaw +90 deg
    assert abs(c.poses[names[2]][2] - math.pi / 2) < 1e-9
    c.click(0, 0, button=3)                            # right click, nothing pending: no change
    c.skip()                                           # 4: skip
    assert c.status[names[3]] == 'skipped' and c.poses[names[3]] == orig[names[3]]
    c.undo()                                           # back to 4
    assert c.current == names[3] and names[3] not in c.status
    c.click(5.0, 5.0)
    c.click(5.0, 5.0 + 1.0)                            # 4 clicked, yaw +90
    c.click(9.0, 9.0)
    c.cancel()                                         # half-done point cancelled
    assert c.pending is None and c.current == names[4]
    c.toggle_heading()
    c.key('enter')                                     # 5 kept
    c.key('q')                                         # save + quit early
    assert c.saved and os.path.exists(path + '.bak')
    new, seqs, meta = W.load_waypoints(path)
    assert list(new) == names
    assert new[names[1]][:2] == (1.0, 2.0) and meta[names[1]]['conf'] == 'high'
    assert new[names[0]] == orig[names[0]] and meta[names[0]]['conf'] == 'med'
    assert abs(math.degrees(new[names[3]][2]) - 90.0) < 0.05 and new[names[3]][:2] == (5.0, 5.0)
    assert new[names[5]] == orig[names[5]]
    assert W.check_waypoints(new, seqs) == []
    # --only redo: one name, finishes (and saves) after the single click
    c2 = Clicker(mapdata, path, only=[names[7]])
    c2.click(-1.0, 0.5)
    assert c2.saved and W.load_waypoints(path)[0][names[7]][:2] == (-1.0, 0.5)
    # rendering under the current backend
    import matplotlib.pyplot as plt
    c3 = Clicker(mapdata, path)
    c3.fig, c3.ax = plt.subplots()
    c3.redraw()
    c3.fig.savefig(os.path.join(tmp, 'view.png'))
    print('selftest OK (temp dir %s)' % tmp)
    return tmp


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('map', nargs='?', help='map_server .yaml')
    ap.add_argument('--topic', help='grab one OccupancyGrid from this topic instead of a file')
    ap.add_argument('--waypoints', default=None, help='waypoint file (default: see save_pose)')
    ap.add_argument('--only', default='', help='comma-separated names to (re)do')
    ap.add_argument('--heading', action='store_true', help='start with heading-click mode on')
    ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args(argv)
    if not a.map and not a.topic:
        ap.error('give a MAP.yaml or --topic /map')
    from robot_slam import save_pose
    path = os.path.expanduser(a.waypoints) if a.waypoints else save_pose.default_file()
    md = load_map_topic(a.topic) if a.topic else load_map_yaml(a.map)
    if a.selftest:
        selftest(md, path if os.path.exists(path) else save_pose.packaged_draft())
        return 0
    if save_pose.ensure_file(path):
        print('created %s from the packaged draft' % path)
    only = [s.strip() for s in a.only.split(',') if s.strip()]
    unknown = [n for n in only if n not in W.load_waypoints(path)[0]]
    if unknown:
        ap.error('unknown names in --only: %s' % ', '.join(unknown))
    Clicker(md, path, only or None, a.heading).run()
    return 0


if __name__ == '__main__':
    sys.exit(main())

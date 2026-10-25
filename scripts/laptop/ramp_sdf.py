#!/usr/bin/env python3
"""Print an SDF ramp model (up-slope, plateau, down-slope, each a third of the length) for ros_gz_sim create.
   ramp_sdf.py <height_mm> [length_mm=1140] [width_mm=500]      built along +x, rotate with -Y at spawn."""
import math
import sys

h = float(sys.argv[1]) / 1000
L = (float(sys.argv[2]) if len(sys.argv) > 2 else 1140) / 1000
W = (float(sys.argv[3]) if len(sys.argv) > 3 else 500) / 1000
seg, t = L / 3, 0.01
slope, hyp = math.atan2(h, seg), math.hypot(seg, h)


def box(name, x, z, sx, sz, pitch=0.0):
    g = f'<geometry><box><size>{sx:.4f} {W:.4f} {sz:.4f}</size></box></geometry>'
    return (f'<link name="{name}"><pose>{x:.4f} 0 {z:.4f} 0 {pitch:.4f} 0</pose>'
            f'<collision name="c">{g}</collision><visual name="v">{g}</visual></link>')


print(f'<sdf version="1.6"><model name="ramp"><static>true</static>'
      f'{box("up", -seg, h / 2, hyp, t, -slope)}{box("top", 0, h - t / 2, seg, t)}'
      f'{box("down", seg, h / 2, hyp, t, slope)}{box("fill", 0, (h - t) / 2, seg, h - t)}</model></sdf>')

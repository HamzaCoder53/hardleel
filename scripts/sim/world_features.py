#!/usr/bin/env python3
"""Spawn the course features (ramps, bumps, docks, screen, bins, cylinders, grass) into the running sim.

    world_features.py [--config FILE] [--only dirt_ramp,docks,...] [--screen CLASS]    ( --screen only swaps the image )
Models are static SDF created through Gazebo's create service, so run it AFTER sim.launch.py is up.
The screen shows a real waste photo (test_photos/<class>*.jpg) as a texture; --screen plastic|metal|organic swaps it.
"""
import argparse
import glob
import math
import os
import subprocess
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '../..'))
DEFAULT = os.path.join(REPO, 'ros2_ws/src/rdj2026_sim/config/course_features.yaml')
WORLD = 'rdj2026'


def create(name, sdf, x, y, z=0.0, yaw=0.0):
    r = subprocess.run(['gz', 'service', '-s', f'/world/{WORLD}/create', '--reqtype', 'gz.msgs.EntityFactory',
                        '--reptype', 'gz.msgs.Boolean', '--timeout', '5000', '--req',
                        'sdf: "%s" name: "%s" pose: {position: {x: %f y: %f z: %f} orientation: {z: %f w: %f}}'
                        % (sdf.replace('"', '\\"').replace('\n', ' '), name, x, y, z, math.sin(yaw / 2), math.cos(yaw / 2))],
                       capture_output=True, text=True)
    ok = 'true' in r.stdout
    print(f'{"ok  " if ok else "FAIL"} {name}' + ('' if ok else f' {r.stdout.strip()} {r.stderr.strip()[:120]}'))
    return ok


def remove(name):
    subprocess.run(['gz', 'service', '-s', f'/world/{WORLD}/remove', '--reqtype', 'gz.msgs.Entity', '--reptype',
                    'gz.msgs.Boolean', '--timeout', '3000', '--req', f'name: "{name}" type: MODEL'], capture_output=True)


def box_link(name, x, y, z, sx, sy, sz, pitch=0.0, color=None, collide=True, mu=None):
    g = f'<geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>'
    mat = ''
    if color:
        mat = f'<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>'
    fric = f'<surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface>' if mu else ''
    col = f'<collision name="c">{g}{fric}</collision>' if collide else ''
    return (f'<link name="{name}"><pose>{x:.4f} {y:.4f} {z:.4f} 0 {pitch:.4f} 0</pose>{col}'
            f'<visual name="v">{g}{mat}</visual></link>')


def ramp_sdf(name, L, W, h, bumps, bump_h, color='0.55 0.42 0.25 1'):
    """Up-slope, plateau (with bumps), down-slope along +x; rotate at spawn for other directions."""
    seg, t = L / 3, 0.01
    slope, hyp = math.atan2(h, seg), math.hypot(seg, h)
    links = [box_link('up', -seg, 0, h / 2, hyp, W, t, -slope, color), box_link('top', 0, 0, h - t / 2, seg, W, t, 0, color, mu=1.5),
             box_link('down', seg, 0, h / 2, hyp, W, t, slope, color), box_link('fill', 0, 0, (h - t) / 2, seg, W, h - t, 0, color)]
    for i in range(bumps):                      # rough middle: small bars across the plateau, higher friction
        bx = -seg / 2 + (i + 0.5) * seg / max(bumps, 1)
        links.append(box_link(f'b{i}', bx, 0, h + bump_h / 2, 0.03, W * 0.9, bump_h, 0, '0.4 0.3 0.2 1', mu=2.0))
    return f'<sdf version="1.6"><model name="{name}"><static>true</static>{"".join(links)}</model></sdf>'


def static_model(name, links):
    return f'<sdf version="1.6"><model name="{name}"><static>true</static>{links}</model></sdf>'


def textured_screen(w, h, image):
    g = f'<geometry><box><size>0.01 {w} {h}</size></box></geometry>'
    mat = (f'<material><diffuse>1 1 1 1</diffuse><pbr><metal><albedo_map>file://{image}</albedo_map>'
           f'<metalness>0.0</metalness><roughness>1.0</roughness></metal></pbr></material>') if image else \
        '<material><ambient>0.9 0.9 0.9 1</ambient><diffuse>0.9 0.9 0.9 1</diffuse></material>'
    return f'<link name="l"><collision name="c">{g}</collision><visual name="v">{g}{mat}</visual></link>'


def photo_for(cls):
    for pat in (f'test_photos/{cls}*.jpg', f'test_photos/{cls}*.png'):
        f = sorted(glob.glob(os.path.join(REPO, pat)))
        if f:
            return f[0]
    return None


def spawn_screen(c, cls):
    s = c['screen']
    remove('screen')
    img = photo_for(cls)
    yaw = math.pi                               # panel faces west (towards the robot)
    ok = create('screen', static_model('screen', textured_screen(s['width_m'], s['height_m'], img)),
                s['x'], s['y'], s['z_center_m'], 0.0)
    print(f'screen shows {cls} ({os.path.basename(img) if img else "no photo found - plain panel"})')
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', default=DEFAULT)
    ap.add_argument('--only', default='')
    ap.add_argument('--screen', default=None)
    a = ap.parse_args()
    c = yaml.safe_load(open(a.config))
    only = set(filter(None, a.only.split(',')))
    want = lambda k: (not only or k in only) and (c.get(k) or {}).get('enabled', False) if isinstance(c.get(k), dict) else False
    if a.screen:
        return 0 if spawn_screen(c, a.screen) else 1
    ok = True
    if want('dirt_ramp'):
        d = c['dirt_ramp']
        ok &= create('dirt_ramp', ramp_sdf('dirt_ramp', d['length_m'], d['width_m'], d['height_m'], d['rough_bumps'], d['bump_height_m']),
                     d['x'], d['y_center'], 0.0, math.pi / 2)            # along +y (up-slope first when driving north)
    if want('gravel_ramp'):
        d = c['gravel_ramp']
        ok &= create('gravel_ramp', ramp_sdf('gravel_ramp', d['length_m'], d['width_m'], d['height_m'], d['rough_bumps'], d['bump_height_m'], '0.6 0.6 0.6 1'),
                     d['x_center'], d['y'], 0.0, 0.0)
    if want('docks'):
        d = c['docks']
        for k, y in d['y'].items():
            ok &= create(f'dock_{k}', static_model(f'dock_{k}', box_link('l', 0, 0, d['height_m'] / 2, d['x_max'] - d['x_min'], d['thickness_m'],
                                                                           d['height_m'], 0, '0.1 0.6 0.4 1')),
                         (d['x_min'] + d['x_max']) / 2, y)
    if want('screen'):
        ok &= spawn_screen(c, c['screen']['default_image'])
    if want('bins'):
        b = c['bins']
        cols = {'blue': '0.1 0.2 0.9 1', 'white': '0.95 0.95 0.95 1', 'black': '0.05 0.05 0.05 1'}
        for k, (x, y) in b['positions'].items():
            ok &= create(f'bin_{k}', static_model(f'bin_{k}', box_link('l', 0, 0, b['height_m'] / 2, b['size_m'], b['size_m'], b['height_m'], 0, cols[k])), x, y)
    for cyl in c.get('cylinders', []):
        if cyl.get('enabled') and (not only or 'cylinders' in only):
            g = f'<geometry><cylinder><radius>{cyl["radius_m"]}</radius><length>{cyl["height_m"]}</length></cylinder></geometry>'
            ok &= create(cyl['name'], static_model(cyl['name'], f'<link name="l"><collision name="c">{g}</collision><visual name="v">{g}</visual></link>'),
                         cyl['x'], cyl['y'], cyl['height_m'] / 2)
    if want('grass_strip'):
        d = c['grass_strip']
        ok &= create('grass', static_model('grass', box_link('l', 0, 0, 0.005, d['x_max'] - d['x_min'], d['width_m'], 0.01, 0, '0.1 0.6 0.1 1', mu=1.8)),
                     (d['x_min'] + d['x_max']) / 2, d['y_center'])
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())

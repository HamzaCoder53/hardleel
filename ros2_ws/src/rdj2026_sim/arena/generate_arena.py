#!/usr/bin/env python3
"""Build the RDJ 2026 simulation assets from arena_layout.yaml.

Writes:
  worlds/rdj2026.sdf          Gazebo (Ignition) Fortress world
  maps/rdj2026_truth.pgm/.yaml ground-truth occupancy map for Nav2 (walls + fixed fixtures)
  arena/preview.png           labelled top view (only if Pillow is installed)

Usage:  python3 arena/generate_arena.py   (from the package root, or any directory)
"""
import math
import os

import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
L = yaml.safe_load(open(os.path.join(PKG, 'arena', 'arena_layout.yaml')))

MM = 0.001
F = L['field']
FL, FW = F['length'] * MM, F['width'] * MM
WH, WT = F['wall_height'] * MM, F['wall_thickness'] * MM

COLOURS = {  # rgb 0..1
    'wall': (0.72, 0.52, 0.30), 'blue': (0.05, 0.25, 0.85), 'white': (0.95, 0.95, 0.95),
    'black': (0.05, 0.05, 0.05), 'sand': (0.86, 0.75, 0.52), 'gravel': (0.45, 0.45, 0.45),
    'green': (0.20, 0.60, 0.25), 'red': (0.80, 0.10, 0.10), 'gate': (0.95, 0.75, 0.05),
    'cyl': (0.55, 0.55, 0.60), 'screen': (0.10, 0.10, 0.15), 'floor': (0.80, 0.68, 0.48),
    'grass': (0.15, 0.55, 0.15), 'soil': (0.25, 0.20, 0.15), 'wood': (0.70, 0.55, 0.35),
}


# --------------------------------------------------------------------------- geometry
def segment_boxes():
    """All wall segments (x1,y1,x2,y2,height) in metres, outer walls included."""
    segs = [(0, 0, FL, 0), (FL, 0, FL, FW), (FL, FW, 0, FW), (0, FW, 0, 0)]
    segs += [tuple(v * MM for v in w) for w in L['walls']]
    bays = L['offloading_bays']
    for x in bays['x_edges']:
        segs.append((x * MM, FW, x * MM, FW - bays['depth'] * MM))
    out = [s + (WH,) for s in segs]
    for b in L.get('baffles', {}).get('boards', []):
        out.append(tuple(v * MM for v in b) + (L['baffles']['height'] * MM,))
    return out


def box_from_segment(x1, y1, x2, y2):
    length = math.hypot(x2 - x1, y2 - y1) + WT
    return ((x1 + x2) / 2, (y1 + y2) / 2, math.atan2(y2 - y1, x2 - x1), length)


def rgba(c, a=1.0):
    r, g, b = COLOURS[c]
    return f'{r} {g} {b} {a}'


def material(c):
    return (f'<material><ambient>{rgba(c)}</ambient><diffuse>{rgba(c)}</diffuse>'
            f'<specular>0.1 0.1 0.1 1</specular></material>')


def box_link(name, x, y, z, yaw, sx, sy, sz, colour, collide=True, pitch=0.0):
    sx, sy, sz = max(sx, 0.002), max(sy, 0.002), max(sz, 0.002)    # Gazebo aborts on zero/negative sizes
    geo = f'<geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>'
    col = f'<collision name="c">{geo}</collision>' if collide else ''
    return (f'<link name="{name}"><pose>{x:.4f} {y:.4f} {z:.4f} 0 {pitch:.4f} {yaw:.4f}</pose>'
            f'{col}<visual name="v">{geo}{material(colour)}</visual></link>')


def static_model(name, links, pose='0 0 0 0 0 0'):
    return f'<model name="{name}"><static>true</static><pose>{pose}</pose>{"".join(links)}</model>'


# --------------------------------------------------------------------------- world
def build_world():
    models = []

    walls = []
    for i, (x1, y1, x2, y2, h) in enumerate(segment_boxes()):
        cx, cy, yaw, length = box_from_segment(x1, y1, x2, y2)
        walls.append(box_link(f'wall_{i}', cx, cy, h / 2, yaw, length, WT, h, 'wall'))
    models.append(static_model('field_walls', walls))

    # floor markings (no collision, 2 mm proud of the ground)
    marks = []
    bays = L['offloading_bays']
    xe = bays['x_edges']
    for i, colour in enumerate(bays['colours']):
        w, d = (xe[i + 1] - xe[i]) * MM, bays['depth'] * MM
        marks.append(box_link(f'bay_{i}_{colour}', (xe[i] + xe[i + 1]) / 2 * MM, FW - d / 2, 0.001, 0,
                              w - WT, d - WT, 0.002, colour, collide=False))
    sx, sy, _ = L['start']['pose']
    px, py = L['start']['pad_size']
    marks.append(box_link('start_pad', sx * MM, sy * MM, 0.001, 0, px * MM, py * MM, 0.002, 'red', collide=False))
    models.append(static_model('floor_markings', marks))

    # deposit deck: plate at wall height over the bays with one notch per bay (photos 23-29)
    deck = bays.get('deck')
    if deck:
        t, dz = 0.018, deck['height'] * MM - 0.009
        nd, nw = deck['notch_depth'] * MM, deck['notch_width'] * MM
        d = bays['depth'] * MM
        x0, x1 = xe[0] * MM, xe[-1] * MM
        links = [box_link('back', (x0 + x1) / 2, FW - (d - nd) / 2, dz, 0, x1 - x0, d - nd, t, 'wall')]
        for i in range(len(xe) - 1):
            a, b = xe[i] * MM, xe[i + 1] * MM
            c = (a + b) / 2
            yc = FW - d + nd / 2
            for k, (l0, l1) in enumerate(((a, c - nw / 2), (c + nw / 2, b))):
                links.append(box_link(f'front_{i}_{k}', (l0 + l1) / 2, yc, dz, 0, l1 - l0, nd, t, 'wall'))
        models.append(static_model('deposit_deck', links))

    # flat terrain patches (grass, soil...) - visual + thin collision
    for ptch in L.get('patches', []):
        cx, cy = (v * MM for v in ptch['centre'])
        sx, sy = ptch['size'][0] * MM, ptch['size'][1] * MM
        h = ptch.get('height', 10) * MM
        models.append(static_model(ptch['name'], [box_link('p', cx, cy, h / 2, math.radians(ptch.get('yaw', 0)),
                                                             sx, sy, h, ptch['surface'])]))

    # loading station
    ls = L['loading_station']
    lx, ly = ls['centre']
    lsx, lsy = ls['size']
    lh = ls['height'] * MM
    links = [box_link('table', lx * MM, ly * MM, lh / 2, 0, lsx * MM, lsy * MM, lh, 'green')]
    slot_cols = {'plastic': 'blue', 'organic': 'white', 'metal': 'black'}
    for i, slot in enumerate(ls['slots']):
        oy = (ly - lsy / 2 + lsy * (i + 0.5) / len(ls['slots'])) * MM
        links.append(box_link(f'slot_{i}_{slot}', lx * MM, oy, lh + 0.02, 0, 0.08, 0.08, 0.04,
                              slot_cols.get(slot, 'gate'), collide=False))
    models.append(static_model('loading_station', links))

    # ramps: up-slope, plateau, down-slope (each a third of the length)
    for r in L['ramps']:
        cx, cy = (v * MM for v in r['centre'])
        length, width, h = r['length'] * MM, r['width'] * MM, r['height'] * MM
        yaw = math.radians(r.get('yaw', 0))
        seg = length / 3
        slope = math.atan2(h, seg)
        hyp = math.hypot(seg, h)
        t = 0.01
        # built along +x in the model frame, then the whole model is rotated by yaw
        links = [
            box_link('up', -seg, 0, h / 2, 0, hyp, width, t, r['surface'], pitch=-slope),
            box_link('top', 0, 0, h - t / 2, 0, seg, width, t, r['surface']),
            box_link('down', seg, 0, h / 2, 0, hyp, width, t, r['surface'], pitch=slope),
            # solid fill under the plateau so nothing falls through
            box_link('fill', 0, 0, (h - t) / 2, 0, seg, width, h - t, r['surface']),
        ]
        models.append(static_model(r['name'], links, pose=f'{cx:.4f} {cy:.4f} 0 0 0 {yaw:.4f}'))

    # display screen
    ds = L['display_screen']
    if not ds.get('hide'):
        dx, dy = ds['centre']
        dw, dd, dh = ds['size']
        models.append(static_model('display_screen', [
            box_link('panel', dx * MM, dy * MM, 0.05 + dh * MM / 2, 0, dw * MM, dd * MM, dh * MM, 'screen')]))

    # cylinders (judges move these between rounds)
    c = L['cylinders']
    for i, (x, y) in enumerate(c['positions']):
        r, h = c['diameter'] * MM / 2, c['height'] * MM
        geo = f'<geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>'
        models.append(
            f'<model name="cylinder_{i}"><static>true</static><pose>{x * MM:.4f} {y * MM:.4f} {h / 2:.4f} 0 0 0</pose>'
            f'<link name="link"><collision name="c">{geo}</collision>'
            f'<visual name="v">{geo}{material("cyl")}</visual></link></model>')

    # gate (moved up/down by rdj2026_sim/gate_controller)
    g = L['gate']
    if not g.get('hide'):
        (gx1, gy1), (gx2, gy2) = g['from'], g['to']
        cx, cy, yaw, length = box_from_segment(gx1 * MM, gy1 * MM, gx2 * MM, gy2 * MM)
        gh = g['height'] * MM
        models.append(
            f'<model name="gate"><static>true</static><pose>{cx:.4f} {cy:.4f} {gh / 2:.4f} 0 0 {yaw:.4f}</pose>'
            + box_link('bar', 0, 0, 0, 0, length - 2 * WT, WT * 2, gh, 'gate')[0:] + '</model>')

    start_x, start_y, start_yaw = L['start']['pose']
    return f'''<?xml version="1.0"?>
<!-- GENERATED by arena/generate_arena.py from arena_layout.yaml - edit the YAML, not this file. -->
<sdf version="1.8">
  <world name="rdj2026">
    <physics name="1ms" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu"/>

    <scene>
      <ambient>0.5 0.5 0.5 1</ambient>
      <background>0.75 0.8 0.85 1</background>
      <shadows>false</shadows>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
    </light>

    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>30 30</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>30 30</size></plane></geometry>
          {material("floor")}</visual>
      </link>
    </model>

    {chr(10).join("    " + m for m in models)}

    <!-- start pose: x={start_x * MM:.3f} y={start_y * MM:.3f} yaw={math.radians(start_yaw):.4f} -->
  </world>
</sdf>
'''


# --------------------------------------------------------------------------- truth map
RES = 0.05
PAD = 0.5


def build_map():
    w, h = int(round((FL + 2 * PAD) / RES)), int(round((FW + 2 * PAD) / RES))
    grid = [[254] * w for _ in range(h)]   # free

    def cell(x, y):
        return int((x + PAD) / RES), h - 1 - int((y + PAD) / RES)

    def fill_rect_rot(cx, cy, yaw, sx, sy):
        c, s = math.cos(yaw), math.sin(yaw)
        r = math.hypot(sx, sy) / 2
        x0, y1 = cell(cx - r, cy - r)
        x1, y0 = cell(cx + r, cy + r)
        for j in range(max(0, y0), min(h, y1 + 1)):
            for i in range(max(0, x0), min(w, x1 + 1)):
                px, py = (i + 0.5) * RES - PAD, (h - 1 - j + 0.5) * RES - PAD
                dx, dy = px - cx, py - cy
                u, v = dx * c + dy * s, -dx * s + dy * c
                if abs(u) <= sx / 2 + RES / 2 and abs(v) <= sy / 2 + RES / 2:
                    grid[j][i] = 0

    # outside the field = unknown
    for j in range(h):
        for i in range(w):
            px, py = (i + 0.5) * RES - PAD, (h - 1 - j + 0.5) * RES - PAD
            if not (0 <= px <= FL and 0 <= py <= FW):
                grid[j][i] = 205
    for (x1, y1, x2, y2, _) in segment_boxes():
        cx, cy, yaw, length = box_from_segment(x1, y1, x2, y2)
        fill_rect_rot(cx, cy, yaw, length, WT)
    ls = L['loading_station']
    fill_rect_rot(ls['centre'][0] * MM, ls['centre'][1] * MM, 0, ls['size'][0] * MM, ls['size'][1] * MM)
    ds = L['display_screen']
    if not ds.get('hide'):
        fill_rect_rot(ds['centre'][0] * MM, ds['centre'][1] * MM, 0, ds['size'][0] * MM, ds['size'][1] * MM)

    os.makedirs(os.path.join(PKG, 'maps'), exist_ok=True)
    with open(os.path.join(PKG, 'maps', 'rdj2026_truth.pgm'), 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode())
        f.write(bytes(v for row in grid for v in row))
    with open(os.path.join(PKG, 'maps', 'rdj2026_truth.yaml'), 'w') as f:
        f.write(f'image: rdj2026_truth.pgm\nmode: trinary\nresolution: {RES}\n'
                f'origin: [{-PAD}, {-PAD}, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    return w, h


# --------------------------------------------------------------------------- preview
def build_preview():
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        print('Pillow not installed - skipping preview.png')
        return
    S = 120  # px per metre
    m = 40
    img = Image.new('RGB', (int(FL * S) + 2 * m, int(FW * S) + 2 * m), (235, 235, 235))
    d = ImageDraw.Draw(img)

    def P(x, y):
        return (m + x * S, m + (FW - y) * S)

    def rgb(c):
        return tuple(int(v * 255) for v in COLOURS[c])

    d.rectangle([P(0, FW), P(FL, 0)], fill=rgb('floor'))
    bays = L['offloading_bays']
    xe = bays['x_edges']
    for i, colour in enumerate(bays['colours']):
        d.rectangle([P(xe[i] * MM, FW), P(xe[i + 1] * MM, FW - bays['depth'] * MM)], fill=rgb(colour), outline=(90, 90, 90))
    def rot_rect(cx, cy, sx, sy, yaw_deg, colour):
        c, s_ = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
        pts = [(cx + u * c - v * s_, cy + u * s_ + v * c) for u, v in
               ((-sx / 2, -sy / 2), (sx / 2, -sy / 2), (sx / 2, sy / 2), (-sx / 2, sy / 2))]
        d.polygon([P(x, y) for x, y in pts], fill=rgb(colour))

    for ptch in L.get('patches', []):
        cx, cy = (v * MM for v in ptch['centre'])
        rot_rect(cx, cy, ptch['size'][0] * MM, ptch['size'][1] * MM, ptch.get('yaw', 0), ptch['surface'])
        d.text(P(cx - 0.2, cy), ptch['name'], fill=(0, 0, 0))
    for r in L['ramps']:
        cx, cy = (v * MM for v in r['centre'])
        rot_rect(cx, cy, r['length'] * MM, r['width'] * MM, r.get('yaw', 0), r['surface'])
        d.text(P(cx - 0.3, cy + 0.05), r['name'], fill=(0, 0, 0))
    sx, sy, _ = L['start']['pose']
    px, py = L['start']['pad_size']
    d.rectangle([P((sx - px / 2) * MM, (sy + py / 2) * MM), P((sx + px / 2) * MM, (sy - py / 2) * MM)], fill=rgb('red'))
    d.text(P(sx * MM - 0.2, sy * MM + 0.3), 'START', fill=(120, 0, 0))
    ls = L['loading_station']
    lx, ly = ls['centre']
    lsx, lsy = ls['size']
    d.rectangle([P((lx - lsx / 2) * MM, (ly + lsy / 2) * MM), P((lx + lsx / 2) * MM, (ly - lsy / 2) * MM)], fill=rgb('green'))
    d.text(P(lx * MM + 0.12, ly * MM), 'LOADING', fill=(0, 80, 0))
    for (x1, y1, x2, y2, _) in segment_boxes():
        d.line([P(x1, y1), P(x2, y2)], fill=rgb('wall'), width=max(3, int(WT * S)))
    g = L['gate']
    if not g.get('hide'):
        d.line([P(*(v * MM for v in g['from'])), P(*(v * MM for v in g['to']))], fill=rgb('gate'), width=6)
        gx = (g['from'][0] + g['to'][0]) / 2 * MM
        d.text(P(gx - 0.2, g['from'][1] * MM - 0.08), 'GATE', fill=(120, 90, 0))
    c = L['cylinders']
    for x, y in c['positions']:
        r = c['diameter'] * MM / 2
        d.ellipse([P(x * MM - r, y * MM + r), P(x * MM + r, y * MM - r)], fill=rgb('cyl'), outline=(0, 0, 0))
    ds = L['display_screen']
    if not ds.get('hide'):
      d.rectangle([P((ds['centre'][0] - ds['size'][0] / 2) * MM, ds['centre'][1] * MM + 0.03),
                 P((ds['centre'][0] + ds['size'][0] / 2) * MM, ds['centre'][1] * MM - 0.03)], fill=rgb('screen'))
      d.text(P(ds['centre'][0] * MM - 0.25, ds['centre'][1] * MM - 0.08), 'SCREEN', fill=(0, 0, 0))
    for gxm in range(0, int(FL) + 1):
        d.text(P(gxm, 0), f'{gxm}m', fill=(60, 60, 60))
    for gym in range(1, int(FW) + 1):
        d.text((4, P(0, gym)[1] - 5), f'{gym}m', fill=(60, 60, 60))
    img.save(os.path.join(PKG, 'arena', 'preview.png'))


if __name__ == '__main__':
    os.makedirs(os.path.join(PKG, 'worlds'), exist_ok=True)
    with open(os.path.join(PKG, 'worlds', 'rdj2026.sdf'), 'w') as f:
        f.write(build_world())
    w, h = build_map()
    build_preview()
    print(f'world: worlds/rdj2026.sdf  map: maps/rdj2026_truth.pgm ({w}x{h} @ {RES} m)  preview: arena/preview.png')

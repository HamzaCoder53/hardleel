#!/usr/bin/env python3
"""Draw a labelled 2D top-view plan of the RDJ 2026 field from arena_layout.yaml.

Writes arena/plan_2d.svg (vector, for printing / posters) and arena/plan_2d.png (if Pillow is installed).
Solid = rulebook or photo-confirmed, dashed = still a guess (not built when photographed).

Usage:  python3 arena/draw_plan.py
"""
import math
import os

import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
L = yaml.safe_load(open(os.path.join(PKG, 'arena', 'arena_layout.yaml')))
FL, FW = L['field']['length'], L['field']['width']           # mm

S = 0.16                     # px per mm in the output (6.7 m -> ~1070 px)
M = 90                       # margin px
W_PX, H_PX = int(FL * S + 2 * M), int(FW * S + 2 * M + 130)

COL = {'wall': '#8a5a2b', 'floor': '#f3e6cf', 'grid': '#d9c9ad', 'dim': '#1f4fbf', 'blue': '#1f5fd6',
       'white': '#ffffff', 'black': '#1a1a1a', 'soil': '#4a3a2a', 'wood': '#c9a06a', 'grass': '#3a9a3a',
       'sand': '#dcc28a', 'gravel': '#8c8c8c', 'gate': '#f0b400', 'loading': '#3aa04a', 'start': '#d62020',
       'screen': '#222233', 'cyl': '#9aa0aa', 'text': '#222222', 'guess': '#b03030'}


def P(x, y):                                                  # field mm -> image px (y up)
    return M + x * S, M + (FW - y) * S


shapes = []                                                   # (kind, attrs) rendered to SVG and PNG


def rect(x0, y0, x1, y1, fill, stroke='none', dashed=False, sw=1.0, opacity=1.0):
    shapes.append(('rect', dict(a=P(min(x0, x1), max(y0, y1)), b=P(max(x0, x1), min(y0, y1)),
                               fill=fill, stroke=stroke, dashed=dashed, sw=sw, op=opacity)))


def line(x0, y0, x1, y1, stroke, sw=2.0, dashed=False):
    shapes.append(('line', dict(a=P(x0, y0), b=P(x1, y1), stroke=stroke, sw=sw, dashed=dashed)))


def circle(x, y, r, fill, stroke='#000', dashed=False):
    shapes.append(('circle', dict(c=P(x, y), r=r * S, fill=fill, stroke=stroke, dashed=dashed)))


def text(x, y, s, size=13, color=COL['text'], anchor='middle', px=False, bold=False):
    shapes.append(('text', dict(p=(x, y) if px else P(x, y), s=s, size=size, color=color, anchor=anchor, bold=bold)))


def poly(pts, fill, stroke='none', dashed=False):
    shapes.append(('poly', dict(pts=[P(*p) for p in pts], fill=fill, stroke=stroke, dashed=dashed)))


def dim(x0, y0, x1, y1, label, offset_px=0, horizontal=True):
    """Dimension line with end ticks and a label (offset in px, perpendicular to the line)."""
    (ax, ay), (bx, by) = P(x0, y0), P(x1, y1)
    if horizontal:
        ay += offset_px; by += offset_px
    else:
        ax += offset_px; bx += offset_px
    shapes.append(('line', dict(a=(ax, ay), b=(bx, by), stroke=COL['dim'], sw=1.2, dashed=False)))
    for (cx, cy) in ((ax, ay), (bx, by)):
        if horizontal:
            shapes.append(('line', dict(a=(cx, cy - 6), b=(cx, cy + 6), stroke=COL['dim'], sw=1.2, dashed=False)))
        else:
            shapes.append(('line', dict(a=(cx - 6, cy), b=(cx + 6, cy), stroke=COL['dim'], sw=1.2, dashed=False)))
    mx, my = (ax + bx) / 2, (ay + by) / 2
    if horizontal:
        text(mx, my - 5, label, 12, COL['dim'], px=True)
    else:
        text(mx - 6, my + 4, label, 12, COL['dim'], anchor='end', px=True)


# ------------------------------------------------------------------ floor, grid
rect(0, 0, FL, FW, COL['floor'])
for gx in range(1000, int(FL), 1000):
    line(gx, 0, gx, FW, COL['grid'], 0.8)
for gy in range(1000, int(FW), 1000):
    line(0, gy, FL, gy, COL['grid'], 0.8)

# ------------------------------------------------------------------ zones
bays = L['offloading_bays']
xe = bays['x_edges']
for i, c in enumerate(bays['colours']):
    rect(xe[i], FW, xe[i + 1], FW - bays['depth'], COL[c], '#555', dashed=False)
    waste = {'blue': 'PLASTIC', 'white': 'ORGANIC', 'black': 'METAL'}.get(c, c.upper())
    text((xe[i] + xe[i + 1]) / 2, FW - bays['depth'] / 2 - 60, waste, 9,
         '#ffffff' if c in ('blue', 'black') else '#222', bold=True)
deck = bays.get('deck')
if deck:
    for i in range(len(xe) - 1):                              # V-notch markers on the deck front edge
        c = (xe[i] + xe[i + 1]) / 2; y = FW - bays['depth']
        poly([(c - deck['notch_width'] / 2, y), (c + deck['notch_width'] / 2, y), (c, y + deck['notch_depth'])],
             'none', '#ffcc00')

for r in L['ramps']:
    cx, cy = r['centre']; ln, wd = r['length'], r['width']; yaw = math.radians(r.get('yaw', 0))
    corners = [(-ln / 2, -wd / 2), (ln / 2, -wd / 2), (ln / 2, wd / 2), (-ln / 2, wd / 2)]
    pts = [(cx + u * math.cos(yaw) - v * math.sin(yaw), cy + u * math.sin(yaw) + v * math.cos(yaw)) for u, v in corners]
    poly(pts, COL.get(r['surface'], '#aaa'), '#333', dashed=True)
    text(cx, cy + 15, f"RAMP ({r['surface']}, {r['height']} mm)", 11, '#fff' if r['surface'] == 'soil' else '#222')
for p_ in L.get('patches', []):
    cx, cy = p_['centre']; sx, sy = p_['size']
    rect(cx - sx / 2, cy - sy / 2, cx + sx / 2, cy + sy / 2, COL.get(p_['surface'], '#aaa'), '#333', dashed=True)
    text(cx, cy, p_.get('label', p_['surface']).upper(), 11, '#fff', bold=True)

ls = L['loading_station']
if not L['loading_station'].get('hide'):
    lx, ly = ls['centre']; lsx, lsy = ls['size']
    rect(lx - lsx / 2, ly - lsy / 2, lx + lsx / 2, ly + lsy / 2, COL['loading'], '#1a5a1a', dashed=True)
    for i, slot in enumerate(ls['slots']):
        sy_ = ly - lsy / 2 + lsy * (i + 0.5) / len(ls['slots'])
        text(lx + lsx / 2 + 25, sy_ - 5, slot, 10, '#1a5a1a', anchor='start')
    text(lx, ly - lsy / 2 - 90, 'LOADING STATION', 12, '#1a5a1a', bold=True)

if not L['start'].get('hide'):
    sx, sy, syaw = L['start']['pose']
    px_, py_ = L['start']['pad_size']
    rect(sx - px_ / 2, sy - py_ / 2, sx + px_ / 2, sy + py_ / 2, COL['start'], '#700', dashed=True)
    a = math.radians(syaw)
    line(sx, sy, sx + 260 * math.cos(a), sy + 260 * math.sin(a), '#fff', 3)
    text(sx, sy - py_ / 2 - 100, 'START', 12, COL['start'], bold=True)

ds = L['display_screen']
if not L['display_screen'].get('hide'):
    dx, dy = ds['centre']; dw = ds['size'][0]
    rect(dx - dw / 2, dy - 25, dx + dw / 2, dy + 25, COL['screen'], dashed=True)
    text(dx, dy - 110, 'SCREEN', 12, COL['screen'], bold=True)

c = L['cylinders']
for x, y in c['positions']:
    circle(x, y, c['diameter'] / 2, COL['cyl'], '#333', dashed=True)

# ------------------------------------------------------------------ walls (solid = rulebook / photo)
T = L['field']['wall_thickness'] * S
for x0, y0, x1, y1 in ([0, 0, FL, 0], [FL, 0, FL, FW], [FL, FW, 0, FW], [0, FW, 0, 0]):
    line(x0, y0, x1, y1, COL['wall'], 5)
for x0, y0, x1, y1 in L['walls']:
    line(x0, y0, x1, y1, COL['wall'], max(3, T))
for x in xe:
    line(x, FW, x, FW - bays['depth'], COL['wall'], max(3, T))
for x0, y0, x1, y1 in L.get('baffles', {}).get('boards', []):
    line(x0, y0, x1, y1, COL['wall'], max(3, T))
g = L['gate']
if not L['gate'].get('hide'):
    line(*g['from'], *g['to'], COL['gate'], 6, dashed=True)
    text((g['from'][0] + g['to'][0]) / 2, g['from'][1] - 120, 'GATE', 12, '#9a7000', bold=True)


# ------------------------------------------------------------------ dimensions
dim(0, 0, FL, 0, f'{FL:.0f} mm', offset_px=40)
dim(FL, 0, FL, FW, f'{FW:.0f}', offset_px=60, horizontal=False)
# extra dimension lines come from the layout: plan_dims: [[x0, y0, x1, y1, offset_px], ...] (label = length)
for x0, y0, x1, y1, off in L.get('plan_dims', []):
    horiz = abs(y1 - y0) < abs(x1 - x0)
    dim(x0, y0, x1, y1, f'{math.hypot(x1 - x0, y1 - y0):.0f}', offset_px=off, horizontal=horiz)
if len(xe) > 1:
    dim(xe[0], FW - bays['depth'] - 150, xe[-1], FW - bays['depth'] - 150, f'{xe[-1] - xe[0]:.0f}', offset_px=0)
for gx in range(0, int(FL) + 1, 1000):
    text(*P(gx, 0), f'{gx // 1000} m', 10, '#8a7a5a', px=True)

# ------------------------------------------------------------------ title + legend
ty = M + FW * S + 75
text(M, ty, 'Robotics Dojo 2026 game field - 2D plan (from arena_layout.yaml)', 16, '#111', anchor='start', px=True, bold=True)
text(M, ty + 22, f"Walls {L['field']['wall_height']} mm high, {L['field']['wall_thickness']} mm plywood. "
     "Solid = rulebook / photo-confirmed.  Dashed = guess (not built when photographed, 15 Oct 2026).",
     12, '#333', anchor='start', px=True)
text(M, ty + 42, 'Dimensions in mm. Origin = bottom-left corner. Same layout as the Gazebo world and maps/rdj2026_truth.yaml.',
     12, '#333', anchor='start', px=True)


# ------------------------------------------------------------------ SVG
def svg():
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W_PX}" height="{H_PX}" viewBox="0 0 {W_PX} {H_PX}" '
           f'font-family="Arial, Helvetica, sans-serif">', f'<rect width="{W_PX}" height="{H_PX}" fill="#ffffff"/>']
    for kind, a in shapes:
        dash = ' stroke-dasharray="7 5"' if a.get('dashed') else ''
        if kind == 'rect':
            (x0, y0), (x1, y1) = a['a'], a['b']
            out.append(f'<rect x="{x0:.1f}" y="{y0:.1f}" width="{x1 - x0:.1f}" height="{y1 - y0:.1f}" fill="{a["fill"]}" '
                       f'stroke="{a["stroke"]}" stroke-width="{a["sw"]}"{dash}/>')
        elif kind == 'line':
            (x0, y0), (x1, y1) = a['a'], a['b']
            out.append(f'<line x1="{x0:.1f}" y1="{y0:.1f}" x2="{x1:.1f}" y2="{y1:.1f}" stroke="{a["stroke"]}" '
                       f'stroke-width="{a["sw"]:.1f}" stroke-linecap="square"{dash}/>')
        elif kind == 'circle':
            out.append(f'<circle cx="{a["c"][0]:.1f}" cy="{a["c"][1]:.1f}" r="{max(a["r"], 4):.1f}" fill="{a["fill"]}" '
                       f'stroke="{a["stroke"]}"{dash}/>')
        elif kind == 'poly':
            pts = ' '.join(f'{x:.1f},{y:.1f}' for x, y in a['pts'])
            out.append(f'<polygon points="{pts}" fill="{a["fill"]}" stroke="{a["stroke"]}" stroke-width="1.5"{dash}/>')
        elif kind == 'text':
            anchor = {'middle': 'middle', 'start': 'start', 'end': 'end'}[a['anchor']]
            weight = ' font-weight="bold"' if a['bold'] else ''
            out.append(f'<text x="{a["p"][0]:.1f}" y="{a["p"][1]:.1f}" font-size="{a["size"]}" fill="{a["color"]}" '
                       f'text-anchor="{anchor}"{weight}>{a["s"]}</text>')
    out.append('</svg>')
    return '\n'.join(out)


# ------------------------------------------------------------------ PNG (Pillow)
def png(path):
    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError:
        print('Pillow not installed - skipping PNG')
        return
    k = 2                                                     # render at 2x for crisp text
    img = Image.new('RGB', (W_PX * k, H_PX * k), '#ffffff')
    d = ImageDraw.Draw(img)

    def font(size, bold):
        for name in (('arialbd.ttf', 'DejaVuSans-Bold.ttf') if bold else ('arial.ttf', 'DejaVuSans.ttf')):
            try:
                return ImageFont.truetype(name, size * k)
            except OSError:
                pass
        return ImageFont.load_default()

    def dashed(p0, p1, color, w):
        (x0, y0), (x1, y1) = p0, p1
        L_ = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(L_ / (12 * k)))
        for i in range(n):
            if i % 2 == 0:
                t0, t1 = i / n, min(1, (i + 1) / n)
                d.line([(x0 + (x1 - x0) * t0, y0 + (y1 - y0) * t0), (x0 + (x1 - x0) * t1, y0 + (y1 - y0) * t1)], fill=color, width=w)

    sc = lambda p: (p[0] * k, p[1] * k)
    for kind, a in shapes:
        if kind == 'rect':
            (x0, y0), (x1, y1) = sc(a['a']), sc(a['b'])
            if a['fill'] != 'none':
                d.rectangle([x0, y0, x1, y1], fill=a['fill'])
            if a['stroke'] != 'none':
                for p0, p1 in (((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))):
                    (dashed if a['dashed'] else lambda p, q, c, w: d.line([p, q], fill=c, width=w))(p0, p1, a['stroke'], 2 * k)
        elif kind == 'line':
            w = max(1, int(a['sw'] * k))
            if a['dashed']:
                dashed(sc(a['a']), sc(a['b']), a['stroke'], w)
            else:
                d.line([sc(a['a']), sc(a['b'])], fill=a['stroke'], width=w)
        elif kind == 'circle':
            (cx, cy), r = sc(a['c']), max(a['r'], 4) * k
            d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=a['fill'], outline=a['stroke'], width=k)
        elif kind == 'poly':
            pts = [sc(p) for p in a['pts']]
            if a['fill'] != 'none':
                d.polygon(pts, fill=a['fill'])
            for p0, p1 in zip(pts, pts[1:] + pts[:1]):
                (dashed if a['dashed'] else lambda p, q, c, w: d.line([p, q], fill=c, width=w))(p0, p1, a['stroke'], 2 * k)
        elif kind == 'text':
            f = font(a['size'], a['bold'])
            x, y = sc(a['p'])
            tw = d.textlength(a['s'], font=f)
            x = x - tw / 2 if a['anchor'] == 'middle' else x - tw if a['anchor'] == 'end' else x
            d.text((x, y - a['size'] * k), a['s'], fill=a['color'], font=f)
    img.save(path)


if __name__ == '__main__':
    with open(os.path.join(PKG, 'arena', 'plan_2d.svg'), 'w') as f:
        f.write(svg())
    png(os.path.join(PKG, 'arena', 'plan_2d.png'))
    print(f'wrote arena/plan_2d.svg and arena/plan_2d.png ({W_PX} x {H_PX} px, 1 m = {1000 * S:.0f} px)')

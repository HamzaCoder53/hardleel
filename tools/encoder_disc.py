#!/usr/bin/env python3
"""Printable DIY encoder disc (SVG, true size - print at 100 % / "actual size", no "fit to page").

    python3 tools/encoder_disc.py --outer 50 --inner 20 --hole 6 --stripes 24 -o disc.svg

--stripes N black + N white sectors -> 2N counts per wheel turn (sketch counts both edges),
so ticks_per_rev = 2 * N. Glue it flat on the inside face of the wheel hub, centred on the
axle, with a TCRT5000 module 2-4 mm in front of the ring. A 4-up sheet (one disc per wheel)
is made with --sheet.
"""
import argparse
import math


def disc(cx, cy, outer, inner, hole, stripes):
    parts = [f'<circle cx="{cx}" cy="{cy}" r="{outer / 2}" fill="white" stroke="black" stroke-width="0.2"/>']
    ro, ri = outer / 2, inner / 2
    step = 2 * math.pi / (2 * stripes)
    for k in range(stripes):
        a0, a1 = 2 * k * step, (2 * k + 1) * step
        p = [(cx + r * math.cos(a), cy + r * math.sin(a)) for r, a in ((ri, a0), (ro, a0), (ro, a1), (ri, a1))]
        parts.append(
            f'<path d="M{p[0][0]:.3f},{p[0][1]:.3f} L{p[1][0]:.3f},{p[1][1]:.3f} '
            f'A{ro},{ro} 0 0 1 {p[2][0]:.3f},{p[2][1]:.3f} L{p[3][0]:.3f},{p[3][1]:.3f} '
            f'A{ri},{ri} 0 0 0 {p[0][0]:.3f},{p[0][1]:.3f} Z" fill="black"/>')
    parts.append(f'<circle cx="{cx}" cy="{cy}" r="{hole / 2}" fill="none" stroke="black" stroke-width="0.2"/>')
    parts.append(f'<line x1="{cx - 1}" y1="{cy}" x2="{cx + 1}" y2="{cy}" stroke="black" stroke-width="0.1"/>')
    parts.append(f'<line x1="{cx}" y1="{cy - 1}" x2="{cx}" y2="{cy + 1}" stroke="black" stroke-width="0.1"/>')
    return parts


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--outer', type=float, default=50.0, help='disc diameter mm')
    ap.add_argument('--inner', type=float, default=20.0, help='stripe ring inner diameter mm')
    ap.add_argument('--hole', type=float, default=6.0, help='centre hole diameter mm (cut-out guide)')
    ap.add_argument('--stripes', type=int, default=24, help='black stripes (counts/turn = 2x)')
    ap.add_argument('--sheet', action='store_true', help='4 discs on one page (one per wheel)')
    ap.add_argument('-o', '--out', default='encoder_disc.svg')
    a = ap.parse_args()
    n = 4 if a.sheet else 1
    gap = 10.0
    w = a.outer * (2 if a.sheet else 1) + gap * (3 if a.sheet else 2)
    h = a.outer * (2 if a.sheet else 1) + gap * (3 if a.sheet else 2) + 8
    body = []
    for i in range(n):
        cx = gap + a.outer / 2 + (i % 2) * (a.outer + gap)
        cy = gap + a.outer / 2 + (i // 2) * (a.outer + gap)
        body += disc(cx, cy, a.outer, a.inner, a.hole, a.stripes)
    body.append(f'<text x="{gap}" y="{h - 3}" font-size="3" font-family="sans-serif">'
                f'{a.stripes} stripes = ticks_per_rev {2 * a.stripes} | outer {a.outer} mm | '
                f'print at 100 % - check the disc measures {a.outer} mm</text>')
    svg = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}mm" height="{h}mm" '
           f'viewBox="0 0 {w} {h}">\n' + '\n'.join(body) + '\n</svg>\n')
    with open(a.out, 'w') as f:
        f.write(svg)
    print(f'wrote {a.out}: {n} disc(s), {a.stripes} stripes -> ticks_per_rev {2 * a.stripes}')


if __name__ == '__main__':
    main()

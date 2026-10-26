#!/usr/bin/env python3
"""Markdown results tables from docs/sim_demo/ (accuracy json, nav json, rehearsal event files).   summarize.py > RESULTS.md"""
import glob
import json
import os
import re

D = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../docs/sim_demo')
j = lambda n: json.load(open(os.path.join(D, n))) if os.path.exists(os.path.join(D, n)) else None

print('### Map accuracy vs ground truth\n')
print('| map | wall error mean (raw) | p95 | after best rigid fit (mean / p95, shift) | true walls covered <10 cm | free-space box (map vs truth) |')
print('|---|---|---|---|---|---|')
for n, label in (('mapping_course_accuracy.json', 'course'), ('mapping_maze_accuracy.json', 'maze (unrelated world)')):
    d = j(n)
    if d:
        r, a, s, al = d['wall_distance_raw'], d['wall_distance_aligned'], d['size'], d['alignment']
        print(f"| {label} | {r['mean_m'] * 100:.1f} cm | {r['p95_m'] * 100:.1f} cm | {a['mean_m'] * 100:.1f} / {a['p95_m'] * 100:.1f} cm "
              f"({al['dx_m'] * 100:.1f}, {al['dy_m'] * 100:.1f} cm, {al['yaw_deg']:.1f} deg) | {d['coverage_of_true_walls_within_10cm'] * 100:.1f} % | "
              f"{s['mapped_free_bbox_m'][0]:.2f} x {s['mapped_free_bbox_m'][1]:.2f} vs {s['truth_free_bbox_m'][0]:.2f} x {s['truth_free_bbox_m'][1]:.2f} m |")

print('\n### Nav2 goals on the saved map (truth-scored)\n')
print('| run | goals reached | mean loc. error at goal | worst loc. error | standing-still recovery |')
print('|---|---|---|---|---|')
for n, label in (('nav_course.json', 'course, 8 goals'), ('nav_maze.json', 'maze, 6 goals'),
                 ('recovery_baseline.json', 'course + teleport 5 m, default stack'),
                 ('recovery_v2_apply.json', 'course + teleport 5 m, loc_v2 + apply_corrections')):
    d = j(n)
    if not d:
        continue
    rs = [r for r in d['results'] if not r['goal'].startswith('_')]
    rec = [r for r in d['results'] if r['goal'].startswith('_')]
    ok = sum(r['status'] == 'SUCCEEDED' for r in rs)
    mean = sum((r['loc_error_end_m'] or 0) for r in rs) / max(len(rs), 1) * 100
    worst = max((r['loc_error_worst_m'] or 0) for r in rs) * 100
    rtxt = '-' if not rec else (f"{rec[0]['seconds']} s" if rec[0]['seconds'] is not None else 'not recovered in 90 s')
    print(f'| {label} | {ok}/{len(rs)} | {mean:.1f} cm | {worst:.0f} cm | {rtxt} |')

print('\n### Mission rehearsal (real mission node, stub classifier, phase 2 on the saved map)\n')
print('| case | gate seen (median ahead) | decision | waited | mission finished at |')
print('|---|---|---|---|---|')
for f in sorted(glob.glob(os.path.join(D, 'rehearsal_*_mission_events.txt'))):
    name = os.path.basename(f).replace('rehearsal_', '').replace('_mission_events.txt', '')
    t = open(f).read()
    m = re.search(r'gate_decision: event \{"decision": "(\w+)"; "wait_s": ([\d.]+).*?"last_median_m": ([\d.]+)', t)
    e = re.search(r'\[ *([\d.]+)s\] DONE', t)
    print(f"| {name} | {m.group(3) + ' m' if m else '-'} | {m.group(1) if m else '-'} | {m.group(2) + ' s' if m else '-'} | {e.group(1) + ' s' if e else 'did not finish'} |")

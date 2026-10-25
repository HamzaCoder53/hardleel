"""Waypoint file I/O (pure python, no ROS). Format: see PROGRESS.md "Interfaces".

    poses, seqs, meta = load_waypoints(path)   # poses: OrderedDict name -> (x_m, y_m, yaw_rad)
    problems = check_waypoints(poses, seqs)    # [] when OK
    save_waypoints(path, poses, seqs, meta)    # keeps order + header comment, writes path.bak first
    set_pose(path, 'screen', x, y, yaw_rad)    # update / insert one pose
"""
import json
import math
import os
import shutil
import time
from collections import OrderedDict

import yaml

REQUIRED = ('screen', 'loading_plastic', 'loading_metal', 'loading_organic', 'junction', 'gate_check',
            'bin_plastic', 'bin_organic', 'bin_metal', 'finish')
REQUIRED_SEQUENCES = ('terrain', 'path1', 'path2')
DEFAULT_HEADER = '# Waypoints in the map frame (origin = start pose, x forward, y left), yaw in degrees.\n'


class _OrderedLoader(yaml.SafeLoader):
    pass


def _construct_map(loader, node):
    loader.flatten_mapping(node)
    return OrderedDict(loader.construct_pairs(node))


_OrderedLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_map)


def _read(path):
    with open(path, encoding='utf-8') as f:
        text = f.read()
    return text, (yaml.load(text, Loader=_OrderedLoader) or {})


def _header(text):
    """Leading comment/blank lines of the file."""
    out = []
    for line in text.splitlines(True):
        if line.strip() == '' or line.lstrip().startswith('#'):
            out.append(line)
        else:
            break
    return ''.join(out)


def load_waypoints(path):
    """-> (poses OrderedDict name->(x, y, yaw_rad), sequences dict name->list, meta dict name->{conf, note, ...})."""
    _, d = _read(path)
    poses, meta = OrderedDict(), {}
    for name, p in (d.get('poses') or {}).items():
        poses[str(name)] = (float(p['x']), float(p['y']), math.radians(float(p.get('yaw_deg', 0.0))))
        meta[str(name)] = {k: v for k, v in p.items() if k not in ('x', 'y', 'yaw_deg')}
    seqs = OrderedDict((str(k), [str(n) for n in (v or [])]) for k, v in (d.get('sequences') or {}).items())
    return poses, dict(seqs), meta


def check_waypoints(poses, sequences, required=REQUIRED):
    """List of human-readable problems (empty list = valid)."""
    bad = []
    for n in required:
        if n not in poses:
            bad.append('missing pose: %s' % n)
    for s in REQUIRED_SEQUENCES:
        if s not in sequences:
            bad.append('missing sequence: %s' % s)
    for s, names in sequences.items():
        if not names:
            bad.append('empty sequence: %s' % s)
        for n in names:
            if n not in poses:
                bad.append('sequence %s references undefined pose %s' % (s, n))
    for n, (x, y, yaw) in poses.items():
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            bad.append('non-finite pose: %s' % n)
    return bad


def _scalar(v):
    return json.dumps(v, ensure_ascii=False)   # JSON scalars are valid YAML



def save_waypoints(path, poses, sequences, meta=None, header=None):
    """Write the file (poses in dict order). Existing file -> path.bak and its header comment is kept."""
    meta = meta or {}
    old_header = None
    if os.path.exists(path):
        with open(path, encoding='utf-8') as f:
            old_header = _header(f.read())
        shutil.copyfile(path, path + '.bak')
    head = header if header is not None else (old_header if old_header else DEFAULT_HEADER)
    if head and not head.endswith('\n'):
        head += '\n'
    lines = [head.rstrip('\n') + '\n' if head.strip() else '', 'frame: map\n', 'poses:\n']
    for n, (x, y, yaw) in poses.items():
        parts = ['x: %.3f' % x, 'y: %.3f' % y, 'yaw_deg: %.1f' % round(math.degrees(yaw), 1)]
        m = dict(meta.get(n, {}))
        for k in ('conf', 'note'):
            if k in m:
                parts.append('%s: %s' % (k, _scalar(m.pop(k))))
        parts += ['%s: %s' % (k, _scalar(v)) for k, v in m.items()]
        lines.append('  %s: {%s}\n' % (n, ', '.join(parts)))
    lines.append('sequences:\n')
    for s, names in sequences.items():
        lines.append('  %s: [%s]\n' % (s, ', '.join(names)))
    d = os.path.dirname(os.path.abspath(path))
    os.makedirs(d, exist_ok=True)
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        f.write(''.join(lines))
    os.replace(tmp, path)


def set_pose(path, name, x, y, yaw_rad, conf='high', note=None):
    """Update (or append) one pose in an existing file; other poses/sequences/header are untouched."""
    poses, seqs, meta = load_waypoints(path)
    poses[name] = (float(x), float(y), float(yaw_rad))
    m = meta.setdefault(name, {})
    m['conf'] = conf
    m['note'] = note if note is not None else 'recorded %s' % time.strftime('%Y-%m-%d %H:%M')
    save_waypoints(path, poses, seqs, meta)
    return poses[name]

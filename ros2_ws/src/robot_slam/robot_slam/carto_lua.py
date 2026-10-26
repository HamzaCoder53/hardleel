"""macOS workaround: the RoboStack Cartographer (Lua 5.4) does not export globals from `include "x.lua"`, so
cartographer_node dies with "attempt to index a nil value (global 'MAP_BUILDER')". On macOS we inline every
include into one file at launch time. The config files in the repo are NOT changed (the robot uses them as is)."""
import os
import re
import sys
import tempfile

_INCLUDE = re.compile(r'^\s*include\s+"([^"]+)"\s*$')
_RETURN = re.compile(r'^\s*return\s+\w+\s*$')


def _inline(name, dirs, seen, root=False):
    for d in dirs:
        path = os.path.join(d, name)
        if os.path.exists(path):
            break
    else:
        raise FileNotFoundError(name)
    out = []
    for line in open(path):
        m = _INCLUDE.match(line)
        if m:
            if m.group(1) not in seen:       # each file once, like cartographer's own include
                seen.add(m.group(1))
                out.append(_inline(m.group(1), dirs, seen))
        elif root or not _RETURN.match(line):
            out.append(line)
    return ''.join(out)


def config_for_platform(config_dir, basename):
    """(dir, basename) to pass to cartographer_node: unchanged on Linux, flattened copy on macOS."""
    if sys.platform != 'darwin':
        return config_dir, basename
    prefix = os.environ.get('CONDA_PREFIX', '')
    dirs = [config_dir, os.path.join(prefix, 'share', 'cartographer', 'configuration_files'),
            os.path.join(prefix, 'share', 'cartographer_ros', 'configuration_files')]
    tmp = tempfile.mkdtemp(prefix='carto_flat_')
    text = _inline(basename, dirs, {basename}, root=True)
    with open(os.path.join(tmp, basename), 'w') as f:
        f.write(text)
    return tmp, basename

"""Saved tuning values for the hardleel robot (used by the launch files, tuning_gui and tuning_helper).

Nothing in here changes unless the tuning GUI's "Save" is pressed. Live changes made in the GUI are
ROS parameter sets: they live only inside the running nodes, so every new run starts from the defaults
plus whatever was saved.

  ~/.hardleel/saved_params.yaml   ROS 2 params file, nested by namespace/node with ros__parameters at the
                                  leaf; the launch files pass it to their nodes (each node takes its own part)
  ~/.hardleel/saved_carto.yaml    {lua_key: value} Cartographer overrides (Cartographer reads lua only at start)
  ~/.hardleel/session_carto.yaml  GUI "Apply" for Cartographer, THIS RUN ONLY (start_robot.sh deletes it)
"""
import copy
import os
import re

import yaml

DIR = os.path.expanduser('~/.hardleel')
SAVED_PARAMS = os.path.join(DIR, 'saved_params.yaml')
SAVED_CARTO = os.path.join(DIR, 'saved_carto.yaml')
SESSION_CARTO = os.path.join(DIR, 'session_carto.yaml')
CARTO_DIR = os.path.join(DIR, 'carto')          # effective lua written here at Cartographer start
NAV2_EFFECTIVE = os.path.join(DIR, 'nav2_params_effective.yaml')

# Lua values the GUI may write: numbers, true/false, math.rad(<number>)
LUA_VALUE = re.compile(r'^\s*(-?\d+(\.\d*)?([eE]-?\d+)?|-?\.\d+([eE]-?\d+)?|true|false|math\.rad\(\s*-?\d+(\.\d*)?\s*\))\s*$')


def load(path):
    try:
        with open(path) as f:
            return yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError):
        return {}


def write(path, data):
    """Write YAML atomically; an empty dict removes the file (= back to the defaults)."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    if not data:
        if os.path.exists(path):
            os.remove(path)
        return
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        yaml.safe_dump(data, f, default_flow_style=False, sort_keys=True)
    os.replace(tmp, path)


def saved_params_file():
    """Path of the saved params file if it holds anything, else None (for launch files)."""
    return SAVED_PARAMS if load(SAVED_PARAMS) else None


def with_saved(*params):
    """Node(parameters=with_saved({...}, 'file.yaml')) -> the given params plus the saved file (saved wins)."""
    f = saved_params_file()
    return list(params) + ([f] if f else [])


# ---- params-file <-> flat {'/ns/node': {param: value}} -------------------------------------------

def flatten(tree, prefix=''):
    """Nested params file -> {'/ns/node': {param: value}}."""
    out = {}
    for key, val in (tree or {}).items():
        if not isinstance(val, dict):
            continue
        if key == 'ros__parameters':
            out[prefix or '/'] = _flat_params(val)
        else:
            out.update(flatten(val, f'{prefix}/{key.strip("/")}'))
    return out


def _flat_params(d, prefix=''):
    """{'a': {'b': 1}} -> {'a.b': 1} (ROS dotted parameter names)."""
    out = {}
    for k, v in d.items():
        name = f'{prefix}.{k}' if prefix else k
        if isinstance(v, dict):
            out.update(_flat_params(v, name))
        else:
            out[name] = v
    return out


def unflatten(flat):
    """{'/ns/node': {param: value}} -> nested params file (dotted names kept as written; rcl accepts them)."""
    tree = {}
    for node, params in flat.items():
        if not params:
            continue
        cur = tree
        for part in [p for p in node.strip('/').split('/') if p]:
            cur = cur.setdefault(part, {})
        cur.setdefault('ros__parameters', {}).update(params)
    return tree


def deep_merge(base, over):
    out = copy.deepcopy(base)
    for k, v in (over or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def _nest_dotted(tree):
    """Turn dotted parameter names under ros__parameters into nested dicts (Nav2 params files are nested)."""
    if not isinstance(tree, dict):
        return tree
    out = {}
    for k, v in tree.items():
        if k == 'ros__parameters' and isinstance(v, dict):
            nested = {}
            for name, val in v.items():
                cur = nested
                parts = name.split('.')
                for part in parts[:-1]:
                    cur = cur.setdefault(part, {})
                cur[parts[-1]] = val
            out[k] = nested
        else:
            out[k] = _nest_dotted(v)
    return out


def load_saved_flat():
    """Saved params as {'/ns/node': {param: value}}."""
    return flatten(load(SAVED_PARAMS))


def save_flat(flat):
    """Store {'/ns/node': {param: value}} as a nested ROS params file (empty -> file removed)."""
    write(SAVED_PARAMS, _nest_dotted(unflatten(flat)))


def merged_params_file(base_yaml, out_path=NAV2_EFFECTIVE):
    """Base params file (e.g. nav2_params.yaml) + saved values -> out_path. Returns base_yaml if nothing saved."""
    saved = load(SAVED_PARAMS)
    if not saved:
        return base_yaml
    merged = deep_merge(load(base_yaml), _nest_dotted(saved))
    write(out_path, merged)
    return out_path


# ---- Cartographer lua --------------------------------------------------------------------------

_OPT_LINE = re.compile(r'^\s+([a-z_][a-z0-9_]*)\s*=\s*([^,]+?)\s*,?\s*(--.*)?$')
_TOP_LINE = re.compile(r'^([A-Z_][A-Z0-9_]*(\.[a-z_][a-z0-9_]*)+)\s*=\s*(.+?)\s*(--.*)?$')


def lua_tunables(lua_text):
    """Tunable values in a Cartographer lua: {'options.lookup_transform_timeout_sec': '0.2',
    'TRAJECTORY_BUILDER_2D.min_range': '0.15', ...} (strings such as frame names are left out)."""
    out = {}
    in_options = False
    for line in lua_text.splitlines():
        if re.match(r'^\s*options\s*=\s*\{', line):
            in_options = True
            continue
        if in_options and line.strip().startswith('}'):
            in_options = False
            continue
        m = _OPT_LINE.match(line) if in_options else _TOP_LINE.match(line)
        if not m:
            continue
        key = f'options.{m.group(1)}' if in_options else m.group(1)
        val = (m.group(2) if in_options else m.group(3)).strip()
        if LUA_VALUE.match(val):
            out[key] = val
    return out


def carto_overrides():
    """Saved Cartographer overrides, then this run's session ones on top."""
    o = {k: str(v) for k, v in load(SAVED_CARTO).items()}
    o.update({k: str(v) for k, v in load(SESSION_CARTO).items()})
    return {k: v for k, v in o.items() if LUA_VALUE.match(v)}


def effective_carto_dir(default_dir, basename='cartographer_2d.lua'):
    """Write <basename> = default lua + overrides into CARTO_DIR (always, also without overrides) and return
    CARTO_DIR. Always the same path, so a respawned Cartographer (launch respawn) reads a rewritten file:
    that is how the tuning GUI's "Apply" works when Cartographer runs inside a launch file."""
    over = carto_overrides()
    with open(os.path.join(default_dir, basename)) as f:
        text = f.read()
    if over:
        block = '\n-- tuning overrides (~/.hardleel saved_carto.yaml / session_carto.yaml)\n' + \
            '\n'.join(f'{k} = {v}' for k, v in sorted(over.items())) + '\n'
        idx = text.rfind('return options')
        text = text[:idx] + block + '\n' + text[idx:] if idx >= 0 else text + block
    os.makedirs(CARTO_DIR, exist_ok=True)
    with open(os.path.join(CARTO_DIR, basename), 'w') as f:
        f.write(text)
    return CARTO_DIR

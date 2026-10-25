"""Persistent motor calibration store with sanity checks and a history log (pure Python + PyYAML)."""
import csv
import os
import tempfile
import time

import yaml

DEFAULT_PATH = '~/hardleel_ws/motor_calib.yaml'
KEYS = ('min_pwm_left', 'min_pwm_right', 'min_turn_pwm', 'max_speed')
BOUNDS = {'min_pwm_left': (0, 150), 'min_pwm_right': (0, 150),
          'min_turn_pwm': (0, 200), 'max_speed': (0.2, 3.0)}


def _atomic_write(path, text):
    d = os.path.dirname(path) or '.'
    os.makedirs(d, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=d, prefix='.calib_', suffix='.tmp')
    try:
        with os.fdopen(fd, 'w') as f:
            f.write(text)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    except BaseException:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise


class CalibStore:
    def __init__(self, path=DEFAULT_PATH, history_path=None, max_rel_change=0.5):
        self.path = os.path.expanduser(path)
        self.history_path = os.path.expanduser(
            history_path or os.path.join(os.path.dirname(self.path), 'motor_calib_history.csv'))
        self.max_rel_change = max_rel_change

    def _read(self):
        try:
            with open(self.path) as f:
                data = yaml.safe_load(f) or {}
        except (FileNotFoundError, yaml.YAMLError):
            data = {}
        return data if isinstance(data, dict) else {}

    def load(self):
        """Return {key: value} of the known keys present in the file."""
        d = self._read()
        return {k: d[k] for k in KEYS if isinstance(d.get(k), (int, float))}

    def get(self, key, default=None):
        return self.load().get(key, default)

    def meta(self):
        m = self._read().get('meta', {})
        return m if isinstance(m, dict) else {}

    def _log(self, key, value, accepted, reason, t):
        new = not os.path.exists(self.history_path)
        os.makedirs(os.path.dirname(self.history_path) or '.', exist_ok=True)
        with open(self.history_path, 'a', newline='') as f:
            w = csv.writer(f)
            if new:
                w.writerow(['time', 'key', 'value', 'accepted', 'reason'])
            w.writerow([time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime(t)), key, value,
                        int(accepted), reason])

    def check(self, key, value, force=False):
        """Return (ok, reason) without writing anything."""
        if key not in BOUNDS:
            return False, 'unknown key'
        try:
            value = float(value)
        except (TypeError, ValueError):
            return False, 'not a number'
        if value != value or value in (float('inf'), float('-inf')):
            return False, 'not finite'
        lo, hi = BOUNDS[key]
        if not lo <= value <= hi:
            return False, f'out of bounds [{lo}, {hi}]'
        last = self.get(key)
        if not force and last:
            rc = abs(value - last) / abs(last)
            if rc > self.max_rel_change:
                return False, (f'change {rc * 100:.0f}% from last good {last} exceeds '
                               f'{self.max_rel_change * 100:.0f}%')
        return True, 'ok'

    def accept(self, key, value, force=False, source='', now=None):
        """Sanity-check and store. Returns (accepted, reason). Rejections are only logged."""
        now = time.time() if now is None else now
        ok, reason = self.check(key, value, force)
        if ok and force and self.get(key) is not None:
            reason = 'forced'
        if source:
            reason = f'{reason} [{source}]'
        try:
            self._log(key, value, ok, reason, now)
        except OSError:
            pass
        if not ok:
            return False, reason
        data = self._read()
        data[key] = float(value)
        meta = data.get('meta') if isinstance(data.get('meta'), dict) else {}
        meta[key] = {'updated': time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime(now)),
                     'source': source, 'forced': bool(force)}
        data['meta'] = meta
        _atomic_write(self.path, yaml.safe_dump(data, sort_keys=True))
        return True, reason

    def effective(self, key, value, force=False, source=''):
        """accept(), then return (value_to_use_live, accepted, reason): the new value if
        accepted, else the last good one (or None)."""
        ok, reason = self.accept(key, value, force, source)
        return (float(value) if ok else self.get(key)), ok, reason

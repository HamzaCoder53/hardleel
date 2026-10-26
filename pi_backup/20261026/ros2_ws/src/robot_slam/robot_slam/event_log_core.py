"""Pure run-log logic for the event logger (no ROS): official event timestamps, derived times, JSON/CSV output.

Events (from /mission/event): autonomous_start, waste_detected, loading_complete, terrain_complete,
obstacle_detected, obstacle_response, gate_decision, deposit_complete, finish.
The FIRST occurrence of an event is the official one; every occurrence is kept in `occurrences`.
"""
import csv
import datetime
import json
import os

MISSING = 'MISSING'


def iso(wall_time):
    if wall_time is None:
        return MISSING
    return datetime.datetime.fromtimestamp(wall_time).isoformat(timespec='milliseconds')


def _num(x):
    try:
        v = float(x)
        return v if v == v and abs(v) != float('inf') else None
    except (TypeError, ValueError):
        return None


class RunLog:
    def __init__(self, robot_id, run_dir=''):
        self.robot_id = robot_id
        self.run_dir = run_dir
        self.run_id = ''
        self.occurrences = []          # every record, in arrival order
        self.first = {}                # event -> first occurrence dict

    # ---- recording
    def record(self, event, detail=None, wall_time=0.0, ros_time=None):
        """Store one occurrence; returns it (with t_rel and is_first)."""
        if not isinstance(detail, dict):
            detail = {} if detail is None else {'value': detail}
        occ = {'event': event, 'detail': detail, 'wall_time': wall_time, 'wall_iso': iso(wall_time),
               'ros_time': ros_time, 'is_first': event not in self.first}
        if occ['is_first']:
            self.first[event] = occ
        self.occurrences.append(occ)
        for o in self.occurrences:      # t_rel may become known late (events before autonomous_start)
            o['t_rel'] = self.t_rel(o['wall_time'])
        return occ

    def t_rel(self, wall_time):
        """Seconds since autonomous_start, None if not started yet."""
        s = self.first.get('autonomous_start')
        if s is None or wall_time is None:
            return None
        return round(wall_time - s['wall_time'], 3)

    # ---- derived fields
    def _t(self, event):
        o = self.first.get(event)
        return None if o is None else o['wall_time']

    def total_run_time_s(self):
        a, f = self._t('autonomous_start'), self._t('finish')
        return None if a is None or f is None else round(f - a, 3)

    def gate_wait_time_s(self):
        o = self.first.get('gate_decision')
        return None if o is None else _num(o['detail'].get('wait_s'))

    def obstacle_response_time_s(self):
        o = self.first.get('obstacle_response')
        if o is None:
            return None
        v = _num(o['detail'].get('response_s'))
        if v is not None:
            return v
        d = self._t('obstacle_detected')
        return None if d is None else round(o['wall_time'] - d, 3)

    # ---- summary
    def summary_rows(self):
        """Required fields in rulebook order: dicts {field, wall_iso, t_rel, value, extra}."""
        rows = [{'field': 'robot_id', 'wall_iso': '', 't_rel': None, 'value': self.robot_id, 'extra': ''}]

        def ev_row(name, extra_keys=()):
            o = self.first.get(name)
            if o is None:
                return {'field': name, 'wall_iso': MISSING, 't_rel': None, 'value': MISSING, 'extra': ''}
            ex = ' '.join('%s=%s' % (k, o['detail'][k]) for k in extra_keys if k in o['detail'])
            return {'field': name, 'wall_iso': o['wall_iso'], 't_rel': o['t_rel'], 'value': 'ok', 'extra': ex}

        def val_row(name, v):
            return {'field': name, 'wall_iso': '', 't_rel': None,
                    'value': MISSING if v is None else v, 'extra': ''}

        rows.append(ev_row('autonomous_start'))
        rows.append(ev_row('waste_detected', ('class', 'confidence')))
        rows.append(ev_row('loading_complete'))
        rows.append(ev_row('terrain_complete'))
        rows.append(ev_row('obstacle_detected'))
        rows.append(ev_row('obstacle_response', ('action', 'response_s')))
        rows.append(ev_row('gate_decision', ('decision',)))
        rows.append(val_row('gate_wait_time', self.gate_wait_time_s()))
        rows.append(ev_row('deposit_complete', ('bin', 'dropped')))
        rows.append(ev_row('finish'))
        rows.append(val_row('total_run_time', self.total_run_time_s()))
        return rows

    def to_dict(self):
        keys = ('event', 'detail', 'wall_iso', 'wall_time', 'ros_time', 't_rel', 'is_first')
        return {'robot_id': self.robot_id, 'run_id': self.run_id,
                'events': [{k: o[k] for k in keys} for o in self.occurrences],
                'required_summary': self.summary_rows(),
                'derived': {'total_run_time_s': self.total_run_time_s(),
                            'gate_wait_time_s': self.gate_wait_time_s(),
                            'obstacle_response_time_s': self.obstacle_response_time_s()}}

    # ---- output (atomic: temp file then os.replace)
    def write(self, json_path, csv_path):
        d = self.to_dict()
        _atomic(json_path, lambda f: json.dump(d, f, indent=2, default=str))

        def wr(f):
            w = csv.writer(f)
            w.writerow(['wall_iso', 'ros_time', 't_rel', 'event', 'detail_json'])
            for o in d['events']:
                w.writerow([o['wall_iso'], '' if o['ros_time'] is None else o['ros_time'],
                            '' if o['t_rel'] is None else o['t_rel'], o['event'],
                            json.dumps(o['detail'], default=str)])
        _atomic(csv_path, wr, newline='')


def _atomic(path, writer, newline=None):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    tmp = path + '.tmp'
    with open(tmp, 'w', newline=newline, encoding='utf-8') as f:
        writer(f)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def format_summary(log, state='', now=None):
    """Compact multi-line live table."""
    start = log.first.get('autonomous_start')
    el = '-' if start is None else '%.1fs' % ((now if now is not None else start['wall_time']) - start['wall_time'])
    lines = ['[event_logger] robot=%s state=%s elapsed=%s' % (log.robot_id, state or '-', el)]
    for r in log.summary_rows()[1:]:
        if r['value'] == MISSING:
            mark = '-'
        elif r['t_rel'] is not None:
            mark = 'OK +%.1fs' % r['t_rel']
        else:
            mark = 'OK' if r['value'] == 'ok' else str(r['value'])
        lines.append('  %-18s %s %s' % (r['field'], mark, r['extra']))
    return '\n'.join(lines)

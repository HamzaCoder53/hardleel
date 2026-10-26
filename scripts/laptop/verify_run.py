#!/usr/bin/env python3
"""Check a run folder's bags: per topic message count, rate, largest gap; duration; flags anything missing.
    verify_run.py <run_dir | bag_dir> [--json out.json]
Exit code 1 when a REQUIRED topic is missing or breaks its limits (gap / rate). Reads MCAP via rosbag2_py.
"""
import glob
import json
import os
import sys

# topic: (required, min_rate_hz, max_gap_s)   - None = not checked
EXPECT = {
    '/scan': (True, 4.5, 0.5),
    '/imu/data_raw': (True, 70.0, 0.1),
    '/imu/data': (False, 70.0, 0.1),
    '/tf': (True, 20.0, 0.5),
    '/motor/pwm_state': (False, 8.0, 0.5),
    '/cmd_vel': (False, None, None),
    '/camera/frame_info': (False, 20.0, 0.5),
    '/map': (False, 0.5, 5.0),
    '/mission/state': (False, 0.5, 3.0),
    '/collision_monitor_state': (False, None, None),
    '/localization/confidence': (False, 1.0, 2.0),
}


def read_bag(path):
    import rosbag2_py
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    stamps = {}
    while r.has_next():
        topic, _, t = r.read_next()
        stamps.setdefault(topic, []).append(t * 1e-9)
    return stamps


def stats(ts):
    ts = sorted(ts)
    dur = ts[-1] - ts[0] if len(ts) > 1 else 0.0
    gaps = [b - a for a, b in zip(ts, ts[1:])]
    return {'count': len(ts), 'rate_hz': round((len(ts) - 1) / dur, 2) if dur > 0 else None,
            'max_gap_s': round(max(gaps), 3) if gaps else None, 'start': ts[0], 'end': ts[-1]}


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    out_json = sys.argv[sys.argv.index('--json') + 1] if '--json' in sys.argv else None
    root = os.path.expanduser(args[0])
    bags = sorted({os.path.dirname(p) for p in glob.glob(os.path.join(root, '**', 'metadata.yaml'), recursive=True)})
    if not bags and glob.glob(os.path.join(root, '*.mcap')):
        bags = [root]
    report, bad = {}, []
    for b in bags:
        try:
            stamps = read_bag(b)
        except Exception as e:                                   # a broken bag is a finding, not a crash
            report[b] = {'error': str(e)}
            bad.append(f'{os.path.basename(b)}: unreadable ({e})')
            continue
        st = {t: stats(v) for t, v in stamps.items()}
        t0 = min(s['start'] for s in st.values()) if st else 0
        t1 = max(s['end'] for s in st.values()) if st else 0
        report[b] = {'duration_s': round(t1 - t0, 1), 'topics': st}
        print(f'\n== {os.path.relpath(b, root)}  duration {t1 - t0:.1f} s, {len(st)} topics')
        print(f'   {"topic":34s} {"count":>8s} {"Hz":>8s} {"max gap":>8s}  check')
        for t in sorted(set(st) | {k for k, v in EXPECT.items() if v[0]}):
            req, rmin, gmax = EXPECT.get(t, (False, None, None))
            s = st.get(t)
            if s is None:
                if req:
                    bad.append(f'{os.path.basename(b)}: {t} MISSING')
                    print(f'   {t:34s} {"-":>8s} {"-":>8s} {"-":>8s}  MISSING')
                continue
            flags = []
            if rmin and s['rate_hz'] is not None and s['rate_hz'] < rmin:
                flags.append(f'rate < {rmin}')
            if gmax and s['max_gap_s'] is not None and s['max_gap_s'] > gmax:
                flags.append(f'gap > {gmax} s')
            if flags and req:
                bad.append(f'{os.path.basename(b)}: {t} ' + ', '.join(flags))
            print(f'   {t:34s} {s["count"]:8d} {str(s["rate_hz"]):>8s} {str(s["max_gap_s"]):>8s}  '
                  f'{"; ".join(flags) or "ok"}')
    if not bags:
        bad.append('no bags found')
    print('\nVERIFY ' + ('OK' if not bad else 'PROBLEMS:\n  - ' + '\n  - '.join(bad)))
    if out_json:
        json.dump({'bags': report, 'problems': bad}, open(out_json, 'w'), indent=1, default=str)
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()

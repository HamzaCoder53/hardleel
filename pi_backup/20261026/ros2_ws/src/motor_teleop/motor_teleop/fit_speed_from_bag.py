#!/usr/bin/env python3
"""Offline max_speed fit from a mapping-phase rosbag2 (mcap or sqlite3).

  ros2 run motor_teleop fit_speed_from_bag <bag_dir> [--apply] [--min-pwm N] [--calib-file F]

Reads /motor/pwm_state (Int32MultiArray [l, r] actually sent) and /tf (map->odom and
odom->base_link from Cartographer, or a direct map->base_link). Straight segments
(|l-r| small, same sign, steady >= 1 s) give forward speed from the pose; max_speed is fitted
with the motor_bridge model v = k * (pwm - min_pwm), max_speed = k * (max_pwm - min_pwm).
"""
import argparse
import sys

from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message

from motor_teleop import calib_math
from motor_teleop.calib_store import CalibStore, DEFAULT_PATH


def read_bag(path):
    """Returns (pwm_samples [(t,l,r)], poses [(t,x,y,yaw)]) with t in seconds (bag receive time)."""
    import glob
    import os
    import subprocess
    if not os.path.isfile(os.path.join(path, 'metadata.yaml')) and glob.glob(os.path.join(path, '*.mcap')):
        # recorder killed before it wrote metadata.yaml (crash, power loss): rebuild it
        print(f'{path}: no metadata.yaml, running ros2 bag reindex ...')
        subprocess.run(['ros2', 'bag', 'reindex', path, '-s', 'mcap'], check=False)
    storage = None
    for sid in ('mcap', 'sqlite3'):
        try:
            reader = rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=sid),
                        rosbag2_py.ConverterOptions('', ''))
            storage = sid
            break
        except Exception:  # noqa: BLE001
            continue
    if storage is None:
        raise RuntimeError(f'cannot open bag {path} (tried mcap, sqlite3)')
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for need in ('/motor/pwm_state', '/tf'):
        if need not in types:
            raise RuntimeError(f'topic {need} not in bag (have: {sorted(types)})')
    msg_types = {n: get_message(t) for n, t in types.items() if n in ('/motor/pwm_state', '/tf')}
    pwm, poses = [], []
    mo = ob = None      # latest map->odom, odom->base_link as (x, y, yaw)
    import math
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in msg_types:
            continue
        t = t_ns * 1e-9
        msg = deserialize_message(data, msg_types[topic])
        if topic == '/motor/pwm_state':
            if len(msg.data) >= 2:
                pwm.append((t, msg.data[0], msg.data[1]))
            continue
        for tf in msg.transforms:
            q, tr = tf.transform.rotation, tf.transform.translation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            pose = (tr.x, tr.y, yaw)
            parent, child = tf.header.frame_id.lstrip('/'), tf.child_frame_id.lstrip('/')
            if (parent, child) == ('map', 'base_link'):
                poses.append((t,) + pose)
            elif (parent, child) == ('map', 'odom'):
                mo = pose
            elif (parent, child) == ('odom', 'base_link'):
                ob = pose
                if mo is not None:
                    poses.append((t,) + calib_math.compose_2d(mo, ob))
    return pwm, poses


def main(argv=None):
    ap = argparse.ArgumentParser(description='Fit max_speed from a mapping rosbag')
    ap.add_argument('bag')
    ap.add_argument('--apply', action='store_true')
    ap.add_argument('--calib-file', default=DEFAULT_PATH)
    ap.add_argument('--min-pwm', type=float, default=None, help='override mean deadband (default: from calib file)')
    ap.add_argument('--max-pwm', type=int, default=255)
    ap.add_argument('--side-tol', type=float, default=8)
    ap.add_argument('--steady-tol', type=float, default=6)
    ap.add_argument('--min-dur', type=float, default=1.0)
    ap.add_argument('--skip', type=float, default=0.4, help='s to ignore at the start of each segment')
    ap.add_argument('--force', action='store_true')
    args = ap.parse_args(argv)

    store = CalibStore(args.calib_file)
    cal = store.load()
    if args.min_pwm is not None:
        min_pwm = args.min_pwm
    else:
        mins = [cal[k] for k in ('min_pwm_left', 'min_pwm_right') if k in cal]
        min_pwm = sum(mins) / len(mins) if mins else 0.0
        if not mins:
            print('WARNING: no min_pwm in calib file; assuming 0')

    pwm, poses = read_bag(args.bag)
    print(f'{len(pwm)} pwm_state msgs, {len(poses)} map->base_link poses, min_pwm {min_pwm:.1f}')
    segs = calib_math.find_straight_segments(pwm, args.side_tol, args.steady_tol, args.min_dur)
    samples = []
    print(f'{"t0":>10} {"t1":>10} {"pwm":>6} {"v m/s":>8}')
    for t0, t1, m in segs:
        seg_poses = [p for p in poses if t0 <= p[0] <= t1]
        v = calib_math.speed_from_poses(seg_poses, args.skip)
        print(f'{t0:10.1f} {t1:10.1f} {m:6.0f} {"-" if v is None else format(abs(v), "8.3f")}')
        if v is not None and abs(v) > 0.02:
            samples.append((m, abs(v)))
    fit = calib_math.fit_speed(samples, min_pwm, args.max_pwm)
    print(f'segments: {len(segs)} found, {len(samples)} usable')
    if fit is None:
        print('no usable segments; cannot fit')
        return 1
    print(f'max_speed = {fit["max_speed"]:.3f} m/s  (k={fit["k"]:.5f}, rms residual {fit["rms"]:.3f} m/s, n={fit["n"]})')
    if args.apply:
        ok, reason = store.accept('max_speed', fit['max_speed'], force=args.force, source='fit_speed_from_bag')
        print(('SAVED' if ok else 'REJECTED (last good kept)') + f': {reason}')
        return 0 if ok else 2
    print('(dry run; use --apply to save)')
    return 0


if __name__ == '__main__':
    sys.exit(main())

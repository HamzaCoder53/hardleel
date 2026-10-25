#!/usr/bin/env python3
"""Move the sim's "worker" model around: it changes place every `period` seconds, like the competition's
workers whose location changes during a run (Gazebo set_pose service; Harmonic `gz`, Fortress `ign`).

    worker_mover.py <period_s> x1,y1 x2,y2 ...      positions in the world frame, visited in a loop
"""
import os
import subprocess
import sys
import time


def main():
    period = float(sys.argv[1])
    pts = [tuple(float(v) for v in p.split(',')) for p in sys.argv[2:]]
    humble = os.environ.get('ROS_DISTRO') == 'humble'
    gz, msg = ('ign', 'ignition.msgs') if humble else ('gz', 'gz.msgs')
    k = 0
    while True:
        x, y = pts[k % len(pts)]
        req = f'name: "worker" position: {{x: {x} y: {y} z: 0.2}}'
        r = subprocess.run([gz, 'service', '-s', '/world/rdj2026/set_pose', '--reqtype', f'{msg}.Pose',
                            '--reptype', f'{msg}.Boolean', '--timeout', '2000', '--req', req],
                           capture_output=True, text=True)
        print(f'{time.strftime("%H:%M:%S")} worker -> ({x:.2f}, {y:.2f}) {r.stdout.strip()}', flush=True)
        k += 1
        time.sleep(period)


if __name__ == '__main__':
    main()

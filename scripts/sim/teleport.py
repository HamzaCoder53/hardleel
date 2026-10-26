"""Teleport the simulated robot (Gazebo Harmonic set_pose service) - used by the recovery test.
    python teleport.py X Y YAW_DEG      (WORLD frame)"""
import math
import subprocess
import sys


def teleport(x, y, yaw, world='rdj2026', name='hardleel', z=0.05):
    req = (f'name: "{name}" position: {{x: {x} y: {y} z: {z}}} '
           f'orientation: {{z: {math.sin(yaw / 2)} w: {math.cos(yaw / 2)}}}')
    r = subprocess.run(['gz', 'service', '-s', f'/world/{world}/set_pose', '--reqtype', 'gz.msgs.Pose',
                        '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', req],
                       capture_output=True, text=True)
    print('teleport', r.stdout.strip() or r.stderr.strip())
    return 'true' in r.stdout


if __name__ == '__main__':
    teleport(float(sys.argv[1]), float(sys.argv[2]), math.radians(float(sys.argv[3])))

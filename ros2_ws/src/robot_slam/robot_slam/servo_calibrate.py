"""Interactive servo calibration (Pi, no ROS).

    python3 -m robot_slam.servo_calibrate --gpio 12 [--gpio-b 13] [--backend pi_pwm|lgpio|null] [--out FILE]
Keys: + / - = 1 deg, ] / [ = 10 deg, o / c = mark open / closed, a / b = select servo,
      t = test drop sequence, w = write YAML (node param format), q = quit (always detaches).
Pi hardware PWM needs `dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4` in /boot/firmware/config.txt.
The first move goes to 90 deg: keep the flaps free of obstructions.
"""
import argparse
import os
import sys

from .dropper import Dropper
from .servo_backends import make_backend
from .servo_core import DEFAULTS, angle_to_pulse_us, clamp


def getch():
    if not sys.stdin.isatty():
        return sys.stdin.read(1)
    import termios
    import tty
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def render_yaml(cfg):
    keys = ['has_drop_bin', 'backend', 'servo_a_gpio', 'servo_b_gpio', 'closed_deg_a', 'closed_deg_b',
            'open_deg_a', 'open_deg_b', 'min_us', 'max_us', 'hold_s', 'move_s', 'close_on_start']
    lines = ['drop_servo:', '  ros__parameters:']
    for k in keys:
        v = cfg[k]
        v = str(v).lower() if isinstance(v, bool) else v
        lines.append('    %s: %s' % (k, v))
    return '\n'.join(lines) + '\n'


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--gpio', type=int, default=DEFAULTS['servo_a_gpio'])
    ap.add_argument('--gpio-b', type=int, default=None)
    ap.add_argument('--backend', default='pi_pwm')
    ap.add_argument('--min-us', type=int, default=DEFAULTS['min_us'])
    ap.add_argument('--max-us', type=int, default=DEFAULTS['max_us'])
    ap.add_argument('--out', default=os.path.expanduser('~/hardleel_ws/drop_servo_calibration.yaml'))
    a = ap.parse_args(argv)

    gpios = [a.gpio] + ([a.gpio_b] if a.gpio_b is not None else [])
    cfg = {**DEFAULTS, 'has_drop_bin': True, 'backend': a.backend, 'servo_a_gpio': gpios[0],
           'servo_b_gpio': gpios[-1] if len(gpios) > 1 else DEFAULTS['servo_b_gpio'],
           'min_us': a.min_us, 'max_us': a.max_us}
    backend = make_backend(a.backend, gpios)
    print('backend:', type(backend).__name__)
    ang, cur = [90.0] * len(gpios), 0
    try:
        while True:
            us = angle_to_pulse_us(ang[cur], a.min_us, a.max_us, cfg['range_deg'])
            backend.set_pulse_us(cur, us)
            print('\rservo %s  %6.1f deg  %4d us   ' % ('AB'[cur], ang[cur], us), end='', flush=True)
            k = getch()
            print()
            step = {'+': 1, '=': 1, '-': -1, ']': 10, '[': -10}.get(k)
            if step:
                ang[cur] = clamp(ang[cur] + step, 0, cfg['range_deg'])
            elif k in ('o', 'c'):
                cfg[('open' if k == 'o' else 'closed') + '_deg_' + 'ab'[cur]] = ang[cur]
                print('marked %s = %.1f' % ('open' if k == 'o' else 'closed', ang[cur]))
            elif k in ('a', 'b') and 'ab'.index(k) < len(gpios):
                backend.release(cur)
                cur = 'ab'.index(k)
            elif k == 't':
                print(Dropper(backend, cfg).dump())
            elif k == 'w':
                os.makedirs(os.path.dirname(a.out) or '.', exist_ok=True)
                with open(a.out, 'w') as f:
                    f.write(render_yaml(cfg))
                print('wrote', a.out)
            elif k in ('q', '\x03', ''):
                break
    finally:
        backend.close()  # detach


if __name__ == '__main__':
    main()

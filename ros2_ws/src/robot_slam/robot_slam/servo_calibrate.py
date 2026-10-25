"""Interactive drop-servo calibration.

    ros2 run robot_slam servo_calibrate                  # default: --backend pico, servos on the Pico (GP12/GP13)
        needs motor_bridge running (it owns the Pico serial port); both servos move together (one angle);
        each move waits for the Pico's SV,done
    python3 -m robot_slam.servo_calibrate --backend pi_pwm --gpio 12 [--gpio-b 13]   # servos on Pi GPIO, no ROS
Options: [--out FILE]
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
    keys = ['has_drop_bin', 'backend', 'pico_dps', 'pico_timeout_s', 'servo_a_gpio', 'servo_b_gpio',
            'closed_deg_a', 'closed_deg_b', 'open_deg_a', 'open_deg_b', 'min_us', 'max_us', 'hold_s', 'move_s',
            'close_on_start']
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
    ap.add_argument('--backend', default='pico')
    ap.add_argument('--min-us', type=int, default=DEFAULTS['min_us'])
    ap.add_argument('--max-us', type=int, default=DEFAULTS['max_us'])
    ap.add_argument('--out', default=os.path.expanduser('~/hardleel_ws/drop_servo_calibration.yaml'))
    a = ap.parse_args(argv)

    gpios = [a.gpio] + ([a.gpio_b] if a.gpio_b is not None else [])
    cfg = {**DEFAULTS, 'has_drop_bin': True, 'backend': a.backend, 'servo_a_gpio': gpios[0],
           'servo_b_gpio': gpios[-1] if len(gpios) > 1 else DEFAULTS['servo_b_gpio'],
           'min_us': a.min_us, 'max_us': a.max_us}
    node = None
    if a.backend == 'pico':                    # talk to motor_bridge over ROS; spin in a background thread
        import threading
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        rclpy.init()
        node = rclpy.create_node('servo_calibrate')
        ex = MultiThreadedExecutor(num_threads=2)
        ex.add_node(node)
        threading.Thread(target=ex.spin, daemon=True).start()
    backend = make_backend(a.backend, gpios, node=node, min_us=a.min_us, max_us=a.max_us,
                           range_deg=cfg['range_deg'], dps=cfg['pico_dps'], timeout_s=cfg['pico_timeout_s'])
    print('backend:', type(backend).__name__, '(both servos together)' if a.backend == 'pico' else '')
    ang, cur = [90.0] * backend.n, 0         # the Pico firmware holds 90 deg from power-up
    try:
        while True:
            us = angle_to_pulse_us(ang[cur], a.min_us, a.max_us, cfg['range_deg'])
            try:
                backend.set_pulse_us(cur, us)
            except RuntimeError as e:           # pico: no SV,done (motor_bridge not running?)
                print('\n', e)
            print('\rservo %s  %6.1f deg  %4d us   ' % ('AB'[cur], ang[cur], us), end='', flush=True)
            k = getch()
            print()
            step = {'+': 1, '=': 1, '-': -1, ']': 10, '[': -10}.get(k)
            if step:
                ang[cur] = clamp(ang[cur] + step, 0, cfg['range_deg'])
            elif k in ('o', 'c'):
                cfg[('open' if k == 'o' else 'closed') + '_deg_' + 'ab'[cur]] = ang[cur]
                print('marked %s = %.1f' % ('open' if k == 'o' else 'closed', ang[cur]))
            elif k in ('a', 'b') and 'ab'.index(k) < backend.n:
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
        if node is not None:
            import rclpy
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()

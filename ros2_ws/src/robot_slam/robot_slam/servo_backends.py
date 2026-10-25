"""Servo backends. Interface: n, set_pulse_us(idx, us), release(idx), close().

Pi hardware PWM (sysfs) needs this in /boot/firmware/config.txt (NOT edited by us; reboot after):
    dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
(GPIO12 = pwmchip0/pwm0, GPIO13 = pwm1; for GPIO18/19 use `dtoverlay=pwm-2chan`.)
sysfs write access needs root or a udev rule; otherwise falls back to lgpio (apt: python3-lgpio).
release() stops the pulses so the servo does not buzz or heat; idle CPU is ~0.
"""
import logging
import os
import time

log = logging.getLogger('servo_backends')
PWM_ROOT = '/sys/class/pwm/pwmchip0'
PERIOD_NS = 20_000_000  # 50 Hz
GPIO_TO_CHANNEL = {12: 0, 18: 0, 13: 1, 19: 1}


class BackendUnavailable(RuntimeError):
    pass


class NullBackend:
    """Logs only; used when there is no drop bin or no hardware."""

    def __init__(self, n=2):
        self.n = n

    def set_pulse_us(self, idx, us):
        log.info('null: servo %d -> %d us', idx, us)

    def release(self, idx):
        log.info('null: servo %d released', idx)

    def close(self):
        pass


class PiHardwarePwmBackend:
    def __init__(self, gpios, root=PWM_ROOT):
        self.root, self.n = root, len(gpios)
        if not os.path.isdir(root):
            raise BackendUnavailable('%s missing (add the pwm-2chan overlay)' % root)
        try:
            self.ch = [GPIO_TO_CHANNEL[int(g)] for g in gpios]
        except KeyError as e:
            raise BackendUnavailable('GPIO %s has no hardware PWM (use 12/13/18/19)' % e)
        if len(set(self.ch)) != len(self.ch):
            raise BackendUnavailable('both servos map to the same PWM channel')
        try:
            for c in self.ch:
                if not os.path.isdir('%s/pwm%d' % (root, c)):
                    self._w('export', c)
                    for _ in range(20):  # wait for udev permissions
                        if os.path.isdir('%s/pwm%d' % (root, c)):
                            break
                        time.sleep(0.05)
                self._w('pwm%d/period' % c, PERIOD_NS)
        except OSError as e:
            raise BackendUnavailable('sysfs pwm not usable: %s' % e)

    def _w(self, name, val):
        with open('%s/%s' % (self.root, name), 'w') as f:
            f.write(str(val))

    def set_pulse_us(self, idx, us):
        c = self.ch[idx]
        self._w('pwm%d/duty_cycle' % c, int(us) * 1000)
        self._w('pwm%d/enable' % c, 1)

    def release(self, idx):
        self._w('pwm%d/enable' % self.ch[idx], 0)

    def close(self):
        for i in range(self.n):
            try:
                self.release(i)
            except OSError:
                pass


class LgpioBackend:
    def __init__(self, gpios):
        try:
            import lgpio
        except ImportError:
            raise BackendUnavailable('python3-lgpio not installed')
        self._lg, self.gpios, self.n = lgpio, [int(g) for g in gpios], len(gpios)
        try:
            self.h = lgpio.gpiochip_open(0)
            for g in self.gpios:
                lgpio.gpio_claim_output(self.h, g, 0)
        except Exception as e:
            raise BackendUnavailable('lgpio open failed: %s' % e)

    def set_pulse_us(self, idx, us):
        self._lg.tx_servo(self.h, self.gpios[idx], int(us), 50)

    def release(self, idx):
        self._lg.tx_servo(self.h, self.gpios[idx], 0)  # 0 = stop pulses
        self._lg.gpio_write(self.h, self.gpios[idx], 0)

    def close(self):
        for i in range(self.n):
            try:
                self.release(i)
            except Exception:
                pass
        try:
            self._lg.gpiochip_close(self.h)
        except Exception:
            pass


class PicoBackend:
    """PLACEHOLDER stub. Would send e.g. 'S,<id>,<us>' + newline over the existing motor_bridge
    serial link ('S,<id>,0' = detach). Needs a Pico firmware change: only with the user's OK."""

    def __init__(self, *a, **k):
        raise NotImplementedError('pico servo backend: firmware command not defined yet')


def make_backend(name, gpios):
    """Build a backend; pi_pwm falls back to lgpio, anything unavailable falls back to Null."""
    order = {'pi_pwm': ['pi_pwm', 'lgpio'], 'lgpio': ['lgpio']}.get(name, [])
    for n in order:
        try:
            return PiHardwarePwmBackend(gpios) if n == 'pi_pwm' else LgpioBackend(gpios)
        except BackendUnavailable as e:
            log.warning('backend %s unavailable: %s', n, e)
    if name == 'pico':
        try:
            return PicoBackend()
        except NotImplementedError as e:
            log.warning('%s', e)
    return NullBackend(len(gpios))

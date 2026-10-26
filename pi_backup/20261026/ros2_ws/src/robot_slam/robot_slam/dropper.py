"""Drop sequencing, independent of ROS: Dropper(backend, cfg, sleep=time.sleep)."""
import threading
import time

from .servo_core import DEFAULTS, DropSequence, angle_to_pulse_us


class Dropper:
    def __init__(self, backend, cfg, sleep=time.sleep, on_state=None):
        self.backend, self.cfg, self.sleep = backend, {**DEFAULTS, **cfg}, sleep
        self.on_state = on_state or (lambda s: None)
        self.seq = DropSequence.from_config(self.cfg)
        self._lock = threading.Lock()

    def _move(self, angles, wait_s):
        for i in range(min(self.backend.n, 2)):
            self.backend.set_pulse_us(i, angle_to_pulse_us(
                angles[i], self.cfg['min_us'], self.cfg['max_us'], self.cfg['range_deg']))
        self.sleep(wait_s)

    def _release(self):
        for i in range(self.backend.n):
            try:
                self.backend.release(i)
            except Exception:
                pass

    def close_flaps(self):
        """(ok, msg): drive to closed then detach. Used at start-up."""
        if not self.cfg['has_drop_bin']:
            return True, 'no drop bin: skipped'
        if not self._lock.acquire(blocking=False):
            return False, 'busy'
        try:
            self._move(self.seq.closed_angles, self.seq.move_s)
            self.on_state('closed')
            return True, 'closed'
        except Exception as e:
            return False, 'hardware error: %s' % e
        finally:
            self._release()
            self._lock.release()

    def dump(self):
        """(ok, msg): open both, hold, close, detach."""
        if not self.cfg['has_drop_bin']:
            return True, 'no drop bin: skipped'
        if not self._lock.acquire(blocking=False):
            return False, 'busy: drop in progress'
        try:
            for name, angles, wait in self.seq.steps():
                if name == 'open':
                    self.on_state('open')
                if name == 'hold':
                    self.sleep(wait)  # pulses already set
                else:
                    self._move(angles, wait)
            self.on_state('closed')
            return True, 'dropped'
        except Exception as e:
            return False, 'hardware error: %s' % e
        finally:
            self._release()
            self._lock.release()
            self.on_state('idle')

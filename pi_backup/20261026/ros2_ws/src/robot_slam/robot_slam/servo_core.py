"""Pure servo maths + drop sequence (no ROS, no hardware).

Pulse mapping: angle 0..range_deg -> min_us..max_us, clamped.
"""
from dataclasses import dataclass

# Defaults = node params. All angles/pins are PLACEHOLDERS until calibrated/mounted.
DEFAULTS = {
    'has_drop_bin': False,
    'backend': 'pico',        # pico (servos on the Pico GP12/GP13, via motor_bridge) | pi_pwm | lgpio | null
    'pico_dps': 240,          # SV slew speed, deg/s (firmware default 240; 0 = jump)
    'pico_timeout_s': 3.0,    # wait for the Pico's SV,done
    'servo_a_gpio': 12,       # PLACEHOLDER
    'servo_b_gpio': 13,       # PLACEHOLDER
    'closed_deg_a': 0.0,      # PLACEHOLDER (calibrate)
    'closed_deg_b': 0.0,      # PLACEHOLDER
    'open_deg_a': 90.0,       # PLACEHOLDER
    'open_deg_b': 90.0,       # PLACEHOLDER
    'min_us': 500,
    'max_us': 2500,
    'range_deg': 180.0,
    'hold_s': 1.0,
    'move_s': 0.4,
    'close_on_start': True,
}


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def angle_to_pulse_us(angle_deg, min_us=500, max_us=2500, range_deg=180):
    """Map an angle to a pulse width in microseconds; angle clamped to [0, range_deg]."""
    a = clamp(float(angle_deg), 0.0, float(range_deg))
    return int(round(min_us + (max_us - min_us) * a / float(range_deg)))


@dataclass(frozen=True)
class DropSequence:
    open_angles: tuple    # (A, B) degrees
    closed_angles: tuple  # (A, B) degrees
    hold_s: float = 1.0
    move_s: float = 0.4

    @classmethod
    def from_config(cls, cfg):
        c = {**DEFAULTS, **cfg}
        return cls((c['open_deg_a'], c['open_deg_b']), (c['closed_deg_a'], c['closed_deg_b']),
                   max(0.0, float(c['hold_s'])), max(0.0, float(c['move_s'])))

    def steps(self):
        """[(name, angles, wait_s)]: open + wait to reach, hold open, close + wait to reach."""
        return [('open', self.open_angles, self.move_s),
                ('hold', self.open_angles, self.hold_s),
                ('close', self.closed_angles, self.move_s)]

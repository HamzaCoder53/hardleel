"""Servo maths, sequence and Dropper logic (no ROS/hardware).

    cd ros2_ws/src/robot_slam && python3 -m pytest test/test_servo_core.py -q
"""
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from robot_slam.dropper import Dropper  # noqa: E402
from robot_slam.servo_backends import NullBackend, make_backend  # noqa: E402
from robot_slam.servo_core import DropSequence, angle_to_pulse_us  # noqa: E402


def test_pulse_mapping_and_clamp():
    assert angle_to_pulse_us(0) == 500
    assert angle_to_pulse_us(90) == 1500
    assert angle_to_pulse_us(180) == 2500
    assert angle_to_pulse_us(-20) == 500 and angle_to_pulse_us(400) == 2500
    assert angle_to_pulse_us(135, 1000, 2000, 270) == 1500


def test_sequence_from_config():
    s = DropSequence.from_config({'open_deg_a': 80, 'open_deg_b': 100, 'closed_deg_a': 5,
                                  'closed_deg_b': 175, 'hold_s': 2, 'move_s': 0.3})
    assert s.steps() == [('open', (80, 100), 0.3), ('hold', (80, 100), 2.0), ('close', (5, 175), 0.3)]


class Rec(NullBackend):
    def __init__(self, fail=False):
        super().__init__(2)
        self.log, self.fail = [], fail

    def set_pulse_us(self, i, us):
        if self.fail:
            raise OSError('boom')
        self.log.append(('p', i, us))

    def release(self, i):
        self.log.append(('r', i))


def test_dump_skipped_without_bin():
    b = Rec()
    assert Dropper(b, {'has_drop_bin': False}, sleep=lambda s: None).dump() == (True, 'no drop bin: skipped')
    assert b.log == []


def test_dump_sequence_and_detach():
    b, waits, states = Rec(), [], []
    d = Dropper(b, {'has_drop_bin': True, 'open_deg_a': 90, 'open_deg_b': 90, 'closed_deg_a': 0,
                    'closed_deg_b': 180}, sleep=waits.append, on_state=states.append)
    assert d.dump() == (True, 'dropped')
    assert waits == [0.4, 1.0, 0.4]
    assert b.log == [('p', 0, 1500), ('p', 1, 1500), ('p', 0, 500), ('p', 1, 2500), ('r', 0), ('r', 1)]
    assert states == ['open', 'closed', 'idle']


def test_hardware_error_reported_not_raised():
    b = Rec(fail=True)
    ok, msg = Dropper(b, {'has_drop_bin': True}, sleep=lambda s: None).dump()
    assert not ok and 'boom' in msg and ('r', 0) in b.log


def test_overlap_rejected():
    holder = {}

    def sleep(_):
        holder['inner'] = holder['d'].dump()

    d = holder['d'] = Dropper(Rec(), {'has_drop_bin': True}, sleep=sleep)
    assert d.dump()[0] is True
    assert holder['inner'][0] is False and 'busy' in holder['inner'][1]


def test_factory_falls_back_to_null():
    assert isinstance(make_backend('pico', [12, 13]), NullBackend)
    assert isinstance(make_backend('bogus', [12, 13]), NullBackend)

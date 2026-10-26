"""motor_bridge: deadband offset, and the new physical->channel mapping equals the original one."""
import math
import types

import pytest

mb = pytest.importorskip('motor_teleop.motor_bridge')


def fake(invert_linear=True, invert_left=False, invert_right=False, min_l=0, min_r=0, min_turn_pwm=0):
    """A MotorBridge-shaped object without ROS/serial: only what wheel_targets needs."""
    o = types.SimpleNamespace(max_speed=1.0, max_pwm=255, turn_gain=0.5, min_turn=0.0, invert_linear=invert_linear,
                              invert_left=invert_left, invert_right=invert_right, min_l=min_l, min_r=min_r,
                              min_turn_pwm=min_turn_pwm, speed_scale_enabled=False, speed_scale=1.0,
                              yaw_feedback=False, yaw_i=0.0, dir_l=0, dir_r=0, cmd_lin=0.0, cmd_ang=0.0)
    for name in ('wheel_targets', 'to_sent', 'physical', 'to_pwm'):
        setattr(o, name, types.MethodType(getattr(mb.MotorBridge, name), o))
    return o


def original(lin, ang, invert_linear=True, invert_left=False, invert_right=False):
    """The pre-25-Oct on_cmd_vel mapping (min_pwm 0)."""
    def to_pwm(v, inv):
        p = int(round(max(-255, min(255, v / 1.0 * 255))))
        return -p if inv else p
    if invert_linear:
        lin = -lin
    return to_pwm(lin - ang * 0.5, invert_left), to_pwm(lin + ang * 0.5, invert_right)


@pytest.mark.parametrize('lin,ang', [(0.3, 0.0), (-0.2, 0.0), (0.0, 1.0), (0.0, -1.0), (0.25, 0.6), (-0.1, -0.4)])
@pytest.mark.parametrize('il', [True, False])
def test_mapping_matches_original_without_deadband(lin, ang, il):
    o = fake(invert_linear=il)
    o.cmd_lin, o.cmd_ang = lin, ang
    assert o.wheel_targets() == original(lin, ang, invert_linear=il)


def test_physical_is_inverse_of_to_sent():
    for il in (True, False):
        for inv_l in (True, False):
            o = fake(invert_linear=il, invert_left=inv_l)
            for pl, pr in [(100, 50), (-80, 120), (0, -30)]:
                assert o.physical(*o.to_sent(pl, pr)) == [pl, pr]


def test_deadband_offset():
    assert mb.deadband(0, 60, 255) == 0
    assert mb.deadband(1, 60, 255) == 61               # 60 + 1 * 195 / 255
    assert mb.deadband(-255, 60, 255) == -255
    assert mb.deadband(128, 60, 255) == round(60 + 128 * 195 / 255)
    assert mb.deadband(100, 0, 255) == 100


def test_deadband_applies_to_physical_side():
    o = fake(min_l=70, min_r=0)                       # only the physical LEFT side has a deadband
    o.cmd_lin, o.cmd_ang = 0.05, 0.0                  # slow straight: raw 13 PWM
    pl, pr = o.physical(*o.wheel_targets())
    assert pl >= 70 and pr == 13


def test_min_turn_floor_only_for_spins():
    o = fake(min_turn_pwm=120)
    o.cmd_lin, o.cmd_ang = 0.0, 0.2                   # slow spin: raw +-26 -> floor 120
    pl, pr = o.physical(*o.wheel_targets())
    assert (pl, pr) == (-120, 120)
    o.cmd_lin, o.cmd_ang = 0.3, 0.2                   # driving + turning: no floor
    pl, pr = o.physical(*o.wheel_targets())
    assert abs(pl) < 120

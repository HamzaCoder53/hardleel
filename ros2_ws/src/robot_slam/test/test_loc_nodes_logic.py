"""Pure logic of the final-stack localization nodes: scan gate, gyro bias, confidence, checkpoint verdicts, ramp
pitch transitions."""
import math

import pytest

sg = pytest.importorskip('robot_slam.scan_gate')
ib = pytest.importorskip('robot_slam.imu_bias')
lm = pytest.importorskip('robot_slam.loc_monitor')


def test_gate_opens_only_after_settle():
    g = sg.GateLogic(0.5)
    assert g.update(0.0, None) == (False, 'settling (start)')
    assert g.update(0.6, None)[0] is True
    assert g.update(0.7, 'tilt +4.0/+0.0 deg') == (False, 'tilt +4.0/+0.0 deg')
    assert g.update(0.9, None)[0] is False                 # settling again
    assert g.update(1.3, None)[0] is False                 # 0.4 s
    assert g.update(1.45, None)[0] is True                 # 0.55 s level and steady


def test_in_zone():
    zones = [{'name': 'dirt_ramp', 'x_min': -4.6, 'x_max': -3.88, 'y_min': -2.65, 'y_max': 0.4}]
    assert sg.in_zone(-4.2, -1.0, zones) == 'dirt_ramp'
    assert sg.in_zone(-3.0, -1.0, zones) is None


def test_bias_tracks_when_still_and_freezes_when_moving():
    est = ib.BiasEstimator(still_s=0.5, tau_s=2.0)
    b = math.radians(0.8)
    t = 0.0
    for _ in range(1000):                                   # 10 s still with a 0.8 deg/s residual bias
        t += 0.01
        out = est.update(t, (0.0, 0.0, b), (0.0, 0.0, 9.81))
    assert abs(est.bias[2] - b) < math.radians(0.05)
    assert abs(out[2]) < math.radians(0.05)
    frozen = est.bias[2]
    for _ in range(300):                                    # turning: bias must not follow the real rate
        t += 0.01
        est.update(t, (0.0, 0.0, 1.0), (0.0, 0.0, 9.81))
    assert est.bias[2] == frozen


def test_bias_clamped():
    est = ib.BiasEstimator(still_rate=math.radians(10), still_s=0.1, tau_s=0.5, max_bias=math.radians(3))
    t = 0.0
    for _ in range(500):
        t += 0.01
        est.update(t, (0.0, 0.0, math.radians(8)), (0.0, 0.0, 9.81))
    assert est.bias[2] <= math.radians(3) + 1e-9


def test_confidence():
    C = lm.Confidence
    assert C.agreement((0.5, 0.1), (0.5, 0.1)) == pytest.approx(1.0)
    assert C.agreement((0.5, 0.0), (0.2, 0.0)) < 0.1
    assert C.combine(1.0, 1.0, True) == pytest.approx(1.0)
    assert C.combine(1.0, 1.0, False) == pytest.approx(0.2)


P = {'ok_m': 0.03, 'ok_deg': 1.5, 'correct_m': 0.20, 'correct_deg': 8.0, 'min_match_score': 0.45}


@pytest.mark.parametrize('dxy,ddeg,score,verdict', [
    (0.01, 0.5, 0.9, 'ok'), (0.10, 3.0, 0.9, 'correct'), (0.30, 1.0, 0.9, 'recover'),
    (0.05, 12.0, 0.9, 'recover'), (0.01, 0.0, 0.2, 'recover')])
def test_classify_error(dxy, ddeg, score, verdict):
    assert lm.classify_error(dxy, math.radians(ddeg), score, P) == verdict


def test_ramp_watch_sequence():
    r = lm.RampWatch()
    seq = [0, 2, 8, 12, 10, 1, -8, -12, -3, 0.5]
    evs = [e for e in (r.update(math.radians(v)) for v in seq) if e]
    assert evs == ['climb_start', 'descent_start', 'ramp_exit']

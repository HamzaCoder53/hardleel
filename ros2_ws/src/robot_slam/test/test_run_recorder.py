"""run_recorder pure logic: phase split from the mission state, stop conditions."""
import pytest

rr = pytest.importorskip('robot_slam.run_recorder')


def test_phase_from_state():
    p = 'mapping'
    for s, want in [('WAIT_START', 'mapping'), ('GO_SCREEN', 'mission'), ('TERRAIN', 'mission'), ('DONE', 'post')]:
        p = rr.phase_from_state(p, s)
        assert p == want
    assert rr.phase_from_state('post', 'GO_SCREEN') == 'post'      # never back


def test_stop_reason():
    pi = {'stop_temp_c': 80.0, 'stop_on_throttle': True, 'min_free_gb': 3.0}
    assert rr.stop_reason(pi, 60.0, 0x0, 20.0) is None
    assert rr.stop_reason(pi, 60.0, 0x50000, 20.0) is None        # history bits only
    assert 'temperature' in rr.stop_reason(pi, 81.0, 0x0, 20.0)
    assert 'throttled' in rr.stop_reason(pi, 60.0, 0x50005, 20.0)
    assert 'disk' in rr.stop_reason(pi, 60.0, 0x0, 2.5)
    laptop = {'stop_temp_c': 0.0, 'stop_on_throttle': False, 'min_free_gb': 3.0}
    assert rr.stop_reason(laptop, 95.0, 0x5, 10.0) is None

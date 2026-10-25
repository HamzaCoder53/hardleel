import csv
import math

from motor_teleop import calib_math as cm
from motor_teleop.calib_store import CalibStore


def make(tmp_path):
    return CalibStore(str(tmp_path / 'c.yaml'), str(tmp_path / 'h.csv'))


def rows(tmp_path):
    with open(tmp_path / 'h.csv') as f:
        return list(csv.DictReader(f))


def test_store_accept_and_load(tmp_path):
    s = make(tmp_path)
    ok, _ = s.accept('min_pwm_left', 40)
    assert ok and s.load() == {'min_pwm_left': 40.0}
    assert 'min_pwm_left' in s.meta()


def test_store_bounds_reject(tmp_path):
    s = make(tmp_path)
    assert not s.accept('min_pwm_left', 151)[0]
    assert not s.accept('min_turn_pwm', 201)[0]
    assert not s.accept('max_speed', 0.1)[0]
    assert not s.accept('max_speed', 3.5)[0]
    assert not s.accept('max_speed', float('nan'))[0]
    assert not s.accept('bogus', 1)[0]
    assert s.load() == {}
    assert len(rows(tmp_path)) == 6


def test_store_rel_change_reject_and_fallback(tmp_path):
    s = make(tmp_path)
    s.accept('max_speed', 1.0)
    ok, reason = s.accept('max_speed', 1.6)
    assert not ok and 'exceeds' in reason
    assert s.get('max_speed') == 1.0
    val, ok, _ = s.effective('max_speed', 0.4)
    assert val == 1.0 and not ok
    assert s.accept('max_speed', 1.4)[0]
    assert s.accept('max_speed', 2.9, force=True)[0]
    assert s.get('max_speed') == 2.9


def test_store_history(tmp_path):
    s = make(tmp_path)
    s.accept('min_turn_pwm', 60)
    s.accept('min_turn_pwm', 150)
    r = rows(tmp_path)
    assert [x['accepted'] for x in r] == ['1', '0']
    assert r[0]['key'] == 'min_turn_pwm' and r[1]['value'] == '150'


def test_store_atomic_no_tmp_left(tmp_path):
    s = make(tmp_path)
    s.accept('min_pwm_right', 30)
    assert sorted(p.name for p in tmp_path.iterdir()) == ['c.yaml', 'h.csv']


def test_turn_threshold():
    steps = [(30, 0.0), (35, 0.1), (40, 0.3), (45, 0.1), (50, 0.4), (55, 0.5), (60, 0.9)]
    assert cm.detect_turn_threshold(steps, 0.25, 2) == 50     # lone blip at 40 ignored
    assert cm.detect_turn_threshold(steps[:5], 0.25, 2) is None
    assert cm.detect_turn_threshold([(30, -0.4), (35, -0.5)], 0.25) == 30  # sign independent


def test_window_mean():
    s = [(0.0, 1.0), (0.5, 2.0), (0.9, 4.0), (1.0, 6.0)]
    assert cm.window_mean(s, 1.0, 0.3) == 5.0
    assert cm.window_mean(s, 10.0, 0.3) is None


def synth_poses(v, yaw=0.0, dur=1.5, rate=6.0, noise=0.0):
    n = int(dur * rate) + 1
    return [(i / rate, v * (i / rate) * math.cos(yaw), v * (i / rate) * math.sin(yaw), yaw)
            for i in range(n)]


def test_speed_from_poses():
    assert abs(cm.speed_from_poses(synth_poses(0.5, 0.7)) - 0.5) < 1e-6
    assert cm.speed_from_poses(synth_poses(0.5, 0.7)[:1]) is None
    # driving backwards relative to heading -> negative
    back = [(t, -x, -y, th) for t, x, y, th in synth_poses(0.4)]
    assert abs(cm.speed_from_poses(back) + 0.4) < 1e-6


def test_fit_speed_model():
    min_pwm, max_pwm, true_max = 40.0, 255, 1.2
    k = true_max / (max_pwm - min_pwm)
    samples = [(p, k * (p - min_pwm)) for p in (100, 160, 220)]
    f = cm.fit_speed(samples, min_pwm, max_pwm)
    assert abs(f['max_speed'] - true_max) < 1e-9 and f['rms'] < 1e-9 and f['n'] == 3
    noisy = [(100, k * 60 * 1.03), (160, k * 120 * 0.97), (220, k * 180 * 1.01)]
    assert abs(cm.fit_speed(noisy, min_pwm, max_pwm)['max_speed'] - true_max) < 0.05
    assert cm.fit_speed([(30, 0.1)], min_pwm, max_pwm) is None


def test_find_straight_segments():
    s = [(i * 0.1, 100, 102) for i in range(15)]                       # 1.4 s straight
    s += [(1.5 + i * 0.1, 100, -100) for i in range(5)]                # spin
    s += [(2.0 + i * 0.1, 150, 150) for i in range(5)]                 # too short
    s += [(3.0 + i * 0.1, -150, -148) for i in range(12)]              # reverse straight
    segs = cm.find_straight_segments(s, min_dur=1.0)
    assert len(segs) == 2
    assert abs(segs[0][2] - 101) < 1e-9 and abs(segs[1][2] - 149) < 0.5
    # step change in pwm splits segments
    s2 = [(i * 0.1, 80, 80) for i in range(12)] + [(1.2 + i * 0.1, 200, 200) for i in range(12)]
    assert len(cm.find_straight_segments(s2, min_dur=1.0)) == 2


def test_compose_2d():
    x, y, th = cm.compose_2d((1.0, 2.0, math.pi / 2), (1.0, 0.0, 0.0))
    assert abs(x - 1.0) < 1e-9 and abs(y - 3.0) < 1e-9 and abs(th - math.pi / 2) < 1e-9

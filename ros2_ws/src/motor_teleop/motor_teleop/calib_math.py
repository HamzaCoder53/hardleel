"""Pure (ROS-free) math for the motor calibration tools."""
import math


def detect_turn_threshold(steps, thresh, consecutive=2):
    """steps: iterable of (pwm, mean_gyro_z) in the order tested.
    Returns the pwm of the first step of the first run of `consecutive` steps whose
    |mean_gyro_z| > thresh, or None."""
    run = 0
    first = None
    for pwm, gz in steps:
        if abs(gz) > thresh:
            if run == 0:
                first = pwm
            run += 1
            if run >= consecutive:
                return first
        else:
            run = 0
            first = None
    return None


def window_mean(samples, t_end, window):
    """samples: list of (t, value). Mean of values with t_end-window < t <= t_end, or None."""
    vals = [v for t, v in samples if t_end - window < t <= t_end]
    return sum(vals) / len(vals) if vals else None


def speed_from_poses(poses, skip_s=0.4):
    """poses: list of (t, x, y, yaw) of one straight drive. Ignores the first skip_s seconds.
    Returns signed forward speed (m/s, + = along heading) from a least-squares slope of the
    position projected on the net displacement direction, or None if unusable."""
    if len(poses) < 2:
        return None
    t0 = poses[0][0]
    p = [q for q in poses if q[0] - t0 >= skip_s]
    if len(p) < 2 or p[-1][0] - p[0][0] <= 1e-6:
        return None
    dx, dy = p[-1][1] - p[0][1], p[-1][2] - p[0][2]
    d = math.hypot(dx, dy)
    if d < 1e-6:
        return 0.0
    ux, uy = dx / d, dy / d
    ts = [q[0] for q in p]
    ss = [(q[1] - p[0][1]) * ux + (q[2] - p[0][2]) * uy for q in p]
    tm, sm = sum(ts) / len(ts), sum(ss) / len(ss)
    den = sum((t - tm) ** 2 for t in ts)
    if den <= 0:
        return None
    slope = sum((t - tm) * (s - sm) for t, s in zip(ts, ss)) / den
    yaw = math.atan2(sum(math.sin(q[3]) for q in p), sum(math.cos(q[3]) for q in p))
    sign = 1.0 if (ux * math.cos(yaw) + uy * math.sin(yaw)) >= 0 else -1.0
    return sign * abs(slope)


def fit_speed(samples, min_pwm, max_pwm=255):
    """samples: list of (pwm_sent_magnitude, speed_m_s). Model (same as motor_bridge):
    v = k * (pwm - min_pwm), max_speed = k * (max_pwm - min_pwm). Least squares through origin.
    Returns dict(max_speed, k, rms, n) or None."""
    pts = [(abs(p) - min_pwm, abs(v)) for p, v in samples if v is not None and abs(p) > min_pwm]
    if not pts or max_pwm <= min_pwm:
        return None
    den = sum(x * x for x, _ in pts)
    if den <= 0:
        return None
    k = sum(x * v for x, v in pts) / den
    rms = math.sqrt(sum((v - k * x) ** 2 for x, v in pts) / len(pts))
    return {'max_speed': k * (max_pwm - min_pwm), 'k': k, 'rms': rms, 'n': len(pts)}


def find_straight_segments(samples, side_tol=8, steady_tol=6, min_dur=1.0):
    """samples: list of (t, l, r) pwm actually sent (+ = forward). Returns list of
    (t_start, t_end, mean_pwm_magnitude) for stretches where l,r have the same sign, are nonzero,
    |l-r| <= side_tol and the mean magnitude varies within steady_tol, lasting >= min_dur."""
    segs = []
    cur = []

    def flush():
        if len(cur) >= 2 and cur[-1][0] - cur[0][0] >= min_dur:
            segs.append((cur[0][0], cur[-1][0], sum(m for _, m in cur) / len(cur)))

    for t, l, r in samples:
        if not (l * r > 0 and abs(l - r) <= side_tol):
            flush()
            cur = []
            continue
        m = (abs(l) + abs(r)) / 2.0
        mags = [x for _, x in cur] + [m]
        if cur and max(mags) - min(mags) > steady_tol:
            flush()
            cur = []
        cur.append((t, m))
    flush()
    return segs


def compose_2d(a, b):
    """Compose planar transforms a*b, each (x, y, yaw)."""
    ax, ay, ath = a
    bx, by, bth = b
    c, s = math.cos(ath), math.sin(ath)
    th = math.atan2(math.sin(ath + bth), math.cos(ath + bth))
    return (ax + c * bx - s * by, ay + s * bx + c * by, th)

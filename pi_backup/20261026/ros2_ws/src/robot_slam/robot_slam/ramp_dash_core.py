"""Ramp dash logic (no ROS): wall geometry from the lidar's side sectors, attitude from the raw IMU, and the
dash state machine. Used by ramp_dash_node (runs on the Pi, so the control loop never crosses Wi-Fi).

Conventions: robot frame x forward, y left (REP-103). Heading/yaw + = CCW (left). PWM is PHYSICAL per side,
+ = forward, -255..255 (motor_bridge maps it to the Pico channels). lat = lateral offset from the strip's centre
line, + = robot is LEFT of centre.

Dash (team notes, 25 Oct):
  ALIGN   heading within ~2 deg of the strip walls and centred within ~2 cm (side-sector line fits). Heading: turn in
          place. Lateral: reverse with the nose turned toward the side we must move away from (keeps the run-up).
  CHECK   ramp zone clear: no lidar points in the corridor ahead (plus an optional camera 'clear' flag) - wait a
          little, then give up with 'blocked'. Never launch into an obstacle; never stop inside the ramp zone.
  RUNUP   PWM rises smoothly from runup_start_pwm to full so the wheels do not spin.
  DASH    full power. Steering only REDUCES the faster side (nothing above 100 %). Inner loop: gyro heading hold at
          IMU rate. Outer loop: gentle, rate-limited heading target from the side-wall centring; stronger near a
          wall (side clearance < near_wall_m). Front/back beams ignored while tilted.
  BOG     pitched up + no progress for bog_s -> reverse down, back up further, retry ONCE with a longer run-up.
  DONE    climbed and back on the flat (profile 'up_down': up, down, flat; 'up_only': up, flat).
"""
import math

import numpy as np

PHASES = ('IDLE', 'ALIGN', 'CHECK', 'RUNUP', 'DASH', 'RECOVER', 'DONE', 'FAILED')

DEFAULTS = {
    # lidar geometry
    'side_sector_deg': (70.0, 110.0),     # |angle| range of each side sector (left +, right -)
    'front_half_deg': 10.0,               # front sector (progress / clear check), flat only
    'min_side_points': 5,
    'robot_half_width_m': 0.135,
    # attitude
    'pitch_up_sign': 1.0,                 # +1: accel x > 0 when the nose is up (REP-103 IMU); flip if mounted reversed
    'gyro_z_sign': 1.0,
    'comp_alpha': 0.98,                   # complementary filter weight of the gyro
    'tilt_deg': 4.0,                      # |pitch| above this = tilted (front/back beams dropped)
    # align
    'align_heading_tol_deg': 2.0,
    'align_lat_tol_m': 0.02,
    'align_turn_pwm': 110,                # raw PWM (no deadband help): must beat the motors' deadband
    'align_drive_pwm': 110,
    'align_s_angle_deg': 20.0,            # nose angle while reversing sideways
    'align_max_s': 10.0,                  # give up aligning after this (log it, dash anyway)
    # clear check
    'clear_len_m': 0.60,                  # corridor ahead checked for points (keep short of where beams hit the ramp)
    'clear_margin_m': 0.06,               # ignore points this close to the strip walls
    'clear_min_points': 3,
    'clear_wait_s': 5.0,
    'require_camera_clear': False,
    # run-up / dash
    'runup_start_pwm': 153,               # 60 % -> 100 % ...
    'runup_s': 0.25,                      # ... over 0.2-0.3 s (team plan 26 Oct)
    'full_pwm': 255,
    'downhill_pwm': 180,                  # nose-down part: less than full (PLACEHOLDER: tune on the board ramp)
    'max_steer_frac': 0.6,                # the faster side is never reduced by more than this
    'k_heading': 2.5,                     # steer fraction per rad of heading error
    'k_rate': 0.15,                       # steer fraction per rad/s of yaw rate (damping)
    'k_lat': 1.5,                         # heading target (rad) per m of lateral offset
    'max_target_deg': 8.0,
    'near_wall_m': 0.05,                  # side clearance below this -> stronger centring
    'near_gain': 2.5,
    'near_max_target_deg': 18.0,
    'target_rate_dps': 30.0,              # heading target rate limit (no oscillation)
    'arm_deg': 5.0,                       # pitch beyond this = on the slope
    'release_deg': 2.0,
    'profile': 'up_down',                 # up_down | up_only
    'done_flat_s': 0.4,
    'no_climb_s': 3.0,                    # never tilted within this after the run-up -> FAILED no_ramp
    'max_dash_s': 12.0,
    # bog
    'bog_pitch_deg': 6.0,
    'bog_s': 1.0,                         # no progress this long while pitched up
    'bog_min_progress_m': 0.03,
    'max_climb_s': 3.0,                   # pitched up longer than this = bogged even without a range reading
    'reverse_pwm': 160,
    'reverse_extra_s': 0.6,               # keep reversing after the flat: back up further for a longer run-up
    'retry_runup_gain': 1.5,
    'max_attempts': 2,
}


def wall_fit(ranges, angle_min, angle_inc, lo_deg, hi_deg, rmax=3.0):
    """Least-squares line y = c + s*x through the beams with angle in [lo, hi] deg (radians internally).
    -> (perpendicular distance m, wall angle rad (+ = wall turns left ahead), n points) or None."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_inc * np.arange(len(r))
    lo, hi = math.radians(lo_deg), math.radians(hi_deg)
    m = (a >= min(lo, hi)) & (a <= max(lo, hi)) & np.isfinite(r) & (r > 0.05) & (r < rmax)
    if m.sum() < 2:
        return None
    x, y = r[m] * np.cos(a[m]), r[m] * np.sin(a[m])
    if np.ptp(x) < 0.05:                                     # all points straight to the side: flat wall, angle 0
        c, s = float(np.median(y)), 0.0
    else:
        s, c = np.polyfit(x, y, 1)
    return abs(c) / math.sqrt(1.0 + s * s), math.atan(s), int(m.sum())


def front_range(ranges, angle_min, angle_inc, half_deg):
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_inc * np.arange(len(r))
    m = (np.abs(a) <= math.radians(half_deg)) & np.isfinite(r) & (r > 0.05)
    return float(np.median(r[m])) if m.sum() >= 2 else None


def rear_range(ranges, angle_min, angle_inc, half_deg):
    """Median range straight behind (|angle| >= 180 - half_deg)."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_inc * np.arange(len(r))
    m = (np.abs(np.abs((a + math.pi) % (2 * math.pi) - math.pi)) >= math.pi - math.radians(half_deg)) & \
        np.isfinite(r) & (r > 0.05)
    return float(np.median(r[m])) if m.sum() >= 2 else None


def corridor_points(ranges, angle_min, angle_inc, length, half_width, lat=0.0, heading=0.0):
    """Number of beam end points in the strip ahead: 0.15 < along < length and |across| < half_width, in the STRIP
    frame (robot offset lat, heading vs the strip), so the strip walls never count when the robot is off-centre."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_inc * np.arange(len(r))
    ok = np.isfinite(r) & (r > 0.05)
    th = a[ok] + heading
    along, across = r[ok] * np.cos(th), r[ok] * np.sin(th) + lat
    return int(((along > 0.15) & (along < length) & (np.abs(across) < half_width)).sum())


class ScanGeometry:
    """Per-scan summary: side wall distances/angles, lateral offset, heading vs the strip, front range, clear."""

    def __init__(self, cfg):
        self.c = cfg

    def __call__(self, ranges, angle_min, angle_inc, tilted):
        c = self.c
        lo, hi = c['side_sector_deg']
        left = wall_fit(ranges, angle_min, angle_inc, lo, hi)
        right = wall_fit(ranges, angle_min, angle_inc, -hi, -lo)
        n = c['min_side_points']
        left = left if left and left[2] >= n else None
        right = right if right and right[2] >= n else None
        g = {'d_left': left[0] if left else None, 'd_right': right[0] if right else None,
             'lat': None, 'heading': None, 'front': None, 'front_tilted': None, 'clear_pts': None,
             'half_width': None}
        if left and right:
            g['lat'] = (right[0] - left[0]) / 2.0              # + = robot left of centre
            g['heading'] = -(left[1] + right[1]) / 2.0          # walls turn left ahead = robot points right of them
            g['half_width'] = (left[0] + right[0]) / 2.0
        elif left or right:
            g['heading'] = -(left or right)[1]
        fr = front_range(ranges, angle_min, angle_inc, c['front_half_deg'])
        rr = rear_range(ranges, angle_min, angle_inc, c['front_half_deg'])
        g['rear'] = rr                                          # speed = its rate (logged; flat parts only valid)
        if not tilted:
            g['front'] = fr
            hw = (g['half_width'] or 0.35) - c['clear_margin_m']
            g['clear_pts'] = corridor_points(ranges, angle_min, angle_inc, c['clear_len_m'], max(hw, 0.05),
                                             g['lat'] or 0.0, g['heading'] or 0.0)
        else:                                                   # tilted: front beams may hit the ramp or nothing;
            g['front_tilted'] = fr                              # used ONLY as a progress hint for the bog check
        return g


class Attitude:
    """Complementary filter on the raw IMU: pitch (+ = nose up) and integrated yaw (+ = left)."""

    def __init__(self, cfg):
        self.c = cfg
        self.pitch = None
        self.yaw = 0.0
        self.yaw_rate = 0.0
        self.t = None

    def update(self, t, ax, ay, az, gy, gz):
        s = self.c['pitch_up_sign']
        acc_pitch = math.atan2(s * ax, math.sqrt(ay * ay + az * az) or 1e-6)
        self.yaw_rate = self.c['gyro_z_sign'] * gz
        if self.pitch is None or self.t is None:
            self.pitch, self.t = acc_pitch, t
            return
        dt = min(max(t - self.t, 0.0), 0.1)
        self.t = t
        # nose-up is a NEGATIVE rotation about +y (REP-103), so pitch_up rate = -gy (times the mounting sign)
        a = self.c['comp_alpha']
        self.pitch = a * (self.pitch - s * gy * dt) + (1.0 - a) * acc_pitch
        self.yaw += self.yaw_rate * dt


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def steer_pwm(base, u, max_frac):
    """Full power both sides; turn by REDUCING the faster side only. u > 0 = turn left (left side slower)."""
    u = clamp(u, -max_frac, max_frac)
    left = base * (1.0 - max(0.0, u))
    right = base * (1.0 - max(0.0, -u))
    return int(round(left)), int(round(right))


class Dash:
    """State machine. Call scan() with every ScanGeometry result and step(t, att) at IMU rate; step returns
    (pwm_left, pwm_right) physical. self.phase / self.result / self.log_row() for the node."""

    def __init__(self, cfg=None):
        self.c = dict(DEFAULTS, **(cfg or {}))
        self.phase = 'IDLE'
        self.result = None
        self.geo = {}
        self.camera_clear = None
        self.attempt = 0
        self.events = []                       # (t, text) for the node log

    # ------------------------------------------------------------------ inputs
    def scan(self, geo):
        self.geo = geo

    def start(self, t, att):
        self.phase, self.result, self.attempt = 'ALIGN', None, 1
        self.t_phase = self.t_start = self.t_last = t
        self.align_sub, self.yaw0 = 'heading', att.yaw
        self.target, self.target_cmd = 0.0, 0.0
        self.pwm = (0, 0)
        self.runup_s = self.c['runup_s']
        self.reset_dash(t)
        self.note(t, f'start attempt 1')

    def reset_dash(self, t):
        self.seen_up = self.seen_down = False
        self.flat_since = None
        self.up_since = None
        self.prog_ref = None                   # (t, front range) for the bog check
        self.recover_flat_since = None

    def note(self, t, text):
        self.events.append((t, text))

    def go(self, t, phase):
        self.note(t, f'{self.phase} -> {phase}')
        self.phase, self.t_phase = phase, t

    def finish(self, t, ok, reason):
        self.result = {'ok': ok, 'reason': reason, 'attempts': self.attempt, 'time_s': round(t - self.t_start, 2)}
        self.go(t, 'DONE' if ok else 'FAILED')

    # ------------------------------------------------------------------ control
    def heading_hold(self, att, base):
        """Inner loop: heading (gyro-integrated, relative to the dash start) to the rate-limited target."""
        err = self.target - (att.yaw - self.yaw0)
        u = self.c['k_heading'] * err - self.c['k_rate'] * att.yaw_rate
        if base < 0:                    # reversing: slowing the left side now turns the robot RIGHT
            u = -u
        return steer_pwm(base, u, self.c['max_steer_frac']), err

    def outer_target(self, dt):
        """Outer loop: heading target from the lateral offset (gentle, rate limited; stronger near a wall)."""
        c, g = self.c, self.geo
        lat = g.get('lat')
        if lat is None:                                         # walls not seen this scan: relax toward straight
            want = 0.0
        else:
            clear = min(g['d_left'], g['d_right']) - c['robot_half_width_m']
            near = clear < c['near_wall_m']
            k = c['k_lat'] * (c['near_gain'] if near else 1.0)
            lim = math.radians(c['near_max_target_deg'] if near else c['max_target_deg'])
            want = clamp(-k * lat, -lim, lim)                   # left of centre -> aim right
        step = math.radians(c['target_rate_dps']) * dt
        self.target += clamp(want - self.target, -step, step)

    def step(self, t, att):
        c = self.c
        dt = clamp(t - getattr(self, 't_last', t), 0.0, 0.1)
        self.t_last = t
        if self.phase in ('IDLE', 'DONE', 'FAILED') or att.pitch is None:
            self.pwm = (0, 0)
            return self.pwm
        pitch = math.degrees(att.pitch)
        handler = getattr(self, 'do_' + self.phase.lower())
        self.pwm = handler(t, dt, att, pitch)
        return self.pwm

    def do_align(self, t, dt, att, pitch):
        c, g = self.c, self.geo
        if t - self.t_phase > c['align_max_s']:
            self.note(t, f'align gave up (heading {g.get("heading")}, lat {g.get("lat")}): dashing anyway')
            return self.enter_check(t, att)
        h, lat = g.get('heading'), g.get('lat')
        if h is None:
            self.note(t, 'align: no walls seen')
            return self.enter_check(t, att)
        if self.align_sub == 'heading':
            want = 0.0
        else:                                                   # lateral: nose turned while reversing
            want = math.radians(c['align_s_angle_deg']) * (1 if (lat or 0) > 0 else -1)
        err = want - h
        if abs(err) > math.radians(c['align_heading_tol_deg']):
            p = c['align_turn_pwm'] * (1 if err > 0 else -1)
            return (-p, p)                                      # turn in place toward want
        if self.align_sub == 'heading':
            if lat is not None and abs(lat) > c['align_lat_tol_m']:
                self.align_sub = 'lateral'
                self.note(t, f'align: lateral {lat:+.3f} m -> reverse sideways')
                return (0, 0)
            return self.enter_check(t, att)
        # lateral: reverse until centred, then square up again
        if lat is None or abs(lat) <= c['align_lat_tol_m'] * 0.5:
            self.align_sub = 'heading'
            return (0, 0)
        p = -c['align_drive_pwm']
        return (p, p)

    def enter_check(self, t, att):
        self.go(t, 'CHECK')
        self.yaw0 = att.yaw                                     # aligned: this heading is "along the strip"
        self.target = 0.0
        return (0, 0)

    def do_check(self, t, dt, att, pitch):
        c, g = self.c, self.geo
        pts = g.get('clear_pts')
        cam_ok = self.camera_clear is not False if not c['require_camera_clear'] else self.camera_clear is True
        if pts is not None and pts < c['clear_min_points'] and cam_ok:
            self.go(t, 'RUNUP')
            self.reset_dash(t)
            return (0, 0)
        if t - self.t_phase > c['clear_wait_s']:
            self.finish(t, False, f'blocked: {pts} points in the ramp zone, camera_clear={self.camera_clear}')
        return (0, 0)

    def do_runup(self, t, dt, att, pitch):
        c = self.c
        f = clamp((t - self.t_phase) / max(self.runup_s, 1e-3), 0.0, 1.0)
        base = c['runup_start_pwm'] + f * (c['full_pwm'] - c['runup_start_pwm'])
        self.outer_target(dt)
        pwm, _ = self.heading_hold(att, base)
        if f >= 1.0 or pitch > c['arm_deg']:
            self.go(t, 'DASH')
        return pwm

    def do_dash(self, t, dt, att, pitch):
        c, g = self.c, self.geo
        if pitch > c['arm_deg']:
            self.seen_up = True
            self.up_since = self.up_since or t
        elif pitch < -c['arm_deg']:
            self.seen_down = True
        if pitch <= c['arm_deg']:
            self.up_since = None
        flat = abs(pitch) < c['release_deg']
        self.flat_since = (self.flat_since or t) if flat else None
        done = self.seen_up and (self.seen_down or c['profile'] == 'up_only')
        if done and self.flat_since is not None and t - self.flat_since >= c['done_flat_s']:
            self.finish(t, True, 'climbed')
            return (0, 0)
        if not self.seen_up and t - self.t_phase > c['no_climb_s']:
            self.finish(t, False, 'no_ramp: never tilted')
            return (0, 0)
        if t - self.t_phase > c['max_dash_s']:
            self.finish(t, False, 'timeout')
            return (0, 0)
        if self.bogged(t, pitch):
            return self.enter_recover(t)
        base = c['full_pwm'] if pitch > -c['arm_deg'] else c['downhill_pwm']
        self.outer_target(dt)
        pwm, _ = self.heading_hold(att, base)
        return pwm

    def bogged(self, t, pitch):
        c, g = self.c, self.geo
        if pitch < c['bog_pitch_deg']:
            self.prog_ref = None
            return False
        if self.up_since is not None and t - self.up_since > c['max_climb_s']:
            self.note(t, f'bogged: pitched up {t - self.up_since:.1f} s')
            return True
        fr = g.get('front_tilted')                              # front range while tilted (node sets it if usable)
        if fr is None:
            return False
        if self.prog_ref is None or abs(fr - self.prog_ref[1]) >= c['bog_min_progress_m']:
            self.prog_ref = (t, fr)
            return False
        if t - self.prog_ref[0] >= c['bog_s']:
            self.note(t, f'bogged: front range stuck at {fr:.2f} m for {t - self.prog_ref[0]:.1f} s')
            return True
        return False

    def enter_recover(self, t):
        self.go(t, 'RECOVER')
        self.recover_flat_since = None
        return (0, 0)

    def do_recover(self, t, dt, att, pitch):
        c = self.c
        if abs(pitch) < c['release_deg']:
            self.recover_flat_since = self.recover_flat_since or t
        else:
            self.recover_flat_since = None
        if self.recover_flat_since is not None and t - self.recover_flat_since >= c['reverse_extra_s']:
            if self.attempt >= c['max_attempts']:
                self.finish(t, False, f'bogged {self.attempt}x')
                return (0, 0)
            self.attempt += 1
            self.runup_s = c['runup_s'] * c['retry_runup_gain']
            self.note(t, f'retry: attempt {self.attempt}, run-up {self.runup_s:.1f} s')
            self.align_sub = 'heading'
            self.go(t, 'ALIGN')
            return (0, 0)
        if t - self.t_phase > c['max_dash_s']:
            self.finish(t, False, 'recover timeout')
            return (0, 0)
        self.target = 0.0                                       # straight back down
        pwm, _ = self.heading_hold(att, -c['reverse_pwm'])
        return pwm

    # ------------------------------------------------------------------ logging
    LOG_FIELDS = ('t', 'phase', 'attempt', 'heading_deg', 'target_deg', 'err_deg', 'd_left', 'd_right', 'lat',
                  'pitch_deg', 'yaw_rate', 'pwm_left', 'pwm_right', 'front', 'rear', 'speed_rear')

    def scan_speed(self, t):
        """Forward speed from the rear-wall range rate (m/s, + = away from it); None until two readings."""
        rr = self.geo.get('rear')
        prev = getattr(self, '_rear_prev', None)
        if rr is None:
            return None
        self._rear_prev = (t, rr)
        if prev is None or t - prev[0] < 0.05 or t - prev[0] > 1.0:
            return getattr(self, '_speed', None)
        self._speed = (rr - prev[1]) / (t - prev[0])
        return self._speed

    def log_row(self, t, att):
        g = self.geo
        rel = math.degrees(att.yaw - getattr(self, 'yaw0', 0.0))
        tgt = math.degrees(getattr(self, 'target', 0.0))
        nan = float('nan')
        f = lambda v: nan if v is None else float(v)
        return [t, float(PHASES.index(self.phase)), float(self.attempt), rel, tgt, tgt - rel, f(g.get('d_left')),
                f(g.get('d_right')), f(g.get('lat')), math.degrees(att.pitch or 0.0), att.yaw_rate,
                float(self.pwm[0]), float(self.pwm[1]), f(g.get('front') or g.get('front_tilted')),
                f(g.get('rear')), f(getattr(self, '_speed', None))]

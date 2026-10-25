"""Pure-Python decision logic for competition_mission (no ROS imports, unit-testable).

State order, gate decision, class -> bay mapping, deadline, per-goal retry/skip policy, course loading.
"""
import math
import sys

STATES = ['WAIT_START', 'GO_SCREEN', 'CLASSIFY', 'GO_LOADING', 'LOAD', 'TERRAIN', 'OBSTACLES', 'JUNCTION',
          'GATE_DECISION', 'GO_PATH', 'DEPOSIT', 'FINISH', 'DONE']
TERMINAL = ('DONE', 'ABORTED')
# states that are still allowed to run after the deadline / during wrap-up
ENDGAME = ('FINISH', 'DONE', 'ABORTED')

DEFAULT_BAY_FOR_CLASS = {'plastic': 'blue', 'organic': 'white', 'metal': 'black'}


def next_state(state):
    """Nominal successor of `state` (the mission never goes backwards)."""
    if state in TERMINAL:
        return state
    return STATES[STATES.index(state) + 1]


def state_after_deadline(state):
    """On the global deadline go straight to FINISH, unless already finishing/finished."""
    return state if state in ENDGAME else 'FINISH'


class Deadline:
    def __init__(self, t_start, limit_s):
        self.t_start, self.limit_s = float(t_start), float(limit_s)

    def elapsed(self, now):
        return now - self.t_start

    def remaining(self, now):
        return self.limit_s - self.elapsed(now)

    def expired(self, now):
        return self.remaining(now) <= 0.0


# ------------------------------------------------------------------ clock-aware decisions (25 Oct)
def state_after_soft_deadline(state):
    """Little time left: skip whatever is still ahead of DEPOSIT (depositing scores, terrain left half-done does
    not lose what was already earned). DEPOSIT / FINISH and later run normally."""
    if state in TERMINAL or STATES.index(state) >= STATES.index('DEPOSIT'):
        return state
    return 'DEPOSIT'


def soft_deadline_due(state, remaining_s, reserve_s):
    """True if the mission should jump to DEPOSIT now: still before it and less than reserve_s left."""
    return state not in TERMINAL and STATES.index(state) < STATES.index('DEPOSIT') and remaining_s <= reserve_s


def path_switch_possible(state, current_path=''):
    """Blocked by an obstacle: is there another way? Yes on the way to the junction / gate check (take path1
    instead of waiting for the gate lane) and on path2 (path1 is always open). Elsewhere the only way is ahead:
    wait (the soft deadline still sends the robot to deposit when time runs short)."""
    return state in ('JUNCTION', 'GATE_DECISION') or (state == 'GO_PATH' and current_path == 'path2')


def gate_wait_budget(remaining_s, max_wait_s, reserve_s):
    """Seconds the gate may still be waited for: the configured max, but never eating into reserve_s (what the
    path, deposit and finish need after the decision). 0 = decide on the current reading: take whatever is
    open NOW (open gate -> path2, else path1, which is always open)."""
    return max(0.0, min(float(max_wait_s), float(remaining_s) - float(reserve_s)))


class ProgressWatch:
    """'Stuck behind an obstacle': an obstacle was seen recently and the robot has not moved min_move_m for
    max_wait_s. update(t, xy, obstacle_seen) -> True once that holds (then the mission gives up the waypoint)."""

    def __init__(self, t0, xy, max_wait_s=10.0, min_move_m=0.10, seen_hold_s=1.5):
        self.max_wait_s, self.min_move_m, self.seen_hold_s = max_wait_s, min_move_m, seen_hold_s
        self.anchor_t, self.anchor_xy = t0, xy
        self.last_seen = None

    def update(self, t, xy, obstacle_seen):
        if obstacle_seen:
            self.last_seen = t
        if xy is not None and (self.anchor_xy is None or
                               math.hypot(xy[0] - self.anchor_xy[0], xy[1] - self.anchor_xy[1]) >= self.min_move_m):
            self.anchor_t, self.anchor_xy = t, xy            # progress made: restart the clock
        recently_seen = self.last_seen is not None and t - self.last_seen <= self.seen_hold_s
        return recently_seen and t - self.anchor_t >= self.max_wait_s


class ObstacleEpisodes:
    """Split detections into episodes: a new episode starts when an obstacle is seen after clear_s without one.
    update(t, seen) -> 'start' (log obstacle_detected), 'cleared' (episode over without any response yet), None."""

    def __init__(self, clear_s=3.0):
        self.clear_s = clear_s
        self.active = False
        self.start_t = self.last_seen = None
        self.responded = False

    def update(self, t, seen):
        if seen:
            self.last_seen = t
            if not self.active:
                self.active, self.start_t, self.responded = True, t, False
                return 'start'
            return None
        if self.active and t - self.last_seen >= self.clear_s:
            self.active = False
            return 'cleared'
        return None


# ------------------------------------------------------------------ classification
def parse_classify(text):
    """'plastic:0.93:blue' -> (class, confidence, colour); malformed -> ('unknown', 0.0, '')."""
    try:
        cls, conf, colour = (text or '').strip().split(':')[:3]
        return cls.strip().lower(), float(conf), colour.strip().lower()
    except (ValueError, TypeError):
        return 'unknown', 0.0, ''


def usable_class(cls, valid=('plastic', 'organic', 'metal')):
    return cls in valid


def bay_for_class(cls, mapping=None, default_class='plastic'):
    """Bay colour for a waste class; unknown classes use `default_class`."""
    mapping = mapping or DEFAULT_BAY_FOR_CLASS
    if cls not in mapping:
        cls = default_class
    return mapping.get(cls, next(iter(mapping.values())))


def bay_stage(colour):
    return 'bay_' + colour


# ------------------------------------------------------------------ gate
def gate_region_free(ranges, angle_min, angle_inc, max_range, half_angle, min_range=0.12, min_hits=3):
    """True if fewer than `min_hits` beams inside +-half_angle of straight ahead hit something within max_range.

    Invalid readings (nan/inf/< min_range) count as free; the check is robot-relative, so call it at the
    gate_check pose facing the gate. Empty scans return None (unknown).
    """
    if not ranges:
        return None
    hits = 0
    for i, r in enumerate(ranges):
        a = angle_min + i * angle_inc
        a = math.atan2(math.sin(a), math.cos(a))
        if abs(a) > half_angle:
            continue
        if r is not None and math.isfinite(r) and min_range <= r <= max_range:
            hits += 1
    return hits < min_hits


class GateDecider:
    """Path 2 if the gate region has been free for `stable_s` within `wait_s`, otherwise Path 1.

    update(t, free) returns None while undecided, then 'path2' or 'path1'. free=None (no data) is ignored
    but still lets the timeout run, so a dead lidar falls back to Path 1 (always open).
    """

    def __init__(self, t_start, wait_s, stable_s=0.8):
        self.t_start, self.wait_s, self.stable_s = t_start, wait_s, stable_s
        self.free_since = None
        self.decision = None

    def update(self, t, free):
        if self.decision:
            return self.decision
        if free is True:
            if self.free_since is None:
                self.free_since = t
            if t - self.free_since >= self.stable_s:
                self.decision = 'path2'
        elif free is False:
            self.free_since = None
        if not self.decision and t - self.t_start >= self.wait_s:
            self.decision = 'path1'
        return self.decision


# ------------------------------------------------------------------ goal retry / skip policy
class GoalPolicy:
    """Per-goal policy: result 'ok' -> done; failure -> clear costmaps and retry up to max_retries; then skip."""

    def __init__(self, max_retries=1):
        self.max_retries = int(max_retries)
        self.attempts = 0

    def on_result(self, result):
        """Returns 'done', 'retry' (clear costmaps, resend) or 'skip'."""
        self.attempts += 1
        if result == 'ok':
            return 'done'
        if result == 'cancelled':
            return 'skip'
        return 'retry' if self.attempts <= self.max_retries else 'skip'


def stage_result(results):
    """Summarise per-waypoint results ('ok'/'skipped') -> 'ok' | 'partial' | 'skipped'."""
    if not results:
        return 'ok'
    n_ok = sum(1 for r in results if r == 'ok')
    return 'ok' if n_ok == len(results) else ('skipped' if n_ok == 0 else 'partial')


# ------------------------------------------------------------------ course
def arena_to_map(arena, start):
    """(X_mm, Y_mm, yaw_deg) in the arena frame -> (x_m, y_m, yaw_rad) in the start-pose-origin map frame."""
    sx, sy, syaw = start
    t = math.radians(syaw)
    dx, dy = arena[0] - sx, arena[1] - sy
    x = (dx * math.cos(t) + dy * math.sin(t)) / 1000.0
    y = (-dx * math.sin(t) + dy * math.cos(t)) / 1000.0
    dyaw = math.radians(arena[2] - syaw)
    return x, y, math.atan2(math.sin(dyaw), math.cos(dyaw))


def load_course(path):
    """Read competition_course.yaml -> (waypoints {name: (x, y, yaw)}, stages {stage: [names]})."""
    import yaml
    with open(path) as f:
        c = yaml.safe_load(f)['course']
    wps = {n: (float(w['x']), float(w['y']), float(w['yaw'])) for n, w in c['waypoints'].items()}
    stages = {s: list(v) for s, v in c['stages'].items()}
    for s, names in stages.items():
        for n in names:
            if n not in wps:
                raise ValueError(f'stage {s}: unknown waypoint {n}')
    return wps, stages


def check_course(path):
    """Print waypoints whose stored map pose differs from the one re-derived from `arena` + `start_arena`."""
    import yaml
    with open(path) as f:
        c = yaml.safe_load(f)['course']
    s = c['start_arena']
    start = (s['x_mm'], s['y_mm'], s['yaw_deg'])
    worst = 0.0
    for n, w in c['waypoints'].items():
        x, y, yaw = arena_to_map(w['arena'], start)
        d = max(abs(x - w['x']), abs(y - w['y']), abs(math.atan2(math.sin(yaw - w['yaw']), math.cos(yaw - w['yaw']))))
        if d > 0.002:
            print(f'{n}: stored ({w["x"]}, {w["y"]}, {w["yaw"]}) derived ({x:.3f}, {y:.3f}, {yaw:.3f})')
        worst = max(worst, d)
    print(f'max deviation {worst:.4f}')
    return worst


if __name__ == '__main__':
    check_course(sys.argv[1])

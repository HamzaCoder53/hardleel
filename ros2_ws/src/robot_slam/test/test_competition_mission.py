"""Unit tests of the pure decision logic of competition_mission (no ROS needed).

    cd ros2_ws/src/robot_slam && python3 -m pytest test/test_competition_mission.py -q
"""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from robot_slam import competition_core as core  # noqa: E402

COURSE = os.path.join(os.path.dirname(__file__), '..', 'config', 'competition_course.yaml')


# ---- state order / deadline
def test_state_order_is_the_mission_sequence():
    seq, s = ['WAIT_START'], 'WAIT_START'
    while s != 'DONE':
        s = core.next_state(s)
        seq.append(s)
    assert seq == ['WAIT_START', 'GO_SCREEN', 'CLASSIFY', 'GO_LOADING', 'LOAD', 'TERRAIN', 'OBSTACLES',
                   'JUNCTION', 'GATE_DECISION', 'GO_PATH', 'DEPOSIT', 'FINISH', 'DONE']
    assert core.next_state('DONE') == 'DONE' and core.next_state('ABORTED') == 'ABORTED'


@pytest.mark.parametrize('state', ['GO_SCREEN', 'CLASSIFY', 'LOAD', 'TERRAIN', 'GATE_DECISION', 'DEPOSIT'])
def test_deadline_jumps_to_finish(state):
    assert core.state_after_deadline(state) == 'FINISH'


@pytest.mark.parametrize('state', ['FINISH', 'DONE', 'ABORTED'])
def test_deadline_does_not_restart_endgame(state):
    assert core.state_after_deadline(state) == state


def test_deadline_arithmetic():
    d = core.Deadline(100.0, 175.0)
    assert not d.expired(274.9) and d.expired(275.0) and d.expired(300.0)
    assert d.remaining(150.0) == pytest.approx(125.0)
    assert d.elapsed(130.0) == pytest.approx(30.0)


# ---- class -> bay
@pytest.mark.parametrize('cls,bay', [('plastic', 'blue'), ('organic', 'white'), ('metal', 'black')])
def test_class_to_bay_default_mapping(cls, bay):
    assert core.bay_for_class(cls) == bay


def test_unknown_class_uses_default_class():
    assert core.bay_for_class('unknown', default_class='metal') == 'black'
    assert core.bay_for_class('', default_class='plastic') == 'blue'


def test_custom_mapping_and_bay_stage():
    m = {'plastic': 'white', 'organic': 'blue', 'metal': 'black'}
    assert core.bay_for_class('plastic', m) == 'white'
    assert core.bay_stage('white') == 'bay_white'


def test_parse_classify():
    assert core.parse_classify('plastic:0.93:blue') == ('plastic', 0.93, 'blue')
    assert core.parse_classify('unknown:0.0:') == ('unknown', 0.0, '')
    assert core.parse_classify('garbage') == ('unknown', 0.0, '')
    assert core.parse_classify(None) == ('unknown', 0.0, '')
    assert core.usable_class('metal') and not core.usable_class('unknown')


# ---- gate
def scan(n=360, value=3.0, blocked=None):
    r = [value] * n
    for i in blocked or []:
        r[i] = 0.5
    return r, -math.pi, 2 * math.pi / n


def test_gate_region_blocked_and_free():
    r, a0, inc = scan(blocked=range(175, 186))            # beams around 180 = straight ahead (angle 0)
    assert core.gate_region_free(r, a0, inc, 0.9, math.radians(20)) is False
    r, a0, inc = scan(blocked=range(0, 20))               # obstacle behind the robot only
    assert core.gate_region_free(r, a0, inc, 0.9, math.radians(20)) is True
    assert core.gate_region_free([], 0.0, 0.1, 0.9, 0.3) is None


def test_gate_region_ignores_nan_inf_and_far():
    r, a0, inc = scan(value=float('inf'))
    assert core.gate_region_free(r, a0, inc, 0.9, math.radians(20)) is True
    r = [float('nan')] * 360
    assert core.gate_region_free(r, a0, inc, 0.9, math.radians(20)) is True


def test_gate_open_in_time_chooses_path2():
    g = core.GateDecider(0.0, wait_s=15, stable_s=1.0)
    assert g.update(0.0, False) is None
    assert g.update(5.0, False) is None
    assert g.update(6.0, True) is None            # just opened
    assert g.update(6.9, True) is None
    assert g.update(7.0, True) == 'path2'
    assert g.update(30.0, False) == 'path2'       # decision is final


def test_gate_flicker_resets_stability():
    g = core.GateDecider(0.0, wait_s=15, stable_s=1.0)
    g.update(1.0, True)
    g.update(1.8, False)
    assert g.update(2.5, True) is None            # free_since restarted at 2.5
    assert g.update(3.4, True) is None
    assert g.update(3.5, True) == 'path2'


def test_gate_closed_past_timeout_chooses_path1():
    g = core.GateDecider(10.0, wait_s=15, stable_s=1.0)
    assert g.update(24.9, False) is None
    assert g.update(25.0, False) == 'path1'


def test_gate_no_scan_data_falls_back_to_path1():
    g = core.GateDecider(0.0, wait_s=5, stable_s=1.0)
    assert g.update(2.0, None) is None
    assert g.update(5.0, None) == 'path1'


def test_gate_open_already_at_start_still_needs_stable_time():
    g = core.GateDecider(0.0, wait_s=5, stable_s=0.8)
    assert g.update(0.0, True) is None
    assert g.update(0.8, True) == 'path2'


# ---- retry / skip
def test_goal_policy_success_first_time():
    assert core.GoalPolicy(1).on_result('ok') == 'done'


def test_goal_policy_one_retry_then_skip():
    p = core.GoalPolicy(1)
    assert p.on_result('failed') == 'retry'
    assert p.on_result('timeout') == 'skip'


def test_goal_policy_retry_then_success():
    p = core.GoalPolicy(1)
    assert p.on_result('rejected') == 'retry'
    assert p.on_result('ok') == 'done'


def test_goal_policy_zero_retries_and_cancel():
    assert core.GoalPolicy(0).on_result('failed') == 'skip'
    assert core.GoalPolicy(3).on_result('cancelled') == 'skip'


def test_stage_result_summary():
    assert core.stage_result([]) == 'ok'
    assert core.stage_result(['ok', 'ok']) == 'ok'
    assert core.stage_result(['ok', 'skipped']) == 'partial'
    assert core.stage_result(['skipped', 'skipped']) == 'skipped'


# ---- course file
def test_course_loads_and_covers_every_stage():
    wps, stages = core.load_course(COURSE)
    for s in ['screen', 'loading', 'terrain', 'obstacles', 'junction', 'gate_path2', 'path1_detour',
              'bay_blue', 'bay_white', 'bay_black', 'finish']:
        assert stages[s], s
    for c in core.DEFAULT_BAY_FOR_CLASS.values():
        assert core.bay_stage(c) in stages


def test_course_map_poses_match_arena_poses():
    assert core.check_course(COURSE) < 0.002


def test_arena_to_map_start_is_origin():
    x, y, yaw = core.arena_to_map((6215, 300, 90), (6215, 300, 90))
    assert (x, y, yaw) == pytest.approx((0, 0, 0))
    # 1 m ahead of the start (arena +y) is map +x; 1 m to its left (arena -x) is map +y
    assert core.arena_to_map((6215, 1300, 90), (6215, 300, 90))[:2] == pytest.approx((1.0, 0.0))
    assert core.arena_to_map((5215, 300, 90), (6215, 300, 90))[:2] == pytest.approx((0.0, 1.0))


# --- clock-aware decisions, obstacle wait cap, episodes (25 Oct) -------------------------------------------
def test_soft_deadline_skips_to_deposit_only_before_it():
    assert core.state_after_soft_deadline('TERRAIN') == 'DEPOSIT'
    assert core.state_after_soft_deadline('GATE_DECISION') == 'DEPOSIT'
    assert core.state_after_soft_deadline('DEPOSIT') == 'DEPOSIT'
    assert core.state_after_soft_deadline('FINISH') == 'FINISH'
    assert core.soft_deadline_due('GO_PATH', 30.0, 35.0)
    assert not core.soft_deadline_due('GO_PATH', 40.0, 35.0)
    assert not core.soft_deadline_due('DEPOSIT', 10.0, 35.0)


def test_gate_wait_budget():
    assert core.gate_wait_budget(120.0, 5.0, 55.0) == 5.0          # plenty of time: full wait
    assert core.gate_wait_budget(58.0, 5.0, 55.0) == pytest.approx(3.0)
    assert core.gate_wait_budget(40.0, 5.0, 55.0) == 0.0           # short: take what is open now


def test_gate_budget_zero_decides_on_one_reading():
    from robot_slam.gate_check import GatePoller
    assert GatePoller(0.0, 0.0, 1.5, 1).update(0.0, 3.0) == ('path2', 0.0)   # open now
    assert GatePoller(0.0, 0.0, 1.5, 1).update(0.0, 0.9) == ('path1', 0.0)   # closed now
    assert GatePoller(0.0, 0.0, 1.5, 1).update(0.0, None) == ('path1', 0.0)  # no data: the always-open path


def test_progress_watch_gives_up_after_max_wait_only_with_an_obstacle():
    w = core.ProgressWatch(0.0, (0.0, 0.0), max_wait_s=10.0)
    assert not w.update(5.0, (0.0, 0.0), True)
    assert w.update(10.5, (0.02, 0.0), True)                      # 10 s, no progress, obstacle in view
    w = core.ProgressWatch(0.0, (0.0, 0.0), max_wait_s=10.0)
    assert not w.update(10.5, (0.0, 0.0), False)                  # stuck but no obstacle: not this rule
    w = core.ProgressWatch(0.0, (0.0, 0.0), max_wait_s=10.0)
    w.update(6.0, (0.5, 0.0), True)                               # moved: clock restarts
    assert not w.update(12.0, (0.5, 0.0), True)
    assert w.update(16.5, (0.52, 0.0), True)


def test_obstacle_episodes():
    e = core.ObstacleEpisodes(clear_s=3.0)
    assert e.update(0.0, True) == 'start'
    assert e.update(1.0, True) is None
    assert e.update(2.0, False) is None                           # brief gap: same episode
    assert e.update(4.5, False) == 'cleared'
    assert e.update(6.0, True) == 'start'                         # a new worker: logged again


def test_path_switch_only_where_an_alternative_exists():
    assert core.path_switch_possible('JUNCTION')
    assert core.path_switch_possible('GATE_DECISION')
    assert core.path_switch_possible('GO_PATH', 'path2')
    assert not core.path_switch_possible('GO_PATH', 'path1')        # already on the open path: wait
    assert not core.path_switch_possible('TERRAIN')                 # only way is ahead: wait
    assert not core.path_switch_possible('DEPOSIT')

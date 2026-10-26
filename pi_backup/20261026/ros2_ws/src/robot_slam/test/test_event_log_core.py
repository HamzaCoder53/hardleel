"""cd ros2_ws/src/robot_slam && python3 -m pytest test/test_event_log_core.py -q"""
import json
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from robot_slam.event_log_core import MISSING, RunLog, format_summary  # noqa: E402
from robot_slam.judges_sender import HttpJsonSender, StubSender, make_sender  # noqa: E402

T0 = 1_800_000_000.0


def rows(log):
    return {r['field']: r for r in log.summary_rows()}


def test_row_order_and_all_missing():
    log = RunLog('R1', '.')
    names = [r['field'] for r in log.summary_rows()]
    assert names == ['robot_id', 'autonomous_start', 'waste_detected', 'loading_complete', 'terrain_complete',
                     'obstacle_detected', 'obstacle_response', 'gate_decision', 'gate_wait_time',
                     'deposit_complete', 'finish', 'total_run_time']
    r = rows(log)
    assert r['robot_id']['value'] == 'R1'
    assert all(r[n]['value'] == MISSING for n in names[1:])


def test_t_rel_and_total_time():
    log = RunLog('R', '.')
    log.record('autonomous_start', {}, T0)
    log.record('waste_detected', {'class': 'metal', 'confidence': 0.9}, T0 + 10)
    log.record('finish', {}, T0 + 100.5)
    r = rows(log)
    assert r['autonomous_start']['t_rel'] == 0
    assert r['waste_detected']['t_rel'] == 10 and 'class=metal' in r['waste_detected']['extra']
    assert r['total_run_time']['value'] == 100.5
    assert r['loading_complete']['value'] == MISSING


def test_repeated_event_keeps_first():
    log = RunLog('R', '.')
    log.record('autonomous_start', {}, T0)
    log.record('waste_detected', {'class': 'a'}, T0 + 5)
    o = log.record('waste_detected', {'class': 'b'}, T0 + 9)
    assert o['is_first'] is False
    assert rows(log)['waste_detected']['t_rel'] == 5
    assert len(log.to_dict()['events']) == 3


def test_derived_times():
    log = RunLog('R', '.')
    log.record('autonomous_start', {}, T0)
    log.record('obstacle_detected', {}, T0 + 20)
    log.record('obstacle_response', {'action': 'wait'}, T0 + 23)
    log.record('gate_decision', {'decision': 'path2', 'wait_s': 7.5}, T0 + 40)
    assert log.obstacle_response_time_s() == 3
    assert log.gate_wait_time_s() == 7.5
    assert rows(log)['gate_decision']['extra'] == 'decision=path2'
    log2 = RunLog('R', '.')
    log2.record('obstacle_response', {'response_s': 1.25}, T0)
    assert log2.obstacle_response_time_s() == 1.25


def test_events_before_start_get_t_rel_later():
    log = RunLog('R', '.')
    log.record('waste_detected', {}, T0 + 3)
    assert log.occurrences[0]['t_rel'] is None
    log.record('autonomous_start', {}, T0)
    assert log.occurrences[0]['t_rel'] == 3


def test_atomic_write_valid_json_and_csv(tmp_path):
    log = RunLog('R', str(tmp_path))
    log.run_id = 'run_x'
    log.record('autonomous_start', {}, T0, 12.5)
    log.record('finish', {}, T0 + 50)
    j, c = str(tmp_path / 'sub' / 'a.json'), str(tmp_path / 'sub' / 'a.csv')
    log.write(j, c)
    log.write(j, c)
    d = json.load(open(j))
    assert d['robot_id'] == 'R' and d['run_id'] == 'run_x' and len(d['events']) == 2
    assert d['derived']['total_run_time_s'] == 50
    lines = open(c).read().strip().splitlines()
    assert lines[0] == 'wall_iso,ros_time,t_rel,event,detail_json' and len(lines) == 3
    assert not [f for f in os.listdir(tmp_path / 'sub') if f.endswith('.tmp')]


def test_format_summary_and_senders():
    log = RunLog('R', '.')
    log.record('autonomous_start', {}, T0)
    s = format_summary(log, 'GO_SCREEN', T0 + 4)
    assert 'GO_SCREEN' in s and '4.0s' in s and 'finish' in s
    out = []
    st = make_sender('stub', logger=out.append)
    assert isinstance(st, StubSender) and st.send_event('R', 'x', {'a': 1}) and out
    h = make_sender('http', url='http://127.0.0.1:9/none', timeout=0.2, logger=out.append)
    assert isinstance(h, HttpJsonSender) and h.send_event('R', 'x', {}) is False

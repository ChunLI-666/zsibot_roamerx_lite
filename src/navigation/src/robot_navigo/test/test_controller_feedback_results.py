"""Regression tests for evaluation errors that could hide an unsafe outcome."""
import csv
import importlib.util
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts' / 'controller_feedback'


def load(name):
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_trace(path, success):
    rows = []
    for index, reached in enumerate(success):
        rows.append(dict(step=index, t=index*.1, raw_vx=-.01, vx=0, raw_wz=.005,
                         xy_error=.1, yaw_error=.1, collision=0, exception='',
                         success=int(reached)))
    with path.open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)


def test_early_arrival_followed_by_drift_is_not_a_success(tmp_path):
    path = tmp_path/'drift.csv'
    write_trace(path, [True, False])
    result = load('run_cases').summarize(path, 1)
    assert result['reached_any']
    assert not result['success']


def test_deadband_command_does_not_count_as_executed_reverse_distance(tmp_path):
    path = tmp_path/'deadband.csv'
    write_trace(path, [False, False])
    result = load('run_cases').summarize(path, 1)
    assert result['negative_raw_vx_samples'] == 2
    assert result['reverse_command_distance_m'] == 0


def test_shadow_and_intentional_hold_are_not_reachable_trial_denominators():
    kind = load('analyze_results').kind
    assert kind('0829_reverse_20s_shadow_seed42') == 'shadow'
    assert kind('warehouse_blocked_rotation_seed42') == 'intentional_hold'
    assert kind('warehouse_tiny_limit_seed42') == 'intentional_hold'
    assert kind('warehouse_heading_180_seed42') == 'reachable_feedback'


def test_crash_or_timeout_does_not_score_stale_success(tmp_path):
    path = tmp_path/'crash.csv'
    write_trace(path, [True, True])
    summarize = load('run_cases').summarize
    assert not summarize(path, 139)['success']
    assert not summarize(path, 124)['success']


def test_collision_cannot_score_success_even_with_zero_returncode(tmp_path):
    path = tmp_path/'collision.csv'
    write_trace(path, [True, True])
    rows = list(csv.DictReader(path.open()))
    rows[-1]['collision'] = '1'
    with path.open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)
    assert not load('run_cases').summarize(path, 0)['success']


def continuous_trace(path, terminal_collision=False):
    rows=[]
    for index in range(3):
        rows.append(dict(step=index,t=index*.1,x=-index*.01,y=0,yaw=0,
            vx=-.1,vy=0,wz=.1,raw_vx=-.1,raw_vy=0,raw_wz=.1,
            xy_error=1,yaw_error=1,collision=int(terminal_collision and index==2),
            exception='',success=0,motion_mode='',collision_reason=''))
    with path.open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=rows[0]);writer.writeheader();writer.writerows(rows)


def test_continuous_metrics_censor_terminal_interval(tmp_path):
    path=tmp_path/'trace.csv'
    continuous_trace(path,terminal_collision=True)
    result=load('continuous_audit').metrics(path,4)
    assert abs(result['reverse_distance_m']-.02)<1e-9
    assert abs(result['longest_reverse_sec']-.2)<1e-9
    assert result['collision_checks_failed']==1
    assert not result['success']


def test_continuous_rotation_is_not_stationary(tmp_path):
    path=tmp_path/'trace.csv'
    continuous_trace(path)
    rows=list(csv.DictReader(path.open()))
    for row in rows:
        row['vx']='0';row['raw_vx']='-.01'
    with path.open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=rows[0]);writer.writeheader();writer.writerows(rows)
    result=load('continuous_audit').metrics(path,1)
    assert result['longest_stationary_sec']==0
    assert result['reverse_distance_m']==0
    assert result['negative_raw_samples']==3

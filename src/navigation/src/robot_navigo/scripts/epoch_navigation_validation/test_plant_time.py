import math
import pytest
from plant_time import advance_interval
from scene import integrate


def test_stall_integrates_until_watchdog_then_stops_without_lost_time():
    pose, executed, steps = advance_interval([0.,0.,0.],[.1,0.,0.],0,1_000_000_000,0)
    assert pose == pytest.approx([.035,0.,0.])
    assert executed == [0.,0.,0.]
    assert sum(s['dt'] for s in steps) == pytest.approx(1.)
    assert all(a['interval_end_ns']==b['interval_start_ns'] for a,b in zip(steps,steps[1:]))
    assert steps[0]['interval_start_ns']==0 and steps[-1]['interval_end_ns']==1_000_000_000
    assert max(s['dt'] for s in steps if any(s['executed'])) <= .02


def test_command_switch_settles_old_velocity_before_new_command():
    pose, _, first = advance_interval([0.,0.,0.],[.1,0.,0.],0,200_000_000,0)
    pose, _, second = advance_interval(pose,[-.1,0.,0.],200_000_000,400_000_000,200_000_000)
    assert pose == pytest.approx([0.,0.,0.],abs=1e-15)
    assert all(s['executed'][0] > 0 for s in first)
    assert all(s['executed'][0] < 0 for s in second)


def test_watchdog_exact_boundary_and_receipt_in_future():
    pose, executed, _ = advance_interval([0.,0.,0.],[.1,0.,0.],0,350_000_000,0)
    assert pose[0] == pytest.approx(.035)
    assert executed == [0.,0.,0.]
    pose, _, steps = advance_interval([0.,0.,0.],[.1,0.,0.],0,400_000_000,200_000_000)
    assert pose[0] == pytest.approx(.02)
    assert steps[0]['executed'] == [0.,0.,0.]


def test_constraint_transition_preserves_motion_before_freeze():
    pose, _, _ = advance_interval([0.,0.,0.],[.1,0.,0.],0,200_000_000,0)
    pose, executed, _ = advance_interval(pose,[.1,0.,0.],200_000_000,400_000_000,0,blocked=True)
    assert pose[0] == pytest.approx(.02)
    assert executed == [0.,0.,0.]


def test_exact_body_twist_arc_is_independent_of_subdivision():
    command = [.1,.03,math.pi/2]
    whole = integrate([0.,0.,0.],command,1.)
    parts = [0.,0.,0.]
    for _ in range(100):
        parts = integrate(parts,command,.01)
    assert whole == pytest.approx(parts,abs=1e-14)
    assert whole[:2] == pytest.approx([.07/(math.pi/2),.13/(math.pi/2)])
    assert integrate([0.,0.,0.],[.1,.03,1e-12],1.) == pytest.approx([.1,.03,1e-12],abs=1e-12)


@pytest.mark.parametrize('end,command', [(-1,[0.,0.,0.]),(1,[math.nan,0.,0.])])
def test_invalid_intervals_and_nonfinite_motion_rejected(end,command):
    with pytest.raises(ValueError):
        advance_interval([0.,0.,0.],command,0,end,0)

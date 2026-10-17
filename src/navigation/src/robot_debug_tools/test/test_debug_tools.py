from action_msgs.msg import GoalStatus
import pytest
from robot_debug_msgs.msg import StatusObservation, CheckerSample, BranchSample
from robot_debug_tools.model import ObservationStore, uuid_key
from robot_debug_tools.build import stamp_from_dict


def native_status(value, identifier=1):
    status = GoalStatus(status=value)
    status.goal_info.goal_id.uuid[0] = identifier
    return status


def sample(identifier=1, revision=1):
    msg = StatusObservation(component='controller', source_instance='instance',
                            endpoint='/follow_path', goal_id_valid=True, path_revision=revision)
    msg.goal_id.uuid[0] = identifier
    msg.checker = [CheckerSample(evaluated=True, result_valid=True, result=True)]
    return msg


def test_current_checker_never_crosses_goal_or_terminal():
    model = ObservationStore()
    model.action('/follow_path', native_status(2), native_status(2).goal_info.stamp)
    model.observation(sample())
    assert model.current_checker()[0].checker[0].result is True
    model.action('/follow_path', native_status(5), native_status(5).goal_info.stamp)
    assert model.current_checker()[0] is None
    model.action('/follow_path', native_status(2, 2), native_status(2).goal_info.stamp)
    assert model.current_checker()[0] is None


def test_new_path_and_failed_precondition_invalidate_old_geometry():
    model = ObservationStore()
    model.action('/follow_path', native_status(2), native_status(2).goal_info.stamp)
    model.observation(sample())
    update = sample(revision=2)
    update.checker = []
    update.branch = [BranchSample(site='path_update', branch='installed')]
    model.observation(update)
    assert model.current_checker()[1] == 'path_replaced_since_evaluation'
    invalid = sample(revision=2)
    invalid.checker[0].evaluated = False
    invalid.checker[0].result_valid = False
    model.observation(invalid)
    assert model.current_checker()[1] == 'checker_not_evaluated'


def test_multiple_native_uuids_are_not_collapsed_to_current_first():
    model = ObservationStore()
    for identifier in (1, 2):
        model.action('/follow_path', native_status(2, identifier), native_status(2).goal_info.stamp)
    model.observation(sample())
    assert len(model.actions) == 2
    assert model.current_checker()[1] == 'ambiguous_executing_follow_path'


def test_unknown_source_version_is_not_invented():
    assert not stamp_from_dict({}).known
    assert not stamp_from_dict({'known': True, 'git_commit': 'main'}).known
    stamp = stamp_from_dict({'known': True, 'git_commit': 'a' * 40, 'dirty_state': 2})
    assert stamp.known and stamp.dirty_state == 2


def test_checker_is_not_current_after_observation_gap_or_endpoint_collision():
    now = [0.0]
    model = ObservationStore(clock=lambda: now[0])
    model.action('/other/follow_path', native_status(2), native_status(2).goal_info.stamp)
    model.observation(sample())
    assert model.current_checker()[0] is None
    model.clear()
    model.action('/follow_path', native_status(2), native_status(2).goal_info.stamp)
    model.observation(sample())
    assert model.current_checker()[0] is not None
    now[0] = 2.0
    assert model.current_checker()[1] == 'checker_observation_not_recent'


@pytest.mark.parametrize('active_status', [1, 2, 3])
def test_long_native_task_survives_many_completed_planner_goals(active_status):
    model = ObservationStore()
    task = native_status(active_status)
    task_key = ('/navigate_to_pose', uuid_key(task.goal_info.goal_id))
    model.action('/navigate_to_pose', task, task.goal_info.stamp)
    for identifier in range(10, 110):
        completed = native_status(4 + identifier % 3, identifier)
        model.action('/compute_path_to_pose', completed, completed.goal_info.stamp)
    assert model.actions[task_key].value == active_status
    assert len(model.actions) == 64
    assert model.dropped == 37


def test_oldest_terminal_is_evicted_before_older_active_task():
    model = ObservationStore()
    task = native_status(2)
    task_key = ('/navigate_to_pose', uuid_key(task.goal_info.goal_id))
    model.action('/navigate_to_pose', task, task.goal_info.stamp)
    for identifier in range(2, 66):
        completed = native_status(4, identifier)
        model.action('/compute_path_to_pose', completed, completed.goal_info.stamp)
    old = native_status(4, 2)
    newest = native_status(4, 65)
    assert task_key in model.actions
    assert ('/compute_path_to_pose', uuid_key(old.goal_info.goal_id)) not in model.actions
    assert ('/compute_path_to_pose', uuid_key(newest.goal_info.goal_id)) in model.actions
    assert len(model.actions) == 64
    assert model.dropped == 1


def test_completed_task_no_longer_has_active_eviction_priority():
    model = ObservationStore()
    for identifier in range(1, 65):
        active = native_status(2, identifier)
        model.action('/navigate_to_pose', active, active.goal_info.stamp)
    completed = native_status(4)
    model.action('/navigate_to_pose', completed, completed.goal_info.stamp)
    accepted = native_status(1, 65)
    model.action('/navigate_to_pose', accepted, accepted.goal_info.stamp)
    assert ('/navigate_to_pose', uuid_key(completed.goal_info.goal_id)) not in model.actions
    assert len(model.actions) == 64
    assert all(state.value in (1, 2, 3) for state in model.actions.values())
    assert model.dropped == 1


def test_all_active_overflow_is_bounded_and_records_each_drop():
    model = ObservationStore()
    for identifier in range(1, 91):
        active = native_status(2, identifier)
        model.action('/navigate_to_pose', active, active.goal_info.stamp)
        assert len(model.actions) <= 64
    oldest_retained = native_status(2, 27)
    assert next(iter(model.actions)) == (
        '/navigate_to_pose', uuid_key(oldest_retained.goal_info.goal_id))
    assert model.dropped == 26
    assert all(state.value == 2 for state in model.actions.values())

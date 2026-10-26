"""Pure graph mocks: no ROS node, recorder, player or synthetic messages."""
import ast
from pathlib import Path
from types import MethodType, SimpleNamespace
from unittest.mock import Mock

import pytest


def actual_methods():
    tree = ast.parse(Path(__file__).with_name('observer.py').read_text())
    monitor = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'Monitor')
    names = {'recorder_topics', 'recorder_ready'}
    selected = [node for node in tree.body if isinstance(node, ast.FunctionDef)
                and node.name == 'recorder_endpoint_evidence']
    selected.extend(node for node in monitor.body if isinstance(node, ast.FunctionDef) and node.name in names)
    namespace = {}
    exec(compile(ast.Module(body=selected, type_ignores=[]), '<actual recorder readiness>', 'exec'), namespace)
    return namespace


def endpoint(topic_type, node='rosbag2_recorder'):
    duration = SimpleNamespace(nanoseconds=123)
    return SimpleNamespace(node_name=node, node_namespace='/capture', topic_type=topic_type,
        endpoint_type=SimpleNamespace(name='SUBSCRIPTION'), endpoint_gid=bytes(range(24)),
        qos_profile=SimpleNamespace(history=SimpleNamespace(name='KEEP_LAST'), depth=2000,
            reliability=SimpleNamespace(name='RELIABLE'), durability=SimpleNamespace(name='VOLATILE'),
            liveliness=SimpleNamespace(name='AUTOMATIC'), avoid_ros_namespace_conventions=False,
            deadline=duration, lifespan=duration, liveliness_lease_duration=duration))


def mock_monitor(tmp_path, full=True, debug=True, plant=False, extras=()):
    config = tmp_path / 'source.yaml'
    config.write_text('debug:\n  enable: ' + str(debug).lower() + '\n')
    args = SimpleNamespace(output=tmp_path, config=config, full_bag_mode=full,
        record_topics_extra=list(extras), observer_off=True, observations_off=True)
    monitor = SimpleNamespace(args=args, plant=plant, goal_input='topic',
        raw_source_types={'/livox/imu': 'sensor_msgs/msg/Imu'},
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=99)),
        records=[], resume_source=Mock())
    methods = actual_methods()
    for name in ('recorder_topics', 'recorder_ready'):
        setattr(monitor, name, MethodType(methods[name], monitor))
    monitor.record = lambda name, data: monitor.records.append((name, data))
    graph = {topic: [endpoint(kind)] for topic, kind in monitor.recorder_topics().items()}
    monitor.get_subscriptions_info_by_topic = lambda topic: graph.get(topic, [])
    def spin_until(predicate, seconds, label, processes):
        if not predicate():
            raise TimeoutError(label)
    monitor.spin_until = spin_until
    return monitor, graph


@pytest.mark.parametrize('missing', ['/odom/current_pose', '/lightning/pose', '/lightning/debug', '/livox/imu'])
def test_missing_native_or_selected_raw_subscription_blocks_resume(tmp_path, missing):
    monitor, graph = mock_monitor(tmp_path, extras=['/livox/imu'])
    del graph[missing]
    with pytest.raises(TimeoutError, match='all pre-resume topics'):
        monitor.recorder_ready(object())
        monitor.resume_source()
    monitor.resume_source.assert_not_called()
    assert monitor.records[-1][0] == 'recorder_not_ready'
    assert missing in monitor.records[-1][1]['missing_topics']


def test_full_enabled_topics_ignore_unrelated_nav_observer_switches(tmp_path):
    monitor, _ = mock_monitor(tmp_path, extras=['/livox/imu'])
    required = monitor.recorder_topics()
    assert set(required) == {'/clock', '/goal_pose', '/odom/current_pose',
                             '/lightning/pose', '/lightning/debug', '/livox/imu'}
    assert not {'/cmd_vel_safe', '/map', '/plan', '/debug/observations'} & set(required)


@pytest.mark.parametrize('full,plant', [(False, False), (True, True)])
def test_native_readiness_only_gates_full_real_sensor_replay(tmp_path, full, plant):
    monitor, _ = mock_monitor(tmp_path, full=full, plant=plant)
    assert set(monitor.recorder_topics()) == {'/clock', '/goal_pose'}


def test_debug_yaml_disabled_and_effective_config_takes_precedence(tmp_path):
    monitor, _ = mock_monitor(tmp_path, debug=True)
    (tmp_path / 'lightning_config.yaml').write_text('debug:\n  enable: false\n')
    required = monitor.recorder_topics()
    assert '/lightning/debug' not in required
    assert '/odom/current_pose' in required and '/lightning/pose' in required


def test_unrelated_node_or_wrong_datatype_cannot_satisfy_gate(tmp_path):
    monitor, graph = mock_monitor(tmp_path)
    graph['/lightning/pose'] = [endpoint('geometry_msgs/msg/PoseStamped', 'observer'),
                              endpoint('std_msgs/msg/String')]
    with pytest.raises(TimeoutError):
        monitor.recorder_ready(object())


def test_all_endpoints_appear_before_resume_with_exact_endpoint_evidence(tmp_path):
    monitor, graph = mock_monitor(tmp_path, extras=['/livox/imu'])
    delayed = graph.pop('/lightning/debug')
    def spin_until(predicate, seconds, label, processes):
        assert not predicate()
        monitor.resume_source.assert_not_called()
        graph['/lightning/debug'] = delayed
        assert predicate()
    monitor.spin_until = spin_until
    # No spin_for method exists: readiness must use discovery, never a sleep.
    monitor.recorder_ready(object())
    monitor.resume_source()
    monitor.resume_source.assert_called_once()
    name, data = monitor.records[-1]
    assert name == 'recorder_ready'
    assert set(data['subscription_endpoints']) == set(monitor.recorder_topics())
    evidence = data['subscription_endpoints']['/lightning/debug'][0]
    assert evidence['node_name'] == 'rosbag2_recorder'
    assert evidence['node_namespace'] == '/capture'
    assert evidence['topic_type'] == 'lightning/msg/LightningDebug'
    assert evidence['endpoint_type'] == 'SUBSCRIPTION'
    assert evidence['endpoint_gid'] == list(range(24))
    assert evidence['qos']['reliability'] == 'RELIABLE'
    assert evidence['qos']['depth'] == 2000
    assert evidence['qos']['deadline_ns'] == 123


def test_zero_clock_still_blocks_complete_graph(tmp_path):
    monitor, _ = mock_monitor(tmp_path)
    monitor.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=0))
    with pytest.raises(TimeoutError):
        monitor.recorder_ready(object())


def test_canonical_replay_orders_recorder_gate_before_stack_and_resume():
    tree = ast.parse(Path(__file__).with_name('replay.py').read_text())
    calls = sorted((node.lineno, node.func.attr if isinstance(node.func, ast.Attribute)
                    else node.func.id if isinstance(node.func, ast.Name) else '')
                   for node in ast.walk(tree) if isinstance(node, ast.Call))
    gate = next(line for line, name in calls if name == 'recorder_ready')
    stack = next(line for line, name in calls if name == 'start_stack')
    resume = next(line for line, name in calls if name == 'resume_source')
    assert gate < stack < resume

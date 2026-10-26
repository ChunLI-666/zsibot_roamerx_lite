"""Pure mock lifecycle regressions; no ROS imports, services or workloads start."""
import ast
from pathlib import Path
from types import MethodType, SimpleNamespace
from unittest.mock import Mock

import pytest

from replay_support import finish_original_source


def method_from_source():
    tree = ast.parse(Path(__file__).with_name('observer.py').read_text())
    monitor = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'Monitor')
    method = next(node for node in monitor.body if isinstance(node, ast.FunctionDef)
                  and node.name == 'lifecycle_shutdown')
    request = type('Request', (), {'SHUTDOWN': 4})
    namespace = dict(ManageLifecycleNodes=SimpleNamespace(Request=request),
                     GetState=SimpleNamespace(Request=lambda: object()))
    exec(compile(ast.Module(body=[method], type_ignores=[]), '<actual lifecycle_shutdown>', 'exec'), namespace)
    return namespace['lifecycle_shutdown']


def mock_monitor(success=True):
    state = dict(done=True)
    future = SimpleNamespace(done=lambda: state['done'], result=lambda: SimpleNamespace(success=success))
    manager = SimpleNamespace(service_is_ready=lambda: True, call_async=Mock(return_value=future))
    events = []
    def create_client(kind, name):
        if name.endswith('/manage_nodes'):
            return manager
        state_future = SimpleNamespace(done=lambda: True,
            result=lambda: SimpleNamespace(current_state=SimpleNamespace(id=4)))
        return SimpleNamespace(service_is_ready=lambda: True, call_async=lambda request: state_future)
    def spin_until(predicate, timeout, label, processes):
        for _ in range(3):
            if predicate():
                return
        raise RuntimeError('player exited prematurely: 0')
    monitor = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=99)),
        record=lambda name, data: events.append(name), create_client=Mock(side_effect=create_client),
        spin_until=spin_until)
    monitor.lifecycle_shutdown = MethodType(method_from_source(), monitor)
    return monitor, manager, state, events


def test_exact_phase_events_order_and_single_shutdown_request():
    monitor, manager, _, events = mock_monitor()
    monitor.lifecycle_shutdown(object())
    manager.call_async.assert_called_once()
    assert events[:6] == ['lifecycle_shutdown_begin', 'lifecycle_shutdown_client_create_started',
        'lifecycle_shutdown_client_created', 'lifecycle_shutdown_service_ready',
        'lifecycle_shutdown_send_started', 'lifecycle_shutdown_requested']
    assert events[-2:] == ['lifecycle_shutdown_response', 'lifecycle_finalized']


def test_finally_reuses_pending_future_instead_of_sending_second_shutdown():
    monitor, manager, state, events = mock_monitor()
    state['done'] = False
    with pytest.raises(RuntimeError, match='player exited prematurely'):
        monitor.lifecycle_shutdown(object())
    original = monitor._lifecycle_shutdown_future
    state['done'] = True
    monitor.lifecycle_shutdown(object())
    assert monitor._lifecycle_shutdown_future is original
    manager.call_async.assert_called_once()
    assert 'lifecycle_shutdown_existing_request_reused' in events


def test_negative_shutdown_response_is_not_hidden_by_reuse():
    monitor, manager, _, _ = mock_monitor(success=False)
    for _ in range(2):
        with pytest.raises(AssertionError, match='returned failure'):
            monitor.lifecycle_shutdown(object())
    manager.call_async.assert_called_once()


def test_eof_before_finalization_still_fails_strict_teardown_contract():
    player = SimpleNamespace(returncode=None)
    player.poll = lambda: player.returncode
    def shutdown(processes):
        player.returncode = 0
    monitor = SimpleNamespace(lifecycle_shutdown=shutdown,
        record=Mock(), get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=99)),
        spin_until=Mock())
    with pytest.raises(AssertionError, match='ended before navigation teardown completed'):
        finish_original_source(monitor, object(), player, dict(end_ns=100), {}, 1.)
    monitor.spin_until.assert_not_called()
    assert monitor.record.call_args.args[1]['player_still_running'] is False

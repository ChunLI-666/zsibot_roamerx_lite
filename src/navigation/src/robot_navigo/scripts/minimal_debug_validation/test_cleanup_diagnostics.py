"""Exercise actual lifecycle methods with fake services; no ROS imports or nodes."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest


HERE = Path(__file__).resolve().parent
TREE = ast.parse((HERE / 'observer.py').read_text())
CLASS = next(node for node in TREE.body if isinstance(node, ast.ClassDef) and node.name == 'Monitor')
METHODS = {'_cleanup_diagnostic_record', '_prepare_cleanup_diagnostics',
           '_cleanup_future_snapshot', '_diagnose_lifecycle_timeout', 'lifecycle_shutdown'}
EXTRACTED = ast.ClassDef(name='Monitor', bases=[], keywords=[],
    body=[node for node in CLASS.body if isinstance(node, ast.FunctionDef) and node.name in METHODS],
    decorator_list=[])


class Request:
    SHUTDOWN = 4


Manage = SimpleNamespace(Request=Request)
GetState = SimpleNamespace(Request=Request)
SPACE = dict(ManageLifecycleNodes=Manage, GetState=GetState)
exec(compile(ast.fix_missing_locations(ast.Module(body=[EXTRACTED], type_ignores=[])),
             str(HERE / 'observer.py'), 'exec'), SPACE)
Monitor = SPACE['Monitor']


class Future:
    def __init__(self, answer=None, done=True):
        self.answer, self.ready = answer, done
        self.callbacks = []
        self.callback_error = False

    def done(self):
        return self.ready

    def cancelled(self):
        return False

    def exception(self):
        return None

    def result(self):
        return self.answer

    def add_done_callback(self, callback):
        if self.callback_error:
            raise RuntimeError('callback registration error')
        self.callbacks.append(callback)
        if self.ready:
            callback(self)

    def complete(self):
        self.ready = True
        for callback in self.callbacks:
            callback(self)


class Client:
    def __init__(self, future, ready=True):
        self.future, self.ready, self.calls = future, ready, []
        self.ready_error = False

    def service_is_ready(self):
        if self.ready_error:
            raise RuntimeError('graph unavailable')
        return self.ready

    def call_async(self, request):
        self.calls.append(request)
        return self.future


class Harness:
    def __init__(self, diagnostics=False, pending=False, arm_runtime=False):
        self.node = Monitor()
        self.node.args = SimpleNamespace(arm_runtime=arm_runtime)
        self.node.cleanup_diagnostics = diagnostics
        self.events, self.created, self.waits = [], [], []
        self.original_error = TimeoutError('standard lifecycle shutdown response')
        self.late_response = False
        self.manager = Client(Future(SimpleNamespace(success=True), done=not pending))
        self.states_ready = True
        self.node.record = lambda event, data: self.events.append((event, data))
        self.node.create_client = self.create_client
        self.node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=123))
        self.node.spin_until = self.spin_until
        if diagnostics:
            self.node._prepare_cleanup_diagnostics()

    def create_client(self, kind, name):
        self.created.append((kind, name))
        if name.endswith('/manage_nodes'):
            return self.manager
        return Client(Future(SimpleNamespace(current_state=SimpleNamespace(id=4, label='finalized'))),
                      ready=self.states_ready)

    def spin_until(self, predicate, seconds, label, processes=None):
        self.waits.append((label, seconds))
        if label == 'standard lifecycle shutdown response' and not predicate():
            raise self.original_error
        if label == 'diagnostic lifecycle state snapshots' and self.late_response:
            self.manager.future.complete()
        for _ in range(3):
            if predicate():
                return
        raise TimeoutError(label)


class LifecycleDiagnosticsTest(unittest.TestCase):
    def test_arm_runtime_diagnostics_and_shutdown_only_query_installed_nodes(self):
        for diagnostics in (False, True):
            with self.subTest(diagnostics=diagnostics):
                h = Harness(diagnostics=diagnostics, arm_runtime=True)
                h.node.lifecycle_shutdown(None)
                self.assertFalse(any('/waypoint_follower/' in name for _, name in h.created))
                final = h.events[-1]
                self.assertEqual(final[0], 'lifecycle_finalized')
                self.assertEqual(len(final[1]['nodes']), 7)
                self.assertIn('smoother_server', final[1]['nodes'])
                self.assertEqual(len(h.manager.calls), 1)

    def test_default_path_remains_lazy_and_normal(self):
        h = Harness()
        self.assertEqual(h.created, [])
        h.node.lifecycle_shutdown(None)
        self.assertEqual(len(h.created), 9)
        self.assertEqual(len(h.manager.calls), 1)
        self.assertFalse(any(name.startswith('lifecycle_diagnostic') for name, _ in h.events))
        self.assertEqual(h.events[-1][0], 'lifecycle_finalized')

    def test_precreated_client_does_not_send_a_request(self):
        h = Harness(diagnostics=True)
        self.assertEqual(len(h.created), 9)
        self.assertEqual(h.manager.calls, [])
        h.node.lifecycle_shutdown(None)
        self.assertEqual(sum(name.endswith('/manage_nodes') for _, name in h.created), 1)
        self.assertEqual(len(h.manager.calls), 1)
        self.assertEqual(h.manager.calls[0].command, 4)
        self.assertEqual(h.events[-1][0], 'lifecycle_finalized')

    def test_late_success_and_finalized_states_cannot_clear_timeout(self):
        h = Harness(diagnostics=True, pending=True)
        h.late_response = True
        with self.assertRaises(TimeoutError) as caught:
            h.node.lifecycle_shutdown(None)
        self.assertIs(caught.exception, h.original_error)
        self.assertEqual(len(h.manager.calls), 1)
        self.assertTrue(h.manager.future.done())
        self.assertFalse(any(name in ('lifecycle_shutdown_response', 'lifecycle_finalized') for name, _ in h.events))
        states = next(data for name, data in h.events if name == 'lifecycle_diagnostic_states')
        self.assertEqual(len(states['states']), 8)
        self.assertTrue(all(item['state_id'] == 4 for item in states['states'].values()))
        self.assertTrue(states['original_shutdown_failed'])

    def test_reentry_keeps_original_future_and_sends_no_second_shutdown(self):
        h = Harness(diagnostics=True, pending=True)
        for _ in range(2):
            with self.assertRaises(TimeoutError):
                h.node.lifecycle_shutdown(None)
        self.assertIs(h.node._lifecycle_shutdown_future, h.manager.future)
        self.assertEqual(len(h.manager.calls), 1)
        self.assertTrue(any(name == 'lifecycle_shutdown_existing_request_reused' for name, _ in h.events))

    def test_missing_state_services_have_one_common_five_second_deadline(self):
        h = Harness(diagnostics=True, pending=True)
        for client in h.node._cleanup_state_clients.values():
            client.ready = False
        with self.assertRaises(TimeoutError) as caught:
            h.node.lifecycle_shutdown(None)
        self.assertIs(caught.exception, h.original_error)
        self.assertEqual(h.waits.count(('diagnostic lifecycle state snapshots', 5.)), 1)
        states = next(data for name, data in h.events if name == 'lifecycle_diagnostic_states')
        self.assertEqual(len(states['states']), 8)
        self.assertTrue(all(item['status'] == 'service_not_ready' for item in states['states'].values()))

    def test_diagnostic_service_error_cannot_replace_original_failure(self):
        h = Harness(diagnostics=True, pending=True)
        next(iter(h.node._cleanup_state_clients.values())).ready_error = True
        with self.assertRaises(TimeoutError) as caught:
            h.node.lifecycle_shutdown(None)
        self.assertIs(caught.exception, h.original_error)
        self.assertEqual(len(h.manager.calls), 1)

    def test_diagnostic_callback_error_cannot_break_a_successful_shutdown(self):
        h = Harness(diagnostics=True)
        h.manager.future.callback_error = True
        h.node.lifecycle_shutdown(None)
        self.assertTrue(any(name == 'lifecycle_diagnostic_callback_error' for name, _ in h.events))
        self.assertEqual(h.events[-1][0], 'lifecycle_finalized')

    def test_opt_in_defaults_false_and_is_forwarded_only_when_enabled(self):
        init = next(node for node in CLASS.body if isinstance(node, ast.FunctionDef) and node.name == '__init__')
        self.assertIn("getattr(args, 'cleanup_diagnostics', False)", ast.unparse(init))
        tree = ast.parse((HERE / 'closed_loop.py').read_text())
        forwards = [node for node in ast.walk(tree) if isinstance(node, ast.If)
                    and ast.unparse(node.test) == 'args.cleanup_diagnostics']
        self.assertEqual(len(forwards), 1)
        self.assertIn("command += ['--cleanup-diagnostics']", ast.unparse(forwards[0]))


if __name__ == '__main__':
    unittest.main()

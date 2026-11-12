#!/usr/bin/python3
"""Recorder parent process integration with harmless stand-ins only."""
import hashlib
import json
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time
import unittest
import uuid

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import full_scene_recording_owner as owner

ROOT = Path(sys.argv.pop(1)).resolve() if len(sys.argv) > 1 and not sys.argv[1].startswith('-') else HERE/'standin_tests_v1'
ROOT.mkdir(exist_ok=False)


def wait_for(check, timeout=5.):
    until = time.monotonic() + timeout
    while time.monotonic() < until:
        value = check()
        if value:
            return value
        time.sleep(.02)
    raise AssertionError('fixture wait timed out')


class ParentTests(unittest.TestCase):
    def setUp(self):
        self.path = ROOT/self._testMethodName
        self.path.mkdir()
        self.children = []

    def tearDown(self):
        for process in self.children:
            if process.poll() is None:
                # Exact, locally created harmless fixture only. No ROS process is started or signalled.
                fd = os.pidfd_open(process.pid)
                try:
                    signal.pidfd_send_signal(fd, signal.SIGINT)
                finally:
                    os.close(fd)
                try:
                    process.wait(timeout=3.)
                except subprocess.TimeoutExpired:
                    self.fail('harmless fixture did not stop')

    def config(self, mode='missing', seconds=6., wait=.7):
        rid, cid = uuid.uuid4().hex, uuid.uuid4().hex
        artifact = self.path/'artifact.txt'
        artifact.write_text('fixture only; no production dependency\n')
        artifacts = {role:dict(path=str(artifact), sha256=owner.sha(artifact), kind='script') for role in owner.REQUIRED_ARTIFACT_ROLES}
        artifacts['fixture_elf'] = dict(path=sys.executable, sha256=owner.sha(sys.executable), kind='elf')
        expected_env = {key:os.environ.get(key, '') for key in owner.ENV_BINDING}
        expected_env.update(owner.ENV_REQUIRED)
        expected = dict(run_id=cid, case_id=self._testMethodName, recording_run_id=rid,
            output_directory=str(self.path/'run/controller'), root_release_path=str(self.path/'controller_release.json'),
            expected_argv=[sys.executable, str(HERE/'_test_standins.py'), 'controller', str(self.path/'config.json'), mode],
            max_runtime_sec=1., route_sha256=owner.sha(artifact), oracle_sha256=owner.sha(artifact), sources_sha256={'fixture':owner.sha(artifact)})
        config = dict(schema_version=1, scope=owner.SCOPE, executable=True, recording_run_id=rid, case_id=self._testMethodName,
            max_runtime_sec=seconds, controller_wait_sec=wait, controller=expected, scene_instance_id={'fixture':True},
            required_recorded_topics=['/fixture'], record_all_including_hidden=True, original_cdr_no_reencoding=True,
            expected_environment=expected_env, artifacts=artifacts, artifact_inventory_complete_root_reviewed=True,
            source_processes=[dict(role='unrelated_harmless_fixture_harness',identity=owner.deadline.process_identity(os.getpid()))],
            repositories=[{'fixture_only':True}])
        owner.write(self.path/'config.json', config)
        release = dict(scope=owner.SCOPE, allow_execute=True, recording_run_id=rid, case_id=self._testMethodName,
            output_directory=str(self.path/'run'), config_sha256=owner.sha(self.path/'config.json'),
            boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(), network_namespace=os.readlink('/proc/self/ns/net'),
            package_sha256={name:owner.sha(HERE/name) for name in owner.PACKAGE})
        owner.write(self.path/'release.json', release)
        return config

    def start(self, config, exit_parent=False):
        log = (self.path/'runner.log').open('w')
        process = subprocess.Popen([sys.executable, str(HERE/'_test_runner.py'), str(self.path)] + (['exit'] if exit_parent else []),
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        log.close()
        self.children.append(process)
        wait_for(lambda: (self.path/'run/recorder_ready.json').exists() or process.poll() is not None)
        self.assertIsNone(process.poll(), (self.path/'runner.log').read_text())
        ready = owner.read_json(self.path/'run/recorder_ready.json')
        self.assertFalse(ready['authorizes_controller_or_motion'])
        self.assertEqual(ready['node_name'], ready['recorder_node_name'])
        self.assertEqual(ready['node_namespace'], '/')
        registry = owner.read_json(self.path/'run/recording_deadline_children_armed.json')
        self.assertEqual(list(registry['registrations']), ['recorder'])
        return process, ready

    def bind(self, config, ready, wrong_identity=False, wrong_scene=False):
        expected = config['controller']
        release = dict(scope='MAPPING_ACQUISITION_ONLY', mode='controlled_static_sim_scene', allow_execute=True,
            release_id=expected['run_id'], case_id=expected['case_id'], recording_run_id=expected['recording_run_id'],
            route_sha256=expected['route_sha256'], oracle_sha256=expected['oracle_sha256'], sources_sha256=expected['sources_sha256'],
            output_directory=expected['output_directory'], max_runtime_sec=expected['max_runtime_sec'],
            recorder_ready_path=str(self.path/'run/recorder_ready.json'), recorder_ready_sha256=owner.sha(self.path/'run/recorder_ready.json'),
            scene_instance_id={'wrong_fixture':True} if wrong_scene else config['scene_instance_id'])
        owner.write(Path(expected['root_release_path']), release)
        environment = dict(os.environ, MAPPING_OWNER_RUN_ID=expected['run_id'], MAPPING_OWNER_OUTPUT=expected['output_directory'])
        process = subprocess.Popen(expected['expected_argv'], env=environment, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.children.append(process)
        identity = owner.deadline.process_identity(process.pid)
        if wrong_identity:
            identity['starttime_ticks'] += 1
        binding = dict(scope='MAPPING_ACQUISITION_ONLY', **{key:expected[key] for key in ('run_id','case_id','recording_run_id','output_directory')},
            root_release_path=expected['root_release_path'], root_release_sha256=owner.sha(expected['root_release_path']),
            recorder_ready_sha256=owner.sha(self.path/'run/recorder_ready.json'), identity=identity)
        owner.write(self.path/'run/controller_binding.json', binding)
        return process

    def finish(self, process):
        process.wait(timeout=8.)
        self.assertEqual(process.returncode, 0, (self.path/'runner.log').read_text())
        result = owner.read_json(self.path/'run/recording_result.json')
        self.assertTrue(result['recorder_exited'], result)
        self.assertEqual(result['controller_signals_sent'], 0)
        self.assertTrue(result['bag_audit']['complete'], result)
        self.assertTrue(owner.deadline.process_identity(os.getpid()))
        self.assertEqual(len((self.path/'popen_events.jsonl').read_text().splitlines()), 2)
        return result

    def test_no_client_closes_within_finite_preparation_wait(self):
        config = self.config()
        process, ready = self.start(config)
        result = self.finish(process)
        self.assertEqual(result['classification'], 'CLOSED_no_controller_within_preparation_wait')

    def test_exited_client_missing_result_closes_immediately(self):
        config = self.config('missing')
        process, ready = self.start(config)
        client = self.bind(config, ready)
        result = self.finish(process)
        self.assertEqual(client.wait(timeout=1.), 0)
        self.assertEqual(result['classification'], 'CLOSED_controller_exited_result_missing')
        self.assertNotIn('extra_tail_seconds', result)

    def test_bound_success_gets_extra_raw_tail(self):
        config = self.config('success')
        process, ready = self.start(config)
        self.bind(config, ready)
        result = self.finish(process)
        self.assertEqual(result['classification'], 'CLOSED_mapping_success_with_extra_stationary_raw_tail')
        self.assertGreaterEqual(result['extra_tail_completed_boottime_sec'] - result['extra_tail_started_boottime_sec'], .2)

    def test_failure_has_no_extra_tail(self):
        config = self.config('failure')
        process, ready = self.start(config)
        self.bind(config, ready)
        result = self.finish(process)
        self.assertEqual(result['classification'], 'CLOSED_controller_failed_or_unbound_result')
        self.assertNotIn('extra_tail_seconds', result)

    def test_wrong_identity_refuses_binding_without_signalling_client(self):
        config = self.config('hang')
        process, ready = self.start(config)
        client = self.bind(config, ready, wrong_identity=True)
        result = self.finish(process)
        self.assertEqual(result['classification'], 'HOLD_recording_owner_failure')
        self.assertIsNone(client.poll())

    def test_wrong_scene_refuses_binding_without_signalling_client(self):
        config = self.config('hang')
        process, ready = self.start(config)
        client = self.bind(config, ready, wrong_scene=True)
        result = self.finish(process)
        self.assertEqual(result['classification'], 'HOLD_recording_owner_failure')
        self.assertIn('release differs', result['error'])
        self.assertIsNone(client.poll())

    def test_exec_empty_argv_before_ack_does_not_break_startup(self):
        config = self.config()
        config['fixture_reject_identity_before_exec_ack'] = True
        owner.write(self.path/'config.json', config)
        release = owner.read_json(self.path/'release.json')
        release['config_sha256'] = owner.sha(self.path/'config.json')
        owner.write(self.path/'release.json', release)
        process, ready = self.start(config)
        result = self.finish(process)
        self.assertEqual(result['classification'], 'CLOSED_no_controller_within_preparation_wait')

    def test_deadline_interrupts_parent_but_does_not_signal_client(self):
        config = self.config('hang', seconds=2.5, wait=.5)
        process, ready = self.start(config)
        client = self.bind(config, ready)
        result = self.finish(process)
        self.assertEqual(result['classification'], 'CLOSED_parent_interrupted')
        self.assertIn(signal.SIGINT, result['parent_signals'])
        self.assertIsNone(client.poll())

    def test_parent_exit_independent_guard_closes_only_recorder(self):
        config = self.config()
        process, ready = self.start(config, exit_parent=True)
        recorder_fd = os.pidfd_open(ready['recorder_process_identity']['pid'])
        try:
            self.assertEqual(process.wait(timeout=3.), 17)
            self.assertTrue(select.select([recorder_fd], [], [], 4.)[0])
            wait_for(lambda: (self.path/'run/bag/metadata.yaml').exists())
            audit = owner.bag_summary.audit(self.path/'run/bag', ['/fixture'])
            self.assertTrue(audit['complete'])
        finally:
            os.close(recorder_fd)

    def test_once_marker_prevents_any_second_popen(self):
        config = self.config()
        marker = self.path/(config['recording_run_id'] + '.recording_once.json')
        owner.claim_once(marker, {'fixture_prior_claim':True})
        process = subprocess.Popen([sys.executable, str(HERE/'_test_runner.py'), str(self.path)],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.children.append(process)
        self.assertEqual(process.wait(timeout=3.), 0)
        self.assertFalse((self.path/'popen_events.jsonl').exists())
        result = owner.read_json(self.path/'run/recording_result.json')
        self.assertEqual(result['error_type'], 'FileExistsError')

    def valid_pure_result(self):
        controller = self.config()['controller']
        identity = owner.deadline.process_identity(os.getpid())
        steady, unix_ns = time.monotonic_ns(), time.time_ns()
        ready_sha, release_sha = 'a'*64, 'b'*64
        runtime_binding = {key:controller[key] for key in ('route_sha256','oracle_sha256','sources_sha256',
            'output_directory','case_id','recording_run_id')}
        runtime_binding.update(root_release_sha256=release_sha, recorder_ready_sha256=ready_sha)
        value = dict(scope='MAPPING_ACQUISITION_ONLY', mode='controlled_static_sim_scene', run_id=controller['run_id'],
            owner_process_identity=identity, runtime_binding=runtime_binding, formal_navigation_acceptance=False,
            classification=owner.mapping_result_adapter.SUCCESS, stop_verified=True, relay_closed=True,
            relay_close_errors=[], relay_returncode=0,
            writer=dict(evidence_complete=True, dropped=0, worker_error=None, writer_alive=False),
            final_stop_observation=dict(command=[0.,0.], continuous_seconds=.3, checked_steady_ns=steady,
                checked_unix_ns=unix_ns, truth=dict(xyz=[0.,0.,.33], q=[0.,0.,0.,1.], twist=[0.]*6,
                stamp_ns=unix_ns-10_000_000, received_steady=steady/1e9-.01, frame_id='map', child_frame_id='BASE_LINK')))
        arguments = (controller, identity, release_sha, ready_sha, steady-1_000_000_000)
        self.assertTrue(owner.mapping_result_adapter.assess(value, *arguments)[0])
        return value, arguments

    def test_stop_adapter_rejects_stale_original_header(self):
        value, arguments = self.valid_pure_result()
        stop = value['final_stop_observation']
        stop['truth']['stamp_ns'] = stop['checked_unix_ns']-310_000_000
        self.assertEqual(owner.mapping_result_adapter.assess(value, *arguments),
                         (False, 'final_original_truth_stop_stale_or_future'))

    def test_stop_adapter_rejects_stale_original_receipt(self):
        value, arguments = self.valid_pure_result()
        stop = value['final_stop_observation']
        stop['truth']['received_steady'] = stop['checked_steady_ns']/1e9-.31
        self.assertEqual(owner.mapping_result_adapter.assess(value, *arguments),
                         (False, 'final_original_truth_stop_stale_or_future'))


if __name__ == '__main__':
    started = time.monotonic()
    outcome = unittest.main(exit=False)
    owner.write(ROOT/'test_result.json', dict(run=outcome.result.testsRun,
        passed=outcome.result.testsRun-len(outcome.result.failures)-len(outcome.result.errors),
        failures=len(outcome.result.failures), errors=len(outcome.result.errors), elapsed_sec=time.monotonic()-started,
        production_package_sha256={name:owner.sha(HERE/name) for name in owner.PACKAGE},
        test_only='harmless local Python stand-ins; no ROS, socket, real recorder, deployment or real controller signal'))
    raise SystemExit(not outcome.result.wasSuccessful())

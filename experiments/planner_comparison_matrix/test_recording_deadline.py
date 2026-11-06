#!/usr/bin/env python3
"""Harmless real-process deadline checks. No ROS, services, goals or runtime overlays."""

import argparse
import ast
import hashlib
import json
import os
import pathlib
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest import mock

import recording_deadline as deadline


EVIDENCE_ROOT = None
HELPER = pathlib.Path(deadline.__file__).resolve()
DUMMY = '''import json,os,pathlib,signal,sys,time
sys.dont_write_bytecode=True
sys.path.insert(0,sys.argv[1])
import recording_deadline as deadline
output=pathlib.Path(sys.argv[2]);mode=sys.argv[3];budget=float(sys.argv[4])
stopped=False
def event(kind,**kw):
 with (output/'dummy_events.jsonl').open('a') as stream:
  stream.write(json.dumps(dict(kind=kind,pid=os.getpid(),boottime_sec=time.clock_gettime(time.CLOCK_BOOTTIME),**kw))+'\\n')
def stop(signum,frame):
 global stopped
 stopped=True;event('sigint',signum=signum)
signal.signal(signal.SIGINT,stop)
event('started')
if mode=='plain':
 while True:time.sleep(.02)
if mode=='delayed':time.sleep(.55)
guard=deadline.arm(output,budget,.25 if mode.endswith('_children') else 15.);event('armed')
if mode.endswith('_children'):
 child=output/'owned_child.py'
 child.write_text("import json,os,pathlib,signal,sys,time\\noutput=pathlib.Path(sys.argv[1]);name=sys.argv[2]\\ndef event(kind):\\n with (output/('child_'+name+'.jsonl')).open('a') as stream:stream.write(json.dumps(dict(kind=kind,pid=os.getpid(),boottime_sec=time.clock_gettime(time.CLOCK_BOOTTIME)))+'\\\\n')\\ndef stop(signum,frame):\\n event('sigint');sys.exit(0)\\nsignal.signal(signal.SIGINT,stop);event('started')\\nwhile True:\\n if name=='recorder':\\n  with (output/'dummy_recording.bin').open('ab') as stream:stream.write(b'x'*1024)\\n time.sleep(.02)\\n")
 import subprocess
 for name in (['recorder'] if mode.startswith('gap_') else ['waypoint_follower','controller_server','velocity_smoother','recorder'] if mode=='waypoint_children' else ['controller_server','velocity_smoother','recorder']):
  command,environment=guard.owned_command(name,[sys.executable,'-I',str(child),str(output),name],os.environ)
  process=subprocess.Popen(command,env=environment,stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
  guard.runtime_children_started=True
  if mode.startswith('gap_'):
   event('parent_before_first_registration',child_pid=process.pid)
   if mode=='gap_exception_children':raise RuntimeError('intentional exception immediately after Popen')
   while True:time.sleep(.02)
  guard.wait_for_child(name,process.pid)
 event('children_registered')
 if mode=='exception_children':
  guard.begin_shutdown('dummy_cleanup_started');raise RuntimeError('intentional harmless cleanup exception')
 while True:time.sleep(.02)
if mode=='normal_exit':sys.exit(0)
if mode=='normal_finalize':
 guard.begin_shutdown('dummy_normal_shutdown');event('finalizer_started')
 guard.recording_closed('dummy_recorder_confirmed_closed')
 time.sleep(1.8);event('finalizer_completed');sys.exit(0)
if mode=='shutdown_stalled':guard.begin_shutdown('dummy_shutdown_started_but_not_closed')
while not stopped:pass
event('finalizer_started')
# Deliberately omit the shutdown marker while simulating slow post-closure hashing:
# the helper must have exited after its first signal even without a marker.
time.sleep(1.2);event('finalizer_completed')
guard.begin_shutdown('dummy_deadline_shutdown')
guard.recording_closed('dummy_recorder_confirmed_closed')
'''


class RecordingDeadlineTests(unittest.TestCase):
    def setUp(self):
        if EVIDENCE_ROOT:
            self.directory = EVIDENCE_ROOT / self._testMethodName
            self.directory.mkdir()
        else:
            temporary = tempfile.TemporaryDirectory(prefix='recording-deadline-test-')
            self.addCleanup(temporary.cleanup)
            self.directory = pathlib.Path(temporary.name)
        self.processes = []
        self.addCleanup(self.cleanup_processes)
        self.dummy = self.directory / 'dummy.py'
        self.dummy.write_text(DUMMY)

    def cleanup_processes(self):
        for process in self.processes:
            if process.poll() is None:
                process.terminate()
            process.wait(timeout=3)

    def output(self, name='run'):
        output = self.directory / name
        output.mkdir()
        shutil.copy2(HELPER, output / HELPER.name)
        return output

    def spawn(self, output, mode, budget=0.7):
        log = (output / 'dummy.log').open('w')
        self.addCleanup(log.close)
        environment = os.environ.copy()
        # An unrelated cwd/PYTHONPATH must not affect the detached helper imports.
        environment['PYTHONPATH'] = str(self.directory / 'nonexistent-pythonpath')
        process = subprocess.Popen([sys.executable, '-I', str(self.dummy), str(HELPER.parent),
                                    str(output), mode, str(budget)], cwd='/', env=environment,
                                   stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        self.processes.append(process)
        return process

    def wait_for(self, predicate, timeout=4):
        until = time.monotonic() + timeout
        while time.monotonic() < until:
            value = predicate()
            if value:
                return value
            time.sleep(.02)
        self.fail('Timed out waiting for harmless test process evidence')

    @staticmethod
    def events(output, name=deadline.EVENTS):
        path = output / name
        if not path.exists():
            return []
        return [json.loads(line) for line in path.read_text().splitlines()]

    def observed(self, output, kind, name=deadline.EVENTS):
        return [event for event in self.events(output, name) if event['kind'] == kind]

    def manually_watch(self, output, target, change=None):
        identity = deadline.process_identity(target.pid)
        if change:
            identity.update(change)
        manifest = dict(run_id='manual-harmless-process', output_directory=str(output),
                        launcher=identity, max_runtime_sec=.3,
                        deadline_boottime_sec=time.clock_gettime(time.CLOCK_BOOTTIME) + .3)
        path = output / deadline.MANIFEST
        deadline.atomic_json(path, manifest)
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        return path, digest, manifest['run_id']

    def test_deadline_interrupts_busy_loop_once_and_does_not_repeat_during_finalization(self):
        output = self.output()
        process = self.spawn(output, 'busy')
        self.assertEqual(process.wait(timeout=4), 0)
        self.assertEqual(len(self.observed(output, 'sigint_sent')), 1)
        self.assertEqual(len(self.observed(output, 'sigint', 'dummy_events.jsonl')), 1)
        manifest = json.loads((output / deadline.MANIFEST).read_text())
        sent = self.observed(output, 'sigint_sent')[0]['boottime_sec']
        self.assertGreaterEqual(sent, manifest['deadline_boottime_sec'])
        self.assertLess(sent - manifest['deadline_boottime_sec'], .5)

    def test_startup_time_counts_toward_the_fixed_deadline(self):
        output = self.output()
        process = self.spawn(output, 'delayed', 1.0)
        self.assertEqual(process.wait(timeout=4), 0)
        manifest = json.loads((output / deadline.MANIFEST).read_text())
        armed = self.observed(output, 'armed')[0]['boottime_sec']
        sent = self.observed(output, 'sigint_sent')[0]['boottime_sec']
        self.assertGreater(armed - manifest['launcher_started_boottime_sec'], .5)
        self.assertLess(sent - manifest['launcher_started_boottime_sec'], 1.5)

    def test_normal_shutdown_disarms_before_slow_finalization(self):
        output = self.output()
        process = self.spawn(output, 'normal_finalize', 1.0)
        self.wait_for(lambda: self.observed(output, 'shutdown_observed'))
        self.assertIsNone(process.poll())
        self.assertEqual(process.wait(timeout=3), 0)
        self.assertFalse(self.observed(output, 'sigint_sent'))
        self.assertFalse(self.observed(output, 'sigint', 'dummy_events.jsonl'))

    def test_normal_launcher_exit_ends_watchdog_without_a_signal(self):
        output = self.output()
        process = self.spawn(output, 'normal_exit', 3.0)
        self.assertEqual(process.wait(timeout=3), 0)
        self.wait_for(lambda: self.observed(output, 'launcher_exited'))
        self.assertFalse(self.observed(output, 'sigint_sent'))

    def test_shutdown_started_does_not_cancel_the_cutoff(self):
        output = self.output()
        process = self.spawn(output, 'shutdown_stalled')
        self.assertEqual(process.wait(timeout=4), 0)
        self.assertEqual(len(self.observed(output, 'sigint_sent')), 1)
        self.assertEqual(self.observed(output, 'shutdown_observed')[0]['stage'], 'shutdown_started')

    def verify_fallback_order(self, output):
        self.wait_for(lambda: self.observed(output, 'fallback_completed'), timeout=6)
        signals = self.observed(output, 'fallback_child_sigint_sent')
        self.assertEqual([row['name'] for row in signals], ['controller_server', 'velocity_smoother', 'recorder'])
        self.assertGreaterEqual(signals[1]['boottime_sec'] - signals[0]['boottime_sec'], .95)
        self.assertGreaterEqual(signals[2]['boottime_sec'] - signals[1]['boottime_sec'], .45)
        self.assertTrue(self.observed(output, 'fallback_completed')[0]['all_registered_owned_processes_exited'])
        for name in ['controller_server', 'velocity_smoother', 'recorder']:
            self.assertEqual(len(self.observed(output, 'sigint', 'child_' + name + '.jsonl')), 1)

    def test_blocked_launcher_falls_back_to_exact_registered_children(self):
        output = self.output()
        process = self.spawn(output, 'blocked_children', 1.2)
        unrelated_output = self.output('unrelated_source')
        unrelated = self.spawn(unrelated_output, 'plain')
        self.wait_for(lambda: self.observed(output, 'children_registered', 'dummy_events.jsonl'))
        self.verify_fallback_order(output)
        self.assertIsNone(process.poll())
        self.assertIsNone(unrelated.poll())
        self.assertEqual(len(self.observed(output, 'sigint_sent')), 1)
        self.assertFalse(self.observed(unrelated_output, 'sigint', 'dummy_events.jsonl'))

    def test_cleanup_exception_does_not_orphan_registered_children(self):
        output = self.output()
        process = self.spawn(output, 'exception_children', 3.0)
        self.assertNotEqual(process.wait(timeout=3), 0)
        self.verify_fallback_order(output)
        self.assertFalse(self.observed(output, 'sigint_sent'))
        self.assertEqual(self.observed(output, 'fallback_started')[0]['reason'], 'launcher_exited_before_recording_closed')

    def test_waypoint_follower_is_bound_and_stopped_in_command_producer_phase(self):
        output = self.output()
        process = self.spawn(output, 'waypoint_children', 1.5)
        self.wait_for(lambda: self.observed(output, 'children_registered', 'dummy_events.jsonl'))
        self.wait_for(lambda: self.observed(output, 'fallback_completed'), timeout=6)
        signals = self.observed(output, 'fallback_child_sigint_sent')
        self.assertEqual([row['name'] for row in signals],
                         ['waypoint_follower', 'controller_server', 'velocity_smoother', 'recorder'])
        self.assertEqual([row['phase'] for row in signals],
                         ['command_producers', 'command_producers', 'remaining_navigation', 'recorder'])
        self.assertLess(signals[1]['boottime_sec']-signals[0]['boottime_sec'], .3)
        self.assertGreaterEqual(signals[2]['boottime_sec']-signals[1]['boottime_sec'], .95)
        self.assertGreaterEqual(signals[3]['boottime_sec']-signals[2]['boottime_sec'], .45)
        self.assertTrue(self.observed(output, 'fallback_completed')[0]['all_registered_owned_processes_exited'])
        self.assertIsNone(process.poll())

    def test_gate_registers_and_stops_recording_when_parent_blocks_immediately_after_popen(self):
        output = self.output()
        process = self.spawn(output, 'gap_blocked_children', 1.2)
        self.wait_for(lambda: self.observed(output, 'parent_before_first_registration', 'dummy_events.jsonl'))
        self.wait_for(lambda: self.observed(output, 'started', 'child_recorder.jsonl'))
        self.assertTrue(self.observed(output, 'child_pidfd_retained'))
        self.wait_for(lambda: self.observed(output, 'fallback_completed'), timeout=6)
        self.assertIsNone(process.poll())
        self.assertEqual(len(self.observed(output, 'fallback_child_sigint_sent')), 1)
        size = (output / 'dummy_recording.bin').stat().st_size
        self.assertGreater(size, 0)
        time.sleep(.3)
        self.assertEqual((output / 'dummy_recording.bin').stat().st_size, size)
        self.assertTrue(self.observed(output, 'fallback_completed')[0]['all_registered_owned_processes_exited'])

    def test_gate_never_orphans_recording_if_parent_exits_immediately_after_popen(self):
        output = self.output()
        process = self.spawn(output, 'gap_exception_children', 2.0)
        self.assertNotEqual(process.wait(timeout=3), 0)
        self.wait_for(lambda: self.observed(output, 'launcher_exited'))
        if self.observed(output, 'child_pidfd_retained'):
            self.wait_for(lambda: self.observed(output, 'fallback_completed'), timeout=5)
        else:
            time.sleep(.3)  # Unacknowledged wrapper sees the exited watchdog and cannot exec.
        recording = output / 'dummy_recording.bin'
        size = recording.stat().st_size if recording.exists() else 0
        time.sleep(.3)
        self.assertEqual(recording.stat().st_size if recording.exists() else 0, size)

    def test_unacknowledged_gate_reaches_deadline_without_starting_real_process(self):
        output = self.output()
        fake_guard = self.spawn(output, 'plain')
        self.wait_for(lambda: self.observed(output, 'started', 'dummy_events.jsonl'))
        identity = deadline.process_identity(fake_guard.pid)
        manifest = dict(run_id='no-ack-test', output_directory=str(output), launcher=identity,
                        max_runtime_sec=.4, deadline_boottime_sec=time.clock_gettime(time.CLOCK_BOOTTIME) + .4)
        deadline.atomic_json(output / deadline.MANIFEST, manifest)
        digest = hashlib.sha256((output / deadline.MANIFEST).read_bytes()).hexdigest()
        deadline.atomic_json(output / deadline.ARMED, dict(run_id=manifest['run_id'], manifest_sha256=digest,
                                                         watchdog=identity))
        guard = deadline.DeadlineGuard(output, manifest, digest, fake_guard)
        forbidden = output / 'must_not_exist'
        command, environment = guard.owned_command('recorder', [sys.executable, '-c',
                                                    'import pathlib;pathlib.Path(' + repr(str(forbidden)) + ').touch()'], os.environ)
        wrapper = subprocess.Popen(command, env=environment, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                   start_new_session=True)
        self.processes.append(wrapper)
        self.assertEqual(wrapper.wait(timeout=2), 2)
        self.assertFalse(forbidden.exists())

    def test_direct_shebang_command_contract_matches_kernel_cmdline(self):
        script = self.directory / 'direct_shebang'
        script.write_text('#!' + sys.executable + '\nimport time\ntime.sleep(3)\n')
        script.chmod(0o755)
        command = [str(script), 'harmless-argument']
        process = subprocess.Popen(command)
        self.processes.append(process)
        identity = deadline.process_identity(process.pid)
        self.assertEqual(deadline.identity_after_exec(identity, deadline.command_argv(command)), identity)

    def test_parent_session_can_exit_before_detached_deadline(self):
        output = self.output()
        parent = self.directory / 'disconnected_parent.py'
        parent.write_text('import subprocess,sys\nsubprocess.Popen(sys.argv[1:],'
                          'stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,'
                          'stderr=subprocess.DEVNULL,start_new_session=True)\n')
        process = subprocess.Popen([sys.executable, '-I', str(parent), sys.executable, '-I',
                                    str(self.dummy), str(HELPER.parent), str(output), 'busy', '.8'],
                                   start_new_session=True)
        self.processes.append(process)
        self.assertEqual(process.wait(timeout=2), 0)
        self.wait_for(lambda: (output / deadline.ARMED).exists())
        armed = json.loads((output / deadline.ARMED).read_text())
        self.assertEqual(armed['watchdog_session_id'], armed['watchdog']['pid'])
        self.assertEqual(armed['watchdog_process_group_id'], armed['watchdog']['pid'])
        self.assertNotEqual(armed['watchdog_session_id'], process.pid)
        self.wait_for(lambda: self.observed(output, 'finalizer_completed', 'dummy_events.jsonl'))
        manifest = json.loads((output / deadline.MANIFEST).read_text())
        self.assertNotEqual(armed['watchdog']['pid'], manifest['launcher']['pid'])
        self.assertEqual(len(self.observed(output, 'sigint_sent')), 1)

    def test_starttime_or_commandline_mismatch_never_signals_target(self):
        for name, change in [('starttime', dict(starttime_ticks=0)),
                             ('commandline', dict(cmdline_sha256='0' * 64))]:
            with self.subTest(identity=name):
                output = self.output(name)
                target = self.spawn(output, 'plain')
                self.wait_for(lambda: self.observed(output, 'started', 'dummy_events.jsonl'))
                path, digest, run_id = self.manually_watch(output, target, change)
                self.assertEqual(deadline.watch(path, digest, run_id), 0)
                self.assertTrue(self.observed(output, 'identity_mismatch'))
                self.assertIsNone(target.poll())
                self.assertFalse(self.observed(output, 'sigint', 'dummy_events.jsonl'))

    def test_changed_run_manifest_never_signals_original_or_other_process(self):
        output = self.output()
        target = self.spawn(output, 'busy', 2.0)
        other_output = self.output('unrelated')
        unrelated = self.spawn(other_output, 'plain')
        self.wait_for(lambda: (output / deadline.ARMED).exists())
        manifest = json.loads((output / deadline.MANIFEST).read_text())
        manifest['run_id'] = 'changed-run'
        manifest['launcher'] = deadline.process_identity(unrelated.pid)
        deadline.atomic_json(output / deadline.MANIFEST, manifest)
        self.wait_for(lambda: self.observed(output, 'identity_mismatch'))
        self.assertIsNone(target.poll())
        self.assertIsNone(unrelated.poll())
        self.assertFalse(self.observed(output, 'sigint_sent'))
        self.assertFalse(self.observed(other_output, 'sigint', 'dummy_events.jsonl'))

    def test_pidfd_cannot_signal_another_process_after_target_exits_at_send(self):
        output = self.output()
        target = self.spawn(output, 'plain')
        other_output = self.output('unrelated')
        unrelated = self.spawn(other_output, 'plain')
        self.wait_for(lambda: self.observed(output, 'started', 'dummy_events.jsonl'))
        path, digest, run_id = self.manually_watch(output, target)
        send_signal = signal.pidfd_send_signal

        def exit_between_check_and_signal(descriptor, signum):
            if signum == signal.SIGINT:
                target.terminate()
                target.wait(timeout=2)
            send_signal(descriptor, signum)

        with mock.patch.object(deadline.signal, 'pidfd_send_signal', side_effect=exit_between_check_and_signal) as sender:
            self.assertEqual(deadline.watch(path, digest, run_id), 0)
            self.assertEqual([call.args[1] for call in sender.call_args_list], [0, signal.SIGINT])
        self.assertTrue(self.observed(output, 'launcher_exited'))
        self.assertIsNone(unrelated.poll())
        self.assertFalse(self.observed(other_output, 'sigint', 'dummy_events.jsonl'))

    def test_runtime_validation_rejects_unbounded_or_over_budget_values(self):
        for value in ['0', '-1', 'nan', 'inf', '3601']:
            with self.subTest(value=value), self.assertRaises(argparse.ArgumentTypeError):
                deadline.runtime_seconds(value)
        self.assertEqual(deadline.runtime_seconds('600'), 600.)
        self.assertEqual(deadline.runtime_seconds('3600'), 3600.)
        self.assertEqual(deadline.runtime_seconds('.25'), .25)

    def test_launcher_ast_preserves_close_order_and_marks_closed_before_audit(self):
        tree = ast.parse((HELPER.parent / 'matrix_navigation.py').read_text())
        run = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == 'run')
        final = next(node.finalbody for node in run.body if isinstance(node, ast.Try) and node.finalbody)
        self.assertEqual(ast.unparse(final[0]), 'shutdown_started = True')
        self.assertIn('deadline_guard.begin_shutdown(', ast.unparse(final[1]))
        events = [node.value.args[0].value for node in final if isinstance(node, ast.Expr)
                  and isinstance(node.value, ast.Call) and isinstance(node.value.func, ast.Name)
                  and node.value.func.id == 'event']
        self.assertEqual(events, ['shutdown_started', 'command_producers_stop_requested',
                                  'smoother_gate_stop_requested', 'recorder_stop_requested',
                                  'owned_processes_closed', 'stopped'])
        pumps = [node.value.args[0].value for node in final if isinstance(node, ast.Expr)
                 and isinstance(node.value, ast.Call) and isinstance(node.value.func, ast.Name)
                 and node.value.func.id == 'pump']
        self.assertEqual(pumps, [1., .5])
        statements = [ast.unparse(node) for node in final]
        waits = next(i for i, text in enumerate(statements) if 'p.wait(timeout=' in text)
        closed = next(i for i, text in enumerate(statements) if text.startswith('deadline_guard.recording_closed('))
        audit = next(i for i, text in enumerate(statements) if text.startswith('if metadata.exists():'))
        self.assertLess(waits, closed)
        self.assertLess(closed, audit)
        wait = next(node for node in run.body if isinstance(node, ast.FunctionDef) and node.name == 'wait')
        self.assertIn('time.monotonic() < end and (not stopped)', ast.unparse(wait))
        self.assertIn('future.cancel()', ast.unparse(wait))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--evidence-output', type=pathlib.Path)
    options, remaining = parser.parse_known_args()
    EVIDENCE_ROOT = options.evidence_output
    if EVIDENCE_ROOT:
        EVIDENCE_ROOT = EVIDENCE_ROOT.resolve()
        EVIDENCE_ROOT.mkdir(parents=True, exist_ok=False)
    unittest.main(argv=[sys.argv[0], *remaining])

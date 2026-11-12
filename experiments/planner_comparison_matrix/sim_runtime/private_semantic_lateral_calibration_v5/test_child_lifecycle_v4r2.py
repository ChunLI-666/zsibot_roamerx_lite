"""Pure exact-Popen lifecycle regressions; no process, Node, or socket created."""
import builtins
import copy
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest import mock

import lateral_once as o

ARGV=['/usr/bin/python3','-u','/pure/backend.py']
BIRTH=dict(pid=123,starttime_ticks=456,boot_id='pure-boot')
IDENTITY=dict(BIRTH,argv=ARGV,cmdline_sha256='a'*64)


class Process:
    pid=123
    args=ARGV
    def __init__(self,exits=False,poll_error=False):
        self.returncode=None;self.wait_calls=[];self.exits=exits;self.poll_error=poll_error
    def poll(self):
        if self.poll_error:raise OSError('pure poll failure')
        return self.returncode
    def wait(self,timeout):
        self.wait_calls.append(timeout)
        if not self.exits:raise subprocess.TimeoutExpired(ARGV,timeout)
        self.returncode=0;return 0


class Lifecycle(unittest.TestCase):
    def invoke(self,*,fd_error=False,birth=None,identity=None,guard_error=False,exits=False,
               signal_error=False,interrupt_before_retain=False,log_close_error=False,evidence_error=False,poll_error=False):
        process=Process(exits,poll_error);signals=[];closes=[]
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'child.log'
            class Log:
                def open(self,mode):
                    stream=path.open(mode)
                    class Context:
                        def __enter__(self):return stream
                        def __exit__(self,*args):
                            stream.close()
                            if log_close_error:raise OSError('pure log close failure')
                    return Context()
                def with_suffix(self,suffix):return path.with_suffix(suffix)
            def stop(fd,sig):
                signals.append((fd,sig))
                if signal_error:raise OSError('pure signal failure')
            def check():
                if guard_error:raise o.c.Hold('pure stale guard')
            dictionaries=[True]
            def dictionary(*a,**kw):
                if interrupt_before_retain and dictionaries and dictionaries.pop():raise KeyboardInterrupt('pure between Popen and retention')
                return builtins.dict(*a,**kw)
            original_atomic=o.deadline.atomic_json
            def atomic(p,value):
                if evidence_error:raise OSError('pure evidence I/O failure')
                return original_atomic(p,value)
            with mock.patch.object(o.subprocess,'Popen',return_value=process),\
                 mock.patch.object(o,'stat_birth',side_effect=birth or (lambda pid:copy.deepcopy(BIRTH))),\
                 mock.patch.object(o.deadline,'process_identity',side_effect=identity or (lambda pid:copy.deepcopy(IDENTITY))),\
                 mock.patch.object(o.deadline,'command_argv',return_value=ARGV),\
                 mock.patch.object(o.os,'pidfd_open',side_effect=OSError('pure fd failure') if fd_error else lambda pid:999) as acquire,\
                 mock.patch.object(o.os,'close',side_effect=lambda fd:closes.append(fd)),\
                 mock.patch.object(o.select,'select',return_value=([],[],[])),\
                 mock.patch.object(o.signal,'pidfd_send_signal',side_effect=stop),\
                 mock.patch.object(o,'dict',side_effect=dictionary,create=True),\
                 mock.patch.object(o.deadline,'atomic_json',side_effect=atomic):
                try:o.child(ARGV,Log(),{},check=check)
                except BaseException as error:
                    self.assertTrue(hasattr(error,'owned_spawn_cleanup'))
                    value=copy.deepcopy(error.owned_spawn_cleanup)
                else:raise AssertionError('fixture requires child failure')
            receipt=path.with_suffix('.spawn_cleanup.json')
            if not evidence_error:self.assertEqual(json.loads(receipt.read_text()),value)
        self.assertEqual(process.wait_calls,[3.]);self.assertEqual(value['pid'],123);self.assertEqual(value['expected_argv'],ARGV)
        self.assertTrue(value['wait_attempted']);self.assertIn('final_snapshot',value)
        return value,signals,closes,acquire.call_count

    def test_pidfd_failure_keeps_live_child_record_and_always_waits(self):
        value,signals,closes,attempts=self.invoke(fd_error=True)
        self.assertEqual(value['birth3'],BIRTH);self.assertFalse(value['exited']);self.assertTrue(value['still_live'])
        self.assertEqual(value['state'],'HOLD_OWNED_CHILD_OUTSTANDING');self.assertEqual(attempts,2)
        self.assertFalse(signals);self.assertFalse(closes)

    def test_initial_unknown_birth_never_bootstraps_a_later_pidfd(self):
        calls=[]
        def birth(pid):
            calls.append(pid)
            if len(calls)==1:raise ProcessLookupError('pure initial birth unavailable')
            return copy.deepcopy(BIRTH)
        value,signals,closes,attempts=self.invoke(birth=birth)
        self.assertIsNone(value['birth3']);self.assertEqual(value['final_snapshot']['birth3'],BIRTH)
        self.assertEqual(attempts,0);self.assertFalse(signals)

    def test_birth_before_or_after_open_mismatch_never_signals(self):
        for mismatch in [2,3]:
            calls=[]
            def birth(pid):
                calls.append(pid)
                return dict(BIRTH,starttime_ticks=456 if len(calls)<mismatch else 999)
            value,signals,closes,attempts=self.invoke(birth=birth)
            self.assertFalse(signals);self.assertFalse(value['exited'])
            self.assertEqual(attempts,0 if mismatch==2 else 1)
            self.assertEqual(closes,[] if mismatch==2 else [999])

    def test_guard_failure_exec_empty_argv_still_signals_only_retained_pidfd(self):
        def identity(pid):raise ProcessLookupError('pure empty exec argv')
        value,signals,closes,attempts=self.invoke(identity=identity,guard_error=True,exits=True)
        self.assertFalse(value['argv_ready']);self.assertEqual(signals,[(999,o.signal.SIGINT)])
        self.assertTrue(value['exited']);self.assertEqual(closes,[999])

    def test_admitted_wrong_argv_or_signal_error_still_waits(self):
        value,signals,closes,attempts=self.invoke(identity=lambda pid:dict(IDENTITY,argv=['/wrong']))
        self.assertFalse(signals);self.assertFalse(value['exited'])
        value,signals,closes,attempts=self.invoke(guard_error=True,signal_error=True,exits=True)
        self.assertEqual(signals,[(999,o.signal.SIGINT)]);self.assertTrue(value['exited']);self.assertFalse(value['sigint_sent'])

    def test_log_close_failure_and_pre_retain_interruption_are_accounted(self):
        value,signals,closes,attempts=self.invoke(log_close_error=True,exits=True)
        self.assertEqual(value['birth3'],BIRTH);self.assertTrue(value['exited']);self.assertEqual(signals,[(999,o.signal.SIGINT)])
        value,signals,closes,attempts=self.invoke(interrupt_before_retain=True)
        self.assertIsNone(value['birth3']);self.assertEqual(attempts,0);self.assertFalse(signals)

    def test_evidence_write_failure_keeps_record_on_original_exception(self):
        value,signals,closes,attempts=self.invoke(fd_error=True,evidence_error=True)
        self.assertIn('evidence_write_error',value);self.assertFalse(value['exited'])

    def test_final_poll_unknown_is_not_closed(self):
        value,signals,closes,attempts=self.invoke(poll_error=True)
        self.assertFalse(value['exited']);self.assertIsNone(value['still_live']);self.assertEqual(value['state'],'HOLD_OWNED_CHILD_OUTSTANDING')
        self.assertFalse(signals);self.assertEqual(attempts,0)

    def test_transient_fd_failure_recovers_only_same_birth_once(self):
        process=Process(exits=True);signals=[]
        with tempfile.TemporaryDirectory() as temp,mock.patch.object(o.subprocess,'Popen',return_value=process),\
             mock.patch.object(o,'stat_birth',return_value=copy.deepcopy(BIRTH)),\
             mock.patch.object(o.deadline,'command_argv',return_value=ARGV),\
             mock.patch.object(o.deadline,'process_identity',return_value=copy.deepcopy(IDENTITY)),\
             mock.patch.object(o.os,'pidfd_open',side_effect=[OSError('first open failed'),999]) as opened,\
             mock.patch.object(o.os,'close') as closed,\
             mock.patch.object(o.select,'select',return_value=([],[],[])),\
             mock.patch.object(o.signal,'pidfd_send_signal',side_effect=lambda fd,sig:signals.append((fd,sig))):
            with self.assertRaises(OSError) as caught:o.child(ARGV,Path(temp)/'child.log',{})
        value=caught.exception.owned_spawn_cleanup
        self.assertTrue(value['exited']);self.assertEqual(opened.call_count,2);self.assertEqual(signals,[(999,o.signal.SIGINT)])
        self.assertEqual([v['purpose'] for v in value['pidfd_attempts']],['initial','cleanup_recovery_once']);closed.assert_called_once_with(999)

    def test_actual_parent_preserves_outstanding_spawn_and_does_not_claim_helper_closed(self):
        process=Process();events=[]
        class Guard:
            def owned_command(self,*args):return ARGV,{}
            def begin_shutdown(self,state):events.append('shutdown')
            def recording_closed(self,state):raise AssertionError('must not claim outstanding recorder closed')
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);cp=root/'case.json';rp=root/'release.json';output=root/'output'
            cp.write_text(json.dumps(o.c.read(o.c.HERE/'case.HOLD.json')))
            rp.write_text(json.dumps(dict(release_id='f'*32,package_sha256=o.c.package_hashes())))
            with mock.patch.object(o.c,'validate'),mock.patch.object(o.c,'patch_check',return_value={}),\
                 mock.patch.object(o.deadline,'arm',return_value=Guard()),\
                 mock.patch.object(o.deadline,'process_identity',return_value=copy.deepcopy(IDENTITY)),\
                 mock.patch.object(o.deadline,'command_argv',return_value=ARGV),\
                 mock.patch.object(o,'stat_birth',return_value=copy.deepcopy(BIRTH)),\
                 mock.patch.object(o.subprocess,'Popen',return_value=process) as popen,\
                 mock.patch.object(o.os,'pidfd_open',side_effect=OSError('pure pidfd failure')),\
                 mock.patch.object(o.signal,'signal'),mock.patch.object(o.signal,'pidfd_send_signal') as signal,\
                 mock.patch.object(o,'Observer',side_effect=AssertionError('no Node in fixture')):
                result=o.run(cp,rp,output)
            self.assertEqual(json.loads((output/'result.json').read_text()),result)
        self.assertEqual(popen.call_count,1);signal.assert_not_called();self.assertEqual(events,['shutdown'])
        self.assertEqual(result['state'],'HOLD_DIAGNOSTIC_FAILED');self.assertFalse(result['diagnostic_passed'])
        self.assertFalse(result['all_spawn_failed_children_closed']);self.assertEqual(result['spawn_failed_child']['pid'],123)
        self.assertEqual(process.wait_calls,[3.]);self.assertEqual(result['spawn_failed_child']['birth3'],BIRTH)


if __name__=='__main__':unittest.main(verbosity=2)

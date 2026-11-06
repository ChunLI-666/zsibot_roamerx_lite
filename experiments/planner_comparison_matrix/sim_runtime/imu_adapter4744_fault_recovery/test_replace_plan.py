#!/usr/bin/python3
"""Pure mocks only: no real signal, Popen, Node, ROS or source action."""
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

import replace_adapter4744 as d

class Tests(unittest.TestCase):
    def test_default_HOLD_never_opens_pidfd_or_starts(self):
        with mock.patch.object(sys,'argv',['replace_adapter4744.py']),mock.patch('sys.stdout',new=io.StringIO()),\
            mock.patch.object(d.os,'pidfd_open',side_effect=AssertionError('pidfd forbidden')),\
            mock.patch.object(d.subprocess,'Popen',side_effect=AssertionError('Popen forbidden')):
            self.assertEqual(d.main(),0)

    def test_hold_templates_cannot_execute(self):
        plan=d.read(d.HERE/'replacement_plan.HOLD.json');release=d.read(d.HERE/'root_replacement_release.HOLD.json')
        with self.assertRaises(d.Hold):d.validate(plan,release,d.HERE/'replacement_plan.HOLD.json',Path('/never_created'))
        self.assertEqual(plan['old_adapter']['pid'],4744);self.assertEqual(plan['old_host_identity']['pid'],1270684)

    def test_mock_lifecycle_once_order_and_failures(self):
        for mode in ['normal','timeout','wrong_old','retained_changed','Popen_failed','new_exited','expired_host']:
            with self.subTest(mode=mode),tempfile.TemporaryDirectory() as directory:
                root=Path(directory);plan=d.read(d.HERE/'replacement_plan.HOLD.json');pp=root/'plan';pp.write_text(json.dumps(plan))
                release=dict(release_id='e'*32,adapter_release_path=str(root/'inner'));rp=root/'release';rp.write_text(json.dumps(release));out=root/'attempt'
                argv=['/usr/bin/python3','-u',str(Path(plan['source_directory'])/'sim_imu_adapter.py'),'--execute','--runtime-config',plan['runtime_config'],'--root-release',release['adapter_release_path']]
                process=SimpleNamespace(pid=777,poll=lambda:1 if mode=='new_exited' else None)
                events=[];calls={'select':0,'retained':0}
                def fd_open(pid):events.append(('pidfd',pid));return 160 if pid==4744 else 877
                def exact(expected):
                    if mode=='wrong_old':raise d.Hold('fixture old identity mismatch')
                    return expected
                def select(fds,*args):
                    calls['select']+=1
                    return (([] if calls['select']==1 or mode=='timeout' else fds),[],[])
                def signal(fd,number):
                    self.assertEqual(fd,160);self.assertTrue((root/('e'*32+'.replace_adapter4744_once.json')).exists())
                    events.append(('SIGINT',fd))
                def retained(plan):
                    calls['retained']+=1
                    if mode=='retained_changed' and calls['retained']>=2:raise d.Hold('fixture retained changed')
                def popen(*args,**kwargs):
                    self.assertEqual(args[0],argv);self.assertTrue((out/'new_start_once.json').exists())
                    self.assertEqual(events[-1],('SIGINT',160));events.append(('Popen',777))
                    if mode=='Popen_failed':raise OSError('fixture exec failed')
                    return process
                times=iter([0.,20.])
                def host(*args):
                    if mode=='expired_host':raise d.Hold('fixture host expired')
                with mock.patch.object(d,'validate'),mock.patch.object(d,'exact',side_effect=exact),mock.patch.object(d,'retained',side_effect=retained),\
                    mock.patch.object(d,'host_proof',side_effect=host),mock.patch.object(d,'retirement',return_value='Z'),\
                    mock.patch.object(d.os,'pidfd_open',side_effect=fd_open),mock.patch.object(d.os,'close'),\
                    mock.patch.object(d.select,'select',side_effect=select),mock.patch.object(d.signal,'pidfd_send_signal',side_effect=signal),\
                    mock.patch.object(d.subprocess,'Popen',side_effect=popen) as spawn,\
                    mock.patch.object(d,'identity',return_value=dict(pid=777,starttime_ticks=9,boot_id='fixture',cmdline_sha256='a'*64,argv=argv,uid=[1000]*4)),\
                    mock.patch.object(d.time,'monotonic',side_effect=(lambda:next(times)) if mode=='timeout' else __import__('time').monotonic):
                    result=d.run(pp,rp,out)
                    if mode=='normal':
                        self.assertEqual(result['state'],'SOURCE_STARTED_PENDING_ROOT_READINESS');self.assertFalse(result['ready_for_motion'])
                        self.assertEqual(spawn.call_count,1)
                        # Same root release cannot authorize a second signal/start.
                        second=d.run(pp,rp,root/'second');self.assertEqual(second['state'],'HOLD_OPERATION_INCOMPLETE');self.assertEqual(spawn.call_count,1)
                    else:self.assertEqual(result['state'],'HOLD_OPERATION_INCOMPLETE')
                self.assertEqual(sum(e[0]=='SIGINT' for e in events),0 if mode in ['wrong_old','expired_host'] else 1)
                self.assertEqual(sum(e[0]=='Popen' for e in events),1 if mode in ['normal','Popen_failed','new_exited'] else 0)
                self.assertEqual(result['signals_to24_61_UE_new_source'],0);self.assertFalse(result['automatic_rollback_or_retry'])

    def test_hash_has_no_file_digest_dependency(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)/'data';p.write_bytes(b'fixture')
            with mock.patch.object(d.hashlib,'file_digest',create=True,side_effect=AssertionError('Python3.10 missing')):
                self.assertEqual(d.sha(p),d.hashlib.sha256(b'fixture').hexdigest())

if __name__=='__main__':unittest.main(verbosity=2)

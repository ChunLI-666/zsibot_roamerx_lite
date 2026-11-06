#!/usr/bin/python3
"""Harmless fixtures only: fake sockets/samples and exact local standin PIDs."""
import copy
import importlib.util
import io
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest import mock

import lateral_contract as c
import lateral_once as driver
import official_tool_timed as timed
import host_preflight


def gt(stamp=1_000_000_000,steady=None):
    return dict(stamp_ns=stamp,received_steady_ns=time.monotonic_ns()-1_000_000_000 if steady is None else steady,
        frame='map',child='BASE_LINK',position=[.073,0.,.33],quaternion=[0.,0.,0.,1.],linear=[0.,0.,0.],angular=[0.,0.,0.],
        payload_valid=True,position_speed_mps=0.)

class Pure(unittest.TestCase):
    def test_default_hold_has_no_runtime_side_effect(self):
        with mock.patch.object(sys,'argv',['lateral_once.py']),mock.patch.object(subprocess,'Popen',side_effect=AssertionError('Popen forbidden')):
            with mock.patch('sys.stdout',new=io.StringIO()):self.assertEqual(driver.main(),0)
        with mock.patch.object(sys,'argv',['official_tool_timed.py']),mock.patch.object(subprocess,'Popen',side_effect=AssertionError('Popen forbidden')):
            with mock.patch('sys.stdout',new=io.StringIO()):self.assertEqual(timed.main(),0)

    def test_hold_case_cannot_validate(self):
        case=c.read(c.HERE/'case.HOLD.json');release=c.read(c.HERE/'root_release.HOLD.json')
        with self.assertRaises(c.Hold):c.validate(case,release,c.HERE/'case.HOLD.json',Path('/uncreated'))

    def test_once_marker_no_repeat(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'once';c.claim(p,{'first':True})
            with self.assertRaises(FileExistsError):c.claim(p,{'repeat':True})
            self.assertEqual(c.read(p),{'first':True})

    def test_raw_patch_and_full_stop_envelope(self):
        x=c.patch_check(c.read(c.HERE/'case.HOLD.json'));self.assertEqual(x['complete_patch_cells'],7200)
        self.assertFalse(x['hard_plant_guarantee']);self.assertTrue(x['no_grid_erosion'])
        bad=c.read(c.HERE/'case.HOLD.json');bad['patch_xyxy']=[-1.,-2.5,1.,2.5]
        with self.assertRaises(c.Hold):c.patch_check(bad)

    def test_strict_stop_and_advancing_window(self):
        for linear,angular,expected in [(0.,0.,True),(.03,.05,True),(.031,0.,False),(0.,.051,False)]:
            s=c.StableStop()
            for i in range(32):
                row=gt(1_000_000_000+i*10_000_000);row['linear'][0]=linear;row['angular'][2]=angular;s.update(row)
            self.assertEqual(s.ready(),expected)
        with self.assertRaises(c.Hold):s.update(row)
        s=c.StableStop();s.update(gt());s.update(gt(1_500_000_000));self.assertFalse(s.ready())

    def test_freshness_standing_and_excursion_fail_closed(self):
        now=10_000_000_000;steady=time.monotonic_ns()
        samples={t:dict(stamp_ns=now,received_steady_ns=steady,payload_valid=True) for t in c.TOPICS};samples['/odom']=gt(now,steady)
        c.check_live(samples,(.073,0.,0.),now,steady)
        mutations=[lambda d:d['/front_lidar'].update(stamp_ns=now-301_000_000),
            lambda d:d['/front_lidar/imu'].update(stamp_ns=now+51_000_000),
            lambda d:d['/odom']['position'].__setitem__(2,.19),lambda d:d['/odom']['position'].__setitem__(1,.801),
            lambda d:d['/odom']['position'].__setitem__(0,.174),lambda d:d['/odom']['linear'].__setitem__(1,1.001),
            lambda d:d['/odom'].update(child='base_link')]
        for mutate in mutations:
            changed=copy.deepcopy(samples);mutate(changed)
            with self.assertRaises(c.Hold):c.check_live(changed,(.073,0.,0.),now,steady)

    def test_packet_axis_and_official_finally(self):
        spec=importlib.util.spec_from_file_location('test_original_official',c.HERE/'matrix_mc_udp_test.py');official=importlib.util.module_from_spec(spec);spec.loader.exec_module(official)
        command=official.neutral_command();command['LX']=-.2;c.permitted_payload(command,.2)
        for key,value in [('LY',.2),('RX',.2),('Y',1),('LX',.2)]:
            bad=dict(command);bad[key]=value
            with self.assertRaises(c.Hold):c.permitted_payload(bad,.2)
        calls=[]
        class FakeController(official.UdpRobotController):
            def __init__(self):pass
            def hold(self,command,duration):
                calls.append(dict(command))
                if command['LX']:raise KeyboardInterrupt()
        with self.assertRaises(KeyboardInterrupt):FakeController().move(lateral=.2,duration=1.)
        self.assertEqual(calls[0]['LX'],-.2);self.assertTrue(any(p['LB']==p['B']==1 for p in calls[1:]))
        self.assertTrue(all(not p['LY'] and not p['RX'] for p in calls))

    def test_timed_send_guards_each_nonzero_and_neutral_survives(self):
        commands=[];ticks=[];stream=io.StringIO()
        class FakeSocket:
            def sendto(self,payload,target):commands.append(json.loads(payload));return len(payload)
            def close(self):pass
        s=timed.TimedSocket(FakeSocket(),.2,stream,lambda:ticks.append('first'),lambda:ticks.append('fresh'))
        neutral={k:0 for k in ['LX','LY','RX','RY','LT','RT','A','B','X','Y','LB','RB','VIEW','MENU','GUIDE','LS','RS']};moving=dict(neutral,LX=-.2)
        s.sendto(json.dumps(moving).encode(),('127.0.0.1',7447));s.sendto(json.dumps(moving).encode(),('127.0.0.1',7447))
        self.assertEqual(ticks,['first','fresh','fresh'])
        s.before_nonzero=lambda:(_ for _ in ()).throw(c.Hold('stale lease'))
        with self.assertRaises(c.Hold):s.sendto(json.dumps(moving).encode(),('127.0.0.1',7447))
        s.sendto(json.dumps(neutral).encode(),('127.0.0.1',7447));self.assertEqual(len(commands),3)
        for line in stream.getvalue().splitlines():
            row=json.loads(line);self.assertLessEqual(row['send_before_unix_ns'],row['send_after_unix_ns'])
        s.before_nonzero=lambda:None;s.first=time.monotonic()-1.01
        with self.assertRaises(c.Hold):s.sendto(json.dumps(moving).encode(),('127.0.0.1',7447))
        self.assertEqual(len(commands),3)

    def test_external_host_proof_age_hash_and_honest_scope(self):
        case=c.read(c.HERE/'case.HOLD.json');now=100_000_000_000
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'proof.json';release=dict(boot_id='fixture_boot',network_namespace='fixture_net',controller_pid_namespace='fixture_pid',
                host_preflight=dict(mode='root_external_host_attestation',path=str(path),sha256=None,max_age_sec=30.))
            proof=dict(state='PASS',completed_unix_ns=now,scene_identity=case['scene_instance_id'],boot_id='fixture_boot',uid=1000,
                network_namespace='fixture_net',container_pid_namespace='fixture_pid',container_id=case['container_id'],shared_host_network=True,
                port7447_exact_scene_owner=True,retired_workers_verified=True,forbidden_live_processes=[],artifacts_verified=[{'fixture':True}])
            path.write_text(json.dumps(proof));release['host_preflight']['sha256']=c.sha(path)
            self.assertEqual(c.host_proof(release,case,now)['completed_unix_ns'],now)
            with self.assertRaises(c.Hold):c.host_proof(release,case,now+30_000_000_001)
            path.write_text(json.dumps(dict(proof,state='HOLD')))
            with self.assertRaises(c.Hold):c.host_proof(release,case,now)

    def test_own_pidfd_closes_standin_and_guard_failure_does_not_orphan(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);ready=root/'ready';done=root/'finally'
            code="import pathlib,time\np=pathlib.Path("+repr(str(ready))+");q=pathlib.Path("+repr(str(done))+ ")\ntry:\n p.write_text('ready')\n while True:time.sleep(.01)\nexcept KeyboardInterrupt:pass\nfinally:q.write_text('neutral_fixture_only')\n"
            argv=[sys.executable,'-u','-c',code]
            def fail():
                until=time.monotonic()+1.
                while not ready.exists() and time.monotonic()<until:time.sleep(.005)
                raise c.Hold('fixture stale during spawn')
            with self.assertRaises(c.Hold):driver.child(argv,root/'log',dict(__import__('os').environ),check=fail)
            self.assertEqual(done.read_text(),'neutral_fixture_only')

    def test_no_tool_before_ack_or_mcap(self):
        for ack in [False,True]:
            with self.subTest(ack=ack),tempfile.TemporaryDirectory() as directory:
                root=Path(directory);out=root/'run';ready=root/'standin_ready';events=[]
                argv=[sys.executable,'-u','-c',"import pathlib,time\npathlib.Path("+repr(str(ready))+").write_text('ready')\ntry:\n time.sleep(10)\nexcept KeyboardInterrupt:pass\n"]
                class Guard:
                    manifest={'deadline_boottime_sec':time.clock_gettime(time.CLOCK_BOOTTIME)+60}
                    children={'recorder':{'wrapper_argv':argv,'executed_argv':argv}}
                    process=SimpleNamespace(wait=lambda timeout:None)
                    def check_live(self):pass
                    def owned_command(self,name,command,env):
                        self_case.assertIn('-a',command);self_case.assertIn('--include-hidden-topics',command);return argv,env
                    def wait_for_child(self,name,pid):
                        until=time.monotonic()+1.
                        while not ready.exists() and time.monotonic()<until:time.sleep(.005)
                        if not ack:raise c.Hold('fixture missing independent ACK')
                        driver.deadline.atomic_json(out/'recording_deadline_recorder_registration.json',{'identity':driver.deadline.process_identity(pid)})
                    def begin_shutdown(self,reason):events.append('closing')
                    def recording_closed(self,reason):events.append('closed')
                self_case=self;guard=Guard();case=c.read(c.HERE/'case.HOLD.json');cp=root/'case.json';cp.write_text(json.dumps(case))
                release={'release_id':'f'*32,'package_sha256':c.package_hashes()};rp=root/'release.json';rp.write_text(json.dumps(release))
                class FastClock:
                    offset=0.
                    def monotonic(self):return time.monotonic()+self.offset
                    def sleep(self,n):self.offset+=1.;time.sleep(.002)
                    time_ns=staticmethod(time.time_ns);monotonic_ns=staticmethod(time.monotonic_ns);clock_gettime=staticmethod(time.clock_gettime);CLOCK_BOOTTIME=time.CLOCK_BOOTTIME
                with mock.patch.object(c,'validate'),mock.patch.object(driver.deadline,'arm',return_value=guard),mock.patch.object(driver,'time',FastClock()):
                    result=driver.run(cp,rp,out)
                self.assertEqual(result['state'],'HOLD_DIAGNOSTIC_FAILED');self.assertEqual(result['probes'],[])
                self.assertTrue(result['recorder_exited']);self.assertTrue(result['recorder_sigint_sent']);self.assertEqual(events,['closing','closed'])

    def test_host_proc_fixture_catches_actual_calibration_writers(self):
        names=['matrix_mc_udp_test.py','official_tool_timed.py','lateral_once.py','sim_udp_relay_frozen.py']
        with tempfile.TemporaryDirectory() as directory:
            proc=Path(directory)
            for i,name in enumerate(names+['benign_matrix_mc_udp_test.py']):
                p=proc/str(70+i);p.mkdir();(p/'status').write_text('Uid:\t1000\t1000\t1000\t1000\n')
                (p/'stat').write_text(str(70+i)+' (python3) S '+' '.join(['0']*18+['123'])+'\n')
                (p/'cmdline').write_bytes(b'/usr/bin/python3\0-u\0'+('/safe/'+name).encode()+b'\0')
            found=host_preflight.forbidden_live(proc)
            self.assertEqual({Path(p['argv'][-1]).name for p in found},set(names))

    def test_exec_empty_birth_is_retained_and_wait_is_unconditional(self):
        base=dict(pid=77,starttime_ticks=123,boot_id='fixture');valid=dict(base,argv=['python','owned'],cmdline_sha256='a'*64)
        for mode in ['empty_then_valid','empty_twice','wrong_argv','wrong_birth']:
            with self.subTest(mode=mode),tempfile.TemporaryDirectory() as directory:
                root=Path(directory);events=[]
                class Process:
                    pid=77;returncode=None
                    def poll(self):return self.returncode
                    def wait(self,timeout):events.append('wait');self.returncode=0;return 0
                process=Process();second=copied=dict(valid)
                if mode=='wrong_argv':second=dict(valid,argv=['python','different'])
                identities=[ProcessLookupError('exec empty'),ProcessLookupError('exec empty') if mode=='empty_twice' else second]
                births=[base,dict(base,starttime_ticks=999) if mode=='wrong_birth' else base]
                def fail():events.append('watch');raise c.Hold('fixture stale')
                with mock.patch.object(driver.subprocess,'Popen',return_value=process),mock.patch.object(driver.os,'pidfd_open',return_value=999),\
                    mock.patch.object(driver.select,'select',return_value=([],[],[])),mock.patch.object(driver,'stat_birth',side_effect=births),\
                    mock.patch.object(driver.deadline,'process_identity',side_effect=identities),mock.patch.object(driver.deadline,'command_argv',return_value=['python','owned']),\
                    mock.patch.object(driver.signal,'pidfd_send_signal',side_effect=lambda fd,sig:events.append(('signal',fd,sig))),\
                    mock.patch.object(driver.os,'close',side_effect=lambda fd:events.append(('close',fd))):
                    with self.assertRaises(c.Hold):driver.child(['python','owned'],root/'tool.log',{},check=fail)
                sent=any(isinstance(e,tuple) and e[0]=='signal' for e in events)
                self.assertEqual(sent,mode in ['empty_then_valid','empty_twice']);self.assertIn('wait',events)
                self.assertEqual(events[-1],('close',999));self.assertEqual(process.returncode,0)
                report=c.read(root/'tool.spawn_cleanup.json');self.assertTrue(report['exited']);self.assertEqual(report['sigint_sent'],sent)

    def test_admitted_child_does_not_gain_stat_only_signal_permission(self):
        base=dict(pid=77,starttime_ticks=123,boot_id='fixture');process=SimpleNamespace(pid=77)
        owned=dict(process=process,fd=999,birth3=base,argv_ready=True,signalled=False)
        with mock.patch.object(driver.select,'select',return_value=([],[],[])),mock.patch.object(driver,'stat_birth',return_value=base),\
            mock.patch.object(driver.deadline,'process_identity',side_effect=ProcessLookupError('empty')),\
            mock.patch.object(driver.signal,'pidfd_send_signal') as send:
            with self.assertRaises(c.Hold):driver.interrupt(owned,[['python','owned']])
            send.assert_not_called()

    def test_stop_metrics_reject_gapped_or_nonadvancing_GT(self):
        with tempfile.TemporaryDirectory() as directory:
            trace=Path(directory)/'send.jsonl'
            sends=[]
            for stamp,lx in [(1_000_000_000,-.2),(1_500_000_000,-.2),(1_600_000_000,0.)]:
                sends.append(dict(command={'LX':lx},send_before_unix_ns=stamp,send_after_unix_ns=stamp+1,
                    send_before_steady_ns=stamp,send_after_steady_ns=stamp+1))
            trace.write_text(''.join(json.dumps(row)+'\n' for row in sends))
            moving=[]
            for i in range(6):
                row=gt(1_000_000_000+i*100_000_000);row['position'][1]=i*.03;moving.append(row)
            for kind in ['continuous','gapped','duplicate']:
                if kind=='continuous':stamps=[1_610_000_000+i*10_000_000 for i in range(35)]
                elif kind=='gapped':stamps=[1_650_000_000+i*150_000_000 for i in range(4)]
                else:stamps=[1_650_000_000,1_700_000_000,1_700_000_000,2_000_000_000]
                stopped=[]
                for stamp in stamps:
                    row=gt(stamp);row['position'][1]=.15;stopped.append(row)
                result=driver.metrics(moving+stopped,trace)
                self.assertEqual(result['state']=='MEASURED_SHORT_PROBE',kind=='continuous')
                self.assertEqual(result['gt_first_stopped_header_ns'] is not None,kind=='continuous')


if __name__=='__main__':unittest.main(verbosity=2)

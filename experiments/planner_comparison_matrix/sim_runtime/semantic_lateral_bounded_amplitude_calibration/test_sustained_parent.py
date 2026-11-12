"""Pure mocked parent, real metric/guard functions, no ROS/node/process/socket."""
import ast
import contextlib
import copy
import hashlib
import io
import json
from pathlib import Path
import struct
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

sys.dont_write_bytecode=True
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
import lateral_contract as c
import lateral_once as parent
import calibration_observer as observer_module
import host_preflight
from imu_gate import ImuMonotonicGate,decode_imu_cdr


def gt(stamp=100_000_000_000,y=0.,linear=0.,steady=0):
    return dict(stamp_ns=stamp,received_steady_ns=steady,frame='map',child='BASE_LINK',
        position=[.073,y,.33],quaternion=[0.,0.,0.,1.],linear=[0.,linear,0.],angular=[0.,0.,0.],
        payload_valid=True,position_speed_mps=0.)


def trace_and_history(index=0,response='correct',magnitude=.2,duration=4.):
    sign=[magnitude,-magnitude][index];start=(101+9*index)*1_000_000_000;direction=1 if sign>0 else -1
    if response=='wrong':direction=-direction
    if response=='none':direction=0
    sends=[]
    packets=round(duration*20)
    for i in range(packets+5):
        stamp=start+i*50_000_000;value=sign if i<packets else 0.
        command=dict(version=1,action='walk',forward=0.,lateral=value,yaw=0.)
        raw=json.dumps(command,separators=(',',':')).encode()
        sends.append(dict(event='actual_socket_sendto_interval_not_actuation',send_success=True,target=['127.0.0.1',7447],bytes=len(raw),payload_sha256=hashlib.sha256(raw).hexdigest(),command=command,send_before_unix_ns=stamp,send_after_unix_ns=stamp+1,send_before_steady_ns=stamp,send_after_steady_ns=stamp+1))
    base_y=0. if index==0 else .08*(duration-.01)
    history=[gt(start-10_000_000,y=base_y)]
    for i in range(round(duration*100)):
        elapsed=i*.01;distance=min(elapsed,2.) if response=='tail_zero' else elapsed
        history.append(gt(start+i*10_000_000,y=base_y+direction*.08*distance,linear=0. if response=='tail_zero' and elapsed>=2. else direction*.08))
    final_y=history[-1]['position'][1]
    for i in range(351):
        y=base_y if response=='rebound' else final_y
        history.append(gt(start+round(duration*1e9)+i*10_000_000,y=y,linear=.1 if response=='no_stop' else 0.))
    return sends,history


def write_trace(path,sends):path.write_text(''.join(json.dumps(row)+'\n' for row in sends))


class FakeClock:
    CLOCK_BOOTTIME=7
    def __init__(self):self.t=0.
    def monotonic(self):self.t+=.02;return self.t
    def monotonic_ns(self):return round(self.t*1e9)
    def time_ns(self):return 100_000_000_000+round(self.t*1e9)
    def clock_gettime(self,unused):return self.t
    def sleep(self,n):self.t+=n


class CombinedFixtures(unittest.TestCase):
    def test_parent_rebound_or_zero_final_velocity_never_plans_negative(self):
        for response in ['rebound','tail_zero','between_rebound']:
            with self.subTest(response=response):
                result,tools,plans,events=self.run_parent(response)
                self.assertEqual(tools,[0]);self.assertNotIn('probe_1.planned.json',plans)
                self.assertEqual(result['state'],'HOLD_DIAGNOSTIC_FAILED')

    def test_sustained_original_GT_both_signs_complete_window_and_freeze(self):
        for index in [0,1]:
            rows,history=trace_and_history(index);sign=[.2,-.2][index]
            with tempfile.TemporaryDirectory() as out:
                path=Path(out)/'trace';write_trace(path,rows);legacy=parent.metrics(history,path,sign)
                cutoff=history[-1]['stamp_ns'];r=parent.sustained_classification(history,path,sign,legacy,cutoff)
                self.assertEqual(r['state'],'MEASURED_SUSTAINED_LATERAL_PROBE');self.assertGreater(r['signed_net_stopped_body_y_m'],.05)
                self.assertGreater(r['signed_final_active1s_body_vy_median_mps'],.03);self.assertTrue(r['final_active1s_complete'])
                later=history+[gt(cutoff+1_000_000_000,y=-.7)]
                self.assertEqual(parent.sustained_classification(later,path,sign,legacy,cutoff),r)

    def test_sustained_tail_gap_missing_anchor_and_unmeasured_stop_fail(self):
        rows,history=trace_and_history()
        with tempfile.TemporaryDirectory() as out:
            path=Path(out)/'trace';write_trace(path,rows);legacy=parent.metrics(history,path,.2)
            for changed in [[row for row in history if not 104_200_000_000<row['stamp_ns']<104_450_000_000],history[2:]]:
                result=parent.sustained_classification(changed,path,.2,legacy,changed[-1]['stamp_ns'])
                self.assertEqual(result['state'],'FAILED_NOT_SUSTAINED_LATERAL')
            changed=copy.deepcopy(history)
            for row in changed:
                if row['stamp_ns']>=105_000_000_000:row['linear'][1]=.04
            self.assertFalse(parent.sustained_classification(changed,path,.2,legacy,changed[-1]['stamp_ns'])['strict_stopped_at_cutoff'])
    def test_default_parent_and_case_remain_hold_without_business(self):
        with mock.patch.object(sys,'argv',['lateral_once.py']),mock.patch.object(parent.subprocess,'Popen',side_effect=AssertionError('Popen forbidden')),mock.patch.object(parent,'Observer',side_effect=AssertionError('Node forbidden')):
            with contextlib.redirect_stdout(io.StringIO()) as stream:self.assertEqual(parent.main(),0)
        self.assertEqual(json.loads(stream.getvalue())['state'],'ROOT_HOLD')
        case=c.read(HERE/'case.HOLD.json');release=dict(scope=c.SCOPE,allow_execute=False)
        with self.assertRaises(c.Hold):c.validate(case,release,HERE/'case.HOLD.json',Path('/never-created'))
        self.assertEqual(c.SCOPE,'MAPPING_SEMANTIC_LATERAL_BOUNDED_AMPLITUDE_SUSTAINED_CALIBRATION_ONLY')
        self.assertIs(observer_module.c,c)
        self.assertNotIn('matrix_mc_udp_test.py',c.PACKAGE)
        self.assertEqual(len(c.PACKAGE),8)

    def test_fixed_semantic_metrics_correct_both_signs_and_stop(self):
        for index in [0,1]:
            with tempfile.TemporaryDirectory() as directory:
                rows,history=trace_and_history(index);path=Path(directory)/'trace.jsonl';write_trace(path,rows)
                result=parent.metrics(history,path,[.2,-.2][index])
                self.assertEqual(result['state'],'MEASURED_SHORT_PROBE')
                self.assertIsNotNone(result['gt_first_stopped_header_ns'])
                self.assertEqual(result['policy_lateral'],[.2,-.2][index])

    def test_metrics_reject_illegal_semantics_and_unsuccessful_send(self):
        mutations=[lambda r:r['command'].update(forward=.01),lambda r:r['command'].update(yaw=.1),
            lambda r:r['command'].update(version=True),lambda r:r['command'].update(action='stand'),
            lambda r:r['command'].update(lateral=-.2),lambda r:r['command'].update(LX=-.2),
            lambda r:r.update(send_success=False),lambda r:r.update(event='socket_sendto_failed_or_short_not_actuation'),
            lambda r:r.update(target=['127.0.0.1',7448]),lambda r:r.update(bytes=1),
            lambda r:r.update(payload_sha256='0'*64),lambda r:r.update(send_before_steady_ns=-1)]
        for mutation in mutations:
            rows,history=trace_and_history();mutation(rows[0])
            with self.subTest(mutation=mutation),tempfile.TemporaryDirectory() as directory:
                path=Path(directory)/'trace';write_trace(path,rows)
                with self.assertRaises(c.Hold):parent.metrics(history,path,.2)

    def test_metrics_direction_zero_and_stop_gaps_remain_failures(self):
        for response in ['none','wrong','no_stop']:
            rows,history=trace_and_history(response=response)
            with self.subTest(response=response),tempfile.TemporaryDirectory() as directory:
                path=Path(directory)/'trace';write_trace(path,rows)
                self.assertEqual(parent.metrics(history,path,.2)['state'],'FAILED_NO_MEASURABLE_LATERAL_OR_STOP')
        rows,history=trace_and_history();moving=[s for s in history if s['stamp_ns']<105_000_000_000]
        for kind,stamps in [('gapped',[105_010_000_000+i*150_000_000 for i in range(4)]),('duplicate',[105_010_000_000,105_020_000_000,105_020_000_000,105_400_000_000])]:
            with self.subTest(kind=kind),tempfile.TemporaryDirectory() as directory:
                path=Path(directory)/'trace';write_trace(path,rows)
                self.assertNotEqual(parent.metrics(moving+[gt(s,y=.19) for s in stamps],path,.2)['state'],'MEASURED_SHORT_PROBE')

    def run_parent(self,response='correct',acceptance_io_fail=False,magnitude=.2,duration=4.):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);out=root/'run';clock=FakeClock();events=[];tool_indices=[];instance=[]
            case=c.read(HERE/'case.HOLD.json');case.update(magnitude=magnitude,probes=[magnitude,-magnitude],duration_sec=duration);cp=root/'case.json';cp.write_text(json.dumps(case))
            release=dict(release_id='f'*32,package_sha256=c.package_hashes());rp=root/'release.json';rp.write_text(json.dumps(release))
            class Process:
                def __init__(self,pid,code=None):self.pid=pid;self.returncode=code
                def poll(self):return self.returncode
                def wait(self,timeout):self.returncode=0;return 0
            recorder=Process(201)
            identities={201:dict(pid=201,role='recorder'),202:dict(pid=202,role='helper')}
            class Guard:
                manifest={'deadline_boottime_sec':60.}
                children={'recorder':{'wrapper_argv':['fixture-recorder'],'executed_argv':['fixture-recorder']}}
                process=SimpleNamespace(wait=lambda timeout:0)
                def check_live(self):pass
                def owned_command(self,name,argv,environment):
                    self_case.assertIn('-a',argv);self_case.assertIn('--include-hidden-topics',argv);return ['fixture-recorder'],environment
                def wait_for_child(self,name,pid):
                    parent.deadline.atomic_json(out/'recording_deadline_recorder_registration.json',dict(identity=identities[201]))
                    parent.deadline.atomic_json(out/'recording_deadline_armed.json',dict(watchdog=identities[202]))
                def begin_shutdown(self,reason):events.append('shutdown')
                def recording_closed(self,reason):events.append('closed')
            class FakeObserver:
                def __init__(self,output):
                    self.history=[];self.errors=[];self.counts={t:1 for t in c.TOPICS}
                    self.latest={t:dict(stamp_ns=100_000_000_000,payload_valid=True) for t in c.TOPICS};self.latest['/odom']=gt()
                    self.stop=SimpleNamespace(ready=lambda:True);instance.append(self)
                def spin(self):
                    if response=='between_rebound' and (out/'probe_0.acceptance.json').exists() and not (out/'probe_1.planned.json').exists():
                        row=gt(self.latest['/odom']['stamp_ns']+10_000_000,y=0.);self.history.append(row);self.latest['/odom']=row
                def close(self):events.append('observer_closed')
            def child(argv,log,environment,check=None):
                if argv==['fixture-recorder']:
                    (out/'bag').mkdir();(out/'bag/fixture.mcap').write_bytes(parent.bag_summary.MAGIC)
                    return dict(process=recorder,fd=900,birth3={},argv_ready=True,signalled=False)
                self.assertEqual(Path(argv[2]).name,'semantic_tool_timed.py')
                index=int(argv[argv.index('--probe-index')+1]);tool_indices.append(index)
                if index==1:
                    self.assertEqual(c.read(out/'probe_0.acceptance.json')['state'],'MEASURED_SUSTAINED_LATERAL_PROBE')
                    self.assertEqual(c.read(out/'probe_1.planned.json')['prior_positive_acceptance_sha256'],c.sha(out/'probe_0.acceptance.json'))
                rows,history=trace_and_history(index,response if index==0 else 'correct',magnitude,duration)
                if response=='illegal' and index==0:rows[0]['command']['forward']=.1
                write_trace(out/f'probe_{index}_send.jsonl',rows)
                instance[0].history.extend(history);instance[0].latest['/odom']=history[-1]
                if check:check()
                return dict(process=Process(203+index,0),fd=901+index,birth3={},argv_ready=True,signalled=False)
            def interrupt(owned,allowed):owned['signalled']=True;owned['process'].returncode=0
            original_atomic=parent.deadline.atomic_json
            def atomic(path,value):
                if acceptance_io_fail and Path(path).name.endswith('.acceptance.json'):raise OSError('fixture acceptance disk failure')
                return original_atomic(path,value)
            self_case=self
            with mock.patch.object(c,'validate'),mock.patch.object(c,'patch_check',return_value=dict(fixture=True)),mock.patch.object(c,'host_proof'),\
                mock.patch.object(c,'check_live',side_effect=lambda latest,anchor,now,steady:(latest['/odom'],(0.,0.,0.))),\
                mock.patch.object(parent,'time',clock),mock.patch.object(parent,'Observer',FakeObserver),\
                mock.patch.object(parent,'child',side_effect=child),mock.patch.object(parent,'interrupt',side_effect=interrupt),\
                mock.patch.object(parent.subprocess,'Popen',side_effect=AssertionError('real Popen forbidden')),\
                mock.patch.object(parent.os,'close'),mock.patch.object(parent.signal,'signal',return_value=None),\
                mock.patch.object(parent.deadline,'arm',return_value=Guard()),mock.patch.object(parent.deadline,'process_identity',side_effect=lambda pid:identities.get(pid,dict(pid=pid,role='parent'))),\
                mock.patch.object(parent.deadline,'atomic_json',side_effect=atomic),mock.patch.object(parent.bag_summary,'audit',return_value=dict(complete=True)):
                result=parent.run(cp,rp,out)
            plans=[f.name for f in out.glob('probe_*.planned.json')]
            return copy.deepcopy(result),tool_indices,plans,events

    def test_parent_no_response_does_not_plan_or_launch_negative(self):
        result,tools,plans,events=self.run_parent('none')
        self.assertEqual(tools,[0]);self.assertEqual(plans,['probe_0.planned.json'])
        self.assertEqual(result['state'],'HOLD_DIAGNOSTIC_FAILED');self.assertFalse(result['diagnostic_passed'])
        self.assertEqual([x['index'] for x in result['probes']],[0]);self.assertTrue(result['recorder_exited'])

    def test_parent_wrong_direction_missing_stop_or_illegal_packet_never_negative(self):
        for response in ['wrong','no_stop','illegal']:
            with self.subTest(response=response):
                result,tools,plans,events=self.run_parent(response)
                self.assertEqual(tools,[0]);self.assertNotIn('probe_1.planned.json',plans)
                self.assertEqual(result['state'],'HOLD_DIAGNOSTIC_FAILED');self.assertEqual(len(result['probes']),1)

    def test_parent_success_measures_positive_before_negative_and_no_duplicate_results(self):
        result,tools,plans,events=self.run_parent()
        self.assertEqual(tools,[0,1]);self.assertEqual(result['state'],'COMPLETE_PROBES_STOPPED_PLUS5S_RAW_TAIL')
        self.assertTrue(result['diagnostic_passed']);self.assertEqual([p['index'] for p in result['probes']],[0,1])
        self.assertEqual(events,['observer_closed','shutdown','closed'])

    def test_acceptance_IO_failure_holds_but_does_not_skip_recorder_close(self):
        result,tools,plans,events=self.run_parent(acceptance_io_fail=True)
        self.assertEqual(tools,[0]);self.assertNotIn('probe_1.planned.json',plans)
        self.assertEqual(result['state'],'HOLD_DIAGNOSTIC_FAILED');self.assertTrue(result['recorder_exited'])
        self.assertEqual(events,['observer_closed','shutdown','closed'])

    def test_host_forbidden_includes_backend_but_not_sensor_adapter(self):
        with tempfile.TemporaryDirectory() as directory:
            proc=Path(directory)
            for i,name in enumerate(['semantic_tool_timed.py','sim_imu_adapter.py','adapter_with_gate.py']):
                path=proc/str(90+i);path.mkdir();(path/'status').write_text('Uid:\t1000\t1000\t1000\t1000\n')
                (path/'stat').write_text(str(90+i)+' (python3) S '+' '.join(['0']*18+['123'])+'\n')
                (path/'cmdline').write_bytes(b'/usr/bin/python3\0'+('/safe/'+name).encode()+b'\0')
            self.assertEqual([Path(row['argv'][-1]).name for row in host_preflight.forbidden_live(proc)],['semantic_tool_timed.py'])

    def test_real_original_patch_stop_and_age_guards_preserved(self):
        case=c.read(HERE/'case.HOLD.json');self.assertEqual(c.patch_check(case)['complete_patch_cells'],7200)
        bad=copy.deepcopy(case);bad['patch_xyxy']=[-1.,-2.5,1.,2.5]
        with self.assertRaises(c.Hold):c.patch_check(bad)
        now=10_000_000_000;steady=parent.time.monotonic_ns()
        samples={t:dict(stamp_ns=now,received_steady_ns=steady,payload_valid=True) for t in c.TOPICS};samples['/odom']=gt(now,steady=steady)
        c.check_live(samples,(.073,0.,0.),now,steady)
        for mutate in [lambda d:d['/front_lidar/imu'].update(stamp_ns=now+51_000_000),lambda d:d['/front_lidar'].update(stamp_ns=now-301_000_000),lambda d:d['/odom']['position'].__setitem__(1,.801)]:
            changed=copy.deepcopy(samples);mutate(changed)
            with self.assertRaises(c.Hold):c.check_live(changed,(.073,0.,0.),now,steady)
        for linear,angular,expected in [(0.,0.,True),(.03,.05,True),(.031,0.,False),(0.,.051,False)]:
            tracker=c.StableStop()
            for i in range(32):
                row=gt(1_000_000_000+i*10_000_000,linear=linear,steady=steady-1_000_000_000);row['angular'][2]=angular;tracker.update(row)
            self.assertEqual(tracker.ready(),expected)
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'once';c.claim(path,dict(first=True))
            with self.assertRaises(FileExistsError):c.claim(path,dict(second=True))

    def test_imported_observer_rejects_old_IMU_without_refresh_or_scope_mix(self):
        def cdr(stamp):
            raw=b'\x00\x01\x00\x00'+struct.pack('<iII',stamp//10**9,stamp%10**9,4)+b'map\0'
            raw+=b'\0'*((-len(raw)+4)%8);return raw+struct.pack('<37d',*([0.]*37))
        def decode(raw,unused):
            value=decode_imu_cdr(raw);stamp=value['stamp_ns']
            return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp//10**9,nanosec=stamp%10**9),frame_id='map'),
                angular_velocity=SimpleNamespace(x=0.,y=0.,z=0.),linear_acceleration=SimpleNamespace(x=0.,y=0.,z=0.))
        obj=object.__new__(observer_module.Observer);obj.latest={};obj.history=[];obj.errors=[];obj.counts={t:0 for t in c.TOPICS};obj.stop=c.StableStop()
        obj.stream=io.StringIO();obj.imu_reject_stream=io.StringIO();obj.imu_gate=ImuMonotonicGate();obj.decode=decode
        wall=100_000_000_000
        with mock.patch.object(observer_module,'time',SimpleNamespace(time_ns=lambda:wall,monotonic_ns=lambda:1_000_000_000)):
            obj.capture('/front_lidar/imu',None,cdr(wall));before=copy.deepcopy(obj.latest['/front_lidar/imu'])
        with mock.patch.object(observer_module,'time',SimpleNamespace(time_ns=lambda:wall+200_000_000,monotonic_ns=lambda:1_200_000_000)):
            obj.capture('/front_lidar/imu',None,cdr(wall-1))
        self.assertEqual(obj.latest['/front_lidar/imu'],before);self.assertEqual(obj.errors,[]);self.assertEqual(obj.counts['/front_lidar/imu'],1)
        self.assertFalse(obj.imu_gate.fresh(wall+301_000_000,1_301_000_000));self.assertIs(observer_module.c,c)




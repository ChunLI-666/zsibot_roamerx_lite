"""Pure fixtures only: no ROS context, socket, process launch or real signal."""
import ast
import contextlib
import copy
import io
import json
import math
from pathlib import Path
import struct
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest import mock

sys.dont_write_bytecode=True
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
import readiness_contract as c
import readiness_capture as capture
import owned_recorder as owned
from imu_gate import decode_imu_cdr
from readiness_validator import Validator,GRAPH,TYPES,FRESH,RAW_IMU,IMU,GT,RAW_CAMERA,CAMERA,INFO

CONFIG=c.read(HERE/'sensor_config.json')
def xyz(values):return NS(**dict(zip(('x','y','z'),values)))
def quat(values):return NS(**dict(zip(('x','y','z','w'),values)))
def stamp(ns):return NS(sec=ns//10**9,nanosec=ns%10**9)
def header(ns,frame):return NS(stamp=stamp(ns),frame_id=frame)
def cdr(ns,frame='map',values=None):
    frame=frame.encode()+b'\0';raw=b'\x00\x01\0\0'+struct.pack('<iII',ns//10**9,ns%10**9,len(frame))+frame
    raw+=b'\0'*((-len(raw)+4)%8);return raw+struct.pack('<37d',*(values if values is not None else [0.]*37))
def converted(raw):
    msg=decode_imu_cdr(raw);return cdr(msg['stamp_ns'],'imu_link',msg['values'])
def graph():return {t:[dict(node_name=n,node_namespace=ns,gid=t+':'+n,topic_type=TYPES[t]) for n,ns in owners] for t,owners in GRAPH.items()}
def gid(topic,owner=None):return dict(message_info=dict(source_timestamp=0,received_timestamp=0,publication_sequence_number=None,reception_sequence_number=None),matched_publisher_count=len(GRAPH[topic]))
def message(topic,ns,linear=0.):
    if topic in (RAW_IMU,IMU):return NS(header=header(ns,'map' if topic==RAW_IMU else 'imu_link'))
    if topic==GT:return NS(header=header(ns,'map'),child_frame_id='BASE_LINK',pose=NS(pose=NS(position=xyz([0.,0.,.33]),orientation=quat([0.,0.,0.,1.]))),twist=NS(twist=NS(linear=xyz([linear,0.,0.]),angular=xyz([0.,0.,0.]))))
    if topic=='/front_lidar':return NS(header=header(ns,'map'),width=2,height=1,row_step=52,data=b'x'*52)
    if topic in (RAW_CAMERA,CAMERA):return NS(header=header(ns,'map' if topic==RAW_CAMERA else 'front_camera_optical'),format='jpeg',data=b'x'*120)
    if topic==INFO:
        cam=CONFIG['robot']['sensors']['front_camera'];return NS(header=header(ns,'front_camera_optical'),width=cam['width'],height=cam['height'],k=[cam['fx'],0.,cam['cx'],0.,cam['fy'],cam['cy'],0.,0.,1.])
    if topic=='/clock':return NS(clock=stamp(ns))
    raise AssertionError(topic)
def add_static(v):
    for owner in ['livox_scan_projector','matrix_sensor_contract_adapter']:
        transforms=[]
        for edge,rule in v.static_rules.items():
            if rule['owner'][0]==owner:transforms.append(NS(header=header(0,edge[0]),child_frame_id=edge[1],transform=NS(translation=xyz(rule['values'][:3]),rotation=quat(rule['values'][3:]))))
        v.sample('/tf_static',b'fixture static CDR',NS(transforms=transforms),100_000_000_000,10_000_000_000,gid('/tf_static',owner))
    v.graph(graph(),pin=True)
def feed(v,ns,steady):
    for topic in FRESH:
        raw=cdr(ns) if topic==RAW_IMU else cdr(ns,'imu_link') if topic==IMU else b'fixture CDR'
        v.sample(topic,raw,message(topic,ns),ns,steady,gid(topic))

class Clock:
    def __init__(self):self.t=0.
    def monotonic(self):return self.t
    def monotonic_ns(self):return round(self.t*1e9)
    def time_ns(self):return 100_000_000_000+self.monotonic_ns()
    def sleep(self,n):self.t+=n

class ReadinessFixtures(unittest.TestCase):
    def test_default_HOLD_has_no_node_Popen_or_signal(self):
        with mock.patch.object(sys,'argv',['readiness_capture.py']),mock.patch.object(capture,'Observer',side_effect=AssertionError('Node')),mock.patch.object(capture.subprocess,'Popen',side_effect=AssertionError('Popen')),mock.patch.object(capture,'interrupt',side_effect=AssertionError('signal')),contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertEqual(capture.main(),0)
        self.assertEqual(json.loads(output.getvalue())['state'],'ROOT_HOLD')
        self.assertFalse(json.loads(output.getvalue())['ready_for_motion'])
        with self.assertRaises(c.Hold):c.validate(c.read(HERE/'config.HOLD.json'),c.read(HERE/'root_release.HOLD.json'),HERE/'config.HOLD.json',Path('/not-created'))

    def test_raw_old_rejected_never_refreshes_but_derived_old_is_fatal(self):
        v=Validator(CONFIG,converted);ns=100_000_000_000;steady=10_000_000_000
        v.sample(RAW_IMU,cdr(ns),message(RAW_IMU,ns),ns,steady,gid(RAW_IMU));before=copy.deepcopy(v.latest[RAW_IMU])
        for old in [ns,ns-438744]:
            row=v.sample(RAW_IMU,cdr(old),message(RAW_IMU,old),ns+200_000_000,steady+200_000_000,gid(RAW_IMU))
            self.assertFalse(row['accepted']);self.assertEqual(v.latest[RAW_IMU],before)
        self.assertFalse(v.raw_gate.fresh(ns+301_000_000,steady+301_000_000))
        v.sample(IMU,cdr(ns,'imu_link'),message(IMU,ns),ns,steady,gid(IMU))
        with self.assertRaises(c.Hold):v.sample(IMU,cdr(ns,'imu_link'),message(IMU,ns),ns,steady,gid(IMU))

    def test_all37_IMU_fields_and_header_are_compared_as_full_baseline_CDR(self):
        ns=100_000_000_000;values=[float(i) for i in range(37)];v=Validator(CONFIG,converted)
        v.sample(RAW_IMU,cdr(ns,values=values),message(RAW_IMU,ns),ns,0,gid(RAW_IMU))
        v.sample(IMU,cdr(ns,'imu_link',values),message(IMU,ns),ns,0,gid(IMU));self.assertIn(ns,v.paired)
        ns+=50_000_000;v.sample(RAW_IMU,cdr(ns,values=values),message(RAW_IMU,ns),ns,50_000_000,gid(RAW_IMU))
        values[-1]+=.1
        with self.assertRaises(c.Hold):v.sample(IMU,cdr(ns,'imu_link',values),message(IMU,ns),ns,50_000_000,gid(IMU))

    def test_graph_exact_unique_node_type_GID_and_two_static_publishers(self):
        v=Validator(CONFIG,converted);add_static(v);self.assertEqual(len(v.static),3)
        self.assertEqual(len(graph()['/tf_static']),2)
        for topic,mutate in [(IMU,lambda rows:rows.append(copy.deepcopy(rows[0]))),('/clock',lambda rows:rows[0].update(node_name='other')),
                (RAW_IMU,lambda rows:rows[0].update(topic_type='bad')),('/livox/lidar',lambda rows:rows[0].update(gid='new')),
                ('/tf',lambda rows:rows.append(dict(node_name='native',node_namespace='/',gid='bad',topic_type=TYPES['/tf'])))]:
            with self.subTest(topic=topic):
                bad=graph();mutate(bad[topic])
                with self.assertRaises(c.Hold):v.graph(bad)
        v.measure_start=0
        with self.assertRaises(c.Hold):v.sample(RAW_IMU,cdr(100_000_000_000),message(RAW_IMU,100_000_000_000),100_000_000_000,1,dict(gid(RAW_IMU),matched_publisher_count=2))

    def test_actual_Jazzy_dict_callback_has_no_invented_publisher_GID(self):
        import readiness_observer as module
        obj=object.__new__(module.Observer);obj.validator=Validator(CONFIG,converted);obj.stream=io.StringIO()
        obj.subscriptions={RAW_IMU:NS(get_publisher_count=lambda:1)};ns=100_000_000_000
        obj.decode=lambda raw,typ:message(RAW_IMU,ns)
        info=dict(source_timestamp=ns-1,received_timestamp=ns,publication_sequence_number=3,reception_sequence_number=4)
        with mock.patch.object(module,'time',NS(time_ns=lambda:ns,monotonic_ns=lambda:10_000_000_000)):obj.capture(RAW_IMU,None,cdr(ns),info)
        row=json.loads(obj.stream.getvalue());self.assertEqual(row['message_info'],info);self.assertIsNone(row['publisher_gid']);self.assertEqual(obj.validator.errors,[])

    def test_dynamic_or_new_static_TF_is_rejected(self):
        v=Validator(CONFIG,converted)
        with self.assertRaises(c.Hold):v.sample('/tf',b'x',NS(transforms=[]),0,0,'writer')
        tf=NS(header=header(0,'map'),child_frame_id='odom',transform=NS(translation=xyz([0.,0.,0.]),rotation=quat([0.,0.,0.,1.])))
        with self.assertRaises(c.Hold):v.sample('/tf_static',b'x',NS(transforms=[tf]),0,0,'writer')

    def test_camera_static_rotation_matches_independent_mount_times_optical_matrix(self):
        v=Validator(CONFIG,converted);x,y,z,w=v.static_rules[('base_link','front_camera_optical')]['values'][3:]
        actual=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
        r,p,y=[math.radians(CONFIG['robot']['sensors']['front_camera']['rotation'][k]) for k in ('roll','pitch','yaw')]
        rx=[[1,0,0],[0,math.cos(r),-math.sin(r)],[0,math.sin(r),math.cos(r)]]
        ry=[[math.cos(p),0,math.sin(p)],[0,1,0],[-math.sin(p),0,math.cos(p)]]
        rz=[[math.cos(y),-math.sin(y),0],[math.sin(y),math.cos(y),0],[0,0,1]]
        multiply=lambda a,b:[[sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
        expected=multiply(multiply(multiply(rz,ry),rx),[[0,0,1],[-1,0,0],[0,-1,0]])
        for i in range(3):
            for j in range(3):self.assertAlmostEqual(actual[i][j],expected[i][j],places=12)

    def test_camera_bytes_and_original_headers_are_not_reencoded(self):
        ns=100_000_000_000;v=Validator(CONFIG,converted)
        v.sample(RAW_CAMERA,b'raw CDR',message(RAW_CAMERA,ns),ns,0,gid(RAW_CAMERA))
        v.sample(CAMERA,b'derived CDR',message(CAMERA,ns),ns,0,gid(CAMERA));self.assertIn(ns,v.camera_paired)
        other=message(CAMERA,ns+100_000_000);other.data=b'y'*120
        v.sample(RAW_CAMERA,b'raw CDR2',message(RAW_CAMERA,ns+100_000_000),ns+100_000_000,100_000_000,gid(RAW_CAMERA))
        with self.assertRaises(c.Hold):v.sample(CAMERA,b'derived CDR2',other,ns+100_000_000,100_000_000,gid(CAMERA))

    def test_valid_ten_second_core_window_and_freshness_stop_limits(self):
        v=Validator(CONFIG,converted);add_static(v);current=[10_000_000_000]
        with mock.patch.object(c,'time',NS(monotonic_ns=lambda:current[0])):
            for i in range(8):
                current[0]=10_000_000_000+i*50_000_000;feed(v,100_000_000_000+i*50_000_000,current[0])
            v.measure_start=10_400_000_000
            for i in range(201):
                current[0]=v.measure_start+i*50_000_000;wall=100_400_000_000+i*50_000_000
                feed(v,wall,current[0]);v.check(wall,current[0])
            v.measure_end=current[0];result=v.finish(wall,current[0]);self.assertTrue(result['complete']);self.assertFalse(result['ready_for_motion'])
            for topic in FRESH:self.assertEqual(result['core_topic_statistics'][topic]['count'],201)
            with self.assertRaises(c.Hold):v.check(wall+301_000_000,current[0]+301_000_000)
            bad=message(GT,wall+10_000_000,linear=.031)
            current[0]+=10_000_000;v.sample(GT,b'GT',bad,wall+10_000_000,current[0],gid(GT))
            with self.assertRaises(c.Hold):v.check(wall+10_000_000,current[0])

    def test_unknown_clock_and_unmatched_core_are_reported_not_silently_skipped(self):
        v=Validator(CONFIG,converted);ns=100_000_000_000;v.measure_start=0;v.measure_end=10_000_000_000
        v.sample('/clock',b'clock',message('/clock',ns),ns,1,gid('/clock'))
        report=v.finish(ns,1);self.assertFalse(report['complete']);self.assertIn(['clock',ns],report['unmatched_core_samples'])
        with self.assertRaises(c.Hold):v.sample('/clock',b'clock',message('/clock',ns),ns,2,gid('/clock'))

    def test_source_identity_drift_is_read_only_HOLD(self):
        identities={role:dict(pid=i,starttime_ticks=99,cmdline_sha256='a'*64,argv=['source'],boot_id='fixture') for i,role in enumerate(['new_adapter','bridge24','projector61'],70)}
        with mock.patch.object(capture.deadline,'process_identity',side_effect=lambda pid:dict(next(v for v in identities.values() if v['pid']==pid))):self.assertEqual(c.source_live(dict(source_identities=identities)),identities)
        with mock.patch.object(capture.deadline,'process_identity',return_value=dict(identities['new_adapter'],starttime_ticks=100)):
            with self.assertRaises(c.Hold):c.source_live(dict(source_identities=identities))

    def mock_owner(self,ack=True,magic=True,validation_error=False):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);out=root/'run';clock=Clock();events=[];observer_calls=[];commands=[]
            config=c.read(HERE/'config.HOLD.json');config['runtime_config_artifact']['path']=str(HERE/'sensor_config.json')
            cp=root/'config.json';cp.write_text(json.dumps(config));rp=root/'release.json';rp.write_text(json.dumps(dict(release_id='e'*32,package_sha256=c.package_hashes())))
            class Process:
                pid=201;returncode=None
                def poll(self):return self.returncode
                def wait(self,timeout):self.returncode=0;return 0
            process=Process();identity=dict(pid=201)
            class Guard:
                children={'recorder':dict(wrapper_argv=['fixture'],executed_argv=['fixture'])};process=NS(wait=lambda timeout:0)
                def check_live(self):pass
                def owned_command(self,name,argv,env):
                    commands.append((name,argv));return ['fixture'],env
                def wait_for_child(self,name,pid):
                    events.append('ACK')
                    if not ack:raise c.Hold('fixture missing ACK')
                    capture.deadline.atomic_json(out/'recording_deadline_recorder_registration.json',dict(identity=identity))
                def begin_shutdown(self,reason):events.append('shutdown')
                def recording_closed(self,reason):events.append('closed')
            class FakeValidator:
                errors=[];measure_start=None;measure_end=None
                def graph(self,snapshot,pin=False):pass
                def check(self,wall,steady):
                    if validation_error:raise c.Hold('fixture stale')
                def finish(self,wall,steady):return dict(complete=not validation_error,ready_for_motion=False)
            class FakeObserver:
                def __init__(self,*args):observer_calls.append(True);self.validator=FakeValidator()
                def spin(self):clock.t+=.02
                def graph(self):return {}
                def close(self):events.append('observer_closed')
            def fake_child(argv,log,env,check=None):
                (out/'bag').mkdir()
                if magic:(out/'bag/fixture.mcap').write_bytes(capture.bag_summary.MAGIC)
                if check:check()
                return dict(process=process,fd=999,signalled=False)
            def stop(item,allowed):self.assertIs(item['process'],process);events.append('recorder_SIGINT');item['signalled']=True
            with mock.patch.object(c,'validate'),mock.patch.object(c,'source_live',return_value={}),mock.patch.object(c,'artifacts',return_value=[]),mock.patch.object(c,'host_proof'),\
                mock.patch.object(capture,'time',clock),mock.patch.object(capture,'child',side_effect=fake_child),mock.patch.object(capture,'interrupt',side_effect=stop),\
                mock.patch.object(capture,'Observer',FakeObserver),mock.patch.object(capture.deadline,'arm',return_value=Guard()) as arm,\
                mock.patch.object(capture.deadline,'process_identity',return_value=identity),mock.patch.object(capture.os,'close'),\
                mock.patch.object(capture.signal,'signal',return_value=None),mock.patch.object(capture.subprocess,'Popen',side_effect=AssertionError('real Popen forbidden')),\
                mock.patch.object(capture.bag_summary,'audit',return_value=dict(complete=True)):
                result=capture.run(cp,rp,out);arm.assert_called_once_with(out,45.)
            return result,events,observer_calls,commands

    def test_owner_never_subscribes_before_ACK_and_MCAP(self):
        for ack,magic in [(False,True),(True,False)]:
            result,events,calls,commands=self.mock_owner(ack,magic)
            self.assertEqual(calls,[]);self.assertEqual(result['state'],'HOLD_READINESS_FAILED');self.assertTrue(result['recorder_exited'])
            self.assertEqual([name for name,argv in commands],['recorder']);self.assertIn('recorder_SIGINT',events)
            self.assertLess(events.index('shutdown'),events.index('recorder_SIGINT'));self.assertLess(events.index('recorder_SIGINT'),events.index('closed'))

    def test_owner_only_owned_recorder_closed_and_report_never_authorizes_motion(self):
        for fail in [False,True]:
            result,events,calls,commands=self.mock_owner(validation_error=fail)
            self.assertFalse(result['ready_for_motion']);self.assertEqual(result['source_signals_sent'],0);self.assertEqual(result['scene_signals_sent'],0)
            self.assertEqual(result['readiness_observed'],not fail);self.assertEqual(events.count('recorder_SIGINT'),1)
            argv=commands[0][1];self.assertIn('-a',argv);self.assertIn('--include-hidden-topics',argv);self.assertIn('zstd_fast',argv)

    def test_exec_empty_birth_cleanup_and_wrong_identity_never_signal_source(self):
        base=dict(pid=77,starttime_ticks=123,boot_id='fixture');valid=dict(base,argv=['python','owned'],cmdline_sha256='a'*64)
        for wrong in [False,True]:
            with tempfile.TemporaryDirectory() as directory:
                events=[]
                class Process:
                    pid=77;returncode=None
                    def poll(self):return self.returncode
                    def wait(self,timeout):events.append('wait');self.returncode=0;return 0
                process=Process();second=dict(valid,argv=['source']) if wrong else valid
                def fail():raise c.Hold('fixture source stale during exec')
                with mock.patch.object(owned.subprocess,'Popen',return_value=process),mock.patch.object(owned.os,'pidfd_open',return_value=999),\
                    mock.patch.object(owned.select,'select',return_value=([],[],[])),mock.patch.object(owned,'stat_birth',return_value=base),\
                    mock.patch.object(owned.deadline,'process_identity',side_effect=[ProcessLookupError('exec empty'),second]),mock.patch.object(owned.deadline,'command_argv',return_value=['python','owned']),\
                    mock.patch.object(owned.signal,'pidfd_send_signal',side_effect=lambda fd,sig:events.append(('signal',fd))),mock.patch.object(owned.os,'close',side_effect=lambda fd:events.append(('close',fd))):
                    with self.assertRaises(c.Hold):owned.child(['python','owned'],Path(directory)/'recorder.log',{},check=fail)
                self.assertEqual(('signal',999) in events,not wrong);self.assertIn('wait',events);self.assertEqual(events[-1],('close',999))

if __name__=='__main__':unittest.main(verbosity=2)

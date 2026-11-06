"""Fake executor/time only; no ROS context, Node, child, socket or signal."""
import ast
from collections import deque
import hashlib
import io
import json
from pathlib import Path
import struct
from types import SimpleNamespace as NS
import unittest
from unittest import mock

import readiness_observer as observer
import readiness_contract as c
from readiness_validator import Validator,TOPICS,FRESH,GRAPH,RAW_IMU,IMU,GT,RAW_CAMERA,CAMERA,INFO
from imu_gate import decode_imu_cdr

HERE=Path(__file__).resolve().parent
BASE=next(p for p in HERE.parents if p.name=='planner_comparison_20261105')
CONFIG=json.loads((HERE/'sensor_config.json').read_text())
WALL=1_800_000_000_000_000_000
STEADY=10_000_000_000
LOAD_COMPARISON={}

class Clock:
    def __init__(self):self.now=0
    def monotonic_ns(self):return STEADY+self.now
    def time_ns(self):return WALL+self.now

def bare(fake,clock):
    obj=object.__new__(observer.Observer)
    obj.node=None;obj.rclpy=fake;obj.validator=NS(received={'fixture':0},errors=[])
    return obj

def header(ns,frame):return NS(stamp=NS(sec=ns//10**9,nanosec=ns%10**9),frame_id=frame)
def xyz(a):return NS(x=a[0],y=a[1],z=a[2])
def quat(a):return NS(x=a[0],y=a[1],z=a[2],w=a[3])
def cdr(ns,frame):
    frame=frame.encode()+b'\0';raw=b'\0\1\0\0'+struct.pack('<iII',ns//10**9,ns%10**9,len(frame))+frame
    return raw+b'\0'*((-len(raw)+4)%8)+struct.pack('<37d',*([0.]*37))
def converted(raw):return cdr(decode_imu_cdr(raw)['stamp_ns'],'imu_link')
def message(topic,ns):
    if topic in (RAW_IMU,IMU):return NS(header=header(ns,'map' if topic==RAW_IMU else 'imu_link'))
    if topic==GT:return NS(header=header(ns,'map'),child_frame_id='BASE_LINK',pose=NS(pose=NS(position=xyz([0.,0.,.33]),orientation=quat([0.,0.,0.,1.]))),twist=NS(twist=NS(linear=xyz([0.,0.,0.]),angular=xyz([0.,0.,0.]))))
    if topic=='/front_lidar':return NS(header=header(ns,'map'),width=2,height=1,row_step=52,data=b'x'*52)
    if topic in (RAW_CAMERA,CAMERA):return NS(header=header(ns,'map' if topic==RAW_CAMERA else 'front_camera_optical'),format='jpeg',data=b'x'*120)
    if topic==INFO:
        cam=CONFIG['robot']['sensors']['front_camera'];return NS(header=header(ns,'front_camera_optical'),width=cam['width'],height=cam['height'],k=[cam['fx'],0.,cam['cx'],0.,cam['fy'],cam['cy'],0.,0.,1.])
    if topic=='/clock':return NS(clock=NS(sec=ns//10**9,nanosec=ns%10**9))
    raise AssertionError(topic)
def evidence(topic):return dict(message_info=dict(source_timestamp=0,received_timestamp=0,publication_sequence_number=None,reception_sequence_number=None),matched_publisher_count=len(GRAPH[topic]))

class DigestStream:
    def __init__(self):self.lines=0;self.digest=hashlib.sha256();self.bytes=0
    def write(self,text):self.lines+=text.count('\n');data=text.encode();self.digest.update(data);self.bytes+=len(data)
    def flush(self):pass

class QueueExecutor:
    """Deterministic round-robin per-topic delivery, with explicit fake costs.

    Models1160 standard callbacks/s,1ms source transport,120us dispatch/capture,
    and outer-watch750us+50ns per historical callback. These are stated model
    assumptions, not measured ROS execution times or a predicted live PASS.
    """
    def __init__(self,clock,obj):
        self.clock=clock;self.obj=obj;self.at=0;self.next_topic=0;self.attempts=0;self.processed=0;self.original_headers=[];self.receipts=[]
        self.queues={t:deque() for t in FRESH};self.events=[]
        periods={RAW_IMU:2_000_000,IMU:2_000_000,GT:10_000_000,'/front_lidar':100_000_000,RAW_CAMERA:100_000_000,CAMERA:100_000_000,INFO:100_000_000,'/clock':50_000_000}
        for j,topic in enumerate(FRESH):
            for stamp in range(0,10_750_000_001,periods[topic]):self.events.append((stamp+1_000_000+j*100,topic,WALL+stamp))
        self.events.sort()
    def enqueue(self):
        while self.at<len(self.events) and self.events[self.at][0]<=self.clock.now:
            arrival,topic,stamp=self.events[self.at];self.queues[topic].append((arrival,stamp));self.at+=1
    def spin_once(self,node,timeout_sec):
        self.attempts+=1;self.enqueue()
        if not any(self.queues.values()):
            until=self.clock.now+round(timeout_sec*1e9)
            self.clock.now=min(until,self.events[self.at][0]) if self.at<len(self.events) else until
            self.enqueue()
            if not any(self.queues.values()):return
        for offset in range(len(FRESH)):
            index=(self.next_topic+offset)%len(FRESH);topic=FRESH[index]
            if self.queues[topic]:break
        self.next_topic=(index+1)%len(FRESH)
        arrival,stamp=self.queues[topic].popleft();msg=message(topic,stamp)
        raw=cdr(stamp,'map' if topic==RAW_IMU else 'imu_link') if topic in (RAW_IMU,IMU) else topic.encode()+struct.pack('<q',stamp)
        self.obj.decode=lambda data,typ:msg
        self.obj.capture(topic,None,raw,evidence(topic)['message_info'])
        self.original_headers.append((topic,stamp));self.receipts.append(self.clock.time_ns());self.processed+=1
        self.clock.now+=120_000

def workload(spin):
    clock=Clock();obj=object.__new__(observer.Observer);obj.node=None
    obj.validator=Validator(CONFIG,converted);obj.stream=DigestStream()
    obj.subscriptions={t:NS(get_publisher_count=lambda t=t:len(GRAPH[t])) for t in TOPICS}
    # Static graph/values are the already-reviewed setup; no ROS operation.
    for edge,rule in obj.validator.static_rules.items():obj.validator.static[edge]=dict(values=rule['values'],expected_owner_node=rule['owner'],callback_publisher_gid=None)
    obj.validator.measure_start=STEADY+400_000_000
    executor=QueueExecutor(clock,obj);obj.rclpy=executor;checks=0;max_gap=0;previous=None;error=None
    with mock.patch.object(observer,'time',clock),mock.patch.object(c,'time',clock):
        while clock.now<10_400_000_000:
            spin(obj)
            if clock.now>=400_000_000:
                checks+=1
                if previous is not None:max_gap=max(max_gap,clock.monotonic_ns()-previous)
                previous=clock.monotonic_ns()
                try:
                    if max_gap>100_000_000:raise c.Hold('original100ms watch gap')
                    obj.validator.check(clock.time_ns(),clock.monotonic_ns())
                except c.Hold as exc:error=str(exc);break
            # Deterministic explicit external-check cost, not fake callback age.
            clock.now+=750_000+executor.processed*50
        core_reached=clock.now>=10_400_000_000 and error is None
        if core_reached:
            obj.validator.measure_end=STEADY+10_400_000_000;end=clock.now+350_000_000
            while clock.now<end:
                spin(obj);obj.validator.check(clock.time_ns(),clock.monotonic_ns())
                clock.now+=750_000+executor.processed*50
        report=obj.validator.finish(clock.time_ns(),clock.monotonic_ns())
    retained=executor.processed+sum(len(q) for q in executor.queues.values())+len(executor.events)-executor.at
    return dict(core_reached=core_reached,complete=report['complete'],error=error,simulated_sec=clock.now/1e9,outer_checks=checks,
        callbacks=executor.processed,scheduled=len(executor.events),all_callbacks_retained=retained==len(executor.events),
        logged_callbacks=obj.stream.lines,original_headers_not_rewritten=True,max_watch_gap_ms=max_gap/1e6,
        max_callback_age_ms={t:s['max_callback_age_ns']/1e6 for t,s in obj.validator.stats.items()},
        model_assumptions='1160callbacks/s;1ms transport;120us callback;750us+50ns/history entry outer-watch;round-robin ready topics; no ROS runtime proof')

class BoundedSpinFixtures(unittest.TestCase):
    def test_idle_wait_is_at_most10ms(self):
        clock=Clock();timeouts=[]
        def once(node,timeout_sec):timeouts.append(timeout_sec);clock.now+=round(timeout_sec*1e9)
        obj=bare(NS(spin_once=once),clock)
        with mock.patch.object(observer,'time',clock):obj.spin()
        self.assertEqual(timeouts,[.01]);self.assertEqual(clock.now,10_000_000)
    def test_ready_queue_stops_at32_without_losing_items(self):
        clock=Clock();pending=list(range(100));seen=[];obj=bare(None,clock)
        def once(node,timeout_sec):seen.append(pending.pop(0));obj.validator.received['fixture']+=1
        obj.rclpy=NS(spin_once=once)
        with mock.patch.object(observer,'time',clock):obj.spin()
        self.assertEqual(seen,list(range(32)));self.assertEqual(pending,list(range(32,100)))
    def test_time_budget_prevents_next_dispatch_and_bounds_all_timeouts(self):
        clock=Clock();timeouts=[];obj=bare(None,clock)
        def once(node,timeout_sec):timeouts.append(timeout_sec);clock.now+=4_000_000;obj.validator.received['fixture']+=1
        obj.rclpy=NS(spin_once=once)
        with mock.patch.object(observer,'time',clock):obj.spin()
        self.assertEqual(timeouts,[.01,.006,.002]);self.assertEqual(obj.validator.received['fixture'],3)
        self.assertEqual(clock.now,12_000_000)  # Last synchronous callback is not preemptible.
    def test_slow_single_callback_does_not_start_another(self):
        clock=Clock();obj=bare(None,clock);attempts=[]
        def once(node,timeout_sec):attempts.append(True);clock.now+=120_000_000;obj.validator.received['fixture']+=1
        obj.rclpy=NS(spin_once=once)
        with mock.patch.object(observer,'time',clock):obj.spin()
        self.assertEqual(len(attempts),1);self.assertGreater(clock.now,100_000_000)
    def test_fault_or_nondataprogress_returns_immediately(self):
        for fault in (False,True):
            clock=Clock();obj=bare(None,clock);attempts=[]
            def once(node,timeout_sec):
                attempts.append(True)
                if fault:obj.validator.received['fixture']+=1;obj.validator.errors.append('original fault')
            obj.rclpy=NS(spin_once=once)
            with mock.patch.object(observer,'time',clock):obj.spin()
            self.assertEqual(len(attempts),1)
    def test_callback_exception_is_not_swallowed(self):
        clock=Clock();obj=bare(NS(spin_once=lambda *a,**k:(_ for _ in ()).throw(OSError('callback/log failure'))),clock)
        with mock.patch.object(observer,'time',clock),self.assertRaisesRegex(OSError,'callback/log failure'):obj.spin()
    def test_actual_capture_with_fake_load_preserves_callbacks_and_guards(self):
        old_tree=ast.parse((BASE/'sim_imu_adapter_readiness_v1/review_source_v2/readiness_observer.py').read_text())
        old_spin=next(n for n in ast.walk(old_tree) if isinstance(n,ast.FunctionDef) and n.name=='spin')
        namespace={};exec(compile(ast.Module(body=[old_spin],type_ignores=[]),'<exact v2 spin>','exec'),namespace)
        old=workload(namespace['spin']);new=workload(observer.Observer.spin)
        self.assertFalse(old['core_reached']);self.assertIn('stale',old['error'])
        self.assertTrue(new['core_reached']);self.assertTrue(new['complete']);self.assertIsNone(new['error'])
        self.assertTrue(new['all_callbacks_retained']);self.assertEqual(new['callbacks'],new['logged_callbacks'])
        self.assertLess(new['max_watch_gap_ms'],100.)
        LOAD_COMPARISON.update(old=old,new=new)

if __name__=='__main__':unittest.main(verbosity=2)

"""Actual ROS CDR serializer comparisons and fake executor. No ROS init/Node/goal."""
import ast
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import statistics
import struct
import time
from types import MethodType, SimpleNamespace
import unittest

HERE=Path(__file__).resolve().parent
SOURCE=HERE/'goal_pilot_source/waypoint_batch.py'
FROZEN=HERE/'goal_pilot_source/pilot_scan_timing.py'
assert hashlib.sha256(FROZEN.read_bytes()).hexdigest()=='a63bce9490f55438e7af5d16025e4dd20d86293cec12f1560d403ecb629cbe31'
def function(path,name,namespace,cls=None):
    root=ast.parse(path.read_text());scope=root.body if cls is None else next(n for n in root.body if isinstance(n,ast.ClassDef) and n.name==cls).body
    node=next(n for n in scope if isinstance(n,ast.FunctionDef) and n.name==name)
    exec(compile(ast.Module(body=[node],type_ignores=[]),str(path),'exec'),namespace);return namespace[name]
parser=function(SOURCE,'path_cdr_endpoint',dict(struct=struct,math=math))

class RawPathTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from nav_msgs.msg import Path as NavPath
        from geometry_msgs.msg import PoseStamped
        from rclpy.serialization import serialize_message,deserialize_message
        cls.Path=NavPath;cls.Pose=PoseStamped;cls.serialize=staticmethod(serialize_message);cls.deserialize=staticmethod(deserialize_message)
    def make(self,count=3,variable=True):
        msg=self.Path();msg.header.frame_id='map';msg.header.stamp.sec=1794039000;msg.header.stamp.nanosec=123456789
        for i in range(count):
            p=self.Pose();p.header.frame_id=['','m','map','odom-long-frame','地图'][i%5] if variable else 'map'
            p.header.stamp.sec=i-3;p.header.stamp.nanosec=i*123
            p.pose.position.x=i*.001;p.pose.position.y=-i*.002;p.pose.position.z=i*.0001
            a=i*.003;p.pose.orientation.z=math.sin(a/2.);p.pose.orientation.w=math.cos(a/2.);msg.poses.append(p)
        return msg
    def expected(self,msg):
        endpoint=None
        if msg.poses:
            p=msg.poses[-1].pose;q=p.orientation
            endpoint=dict(x=p.position.x,y=p.position.y,yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        return dict(frame_id=msg.header.frame_id,stamp_ns=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec,poses=len(msg.poses),endpoint=endpoint)
    def encode(self,msg,endian):
        data=bytearray(b'\x00\x01\x00\x00' if endian=='<' else b'\x00\x00\x00\x00')
        def align(n):data.extend(b'\0'*((-(len(data)-4))%n))
        def header(h):
            align(4);encoded=h.frame_id.encode()+b'\0';data.extend(struct.pack(endian+'iII',h.stamp.sec,h.stamp.nanosec,len(encoded)));data.extend(encoded)
        header(msg.header);align(4);data.extend(struct.pack(endian+'I',len(msg.poses)))
        for p in msg.poses:
            header(p.header);align(8);v=p.pose.position;q=p.pose.orientation;data.extend(struct.pack(endian+'7d',v.x,v.y,v.z,q.x,q.y,q.z,q.w))
        return bytes(data)
    def test_actual_serializer_variable_headers_and_5804_pose_payload(self):
        for count in [0,1,3,13,5804]:
            with self.subTest(count=count):
                msg=self.make(count);raw=self.serialize(msg);self.assertEqual(parser(raw),self.expected(msg))
                self.assertEqual(self.deserialize(self.encode(msg,'<'),self.Path),msg)
    def test_big_endian_against_actual_ROS_deserializer(self):
        for count in [0,1,7,5804]:
            with self.subTest(count=count):
                msg=self.make(count);raw=self.encode(msg,'>');decoded=self.deserialize(raw,self.Path)
                self.assertEqual(decoded,msg);self.assertEqual(parser(raw),self.expected(decoded))
    def test_alignment_for_outer_and_inner_variable_utf8_frames(self):
        for frame in ['', 'm','ma','map','odom','abcde','地图','map-long-frame']:
            msg=self.make(9);msg.header.frame_id=frame;raw=self.serialize(msg)
            self.assertEqual(parser(raw),self.expected(self.deserialize(raw,self.Path)))
    def test_malformed_lengths_truncation_and_unknown_encapsulation_rejected(self):
        raw=self.serialize(self.make(3))
        for data in [b'',raw[:3],raw[:13],raw[:-1],b'\x00\x07\0\0'+raw[4:],b'\x00\x03\0\0'+raw[4:]]:
            with self.subTest(length=len(data)),self.assertRaises(ValueError):parser(data)
        bad=bytearray(raw);struct.pack_into('<I',bad,12,0xffffffff)
        with self.assertRaises(ValueError):parser(bad)
    def test_raw_subscription_is_only_original_plan_and_QoS_is_identical(self):
        kind=object();qos=object();calls=[]
        class Base:
            def sub(self,cls,topic,callback):self.subscriptions_kept.append(self.create_subscription(cls,topic,callback,qos))
        frozen=SimpleNamespace(Pilot=Base,NavPath=kind,qos_profile_sensor_data=qos)
        method=function(SOURCE,'sub',dict(frozen=frozen),'WaypointPilot')
        node=SimpleNamespace(subscriptions_kept=[],create_subscription=lambda *args,**kw:(calls.append((args,kw)) or len(calls)))
        for topic in ['/lightning/pose','/odom/current_pose','/lightning/loc_status','/lightning/pose_valid','/laser_scan','/cmd_vel_nav','/cmd_vel','/cmd_vel_safe','/tf','/camera/front','/camera/info','/plan','/rosout']:
            method(node,kind if topic=='/plan' else object(),topic,lambda m:None)
        self.assertEqual(len(calls),13);self.assertEqual([args[1] for args,kw in calls if kw.get('raw')],['/plan'])
        self.assertTrue(all(args[3] is qos for args,kw in calls))
    def test_raw_path_retains_frozen_validator_and_reviewed_wrapper_guards(self):
        self.assertEqual(hashlib.sha256(FROZEN.read_bytes()).hexdigest(),'a63bce9490f55438e7af5d16025e4dd20d86293cec12f1560d403ecb629cbe31')
        # execution_ready is the independently reviewed f163f509 scan-profile
        # wrapper. Default/operational/strict behavior has separate integration
        # fixtures; original cancellation and confirmation methods stay frozen.
        expected={'execution_ready': '039da0ff6cba93ec337c25c03b258a9c3bc54a0781b7ef921259c8bdf4a3fe1c', 'confirm_scan_aging': 'db28cdd3018c42ba7c38169d33f45f046759cc90fc1e277763ec2b426d6cfebb', 'cancel_own': '3c8c0e7e689262980e1bbadbad67bb62f398f23ff4eec3d57bf406033e9b57ce'}
        cls=next(n for n in ast.parse(SOURCE.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='WaypointPilot')
        observed={n.name:hashlib.sha256(ast.dump(n,include_attributes=False).encode()).hexdigest() for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in expected}
        self.assertEqual(observed,expected)

class FakeClock:
    def __init__(self):self.ns=100000000000
    def monotonic_ns(self):return self.ns
    def monotonic(self):return self.ns/1e9
    def thread_time_ns(self):return self.ns//10
class PumpTests(unittest.TestCase):
    def pilot(self,callback_queue=None):
        clock=FakeClock();queue=list(callback_queue or []);terminal=SimpleNamespace(value=False);terminal.done=lambda:terminal.value
        node=SimpleNamespace(counts=Counter(),args=SimpleNamespace(timeout=100.,cancel_after=None),cancel_requested=False,goal_sent_steady=clock.monotonic(),runtime_last_pump=None)
        def spin(n,timeout_sec):
            if queue:queue.pop(0)(n,clock);clock.ns+=500000
            else:clock.ns+=int(timeout_sec*1e9)
        ros=SimpleNamespace(ok=lambda:True,spin_once=spin)
        node.check_interruption=lambda:None
        node.pump=MethodType(function(SOURCE,'pump_runtime_inputs',dict(time=clock,rclpy=ros),'WaypointPilot'),node)
        return node,clock,queue,terminal
    def healthy_snapshot(self,node,clock,source_stamp=None):
        last={k:dict(receive_age_s=clock.monotonic()-node.received.get(k,0.)) for k in ['pose','odom','loc_status','pose_valid','scan']}
        last['pose'].update(frame_id='map',x=0.,y=0.,yaw=0.,header_age_s=0.)
        last['odom'].update(frame_id='odom',child_frame_id='base_link',vx=0.,vy=0.,wz=0.,header_age_s=0.)
        last['loc_status']['value']=1;last['pose_valid']['value']=True
        last['scan'].update(frame_id='base_link',ranges=1,header_age_s=clock.monotonic()-(source_stamp if source_stamp is not None else clock.monotonic()))
        return dict(last=last,tf={k:dict(regressed=False,header_age_s=0.,advance_age_s=0.) for k in ['map->odom','odom->base_link']})
    def test_pending_status_valid_scan_are_serviced_before_original_check(self):
        callbacks=[]
        for topic in ['pose']*8+['odom']*8+['loc_status','pose_valid','scan']:
            callbacks.append(lambda n,c,key=topic:(n.counts.update([key]),n.received.update({key:c.monotonic()})))
        node,clock,queue,terminal=self.pilot(callbacks);node.received={k:clock.monotonic()-.31 for k in ['pose','odom','loc_status','pose_valid','scan']}
        original=function(FROZEN,'input_ready',dict(math=math),'Pilot');node.current_summary=lambda:self.healthy_snapshot(node,clock)
        self.assertFalse(original(node,require_camera=False)[0]);trace=node.pump(terminal)
        self.assertFalse(queue);self.assertTrue(original(node,require_camera=False)[0]);self.assertGreaterEqual(trace['spin_calls'],19)
        for key in ['loc_status','pose_valid','scan']:self.assertEqual(trace['observed_count_deltas'][key],1)
    def test_true_old_header_and_missing_status_still_fail_300ms(self):
        node,clock,queue,terminal=self.pilot();node.received={k:clock.monotonic()-.301 for k in ['pose','odom','loc_status','pose_valid','scan']}
        old=clock.monotonic()-.301;node.pump(terminal)
        # A received scan retains its genuinely old original header; other sources stay missing.
        node.received['scan']=clock.monotonic()
        original=function(FROZEN,'input_ready',dict(math=math),'Pilot');node.current_summary=lambda:self.healthy_snapshot(node,clock,old)
        valid,issues,_=original(node,require_camera=False);self.assertFalse(valid)
        self.assertIn('loc_status:missing_or_stale_receive',issues);self.assertIn('scan:invalid_header_age',issues)
    def test_terminal_and_cancel_stop_after_one_callback(self):
        node,clock,queue,terminal=self.pilot([lambda n,c:setattr(terminal,'value',True)]*20)
        trace=node.pump(terminal);self.assertEqual(trace['spin_calls'],1);self.assertEqual(trace['stop_reason'],'terminal');self.assertEqual(len(queue),19)
        node,clock,queue,terminal=self.pilot([lambda n,c:setattr(n,'cancel_requested',True)]*20)
        trace=node.pump(terminal);self.assertEqual(trace['spin_calls'],1);self.assertEqual(trace['stop_reason'],'cancel_pending')
    def test_deadline_and_interruption_checked_per_callback(self):
        node,clock,queue,terminal=self.pilot([lambda n,c:n.counts.update(['pose'])]*30);node.args.cancel_after=.001
        trace=node.pump(terminal);self.assertEqual(trace['spin_calls'],2);self.assertEqual(trace['stop_reason'],'existing_deadline')
        node,clock,queue,terminal=self.pilot([lambda n,c:n.counts.update(['pose'])]*30);calls=[]
        def interrupt():
            calls.append(1)
            if len(calls)==3:raise InterruptedError('same controlled cleanup path')
        node.check_interruption=interrupt
        with self.assertRaises(InterruptedError):node.pump(terminal)
        self.assertEqual(node.runtime_last_pump['spin_calls'],2)
    def test_single_callback_overrun_is_recorded_not_hidden(self):
        node,clock,queue,terminal=self.pilot([lambda n,c:setattr(c,'ns',c.ns+240000000)])
        trace=node.pump(terminal);self.assertGreaterEqual(trace['overrun_ns'],220000000);self.assertEqual(trace['spin_calls'],1)

if __name__=='__main__':unittest.main(verbosity=2)

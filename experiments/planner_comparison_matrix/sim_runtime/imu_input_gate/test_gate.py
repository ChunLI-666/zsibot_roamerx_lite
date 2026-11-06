#!/usr/bin/python3
"""Pure/CDR fixtures. No rclpy.init, Node, ROS publish, source action or UDP."""
import ast
import base64
import copy
import io
import json
from pathlib import Path
import struct
import sys
from types import SimpleNamespace
import unittest
from unittest import mock

from rclpy.serialization import deserialize_message,serialize_message
from sensor_msgs.msg import Imu,PointCloud2,CompressedImage
from nav_msgs.msg import Odometry

import imu_gate as g
import calibration_observer as observer
import sim_imu_adapter

HERE=Path(__file__).resolve().parent
BASE=100_000_000_000

def cdr(stamp,frame='map',nonfinite=False):
    data=b'\x00\x01\x00\x00'+struct.pack('<iII',stamp//1_000_000_000,stamp%1_000_000_000,len(frame)+1)+frame.encode()+b'\0'
    data+=b'\0'*((-len(data)+4)%8);values=[0.]*37;values[3]=1.
    if nonfinite:values[17]=float('nan')
    return data+struct.pack('<37d',*values)

def fake_observer():
    obj=object.__new__(observer.Observer);obj.latest={};obj.history=[];obj.errors=[];obj.counts={t:0 for t in observer.c.TOPICS}
    obj.stop=observer.c.StableStop();obj.stream=io.StringIO();obj.imu_reject_stream=io.StringIO()
    obj.imu_gate=g.ImuMonotonicGate();obj.decode=deserialize_message
    return obj

class Tests(unittest.TestCase):
    def test_last_accepted_not_last_received(self):
        gate=g.ImuMonotonicGate();accepted=[]
        for i,offset in enumerate([10_000_000,9_000_000,9_500_000,10_000_000,11_000_000]):
            event=gate.inspect(cdr(BASE+offset),BASE+20_000_000+i*100_000,1_000_000_000+i*100_000)
            if event['accepted']:accepted.append(event['header_ns'])
            else:self.assertEqual(event['last_accepted_header_ns'],BASE+10_000_000)
        self.assertEqual(accepted,[BASE+10_000_000,BASE+11_000_000]);self.assertEqual(gate.rejected,3)

    def test_reject_cdr_is_exact_and_no_receipt_refresh(self):
        gate=g.ImuMonotonicGate();gate.inspect(cdr(BASE),BASE,1_000_000_000)
        raw=cdr(BASE-1);event=gate.inspect(raw,BASE+200_000_000,1_200_000_000)
        self.assertEqual(base64.b64decode(event['original_cdr_base64']),raw)
        self.assertEqual(event['callback_receipt_unix_ns'],BASE+200_000_000)
        self.assertEqual(gate.last_accepted_receipt_steady_ns,1_000_000_000)
        self.assertFalse(gate.fresh(BASE+301_000_000,1_301_000_000))

    def test_clock_reset_and_continuous_old_packets_do_not_revive(self):
        gate=g.ImuMonotonicGate();gate.inspect(cdr(BASE),BASE,1_000_000_000)
        for i in range(1,501):gate.inspect(cdr(BASE-1_000_000_000),BASE+i*2_000_000,1_000_000_000+i*2_000_000)
        self.assertEqual(gate.accepted,1);self.assertEqual(gate.last_accepted_ns,BASE)
        self.assertFalse(gate.fresh(BASE+1_000_000_000,2_000_000_000))
        self.assertFalse(gate.fresh(BASE-100_000_000,2_000_000_000))

    def test_age_future_receipt_and_gap_fault_before_commit(self):
        for reason,stamp,receipt,steady,now,nowsteady in [
            ('original_header_stale_or_future',BASE+1,BASE+400_000_000,1_400_000_000,None,None),
            ('original_header_stale_or_future',BASE+51_000_001,BASE,1_000_000_000,None,None),
            ('callback_receipt_stale_or_future',BASE+2_000_000,BASE+3_000_000,1_000_000_000,BASE+3_000_000,1_301_000_000),
            ('accepted_header_gap_exceeded',BASE+100_000_001,BASE+100_000_001,1_100_000_001,None,None)]:
            gate=g.ImuMonotonicGate();gate.inspect(cdr(BASE),BASE,1_000_000_000)
            event=gate.inspect(cdr(stamp),receipt,steady,now,nowsteady)
            self.assertFalse(event['accepted']);self.assertTrue(event['fatal']);self.assertEqual(event['reason'],reason)
            self.assertEqual(gate.last_accepted_ns,BASE);self.assertEqual(gate.last_accepted_receipt_steady_ns,1_000_000_000)

    def test_nan_and_changed_frame_fault_keep_original_header(self):
        for raw,reason in [(cdr(BASE+2_000_000,nonfinite=True),'nonfinite_imu'),(cdr(BASE+2_000_000,'wrong'),'changed_imu_frame')]:
            gate=g.ImuMonotonicGate();gate.inspect(cdr(BASE),BASE,1_000_000_000)
            event=gate.inspect(raw,BASE+3_000_000,1_003_000_000)
            self.assertEqual(event['reason'],reason);self.assertTrue(event['fatal'])
            self.assertEqual(event['header_ns'],BASE+2_000_000);self.assertEqual(gate.accepted,1)

    def test_observer_reject_keeps_latest_receipt_then_freshness_expires(self):
        obj=fake_observer();clock={'wall':BASE,'steady':1_000_000_000}
        fake=SimpleNamespace(time_ns=lambda:clock['wall'],monotonic_ns=lambda:clock['steady'])
        with mock.patch.object(observer,'time',fake):
            obj.capture('/front_lidar/imu',Imu,cdr(BASE));before=copy.deepcopy(obj.latest['/front_lidar/imu'])
            clock.update(wall=BASE+200_000_000,steady=1_200_000_000);obj.capture('/front_lidar/imu',Imu,cdr(BASE-1))
        self.assertEqual(obj.latest['/front_lidar/imu'],before);self.assertEqual(obj.errors,[]);self.assertEqual(obj.counts['/front_lidar/imu'],1)
        samples={t:dict(stamp_ns=BASE+301_000_000,received_steady_ns=1_301_000_000,payload_valid=True) for t in observer.c.TOPICS}
        samples['/front_lidar/imu']=before
        with self.assertRaises(observer.c.Hold):observer.c.check_live(samples,None,BASE+301_000_000,1_301_000_000)

    def test_observer_nan_frame_and_reject_IO_failure_are_faults(self):
        for raw in [cdr(BASE,nonfinite=True),cdr(BASE,'wrong')]:
            obj=fake_observer()
            with mock.patch.object(observer,'time',SimpleNamespace(time_ns=lambda:BASE,monotonic_ns=lambda:1_000_000_000)):
                obj.capture('/front_lidar/imu',Imu,raw)
            self.assertTrue(obj.errors);self.assertNotIn('/front_lidar/imu',obj.latest)
        obj=fake_observer();obj.imu_gate.inspect(cdr(BASE),BASE,1_000_000_000)
        obj.imu_reject_stream=SimpleNamespace(write=lambda x:(_ for _ in ()).throw(OSError('fixture disk full')))
        with mock.patch.object(observer,'time',SimpleNamespace(time_ns=lambda:BASE+1,monotonic_ns=lambda:1_000_000_001)):
            obj.capture('/front_lidar/imu',Imu,cdr(BASE))
        self.assertTrue(obj.errors);self.assertNotIn('/front_lidar/imu',obj.latest)

    def test_other_topic_nonadvancing_still_latches_error(self):
        messages=[]
        gt=Odometry();gt.header.frame_id='map';gt.child_frame_id='BASE_LINK';gt.header.stamp.sec=100;gt.pose.pose.position.z=.33;gt.pose.pose.orientation.w=1.
        cloud=PointCloud2();cloud.header.stamp.sec=100;cloud.width=1;cloud.height=1;cloud.row_step=1;cloud.data=[1]
        camera=CompressedImage();camera.header.stamp.sec=100;camera.data=[1]*101
        for topic,typ,message in [('/odom',Odometry,gt),('/front_lidar',PointCloud2,cloud),('/front_camera/image/compressed',CompressedImage,camera)]:
            obj=fake_observer();raw=bytes(serialize_message(message))
            with mock.patch.object(observer,'time',SimpleNamespace(time_ns=lambda:BASE,monotonic_ns=lambda:1_000_000_000)):
                obj.capture(topic,typ,raw);obj.capture(topic,typ,raw)
            self.assertTrue(obj.errors);self.assertIn('nonadvancing original header',obj.errors[0])

    def test_adapter_reject_does_not_observe_clock_and_IO_failure_is_explicit(self):
        tree=ast.parse((HERE/'adapter_with_gate.py').read_text());cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Adapter')
        fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='imu_raw')
        namespace=dict(time=SimpleNamespace(time_ns=lambda:BASE+1,monotonic_ns=lambda:1_000_000_001),json=json,String=type('String',(),{}))
        exec(compile(ast.Module(body=[fn],type_ignores=[]),'imu_raw_fixture','exec'),namespace)
        for broken in [False,True]:
            gate=g.ImuMonotonicGate();gate.inspect(cdr(BASE),BASE,1_000_000_000);published=[]
            def publish(msg):
                if broken:raise OSError('fixture publish failure')
                published.append(json.loads(msg.data))
            obj=SimpleNamespace(imu_gate=gate,imu_reject_pub=SimpleNamespace(publish=publish),imu_audit_fault=None,counts={'errors':0},imu=mock.Mock())
            if broken:
                with self.assertRaises(RuntimeError):namespace['imu_raw'](obj,cdr(BASE),None)
                self.assertEqual(obj.counts['errors'],1)
            else:namespace['imu_raw'](obj,cdr(BASE),None);self.assertEqual(len(published),1)
            obj.imu.assert_not_called();self.assertEqual(gate.last_accepted_receipt_steady_ns,1_000_000_000)

    def test_non_IMU_behavior_and_converter_unchanged(self):
        old=ast.parse((HERE/'baseline/live_sensor_adapter.py').read_text());new=ast.parse((HERE/'adapter_with_gate.py').read_text())
        classes=[next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Adapter') for tree in [old,new]]
        for name in ['observe','lidar','camera','imu']:
            functions=[next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name==name) for cls in classes]
            self.assertEqual(ast.dump(functions[0]),ast.dump(functions[1]))
        self.assertEqual((HERE/'matrix_sensor_contract.py').read_bytes(),(HERE/'baseline/matrix_sensor_contract.py').read_bytes())

    def test_default_HOLD_does_not_import_ROS_implementation(self):
        with mock.patch.object(sys,'argv',['sim_imu_adapter.py']),mock.patch('sys.stdout',new=io.StringIO()),mock.patch.dict(sys.modules,{'adapter_with_gate':None}):
            self.assertEqual(sim_imu_adapter.main(),0)

if __name__=='__main__':unittest.main(verbosity=2)

"""No ROS: virtual 2ms source queue, original hard-gap and shim boundary tests."""
import ast
import base64
import contextlib
import copy
import hashlib
import io
import json
import math
from pathlib import Path
import runpy
import struct
import sys
import tempfile
from types import SimpleNamespace
import unittest

from imu_backlog_gate import BacklogPreservingImuGate,original,validate_imu_backlog_max_age_sec

ROOT=Path(__file__).resolve().parent
START=1_794_026_000_000_000_000
VALUES=tuple([-.0,.001,.002,1.]+[float(i)/100 for i in range(33)])


def raw(stamp,frame='map',values=VALUES,endian='<'):
    frame=frame.encode()+b'\0'
    data=bytearray((b'\0\1\0\0' if endian=='<' else b'\0\0\0\0')+
        struct.pack(endian+'iII',stamp//10**9,stamp%10**9,len(frame))+frame)
    data+=bytes(4+((len(data)-4+7)//8)*8-len(data))
    return bytes(data)+struct.pack(endian+'37d',*values)


def anchored(limit=1.,receipt_offset=1_000_000):
    gate=BacklogPreservingImuGate(imu_backlog_max_age_sec=limit)
    assert gate.inspect(raw(START),START+receipt_offset,1_000_000_000)['accepted']
    return gate


def inspect(gate,gap=2_000_000,age=500_000_000,**kw):
    stamp=START+gap
    return gate.inspect(raw(stamp,**kw),stamp+age,2_000_000_000)


def shim_method(name,namespace):
    tree=ast.parse((ROOT/'adapter_backlog_shim.py').read_text())
    factory=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='make_adapter_class')
    cls=next(n for n in factory.body if isinstance(n,ast.ClassDef))
    method=copy.deepcopy(next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name==name))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[])),
                 str(ROOT/'adapter_backlog_shim.py'),'exec'),namespace)
    return namespace[name]


class GateTests(unittest.TestCase):
    def test_parameter_default_and_upper_bound(self):
        for value in [.3,.5,1.]:validate_imu_backlog_max_age_sec(value)
        for value in [0.,.299999999,1.000000001,math.inf,math.nan,True,'1']:
            with self.subTest(value=value),self.assertRaises(ValueError):validate_imu_backlog_max_age_sec(value)
        self.assertEqual(.3,BacklogPreservingImuGate().imu_backlog_max_age_sec)

    def test_default_old_events_and_state_identical(self):
        old=original.ImuMonotonicGate();new=BacklogPreservingImuGate()
        for data,wall,steady in [(raw(START),START+1,1),
            (raw(START+2_000_000),START+2_000_001,2),
            (raw(START+2_000_000),START+2_000_001,3),
            (raw(START+1_000_000),START+2_000_001,4),
            (raw(START+4_000_000,frame='bad'),START+4_000_001,5),
            (raw(START+4_000_000,values=(math.nan,)+VALUES[1:]),START+4_000_001,6),
            (raw(START+200_000_000),START+200_000_001,7)]:
            self.assertEqual(old.inspect(data,wall,steady),new.inspect(data,wall,steady))
            for name in ['received','accepted','rejected','reasons','last_accepted_ns','last_reject']:
                self.assertEqual(getattr(old,name),getattr(new,name))

    def test_2ms_virtual_queue_catches_up_without_retimestamps_or_true_gaps(self):
        gate=anchored();headers=[];events=[];fresh=[]
        for index in range(1,351):
            stamp=START+index*2_000_000
            wall=START+750_000_000+index*100_000
            steady=1_750_000_000+index*100_000
            data=raw(stamp)
            event=gate.inspect(data,wall,steady)
            self.assertTrue(event['accepted'],event)
            self.assertEqual(stamp,event['header_ns'])
            self.assertEqual(hashlib.sha256(data).hexdigest(),event['cdr_sha256'])
            headers.append(event['header_ns']);events.append(event)
            fresh.append(gate.fresh(wall,steady))
        self.assertTrue(all(b-a==2_000_000 for a,b in zip(headers,headers[1:])))
        self.assertEqual(236,gate.marked_backlog_samples)
        self.assertFalse(fresh[0]);self.assertTrue(fresh[-1])
        self.assertEqual(351,gate.accepted);self.assertEqual(0,gate.rejected)
        diag=gate.diagnostics(wall,steady)
        self.assertEqual('FAIL',diag['strict_input_timing_status'])
        self.assertFalse(diag['strict_input_timing_accepted'])
        self.assertEqual(2_000_000,events[0]['actual_header_gap_ns'])

    def test_original_policy_virtual_queue_drops_stale_then_rejects_anchor_gap(self):
        gate=anchored(.3)
        reasons=[]
        for index in range(1,351):
            stamp=START+index*2_000_000
            event=gate.inspect(raw(stamp),START+750_000_000+index*100_000,1_750_000_000+index*100_000)
            self.assertFalse(event['accepted']);reasons.append(event['reason'])
        self.assertIn('original_header_stale_or_future',reasons)
        self.assertIn('accepted_header_gap_exceeded',reasons)
        self.assertEqual(START,gate.last_accepted_ns)

    def test_true_100ms_plus_one_ns_gap_fatal_for_fresh_and_backlog(self):
        for age in [1_000_000,500_000_000]:
            event=inspect(anchored(),gap=100_000_001,age=age)
            self.assertFalse(event['accepted']);self.assertTrue(event['fatal'])
            self.assertEqual('accepted_header_gap_exceeded',event['reason'])
        self.assertTrue(inspect(anchored(),gap=100_000_000)['accepted'])

    def test_no_756sec_locked_anchor_repair(self):
        gate=anchored()
        self.assertFalse(inspect(gate,gap=756_000_000_000,age=1_000_000)['accepted'])
        self.assertEqual(START,gate.last_accepted_ns)

    def test_backlog_needs_preexisting_accepted_anchor(self):
        gate=BacklogPreservingImuGate(imu_backlog_max_age_sec=1.)
        event=inspect(gate)
        self.assertFalse(event['accepted']);self.assertTrue(event['fatal'])
        self.assertEqual('backlog_without_accepted_anchor',event['reason'])

    def test_age_limits_and_fresh_stays_original_point_three(self):
        for age in [300_000_000,300_000_001,1_000_000_000]:
            gate=anchored();event=inspect(gate,age=age)
            self.assertTrue(event['accepted'])
            self.assertEqual(age>300_000_000,event.get('marked_backlog',False))
            self.assertEqual(age<=300_000_000,gate.fresh(START+2_000_000+age,2_000_000_000))
        self.assertFalse(inspect(anchored(),age=1_000_000_001)['accepted'])
        self.assertTrue(inspect(anchored(.5),age=500_000_000)['accepted'])
        self.assertFalse(inspect(anchored(.5),age=500_000_001)['accepted'])

    def test_future_and_callback_age_unchanged(self):
        self.assertTrue(inspect(anchored(),gap=100_000_000,age=-50_000_000)['accepted'])
        self.assertFalse(inspect(anchored(),gap=100_000_000,age=-50_000_001)['accepted'])
        for delay in [300_000_001,-1]:
            event=anchored().inspect(raw(START+2_000_000),START+502_000_000,2_000_000_000,
                                    START+502_000_000,2_000_000_000+delay)
            self.assertFalse(event['accepted'])
            self.assertEqual('callback_receipt_stale_or_future',event['reason'])

    def test_all37_numeric_finite_frame_and_malformed(self):
        for index in range(37):
            for invalid in [math.nan,math.inf,-math.inf]:
                values=list(VALUES);values[index]=invalid
                event=inspect(anchored(),values=values)
                self.assertFalse(event['accepted']);self.assertEqual('nonfinite_imu',event['reason'])
        self.assertEqual('changed_imu_frame',inspect(anchored(),frame='imu_link')['reason'])
        event=anchored().inspect(b'bad',START+502_000_000,2_000_000_000)
        self.assertFalse(event['accepted']);self.assertTrue(event['fatal'])

    def test_equal_regression_and_receipt_regression(self):
        for gap,reason in [(0,'equal_header'),(-1,'regressing_header')]:
            event=inspect(anchored(),gap=gap)
            self.assertFalse(event['accepted']);self.assertEqual(reason,event['reason'])
        gate=anchored()
        event=gate.inspect(raw(START+2_000_000),START+502_000_000,999_999_999)
        self.assertFalse(event['accepted']);self.assertEqual('receipt_clock_regression',event['reason'])
        gate=anchored(receipt_offset=200_000_000)
        event=gate.inspect(raw(START+2_000_000),START+150_000_000,2_000_000_000)
        self.assertFalse(event['accepted']);self.assertEqual('receipt_clock_regression',event['reason'])

    def test_original_cdr_exact_endian_bits_and_hash(self):
        for endian in ['<','>']:
            data=raw(START+2_000_000,endian=endian)
            event=anchored().inspect(data,START+502_000_000,2_000_000_000)
            self.assertEqual(data,base64.b64decode(event['original_cdr_base64']))
            self.assertEqual(hashlib.sha256(data).hexdigest(),event['cdr_sha256'])
            self.assertEqual(struct.pack('<37d',*VALUES),struct.pack('<37d',*original.decode_imu_cdr(data)['values']))


class ShimTests(unittest.TestCase):
    def test_default_hold_no_ros_imports(self):
        self.assertNotIn('rclpy',sys.modules)
        with contextlib.redirect_stdout(io.StringIO()) as stream:
            runpy.run_path(str(ROOT/'adapter_backlog_shim.py'),run_name='__main__')
        self.assertEqual('ROOT_HOLD',json.loads(stream.getvalue())['state'])
        self.assertNotIn('rclpy',sys.modules)

    def fake(self,stream,published):
        data=raw(START+2_000_000)
        fake=SimpleNamespace(imu_gate=anchored(),backlog_audit=stream,imu_audit_fault=None,counts={'errors':0})
        def publish(message):
            audit=json.loads(Path(stream.name).read_text())
            self.assertEqual(data,base64.b64decode(audit['original_cdr_base64']))
            published.append('backlog')
        fake.imu_backlog_pub=SimpleNamespace(publish=publish)
        fake.imu_reject_pub=SimpleNamespace(publish=lambda msg:published.append('reject'))
        fake.imu=lambda message:published.append(('imu',message))
        namespace=dict(time=SimpleNamespace(time_ns=lambda:START+502_000_000,monotonic_ns=lambda:2_000_000_000),
            os=__import__('os'),json=json,String=SimpleNamespace,Imu=object,deserialize_message=lambda value,kind:value)
        return fake,data,shim_method('imu_raw',namespace)

    def test_audit_flushed_before_forward_original_bytes_reach_converter(self):
        with tempfile.TemporaryDirectory() as directory:
            with (Path(directory)/'audit.jsonl').open('x',buffering=1) as stream:
                published=[];fake,data,method=self.fake(stream,published)
                method(fake,data,None)
                self.assertEqual(['backlog',('imu',data)],published)
                self.assertEqual(0,fake.counts['errors'])

    def test_audit_failure_prevents_forward(self):
        class Broken:
            def write(self,value):raise OSError('disk error')
        with tempfile.TemporaryDirectory() as directory:
            with (Path(directory)/'audit.jsonl').open('x') as stream:
                published=[];fake,data,method=self.fake(stream,published);fake.backlog_audit=Broken()
                with self.assertRaisesRegex(RuntimeError,'audit failed'):method(fake,data,None)
                self.assertEqual([],published);self.assertEqual(1,fake.counts['errors'])

    def test_true_gap_reject_never_reaches_converter(self):
        with tempfile.TemporaryDirectory() as directory:
            with (Path(directory)/'audit.jsonl').open('x') as stream:
                published=[];fake,_,method=self.fake(stream,published)
                method(fake,raw(START+100_000_001),None)
                self.assertEqual(['reject'],published)
                self.assertEqual(1,fake.counts['errors'])

    def test_actual_diagnostics_error_while_backlogged_then_sticky_warn_after_fresh(self):
        class Array:
            def __init__(self):self.header=SimpleNamespace()
        class Status:
            OK,WARN,ERROR=0,1,2
        gate=anchored();inspect(gate)
        published=[]
        clock=SimpleNamespace(wall=START+502_000_000,steady=2_000_000_000)
        fake=SimpleNamespace(imu_gate=gate,imu_audit_fault=None,counts={'errors':0},ages={},conversion_ms=0.,
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:None)),
            diag_pub=SimpleNamespace(publish=published.append))
        namespace=dict(time=SimpleNamespace(time_ns=lambda:clock.wall,monotonic_ns=lambda:clock.steady),
            DiagnosticArray=Array,DiagnosticStatus=Status,KeyValue=lambda **kw:kw)
        method=shim_method('diagnostics',namespace)
        method(fake)
        self.assertEqual(Status.ERROR,published[-1].status[0].level)
        for stamp,wall,steady in [(START+102_000_000,START+502_000_000,2_001_000_000),
                                  (START+202_000_000,START+503_000_000,2_002_000_000),
                                  (START+204_000_000,START+503_000_000,2_003_000_000)]:
            self.assertTrue(gate.inspect(raw(stamp),wall,steady)['accepted'])
            clock.wall,clock.steady=wall,steady
        method(fake)
        self.assertEqual(Status.WARN,published[-1].status[0].level)
        values={item['key']:item['value'] for item in published[-1].status[0].values}
        self.assertEqual('FAIL',values['imu_gate_strict_input_timing_status'])

    def test_conversion_tf_lidar_camera_inherited(self):
        tree=ast.parse((ROOT/'adapter_backlog_shim.py').read_text())
        factory=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='make_adapter_class')
        cls=next(n for n in factory.body if isinstance(n,ast.ClassDef))
        self.assertEqual('base.Adapter',ast.unparse(cls.bases[0]))
        self.assertEqual({'__init__','imu_raw','diagnostics','destroy_node'},
                         {n.name for n in cls.body if isinstance(n,ast.FunctionDef)})
        source=Path(original.__file__).parent/'matrix_sensor_contract.py'
        converter=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='imu_to_local')
        namespace={'copy':copy}
        exec(compile(ast.fix_missing_locations(ast.Module(body=[converter],type_ignores=[])),'matrix_sensor_contract.py','exec'),namespace)
        msg=SimpleNamespace(header=SimpleNamespace(frame_id='map',stamp=SimpleNamespace(sec=1794026000,nanosec=123)),numeric37=VALUES)
        out=namespace['imu_to_local'](msg)
        self.assertEqual('imu_link',out.header.frame_id)
        self.assertEqual(msg.header.stamp.__dict__,out.header.stamp.__dict__)
        self.assertEqual(struct.pack('<37d',*msg.numeric37),struct.pack('<37d',*out.numeric37))
        self.assertEqual('map',msg.header.frame_id)


if __name__=='__main__':unittest.main(verbosity=2)

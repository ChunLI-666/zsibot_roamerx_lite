#!/usr/bin/python3
"""Read actual failed closed bag; no ROS context, Node, mapper or bag rewrite."""
import ast
import copy
import csv
import hashlib
import io
import json
from pathlib import Path
from types import SimpleNamespace

import rosbag2_py
from rclpy.serialization import deserialize_message,serialize_message
from sensor_msgs.msg import Imu

from imu_gate import ImuMonotonicGate,decode_imu_cdr
import calibration_observer as observer_module

HERE=Path(__file__).resolve().parent
INPUT=HERE.parent/'full_scene_lateral_execution_v2/attempt_v1/bag'
OUT=HERE/'actual_replay_v2';OUT.mkdir(exist_ok=False)
EXPECTED='666dfe6484757ea8a13dbf6731d10e075a71832bb32295d94e4af4f911425aa6'
sha=lambda data:hashlib.sha256(data).hexdigest()
assert sha((INPUT/'bag_0.mcap').read_bytes())==EXPECTED
gates={'/front_lidar/imu':ImuMonotonicGate('map'),'/livox/imu':ImuMonotonicGate('imu_link')}
rows={k:[] for k in gates};rejects={k:[] for k in gates};raw_accepted={};derived={}
reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(INPUT),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
obs=object.__new__(observer_module.Observer);obs.latest={};obs.history=[];obs.errors=[];obs.counts={t:0 for t in observer_module.c.TOPICS}
obs.stream=io.StringIO();obs.imu_reject_stream=io.StringIO();obs.imu_gate=ImuMonotonicGate('map');obs.decode=deserialize_message
clock={'wall':0,'steady':0};observer_module.time=SimpleNamespace(time_ns=lambda:clock['wall'],monotonic_ns=lambda:clock['steady'])
first_record=None;observed_reject_snapshot_unchanged=True
while reader.has_next():
    topic,data,record_ns=reader.read_next()
    if first_record is None:first_record=record_ns
    if topic not in gates:continue
    data=bytes(data);receipt_steady=record_ns-first_record+1_000_000_000
    event=gates[topic].inspect(data,int(record_ns),int(receipt_steady));sample=decode_imu_cdr(data)
    row=dict(index=len(rows[topic])+len(rejects[topic]),header_ns=sample['stamp_ns'],record_ns=record_ns,cdr_sha256=sha(data))
    if event['accepted']:
        rows[topic].append(row)
        if topic=='/front_lidar/imu':raw_accepted[sample['stamp_ns']]=data
        else:derived[sample['stamp_ns']]=data
    else:rejects[topic].append(event)
    if topic=='/front_lidar/imu':
        clock.update(wall=int(record_ns),steady=int(receipt_steady));previous=copy.deepcopy(obs.latest.get(topic))
        observer_module.Observer.capture(obs,topic,Imu,data)
        if not event['accepted'] and obs.latest.get(topic)!=previous:observed_reject_snapshot_unchanged=False
del reader
baseline=ast.parse((HERE/'baseline/matrix_sensor_contract.py').read_text())
node=next(n for n in baseline.body if isinstance(n,ast.FunctionDef) and n.name=='imu_to_local');namespace={'copy':copy}
exec(compile(ast.Module(body=[node],type_ignores=[]),'original_imu_to_local','exec'),namespace)
transform_cdr_equal=0;transform_values_equal=0
for stamp,raw in raw_accepted.items():
    expected=namespace['imu_to_local'](deserialize_message(raw,Imu));actual=derived[stamp]
    transform_cdr_equal+=int(bytes(serialize_message(expected))==actual)
    transform_values_equal+=int(decode_imu_cdr(raw)['values']==decode_imu_cdr(actual)['values'])
report={}
for topic,gate in gates.items():
    deltas=[b['header_ns']-a['header_ns'] for a,b in zip(rows[topic],rows[topic][1:])]
    report[topic]=dict(raw_messages=gate.received,accepted=gate.accepted,rejected=gate.rejected,
        reasons=dict(gate.reasons),strict_monotonic=all(d>0 for d in deltas),max_accepted_gap_ns=max(deltas),
        accepted_cdr_and_header_unchanged=True,original_order_preserved=True,rejects=rejects[topic])
    name=topic.strip('/').replace('/','_')
    with (OUT/(name+'_accepted.csv')).open('x') as f:
        w=csv.DictWriter(f,fieldnames=rows[topic][0].keys());w.writeheader();w.writerows(rows[topic])
    with (OUT/(name+'_rejects.json')).open('x') as f:json.dump(rejects[topic],f,indent=2)
assert gates['/front_lidar/imu'].accepted==2447 and gates['/front_lidar/imu'].rejected==1
assert gates['/livox/imu'].accepted==2450 and gates['/livox/imu'].rejected==1
assert all(v['strict_monotonic'] for v in report.values()) and not obs.errors and observed_reject_snapshot_unchanged
result=dict(state='PASS_OFFLINE_REAL_CDR_ONLY',bag_sha256=EXPECTED,topics=report,
    observer=dict(accepted=obs.counts['/front_lidar/imu'],rejected=obs.imu_gate.rejected,errors=obs.errors,
        reject_did_not_update_latest_or_receipt=observed_reject_snapshot_unchanged),
    shared_accepted_raw_headers=len(raw_accepted),baseline_conversion_full_CDR_equal=transform_cdr_equal,
    baseline_conversion_all37_values_equal=transform_values_equal,
    receipt_basis='Offline bag record time is the receipt proxy; runtime captures actual callback wall/steady entry time.',
    no_rclpy_init_Node_ROS_publish_mapper_or_bag_write=True,upstream_timestamp_cause_fixed=False)
assert sha((INPUT/'bag_0.mcap').read_bytes())==EXPECTED
with (OUT/'result.json').open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
print(json.dumps({k:v for k,v in result.items() if k!='topics'}));print({k:(v['accepted'],v['rejected'],v['strict_monotonic']) for k,v in report.items()})

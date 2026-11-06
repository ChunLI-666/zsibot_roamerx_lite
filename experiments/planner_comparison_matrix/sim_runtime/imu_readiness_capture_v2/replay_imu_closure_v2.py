"""Offline bag storage/CDR only. No rclpy.init(), Node, publish, signal or rewrite."""
import collections
import hashlib
import json
import math
from pathlib import Path
import struct
import rosbag2_py
from rclpy.serialization import deserialize_message,serialize_message
from sensor_msgs.msg import Imu
import readiness_contract as c
from readiness_validator import Validator,RAW_IMU,IMU
from imu_gate import decode_imu_cdr,ImuMonotonicGate
from imu_converter import imu_to_local,imu_cdr_evidence

HERE=Path(__file__).resolve().parent
BASE=next(p for p in HERE.parents if p.name=='planner_comparison_20261105')
BAG=BASE/'sim_imu_adapter_readiness_execution_v1/capture_v1/bag'
SHA='566887494f0fd6d93397e251a16edf828383b0926943ee5ea53116514b7d630a'
def digest(raw):return hashlib.sha256(raw).hexdigest()
def body(raw):
    _,_,length=struct.unpack_from('<iII',raw,4);start=4+((16+length-4+7)//8)*8
    return raw[start:start+296]
def main():
    assert c.sha(BAG/'bag_0.mcap')==SHA
    converter=lambda raw:bytes(serialize_message(imu_to_local(deserialize_message(raw,Imu))))
    raws={};derived={};rows=[];counts=collections.Counter();offsets=collections.Counter();by_sha={}
    reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(BAG),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    while reader.has_next():
        topic,data,record_ns=reader.read_next()
        if topic not in (RAW_IMU,IMU):continue
        raw=bytes(data);sample=decode_imu_cdr(raw)
        counts[topic]+=1;(raws if topic==RAW_IMU else derived)[sample['stamp_ns']]=raw;by_sha[digest(raw)]=raw
    del reader
    nonfinite=0;bits_same=0;whole_equal=0
    for ns in sorted(raws.keys()&derived.keys()):
        raw,actual=raws[ns],derived[ns];expected=converter(raw)
        assert decode_imu_cdr(expected)['stamp_ns']==decode_imu_cdr(actual)['stamp_ns']==ns
        nonfinite+=sum(not math.isfinite(value) for value in decode_imu_cdr(raw)['values']+decode_imu_cdr(actual)['values'])
        bits_same+=int(body(raw)==body(actual));whole_equal+=int(expected==actual)
        assert imu_cdr_evidence(expected)['padding_normalized_cdr_sha256']==imu_cdr_evidence(actual)['padding_normalized_cdr_sha256']
        differing=[i for i,(a,b) in enumerate(zip(expected,actual)) if a!=b]
        assert len(expected)==len(actual)==324 and set(differing)<={25,26,27}
        offsets.update(differing)
        rows.append(dict(stamp_ns=ns,raw_sha256=digest(raw),derived_sha256=digest(actual),expected_sha256=digest(expected),expected_padding_hex=expected[25:28].hex(),derived_padding_hex=actual[25:28].hex(),all37_numeric_bits_equal=body(raw)==body(actual)))
    assert counts[RAW_IMU]==2018 and counts[IMU]==1374 and len(rows)==1374 and bits_same==1374 and nonfinite==0
    gates={RAW_IMU:ImuMonotonicGate('map'),IMU:ImuMonotonicGate('imu_link')};callback_counts=collections.Counter();callback_ages={RAW_IMU:[],IMU:[]};all_events=0
    receipt_path=BAG.parent/'sample_receipts.jsonl'
    for line in receipt_path.read_text().splitlines():
        entry=json.loads(line);all_events+=1;topic=entry['topic']
        if topic not in gates:continue
        raw=by_sha[entry['cdr_sha256']];wall,steady=entry['received_unix_ns'],entry['received_steady_ns']
        event=gates[topic].inspect(raw,wall,steady);assert event['accepted']
        callback_counts[topic]+=1;callback_ages[topic].append(wall-decode_imu_cdr(raw)['stamp_ns'])
    assert callback_counts[RAW_IMU]==549 and callback_counts[IMU]==1
    assert c.sha(BAG/'bag_0.mcap')==SHA
    result=dict(state='PASS_OFFLINE_IMU_FIELD_AND_RECORDED_CALLBACK_GATE_REPLAY_ONLY',bag_sha256=SHA,bag_metadata_sha256=c.sha(BAG/'metadata.yaml'),bag_original_failure_preserved=True,
        counts=dict(counts),shared_headers=len(rows),v2_padding_normalized_pairs=len(rows),all37_numeric_bits_equal=bits_same,nonfinite_numeric_fields=nonfinite,
        local_repeated_expected_serialization_full_CDR_equal=whole_equal,expected_vs_actual_diff_offsets=dict(offsets),
        first_failed_live_derived_header_now_field_equal=1793949937680212215 in {row['stamp_ns'] for row in rows},
        recorded_callback_events=all_events,recorded_callback_receipt_sha256=c.sha(receipt_path),callback_IMU_counts=dict(callback_counts),
        callback_age_ns={topic:dict(min=min(values),max=max(values)) for topic,values in callback_ages.items()},
        raw_gate=dict(accepted=gates[RAW_IMU].accepted,rejected=gates[RAW_IMU].rejected),derived_gate=dict(accepted=gates[IMU].accepted,rejected=gates[IMU].rejected),
        receipt_basis='Only original sample_receipts wall/steady values used for gate replay. Bag recordTime is not substituted for callback receipt; complete1374 pairing is field proof only.',
        original_live_state='HOLD_READINESS_FAILED',new_live_readiness_result=False,no_ROS_context_Node_or_runtime_action=True)
    (HERE/'actual_replay_pairs_v2.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows))
    (HERE/'actual_replay_result_v2.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    return 0
if __name__=='__main__':raise SystemExit(main())

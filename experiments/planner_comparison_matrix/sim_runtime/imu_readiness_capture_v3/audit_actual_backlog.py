"""Read the closed v2 bag/receipts; logTime is not a callback receipt."""
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import statistics
import struct
import rosbag2_py

HERE=Path(__file__).resolve().parent
EXECUTION=HERE.parent/'sim_imu_adapter_readiness_execution_v2'
CAPTURE=EXECUTION/'capture_v1'
EXPECTED='f4976b6f302af80c0bf817c819f621d4fe50fd09cf61f9f05203e05d02cce03e'
TOPICS=['/front_lidar/imu','/livox/imu','/odom','/front_camera/image/compressed']
sha=lambda data:hashlib.sha256(data).hexdigest()
def stats(values):
    values=sorted(values)
    return dict(n=len(values),median_ms=statistics.median(values)/1e6,p99_ms=values[int((len(values)-1)*.99)]/1e6,max_ms=max(values)/1e6) if values else dict(n=0)
def main(output):
    bag=CAPTURE/'bag';path=next(bag.glob('*.mcap'));assert sha(path.read_bytes())==EXPECTED
    original_result=(CAPTURE/'readiness_report.json').read_bytes();receipts=(CAPTURE/'sample_receipts.jsonl').read_bytes()
    rows={t:[] for t in TOPICS};by_header={t:{} for t in TOPICS}
    reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    while reader.has_next():
        topic,raw,log_time=reader.read_next()
        if topic not in rows:continue
        raw=bytes(raw);assert raw[:4]==b'\0\1\0\0'
        sec,nsec=struct.unpack_from('<iI',raw,4);header=sec*10**9+nsec
        rows[topic].append((header,log_time));by_header[topic][header]=dict(log_time=log_time,sha256=sha(raw))
    del reader
    observed={t:[] for t in TOPICS};excess={t:[] for t in TOPICS}
    for line in receipts.splitlines():
        row=json.loads(line);topic=row['topic']
        if topic not in observed or 'stamp_ns' not in row:continue
        observed[topic].append((row['stamp_ns'],row['received_unix_ns']))
        match=by_header[topic].get(row['stamp_ns'])
        if match:
            assert row['cdr_sha256']==match['sha256']
            excess[topic].append(row['received_unix_ns']-match['log_time'])
    report={}
    for topic,samples in rows.items():
        first_log=samples[0][1];log_bins=defaultdict(list)
        for header,log in samples:log_bins[int((log-first_log)//10**9)].append(log-header)
        callback_bins=defaultdict(list)
        first_callback=observed[topic][0][1]
        for header,wall in observed[topic]:callback_bins[int((wall-first_callback)//10**9)].append(wall-header)
        report[topic]=dict(bag_count=len(samples),header_span_s=(samples[-1][0]-samples[0][0])/1e9,
            header_rate_hz=(len(samples)-1)*1e9/(samples[-1][0]-samples[0][0]),
            bag_log_header_age_by_record_second={k:stats(v) for k,v in log_bins.items()},
            bag_after_first_record_second=stats([log-header for header,log in samples if log-first_log>=10**9]),
            callback_count=len(observed[topic]),callback_header_age_by_callback_second={k:stats(v) for k,v in callback_bins.items()},
            callback_header_age=stats([wall-header for header,wall in observed[topic]]),
            callback_minus_same_header_bag_log_time=stats(excess[topic]))
    assert len(rows['/front_lidar/imu'])==1432 and len(rows['/livox/imu'])==2560
    assert report['/front_lidar/imu']['bag_after_first_record_second']['max_ms']<3.
    assert report['/front_lidar/imu']['callback_header_age']['max_ms']>296.
    # Keep the genuine derived startup/discovery record-age burst. No global
    # "all derived data <.3s" conclusion is drawn from later fresh seconds.
    assert any(row['max_ms']>300. for row in report['/livox/imu']['bag_log_header_age_by_record_second'].values())
    assert sha(path.read_bytes())==EXPECTED and (CAPTURE/'readiness_report.json').read_bytes()==original_result
    result=dict(state='OFFLINE_CLOSED_FAILURE_FACTS_ONLY',bag_sha256=EXPECTED,sample_receipts_sha256=sha(receipts),original_result_sha256=sha(original_result),
        topics=report,original_state=json.loads(original_result)['state'],motion_authorized=False,
        interpretation='Recorder observes near-source IMU times while observer IMU callbacks accumulate delay; dispatch/outer-check backlog is supported. Derived record-age startup burst is also preserved. Scheduling v3 still requires a fresh runtime capture.',
        time_basis='MCAP logTime used only for recorder/header age and same-header comparison. Actual recorded callback wall receipt used for observer/header age; neither substitutes for the other.',
        source_or_bag_modified=False,no_ROS_context_Node_Popen_signal_UDP_remote=True)
    output.write_text(json.dumps(result,indent=2))
    return result
if __name__=='__main__':
    result=main(HERE/'actual_backlog_audit_v3.json')
    print(json.dumps({k:v for k,v in result.items() if k!='topics'}))

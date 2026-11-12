#!/usr/bin/env python3
"""Sample owned process CPU/RSS and publish the samples into this run's MCAP."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import time


def sample(pid):
    try:
        p=Path('/proc')/str(pid);s=(p/'stat').read_text().rsplit(')',1)[1].split()
        if s[0] in ('Z','X'):return None
        return dict(pid=pid,birth=int(s[19]),ticks=int(s[11])+int(s[12]),rss_bytes=int(s[21])*os.sysconf('SC_PAGE_SIZE'),threads=int(s[17]))
    except (OSError,ValueError,IndexError):return None


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--nav-dir',type=Path,required=True)
    p.add_argument('--source-manifest',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--run-id',required=True)
    a=p.parse_args();manifest=json.loads((a.nav_dir/'runtime_manifest.json').read_text());deadline=manifest['recording_deadline']
    if deadline['run_id']!=a.run_id:raise ValueError('recording run ID differs')
    launcher=deadline['launcher']; first=sample(launcher['pid'])
    if not first or first['birth']!=launcher['starttime_ticks']:raise ValueError('launcher birth differs')
    processes=json.loads((a.nav_dir/'processes.json').read_text());processes['launcher']=launcher['pid']
    from workflow_inputs import checked_json,process_check
    source=json.loads(a.source_manifest.read_text())
    for role,spec in source['roles'].items():
        actor=checked_json(spec['started']);identity=process_check(actor['child'])
        processes['source_'+role]=identity['pid']
    for index,identity in enumerate(source.get('retained_processes',[])):
        processes['source_retained_'+str(index)]=process_check(identity)['pid']
    originals={role:sample(pid) for role,pid in processes.items()};a.output.mkdir(parents=True,exist_ok=False)
    (a.output/'identity.json').write_text(json.dumps(dict(run_id=a.run_id,identities=originals,deadline_boottime_sec=deadline['deadline_boottime_sec']),indent=2)+'\n')
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    stop=[];handlers={n:signal.signal(n,lambda number,frame:stop.append(number)) for n in (signal.SIGINT,signal.SIGTERM)}
    rclpy.init();node=Node('terra_runtime_performance_'+a.run_id[:8],enable_rosout=False,start_parameter_services=False)
    topic='/experiment/runtime_performance';publisher=node.create_publisher(String,topic,10)
    counts=0;previous={};last_time=time.monotonic();start=last_time;reason='deadline';ready=False
    try:
        with (a.output/'samples.jsonl').open('x',buffering=1) as stream:
            while not stop and time.clock_gettime(time.CLOCK_BOOTTIME)<deadline['deadline_boottime_sec']:
                live=sample(launcher['pid'])
                if not live or live['birth']!=launcher['starttime_ticks']:
                    reason='launcher_closed';break
                rclpy.spin_once(node,timeout_sec=.1);now=time.monotonic()
                if now-last_time<1:continue
                elapsed=now-last_time;rows={}
                for role,pid in processes.items():
                    current=sample(pid);initial=originals[role];prior=previous.get(role)
                    if not current or not initial or current['birth']!=initial['birth']:
                        rows[role]=None;continue
                    current['cpu_cores']=(current['ticks']-prior['ticks'])/os.sysconf('SC_CLK_TCK')/elapsed if prior else None
                    rows[role]=current;previous[role]=current
                row=dict(schema='terra.runtime_performance.sample.v1',run_id=a.run_id,unix_ns=time.time_ns(),
                         monotonic_ns=time.monotonic_ns(),processes=rows,boundary='process CPU/RSS, not per-callback duration')
                raw=json.dumps(row,separators=(',',':'),allow_nan=False);stream.write(raw+'\n');publisher.publish(String(data=raw));counts+=1;last_time=now
                subscribers=[x.node_name for x in node.get_subscriptions_info_by_topic(topic)]
                if not ready and counts>=2 and any(x.startswith('matrix_recorder_') for x in subscribers):
                    info=dict(ready=True,run_id=a.run_id,time_ns=time.time_ns(),topic=topic,subscribers=subscribers,source_samples=counts)
                    (a.output/'ready.json').write_text(json.dumps(info,indent=2)+'\n');ready=True
            if stop:reason='signal'
    finally:
        node.destroy_node();rclpy.shutdown()
        for n,handler in handlers.items():signal.signal(n,handler)
        result=dict(run_id=a.run_id,reason=reason,samples=counts,ready=ready,elapsed_sec=time.monotonic()-start,
                    sample_file_sha256=hashlib.sha256((a.output/'samples.jsonl').read_bytes()).hexdigest(),completed_ns=time.time_ns())
        (a.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result))


if __name__=='__main__':main()

#!/usr/bin/env python3
"""Analyze a newly closed MCAP using native pose/TF only; never consumes GT."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import time

import mission_evidence


def quant(values):
    values=sorted(values)
    if not values:return None
    return {str(q):values[round((len(values)-1)*q)] for q in (0,.5,.95,.99,1)}


def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def wrapped(x):return math.atan2(math.sin(x),math.cos(x))
def stamp(header):return header.stamp.sec*1_000_000_000+header.stamp.nanosec


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run-dir',type=Path,required=True);a=p.parse_args();run=a.run_dir
    audit=json.loads((run/'mcap_audit.json').read_text())
    if not audit['read_to_eof'] or not audit['integrity_pass']:raise ValueError('closed audited MCAP required')
    wf=json.loads((run/'workflow_state.json').read_text());_,_,ref,state,_=mission_evidence.latest(wf)
    import cv2
    import numpy as np
    import yaml
    params=json.loads((run/'nav/runtime_parameters.json').read_text())
    global_costmap=params['global_costmap/global_costmap'];raw=json.loads(global_costmap['footprint']['string_value']);pad=global_costmap['footprint_padding']['double_value']
    footprint=[[x+math.copysign(pad,x) if x else 0.,y+math.copysign(pad,y) if y else 0.] for x,y in raw]
    manifest=json.loads((run/'nav/runtime_manifest.json').read_text());map_path=Path(manifest['map_yaml']);map_data=yaml.safe_load(map_path.read_text())
    image=cv2.imread(str(map_path.parent/map_data['image']),cv2.IMREAD_GRAYSCALE)
    if image is None or abs(map_data['origin'][2])>1e-9:raise ValueError('unsupported or unreadable map raster')
    h,w=image.shape;resolution=float(map_data['resolution']);ox,oy=map_data['origin'][:2]
    occ_thresh=float(map_data['occupied_thresh']);free_thresh=float(map_data['free_thresh']);negate=bool(map_data['negate'])
    def footprint_check(x,y,heading):
        cs,sn=math.cos(heading),math.sin(heading)
        points=[]
        for fx,fy in footprint:
            mx=x+cs*fx-sn*fy;my=y+sn*fx+cs*fy
            points.append((math.floor((mx-ox)/resolution),h-1-math.floor((my-oy)/resolution)))
        xs=[v[0] for v in points];ys=[v[1] for v in points]
        if min(xs)<0 or min(ys)<0 or max(xs)>=w or max(ys)>=h:return 'outside_map'
        x0,x1=min(xs),max(xs);y0,y1=min(ys),max(ys)
        mask=np.zeros((y1-y0+1,x1-x0+1),dtype=np.uint8)
        poly=np.array([[[px-x0,py-y0] for px,py in points]],dtype=np.int32);cv2.fillPoly(mask,poly,1)
        pixels=image[y0:y1+1,x0:x1+1][mask.astype(bool)].astype(np.float32)/255.
        occupied=pixels if negate else 1.-pixels
        if np.any(occupied>occ_thresh):return 'occupied'
        if np.any(occupied>=free_thresh):return 'unknown'
        return 'free'
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(run/'nav/bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    declarations={x.name:x.type for x in reader.get_all_topics_and_types()}
    topics=['/lightning/pose','/odom/current_pose','/laser_scan_leveled_nav_floor_v1','/tf',
            '/lightning/loc_status','/lightning/pose_valid','/experiment/recovery/events']
    available=[t for t in topics if t in declarations];types={t:get_message(declarations[t]) for t in available}
    reader.set_filter(rosbag2_py.StorageFilter(topics=available))
    scan_ages=[];odom_ages=[];tf_ages={'map->odom':[],'odom->base_link':[]};status=Counter();valid=Counter();handoff=[]
    poses=[];footprints=Counter();examples={};last_pose_log=None
    while reader.has_next():
        topic,raw_message,log_ns=reader.read_next();msg=deserialize_message(raw_message,types[topic])
        if topic=='/laser_scan_leveled_nav_floor_v1':scan_ages.append((log_ns-stamp(msg.header))/1e9)
        elif topic=='/odom/current_pose':
            if len(odom_ages)==0 or log_ns-odom_ages[-1][0]>=100_000_000:odom_ages.append((log_ns,(log_ns-stamp(msg.header))/1e9))
        elif topic=='/lightning/loc_status':status[str(msg.data)]+=1
        elif topic=='/lightning/pose_valid':valid[str(msg.data)]+=1
        elif topic=='/tf':
            for t in msg.transforms:
                key=t.header.frame_id+'->'+t.child_frame_id
                if key in tf_ages and (not tf_ages[key] or log_ns-tf_ages[key][-1][0]>=100_000_000):
                    tf_ages[key].append((log_ns,(log_ns-stamp(t.header))/1e9))
        elif topic=='/experiment/recovery/events':
            try:handoff.append(dict(log_ns=log_ns,event=json.loads(msg.data)))
            except (ValueError,TypeError):handoff.append(dict(log_ns=log_ns,invalid_event=msg.data[:200]))
        elif topic=='/lightning/pose' and (last_pose_log is None or log_ns-last_pose_log>=200_000_000):
            last_pose_log=log_ns
            if msg.header.frame_id!='map':continue
            x,y,heading=msg.pose.position.x,msg.pose.position.y,yaw(msg.pose.orientation)
            if not all(math.isfinite(v) for v in (x,y,heading)):continue
            row=dict(log_ns=log_ns,header_ns=stamp(msg.header),x=x,y=y,yaw=heading);poses.append(row)
            kind=footprint_check(x,y,heading);footprints[kind]+=1
            if kind!='free' and kind not in examples:examples[kind]=row
    turns=[];receipts={x['id']:x for x in state['progress']['accepted_receipts']}
    for waypoint_id in ('F009','F010','F011'):
        receipt=receipts.get(waypoint_id)
        if receipt is None:
            turns.append(dict(id=waypoint_id,pose_samples=0,signed_yaw_rad=None,direction='NOT_ACCEPTED_OR_NOT_VISITED'))
            continue
        visit=receipt['visit'];entered=visit.get('entered') or {};passed=visit.get('passed') or {}
        begin,end=entered.get('unix_ns'),passed.get('unix_ns')
        window=[r for r in poses if begin is not None and end is not None and begin<=r['log_ns']<=end]
        angle=sum(wrapped(b['yaw']-a['yaw']) for a,b in zip(window,window[1:])) if len(window)>1 else None
        source='new_closed_mcap_native_pose_samples' if angle is not None else None
        historical_angle=(visit.get('motion') or {}).get('signed_yaw_rad')
        if angle is None and isinstance(historical_angle,(int,float)) and not isinstance(historical_angle,bool) and math.isfinite(historical_angle):
            angle=float(historical_angle);source='client_hash_bound_visit_motion'
        turns.append(dict(id=receipt['id'],legacy_stop_proof=receipt.get('legacy_stop_proof',False),
            entered_ns=begin,passed_ns=end,pose_samples=len(window),signed_yaw_rad=angle,evidence_source=source,
            direction=('left' if angle>0 else 'right' if angle<0 else 'none') if angle is not None else 'UNKNOWN_OUTSIDE_OR_MISSING_RECORDING'))
    handoff_issues=[];last_seq=last_zero=0;released={}
    for record in handoff:
        event=record.get('event')
        if not isinstance(event,dict):handoff_issues.append('invalid_event');continue
        seq=event.get('last_output_sequence');zero=event.get('last_recovery_zero_sequence');epoch=event.get('epoch')
        if not isinstance(seq,int) or not isinstance(zero,int) or zero>seq or seq<last_seq or zero<last_zero:handoff_issues.append('nonmonotonic_or_invalid_sequence')
        if epoch in released and event.get('mode')=='CONTROLLER' and zero!=released[epoch]:handoff_issues.append('recovery_zero_sequence_changed_after_release')
        if event.get('operation')=='RELEASE' and event.get('accepted') is True:released[epoch]=zero
        if isinstance(seq,int):last_seq=seq
        if isinstance(zero,int):last_zero=zero
    age_stats=lambda rows:dict(samples=len(rows),quantiles_sec=quant(rows),future_beyond_50ms=sum(x<-.05 for x in rows),
                              over_300ms=sum(x>.3 for x in rows),over_500ms=sum(x>.5 for x in rows))
    metrics=dict(schema='terra.new_mcap.metrics.v1',run_id=manifest['recording_deadline']['run_id'],
        scan_header_age_vs_bag_log=age_stats(scan_ages),odom_header_age_vs_bag_log=age_stats([x[1] for x in odom_ages]),
        tf_header_age_vs_bag_log={k:age_stats([x[1] for x in v]) for k,v in tf_ages.items()},
        loc_status_counts=dict(status),pose_valid_counts=dict(valid),sampled_native_pose_count=len(poses),
        sampled_static_map_footprint=dict(samples=dict(footprints),examples=examples,map_resolution_m=resolution,raw_footprint=raw,padding_m=pad,
            boundary='Sampled at most 5Hz against static 2D raster only; not a continuous swept, dynamic-obstacle, or physical collision proof.'),
        original_turns=turns,handoff_event_count=len(handoff),handoff_event_issues=handoff_issues,
        handoff_boundary='Sequence checks cover recorded service events only; DDS writer/receive ordering and every output sample require separate proof.',
        metric_window='Complete new closed bag; message headers and bag-log times retained separately; no historical MCAP or GT read.',checked_ns=time.time_ns())
    with (run/'metrics.json').open('x') as f:json.dump(metrics,f,indent=2,ensure_ascii=False,allow_nan=False);f.write('\n')
    print(json.dumps(dict(output=str(run/'metrics.json'),poses=len(poses),scan_samples=len(scan_ages),footprint_samples=dict(footprints),turns=turns)))


if __name__=='__main__':main()

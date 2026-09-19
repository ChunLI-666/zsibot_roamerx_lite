#!/usr/bin/env python3
"""Freeze nine 0829 reverse snapshots and reconstruct only prior published costmaps."""
import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import yaml

WINDOWS=[('reverse_10s',1788003737.6339045,1788003747.6839075),
         ('reverse_14s',1788003822.9337518,1788003836.783998),
         ('reverse_20s',1788004164.6340435,1788004184.6840708)]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();ws=args.workspace.resolve();out=args.output.resolve();out.mkdir(exist_ok=False)
    repo=ws/'src/zsibot/zsibot_roamerx_lite'
    extracted=ws/'docs/progress/2026-09-08_relocalization_and_reverse_design/rosbag2_2026_08_29-04_41_05_extracted.json'
    data=json.loads(extracted.read_text());plans=sorted(data['/transformed_global_plan'],key=lambda r:r['t'])
    times=[r['t'] for r in plans];snapshots=[]
    for label,start,end in WINDOWS:
        rows=[r for r in data['/controller_server/debug'] if start<=r['t']<=end]
        for part,row in [('start',rows[0]),('middle',rows[len(rows)//2]),('end',rows[-1])]:
            index=bisect.bisect_right(times,row['t'])-1
            if index<0:raise ValueError('No preceding reference')
            plan=plans[index];age=row['t']-plan['t']
            if not 0<=age<.15 or not row['frame']==row['end_frame']==plan['frame']=='odom':
                raise ValueError('Unmatched pose/goal/path frames or times')
            distance=math.hypot(row['end_pose'][0]-row['x'],row['end_pose'][1]-row['y'])
            if abs(distance-row['distance'])>1e-6:raise ValueError('Goal distance inconsistent')
            snapshots.append(dict(name=label+'_'+part,window=label,receipt_time=row['t'],
                receipt_ns=round(row['t']*1e9),source_stamp=row['stamp'],
                pose=[row['x'],row['y'],row['yaw']],velocity=row['current_velocity'],
                recorded_command=[row['vx'],row['vy'],row['wz']],goal=row['end_pose'],
                full_goal_distance=distance,path=plan['poses'],path_age_sec=age,
                path_source_stamp=plan['stamp'],path_update_count=row['path_update_count']))
    bag=Path('/home/charles/datasets/rock_dog/20260829/rosbag2_2026_08_29-04_41_05')
    command=[sys.executable,str(Path(__file__).parents[1]/'reconstruct_recorded_costmap.py'),
             '--bag',str(bag),'--output',str(out/'costmaps')]
    for case in snapshots:command+=['--snapshot',case['name']+':'+str(case['receipt_ns'])]
    with (out/'costmap_reconstruction.log').open('w') as log:
        subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
    raw=subprocess.check_output(['git','-C',str(repo),'show','6942974:src/navigation/src/robot_navigo/params/navigo_params.yaml'])
    (out/'source_params.yaml').write_bytes(raw);base=yaml.safe_load(raw)
    cp=base['controller_server']['ros__parameters'];cp['use_sim_time']=True
    cp['general_goal_checker']=dict(plugin='navigo_path_controller::StoppedGoalChecker',stateful=False,
        xy_goal_tolerance=.25,yaw_goal_tolerance=.25,trans_stopped_velocity=.01,rot_stopped_velocity=.01)
    cp['FollowPath']['visualize']=False;cp['FollowPath']['regenerate_noises']=False
    cp['FollowPath']['forward_alignment']={'enabled':False}
    for weight in (0,5,20):
        cp['FollowPath']['PreferForwardCritic']['cost_weight']=float(weight)
        (out/f'params_weight{weight}.yaml').write_text(yaml.safe_dump({'controller_server':{'ros__parameters':cp}}))
    footprint=base['local_costmap']['local_costmap']['ros__parameters']['footprint']
    for case in snapshots:
        context=json.loads((out/'costmaps'/f"{case['name']}.json").read_text())
        if context['frame'] != 'odom':
            raise ValueError('Reconstructed costmap frame must match odom inputs')
        case.update(map=context['map'],costmap_age_sec=context['last_receipt_age_sec'],
                    footprint=footprint,footprint_padding=.01,load_inflation_layer=True,
                    inflation_radius=base['local_costmap']['local_costmap']['ros__parameters']['inflation_layer']['inflation_radius'],
                    cost_scaling_factor=base['local_costmap']['local_costmap']['ros__parameters']['inflation_layer']['cost_scaling_factor'])
    protocol=dict(snapshots=snapshots,seeds=[42,43,44],warm_starts=['zero','measured'],weights=[0,5,20],
        extracted_sha256=hashlib.sha256(extracted.read_bytes()).hexdigest(),source_params_sha256=hashlib.sha256(raw).hexdigest(),
        source_params_commit='6942974',costmap_reconstruction_command=command,
        scope='Current source legacy mode re-evaluation; not historical sample-cost recovery or closed-loop validation',
        limits=['Historical nominal sequence and noise state unavailable; zero/measured warm starts are explicit assumptions',
                'Published occupancy costs are quantized; reconstructed maximum preimage is conservative',
                'Prior received map is not guaranteed to be the exact in-process map at compute time',
                'Full installed goal from debug may use a different TF time from optimizer goal',
                'Repository config and footprint are not a verified historical live parameter dump',
                'Source receipts reconstructed from JSON float timestamps have sub-microsecond rounding uncertainty',
                'Fixed proposal batches change furthest path index; compare costs within each batch, not across different candidate sets'])
    (out/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    print(f'Prepared {len(snapshots)} snapshots')


if __name__=='__main__':main()

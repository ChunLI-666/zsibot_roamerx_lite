#!/usr/bin/env python3
"""Summarize separate reachable, shadow, and intentional-hold experiments."""
import argparse
import csv
import json
from pathlib import Path
import statistics
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def kind(name, row=None):
    if row and row.get("expected_behavior"):
        return row["expected_behavior"]
    if 'shadow' in name:
        return 'shadow'
    if 'blocked_rotation' in name or 'tiny_limit' in name:
        return 'intentional_hold'
    return 'reachable_feedback'


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--root',required=True,type=Path)
    args=parser.parse_args();root=args.root.resolve()
    groups={name:json.loads((root/name/'summary.json').read_text()) for name in ('baseline','feature_disabled','candidate')}
    expected={path.stem for path in (root/'cases').glob('*_seed*.yaml')}
    for group, rows in groups.items():
        if set(rows) != expected:
            raise SystemExit(f'{group}: incomplete case matrix; wait for runner completion')
        if any(row.get('error') for row in rows.values()):
            raise SystemExit(f'{group}: runner errors; matrix is invalid, inspect summary.json')
    manifest=json.loads((root/'cases/manifest.json').read_text())
    expected_hash=manifest['footprint']['source_sha256']
    for group, rows in groups.items():
        if not all(row.get('footprint_verified') and row['footprint']['source_sha256']==expected_hash for row in rows.values()):
            raise SystemExit(f'{group}: footprint provenance mismatch')
    result={'footprint':manifest['footprint']}
    for name,rows in groups.items():
        result[name]={}
        for category in ('reachable_feedback','shadow','intentional_hold','blocked_geometry','invalid_initial_state','recorded_context','clearance_limited'):
            picked={k:v for k,v in rows.items() if kind(k,v)==category}
            result[name][category]=dict(trials=len(picked),successful_endpoints=sum(v['success'] for v in picked.values()),
                negative_raw_vx_samples=sum(v['negative_raw_vx_samples'] for v in picked.values()),
                samples=sum(v['samples'] for v in picked.values()),collisions=sum(v['collisions'] for v in picked.values()),
                exceptions=sum(v['exceptions'] for v in picked.values()),sub_deadband_yaw_samples=sum(v['sub_deadband_yaw_samples'] for v in picked.values()))
    cases=sorted({name.rsplit('_seed',1)[0] for name in groups['candidate']})
    result['cases']={}
    for case in cases:
        result['cases'][case]={}
        for group,allrows in groups.items():
            rows=[v for k,v in allrows.items() if k.rsplit('_seed',1)[0]==case]
            result['cases'][case][group]=dict(trials=len(rows),successes=sum(v['success'] for v in rows),
                median_duration_sim_sec=statistics.median(v['duration_sim_sec'] for v in rows),
                reverse_command_distance_m=sum(v['reverse_command_distance_m'] for v in rows),
                max_abs_vx_after_reset=max(v['max_abs_vx_after_reset'] for v in rows),
                max_abs_wz_after_reset=max(v['max_abs_wz_after_reset'] for v in rows),
                max_final_xy_error_m=max(v['final_xy_error_m'] for v in rows),
                max_final_yaw_error_rad=max(v['final_yaw_error_rad'] for v in rows))
    # Absolute-speed tests are allowed to hold below the executable threshold;
    # report completion and bounds separately rather than equating HOLD to failure.
    (root/'comparison.json').write_text(json.dumps(result,indent=2))
    fig,axes=plt.subplots(2,2,figsize=(12,8))
    for column,case in enumerate(('warehouse_heading_180_seed42','0829_reverse_20s_seed42')):
        for group,color in [('feature_disabled','#bf5936'),('candidate','#2472a4')]:
            rows=list(csv.DictReader((root/group/(case+'.csv')).open()))
            time=[float(r['t']) for r in rows]
            axes[0,column].plot([float(r['x']) for r in rows],[float(r['y']) for r in rows],label=group,color=color)
            axes[1,column].plot(time,[float(r['raw_vx']) for r in rows],label=group,color=color)
        case_data=yaml.safe_load((root/'cases'/(case+'.yaml')).read_text())
        px=[p[0] for p in case_data['path']];py=[p[1] for p in case_data['path']]
        axes[0,column].plot(px,py,'--',color='gray',alpha=.6,label='input path')
        axes[0,column].scatter([px[-1]],[py[-1]],marker='*',s=65,color='black')
        half=max(max(px)-min(px),max(py)-min(py),.5)*.65
        cx=(min(px)+max(px))/2;cy=(min(py)+max(py))/2
        axes[0,column].set(title=case.replace('_seed42',''),xlabel='x [m]',ylabel='y [m]',aspect='equal',
                           xlim=(cx-half,cx+half),ylim=(cy-half,cy+half))
        axes[1,column].set(xlabel='simulated time [s]',ylabel='raw controller vx [m/s]')
        axes[1,column].axhline(0,color='black',linewidth=.5)
        for row in range(2):axes[row,column].grid(alpha=.2);axes[row,column].legend()
    fig.suptitle('Production C++ controller / ideal SE(2) feedback\n0829: recorded initial local-path geometry; no historical costmap reconstruction')
    fig.tight_layout();fig.savefig(root/'feedback_comparison.png',dpi=160);plt.close(fig)
    # Verify all final candidate commands, including during mode changes/resets.
    checks={}
    for case in root.glob('candidate/*.csv'):
        rows=list(csv.DictReader(case.open()))
        expected_behavior=groups['candidate'][case.stem]['expected_behavior']
        checks[case.stem]=dict(nonnegative_vx=all(float(r['raw_vx'])>=-1e-6 for r in rows),
            zero_lateral=all(abs(float(r['raw_vy']))<=1e-6 for r in rows),
            executable_yaw=all(abs(float(r['raw_wz']))<=1e-6 or abs(float(r['raw_wz']))>=.02-1e-6 for r in rows),
            no_collision=all(r['collision']=='0' for r in rows) if expected_behavior!='invalid_initial_state' and not groups['candidate'][case.stem]['footprint'].get('initial_pose_collision') else
                len(rows)==1 and all(abs(float(rows[0][axis]))<=1e-6 for axis in ('raw_vx','raw_vy','raw_wz')),
            footprint_verified=groups['candidate'][case.stem]['footprint_verified'])
        if expected_behavior == 'reachable_feedback':
            checks[case.stem]['endpoint_success']=groups['candidate'][case.stem]['success']
        elif expected_behavior in ('blocked_geometry', 'invalid_initial_state'):
            checks[case.stem]['not_false_success']=not groups['candidate'][case.stem]['success']
        if 'speed_limit' in case.stem or 'absolute_limit' in case.stem:
            post=[r for r in rows if int(r['step'])>=30]
            vx,wz=(.05,.2*.05/.3) if 'absolute_limit' in case.stem else (.075,.05)
            checks[case.stem]['limit_after_reset']=all(abs(float(r['raw_vx']))<=vx+1e-6 and abs(float(r['raw_wz']))<=wz+1e-6 for r in post)
    (root/'acceptance_checks.json').write_text(json.dumps(checks,indent=2))
    if not all(all(row.values()) for row in checks.values()):
        raise SystemExit('Candidate command or collision regression failed; inspect acceptance_checks.json')
    print(json.dumps({k:v for k,v in result.items() if k!='cases'},indent=2))

if __name__=='__main__':
    main()

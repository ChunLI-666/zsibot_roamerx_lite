#!/usr/bin/env python3
"""Run production-plugin feedback/shadow cases in subprocesses; no hardware IO."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import yaml


def summarize(path, returncode):
    if not path.exists():
        return {'returncode': returncode, 'success': False, 'error': 'No CSV produced'}
    with path.open() as file:
        rows = list(csv.DictReader(file))
    if not rows:
        return {'returncode': returncode, 'success': False, 'error': 'Empty CSV'}
    dt = float(rows[1]['t'])-float(rows[0]['t']) if len(rows)>1 else .1
    negative = [r for r in rows if float(r['raw_vx']) < -1e-6]
    dead = [r for r in rows if 1e-6 < abs(float(r['raw_wz'])) < .02-1e-6]
    after_reset = [r for r in rows if int(r['step']) >= 30]
    return dict(returncode=returncode, samples=len(rows), duration_sim_sec=float(rows[-1]['t']),
        success=returncode == 0 and rows[-1]['success']=='1' and not any(r['collision']=='1' for r in rows), reached_any=any(r['success']=='1' for r in rows), collisions=sum(r['collision']=='1' for r in rows),
        exceptions=sum(bool(r['exception']) for r in rows),
        collision_reasons=sorted({r.get('collision_reason','unspecified') for r in rows if r['collision']=='1'}),
        exception_examples=sorted({r['exception'] for r in rows if r['exception']})[:5],
        negative_raw_vx_samples=len(negative), reverse_command_distance_m=sum(max(0,-float(r['vx']))*dt for r in rows),
        sub_deadband_yaw_samples=len(dead), final_xy_error_m=float(rows[-1]['xy_error']),final_yaw_error_rad=float(rows[-1]['yaw_error']),
        max_abs_vx_after_reset=max([abs(float(r['raw_vx'])) for r in after_reset] or [0]),
        max_abs_wz_after_reset=max([abs(float(r['raw_wz'])) for r in after_reset] or [0]),
        csv_sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def verify_footprint(record, case_config):
    if record.get('source_sha256') != case_config['footprint_sha256']:
        return False
    source = yaml.safe_load(Path(case_config['footprint_file']).read_text())
    actual = record.get('effective_polygon', [])
    raw = source['footprint']; padding = source['footprint_padding']
    if len(actual) != len(raw):
        return False
    return all(abs(actual[i][axis] - (value + ((value > 0)-(value < 0))*padding)) <= 1e-6
               for i, point in enumerate(raw) for axis, value in enumerate(point))


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--variant',choices=['baseline','candidate','feature_disabled'],required=True)
    parser.add_argument('--match',default='*')
    args=parser.parse_args()
    root=args.root.resolve(); cases=root/'cases'; output=root/args.variant;output.mkdir(exist_ok=True)
    env=dict(os.environ,ROS_DOMAIN_ID={'baseline':'179','candidate':'180','feature_disabled':'181'}[args.variant],ROS_LOCALHOST_ONLY='1',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    if args.variant=='baseline':
        prefix=root/'baseline_install/navigo_mppi_controller'
        env['AMENT_PREFIX_PATH']=str(prefix)+':'+env.get('AMENT_PREFIX_PATH','')
        env['LD_LIBRARY_PATH']=str(prefix/'lib')+':'+env.get('LD_LIBRARY_PATH','')
    config=cases/('candidate.yaml' if args.variant=='candidate' else 'baseline.yaml')
    summary={}
    for case in sorted(cases.glob(args.match+'_seed*.yaml')):
        stem=case.stem
        csv_path=output/(stem+'.csv'); log=output/(stem+'.log')
        footprint_record=Path(str(csv_path)+'.footprint.yaml')
        footprint_record.unlink(missing_ok=True)
        case_config=yaml.safe_load(case.read_text())
        csv_path.unlink(missing_ok=True)  # Never score a stale successful artifact after a crash.
        command=[str(root/'harness_build/controller_feedback'),str(case),str(config),str(csv_path),'--ros-args','--log-level','warn']
        started=time.monotonic()
        try:
            with log.open('w') as stream:
                result=subprocess.run(command,env=env,stdout=stream,stderr=subprocess.STDOUT,timeout=90,check=False)
            item=summarize(csv_path,result.returncode)
        except subprocess.TimeoutExpired:
            item=summarize(csv_path,124)
            item['timeout']=True
        if footprint_record.exists():
            item['footprint']=yaml.safe_load(footprint_record.read_text())
            item['footprint_verified']=verify_footprint(item['footprint'], case_config)
        else:
            item['footprint_verified']=False
        if not item['footprint_verified']:
            item['success']=False
            item['error']='Missing or mismatched runtime footprint provenance'
        item['expected_behavior']=case_config.get('expected_behavior', 'shadow' if 'shadow' in stem else ('intentional_hold' if 'blocked_rotation' in stem or 'tiny_limit' in stem else 'reachable_feedback'))
        item['runtime_sec']=time.monotonic()-started
        item['command']=command
        summary[stem]=item
        print(args.variant,stem,json.dumps({k:v for k,v in item.items() if k not in ('command','csv_sha256','exception_examples','footprint')}),flush=True)
        (output/'summary.json').write_text(json.dumps(summary,indent=2))
    (output/'environment.json').write_text(json.dumps({key:env[key] for key in ('AMENT_PREFIX_PATH','LD_LIBRARY_PATH','ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY')},indent=2))

if __name__=='__main__':
    main()

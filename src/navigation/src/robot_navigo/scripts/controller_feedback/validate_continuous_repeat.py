#!/usr/bin/env python3
"""Repeat six frozen continuous cases; compare numerical trajectory/control fields."""
import argparse
import csv
import json
import os
from pathlib import Path
import subprocess


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--mppi-prefix',type=Path,required=True)
    args=parser.parse_args();root=args.root.resolve()
    output=root/'repeat_validation';output.mkdir(exist_ok=False)
    env=os.environ.copy();env.update(ROS_DOMAIN_ID='186',ROS_LOCALHOST_ONLY='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1')
    env['LD_LIBRARY_PATH']=str(args.mppi_prefix.resolve()/'lib')+':'+env.get('LD_LIBRARY_PATH','')
    env['AMENT_PREFIX_PATH']=str(args.mppi_prefix.resolve())+':'+env.get('AMENT_PREFIX_PATH','')
    results=[]
    keys=['step','t','x','y','yaw','vx','vy','wz','raw_vx','raw_vy','raw_wz','xy_error','yaw_error','collision','success']
    for run in json.loads((root/'results.json').read_text()):
        if run['seed']!=42 or run['scenario'] not in ('back_facing','side_offset','overshoot_outside'):continue
        command=list(run['command']);original=Path(command[3]);target=output/(run['scenario']+'_'+run['arm']+'.csv');command[3]=str(target)
        with target.with_suffix('.log').open('w') as log:
            code=subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=120).returncode
        a=list(csv.DictReader(original.open()));b=list(csv.DictReader(target.open()))
        if code!=run['exit_code'] or len(a)!=len(b):raise RuntimeError('Repeat status/length mismatch')
        difference=max(abs(float(x[k])-float(y[k])) for x,y in zip(a,b) for k in keys)
        if difference>1e-9:raise RuntimeError(f'Repeat trajectory mismatch {difference}')
        results.append(dict(scenario=run['scenario'],arm=run['arm'],max_numeric_difference=difference,
                            exact_csv=original.read_bytes()==target.read_bytes(),command=command))
    if len(results)!=6:raise RuntimeError('Expected six comparisons')
    (output/'results.json').write_text(json.dumps(results,indent=2)+'\n')
    print('Six deterministic numerical trajectory comparisons passed')


if __name__=='__main__':main()

#!/usr/bin/env python3
"""Run a bounded offline critic matrix with frozen native binaries and no hardware bridge."""
import argparse
import hashlib
import itertools
import json
import os
from pathlib import Path
import subprocess
import yaml


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True)
    args=parser.parse_args();root=args.root.resolve();out=root/'runs';out.mkdir(exist_ok=False)
    protocol=json.loads((root/'inputs/protocol.json').read_text())
    env=os.environ.copy();env.update(ROS_DOMAIN_ID='184',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                                   ROS_LOCALHOST_ONLY='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1')
    env['LD_LIBRARY_PATH']=str(root/'install/lib')+':'+env.get('LD_LIBRARY_PATH','')
    env['AMENT_PREFIX_PATH']=str(root/'install')+':'+env.get('AMENT_PREFIX_PATH','')
    binary=root/'probe_build/cost_probe';identity={str(binary):digest(binary)}
    ldd=subprocess.check_output(['ldd',str(binary)],env=env,text=True);(root/'runtime.ldd.txt').write_text(ldd)
    for line in ldd.splitlines():
        tokens=line.split()
        if '=>' in tokens and len(tokens)>2 and tokens[2].startswith('/'):
            path=Path(tokens[2]);identity[str(path)]=digest(path)
    identity[str(root/'install/lib/libmppi_critics.so')]=digest(root/'install/lib/libmppi_critics.so')
    for path in (root/'inputs').rglob('*'):
        if path.is_file():identity[str(path)]=digest(path)
    (root/'identity_before.json').write_text(json.dumps(identity,indent=2)+'\n')
    jobs=[]
    for snapshot,seed,warm,weight in itertools.product(protocol['snapshots'],protocol['seeds'],protocol['warm_starts'],protocol['weights']):
        jobs.append((snapshot,seed,warm,weight,False))
    for snapshot,warm in itertools.product(protocol['snapshots'],protocol['warm_starts']):
        jobs.append((snapshot,42,warm,5,True))
    records=[]
    for snapshot,seed,warm,weight,fixed in jobs:
        name=f"{snapshot['name']}_s{seed}_{warm}_w{weight}"+('_fixed' if fixed else '')
        directory=out/name;directory.mkdir()
        case={**snapshot,'seed':seed,'warm_start':warm,'fixed_proposals':fixed}
        (directory/'case.yaml').write_text(yaml.safe_dump(case))
        command=[str(binary),str(directory/'case.yaml'),str(root/f'inputs/params_weight{weight}.yaml'),str(directory/'trace')]
        with (directory/'process.log').open('w') as log:
            result=subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=30)
        records.append(dict(name=name,snapshot=snapshot['name'],seed=seed,warm=warm,weight=weight,fixed=fixed,
                            directory=str(directory),command=command,exit_code=result.returncode))
        (root/'runs.json').write_text(json.dumps(records,indent=2)+'\n')
        if result.returncode:raise RuntimeError(f'Failed diagnostic {name}')
        if len(records)%18==0:print(f'{len(records)}/{len(jobs)} completed',flush=True)
    after={path:digest(Path(path)) for path in identity}
    (root/'identity_after.json').write_text(json.dumps(after,indent=2)+'\n')
    if identity!=after:raise RuntimeError('Runtime or input changed during matrix')


if __name__=='__main__':main()

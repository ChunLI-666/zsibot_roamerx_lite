#!/usr/bin/env python3
"""Paired continuous controller feedback; synthetic static geometry, ideal SE2 plant."""
import argparse
import copy
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import yaml


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def merge(target, overlay):
    for key, value in overlay.items():
        if isinstance(value, dict):
            merge(target.setdefault(key, {}), value)
        else:
            target[key] = value


def longest(rows, predicate, dt):
    best = current = 0
    for row in rows:
        current = current + 1 if predicate(row) else 0
        best = max(best, current)
    return best * dt


def metrics(trace, code):
    rows = list(csv.DictReader(trace.open()))
    if not rows:
        raise RuntimeError('Empty feedback trace')
    dt = .1
    def f(row, key):
        return float(row[key])
    collisions = sum(r['collision'] == '1' for r in rows)
    success = code == 0 and rows[-1]['success'] == '1' and collisions == 0
    # The terminal row reports a decision at its timestamp, not an executed
    # interval. Collision runs are censored: do not integrate the blocked command.
    executed = rows[:-1]
    return dict(success=success, exit_code=code, samples=len(rows),
        observed_time_sec=f(rows[-1], 't'),
        final_xy_error=f(rows[-1], 'xy_error'), final_yaw_error=f(rows[-1], 'yaw_error'),
        collision_checks_failed=collisions, exceptions=sum(bool(r['exception']) for r in rows),
        exception_examples=sorted({r['exception'] for r in rows if r['exception']})[:5],
        reverse_distance_m=sum(max(0, -f(r,'vx')) * dt for r in executed),
        longest_reverse_sec=longest(executed, lambda r: f(r,'vx') < -1e-6, dt),
        longest_stationary_sec=longest(executed, lambda r:
            math.hypot(f(r,'vx'),f(r,'vy')) < 1e-6 and abs(f(r,'wz')) < 1e-6, dt),
        negative_raw_samples=sum(f(r,'raw_vx') < -1e-6 for r in rows),
        trajectory_length_m=sum(math.hypot(f(b,'x')-f(a,'x'),f(b,'y')-f(a,'y'))
                                for a,b in zip(rows,rows[1:])),
        final_pose=[f(rows[-1], k) for k in ('x','y','yaw')], csv_sha256=sha(trace),
        metric_boundary='Observed pre-command poses through last timestamp; final interval excluded')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--mppi-prefix', type=Path, required=True)
    args = parser.parse_args()
    root = args.root.resolve(); root.mkdir(exist_ok=False)
    package = Path(__file__).resolve().parents[2]
    repo = package.parents[3]
    footprint = package / 'params/zsl1_model_envelope.yaml'
    base = yaml.safe_load(subprocess.check_output(['git','-C',str(repo),'show',
        '6942974:src/navigation/src/robot_navigo/params/navigo_params.yaml']))
    cp = base['controller_server']['ros__parameters']
    cp['use_sim_time'] = True
    cp['general_goal_checker'] = dict(plugin='navigo_path_controller::StoppedGoalChecker', stateful=False,
        xy_goal_tolerance=.25, yaw_goal_tolerance=.25, trans_stopped_velocity=.01, rot_stopped_velocity=.01)
    cp['FollowPath']['visualize'] = False
    cp['FollowPath']['regenerate_noises'] = False
    cp['FollowPath']['forward_alignment'] = {'enabled': False}
    candidate = copy.deepcopy(base)
    merge(candidate, yaml.safe_load((package/'params/forward_alignment_experiment.yaml').read_text()))
    for name, config in [('legacy',base), ('forward',candidate)]:
        (root/f'{name}.yaml').write_text(yaml.safe_dump(config))
    grid = root/'empty.costmap'; grid.write_bytes(bytes(400*400))
    spec = dict(width=400, height=400, resolution=.05, origin_x=-10., origin_y=-10., data=str(grid))
    path = [[i*.05,0.,0.] for i in range(81)]
    scenarios = [('aligned',[0.,0.,0.]), ('back_facing',[0.,0.,math.pi]),
                 ('side_offset',[0.,.6,0.]), ('side_facing',[0.,0.,math.pi/2]),
                 ('overshoot_inside',[4.15,0.,0.]), ('overshoot_outside',[4.35,0.,0.])]
    common = dict(map=spec, dt=.1, steps=1800, deadband=True, path=path, success_hold_steps=6,
        footprint_file=str(footprint), footprint_sha256=sha(footprint),
        load_inflation_layer=True, inflation_radius=.5, cost_scaling_factor=3.)
    env = os.environ.copy(); env.update(ROS_DOMAIN_ID='185', ROS_LOCALHOST_ONLY='1',
                                       OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1')
    prefix = args.mppi_prefix.resolve()
    env['LD_LIBRARY_PATH'] = str(prefix/'lib')+':'+env.get('LD_LIBRARY_PATH','')
    env['AMENT_PREFIX_PATH'] = str(prefix)+':'+env.get('AMENT_PREFIX_PATH','')
    binary = args.binary.resolve()
    identity = {str(binary):sha(binary), str(footprint):sha(footprint)}
    for library in (prefix/'lib').glob('*.so'):
        identity[str(library)] = sha(library)
    for name, initial in scenarios:
        for seed in (42,43,44):
            case = dict(common, name=name, initial=initial, seed=seed)
            (root/f'{name}_{seed}.yaml').write_text(yaml.safe_dump(case))
    for p in root.iterdir():
        if p.is_file(): identity[str(p)] = sha(p)
    (root/'identity_before.json').write_text(json.dumps(identity,indent=2))
    results=[]
    for name, _ in scenarios:
        for seed in (42,43,44):
            for arm in ('legacy','forward'):
                output=root/f'{name}_{seed}_{arm}'; output.mkdir()
                command=[str(binary),str(root/f'{name}_{seed}.yaml'),str(root/f'{arm}.yaml'),
                         str(output/'trace.csv'),'--ros-args','--log-level','warn']
                with (output/'process.log').open('w') as log:
                    try:
                        code=subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=120).returncode
                    except subprocess.TimeoutExpired:
                        code=124
                trace=output/'trace.csv'
                item=metrics(trace,code) if trace.exists() and trace.stat().st_size else dict(success=False,exit_code=code,error='No trace')
                record=yaml.safe_load(Path(str(trace)+'.footprint.yaml').read_text())
                from run_cases import verify_footprint
                if not verify_footprint(record, common): raise RuntimeError('Footprint provenance mismatch')
                item.update(scenario=name,seed=seed,arm=arm,command=command,footprint_verified=True)
                results.append(item)
                (root/'results.json').write_text(json.dumps(results,indent=2)+'\n')
                print(name,seed,arm,item['success'],item.get('reverse_distance_m'),flush=True)
    after={p:sha(Path(p)) for p in identity}
    (root/'identity_after.json').write_text(json.dumps(after,indent=2))
    if identity != after: raise RuntimeError('Inputs/runtime changed')
    (root/'protocol.json').write_text(json.dumps(dict(base_commit=subprocess.check_output(['git','-C',str(repo),'rev-parse','HEAD'],text=True).strip(), seeds=[42,43,44],
        trials=36, simulation_budget_sec=180, full_path_goal=[4,0,0], footprint_sha256=sha(footprint),
        limits=['Synthetic free-space cases, not recorded 0829 replay',
                'Ideal SE2 direct command feedback with deadband; no smoother, SDK, perception or BT',
                'Same current MPPI binary with legacy versus forward policy; both start with zero history',
                'Sampled footprint collision checks, not physical contact or formal continuous clearance certification',
                'Pre-command metric boundary excludes terminal interval; timeout is not success'],
        source_config_commit='6942974'),indent=2)+'\n')


if __name__ == '__main__':
    main()

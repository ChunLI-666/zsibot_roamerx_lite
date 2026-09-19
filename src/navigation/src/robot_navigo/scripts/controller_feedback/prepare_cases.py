#!/usr/bin/env python3
"""Create frozen-map controller experiments; this does not publish ROS data."""
import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path
import subprocess
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--workspace', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root, out = args.workspace.resolve(), args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    repo = root/'src/zsibot/zsibot_roamerx_lite'
    rel = 'src/navigation/src/robot_navigo/params/navigo_params.yaml'
    base = yaml.safe_load(subprocess.check_output(['git', '-C', str(repo), 'show', '6942974:'+rel]))
    # Keep controller settings from the 0829 branch baseline. Only test-time clock
    # and reproducible noise options differ for both variants.
    cp = base['controller_server']['ros__parameters']
    cp['use_sim_time'] = True
    cp['general_goal_checker'] = dict(plugin='navigo_path_controller::StoppedGoalChecker', stateful=False, xy_goal_tolerance=.25, yaw_goal_tolerance=.25, trans_stopped_velocity=.01, rot_stopped_velocity=.01)
    cp['FollowPath']['regenerate_noises'] = False
    cp['FollowPath']['visualize'] = False
    (out/'baseline.yaml').write_text(yaml.safe_dump(base))
    overlay = yaml.safe_load((repo/'src/navigation/src/robot_navigo/params/forward_alignment_experiment.yaml').read_text())
    def merge(dst, src):
        for key, value in src.items():
            if isinstance(value, dict):
                merge(dst.setdefault(key, {}), value)
            else:
                dst[key] = value
    merge(base, overlay)
    (out/'candidate.yaml').write_text(yaml.safe_dump(base))
    map_path = root/'artifacts/matrix_scene_terrain_wh_gt_filtered_20260827/map.yaml'
    meta = yaml.safe_load(map_path.read_text())
    im = np.array(Image.open(map_path.parent/meta['image']))
    free = im > 255*(1-.196)  # Matrix runner threshold: preserve 205 unknown
    obstacle = im < 255*(1-meta['occupied_thresh'])
    cost = np.where(free, 0, np.where(obstacle, 254, 255)).astype(np.uint8)
    # Frozen static occupancy and identical exponential inflation for both arms.
    distance = distance_transform_edt(np.pad(free, 1, constant_values=False))[1:-1,1:-1]*meta['resolution']
    inflation = np.where(distance <= .16, 253, 252*np.exp(-3*(distance-.16)))
    cost = np.where(free & (distance <= .4), np.maximum(cost, inflation), cost).astype(np.uint8)
    np.flipud(cost).tofile(out/'warehouse.costmap')
    map_spec = dict(width=cost.shape[1], height=cost.shape[0], resolution=meta['resolution'],
                    origin_x=meta['origin'][0], origin_y=meta['origin'][1], data=str(out/'warehouse.costmap'))
    # Select a 2 m horizontal line with largest minimum clearance, deterministically.
    candidates = []
    length_cells = round(2/meta['resolution'])
    for iy in range(0, free.shape[0], 4):
        for ix in range(0, free.shape[1]-length_cells, 4):
            clearance = float(distance[iy, ix:ix+length_cells+1].min())
            if clearance > .8:
                candidates.append((clearance, iy, ix))
    if not candidates:
        raise RuntimeError('No 2 m test segment with >= .8 m map clearance')
    clearance, iy, ix = max(candidates)
    x = meta['origin'][0]+(ix+.5)*meta['resolution']
    y = meta['origin'][1]+(free.shape[0]-iy-.5)*meta['resolution']
    common = dict(map=map_spec, dt=.1, steps=1800, deadband=True)
    path = [[float(x+s), y, 0] for s in np.linspace(0, 2, 81)]
    cases = []
    for deg in (0, 45, 90, 135, 180):
        cases.append(dict(common, name=f'warehouse_heading_{deg}', initial=[x,y,math.radians(deg)], path=path))
    cases.append(dict(common,name='warehouse_speed_limit',initial=[x,y,math.pi/2],path=path, speed_limit_step=1,speed_limit=50.,reset_step=30))
    cases.append(dict(common,name='warehouse_absolute_limit',initial=[x,y,math.pi/2],path=path, speed_limit_step=1,speed_limit=.05,percentage=False,dynamic_step=20,reset_step=30))
    cases.append(dict(common,name='warehouse_tiny_limit',initial=[x,y,math.pi/2],path=path, speed_limit_step=1,speed_limit=10.,reset_step=30,steps=80))
    cases.append(dict(common,name='warehouse_final_yaw',initial=[x+2,y,math.pi/2],path=path[-3:]))
    cases.append(dict(common,name='warehouse_final_xy_drift',initial=[x+2,y,math.pi/2],path=path,
                      perturb_step=20,perturb_x=-.4,steps=800))
    # A point obstacle outside the initial footprint, inside the swept footprint.
    blocked = np.flipud(cost).copy()
    bx=round((x+.05-map_spec['origin_x'])/map_spec['resolution'])
    by=round((y+.29-map_spec['origin_y'])/map_spec['resolution'])
    blocked[by,bx]=254
    blocked.tofile(out/'warehouse_blocked.costmap')
    blocked_spec=dict(map_spec,data=str(out/'warehouse_blocked.costmap'))
    cases.append(dict(common,name='warehouse_blocked_rotation',initial=[x,y,0],
                      path=[[x,float(y+s),math.pi/2] for s in np.linspace(0,2,81)],
                      map=blocked_spec,steps=50,stop_on_exception=False))
    # 0829 same recorded pose + path geometry, empty bounded context map. This is
    # a geometry/behavior counterfactual, NOT reconstruction of historical costmaps.
    extracted=root/'docs/progress/2026-09-08_relocalization_and_reverse_design/rosbag2_2026_08_29-04_41_05_extracted.json'
    bag=json.loads(extracted.read_text())
    debug=bag['/controller_server/debug']; plans=bag['/transformed_global_plan']
    times=[p['t'] for p in plans]
    np.zeros((1200,1200),dtype=np.uint8).tofile(out/'geometry_only.costmap')
    geometry=dict(width=1200,height=1200,resolution=.05,origin_x=-30.,origin_y=-30.,data=str(out/'geometry_only.costmap'))
    for name,start,end in [('reverse_10s',1788003737.6339045,1788003747.6839075),
                           ('reverse_14s',1788003822.9337518,1788003836.783998),
                           ('reverse_20s',1788004164.6340435,1788004184.6840708)]:
        rows=[r for r in debug if start<=r['t']<=end]
        r=rows[0]; index=bisect.bisect_left(times,r['t'])
        p=min(plans[max(0,index-1):index+1],key=lambda p:abs(p['t']-r['t']))
        assert p['frame']==r['frame']=='odom' and abs(p['t']-r['t'])<.15
        initial=[r['x'],r['y'],r['yaw']]
        cases.append(dict(common,name='0829_'+name,initial=initial,initial_velocity=[0.,0.,0.],path=p['poses'],map=geometry,
                          source_time=r['t'],historical_context='geometry_only_no_recorded_costmap'))
        cases.append(dict(common,name='0829_'+name+'_shadow',initial=initial,path=p['poses'],map=geometry,
                          steps=len(rows),shadow_stamps=[r['stamp'] for r in rows],shadow=[[r['x'],r['y'],r['yaw'],*r['current_velocity']] for r in rows],
                          source_time=r['t'],historical_context='fixed_initial_path_historical_poses_no_recorded_costmap'))
    for case in cases:
        for seed in (42,43,44):
            target=out/f"{case['name']}_seed{seed}.yaml"
            target.write_text(yaml.safe_dump(dict(case,seed=seed)))
    (out/'manifest.json').write_text(json.dumps(dict(map=str(map_path),map_sha256=hashlib.sha256((map_path.parent/meta['image']).read_bytes()).hexdigest(),
        extracted_sha256=hashlib.sha256(extracted.read_bytes()).hexdigest(),base_commit='6942974',
        segment=[x,y,x+2,y],minimum_segment_clearance=clearance,case_names=[c['name'] for c in cases],
        limitations=['ideal SE2 plant; no MuJoCo or real quadruped dynamics','frozen static map, no dynamic obstacles, perception, TF latency, global planner, BT or SDK',
                     '0829 uses actual initial pose/path, but no reconstructed recorded costmap; closed-loop starts stationary',
                     'shadow uses each recorded pose with fixed first local path; tests C++ decisions, not navigation success',
                     'real StoppedGoalChecker for both groups .25m/.25rad/.01m_s/.01rad_s, stateful=false; current command must also be stopped']),indent=2))

if __name__ == '__main__':
    main()

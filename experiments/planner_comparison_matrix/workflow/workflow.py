#!/usr/bin/env python3
"""Prepare/run/observe/reconcile/close one immutable private simulation case."""
import argparse
import copy
import json
import math
from pathlib import Path
import sys

import workflow_driver as driver
from workflow_inputs import candidate_check, checked_json, pin, route_check, source_check

HERE=Path(__file__).resolve().parent


def expand(value,tokens):
    if isinstance(value,str):
        for key,item in tokens.items():value=value.replace('{'+key+'}',str(item))
        return value
    if isinstance(value,list):return [expand(item,tokens) for item in value]
    if isinstance(value,dict):return {key:expand(item,tokens) for key,item in value.items()}
    return value


def prepare(args):
    paths={key:Path(getattr(args,key)).resolve(strict=True) for key in
        ('candidate','source','map_identity','route','policy','template')}
    refs={key:pin(path) for key,path in paths.items()}
    candidate=candidate_check(paths['candidate'])
    source=checked_json(refs['source']);source_check(paths['source'],live=False)
    route,policy=route_check(paths['route'],paths['policy'],paths['map_identity'])
    if source['map_identity']['sha256']!=refs['map_identity']['sha256']:
        raise ValueError('Source and run map identities differ')
    scope=candidate['scope']
    if scope.get('map_identity_sha256')!=refs['map_identity']['sha256'] or scope.get('policy_sha256')!=refs['policy']['sha256']:
        raise ValueError('Candidate map/policy scope differs')
    original=scope.get('original_route')
    if not original:raise ValueError('Candidate must pin the reviewed original route')
    original_route=checked_json(original)
    verify_route_family(original_route,route)
    plan=expand(checked_json(refs['template']),dict(paths,run_id=driver.valid_name(args.run_id),
        candidate_commit=candidate['commit'],variant=candidate['variant'],workflow_dir=HERE))
    plan.update(schema='terra.workflow.execution_plan.v1',workflow_run_id=args.run_id,
        variant=candidate['variant'],commit=candidate['commit'],build_receipts=copy.deepcopy(candidate['build_receipts']),
        input_binding=refs)
    plan['inputs']=list(plan.get('inputs',[]))+list(refs.values())+[original]
    # A template describes the already reviewed launch/record/render actors.
    # Its source admission is always supplied by this versioned implementation.
    admission=dict(id='source_admission',kind='command',roles=['environment','localization'],
        cwd=str(HERE),env={'ROS_DOMAIN_ID':'185','PATH':'/usr/bin:/bin'},
        argv=[sys.executable,'-B',str(HERE/'workflow.py'),'source-check','--source',str(paths['source']),
            '--source-sha256',refs['source']['sha256'],
            '--minimum-lease-sec',str(policy['mission_budget_sec']+300.),'--output','{run_dir}/source_admission.json'],
        files=[pin(HERE/'workflow.py'),pin(HERE/'workflow_inputs.py')],
        success_files=[dict(path='{run_dir}/source_admission.json',equals={'ok':True,'live':True})])
    plan['stages'].insert(0,admission)
    for stage in plan['stages'][1:]:
        stage['depends_on']=list(dict.fromkeys(['source_admission',*stage.get('depends_on',[])]))
        if stage['kind']=='client' and stage['client']['operation']=='start':
            stage['client'].setdefault('args',{}).update(route=str(paths['route']),policy=str(paths['policy']))
        if 'closure' in stage.get('roles',[]) and stage.get('exact_owned_cleanup') is not True:
            raise ValueError('Closure must use the bound exact-actor cleanup contract')
    driver.validate_plan(plan);driver.approval_check(plan,paths['candidate'])
    raw=(json.dumps(plan,ensure_ascii=False,sort_keys=True,indent=2)+'\n').encode()
    run_dir=Path(args.run_dir).resolve()
    if run_dir.exists():
        _,old,state=driver.read_run(run_dir)
        if old!=plan:raise ValueError('Run id already bound to different immutable inputs')
        print(json.dumps(dict(mode='REUSED_ORIGINAL_RUN',workflow_run_id=state['workflow_run_id'],
            request_ids=[row['request_id'] for row in state['stages'] if row['request_id']])));return
    run_dir.parent.mkdir(parents=True,exist_ok=True)
    plan_path=run_dir.parent/(args.run_id+'.execution_plan.json')
    if plan_path.exists():
        if plan_path.read_bytes()!=raw:raise ValueError('Prepared plan path already differs')
    else:driver.write_once(plan_path,raw)
    driver.init(plan_path,run_dir)


def verify_route_family(original,route):
    if original==route:return
    # Full-map reanchoring permits only rotation of the same directed cycle.
    rows=original.get('waypoints',[]);target=route.get('waypoints',[])
    if len(rows)!=41 or len(target)!=41 or rows[0]['pose']!=rows[-1]['pose'] or target[0]['pose']!=target[-1]['pose']:
        raise ValueError('Route differs from the reviewed route family')
    base=rows[:-1];rotated=target[:-1]
    if len({row['id'] for row in base})!=40 or {row['id'] for row in base}!={row['id'] for row in rotated}:
        raise ValueError('Original cycle goals differ')
    by_id={row['id']:row for row in base}
    if any(by_id[row['id']]!=row for row in rotated):raise ValueError('Original waypoint changed')
    edges=lambda values:{(values[i]['id'],values[(i+1)%len(values)]['id']) for i in range(len(values))}
    if edges(base)!=edges(rotated) or original['mission_budget_sec']!=route['mission_budget_sec']:
        raise ValueError('Original directed edges or time budget changed')
    if any(target[-1].get(key)!=target[0].get(key) for key in ('pose','require_stopped')):
        raise ValueError('Cycle closure differs')


def prepare_cycle(args):
    original,policy=route_check(args.route,args.policy,args.map_identity)
    anchor=json.loads(Path(args.anchor).read_text());rows=original['waypoints']
    if (len(rows)!=41 or rows[0]['pose']!=rows[-1]['pose'] or anchor.get('frame')!='map'
            or not all(math.isfinite(anchor.get(key,math.nan)) for key in ('x','y','yaw'))):
        raise ValueError('Full original cycle and finite Native map anchor required')
    nodes=rows[:-1];distances=[math.hypot(row['pose'][0]-anchor['x'],row['pose'][1]-anchor['y']) for row in nodes]
    first=min(range(len(nodes)),key=distances.__getitem__)
    if distances[first]>=8.5:raise ValueError('Nearest original point exceeds existing first-leg envelope')
    rotated=copy.deepcopy(nodes[first:]+nodes[:first]);close=copy.deepcopy(rotated[0]);close['id']+='_CLOSE'
    close['tags']=list(close.get('tags',[]))+['closure_duplicate_of_'+rotated[0]['id']]
    route=copy.deepcopy(original);route['waypoints']=rotated+[close]
    route['provenance']=dict(protocol='matrix.same_directed_cycle.v1',source_route=pin(args.route),
        anchor=pin(args.anchor),case_id=driver.valid_name(args.case_id),old_receipts_imported=False)
    verify_route_family(original,route)
    output=Path(args.output_dir).resolve();output.mkdir(parents=True,exist_ok=False)
    path=output/'route.EXECUTABLE.json'
    driver.write_once(path,(json.dumps(route,ensure_ascii=False,indent=2)+'\n').encode())
    receipt=dict(schema='matrix.simulation.route_equivalence.v1',source_route=pin(args.route),route=pin(path),
        policy=pin(args.policy),map_identity=pin(args.map_identity),anchor=pin(args.anchor),
        original_goals_exact=True,directed_edges_equal=True,directed_edge_count=40,new_case_initial_prefix=0,
        first_goal_distance_m=distances[first],selected_first=rotated[0]['id'],
        boundary='Anchor rotates the identical cycle only; normal fresh-input, stopped and body checks remain mandatory.')
    driver.write_once(output/'route_equivalence.json',(json.dumps(receipt,indent=2)+'\n').encode())
    print(json.dumps(receipt))


def main():
    parser=argparse.ArgumentParser(description=__doc__);sub=parser.add_subparsers(dest='command',required=True)
    p=sub.add_parser('prepare')
    for key in ('candidate','source','map-identity','route','policy','template','run-id','run-dir'):p.add_argument('--'+key,required=True)
    p=sub.add_parser('source-check');p.add_argument('--source',required=True)
    p.add_argument('--source-sha256')
    p.add_argument('--minimum-lease-sec',type=float,default=1.);p.add_argument('--output',required=True)
    p=sub.add_parser('prepare-cycle')
    for key in ('route','policy','map-identity','anchor','case-id','output-dir'):p.add_argument('--'+key,required=True)
    for command in ('dry-run','status','run','reconcile','close'):
        p=sub.add_parser(command);p.add_argument('--run-dir',required=True)
        if command in ('run','reconcile','close'):
            p.add_argument('--candidate',required=True);p.add_argument('--execute',action='store_true');p.add_argument('--follow',action='store_true')
        if command=='reconcile':p.add_argument('--client-stage',required=True)
    args=parser.parse_args()
    if args.command=='prepare':prepare(args);return 0
    if args.command=='prepare-cycle':prepare_cycle(args);return 0
    if args.command=='source-check':
        result=source_check(args.source,args.minimum_lease_sec,expected_sha256=args.source_sha256)
        driver.write_once(Path(args.output),(json.dumps(result,indent=2)+'\n').encode())
        print(json.dumps(result));return 0
    if args.command=='status':driver.status(args.run_dir);return 0
    if args.command=='dry-run' or not args.execute:driver.dry_run(args.run_dir);return 0
    if args.command=='reconcile':return driver.reconcile(args.run_dir,args.client_stage,args.candidate,args.follow)
    if args.command=='close':return driver.execute(args.run_dir,args.candidate,args.follow,close_only=True)
    if args.follow:return driver.follow_with_reconciliation(args.run_dir,args.candidate)
    return driver.execute(args.run_dir,args.candidate,False)


if __name__=='__main__':
    try:raise SystemExit(main())
    except (OSError,ValueError,KeyError,TypeError) as error:
        print('HOLD: '+str(error),file=sys.stderr);raise SystemExit(10)

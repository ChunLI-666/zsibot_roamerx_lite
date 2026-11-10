"""Explicit Main-approved mission continuation. File/SQLite operations only; no ROS."""
import copy
import json
import math
from pathlib import Path
import time
import uuid

from mission_state import Hold, MissionStore, IDENTITY_KEYS, atomic_json, boot_clock, digest, encoded, validate_identity

CLIENT_SOURCES=('mission_state.py','mission_resume.py','mission_continuation.py','mission_storage.py')
HERE=Path(__file__).resolve().parent


def require(condition,reason):
    if not condition:raise Hold('Continuation: '+reason)


def finite(value):
    return type(value) in (int,float) and math.isfinite(value)


def decode_json(raw):
    def unique(pairs):
        value={}
        for key,item in pairs:
            if key in value:raise Hold('Duplicate JSON key: '+key)
            value[key]=item
        return value
    return json.loads(raw,object_pairs_hook=unique,
        parse_constant=lambda value:(_ for _ in ()).throw(Hold('Nonfinite JSON value: '+value)))


def read_json(path):return decode_json(Path(path).read_bytes())


def read_ref(reference):
    require(isinstance(reference,dict) and set(reference)=={'path','sha256'},'typed path/SHA reference required')
    path=Path(reference['path']);require(path.is_absolute(),'evidence path must be absolute')
    raw=path.read_bytes();require(digest(raw)==reference['sha256'],'evidence hash mismatch: '+str(path))
    # Parse the bytes just hashed, rather than reopening a mutable evidence path.
    return decode_json(raw)


def verify_ready(ready,identity,now_ns):
    require(ready.get('ready') is True and ready.get('variant')==identity['variant'],'new runtime not ready')
    require(type(ready.get('valid_until_ns')) is int and ready['valid_until_ns']>now_ns,'runtime READY expired')
    require(ready.get('mission_identity')==identity and identity.get('verified') is True,'actual runtime identity differs')
    require(ready.get('recovery_runtime',{}).get('verified') is True,'recovery runtime unverified')
    sources={name:digest((HERE/name).read_bytes()) for name in CLIENT_SOURCES}
    capability=ready.get('waypoint_contract',{}).get('mission_resume',{})
    require(capability.get('schema')=='matrix.mission.resume_capability.v1'
        and capability.get('client_sources_sha256')==sources and capability.get('bt_retries')==1,'runtime client/source capability differs')
    ownership=ready.get('waypoint_ownership',{})
    require(all(ownership.get(key)==[] for key in ('navigate_to_pose_active','follow_waypoints_active','follow_path_active')),
        'active or unknown goal at adoption boundary')
    require(ownership.get('exclusive_nav_client') is True and ownership.get('follow_path_status_publishers_match') is True,
        'exclusive ownership unverified')
    for stamp in (ready.get('time_ns'),ownership.get('checked_unix_ns')):
        require(type(stamp) is int and -.05<=(now_ns-stamp)/1e9<=.5,'stale runtime boundary observation')


def verify_closure(closed,mission_id,action_uuid):
    client=closed.get('client',{});shutdown=closed.get('closure',{}).get('shutdown',{})
    codes=closed.get('closure',{}).get('returncodes',{})
    required={'recorder','nav_safety_gate','matrix_zero_handoff','planner_server','controller_server',
        'bt_navigator','waypoint_follower','velocity_smoother','lifecycle_manager'}
    require(closed.get('schema')=='terra.recording.exact_closure.v1' and closed.get('complete') is True,
        'exact closed runtime required')
    require(client.get('mission_id')==mission_id and client.get('action_uuid')==action_uuid,'closure mission/UUID differs')
    require(bool(closed.get('run_id')) and shutdown.get('run_id')==closed['run_id']
        and shutdown.get('stage')=='recording_closed' and shutdown.get('reason')=='all_owned_processes_waited'
        and required<=codes.keys() and all(type(v) is int and v==0 for v in codes.values()),'owned actors not proven closed')
    return shutdown


def validate_interval(interval,credit,design,state,new_identity,new_identity_ref,closed,now_boot,now_ns,maximum=2400):
    policy=design['credit_policy'];start=credit.get('start_boottime_sec');end=credit.get('end_boottime_sec');grant=credit.get('grant_sec')
    require(all(finite(v) for v in (start,end,grant)) and start<end<=now_boot+.05,'invalid credit window')
    require(credit.get('boot_id')==policy['start_boot_id']==state['budgets']['deadline_boot_id'],'credit boot differs')
    require(start==policy['start_boottime_sec']==closed['closure']['shutdown']['boottime_sec'],'credit start is not exact closure')
    require(grant==min(end-start,float(maximum)) and grant>0,'credit must equal min(actual interval,approved cap), once')
    require(interval.get('schema')=='matrix.mission.inactive_interval.v1'
        and interval.get('mission_id')==state['mission_id'] and interval.get('boot_id')==credit['boot_id']
        and interval.get('start_boottime_sec')==start and interval.get('end_boottime_sec')==end
        and interval.get('start_closure_sha256')==policy['start_evidence_sha256'],'interval identity differs')
    require(interval.get('accounting_scope')=='cooperative_single_owner'
        and interval.get('registered_a_execution_clients')==[] and interval.get('registered_a_action_uuids')==[],
        'registered A execution during credit interval')
    references=interval.get('registration_evidence')
    require(isinstance(references,list) and bool(references),'registration audit evidence required')
    for reference in references:read_ref(reference)
    b_ids=[]
    for record in interval.get('b_runs',[]):
        b=read_ref(record['state']);b_closed=read_ref(record['closure'])
        require(b['mission_id']!=state['mission_id'] and b['variant']=='official','B accounting includes A or wrong variant')
        validate_identity(b['identity'])
        require(b['attempts'] and all(str(uuid.UUID(a['parent_action_uuid']))==a['parent_action_uuid'] for a in b['attempts']),
            'B action UUID provenance missing')
        require(not ({a['parent_action_uuid'] for a in b['attempts']} & {a['parent_action_uuid'] for a in state['attempts']}),
            'B accounting reuses an A action UUID')
        shutdown=verify_closure(b_closed,b['mission_id'],b['attempt']['parent_action_uuid'])
        require(b_closed['client'].get('state_sha256')==record['state']['sha256'],'B closure/state binding differs')
        bs=record.get('start_boottime_sec');be=record.get('end_boottime_sec')
        require(finite(bs) and finite(be) and start<=bs<be<=end and be==shutdown['boottime_sec'],'B interval not independently bounded')
        require(record.get('source_sha')==b['identity']['source_sha'],'B source commit differs')
        b_ids.append(b['mission_id'])
    require(len(b_ids)==len(set(b_ids)) and sorted(b_ids)==sorted(interval.get('registered_b_mission_ids',[])),
        'registered B mission lacks separate accounting')
    end_ns=interval.get('end_unix_ns')
    require(type(end_ns) is int and abs((now_boot-end)-(now_ns-end_ns)/1e9)<=2.,'end clock is not the current same-boot observation')
    require(interval.get('new_identity_sha256')==new_identity_ref['sha256'],'interval captured another new identity')
    verify_ready(read_ref(interval['end_ready']),new_identity,end_ns)
    return dict(boot_id=credit['boot_id'],start_boottime_sec=start,end_boottime_sec=end,
        eligible_interval_sec=end-start,granted_sec=grant,maximum_sec=maximum,application_count=1,
        scope='A inactive source-repair and independently accounted B campaign',separate_B_mission_ids=b_ids)


def apply_authorization(state,authorization,authorization_sha256,ready_path):
    try:return _apply_authorization(state,authorization,authorization_sha256,ready_path)
    except (KeyError,TypeError,IndexError,AttributeError) as error:
        raise Hold('Continuation: malformed authorization/evidence structure') from error


def _apply_authorization(state,authorization,authorization_sha256,ready_path):
    """Validate all evidence before mutating the supplied transactional state."""
    a=authorization;before=copy.deepcopy(state);boot,now=boot_clock();now_ns=time.time_ns()
    require(a.get('schema')=='matrix.mission.continuation_authorization.v1' and a.get('approved_by')=='Main'
        and a.get('decision')=='APPROVED','explicit Main instance authorization required')
    require(str(uuid.UUID(a['authorization_id']))==a['authorization_id'],'noncanonical authorization UUID')
    require(state['mission_id']==a['mission_id'] and state['revision']==a['expected_revision']
        and digest(state)==a['expected_state_sha256'],'old mission/revision/state differs')
    require(not state.get('continuation_authorizations') and not state['budgets'].get('authorized_inactive_credit_sec',0),
        'continuation/credit already applied')
    require(a['old_identity_epoch_id']==state['identity_epoch_id'] and a['old_identity_sha256']==digest(state['identity'])
        and a['old_policy_sha256']==digest(state['policy']),'old epoch/identity/policy differs')
    require(str(uuid.UUID(a['new_identity_epoch_id']))==a['new_identity_epoch_id']
        and a['new_identity_epoch_id']!=state['identity_epoch_id'],'new identity epoch required')
    old_snapshot=read_ref(a['old_state_snapshot']);require(encoded(old_snapshot)==encoded(state),'old snapshot does not match DB')
    design=read_ref(a['design_approval'])
    require(design.get('schema')=='main.continuation.design_approval.v1'
        and design.get('decision')=='APPROVED_WITH_SCOPED_CREDIT_DEFINITION'
        and design.get('operation')=='authorize-continuation' and design.get('motion_in_operation') is False
        and design.get('mission_id')==state['mission_id'],'Main design approval differs')
    require(design.get('parent_cap_after')==10 and design.get('per_index_parent_cap')==3
        and design.get('per_index_child_slots')==6 and design['credit_policy'].get('maximum_grant_sec')==2400
        and design['credit_policy'].get('fresh_full_1800_reset') is False,'approved policy limits differ')
    maximum=2400
    if 'design_supplement' in a:
        supplement=read_ref(a['design_supplement'])
        require(supplement.get('schema') in ('main.continuation.design_supplement.v2','main.continuation.design_supplement.v3')
            and supplement.get('decision')=='APPROVED_BOUNDED_A_INACTIVE_CREDIT_CAP_EXTENSION'
            and supplement.get('mission_id')==state['mission_id']
            and supplement.get('previous_approval',{}).get('sha256')==a['design_approval']['sha256']
            and supplement.get('original_navigation_budget_sec')==1800
            and supplement.get('actual_grant_not_yet_issued') is True,'Main design supplement differs')
        if supplement['schema']=='main.continuation.design_supplement.v2':
            require(supplement.get('maximum_grant_sec')==5400,'Main v2 cap differs')
            maximum=5400
        else:
            maximum=supplement.get('maximum_grant_sec')
            # Check the bounded numeric range before isfinite can convert a huge integer.
            require(type(maximum) in (int,float) and 0<maximum<=86400 and finite(maximum),
                'Main v3 cap must be finite, positive and at most 86400 seconds')
    policy=state['policy'];require(policy['max_parent_dispatches']==3 and policy['max_attempts_per_index']==6
        and policy['bt_retries']==1 and policy['mission_budget_sec']==1800,'old campaign policy differs')
    expected_policy=dict(policy,max_parent_dispatches=10,max_parent_dispatches_per_index=3)
    require(encoded(a['new_policy'])==encoded(expected_policy),'only total parent cap3->10 and explicit per-index3 are allowed')
    require(state['budgets']['mission_dispatches_reserved']==3 and len(state['attempts'])==3
        and state['progress']['accepted_prefix_count']==5 and state['progress']['next_index']==5
        and state['current_target']==state['route'][5] and state['current_target']['id']=='F016'
        and state['budgets']['per_index_parent_dispatches'][5]==2 and state['budgets']['per_index_reserved'][5]==4
        and state['result_flags']['strict_timing_status']=='FAIL','approved A15 progress/consumption boundary differs')
    identity=read_ref(a['new_identity']);validate_identity(identity)
    require(identity.get('verified') is True and identity['source_sha']==a['new_source_sha']
        and identity['source_sha']!=state['identity']['source_sha'],'new verified source commit required')
    for key in (*IDENTITY_KEYS,'variant','effective_config_sha256'):
        require(identity.get(key)==state['identity'].get(key),'immutable identity differs: '+key)
    for key in ('runtime_parameters_sha256','runtime_identities_sha256'):
        require(isinstance(identity.get(key),str) and len(identity[key])==64
            and all(c in '0123456789abcdef' for c in identity[key]),'actual runtime observation hash missing')
    require(boot==state['budgets']['deadline_boot_id'],'current boot differs from original mission')
    verify_ready(read_json(ready_path),identity,now_ns)
    result=read_ref(a['stop_result']);attempt=state['attempt'];closed=read_ref(a['closure'])
    require(state['attempts'] and attempt==state['attempts'][-1],'current attempt/history differ')
    require(attempt['terminal_confirmed'] is True and attempt['children_quiescent'] is True and attempt['stopped'] is True,
        'prior attempt is not terminal/quiescent/stopped')
    require(result.get('action_uuid')==attempt['parent_action_uuid'] and result.get('status')==attempt['native_status']
        and result.get('status') in (4,5,6) and result.get('children_quiescent') is True
        and result.get('final_measured_and_commanded_stop') is True,'exact terminal stop result differs')
    stop=result.get('owned_goal_shutdown',{}).get('stop',{})
    require(stop.get('observed') is True and stop.get('strict_stop_issues')==[],'strict measured stop receipt missing')
    verify_closure(closed,state['mission_id'],attempt['parent_action_uuid'])
    require(closed['client'].get('revision')==state['revision']
        and closed['client'].get('state_sha256')==a['old_state_snapshot']['sha256']
        and closed['client'].get('reconciled_native_result_sha256')==a['stop_result']['sha256']
        and a['closure']['sha256']==design['credit_policy']['start_evidence_sha256'],'closure/result/state evidence differs')
    interval=read_ref(a['inactive_interval'])
    credit=validate_interval(interval,a['credit'],design,state,identity,a['new_identity'],closed,now,now_ns,maximum)
    old_deadline=state['budgets']['deadline_boottime_sec'];require(finite(old_deadline),'original deadline missing')
    deadline=old_deadline+credit['granted_sec'];require(deadline>now,'FAILED_BUDGET after authorized credit')
    # Everything below is additive provenance or one of the explicitly allowed changes.
    epochs=state.setdefault('identity_policy_epochs',[])
    require(not epochs,'unexpected existing epoch history')
    epochs.append(dict(epoch_id=state['identity_epoch_id'],identity=copy.deepcopy(state['identity']),
        policy=copy.deepcopy(policy),origin='preserved_pre_continuation_authority'))
    for old_attempt in state['attempts']:
        require(old_attempt.get('identity_epoch_id',state['identity_epoch_id'])==state['identity_epoch_id'],
            'historical attempt has another authority epoch')
        old_attempt.setdefault('identity_epoch_id',state['identity_epoch_id'])
        if 'historical_result_sha256' in old_attempt:
            old_attempt.setdefault('source_provenance',dict(kind='imported_closed_result',historical_result_sha256=old_attempt['historical_result_sha256'],
                epoch_role='import_authority_not_historical_execution_source'))
        else:old_attempt.setdefault('source_provenance',dict(kind='live_execution',source_sha=state['identity']['source_sha']))
    state['attempt']=copy.deepcopy(state['attempts'][-1]);attempt=state['attempt']
    history=attempt.setdefault('native_result_history',[])
    if attempt.get('native_result') and attempt['native_result'] not in history:history.append(copy.deepcopy(attempt['native_result']))
    evidence=copy.deepcopy(a['stop_result'])
    require(not any(item['path']==evidence['path'] and item['sha256']!=evidence['sha256'] for item in history),
        'stop evidence overwrites historical result path')
    if evidence not in history:history.append(evidence)
    attempt['native_result']=evidence;state['attempts'][-1]=copy.deepcopy(attempt)
    state['stop']=dict(state='PASSED',source_action_uuid=attempt['parent_action_uuid'],receipt=copy.deepcopy(stop),evidence=evidence)
    new_identity=dict(identity,route_raw_sha256=state['identity']['route_raw_sha256'],route_semantic_sha256=state['identity']['route_semantic_sha256'])
    state['identity']=new_identity;state['identity_epoch_id']=a['new_identity_epoch_id'];state['policy']=copy.deepcopy(expected_policy)
    epochs.append(dict(epoch_id=state['identity_epoch_id'],identity=copy.deepcopy(new_identity),policy=copy.deepcopy(expected_policy),
        authorization_id=a['authorization_id'],authorization_sha256=authorization_sha256))
    state['budgets']['deadline_boottime_sec']=deadline
    state['budgets']['authorized_inactive_credit_sec']=credit['granted_sec']
    transition=dict(authorization_id=a['authorization_id'],authorization_sha256=authorization_sha256,
        old_epoch_id=before['identity_epoch_id'],new_epoch_id=state['identity_epoch_id'],
        old_identity_sha256=digest(before['identity']),new_identity_sha256=digest(new_identity),
        old_policy=before['policy'],new_policy=copy.deepcopy(expected_policy),
        old_deadline_boottime_sec=old_deadline,new_deadline_boottime_sec=deadline,credit=credit,
        old_stop=before['stop'],corrected_stop=copy.deepcopy(state['stop']),
        preserved_progress_sha256=digest(before['progress']),preserved_consumption=copy.deepcopy(before['budgets']),
        evidence={name:copy.deepcopy(a[name]) for name in ('design_approval','old_state_snapshot','new_identity','stop_result','closure','inactive_interval')},
        accounting_boundary='Cooperative single-owner registered mission evidence; separately accounted B time is eligible A inactivity.',goals_sent=0)
    if 'design_supplement' in a:transition['evidence']['design_supplement']=copy.deepcopy(a['design_supplement'])
    state['continuation_authorizations']=[copy.deepcopy(transition)]
    return transition


def write_once(path,value):
    raw=(encoded(value)+'\n').encode()
    if path.exists():require(path.read_bytes()==raw,'existing export differs; use a new output directory')
    else:atomic_json(path,value)


def main(args):
    require(all((args.authorization,args.authorization_sha256,args.identity,args.ready,args.output,args.request_id,args.mission_id))
        and args.expect_revision is not None,'authorization/SHA, identity, READY, mission/revision, request and output required')
    uuid.UUID(args.request_id)
    raw=args.authorization.read_bytes();require(digest(raw)==args.authorization_sha256,'instance authorization SHA differs')
    authorization=decode_json(raw)
    require(isinstance(authorization,dict) and all(key in authorization for key in
        ('authorization_id','mission_id','expected_revision','new_identity')),'malformed instance authorization')
    require(authorization['mission_id']==args.mission_id and authorization['expected_revision']==args.expect_revision,
        'CLI mission/revision differs from authorization')
    require(Path(authorization['new_identity']['path']).resolve()==args.identity.resolve(),'CLI identity path differs from authorization')
    if not args.execute:
        state=MissionStore.inspect(args.store)
        transition=apply_authorization(copy.deepcopy(state),authorization,args.authorization_sha256,args.ready)
        print(encoded(dict(operation_outcome='VALIDATED_ONLY',goals_sent=0,transition=transition)));return 0
    store=MissionStore(args.store,args.resource_lock)
    try:
        output=args.output.resolve()
        if output.exists() and any(output.iterdir()):
            prior=output/'continuation_receipt.json'
            require(prior.is_file(),'output is not an empty or matching continuation directory')
            existing=read_json(prior)
            require(existing.get('authorization_id')==authorization['authorization_id']
                and existing.get('authorization_sha256')==args.authorization_sha256,'output belongs to another authorization')
            cached=store.db.execute('SELECT receipt FROM continuations WHERE authorization_id=?',(authorization['authorization_id'],)).fetchone()
            require(cached is not None and encoded(existing)==encoded(json.loads(cached[0])),
                'existing export does not match a committed authorization')
        receipt=store.authorize_continuation(authorization,args.authorization_sha256,args.ready)
        output.mkdir(parents=True,exist_ok=True);write_once(output/'continuation_receipt.json',receipt)
        state=receipt['state'];write_once(output/'state.json',state)
        result=dict(schema='matrix.mission.result.v1',mission_id=state['mission_id'],revision=state['revision'],
            lifecycle=state['lifecycle'],identity=state['identity'],identity_epoch_id=state['identity_epoch_id'],
            identity_policy_epochs=state['identity_policy_epochs'],continuation_authorizations=state['continuation_authorizations'],
            progress=state['progress'],result_flags=state['result_flags'],attempts=state['attempts'],budgets=state['budgets'],
            current_target=state['current_target'],last_attempt=state['attempt'],stop=state['stop'],reason=state['hold_reason'])
        write_once(output/'result.json',result)
        response=dict(schema='matrix.mission.operation.v1',operation='authorize-continuation',operation_outcome='AUTHORIZED_CONTINUATION',
            request_id=args.request_id,mission_id=state['mission_id'],revision=state['revision'],authorization_id=authorization['authorization_id'],
            authorization_sha256=args.authorization_sha256,goals_sent=0,exit_code=0,
            state_path=str(output/'state.json'),state_sha256=digest((output/'state.json').read_bytes()),
            result_path=str(output/'result.json'),result_sha256=digest((output/'result.json').read_bytes()),
            receipt_path=str(output/'continuation_receipt.json'),receipt_sha256=digest((output/'continuation_receipt.json').read_bytes()))
        write_once(output/'operation.json',response);print(encoded(response));return 0
    finally:store.close()

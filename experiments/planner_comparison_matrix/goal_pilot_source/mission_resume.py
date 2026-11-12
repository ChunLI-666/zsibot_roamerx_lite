#!/usr/bin/env python3
"""Explicit durable native FollowWaypoints entry. inspect/import require no ROS."""
import argparse
import copy
from collections import deque
import hashlib
import json
import os
from pathlib import Path
import signal
import time
import uuid

from mission_state import (MissionStore, Hold, atomic_json, digest, validate_identity,
                           check_identity_state, remaining_state)
from waypoint_contract import OrderedVisits, evidence_json, validate_route
from mission_continuation import CLIENT_SOURCES

HERE = Path(__file__).resolve().parent


def validate_closed_runtime(closed, result, manifest, identity):
    if closed.get('schema')=='native.fullmap41.exact_pidfd_closure.v1':
        run=result.get('runtime_binding',{}).get('mission_deadline',{}).get('run_id')
        shutdown=closed.get('recording_shutdown',{})
        if (not run or closed.get('run_id')!=run or closed.get('parent_UUID')!=result['action_uuid']
                or closed.get('registered_route_sha256')!=result['route_sha256']
                or closed.get('complete') is not True or closed.get('all9rc0') is not True
                or closed.get('navigation_exited') is not True or closed.get('observer_exited') is not True
                or shutdown.get('run_id')!=run or shutdown.get('stage')!='recording_closed'
                or shutdown.get('reason')!='all_owned_processes_waited'
                or closed.get('completed_ns',0)<result['finished_ns']):
            raise Hold('Historical native closure does not match the terminal run/UUID/route')
        records=[r for r in closed.get('records',[]) if r.get('role')=='navigation']
        if len(records)!=1:raise Hold('Historical navigation closure missing')
        record=records[0];proof=record.get('action_proof',{})
        statuses=proof.get('observed_status',{}).get('/follow_waypoints',{}).get('all_statuses',[])
        if (record.get('run_id')!=run or record.get('parent_UUID')!=result['action_uuid']
                or proof.get('navigate_to_pose_active')!=[] or proof.get('follow_waypoints_active')!=[]
                or not any(s.get('uuid')==uuid.UUID(result['action_uuid']).hex and s.get('status')==result['status'] for s in statuses)):
            raise Hold('Historical parent/child terminal proof mismatch')
        owned=closed.get('owned_processes_closed',[])
        required={'recorder','controller_server','planner_server','bt_navigator','waypoint_follower','velocity_smoother','nav_safety_gate'}
        if not any(required<=r.get('returncodes',{}).keys() and all(v==0 for v in r['returncodes'].values()) for r in owned):
            raise Hold('Historical owned server closure incomplete')
        argv=record['target']['argv']
        def arg(name):return argv[argv.index(name)+1]
        map_input=manifest['original_map']
        if arg('--map')!=map_input['yaml_path']:raise Hold('Historical closure used another map path')
        import yaml
        map_path=Path(map_input['yaml_path']);image_path=Path(map_input['image_path'])
        if digest(map_path.read_bytes())!=map_input['yaml_sha256'] or digest(image_path.read_bytes())!=map_input['image_sha256']:
            raise Hold('Historical original map artifacts differ')
        doc=yaml.safe_load(map_path.read_text())
        if (map_path.parent/doc['image']).resolve()!=image_path.resolve():raise Hold('Historical map image binding differs')
        doc['image']=map_input['image_sha256']
        if digest(doc)!=identity['map_contract_sha256']:raise Hold('Historical map contract differs from mission')
        if digest(dict(footprint=json.loads(arg('--footprint')),padding=float(arg('--padding'))))!=identity['footprint_sha256']:
            raise Hold('Historical full footprint differs from mission')
        return
    if (closed.get('schema')!='matrix.mission.closed_runtime.v1' or closed.get('action_uuid')!=result['action_uuid']
            or closed.get('parent_terminal_status')!=result['status'] or closed.get('all_owned_processes_terminated') is not True
            or closed.get('recorder_closed') is not True or closed.get('checked_unix_ns',0)<result['finished_ns']):
        raise Hold('Historical runtime closure does not prove this Action and recorder closed')


class Session:
    def __init__(self, store, request_id, output):
        self.store, self.request_id, self.output = store, request_id, output
        state = store.state()
        self.start_index = state['progress']['next_index']
        self.identity_epoch_id = state['identity_epoch_id']
        self.action_uuid = None
        self.send_invoked=False
        self.pending_visits=deque();self.last_visit_key=None;self.storage_failure=None

    def ordered(self, route):
        prefix = [copy.deepcopy(r['visit']) for r in self.store.state()['progress']['accepted_receipts']]
        return OrderedVisits(route, start_index=self.start_index, accepted_visits=prefix, advancing_stop=True)

    def validate_ready(self, ready, *, require_budget=True):
        capability = ready.get('waypoint_contract', {}).get('mission_resume', {})
        if capability.get('schema') != 'matrix.mission.resume_capability.v1' or not ready.get('recovery_runtime', {}).get('verified'):
            raise Hold('Runtime lacks verified measured-stop/mission resume capability')
        expected = {name:digest((HERE/name).read_bytes()) for name in CLIENT_SOURCES}
        if capability.get('client_sources_sha256') != expected:
            raise Hold('Loaded mission client differs from runtime source binding')
        state=(self.store.guard_state() if hasattr(self.store,'guard_state') else self.store.state())
        if capability.get('bt_retries')!=state['policy']['bt_retries']:
            raise Hold('Runtime BT retry limit differs from mission policy')
        actual=ready.get('mission_identity',{})
        if actual.get('verified') is not True:raise Hold('Unverified runtime mission identity')
        check_identity_state(state,actual)
        if ready.get('variant') != state['variant']:
            raise Hold('Runtime variant differs from mission')
        if require_budget and remaining_state(state) < 1:
            raise Hold('FAILED_BUDGET: mission deadline expired')

    def reserve(self):
        try:
            if hasattr(self.store,'call'):
                self.store.call('reserve',self.request_id,self.output,output=self.output.parent)
            else:
                self.store.reserve(self.request_id,self.output);self.store.export(self.output.parent)
        finally:
            # Even export/pump failure after COMMIT retains this operation's UUID.
            attempt=self.store.state().get('attempt') or {}
            if attempt.get('request_id')==self.request_id and attempt.get('output')==str(self.output.resolve()):
                self.action_uuid=uuid.UUID(attempt['parent_action_uuid'])
        return self.action_uuid

    def acknowledge(self, accepted):
        if hasattr(self.store,'call'):
            self.store.call('acknowledge',str(self.action_uuid),accepted,output=self.output.parent)
        else:
            self.store.acknowledge(str(self.action_uuid),accepted);self.store.export(self.output.parent)
        self.drain()

    def observe(self, ordered, *, defer=False):
        if not self.action_uuid or ordered.active is None:return
        key=(ordered.active,tuple((v['index'],digest(v['passed'])) for v in ordered.visits if v.get('passed') is not None))
        if key!=self.last_visit_key:
            if len(self.pending_visits)>=64:raise Hold('Critical visit storage queue overflow')
            self.pending_visits.append(copy.deepcopy(ordered));self.last_visit_key=key
        if not defer:self.drain()

    def drain(self):
        if self.storage_failure is not None:raise Hold('Prior storage checkpoint failed') from self.storage_failure
        while self.pending_visits:
            ordered=self.pending_visits[0]
            try:
                if hasattr(self.store,'call'):
                    self.store.call('observe',str(self.action_uuid),ordered,output=self.output.parent,export_if_changed=True)
                elif self.store.observe(str(self.action_uuid),ordered):self.store.export(self.output.parent)
            except BaseException as error:self.storage_failure=error;raise
            self.pending_visits.popleft()


def runtime_args(args, output, state):
    values=dict(execute=True,ready=Path(args.ready),output=output,route=Path(args.route) if args.route else None,
        case_id='mission_'+state['mission_id'],variant=state['variant'],kind='benchmark',
        observe_seconds=8.,preflight_seconds=40.,stable_preflight_seconds=10.,timeout=1.,settle_seconds=8.,
        cancel_after=None,odom_topic=args.odom_topic,scan_topic=args.scan_topic,
        simulation_first_leg_max_distance_m=args.simulation_first_leg_max_distance_m,
        simulation_scan_max_age_sec=args.simulation_scan_max_age_sec,scan_aging_confirm_sec=0.,
        camera_topic=args.camera_topic,camera_info_topic=args.camera_info_topic,stop_signals=[],goal=None,relative_goal=None)
    return argparse.Namespace(**values)


def run_attempt(args, store, output, reconcile=False):
    # Lazy imports keep inspect/import and offline failure tests independent of ROS.
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from nav2_msgs.action import FollowWaypoints
    from unique_identifier_msgs.msg import UUID
    from waypoint_batch import WaypointPilot, freeze_sources
    from waypoint_contract import OwnedGoalIdentity

    if os.environ.get('ROS_DOMAIN_ID') != '185':
        raise Hold('Only isolated Matrix domain185 is supported')
    output.mkdir(parents=True,exist_ok=False)
    state=store.state(); options=runtime_args(args,output,state)
    # An exhausted motion budget must never disable exact-UUID cancellation.
    options.timeout=10. if reconcile else min(float(state['policy']['mission_budget_sec']),store.remaining())
    if options.timeout <= 0:raise Hold('FAILED_BUDGET')
    source_manifest=freeze_sources(output,args.simulation_scan_max_age_sec)
    for name in CLIENT_SOURCES:
        (output/('source_'+name)).write_bytes((HERE/name).read_bytes())
    session=Session(store,args.request_id,output)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=None; result={}; children=False
    old={n:signal.signal(n,lambda n,f:options.stop_signals.append(n)) for n in (signal.SIGINT,signal.SIGTERM)}
    try:
        node=WaypointPilot(options,output,state['route'],source_manifest,mission=session)
        if hasattr(store,'pump'):store.pump=getattr(node,'pump_storage_wait',None)
        if callable(getattr(node,'emit',None)):
            store.timing_sink=lambda sample:node.emit('mission_persistence_timing',request_id=args.request_id,**sample)
        if reconcile:
            attempt=state.get('attempt')
            if not attempt:raise Hold('No dispatch to reconcile')
            action_uuid=uuid.UUID(attempt['parent_action_uuid'])
            node.owned_identity=OwnedGoalIdentity(action_uuid.bytes)
            node.goal_sent_ns=attempt['created_ns'];node.goal_sent_steady=time.monotonic()
            session.action_uuid=action_uuid
            node.cancel_own('explicit_mission_reconciliation')
            client=node.create_client(FollowWaypoints.Impl.GetResultService,'/follow_waypoints/_action/get_result')
            request=FollowWaypoints.Impl.GetResultService.Request();request.goal_id=UUID(uuid=list(action_uuid.bytes))
            if not client.wait_for_service(timeout_sec=2.):raise Hold('Exact UUID result service unavailable')
            future=client.call_async(request)
            if not node.spin_until(future.done,10.):raise Hold('Exact UUID terminal result remains unknown')
            response=future.result()
            result=dict(action_uuid=str(action_uuid),status=int(response.status),error_code=int(response.result.error_code),
                error_msg=response.result.error_msg,missed_waypoints=[dict(index=int(v.index),error_code=int(v.error_code))
                    for v in response.result.missed_waypoints],functional_status='FAIL',reconciliation=True)
            if response.status in (4,5,6):node.action_result=response
        else:
            result=node.run_batch()
        # A parent result/cancel ACK alone is not child quiescence. Use fresh owned graph observation.
        until=time.monotonic()+3.
        while time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.02)
            try:
                node.execution_ready(phase='settle')
                ownership=node.latest_ready.get('waypoint_ownership',{})
                children=(not ownership.get('navigate_to_pose_active') and not ownership.get('follow_waypoints_active')
                    and not ownership.get('follow_path_active') and ownership.get('follow_path_status_publishers_match') is True
                    and ownership.get('exclusive_nav_client') is True
                    and -.05 <= (time.time_ns()-ownership.get('checked_unix_ns',0))/1e9 <= .5)
                if children:break
            except Exception:
                children=False
    except Exception as error:
        result.update(functional_status='FAIL',exception_type=type(error).__name__,exception_message=str(error))
    finally:
        try:
            if node is not None:
                result['owned_goal_shutdown']=node.finalize_own_goal()
                try:session.drain()
                except Exception as error:
                    result['storage_checkpoint_error']=dict(type=type(error).__name__,message=str(error))
                if hasattr(store,'pump'):store.pump=None
                result['ordered']=node.ordered.summary()
                result['final_measured_and_commanded_stop']=bool(node.stop_evidence and node.stop_evidence.get('observed'))
                if result.get('status')==4 and result.get('error_code')==0 and result.get('missed_waypoints')==[]:
                    try:result['current_final_goal_observation']=node.current_final_goal_observation()
                    except Exception as error:
                        result['current_final_goal_observation']=dict(healthy=False,error_type=type(error).__name__,error_message=str(error))
                store.flush_state_read_timing('attempt_finalization')
                store.timing_sink=None  # Evidence writer closes below; terminal DB work follows it.
                result['mission_persistence_timing_drops']=store.timing_drops
                result['evidence_writer']=node.finish_evidence()
                result['camera_recording_quality']=result['evidence_writer'].get('camera_recording_quality')
                result['strict_scan_timing_status']='FAIL' if (node.strict_scan_latency_first_fault or node.scan_aging_episodes) else 'NO_STRICT_SCAN_FAULT_RECORDED'
                result['children_quiescent']=children
                result['goals_sent']=int(session.send_invoked and not reconcile)
                result['send_goal_async_invoked']=bool(session.send_invoked)
                if session.action_uuid and not reconcile and not session.send_invoked:
                    result.update(status=None,error_code=None,missed_waypoints=None,
                        dispatch_state='NOT_SENT_AFTER_INTENT')
                result['stop_signals']=options.stop_signals
                result['operator_cancel_requested']=args.operation=='cancel'
                result['finished_ns']=time.time_ns()
                events=getattr(node,'recovery_handoffs',[])
                acquired={e['epoch'] for e in events if e.get('accepted') and e.get('operation')=='ACQUIRE'}
                released={e['epoch'] for e in events if e.get('accepted') and e.get('operation')=='RELEASE'}
                entered=sum(v.get('entered') is not None for v in node.ordered.visits[session.start_index:])
                result['bt_recovery_accounting']=dict(acquired_epochs=sorted(acquired),released_epochs=sorted(released),entered_indices=entered)
                # Missing native event evidence cannot support a zero-recovery claim.
                result['bt_recovery_used']=not (entered>0 and len(acquired)==entered and acquired==released
                    and result.get('functional_status')=='PASS' and result['evidence_writer'].get('evidence_complete'))
                if session.action_uuid:
                    result.setdefault('action_uuid',str(session.action_uuid))
                atomic_json(output/'native_result.json',json.loads(evidence_json(result)))
                if session.action_uuid:
                    if not reconcile and not session.send_invoked:
                        store.record_not_sent(str(session.action_uuid),result_path=output/'native_result.json',send_invoked=False)
                    elif not result.get('storage_checkpoint_error'):
                        store.finish_attempt(result,children,result_path=output/'native_result.json')
        finally:
            try:
                if node is not None:node.destroy_node()
            finally:
                if rclpy.ok():rclpy.shutdown()
                for n,handler in old.items():signal.signal(n,handler)
    return result


def parser():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('operation',choices=['inspect','start','import-closed-attempt','resume','reconcile','cancel','authorize-continuation'])
    p.add_argument('--store',type=Path,required=True);p.add_argument('--mission-id')
    p.add_argument('--route',type=Path);p.add_argument('--identity',type=Path);p.add_argument('--policy',type=Path)
    p.add_argument('--closed-attempt-manifest',type=Path);p.add_argument('--expect-revision',type=int)
    p.add_argument('--authorization',type=Path);p.add_argument('--authorization-sha256')
    p.add_argument('--request-id');p.add_argument('--workflow-run-id');p.add_argument('--output',type=Path)
    p.add_argument('--ready',type=Path);p.add_argument('--execute',action='store_true');p.add_argument('--json',action='store_true')
    p.add_argument('--resource-lock',type=Path,default=Path('/tmp/matrix-domain185-mission.lock'))
    p.add_argument('--reason',help='Explicit reviewed retry reason; never supplied by automatic recovery')
    p.add_argument('--odom-topic',default='/odom/current_pose');p.add_argument('--scan-topic',default='/laser_scan')
    p.add_argument('--simulation-scan-max-age-sec',type=float,choices=[.3,.5],default=.3)
    p.add_argument('--simulation-first-leg-max-distance-m',type=float,choices=[8.,8.5],default=8.)
    p.add_argument('--camera-topic',default='/camera/front/image/compressed');p.add_argument('--camera-info-topic',default='/camera/front/camera_info')
    return p


def main(argv=None):
    args=parser().parse_args(argv)
    if args.operation=='authorize-continuation':
        from mission_continuation import main as authorize
        return authorize(args)
    if args.operation=='inspect':
        state=MissionStore.inspect(args.store)
        if args.mission_id and args.mission_id!=state['mission_id']:raise Hold('Mission identity differs')
        print(encoded_output(state));return 0
    if not args.output or not args.request_id:raise ValueError('--output and --request-id required')
    uuid.UUID(args.request_id)
    if not args.execute:
        if args.operation in ('start','import-closed-attempt'):
            if not args.route or not args.identity or not args.policy:raise ValueError('route, identity, policy required')
            validate_route(json.loads(args.route.read_text()));validate_identity(json.loads(args.identity.read_text()))
            policy=json.loads(args.policy.read_text())
            if policy.get('mission_budget_sec')!=json.loads(args.route.read_text())['mission_budget_sec']:
                raise ValueError('Policy budget differs from route')
        else:
            current=MissionStore.inspect(args.store)
            if args.mission_id!=current['mission_id'] or args.expect_revision!=current['revision']:raise Hold('Mission identity or expected revision mismatch')
        print(encoded_output(dict(schema='matrix.mission.operation.v1',operation=args.operation,
            operation_outcome='VALIDATED_ONLY',goals_sent=0,reason='--execute absent; no store opened or reservation made')))
        return 0
    args.output=args.output.resolve()
    if args.operation in ('start','resume','reconcile','cancel'):
        from mission_storage import StorageWorker
        store=StorageWorker(args.store,args.resource_lock)
    else:store=MissionStore(args.store,args.resource_lock)
    try:
        arguments={k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()}
        arguments['input_sha256']={key:digest(getattr(args,key).read_bytes()) for key in
            ('route','identity','policy','closed_attempt_manifest') if getattr(args,key) is not None}
        previous=store.operation(args.request_id,arguments)
        if previous is not None:
            print(encoded_output(previous));return previous.get('exit_code',10)
        args.output.mkdir(parents=True,exist_ok=False)
        identity=json.loads(args.identity.read_text()) if args.identity else None
        if args.operation in ('start','import-closed-attempt'):
            if not args.route or not identity or not args.policy:raise ValueError('route, identity, policy required')
            state=store.create(args.route.read_bytes(),identity,json.loads(args.policy.read_text()),args.mission_id)
        else:
            state=store.state()
            if args.mission_id!=state['mission_id'] or args.expect_revision!=state['revision']:
                raise Hold('Mission identity or expected revision mismatch')
            if identity is not None:store.check_identity(identity)
        if args.operation=='import-closed-attempt':
            manifest=json.loads(args.closed_attempt_manifest.read_text())
            if manifest.get('schema')!='matrix.mission.closed_attempt.v1':raise Hold('Typed historical manifest required')
            for key in ('map_contract_sha256','footprint_sha256','acceptance_policy_sha256'):
                if manifest.get(key)!=state['identity'][key]:raise Hold('Historical '+key+' differs')
            if manifest.get('authorized_new_source_sha')!=state['identity']['source_sha']:
                raise Hold('Historical import requires explicit new-code identity')
            raw=Path(manifest['result_path']).read_bytes()
            if digest(raw)!=manifest['result_sha256']:raise Hold('Historical result hash mismatch')
            result=json.loads(raw)
            closure=manifest['closure'];closed_raw=Path(closure['path']).read_bytes()
            if digest(closed_raw)!=closure['sha256']:raise Hold('Historical closure hash mismatch')
            closed=json.loads(closed_raw)
            validate_closed_runtime(closed,result,manifest,state['identity'])
            for item in manifest.get('evidence_files',[]):
                if digest(Path(item['path']).read_bytes())!=item['sha256']:raise Hold('Historical evidence hash mismatch')
            if not manifest.get('evidence_files'):raise Hold('Closed lifecycle evidence required')
            store.import_closed(result,digest(raw),manifest['historical_bt_retries']);outcome='IMPORTED';code=0
        else:
            if not args.ready:raise ValueError('--ready required for ROS operations')
            if args.operation=='cancel':store.request_cancel(args.request_id,args.reason)
            attempt=store.state().get('attempt')
            if attempt and attempt.get('dispatch_state')=='NOT_SENT_AFTER_INTENT':
                raise Hold('NOT_SENT_AFTER_INTENT: no automatic cross-operation dispatch')
            if attempt and not attempt.get('terminal_confirmed'):
                run_attempt(args,store,args.output/'reconcile',reconcile=True)
                attempt=store.state()['attempt']
                if not attempt.get('terminal_confirmed'):raise Hold('UNKNOWN_DISPATCH')
            if args.operation in ('reconcile','cancel'):
                if attempt and not (attempt.get('children_quiescent') and attempt.get('stopped')):
                    run_attempt(args,store,args.output/'settle',reconcile=True)
                settled_attempt=store.state().get('attempt') or {}
                settled=all(settled_attempt.get(key) for key in
                    ('terminal_confirmed','children_quiescent','stopped'))
                outcome='CANCELED' if settled else 'HELD'
                code=10 if outcome=='HELD' else 11
            else:
                outcome='HELD';code=10
                control=store.state().get('retry_control') or {}
                paused=bool(control.get('paused'))
                if paused and args.operation=='resume' and args.reason:
                    store.review_retry(args.request_id,args.reason)
                    paused=False
                if paused:outcome='NEEDS_TASK_DECISION';code=14
                for number in range(0 if paused else 10):
                    if store.remaining()<=0:raise Hold('FAILED_BUDGET')
                    result=run_attempt(args,store,args.output/('attempt_%02d'%number))
                    s=store.state()
                    if s['result_flags']['operational_complete']:outcome='COMPLETED';code=0;break
                    if s['lifecycle']=='NEEDS_TASK_DECISION':outcome='NEEDS_TASK_DECISION';code=14;break
                    if s['lifecycle']=='CANCELED':outcome='CANCELED';code=11;break
                    if s['lifecycle']=='HOLD':break
                    if result.get('cancel_reason') in ('scheduled_cancel','pilot_interrupted') or result.get('stop_signals'):
                        outcome='CANCELED';code=11;break
                    a=s.get('attempt') or {}
                    if not (a.get('terminal_confirmed') and a.get('children_quiescent') and a.get('stopped')):break
                    # The next attempt's full stable preflight is the bounded health wait.
                    if result.get('goals_sent')==0 or not a.get('parent_action_uuid'):break
        state=store.export(args.output)
        response=dict(schema='matrix.mission.operation.v1',operation=args.operation,request_id=args.request_id,
            workflow_run_id=args.workflow_run_id,operation_outcome=outcome,exit_code=code,
            mission_id=state['mission_id'],revision=state['revision'],attempt_id=(state.get('attempt') or {}).get('attempt_id'),
            parent_action_uuid=(state.get('attempt') or {}).get('parent_action_uuid'),state_path=str(args.output/'state.json'),
            state_sha256=digest((args.output/'state.json').read_bytes()),result_path=str(args.output/'result.json'),
            result_sha256=digest((args.output/'result.json').read_bytes()),reason=state['hold_reason'],result_flags=state['result_flags'])
        store.finish_operation(args.request_id,response);atomic_json(args.output/'operation.json',response)
        print(encoded_output(response));return code
    finally:
        store.close()


def encoded_output(value):
    return json.dumps(value,ensure_ascii=False,allow_nan=False)


if __name__=='__main__':
    try:
        raise SystemExit(main())
    except (Hold,ValueError,OSError) as error:
        print(encoded_output(dict(schema='matrix.mission.operation.v1',operation_outcome='HELD',reason=str(error))))
        raise SystemExit(12 if 'FAILED_BUDGET' in str(error) else 13 if 'IDENTITY_MISMATCH' in str(error) else 10)

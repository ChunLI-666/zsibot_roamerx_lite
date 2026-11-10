"""Transactional mission authority. No ROS imports and no motion side effects."""
from contextlib import contextmanager
import copy
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import sqlite3
import resource
import time
import uuid

from waypoint_contract import validate_route, strict_stopped, angular_difference, evidence_json


class Hold(RuntimeError):
    pass


def encoded(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(',', ':'), allow_nan=False)


def digest(value):
    return hashlib.sha256(value if isinstance(value, bytes) else encoded(value).encode()).hexdigest()


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_name(path.name+'.'+str(uuid.uuid4())+'.tmp')
    with temporary.open('x') as stream:
        stream.write(encoded(value)+'\n'); stream.flush(); os.fsync(stream.fileno())
    os.replace(temporary, path)
    descriptor = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def boot_clock():
    return Path('/proc/sys/kernel/random/boot_id').read_text().strip(), time.clock_gettime(time.CLOCK_BOOTTIME)


IDENTITY_KEYS = ('map_contract_sha256', 'footprint_sha256', 'acceptance_policy_sha256')


def validate_identity(identity):
    for key in (*IDENTITY_KEYS, 'effective_config_sha256'):
        value = identity.get(key)
        if not isinstance(value, str) or len(value) != 64 or any(c not in '0123456789abcdef' for c in value):
            raise ValueError('Missing SHA256 identity: '+key)
    if identity.get('variant') not in ('official', 'segmented'):
        raise ValueError('Mission requires official or segmented identity')
    value = identity.get('source_sha', '')
    if len(value) != 40 or any(c not in '0123456789abcdef' for c in value):
        raise ValueError('Full source commit required')


def validate_visit(route, index, visit, legacy=False):
    if visit.get('index') != index or visit.get('id') != route[index]['id'] or visit.get('original_goal_map') != route[index]['pose']:
        raise Hold('Visit identity does not match exact original index')
    passed = visit.get('passed') or {}
    snapshot = passed.get('snapshot', {})
    pose = snapshot.get('last', {}).get('pose', {})
    goal = route[index]['pose']
    if (pose.get('frame_id') != 'map' or not all(math.isfinite(pose.get(k, math.nan)) for k in ('x','y','yaw'))
            or pose.get('receive_age_s', math.inf) > .3 or not -.05 <= pose.get('header_age_s', math.inf) <= .3
            or math.hypot(pose['x']-goal[0], pose['y']-goal[1]) > .15
            or abs(angular_difference(pose['yaw'], goal[2])) > .08
            or passed.get('continuous_stopped_s', 0) < .3 or not strict_stopped(snapshot)[0]):
        raise Hold('Original geometry/fresh measured stopped visit not proven')
    if not legacy and not passed.get('advancing_odom_dwell'):
        raise Hold('New visit lacks post-zero advancing original Odom dwell')


def failure_decision(result, terminal, quiescent, stopped):
    """Executed by finish_attempt and consumed by the real CLI retry loop."""
    if not terminal:return 'HOLD','UNKNOWN_DISPATCH'
    if not quiescent or not stopped:return 'HOLD','CHILD_OR_STOP_UNPROVEN'
    reason=result.get('cancel_reason') or ''
    if result.get('stop_signals') or reason in ('scheduled_cancel','pilot_interrupted'):
        return 'CANCELED','OPERATOR_CANCEL'
    codes={v.get('error_code') for v in result.get('missed_waypoints') or []}
    if reason.startswith('input_became_invalid:') or codes & {102,202}:
        return 'PAUSED_INPUT_HEALTH','FRESH_STABLE_HEALTH_REQUIRED'
    if codes & {203,204,208}:
        return 'NEEDS_TASK_DECISION','NO_FEASIBLE_ORIGINAL_GOAL_PATH'
    if codes & {101,201}:return 'NEEDS_TASK_DECISION','INVALID_RUNTIME_CONFIGURATION'
    if codes & {205,206}:return 'RECOVERABLE_FAILURE','BOUNDED_OBSTACLE_WAIT'
    if codes & {104,105,106,107,207}:return 'RECOVERABLE_FAILURE','BOUNDED_ORIGINAL_GOAL_REPLAN'
    return 'NEEDS_TASK_DECISION','UNCLASSIFIED_TERMINAL_OR_INDEPENDENT_EVIDENCE_FAILURE'


class MissionStore:
    """One mission per local SQLite DB, with DB and shared robot/domain locks."""
    def __init__(self, path, resource=None):
        self.path = Path(path).resolve()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.locks = []
        self.timing_sink=None
        self.timing_drops=0
        self.state_read_timing={}
        try:
            # Caller supplies a stable resource lock shared across A/B missions.
            for lock in ([Path(resource)] if resource else [])+[self.path.with_suffix('.lock')]:
                lock.parent.mkdir(parents=True, exist_ok=True)
                stream = lock.open('a+')
                self.locks.append(stream)
                fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.db = sqlite3.connect(self.path, isolation_level=None, timeout=0)
            self.db.execute('PRAGMA synchronous=FULL')
            self.db.execute('PRAGMA journal_mode=DELETE')
            self.db.executescript('''
                CREATE TABLE IF NOT EXISTS mission (id INTEGER PRIMARY KEY CHECK(id=1), revision INTEGER NOT NULL, state TEXT NOT NULL);
                CREATE TABLE IF NOT EXISTS events (seq INTEGER PRIMARY KEY, revision INTEGER NOT NULL, kind TEXT NOT NULL, payload TEXT NOT NULL);
                CREATE TABLE IF NOT EXISTS operations (request_id TEXT PRIMARY KEY, arguments_sha256 TEXT NOT NULL, result TEXT);
                CREATE TABLE IF NOT EXISTS continuations (authorization_id TEXT PRIMARY KEY, authorization_sha256 TEXT NOT NULL, receipt TEXT NOT NULL);
            ''')
        except BaseException:
            self.close(); raise

    def close(self):
        if hasattr(self, 'db'):
            self.db.close(); del self.db
        for stream in self.locks:
            stream.close()
        self.locks = []

    @staticmethod
    def inspect(path):
        db = sqlite3.connect(Path(path).resolve().as_uri()+'?mode=ro', uri=True)
        try:
            row = db.execute('SELECT state FROM mission WHERE id=1').fetchone()
            if row is None:
                raise Hold('Mission has not been created')
            return json.loads(row[0])
        finally:
            db.close()

    def state(self):
        observed=self.timing_sink is not None
        if observed:start=time.monotonic_ns();cpu=time.thread_time_ns()
        row = self.db.execute('SELECT state FROM mission WHERE id=1').fetchone()
        if observed:selected=time.monotonic_ns();selected_cpu=time.thread_time_ns()
        if row is None:
            raise Hold('Mission has not been created')
        state=json.loads(row[0])
        if observed:
            ended=time.monotonic_ns();ended_cpu=time.thread_time_ns()
            for phase,wall,used in [('select',selected-start,selected_cpu-cpu),
                                   ('json_decode',ended-selected,ended_cpu-selected_cpu)]:
                item=self.state_read_timing.setdefault(phase,dict(count=0,total_wall_ns=0,max_wall_ns=0,
                    total_thread_cpu_ns=0,max_thread_cpu_ns=0,first_steady_ns=start,last_steady_ns=ended))
                item.update(count=item['count']+1,total_wall_ns=item['total_wall_ns']+wall,
                    max_wall_ns=max(item['max_wall_ns'],wall),total_thread_cpu_ns=item['total_thread_cpu_ns']+used,
                    max_thread_cpu_ns=max(item['max_thread_cpu_ns'],used),last_steady_ns=ended)
        return state

    def flush_state_read_timing(self,checkpoint):
        if self.timing_sink is None or not self.state_read_timing:return
        sample=dict(stage='state_reads:aggregate',checkpoint=checkpoint,success=True,
            phases=self.state_read_timing,emitted_unix_ns=time.time_ns(),
            boundary='Caller-thread SELECT/fetch and JSON decode, aggregated in memory; no per-read event or fsync')
        self.state_read_timing={}
        try:self.timing_sink(sample)
        except Exception:self.timing_drops+=1

    @contextmanager
    def timing(self,stage):
        """Observe caller blocking; never weaken durability or change transaction outcome."""
        sink=self.timing_sink
        if sink is None:
            yield;return
        start=time.monotonic_ns();unix=time.time_ns();cpu=time.thread_time_ns()
        usage=resource.getrusage(resource.RUSAGE_THREAD);success=False
        try:
            yield;success=True
        finally:
            end=time.monotonic_ns();after=resource.getrusage(resource.RUSAGE_THREAD)
            sample=dict(stage=stage,started_unix_ns=unix,finished_unix_ns=time.time_ns(),
                started_steady_ns=start,finished_steady_ns=end,wall_ns=end-start,
                thread_cpu_ns=time.thread_time_ns()-cpu,success=success,
                thread_inblock_delta=after.ru_inblock-usage.ru_inblock,thread_oublock_delta=after.ru_oublock-usage.ru_oublock,
                voluntary_context_switches=after.ru_nvcsw-usage.ru_nvcsw,
                involuntary_context_switches=after.ru_nivcsw-usage.ru_nivcsw,
                io_boundary='getrusage thread block-operation counters; not bytes or proof of device latency')
            try:sink(sample)
            except Exception:self.timing_drops+=1
            if stage.endswith(':commit') or stage.startswith('export_result:'):
                self.flush_state_read_timing(stage)

    def create(self, route_raw, identity, policy, mission_id=None):
        validate_identity(identity)
        document = json.loads(route_raw); route = validate_route(document)
        if policy.get('mission_budget_sec') != document['mission_budget_sec']:
            raise ValueError('Policy must preserve the exact route mission budget')
        if any(type(policy.get(k)) is not int for k in ('max_parent_dispatches','max_attempts_per_index','bt_retries')) or policy.get('max_parent_dispatches') not in range(1, 11) or policy.get('max_attempts_per_index') not in range(1, 7):
            raise ValueError('Finite limits required (parent dispatches<=10, child attempts per index<=6)')
        if policy.get('bt_retries') not in (0, 1):
            raise ValueError('Only zero or one native BT retry is supported')
        if policy.get('max_parent_dispatches_per_index',3)!=3:
            raise ValueError('Per-index parent dispatch cap must remain three')
        ident = dict(identity, route_raw_sha256=digest(route_raw), route_semantic_sha256=digest(route))
        boot, now = boot_clock()
        state = dict(schema='matrix.mission.state.v1', mission_id=str(uuid.UUID(mission_id)) if mission_id else str(uuid.uuid4()),
            revision=0, identity_epoch_id=str(uuid.uuid4()), identity=ident, variant=identity['variant'],
            lifecycle='NEW', hold_reason=None, route=route, route_size=len(route), policy=policy,
            progress=dict(accepted_prefix_count=0,next_index=0,active_index=None,last_failed_index=None,accepted_receipts=[]),
            current_target=route[0], attempt=None, attempts=[], candidates={},
            budgets=dict(mission_dispatches_reserved=0,per_index_reserved=[0]*len(route),
                per_index_parent_dispatches=[0]*len(route),reservation_unit='child_attempt_including_native_BT_retries',
                deadline_boot_id=None,deadline_boottime_sec=None,remaining_sec=policy['mission_budget_sec']),
            result_flags=dict(operational_complete=False,recovery_used=False,single_run_complete=False,
                strict_timing_status='NOT_EVALUATED',simulation_acceptance=False,physical_acceptance=False),
            stop=dict(state='UNPROVEN'), evidence={}, updated=dict(unix_ns=time.time_ns(),boot_id=boot,boottime_sec=now))
        self.db.execute('INSERT INTO mission VALUES (1,0,?)', (encoded(state),))
        return state

    @contextmanager
    def transaction(self, kind, expected=None, before_commit=None):
        with self.timing(kind+':begin'):self.db.execute('BEGIN IMMEDIATE')
        try:
            state = self.state()
            if expected is not None and expected != state['revision']:
                raise Hold('STALE_REVISION')
            yield state
            state['revision'] += 1
            boot, now = boot_clock()
            state['updated'] = dict(unix_ns=time.time_ns(),boot_id=boot,boottime_sec=now)
            if state['budgets']['deadline_boot_id']==boot and state['budgets']['deadline_boottime_sec'] is not None:
                state['budgets']['remaining_sec']=max(0.,state['budgets']['deadline_boottime_sec']-now)
            with self.timing(kind+':encode_and_write'):
                self.db.execute('UPDATE mission SET revision=?,state=? WHERE id=1', (state['revision'],encoded(state)))
                self.db.execute('INSERT INTO events(revision,kind,payload) VALUES (?,?,?)',
                    (state['revision'],kind,encoded(dict(attempt_id=(state.get('attempt') or {}).get('attempt_id'),
                                                      progress=state['progress']['accepted_prefix_count'],time_ns=time.time_ns()))))
            if before_commit is not None:before_commit(state)
            with self.timing(kind+':commit'):self.db.execute('COMMIT')
        except BaseException:
            self.db.execute('ROLLBACK'); raise

    def operation(self, request_id, arguments):
        request_id = str(uuid.UUID(request_id)); sha = digest(arguments)
        row = self.db.execute('SELECT arguments_sha256,result FROM operations WHERE request_id=?',(request_id,)).fetchone()
        if row:
            if row[0] != sha:
                raise Hold('Operation id reused with different arguments')
            return json.loads(row[1]) if row[1] else dict(operation_outcome='HELD',reason='operation interrupted; reconcile, never redispatch')
        self.db.execute('INSERT INTO operations VALUES (?,?,NULL)',(request_id,sha))
        return None

    def finish_operation(self, request_id, result):
        self.db.execute('UPDATE operations SET result=? WHERE request_id=?',(encoded(result),request_id))

    def check_identity(self, identity):
        validate_identity(identity); state = self.state()
        for key in (*IDENTITY_KEYS, 'variant', 'source_sha', 'effective_config_sha256'):
            if identity.get(key) != state['identity'].get(key):
                raise Hold('IDENTITY_MISMATCH:'+key)

    def authorize_continuation(self, authorization, authorization_sha256, ready_path):
        """One atomic, idempotent authority change; never sends or reserves a goal."""
        from mission_continuation import apply_authorization
        key=authorization['authorization_id']
        row=self.db.execute('SELECT authorization_sha256,receipt FROM continuations WHERE authorization_id=?',(key,)).fetchone()
        if row:
            if row[0]!=authorization_sha256:raise Hold('Continuation authorization id reused with different bytes')
            return json.loads(row[1])
        receipt={}
        def persist(state):
            if state['budgets']['remaining_sec']<=0:raise Hold('FAILED_BUDGET at continuation commit')
            receipt.update(after_revision=state['revision'],after_state_sha256=digest(state),state=copy.deepcopy(state))
            self.db.execute('INSERT INTO continuations VALUES (?,?,?)',(key,authorization_sha256,encoded(receipt)))
        with self.transaction('CONTINUATION_AUTHORIZED',authorization['expected_revision'],before_commit=persist) as state:
            receipt.update(schema='matrix.mission.continuation_receipt.v1',authorization_id=key,
                authorization_sha256=authorization_sha256,before_revision=state['revision'],before_state_sha256=digest(state),goals_sent=0)
            receipt['transition']=apply_authorization(state,authorization,authorization_sha256,ready_path)
        return receipt

    def remaining(self):
        s=self.state();boot,now=boot_clock()
        if s['budgets']['deadline_boottime_sec'] is None:
            return s['budgets']['remaining_sec']
        if boot != s['budgets']['deadline_boot_id']:
            raise Hold('CLOCK_UNTRUSTED: mission boot changed')
        return max(0.,s['budgets']['deadline_boottime_sec']-now)

    def reserve(self, request_id, output):
        with self.transaction('SEND_INTENT') as s:
            old=s.get('attempt')
            if old and not old.get('terminal_confirmed'):
                raise Hold('UNKNOWN_DISPATCH: reconcile exact previous UUID first')
            if old and (not old.get('children_quiescent') or not old.get('stopped')):
                raise Hold('Prior attempt lacks child quiescence or measured stop')
            index=s['progress']['next_index']
            if index is None:
                raise Hold('Mission is already complete')
            budget=s['budgets']; policy=s['policy']; boot, now=boot_clock()
            if budget['deadline_boottime_sec'] is None:
                budget['deadline_boot_id']=boot;budget['deadline_boottime_sec']=now+budget['remaining_sec']
            if boot!=budget['deadline_boot_id'] or now>=budget['deadline_boottime_sec']:
                raise Hold('FAILED_BUDGET: deadline exhausted or boot changed')
            child_slots=1+policy['bt_retries']
            if (budget['mission_dispatches_reserved']>=policy['max_parent_dispatches']
                    or any(count>=3 for count in budget['per_index_parent_dispatches'][index:]) or any(
                    count+child_slots>policy['max_attempts_per_index'] for count in budget['per_index_reserved'][index:])):
                raise Hold('FAILED_BUDGET: dispatch allowance exhausted')
            budget['mission_dispatches_reserved']+=1
            for i in range(index,len(s['route'])):
                budget['per_index_reserved'][i]+=child_slots
                budget['per_index_parent_dispatches'][i]+=1
            attempt=dict(attempt_id=str(uuid.uuid4()),parent_action_uuid=str(uuid.uuid4()),request_id=request_id,
                start_index=index,dispatch_state='INTENT_COMMITTED',native_status=None,native_error_code=None,
                missed_waypoints=None,terminal_confirmed=False,children_quiescent=False,stopped=False,
                output=str(Path(output).resolve()),created_ns=time.time_ns(),highest_feedback_index=None)
            attempt['child_slots_reserved']=child_slots
            attempt['identity_epoch_id']=s['identity_epoch_id']
            attempt['source_provenance']=dict(kind='live_execution',source_sha=s['identity']['source_sha'])
            s['attempt']=attempt;s['attempts'].append(copy.deepcopy(attempt))
            s['lifecycle']='DISPATCH_INTENT';s['hold_reason']=None;s['candidates']={}
            s['stop']=dict(state='UNPROVEN');s['progress']['active_index']=index
            if len(s['attempts'])>1: s['result_flags']['recovery_used']=True
        return self.state()['attempt']

    def acknowledge(self, action_uuid, accepted):
        with self.transaction('ACK') as s:
            if s['attempt']['parent_action_uuid']!=str(uuid.UUID(action_uuid)):
                raise Hold('Old action acknowledgement')
            a=s['attempt'];a['dispatch_state']='ACCEPTED' if accepted else 'REJECTED'
            if not accepted:
                a.update(terminal_confirmed=True,children_quiescent=True,stopped=True)
            s['lifecycle']='RUNNING' if accepted else 'RECOVERABLE_FAILURE'

    def observe(self, action_uuid, ordered):
        s=self.state(); a=s.get('attempt') or {}
        if a.get('parent_action_uuid')!=str(uuid.UUID(action_uuid)):
            return False
        active=ordered.active
        if active is None:return False
        changed=a.get('highest_feedback_index')!=active or any(
            v.get('passed') is not None and str(v['index']) not in s['candidates']
            for v in ordered.visits[s['progress']['accepted_prefix_count']:])
        if not changed:return False
        with self.transaction('VISIT_OBSERVATION') as s:
            a=s['attempt'];a['highest_feedback_index']=active;s['progress']['active_index']=active
            for v in ordered.visits[s['progress']['accepted_prefix_count']:]:
                if v.get('passed') is not None:
                    validate_visit(s['route'],v['index'],v)
                    s['candidates'][str(v['index'])]=copy.deepcopy(v)
            self._promote(s,active,'next_native_feedback_under_exclusive_stop_on_failure')
        return True

    @staticmethod
    def _promote(s, until, basis, legacy=False):
        progress=s['progress']
        while progress['accepted_prefix_count']<until:
            index=progress['accepted_prefix_count']; v=s['candidates'].get(str(index))
            if v is None:break
            validate_visit(s['route'],index,v,legacy)
            receipt=dict(index=index,id=s['route'][index]['id'],original_goal_map=s['route'][index]['pose'],
                source_attempt_id=s['attempt']['attempt_id'],source_action_uuid=s['attempt']['parent_action_uuid'],
                native_completion_basis=basis,visit=copy.deepcopy(v),legacy_stop_proof=legacy)
            receipt['sha256']=digest(receipt)
            progress['accepted_receipts'].append(receipt);progress['accepted_prefix_count']+=1
        index=progress['accepted_prefix_count'];progress['next_index']=index if index<len(s['route']) else None
        s['current_target']=s['route'][index] if index<len(s['route']) else None

    def finish_attempt(self, result, children_quiescent, *, result_path):
        with self.transaction('ATTEMPT_TERMINAL') as s:
            a=s['attempt']
            if result.get('action_uuid')!=a['parent_action_uuid']:
                raise Hold('Terminal result does not match owned Action UUID')
            # A reconcile observation has its own output, independent of the dispatch.
            # Bind the exact persisted observation before changing any mission state.
            try:
                native_path=Path(result_path).resolve(strict=True)
                native_raw=native_path.read_bytes()
                persisted=json.loads(native_raw)
            except (OSError,ValueError,TypeError) as error:
                raise Hold('Native result evidence unavailable or invalid') from error
            if (encoded(persisted)!=encoded(json.loads(evidence_json(result)))
                    or type(children_quiescent) is not bool
                    or persisted.get('children_quiescent') is not children_quiescent):
                raise Hold('Native result evidence differs from terminal observation')
            evidence=dict(path=str(native_path),sha256=digest(native_raw))
            history=a.setdefault('native_result_history',[])
            if a.get('native_result') and a['native_result'] not in history:
                history.append(copy.deepcopy(a['native_result']))
            if any(item['path']==evidence['path'] and item['sha256']!=evidence['sha256'] for item in history):
                raise Hold('Native result evidence path was reused with different bytes')
            if evidence not in history:history.append(evidence)
            status=result.get('status');missed=result.get('missed_waypoints')
            a.update(native_status=status,native_error_code=result.get('error_code'),missed_waypoints=missed,
                native_error_msg=result.get('error_msg'),
                terminal_confirmed=status in (4,5,6) or a['dispatch_state']=='REJECTED',children_quiescent=bool(children_quiescent),
                stopped=result.get('final_measured_and_commanded_stop',result.get('settled_measured_and_commanded_stop')) is True)
            a['dispatch_state']='TERMINAL_CONFIRMED' if a['terminal_confirmed'] else 'ACCEPTANCE_UNKNOWN'
            successful=status==4 and result.get('error_code')==0 and missed==[]
            if successful and result.get('functional_status')=='PASS' and result.get('evidence_writer',{}).get('evidence_complete') is True:
                self._promote(s,len(s['route']),'successful_native_parent_and_independent_visits')
            failed=min((v['index'] for v in missed),default=s['progress']['next_index']) if isinstance(missed,list) else s['progress']['next_index']
            if missed and (type(failed) is not int or not a['start_index']<=failed<len(s['route'])):
                raise Hold('Invalid absolute failed waypoint index')
            if not successful:s['progress']['last_failed_index']=failed
            a['failure']=None
            if not successful and failed is not None:
                goal=s['route'][failed];pose=result.get('final_snapshot',{}).get('last',{}).get('pose')
                error=None
                if pose and all(math.isfinite(pose.get(k,math.nan)) for k in ('x','y','yaw')):
                    error=dict(xy_m=math.hypot(pose['x']-goal['pose'][0],pose['y']-goal['pose'][1]),
                        yaw_rad=abs(angular_difference(pose['yaw'],goal['pose'][2])),original_pose_snapshot=pose)
                a['failure']=dict(index=failed,id=goal['id'],original_goal_map=goal['pose'],actual_error=error)
            a['native_result']=evidence
            a['bt_recovery_accounting']=result.get('bt_recovery_accounting')
            s['stop']=dict(state='PASSED' if a['stopped'] else 'UNPROVEN',source_action_uuid=a['parent_action_uuid'],
                receipt=result.get('owned_goal_shutdown',{}).get('stop'),evidence=a['native_result'])
            # Only a terminal failed native batch proves its later suffix was not entered.
            if status==6 and not a.get('budget_reconciled') and isinstance(missed,list) and len(missed)==1 and isinstance(failed,int):
                for i in range(failed+1,len(s['route'])):
                    s['budgets']['per_index_reserved'][i]-=a['child_slots_reserved']
                    s['budgets']['per_index_parent_dispatches'][i]-=1
                a['budget_reconciled']=True
            complete=successful and s['progress']['next_index'] is None and a['stopped'] and a['children_quiescent']
            s['result_flags']['recovery_used'] |= bool(result.get('bt_recovery_used',True))
            s['result_flags']['operational_complete']=complete
            s['result_flags']['single_run_complete']=complete and not s['result_flags']['recovery_used'] and not result.get('bt_recovery_used',True)
            if result.get('strict_scan_timing_status')=='FAIL':s['result_flags']['strict_timing_status']='FAIL'
            camera=result.get('camera_recording_quality') or {}
            if camera.get('status')=='FAIL':
                s['result_flags']['camera_recording_quality_status']='FAIL'
                s['result_flags']['strict_timing_status']='FAIL'
            if complete:
                s['lifecycle']='COMPLETED_WITH_RECOVERY' if s['result_flags']['recovery_used'] else 'COMPLETED_SINGLE_RUN'
                s['hold_reason']=None
            else:
                s['lifecycle'],s['hold_reason']=failure_decision(result,a['terminal_confirmed'],a['children_quiescent'],a['stopped'])
            s['attempts'][-1]=copy.deepcopy(a)

    def import_closed(self, result, result_sha, historical_bt_retries=1):
        if result.get('status') not in (4,5,6) or not result.get('final_measured_and_commanded_stop'):
            raise Hold('Historical attempt is not terminal and stopped')
        with self.transaction('IMPORT_CLOSED') as s:
            if s['attempts'] or result.get('route_sha256')!=s['identity']['route_raw_sha256']:
                raise Hold('Import requires empty mission and identical original route')
            visits=result.get('ordered',{}).get('visits',[])
            if len(visits)!=len(s['route']):raise Hold('Historical route size differs')
            missed=result.get('missed_waypoints')
            if not isinstance(missed,list):raise Hold('Historical missed list missing')
            end=min((v['index'] for v in missed),default=len(visits))
            if not isinstance(end,int) or not 0<=end<=len(visits):raise Hold('Invalid historical failed index')
            for i in range(end):validate_visit(s['route'],i,visits[i],True)
            a=dict(attempt_id=result.get('attempt_uuid',str(uuid.uuid4())),parent_action_uuid=str(uuid.UUID(result['action_uuid'])),
                dispatch_state='TERMINAL_CONFIRMED',terminal_confirmed=True,children_quiescent=True,stopped=True,
                historical_result_sha256=result_sha,start_index=0,native_status=result['status'],missed_waypoints=missed)
            if type(historical_bt_retries) is not int or historical_bt_retries not in (0,1):raise Hold('Historical BT retry count required')
            a['child_slots_reserved']=1+historical_bt_retries
            a['identity_epoch_id']=s['identity_epoch_id']
            a['source_provenance']=dict(kind='imported_closed_result',historical_result_sha256=result_sha,
                epoch_role='import_authority_not_historical_execution_source')
            a['budget_reconciled']=True  # Import already excludes the proven unentered suffix.
            s['attempt']=a;s['attempts']=[copy.deepcopy(a)]
            s['candidates']={str(i):visits[i] for i in range(end)}
            self._promote(s,end,'sealed_historical_native_result_and_stopped_visits',True)
            s['progress']['last_failed_index']=end if end<len(visits) else None
            elapsed=max(0.,(result.get('finished_ns',0)-result.get('created_ns',0))/1e9)
            if not 0<elapsed<s['policy']['mission_budget_sec']:raise Hold('Historical consumed time unavailable or budget exhausted')
            s['budgets']['remaining_sec']-=elapsed
            s['budgets']['historical_debited_sec']=elapsed
            s['budgets']['mission_dispatches_reserved']=1
            s['budgets']['per_index_reserved']=[(1+historical_bt_retries)*int(i<=end) for i in range(len(visits))]
            s['budgets']['per_index_parent_dispatches']=[int(i<=end) for i in range(len(visits))]
            s['result_flags'].update(recovery_used=True,strict_timing_status=result.get('strict_scan_timing_status','NOT_EVALUATED'))
            s['lifecycle']='RECOVERABLE_FAILURE'
            s['stop']=dict(state='PASSED',source_action_uuid=a['parent_action_uuid'],legacy_stop_proof=True,
                historical_result_sha256=result_sha)

    def export(self, output):
        path=Path(output);path.mkdir(parents=True,exist_ok=True)
        state=self.state()
        with self.timing('export_state:atomic_file_and_directory_fsync'):atomic_json(path/'state.json',state)
        result=dict(schema='matrix.mission.result.v1',mission_id=state['mission_id'],revision=state['revision'],
            lifecycle=state['lifecycle'],identity=state['identity'],progress=state['progress'],
            result_flags=state['result_flags'],attempts=state['attempts'],budgets=state['budgets'],
            current_target=state['current_target'],last_attempt=state['attempt'],stop=state['stop'],reason=state['hold_reason'])
        result['identity_epoch_id']=state['identity_epoch_id']
        result['identity_policy_epochs']=state.get('identity_policy_epochs',[])
        result['continuation_authorizations']=state.get('continuation_authorizations',[])
        with self.timing('export_result:atomic_file_and_directory_fsync'):atomic_json(path/'result.json',result)
        return state

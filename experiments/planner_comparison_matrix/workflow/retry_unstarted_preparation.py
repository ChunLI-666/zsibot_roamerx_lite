#!/usr/bin/env python3
"""Retry one proven unstarted preparation with an explicit Main operator permit.

This separate control-plane tool never changes the frozen case implementation,
creates a mission, starts a worker, or issues a robot command.
"""
import argparse
import copy
import fcntl
import hashlib
import importlib
import json
import math
import os
from pathlib import Path
import sys
import time


def sha(raw):return hashlib.sha256(raw).hexdigest()
def encoded(value):return (json.dumps(value,ensure_ascii=False,sort_keys=True,indent=2,allow_nan=False)+'\n').encode()
def pending_state(original,permit,permit_sha):
    changed=copy.deepcopy(original);row=next(r for r in changed['stages'] if r['id']==permit['stage_id'])
    if (row['state']!='HELD' or row['request_id']!=permit['request_id'] or row.get('invocations')!=[] or
            row.get('reason')!=permit['held_reason']):raise ValueError('Original preparation identity differs')
    row.setdefault('preparation_retry_history',[]).append(dict(permit_sha256=permit_sha,
        original_reason=row['reason'],original_state_sha256=permit['before_state_sha256'],request_id=row['request_id']))
    row.update(state='PENDING');row.pop('reason');return changed
def checked(ref):
    path=Path(ref['path']).resolve(strict=True);raw=path.read_bytes()
    if sha(raw)!=ref['sha256']:raise ValueError('Operator reference changed: '+str(path))
    return path,raw,json.loads(raw)


def retry(permit_path,execute=False):
    permit_raw=Path(permit_path).read_bytes();permit=json.loads(permit_raw);permit_sha=sha(permit_raw)
    if (permit.get('schema')!='matrix.workflow.unstarted_preparation_retry.v1' or
            permit.get('decision')!='APPROVED' or permit.get('tool_sha256')!=sha(Path(__file__).read_bytes())):
        raise ValueError('Exact Main operator permit required')
    if os.environ.get('ROS_DOMAIN_ID')!='185':raise ValueError('Private domain185 required for the driver parent')
    candidate_path,_,candidate=checked(permit['candidate'])
    workflow=Path(candidate['entrypoints']['workflow_driver.py']).parent
    # Import only hash-bound dependencies of the ORIGINAL accepted case.
    accepted={row['path']:row['sha256'] for row in candidate['source_files']}
    for name in ('workflow_driver.py','workflow_worker.py','workflow_inputs.py','mission_ledger.py'):
        path=workflow/name
        if accepted.get(str(path))!=sha(path.read_bytes()):raise ValueError('Original workflow dependency changed')
    sys.path.insert(0,str(workflow));driver=importlib.import_module('workflow_driver')
    if Path(driver.__file__).resolve()!=workflow/'workflow_driver.py':raise ValueError('Unexpected imported driver')
    root=Path(permit['run_dir']).resolve(strict=True)
    lock=(root/'workflow.lock').open('a+')
    try:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        root,plan,state=driver.read_run(root)
        if (state['plan_sha256']!=permit['plan_sha256'] or state['workflow_run_id']!=permit['workflow_run_id'] or
                state.get('approval_sha256')!=permit['candidate']['sha256']):raise ValueError('Original case binding differs')
        driver.approval_check(plan,candidate_path)
        state_path=root/'workflow_state.json';before=state_path.read_bytes()
        audit=root/'preparation_retries'/permit_sha
        if (not audit.resolve().is_relative_to(root) or audit.is_symlink() or audit.parent.is_symlink()):
            raise ValueError('Retry audit escapes original run')
        before_path=audit/'before_state.json';intent_path=audit/'intent.json';result_path=audit/'result.json'
        if intent_path.exists():
            intent=json.loads(intent_path.read_text());original=before_path.read_bytes()
            if (intent.get('permit_sha256')!=permit_sha or sha(original)!=permit['before_state_sha256'] or
                    intent['before_sha256']!=sha(original)):raise ValueError('Retry intent differs')
            expected=dict(permit_sha256=permit_sha,tool_sha256=permit['tool_sha256'],before_sha256=sha(original),
                after_sha256=sha(encoded(pending_state(json.loads(original),permit,permit_sha))),
                stage_id=permit['stage_id'],request_id=permit['request_id'],
                original_empty_directory=str(root/'stages'/permit['stage_id']/'invocation_000'),
                retained_empty_directory=str(audit/'empty_invocation_000'),starts_worker=False)
            if intent!=expected:raise ValueError('Conflicting preparation retry intent')
            if sha(before)==intent['after_sha256']:
                if execute and not result_path.exists():driver.write_once(result_path,encoded(dict(status='APPLIED',**intent)))
                return dict(status='ALREADY_REQUEUED',request_id=permit['request_id'],audit=str(audit),starts_worker=False)
            history=next(r for r in state['stages'] if r['id']==permit['stage_id']).get('preparation_retry_history',[])
            expected_history=dict(permit_sha256=permit_sha,original_reason=permit['held_reason'],
                original_state_sha256=permit['before_state_sha256'],request_id=permit['request_id'])
            if (intent.get('tool_sha256')==permit['tool_sha256'] and intent.get('stage_id')==permit['stage_id'] and
                intent.get('request_id')==permit['request_id'] and expected_history in history):
                return dict(status='ALREADY_REQUEUED',request_id=permit['request_id'],audit=str(audit),starts_worker=False)
            if before!=original:raise ValueError('State changed during preparation retry')
        elif sha(before)!=permit['before_state_sha256']:raise ValueError('Original held state changed')
        stage,row=next((s,r) for s,r in zip(plan['stages'],state['stages']) if s['id']==permit['stage_id'])
        if (stage['kind']!='client' or stage['client']['operation']!='start' or row['state']!='HELD' or
                row['request_id']!=permit['request_id'] or row.get('invocations')!=[] or
                row.get('reason')!=permit['held_reason'] or not row['reason'].startswith('preparation failed:')):
            raise ValueError('Not an exact unstarted preparation failure')
        if set(row)-{'id','state','request_id','invocations','reason','preparation_retry_history'}:
            raise ValueError('Preparation has intent, output or unknown state; reconcile instead')
        if sum(s['kind']=='client' for s in plan['stages'])!=1:raise ValueError('One original client stage required')
        for s,r in zip(plan['stages'],state['stages']):
            if s['kind']=='client' and r.get('invocations'):raise ValueError('A client invocation already exists')
            if r.get('actor_identity') and not driver.process_alive(r['actor_identity']):
                raise ValueError('CLOSE_ONLY: original actor is no longer alive')
        store=Path(driver.replace_run(stage['client']['store'],root))
        for suffix in ('','-wal','-shm','-journal','.lock'):
            if os.path.lexists(str(store)+suffix):raise ValueError('Mission/store evidence already exists')
        if os.path.lexists(store.with_suffix('.lock')):raise ValueError('Mission store lock already exists')
        if not store.resolve().is_relative_to(root):raise ValueError('Store escapes original run')
        deadline_path,_,deadline=checked(permit['recording_deadline'])
        if (deadline_path!=root/'nav/recording_deadline.json' or deadline['run_id']!=permit['recording_run_id'] or
                deadline['launcher']!=permit['launcher'] or
                Path('/proc/sys/kernel/random/boot_id').read_text().strip()!=deadline['launcher']['boot_id']):
            raise ValueError('Original recording owner/deadline differs')
        remaining=deadline['deadline_boottime_sec']-time.clock_gettime(time.CLOCK_BOOTTIME)
        minimum=permit['minimum_remaining_sec']
        if not math.isfinite(remaining) or not math.isfinite(minimum) or minimum<=0 or remaining<minimum:
            raise ValueError('CLOSE_ONLY: original recording deadline has insufficient time')
        actual=driver.workflow_worker.identity(deadline['launcher']['pid'])
        if any(actual[k]!=deadline['launcher'][k] for k in ('pid','starttime_ticks','boot_id','cmdline_sha256')):
            raise ValueError('CLOSE_ONLY: original recording launcher identity changed')
        actors=[r.get('actor_identity') for r in state['stages'] if r.get('actor_identity')]
        if not any(a['pid']==deadline['launcher']['pid'] and
                   a['starttime_ticks']==deadline['launcher']['starttime_ticks'] and
                   a['boot_id']==deadline['launcher']['boot_id'] and
                   a['cmdline_sha256']==deadline['launcher']['cmdline_sha256'] for a in actors):
            raise ValueError('Recording launcher is not an original workflow actor')
        directory=root/'stages'/permit['stage_id'];empty=directory/'invocation_000';retained=audit/'empty_invocation_000'
        if (not directory.resolve().is_relative_to(root) or directory.is_symlink() or empty.is_symlink() or
                retained.is_symlink()):
            raise ValueError('Invocation path differs')
        if any(p.name!='invocation_000' for p in directory.iterdir()):raise ValueError('Unexpected invocation residue')
        if empty.exists():
            if not empty.is_dir() or any(empty.iterdir()):raise ValueError('Invocation is not empty; reconcile instead')
            if os.path.lexists(retained):raise ValueError('Conflicting retained invocation')
        elif not (intent_path.exists() and retained.is_dir() and not any(retained.iterdir())):
            raise ValueError('Original empty invocation is not evidenced')
        changed=pending_state(state,permit,permit_sha)
        after=encoded(changed)
        intent=dict(permit_sha256=permit_sha,tool_sha256=permit['tool_sha256'],before_sha256=sha(before),
            after_sha256=sha(after),stage_id=permit['stage_id'],request_id=row['request_id'],
            original_empty_directory=str(empty),retained_empty_directory=str(retained),starts_worker=False)
        if not execute:return dict(status='DRY_RUN',remaining_sec=remaining,**intent)
        audit.parent.mkdir(exist_ok=True);driver.sync_dir(root)
        audit.mkdir(exist_ok=True);driver.sync_dir(audit.parent)
        if before_path.exists():
            if before_path.read_bytes()!=before:raise ValueError('Conflicting original held-state evidence')
        else:driver.write_once(before_path,before)
        if intent_path.exists():
            if json.loads(intent_path.read_text())!=intent:raise ValueError('Conflicting preparation retry intent')
        else:driver.write_once(intent_path,encoded(intent))
        if empty.exists():
            os.rename(empty,retained);driver.sync_dir(directory);driver.sync_dir(audit)
        if deadline['deadline_boottime_sec']-time.clock_gettime(time.CLOCK_BOOTTIME)<minimum:
            raise ValueError('CLOSE_ONLY: original recording deadline expired during durable preparation')
        driver.atomic_json(state_path,changed)
        result=dict(status='APPLIED',**intent);driver.write_once(result_path,encoded(result))
        return result
    finally:lock.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--permit',type=Path,required=True)
    parser.add_argument('--execute',action='store_true');args=parser.parse_args()
    print(json.dumps(retry(args.permit,args.execute),ensure_ascii=False))


if __name__=='__main__':main()

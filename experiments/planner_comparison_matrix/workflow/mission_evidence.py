"""Select the latest hash-bound client projection for offline evidence only."""
import hashlib
import json
from pathlib import Path


def sha(path):
    with Path(path).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()


def latest(workflow,stage_id='mission_execute'):
    row=next(x for x in workflow['stages'] if x['id']==stage_id)
    responses=[]
    if row.get('client_response'):responses.append(('operation',row['client_response'],None))
    for item in row.get('reconciliations',[]):
        if item.get('response'):responses.append(('reconcile',item['response'],item))
    found=[];mission_id=None
    for kind,response,record in responses:
        refs=response.get('references') or {};ref=refs.get('state') or {}
        if not ref.get('path') or not ref.get('sha256'):continue
        if sha(ref['path'])!=ref['sha256']:raise ValueError('client state hash mismatch')
        state=json.loads(Path(ref['path']).read_text())
        if state.get('schema')!='matrix.mission.state.v1':raise ValueError('invalid client state schema')
        if mission_id is None:mission_id=state['mission_id']
        if state['mission_id']!=mission_id:raise ValueError('client mission identity changed')
        op=refs.get('operation')
        if op:
            if sha(op['path'])!=op['sha256']:raise ValueError('client operation hash mismatch')
            data=json.loads(Path(op['path']).read_text())
            if data.get('state_sha256')!=ref['sha256'] or data.get('mission_id')!=mission_id or data.get('revision')!=state['revision']:
                raise ValueError('client operation does not bind selected state')
            if record and (data.get('operation')!='reconcile' or data.get('request_id')!=record['request_id'] or
                           data.get('parent_action_uuid')!=record['original_action_uuid']):
                raise ValueError('reconciliation identity mismatch')
        found.append((state['revision'],kind,response,ref,state))
    if not found:raise ValueError('no hash-bound client state')
    _,kind,response,ref,state=max(found,key=lambda x:x[0])
    return row,response,ref,state,kind

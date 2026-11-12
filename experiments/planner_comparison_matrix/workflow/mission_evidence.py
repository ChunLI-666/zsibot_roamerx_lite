"""Select the latest hash-bound client projection for offline evidence only."""
import hashlib
import json
from pathlib import Path


def sha(path):
    with Path(path).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()


def latest(workflow,stage_id='mission_execute'):
    row=next(x for x in workflow['stages'] if x['id']==stage_id)
    # The driver publishes the sole current projection after binding its
    # identity and revision. History is evidence, never a fallback authority.
    if row.get('client_projection_invalid'):
        raise ValueError('latest client observation is pending or invalid')
    response=row.get('client_response')
    if not isinstance(response,dict):raise ValueError('no hash-bound client state')
    kind=row.get('client_response_source','operation')
    refs=response.get('references') or {};ref=refs.get('state') or {}
    if not ref.get('path') or not ref.get('sha256'):raise ValueError('missing client state reference')
    if sha(ref['path'])!=ref['sha256']:raise ValueError('client state hash mismatch')
    state=json.loads(Path(ref['path']).read_text())
    if state.get('schema')!='matrix.mission.state.v1':raise ValueError('invalid client state schema')
    op=refs.get('operation')
    if op:
        if sha(op['path'])!=op['sha256']:raise ValueError('client operation hash mismatch')
        data=json.loads(Path(op['path']).read_text())
        if (data.get('state_sha256')!=ref['sha256'] or data.get('mission_id')!=state.get('mission_id')
                or data.get('revision')!=state.get('revision')):
            raise ValueError('client operation does not bind selected state')
        if kind=='reconcile':
            records=[item for item in row.get('reconciliations',[]) if item.get('response')==response]
            if (len(records)!=1 or data.get('operation')!='reconcile'
                    or data.get('request_id')!=records[0]['request_id']
                    or data.get('parent_action_uuid')!=records[0]['original_action_uuid']):
                raise ValueError('reconciliation identity mismatch')
    return row,response,ref,state,kind

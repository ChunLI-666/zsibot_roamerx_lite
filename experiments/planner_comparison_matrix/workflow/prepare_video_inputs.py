#!/usr/bin/env python3
"""Adapt hash-bound client visits to the existing native-map phone renderer."""
import argparse
import copy
import hashlib
import json
from pathlib import Path

import mission_evidence


def sha(path):
    with Path(path).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run-dir',type=Path,required=True);a=p.parse_args();run=a.run_dir
    workflow=json.loads((run/'workflow_state.json').read_text())
    row,response,ref,state,state_source=mission_evidence.latest(workflow)
    route=state['route'];accepted=state['progress']['accepted_receipts']
    audit=json.loads((run/'mcap_audit.json').read_text())
    if not audit['read_to_eof'] or not audit['integrity_pass']:raise ValueError('recording not closed and verified')
    native=[]
    for attempt in state['attempts']:
        item=attempt.get('native_result')
        if item:
            path=Path(item['path'])
            if sha(path)!=item['sha256']:raise ValueError('native attempt changed')
            native.append((attempt,json.loads(path.read_text())))
    if not native:raise ValueError('no new native attempt to render')
    result=copy.deepcopy(native[-1][1]);visits=[dict(index=i) for i in range(len(route))]
    accepted_sources={(receipt['index'],receipt['source_attempt_id'],receipt['source_action_uuid']) for receipt in accepted}
    unaccepted_native_passes=[]
    for attempt,raw in native:
        for visit in raw.get('ordered',{}).get('visits',[]):
            index=visit.get('index')
            if isinstance(index,int) and 0<=index<len(visits) and (visit.get('entered') or visit.get('passed')):
                display=copy.deepcopy(visit)
                display.pop('passed',None)
                visits[index]=display
                if (index,attempt['attempt_id'],attempt['parent_action_uuid']) not in accepted_sources and visit.get('passed'):
                    unaccepted_native_passes.append(dict(index=index,attempt_id=attempt['attempt_id'],
                        parent_action_uuid=attempt['parent_action_uuid'],native_result=copy.deepcopy(attempt['native_result']),
                        visit=copy.deepcopy(visit)))
    for receipt in accepted:
        index=receipt['index'];visit=receipt['visit']
        if (not 0<=index<len(visits) or visit['index']!=index or not visit.get('passed')):
            raise ValueError('accepted visit index or proof differs')
        visits[index]=copy.deepcopy(visit)
    result['ordered']=dict(visits=visits,all_visited=bool(state['result_flags']['operational_complete']))
    result['unaccepted_native_pass_observations']=unaccepted_native_passes
    result['mission_projection']=dict(mission_id=state['mission_id'],lifecycle=state['lifecycle'],
        accepted_visits=len(accepted),total_visits=len(route),state_sha256=ref['sha256'])
    result['derived_render_input_boundary']='Displayed passed visits come only from MissionStore accepted receipts. Native observations and original status are retained separately; not an independent success proof.'
    case=run/'video_case';case.mkdir(exist_ok=False)
    (case/'result.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    events=[]
    for attempt,raw in native:
        path=Path(attempt['output'])/'events.jsonl'
        if path.exists():
            for line in path.read_text().splitlines():
                event=json.loads(line)
                if event.get('kind') in ('goal_send','action_result'):events.append(event)
    if events:
        # The legacy renderer uses the first goal_send and first action_result.
        # For a multi-attempt mission, keep its displayed terminal at the
        # actual final result; preserve every original event separately.
        (case/'all_attempt_events.jsonl').write_text(''.join(json.dumps(x,separators=(',',':'))+'\n' for x in events))
        sends=[x for x in events if x.get('kind')=='goal_send'];terminals=[x for x in events if x.get('kind')=='action_result']
        selected=([sends[0]] if sends else [])+([terminals[-1]] if terminals else [])
        (case/'events.jsonl').write_text(''.join(json.dumps(x,separators=(',',':'))+'\n' for x in selected))
    output=dict(case_dir=str(case),start_ns=audit['first_log_ns'],end_ns=audit['last_log_ns'],
                mission_state_sha256=ref['sha256'],mission_state_source=state_source,native_attempts=[dict(path=x['native_result']['path'],sha256=x['native_result']['sha256']) for x,_ in native],
                historical_visits=sum(bool(x.get('legacy_stop_proof')) for x in accepted),
                accepted_visits=len(accepted),unaccepted_native_pass_observations=len(unaccepted_native_passes),
                boundary='Original MCAP clock and native map; no GT used. Derived case is not mission acceptance.')
    if output['end_ns']<=output['start_ns']:raise ValueError('invalid recorded time range')
    (run/'video_inputs.json').write_text(json.dumps(output,indent=2)+'\n');print(json.dumps(output))


if __name__=='__main__':main()

#!/usr/bin/env python3
"""Independent observation audit; never participates in Nav2 authority decisions."""
import argparse
import bisect
import json
import math
from pathlib import Path


def nonzero(velocity):
    return any(not math.isfinite(value) or abs(value)>1e-5 for value in velocity)


def command_velocity(data):
    value=data['velocity'];return [value['linear']['x'],value['linear']['y'],value['angular']['z']]


def audit_callback(rows, require_motion=True, source_age_margin_ms=30., timestamp_uncertainty_ns=0):
    commands=[r for r in rows if r['event']=='/cmd_vel_epoch']
    raw_commands=[r for r in rows if r['event']=='/cmd_vel_epoch_raw']
    command_times=[r['monotonic_ns'] for r in commands]
    safe=[r for r in rows if r['event']=='/cmd_vel_safe']
    authorities={topic:[r for r in rows if r['event']==topic] for topic in ('/nav_epoch/intent','/nav_epoch/execution')}
    authority_times={topic:[r['monotonic_ns'] for r in values] for topic,values in authorities.items()}
    latest={};failures=[];positive=0;matched=0;max_source_age_ms=0.;margins=[]
    def boundary(at,name,margin_ns,multiplier=1):
        if not timestamp_uncertainty_ns:return
        margins.append((name,margin_ns))
        if 0<=margin_ns<=multiplier*timestamp_uncertainty_ns:
            failures.append(dict(reason='timestamp boundary is unscorable',at=at,boundary=name,margin_ns=margin_ns,uncertainty_ns=multiplier*timestamp_uncertainty_ns))
    for row in rows:
        topic=row['event'];latest[topic]=row
        if topic!='/cmd_vel_safe' or not nonzero(row['data']['velocity']):continue
        positive+=1;t=row['monotonic_ns'];velocity=row['data']['velocity']
        if not all(math.isfinite(v) for v in velocity):failures.append(dict(reason='nonfinite safe velocity',at=t));continue
        index=bisect.bisect_left(command_times,t)
        # DDS callback ordering across topics is not causal delivery ordering.
        # Match the same velocity within a bounded 50 ms observer window.
        candidates=[cmd for cmd in commands[max(0,index-8):index+8]
                    if abs(cmd['monotonic_ns']-t)<=50_000_000 and
                    all(abs(a-b)<1e-8 for a,b in zip(command_velocity(cmd['data']),velocity))]
        if not candidates:failures.append(dict(reason='safe output has no nearby typed command',at=t));continue
        matched+=1;witness=min(candidates,key=lambda r:abs(r['monotonic_ns']-t));cmd=witness['data']
        boundary(t,'relay matching window',50_000_000-abs(witness['monotonic_ns']-t),2)
        raw=[r for r in raw_commands if -350_000_000<=r['monotonic_ns']-t<=50_000_000 and
             all(r['data'][key]==cmd[key] for key in ('token','controller_session_id','command_sequence','boot_id','source_steady_time_ns','max_age_ms')) and
             r['data'].get('source_motion_phase',0)==cmd.get('source_motion_phase',0) and not r['data'].get('transition_braking',False) and r['data'].get('braking_from_phase',0)==0]
        if not raw:failures.append(dict(reason='smoothed command has no matching raw controller provenance',at=t))
        boot=latest.get('fixture_start',{}).get('data',{}).get('boot_id')
        if cmd.get('boot_id')!=boot:failures.append(dict(reason='command boot identity mismatch',at=t))
        for authority in ('/nav_epoch/intent','/nav_epoch/execution'):
            observed=authorities[authority];times=authority_times[authority]
            preceding=bisect.bisect_right(times,t)-1
            # Require an exact positive witness: the latest preceding authority,
            # or a matching callback delivered within the bounded DDS window.
            candidates=observed[max(0,preceding):bisect.bisect_right(times,t+50_000_000)]
            matching=[r for r in candidates if r['data']['active'] and r['data']['token']==cmd['token']
                      and r['data']['boot_id']==boot and -2_000_000<=t-r['data']['source_steady_time_ns']<=(330_000_000 if authority=='/nav_epoch/execution' else 430_000_000)
                      and (authority!='/nav_epoch/execution' or r['data']['controller_session_id']==cmd['controller_session_id'])]
            if not matching:failures.append(dict(reason='safe command without exact fresh active '+authority,at=t))
            else:
                best=max((330_000_000 if authority=='/nav_epoch/execution' else 430_000_000)-(t-r['data']['source_steady_time_ns']) for r in matching)
                boundary(t,authority+' freshness',best)
        gate=latest.get('/nav_epoch/gate_session',{}).get('data',{}).get('data')
        if not gate or gate!=cmd['token']['gate_session_id']:
            failures.append(dict(reason='safe command gate session mismatch',at=t))
        age=(t-cmd['source_steady_time_ns'])/1e6;max_source_age_ms=max(max_source_age_ms,age)
        boundary(t,'command TTL',(cmd['max_age_ms']-age)*1e6)
        boundary(t,'command future bound',(age+2)*1e6)
        if age < -2 or age>cmd['max_age_ms']+source_age_margin_ms:
            failures.append(dict(reason='safe command source age invalid',at=t,age_ms=age,max_age_ms=cmd['max_age_ms']))
        loc=latest.get('localization_publication',{}).get('data')
        transition=latest.get('localization_transition')
        publication=latest.get('localization_publication')
        if not loc:failures.append(dict(reason='no observed localization authority',at=t))
        if loc:
            if cmd['token']['localization']!=loc['identity']:
                failures.append(dict(reason='safe output uses a different localization identity',at=t))
            if loc['health']!=1:
                failures.append(dict(reason='safe output during non-NORMAL localization',at=t))
        # Event-triggered stop has a separately asserted bounded receipt latency.
        # Record violations only after 100 ms of a loss event, avoiding callback races.
        if transition and transition['data']['health']!=1 and t-transition['monotonic_ns']>100_000_000 and (not publication or publication['monotonic_ns']<transition['monotonic_ns'] or publication['data']['health']!=1):
            failures.append(dict(reason='safe output persists after loss transition',at=t))
    if not safe:failures.append(dict(reason='no safe command observations'))
    if require_motion and not positive:failures.append(dict(reason='no witnessed authorized motion; integration coverage absent'))
    return dict(require_motion=require_motion,safe_samples=len(safe),nonzero_safe_samples=positive,matched_typed_samples=matched,
                max_nonzero_source_age_ms=max_source_age_ms,failures=failures,
                observer_matching_window_ms=50,source_age_observer_margin_ms=source_age_margin_ms,
                minimum_timestamp_boundary_margin_ns=min((m for _,m in margins),default=None),timestamp_uncertainty_ns=timestamp_uncertainty_ns,
                note='Wire audit complements action/scenario assertions; it is not a gate implementation.')


def audit_intervals(rows, require_motion=True):
    """Independent publication evidence. Uncertainty is not a safe-output pass."""
    from publication_clock import bounds,difference,within
    topics={}
    for row in rows:topics.setdefault(row['event'],[]).append(row)
    failures=[];uncertain=[];margins={};matched=positive=0;maximum_age=0.
    def check(row,reason,interval,low,high):
        state=within(interval,low,high)
        margin=min(interval[0]-low,high-interval[1])
        margins[reason]=min(margins.get(reason,margin),margin)
        if state!='pass':
            entry=dict(reason=reason if state=='fail' else 'timestamp boundary is unscorable',
                boundary=reason,at=row['callback_monotonic_ns'],publication_interval_ns=list(bounds(row)),
                difference_interval_ns=list(interval),allowed_interval_ns=[low,high],classification=state)
            (failures if state=='fail' else uncertain).append(entry)
        return state
    def fail(row,reason):failures.append(dict(reason=reason,at=row.get('callback_monotonic_ns',row['monotonic_ns']),classification='fail'))
    def state_witness(row,topic,predicate):
        # Topic publishers are ordered. Include every state that could be latest,
        # stopping only after a definitely preceding witness. Never order a
        # local publish by its bracket midpoint to turn a loss into NORMAL.
        possible=[];definite=False
        for item in reversed(topics.get(topic,[])):
            lo,hi=difference(row,item)
            if hi<0:continue
            possible.append(item)
            if lo>=0:definite=True;break
        outcomes=[predicate(item) for item in possible]
        outcomes=['pass' if value is True else 'fail' if value is False else value for value in outcomes]
        if definite and outcomes and all(value=='pass' for value in outcomes):return possible
        if definite and outcomes and all(value=='fail' for value in outcomes):fail(row,'safe command without matching active '+topic)
        else:uncertain.append(dict(reason='authority ordering is unscorable',at=row['callback_monotonic_ns'],topic=topic,
            publication_interval_ns=list(bounds(row)),candidate_intervals_ns=[list(bounds(x)) for x in possible],candidate_outcomes=outcomes,classification='unscorable'))
        return []
    boot=next((r['data']['boot_id'] for r in rows if r['event']=='fixture_start'),None)
    raw_index={}
    keys=('token','controller_session_id','command_sequence','boot_id','source_steady_time_ns','max_age_ms')
    def key(data):return json.dumps([data[k] for k in keys],sort_keys=True)
    for row in topics.get('/cmd_vel_epoch_raw',[]):raw_index.setdefault(key(row['data']),[]).append(row)
    safe=topics.get('/cmd_vel_safe',[])
    for row in safe:
        velocity=row['data']['velocity']
        if not nonzero(velocity):continue
        positive+=1
        if not all(math.isfinite(v) for v in velocity):fail(row,'nonfinite safe velocity');continue
        candidates=[];possible=[]
        for relay in topics.get('/cmd_vel_epoch',[]):
            if not all(abs(a-b)<1e-8 for a,b in zip(command_velocity(relay['data']),velocity)):continue
            verdict=within(difference(row,relay),-50_000_000,50_000_000)
            if verdict=='pass':candidates.append(relay)
            elif verdict=='unscorable':possible.append(relay)
        if not candidates:
            if possible:
                check(row,'relay matching window',difference(row,possible[0]),-50_000_000,50_000_000)
            else:fail(row,'safe output has no nearby typed command')
            continue
        relay=min(candidates,key=lambda r:max(abs(x) for x in difference(row,r)));cmd=relay['data'];matched+=1
        check(row,'relay matching window',difference(row,relay),-50_000_000,50_000_000)
        originals=[r for r in raw_index.get(key(cmd),[]) if r['data'].get('source_motion_phase',0)==cmd.get('source_motion_phase',0)
            and not r['data'].get('transition_braking',False) and r['data'].get('braking_from_phase',0)==0]
        states=[within(difference(r,row),-350_000_000,50_000_000) for r in originals]
        if 'pass' not in states:
            if 'unscorable' in states:check(row,'raw provenance window',difference(originals[states.index('unscorable')],row),-350_000_000,50_000_000)
            else:fail(row,'smoothed command has no matching raw controller provenance')
        if cmd.get('boot_id')!=boot:fail(row,'command boot identity mismatch')
        # Source ages use monotonic integers from the producer, not refreshed
        # relay timestamps. Production freshness permits no future source.
        a,b=bounds(row);age=(a-cmd['source_steady_time_ns'],b-cmd['source_steady_time_ns']);maximum_age=max(maximum_age,age[1]/1e6)
        check(row,'safe command source age invalid',age,0,cmd['max_age_ms']*1_000_000)
        for topic,ttl in (('/nav_epoch/intent',400_000_000),('/nav_epoch/execution',300_000_000)):
            def predicate(item):
                d=item['data']
                matches=d['active'] and d['token']==cmd['token'] and d['boot_id']==boot and (topic!='/nav_epoch/execution' or d['controller_session_id']==cmd['controller_session_id'])
                return within((a-d['source_steady_time_ns'],b-d['source_steady_time_ns']),0,ttl) if matches else 'fail'
            witnesses=state_witness(row,topic,predicate)
            for witness in witnesses:
                check(row,topic+' source freshness',(a-witness['data']['source_steady_time_ns'],b-witness['data']['source_steady_time_ns']),0,ttl)
        state_witness(row,'/nav_epoch/gate_session',lambda r:r['data']['data']==cmd['token']['gate_session_id'])
        loc_witnesses=state_witness(row,'localization_publication',lambda r:r['data']['health']==1 and r['data']['identity']==cmd['token']['localization'])
        transitions=[r for r in topics.get('localization_transition',[]) if difference(row,r)[1]>=0]
        if transitions and transitions[-1]['data']['health']!=1:
            transition=transitions[-1];elapsed=difference(row,transition)
            recovered=loc_witnesses and all(difference(loc,transition)[0]>0 for loc in loc_witnesses)
            if not recovered and elapsed[1]>100_000_000:
                if elapsed[0]>100_000_000:fail(row,'safe output persists after loss transition')
                else:uncertain.append(dict(reason='loss-stop boundary is unscorable',at=row['callback_monotonic_ns'],difference_interval_ns=list(elapsed),classification='unscorable'))
        if cmd['token'].get('execution_kind',0)==1:
            # BackUp deadline is exclusive in the actual gate contract.
            deadline=cmd['token']['execution_deadline_ns']
            check(row,'backup deadline',(a-deadline,b-deadline),-10**30,-1)
    if not safe:failures.append(dict(reason='no safe command observations',classification='fail'))
    if require_motion and not positive:failures.append(dict(reason='no witnessed authorized motion; integration coverage absent',classification='fail'))
    assessment='FAIL' if failures else ('UNSCORABLE' if uncertain else 'PASS')
    return dict(require_motion=require_motion,safe_samples=len(safe),nonzero_safe_samples=positive,matched_typed_samples=matched,
        max_nonzero_source_age_ms=maximum_age,failures=failures+uncertain,definite_failure_count=len(failures),unscorable_count=len(uncertain),
        assessment=assessment,wire_pass=assessment=='PASS',observer_matching_window_ms=50,source_age_observer_margin_ms=0,
        minimum_boundary_margins_ns=margins,note='Interval publication evidence; gate receipt times are not observed. FAIL is an inconsistent evidence trace, not by itself proof of a gate implementation defect.')


def audit(rows, require_motion=True):
    from publication_clock import timeline
    callback=audit_callback(rows,require_motion)
    projected,clock=timeline(rows)
    if clock['basis']=='legacy_callback':
        callback['publication_clock']=clock;return callback
    if projected is None:
        # Callback age/order failures cannot become definite publication-time
        # failures when the clock is invalid. Timeless violations still fail.
        hard=[dict(item,classification='fail') for item in callback['failures'] if item['reason'] in (
            'nonfinite safe velocity','no safe command observations',
            'no witnessed authorized motion; integration coverage absent')]
        result={key:value for key,value in callback.items() if key!='failures'}
        result.update(publication_clock=clock,callback_audit=callback,
            failures=hard+[dict(reason='DDS publication time is unscorable',clock_issues=clock['issues'],classification='unscorable')],
            definite_failure_count=len(hard),unscorable_count=1,
            assessment='FAIL' if hard else 'UNSCORABLE',wire_pass=False)
        return result
    # Preserve old callback-based evidence alongside publication-based scoring.
    # No source TTL is renewed and the matching window remains 50 ms.
    result=audit_intervals(projected,require_motion)
    result['publication_clock']=clock
    result['callback_audit']=callback
    return result


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--events',required=True,type=Path);parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args();result=audit([json.loads(line) for line in args.events.read_text().splitlines()])
    args.output.write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='failures'},indent=2))
    if result['failures']:raise SystemExit(1)


if __name__=='__main__':main()

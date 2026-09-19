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


def audit(rows):
    commands=[r for r in rows if r['event']=='/cmd_vel_epoch']
    raw_commands=[r for r in rows if r['event']=='/cmd_vel_epoch_raw']
    command_times=[r['monotonic_ns'] for r in commands]
    safe=[r for r in rows if r['event']=='/cmd_vel_safe']
    authorities={topic:[r for r in rows if r['event']==topic] for topic in ('/nav_epoch/intent','/nav_epoch/execution')}
    authority_times={topic:[r['monotonic_ns'] for r in values] for topic,values in authorities.items()}
    latest={};failures=[];positive=0;matched=0;max_source_age_ms=0.
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
        matched+=1;cmd=min(candidates,key=lambda r:abs(r['monotonic_ns']-t))['data']
        raw=[r for r in raw_commands if -350_000_000<=r['monotonic_ns']-t<=50_000_000 and
             all(r['data'][key]==cmd[key] for key in ('token','controller_session_id','command_sequence','boot_id','source_steady_time_ns','max_age_ms'))]
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
        gate=latest.get('/nav_epoch/gate_session',{}).get('data',{}).get('data')
        if not gate or gate!=cmd['token']['gate_session_id']:
            failures.append(dict(reason='safe command gate session mismatch',at=t))
        age=(t-cmd['source_steady_time_ns'])/1e6;max_source_age_ms=max(max_source_age_ms,age)
        if age < -2 or age>cmd['max_age_ms']+30:
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
    if not positive:failures.append(dict(reason='no witnessed authorized motion; integration coverage absent'))
    return dict(safe_samples=len(safe),nonzero_safe_samples=positive,matched_typed_samples=matched,
                max_nonzero_source_age_ms=max_source_age_ms,failures=failures,
                observer_matching_window_ms=50,source_age_observer_margin_ms=30,
                note='Wire audit complements action/scenario assertions; it is not a gate implementation.')


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--events',required=True,type=Path);parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args();result=audit([json.loads(line) for line in args.events.read_text().splitlines()])
    args.output.write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='failures'},indent=2))
    if result['failures']:raise SystemExit(1)


if __name__=='__main__':main()

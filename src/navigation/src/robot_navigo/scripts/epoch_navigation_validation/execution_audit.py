"""Independent phase/provenance observer; legacy vectors and explicit phases differ.

TRACK legitimately includes the optimizer's curved trajectories. A phase change
must brake under its witnessed old phase and pass through an exact zero relay
sample. Amplitude/within-TRACK lag remains a diagnostic, not a proof that MPPI
models the complete smoother dynamics.
"""
import bisect
import json
import math
from publication_clock import difference,within,timeline


def velocity(data):
    value=data['velocity'];return [value['linear']['x'],value['linear']['y'],value['angular']['z']]


def identity(data):
    return (json.dumps(data['token'],sort_keys=True),data['controller_session_id'],data['command_sequence'])


def nonzero(v):return any(a!=0. for a in v)
def mixed(v):return (v[0]!=0. or v[1]!=0.) and v[2]!=0.
def phase_valid(phase,v):
    return phase in (0,1,2,3,4) and (phase!=1 or not nonzero(v)) and (phase!=2 or v[2]==0.) and (phase!=3 or v[0]==v[1]==0.)


def audit_execution_timeline(rows):
    raw=[r for r in rows if r['event']=='/cmd_vel_epoch_raw'];smooth=[r for r in rows if r['event']=='/cmd_vel_epoch']
    modes=[r for r in rows if r['event']=='/controller_server/FollowPath/motion_mode'];mode_times=[r['monotonic_ns'] for r in modes]
    raw_index={identity(r['data']):r for r in raw};smooth_times=[r['monotonic_ns'] for r in smooth]
    counts={};changed=[];reverse=[];violations=[];axis_lags=[];legacy_leaks=[];simultaneous={};previous={}
    safe_count=matched=braking_count=phase_changes=timing_uncertain=0
    def fail(row,reason):violations.append(dict(at=row['monotonic_ns'],reason=reason))
    for row in raw:
        data=row['data'];v=velocity(data)
        if not all(math.isfinite(a) for a in v) or not phase_valid(data.get('source_motion_phase',0),v):fail(row,'invalid raw phase/velocity')
        if data.get('transition_braking',False) or data.get('braking_from_phase',0)!=0:fail(row,'raw carries relay-only braking metadata')
    for row in raw+smooth:
        v=velocity(row['data'])
        if mixed(v):simultaneous[row['event']]=simultaneous.get(row['event'],0)+1
        if v[0]<-1e-6:reverse.append(dict(event=row['event'],at=row['monotonic_ns'],velocity=v))
    for row in smooth:
        data=row['data'];v=velocity(data);prior=previous.get(data['controller_session_id']);previous[data['controller_session_id']]=row
        original=raw_index.get(identity(data))
        if original is None:
            if nonzero(v):fail(row,'nonzero relay source missing')
            continue
        source=velocity(original['data']);phase=data.get('source_motion_phase',0);braking=data.get('transition_braking',False);origin=data.get('braking_from_phase',0)
        if not all(math.isfinite(a) for a in v):fail(row,'nonfinite relay')
        if phase!=original['data'].get('source_motion_phase',0):fail(row,'source phase changed by relay')
        prior_v=velocity(prior['data']) if prior else [0.,0.,0.]
        prior_phase=prior['data'].get('source_motion_phase',0) if prior else 0
        prior_origin=prior['data'].get('braking_from_phase',0) if prior and prior['data'].get('transition_braking',False) else prior_phase
        if braking:
            braking_count+=1
            if origin not in (1,2,3,4):fail(row,'unknown braking origin')
            if nonzero(v):
                if origin!=prior_origin:fail(row,'braking origin has no previous relay evidence')
                if not phase_valid(origin,v):fail(row,'braking axes contradict old phase')
                if any(abs(a)>abs(b)+1e-12 or a*b<0. for a,b in zip(v,prior_v)):fail(row,'braking increases or reverses previous relay velocity')
        else:
            if origin!=0:fail(row,'nonbraking relay carries old phase')
            if prior and prior['data'].get('transition_braking',False) and nonzero(prior_v) and nonzero(v):fail(row,'braking cancelled before zero relay cycle')
            if not phase_valid(phase,v):fail(row,'relay axes contradict source phase')
            if phase and prior and phase!=prior_origin and nonzero(v) and nonzero(prior_v):
                phase_changes+=1;fail(row,'new phase began without a zero relay cycle')
        leaking=[i for i in range(3) if source[i]==0. and abs(v[i])>1e-6]
        if leaking:
            entry=dict(at=row['monotonic_ns'],raw=source,smooth=v,phase=phase,braking=braking,axes=leaking);axis_lags.append(entry)
            if phase==0 and nonzero(source) and not braking:legacy_leaks.append(entry)
        if any(abs(a-b)>1e-12 for a,b in zip(v,source)) and nonzero(v):changed.append(dict(at=row['monotonic_ns'],raw=source,smooth=v,sequence=data['command_sequence']))
        at=original['monotonic_ns'];i=bisect.bisect_left(mode_times,at)
        nearby=[m for m in modes[max(0,i-2):i+2] if abs(m['monotonic_ns']-at)<=50_000_000]
        if nearby:
            mode=min(nearby,key=lambda m:abs(m['monotonic_ns']-at))['data']['data'].split()[0].removeprefix('mode=');counts[mode]=counts.get(mode,0)+1
    for row in rows:
        if row['event']!='/cmd_vel_safe':continue
        v=row['data']['velocity']
        if mixed(v):simultaneous[row['event']]=simultaneous.get(row['event'],0)+1
        if v[0]<-1e-6:reverse.append(dict(event=row['event'],at=row['monotonic_ns'],velocity=v))
        if not nonzero(v):continue
        safe_count+=1;t=row['monotonic_ns'];i=bisect.bisect_left(smooth_times,t)
        equal=[r for r in smooth if all(abs(a-b)<=1e-12 for a,b in zip(v,velocity(r['data'])))]
        candidates=[r for r in equal if within(difference(row,r),-50_000_000,50_000_000)=='pass']
        if not candidates:
            if any(within(difference(row,r),-50_000_000,50_000_000)=='unscorable' for r in equal):
                timing_uncertain+=1;fail(row,'safe relay matching interval crosses 50ms boundary')
            else:fail(row,'safe has no equal relay velocity')
            continue
        relay=min(candidates,key=lambda r:abs(r['monotonic_ns']-t))
        if identity(relay['data']) not in raw_index:fail(row,'safe relay source missing');continue
        matched+=1
    return dict(safe_nonzero_samples=safe_count,raw_relay_safe_matches=matched,mode_counts=counts,
        changed_nonzero_relay_count=len(changed),changed_examples=changed[:20],declared_braking_samples=braking_count,
        reverse_command_count=len(reverse),reverse_examples=reverse[:20],violations=violations[:50],violation_count=len(violations),
        phase_change_without_zero_count=phase_changes,raw_zero_axis_leak_count=len(legacy_leaks),axis_leak_examples=legacy_leaks[:20],
        within_or_braking_axis_lag_count=len(axis_lags),axis_lag_examples=axis_lags[:20],simultaneous_translation_rotation=simultaneous,
        execution_pass=any(r['event']=='/cmd_vel_safe' for r in rows) and not violations and not reverse and not legacy_leaks,
        observer_window_ms=50,timing_unscorable_count=timing_uncertain,note=__doc__)


def audit_execution(rows):
    callback=audit_execution_timeline(rows)
    projected,clock=timeline(rows)
    if clock['basis']=='legacy_callback':
        callback['publication_clock']=clock;return callback
    if projected is None:
        callback['execution_pass']=False
        callback['violation_count']+=1
        callback['violations'].append(dict(reason='publication intervals unavailable',issues=clock['issues']))
        callback['publication_clock']=clock
        return callback
    result=audit_execution_timeline(projected)
    result['publication_clock']=clock
    result['callback_audit']=callback
    return result

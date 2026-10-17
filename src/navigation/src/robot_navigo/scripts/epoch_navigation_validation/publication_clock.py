"""Publication intervals without changing recorded callback/plant time.

A shared REALTIME-MONOTONIC offset cancels in DDS-DDS comparisons. If samples
cannot share one offset but differ by <=1 ms, retain that drift uncertainty.
Local publish brackets are never narrowed to pass a timing boundary.
"""
import time

WIRE_TOPICS={'/cmd_vel_epoch_raw','/cmd_vel_epoch','/cmd_vel_safe','/nav_epoch/intent','/nav_epoch/execution','/nav_epoch/gate_session'}
MAX_OFFSET_SPREAD_NS=1_000_000


def capture(info):
    before=time.monotonic_ns();wall=time.time_ns();after=time.monotonic_ns()
    return dict(callback_clock=dict(monotonic_before=before,wall=wall,monotonic_after=after),
        dds={key:(int(info[key]) if info.get(key) is not None else None) for key in ('source_timestamp','received_timestamp','publication_sequence_number','reception_sequence_number')})


def bounds(row):
    return tuple(row.get('publication_interval_ns',(row['monotonic_ns'],row['monotonic_ns'])))


def difference(left,right):
    """Bounds on left minus right; preserve shared-clock correlation."""
    if '_dds_wall_ns' in left and '_dds_wall_ns' in right:
        value=left['_dds_wall_ns']-right['_dds_wall_ns']
        drift=max(left['_offset_drift_ns'],right['_offset_drift_ns'])
        return value-drift,value+drift
    a,b=bounds(left);c,d=bounds(right)
    return a-d,b-c


def within(interval,lower,upper):
    """Three-valued closed-boundary predicate; no threshold is enlarged."""
    a,b=interval
    if lower<=a and b<=upper:return 'pass'
    if b<lower or a>upper:return 'fail'
    return 'unscorable'


def timeline(rows):
    required=[r for r in rows if r['event'] in WIRE_TOPICS]
    if not any('observation' in row for row in required):return None,dict(available=False,basis='legacy_callback',issues=[])
    issues=[];offsets=[];queue=[];source_delays=[];local_widths=[];clock_widths=[]
    for row in rows:
        if row['event']!='localization_publication':continue
        bracket=row.get('publication_bracket')
        if not bracket or len(bracket)!=2 or not 0<=bracket[0]<=bracket[1]:
            issues.append(dict(at=row['monotonic_ns'],reason='local publication bracket missing or invalid'))
        else:local_widths.append(bracket[1]-bracket[0])
    for row in required:
        observation=row.get('observation');at=row['monotonic_ns']
        if not observation:issues.append(dict(at=at,reason='missing DDS observation'));continue
        c=observation.get('callback_clock',{});d=observation.get('dds',{})
        if not all(isinstance(c.get(k),int) for k in ('monotonic_before','wall','monotonic_after')) or c['monotonic_before']>c['monotonic_after']:
            issues.append(dict(at=at,reason='callback clock bracket invalid'));continue
        clock_widths.append(c['monotonic_after']-c['monotonic_before'])
        if not d.get('source_timestamp') or not d.get('received_timestamp') or d['source_timestamp']<=0 or d['received_timestamp']<=0 or d['source_timestamp']>d['received_timestamp'] or d['received_timestamp']>c['wall']:
            issues.append(dict(at=at,reason='DDS source/received timestamp missing or invalid'));continue
        offsets.append((c['wall']-c['monotonic_after'],c['wall']-c['monotonic_before']))
        queue.append((c['wall']-d['received_timestamp'])/1e6);source_delays.append((c['wall']-d['source_timestamp'])/1e6)
    common_low=max((a for a,b in offsets),default=0);common_high=min((b for a,b in offsets),default=0)
    common=common_low<=common_high
    low=common_low if common else min(a for a,b in offsets)
    high=common_high if common else max(b for a,b in offsets)
    drift=0 if common else high-low
    if common_low-common_high>MAX_OFFSET_SPREAD_NS:
        issues.append(dict(reason='wall-to-monotonic offset changed',minimum_incompatible_offset_ns=common_low-common_high))
    metadata=dict(available=bool(offsets) and not issues,basis='dds_publication_intervals',issues=issues,
        observed_wire_messages=len(required),offset_lower_ns=low,offset_upper_ns=high,
        offset_model='shared_constant' if common else 'bounded_drift',dds_difference_uncertainty_ns=drift,
        timestamp_uncertainty_ns=(high-low+1)//2,max_local_publication_bracket_ns=max(local_widths,default=0),
        max_observed_clock_bracket_ns=max(clock_widths,default=0),max_offset_spread_ns=MAX_OFFSET_SPREAD_NS,
        max_observer_queue_ms=max(queue,default=0.),max_publication_to_callback_ms=max(source_delays,default=0.))
    if issues or not offsets:return None,metadata
    projected=[]
    for original in rows:
        row=dict(original)
        if original['event'] in WIRE_TOPICS:
            wall=original['observation']['dds']['source_timestamp']
            row['publication_interval_ns']=[wall-high,wall-low]
            row['_dds_wall_ns']=wall;row['_offset_drift_ns']=drift
        elif original['event']=='localization_publication':
            row['publication_interval_ns']=list(original['publication_bracket'])
        if 'publication_interval_ns' in row:
            row['callback_monotonic_ns']=original['monotonic_ns']
            row['monotonic_ns']=sum(row['publication_interval_ns'])//2  # Display/sort only.
        projected.append(row)
    projected.sort(key=lambda r:r['monotonic_ns'])
    return projected,metadata

#!/usr/bin/env python3
"""Default-HOLD acquisition-only semantic relay, never the formal Nav relay."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import select
import socket
import sys
import time

SCOPE='MAPPING_ACQUISITION_LATERAL_RECT_V5_ONLY'
CAPABILITY='semantic_lateral_mag04_rect_v5'
READY='MAPPING_LATERAL_RELAY_RECT_V5_READY 127.0.0.1:7447'
ZERO=dict(version=1,action='walk',forward=0.,lateral=0.,yaw=0.)
PACKET_AGE_NS=100_000_000
WATCHDOG_SEC=.15
SELECT_MAX_SEC=.05
SOCKET_TIMEOUT_SEC=.05

def identity(pid):
    path=Path('/proc')/str(pid);parts=(path/'stat').read_text().rsplit(')',1)[1].split();raw=(path/'cmdline').read_bytes()
    if not raw or parts[0]=='Z':raise ProcessLookupError('parent not live')
    return dict(pid=pid,starttime_ticks=int(parts[19]),cmdline_sha256=hashlib.sha256(raw).hexdigest(),
        argv=[p.decode(errors='backslashreplace') for p in raw.rstrip(b'\0').split(b'\0')],
        boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip())

def validated_axes(msg,now_ns,fine_forward_profile=None):
    plain={'scope','capability','issued_unix_ns','forward','lateral','yaw'}
    fine=isinstance(msg,dict) and set(msg)==plain|{'primitive_phase','fine_forward_subcapability'}
    if not isinstance(msg,dict) or (set(msg)!=plain and not fine):
        raise ValueError('exact scoped primitive required')
    if msg['scope']!=SCOPE or msg['capability']!=CAPABILITY or type(msg['issued_unix_ns']) is not int:
        raise ValueError('wrong acquisition capability/stamp')
    if not 0<=now_ns-msg['issued_unix_ns']<=PACKET_AGE_NS:raise ValueError('stale/future command')
    values=[msg[k] for k in ('forward','lateral','yaw')]
    if not all(type(v) in (int,float) and math.isfinite(v) for v in values):raise ValueError('nonfinite/boolean axes')
    f,l,y=values
    if fine:
        import lateral_capability as proof
        selected=proof.fine_profile(fine_forward_profile)
        if msg['primitive_phase']!='forward_terminal_fine' or msg['fine_forward_subcapability']!=selected['subcapability'] or f!=selected['selected_fine_forward_mps']/2. or l!=0. or y!=0.:raise ValueError('fine phase/value differs from exact root profile')
    elif f not in (0.,.2) or l not in (-.4,0.,.4) or y not in (-.3,0.,.3) or sum(v!=0 for v in values)>1:
        raise ValueError('only exclusive forward+.2/lateral±.4/yaw±.3; no reverse/mixed/micro axes')
    return dict(version=1,action='walk',forward=f,lateral=l,yaw=y)

def run_loop(sock,stream,ready,clock=time,selector=select.select,parent_live=lambda:True,fine_forward_profile=None,endpoint=('127.0.0.1',7447)):
    if (type(endpoint) is not tuple or len(endpoint)!=2 or type(endpoint[0]) is not str or type(endpoint[1]) is not int
        or endpoint not in (('127.0.0.1',7447),('192.168.1.219',7447))):raise ValueError('exact SIM endpoint tuple required')
    current=ZERO;issued_ns=None;last_valid=None;accepted=0;rejected=0;expired=0;clock_pair=None
    def clock_check(unix_ns,steady):
        nonlocal clock_pair
        if clock_pair is not None and (steady<clock_pair[1] or abs((unix_ns-clock_pair[0])/1e9-(steady-clock_pair[1]))>.05):raise ValueError('relay wall/steady clock step')
        clock_pair=(unix_ns,steady)
    try:
        ready()
        while True:
            if not parent_live():raise ValueError('exact parent exited/changed')
            unix_ns,steady=clock.time_ns(),clock.monotonic();clock_check(unix_ns,steady)
            timeout=SELECT_MAX_SEC
            if current!=ZERO:
                timeout=max(0.,min(timeout,last_valid+WATCHDOG_SEC-steady,(issued_ns+PACKET_AGE_NS-unix_ns)/1e9))
            readable,_,_=selector([stream],[],[],timeout)
            if readable:
                line=stream.readline()
                if not line:break
                try:
                    msg=json.loads(line);current=validated_axes(msg,clock.time_ns(),fine_forward_profile);issued_ns=msg['issued_unix_ns'];last_valid=clock.monotonic();accepted+=1
                except (ValueError,TypeError,KeyError,json.JSONDecodeError):current=ZERO;issued_ns=None;rejected+=1
            # Acceptance is not a fresh timestamp. Recheck original expiry and
            # watchdog immediately before each positive send, including replay.
            unix_ns,steady=clock.time_ns(),clock.monotonic();clock_check(unix_ns,steady)
            if current!=ZERO and (steady-last_valid>=WATCHDOG_SEC or not 0<=unix_ns-issued_ns<PACKET_AGE_NS):
                current=ZERO;issued_ns=None;expired+=1
            sock.sendto(json.dumps(current,separators=(',',':'),allow_nan=False).encode(),endpoint)
    finally:
        # EOF, rejected packet, parent failure and normal SIGINT all retain zero cleanup.
        try:
            for _ in range(5):sock.sendto(json.dumps(ZERO,separators=(',',':')).encode(),endpoint);clock.sleep(.02)
        finally:sock.close()
    return dict(accepted=accepted,rejected=rejected,expired_before_send=expired,stopped_with_zero=True)

def admit(path,digest):
    sys.path.insert(0,str(Path(__file__).resolve().parent))
    import lateral_capability as proof
    if not path.is_absolute() or proof.sha(path)!=digest:raise ValueError('frozen relay admission changed')
    a=proof.read(path);release_path=Path(a['root_release_path']);r=proof.read(release_path)
    proof.fine_admission_profile(a,r)
    rid=r.get('release_id','')
    if (proof.sha(release_path)!=a['root_release_sha256'] or r.get('allow_execute') is not True or r.get('scope')!=SCOPE
        or r.get('mode')!='controlled_static_sim_scene' or r.get('lateral_capability')!=CAPABILITY
        or len(rid)!=32 or any(ch not in '0123456789abcdef' for ch in rid)):
        raise ValueError('root once release/capability missing')
    if not release_path.is_absolute() or r.get('output_directory')!=str(path.parent):raise ValueError('relay output differs from same root release')
    if os.environ.get('MAPPING_OWNER_RUN_ID')!=r['release_id'] or os.environ.get('MAPPING_OWNER_OUTPUT')!=str(path.parent):raise ValueError('relay root environment differs')
    if a['owner_identity']['pid']!=os.getppid() or identity(os.getppid())!=a['owner_identity']:raise ValueError('exact admitting parent missing')
    marker=proof.read(release_path.parent/(r['release_id']+'.mapping_once.json'))
    if marker.get('release_sha256')!=a['root_release_sha256'] or marker.get('output_directory')!=str(path.parent) or marker.get('owner_identity')!=a['owner_identity']:
        raise ValueError('same once-admitted exact owner missing')
    if r['sources_sha256']['mapping_lateral_relay.py']!=proof.sha(__file__) or r['sources_sha256']['lateral_capability.py']!=proof.sha(Path(__file__).with_name('lateral_capability.py')):raise ValueError('frozen relay/proof module differs')
    if r['sources_sha256']['full_scene_mapping_contract.py']!=proof.sha(Path(__file__).with_name('full_scene_mapping_contract.py')):raise ValueError('frozen endpoint contract module differs')
    import full_scene_mapping_contract as endpoint_contract
    endpoint_contract.sim_endpoint_from_admission(a,r)
    admitted=proof.validate_proof(r['lateral_calibration_proof'],r['scene_instance_id'],r['sensor_config_sha256'],r['sensor_adapter_process_identity'])
    proof.validate_model_evidence(a['motion_model'],admitted)
    if identity(r['sensor_adapter_process_identity']['pid'])!=r['sensor_adapter_process_identity']:raise ValueError('calibrated adapter changed')
    return a

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--execute',action='store_true')
    parser.add_argument('--admission',type=Path);parser.add_argument('--admission-sha256');args=parser.parse_args()
    if not args.execute:print('ROOT_HOLD: no socket created');return 0
    if args.admission is None:parser.error('same root release/actual calibration admission required')
    a=admit(args.admission,args.admission_sha256)
    descriptor=os.open(args.admission.parent/'relay_start.once',os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
    with os.fdopen(descriptor,'w') as stream:
        stream.write(json.dumps(dict(admission_sha256=args.admission_sha256,parent=a['owner_identity']))+'\n');stream.flush();os.fsync(stream.fileno())
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.settimeout(SOCKET_TIMEOUT_SEC)
    endpoint=(a['sim_command_endpoint']['ipv4'],a['sim_command_endpoint']['port'])
    ready=READY.rsplit(' ',1)[0]+' '+endpoint[0]+':'+str(endpoint[1])
    result=run_loop(sock,sys.stdin,lambda:print(ready,flush=True),parent_live=lambda:identity(os.getppid())==a['owner_identity'],fine_forward_profile=a.get('fine_forward_profile'),endpoint=endpoint)
    print(json.dumps(result),flush=True);return 0

if __name__=='__main__':raise SystemExit(main())

#!/usr/bin/python3
"""Thin sendto-timing wrapper; official tool bytes and finally remain unchanged."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time

import lateral_contract as c
import recording_deadline as deadline


class TimedSocket:
    def __init__(self,socket,sign,stream,before_first,before_nonzero=lambda:None):
        self.socket,self.sign,self.stream,self.before_first=socket,sign,stream,before_first
        self.before_nonzero=before_nonzero
        self.first=None
    def close(self):return self.socket.close()
    def sendto(self,payload,target):
        command=json.loads(payload);c.permitted_payload(command,self.sign)
        if target!=('127.0.0.1',7447):raise c.Hold('only frozen loopback7447 target')
        if command['LX']:
            if self.first is None:self.before_first();self.first=time.monotonic()
            if time.monotonic()-self.first>=1.:raise c.Hold('one-second motion-send limit')
            self.before_nonzero()
        before_unix=time.time_ns();before_steady=time.monotonic_ns()
        count=self.socket.sendto(payload,target)
        after_steady=time.monotonic_ns();after_unix=time.time_ns()
        self.stream.write(json.dumps(dict(event='actual_socket_sendto_interval_not_actuation',
            send_before_unix_ns=before_unix,send_after_unix_ns=after_unix,
            send_before_steady_ns=before_steady,send_after_steady_ns=after_steady,
            target=list(target),bytes=count,payload_sha256=hashlib.sha256(payload).hexdigest(),command=command))+'\n')
        self.stream.flush()
        return count


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute',action='store_true');p.add_argument('--case',type=Path)
    p.add_argument('--root-release',type=Path);p.add_argument('--ready',type=Path)
    p.add_argument('--ready-sha256');p.add_argument('--probe-index',type=int,choices=[0,1])
    args=p.parse_args()
    if not args.execute:
        print(json.dumps(dict(state='ROOT_HOLD',no_socket_created=True)));return 0
    case,release=c.read(args.case),c.read(args.root_release);output=Path(release['output_directory'])
    c.validate(case,release,args.case,output)
    ready=c.read(args.ready)
    if (args.ready!=output/'recorder_ready.json' or c.sha(args.ready)!=args.ready_sha256
        or ready.get('release_id')!=release['release_id'] or ready.get('mcap_magic_verified') is not True
        or ready.get('deadline_ack_verified') is not True):raise c.Hold('bound complete recorder readiness missing')
    def live():
        c.host_proof(release,case)
        if deadline.process_identity(ready['recorder_identity']['pid'])!=ready['recorder_identity']:raise c.Hold('owned recorder not live')
        if deadline.process_identity(ready['helper_identity']['pid'])!=ready['helper_identity']:raise c.Hold('independent helper not live')
        if time.clock_gettime(time.CLOCK_BOOTTIME)>=ready['deadline_boottime_sec']-10.:raise c.Hold('insufficient capped recording time')
    def fresh_parent():
        lease=c.read(output/'observation_lease.json')
        if (lease.get('release_id')!=release['release_id'] or lease.get('parent_identity')!=ready['parent_identity']
            or deadline.process_identity(ready['parent_identity']['pid'])!=ready['parent_identity']
            or not 0<=(time.monotonic_ns()-lease['written_steady_ns'])/1e9<=.15
            or not isinstance(lease.get('anchor'),list) or len(lease['anchor'])!=3):raise c.Hold('parent observation lease missing/stale/exited')
        c.check_live(lease['samples'],lease['anchor'],time.time_ns(),time.monotonic_ns())
    live()
    if not (output/f'probe_{args.probe_index}.planned.json').is_file():raise c.Hold('parent probe plan absent')
    c.claim(output/f'probe_{args.probe_index}.tool_once.json',dict(release_id=release['release_id'],probe=args.probe_index,
        identity=deadline.process_identity(__import__('os').getpid()),claimed_unix_ns=time.time_ns()))
    sign=case['probes'][args.probe_index]
    spec=importlib.util.spec_from_file_location('frozen_official_matrix_mc',c.HERE/'matrix_mc_udp_test.py')
    official=importlib.util.module_from_spec(spec);spec.loader.exec_module(official)
    original_init=official.UdpRobotController.__init__
    with (output/f'probe_{args.probe_index}_send.jsonl').open('x') as stream:
        def timed_init(self,*arguments,**keywords):
            original_init(self,*arguments,**keywords)
            self.socket=TimedSocket(self.socket,sign,stream,live,fresh_parent)
        official.UdpRobotController.__init__=timed_init
        sys.argv=[str(c.HERE/'matrix_mc_udp_test.py'),'--host','127.0.0.1','--port','7447','--rate','20',
                  '--verbose','walk','--forward','0','--lateral',str(sign),'--yaw','0','--duration','1']
        return official.main()


if __name__=='__main__':raise SystemExit(main())

#!/usr/bin/python3
"""One read-only host snapshot; external evidence, never a host monitor."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import socket
import subprocess
import time
import lateral_contract as c


def identity(pid):
    p=Path('/proc')/str(pid);fields=(p/'stat').read_text().rsplit(')',1)[1].split();raw=(p/'cmdline').read_bytes()
    status=(p/'status').read_text().splitlines()
    def values(key):return [int(v) for v in next(x for x in status if x.startswith(key+':')).split()[1:]]
    return dict(pid=pid,starttime_ticks=int(fields[19]),state=fields[0],cmdline_sha256=hashlib.sha256(raw).hexdigest(),
        argv=[x.decode(errors='backslashreplace') for x in raw.rstrip(b'\0').split(b'\0')] if raw else [],
        boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),uid=values('Uid'),gid=values('Gid'),
        namespace_pids=values('NSpid'),pid_namespace=os.readlink(p/'ns/pid'),mount_namespace=os.readlink(p/'ns/mnt'),
        executable=os.readlink(p/'exe') if fields[0]!='Z' else None,cwd=os.readlink(p/'cwd') if fields[0]!='Z' else None,
        cgroup_sha256=hashlib.sha256((p/'cgroup').read_bytes()).hexdigest())


def exact(expected):
    actual=identity(expected['pid'])
    if actual['state']=='Z' or any(actual.get(k)!=v for k,v in expected.items()):raise c.Hold('full host identity changed: '+str(expected['pid']))
    return actual


def retired(expected):
    try:actual=identity(expected['pid'])
    except FileNotFoundError:return dict(pid=expected['pid'],state='ABSENT',original_retired=True)
    if actual['starttime_ticks']!=expected['starttime_ticks'] or actual['boot_id']!=expected['boot_id']:raise c.Hold('retired numeric PID reused; root must refresh evidence')
    if actual['state']!='Z':raise c.Hold('old command/native worker still live')
    return dict(pid=expected['pid'],state='Z',starttime_ticks=actual['starttime_ticks'],original_retired=True)


def port_owner(scene_pid):
    sockets=set()
    for fd in (Path('/proc')/str(scene_pid)/'fd').iterdir():
        try:target=os.readlink(fd)
        except FileNotFoundError:continue
        if target.startswith('socket:['):sockets.add(target[8:-1])
    rows=[]
    for proto in ['tcp','udp','tcp6','udp6']:
        for line in (Path('/proc/net')/proto).read_text().splitlines()[1:]:
            parts=line.split()
            if int(parts[1].split(':')[1],16)!=7447 or proto.startswith('tcp') and parts[3]!='0A':continue
            rows.append(dict(protocol=proto,local=parts[1],state=parts[3],inode=parts[9],owned_by_frozen_scene=parts[9] in sockets))
    if not {'tcp','udp'}<={x['protocol'] for x in rows} or not all(x['owned_by_frozen_scene'] for x in rows):raise c.Hold('TCP/UDP7447 not exclusively owned by exact new scene')
    return rows


def forbidden_live(proc=Path('/proc')):
    names={'run_loc_online','sim_command_adapter.py','sim_udp_relay.py','full_scene_mapping.py','matrix_navigation.py',
        'map_server','planner_server','controller_server','bt_navigator','behavior_server','velocity_smoother',
        'nav_safety_gate.py','waypoint_follower','waypoint_batch.py',
        'matrix_mc_udp_test.py','official_tool_timed.py','semantic_tool_timed.py','lateral_once.py','sim_udp_relay_frozen.py'}
    bad=[]
    for path in proc.iterdir():
        if not path.name.isdigit():continue
        try:
            status=(path/'status').read_text().splitlines()
            uid=int(next(x for x in status if x.startswith('Uid:')).split()[1])
            if uid!=1000:continue
            fields=(path/'stat').read_text().rsplit(')',1)[1].split()
            if fields[0]=='Z':continue
            argv=(path/'cmdline').read_bytes().rstrip(b'\0').split(b'\0')
            texts=[x.decode(errors='backslashreplace') for x in argv]
            if (any(Path(x).name in names for x in texts) or any('rosbag2_transport/record' in x for x in texts)
                or any(texts[i:i+2]==['bag','record'] for i in range(len(texts)-1))):
                bad.append(dict(pid=int(path.name),argv=texts,starttime_ticks=int(fields[19])))
        except FileNotFoundError:continue
    return bad


def collect(config):
    if config.get('root_inputs_reviewed') is not True:raise c.Hold('host input remains ROOT_HOLD')
    expected=config['expected_host'];boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    if os.getuid()!=1000 or expected['uid']!=1000 or boot!=expected['boot_id'] or socket.gethostname()!=expected['hostname'] or os.readlink('/proc/self/ns/pid')!=expected['pid_namespace']:raise c.Hold('wrong host UID/boot/PID namespace')
    scene=exact(config['expected_scene_identity']);init=exact(config['expected_container_init'])
    if scene['pid']!=1172665 or scene['starttime_ticks']!=9427026:raise c.Hold('only exact new UE allowed')
    fmt='{"id":{{json .Id}},"pid":{{json .State.Pid}},"running":{{json .State.Running}},"network":{{json .HostConfig.NetworkMode}}}'
    container=json.loads(subprocess.check_output(['/usr/bin/docker','inspect','--format',fmt,config['expected_container_id']],timeout=10))
    net=os.readlink('/proc/self/ns/net');container_net=os.readlink(Path('/proc')/str(init['pid'])/'ns/net')
    if container!={'id':config['expected_container_id'],'pid':init['pid'],'running':True,'network':'host'} or container_net!=net:raise c.Hold('container ID/running/shared-net differs')
    old=[dict(role=item['role'],**retired(item['identity'])) for item in config['retired_workers']]
    if set(x['role'] for x in old)!={'63','123','2689'}:raise c.Hold('three exact retired worker identities required')
    ports=port_owner(scene['pid']);bad=forbidden_live()
    if bad:raise c.Hold('Nav/recorder/command writer remains live: '+json.dumps(bad))
    artifacts=[]
    if not config['artifacts']:raise c.Hold('root package/config/datafreeze artifact list required')
    for item in config['artifacts']:
        if not Path(item['path']).is_absolute() or c.sha(item['path'])!=item['sha256']:raise c.Hold('host source/datafreeze artifact changed')
        artifacts.append(item)
    exact(config['expected_scene_identity']);exact(config['expected_container_init'])
    return dict(state='PASS',mode='root_external_host_attestation',completed_unix_ns=time.time_ns(),
        boot_id=boot,uid=1000,host_pid_namespace=os.readlink('/proc/self/ns/pid'),network_namespace=net,
        container_id=container['id'],container_pid_namespace=init['pid_namespace'],shared_host_network=True,
        scene_identity=config['expected_scene_identity'],port7447_exact_scene_owner=True,ports=ports,
        retired_workers_verified=True,retired_workers=old,forbidden_live_processes=bad,process_scan_scope='host UID1000 processes',
        artifacts_verified=artifacts,collector_sha256=c.sha(__file__),
        evidence_boundary='Instantaneous external host snapshot; no continuous host monitoring, no mount or runtime controls.')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--input',required=True,type=Path)
    p.add_argument('--output',required=True,type=Path);p.add_argument('--collect',action='store_true');args=p.parse_args()
    if not args.collect:print(json.dumps(dict(state='ROOT_HOLD',no_subprocess_or_signal=True)));return 0
    result=collect(c.read(args.input));c.claim(args.output,result)
    print(json.dumps(dict(state='PASS',proof_sha256=c.sha(args.output),completed_unix_ns=result['completed_unix_ns'])));return 0


if __name__=='__main__':raise SystemExit(main())

#!/usr/bin/python3
"""Default-HOLD entry. Root must plan the sole adapter60 replacement."""
import argparse
import hashlib
import json
import os
from pathlib import Path

from imu_gate import SCOPE

HERE=Path(__file__).resolve().parent
FILES=('sim_imu_adapter.py','adapter_with_gate.py','imu_gate.py','matrix_sensor_contract.py')

def sha(path):
    with Path(path).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()

def identity(pid):
    p=Path('/proc')/str(pid);fields=(p/'stat').read_text().rsplit(')',1)[1].split();raw=(p/'cmdline').read_bytes()
    if fields[0]=='Z' or not raw:raise ProcessLookupError('exited')
    return dict(pid=pid,starttime_ticks=int(fields[19]),cmdline_sha256=hashlib.sha256(raw).hexdigest(),
        argv=[v.decode() for v in raw.rstrip(b'\0').split(b'\0')],boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip())

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--execute',action='store_true')
    p.add_argument('--runtime-config',type=Path);p.add_argument('--root-release',type=Path);args=p.parse_args()
    if not args.execute:
        print(json.dumps(dict(state='ROOT_HOLD',scope=SCOPE,no_ROS_or_Popen_or_source_action=True,
            sources_sha256={n:sha(HERE/n) for n in FILES})));return 0
    if args.root_release is None or args.runtime_config is None:p.error('separate root release/runtime config required')
    r=json.loads(args.root_release.read_text())
    if r.get('scope')!=SCOPE or r.get('allow_execute') is not True:raise ValueError('ROOT_HOLD')
    if r.get('sources_sha256')!={n:sha(HERE/n) for n in FILES} or r.get('runtime_config_sha256')!=sha(args.runtime_config):raise ValueError('frozen source/config differs')
    if (os.getuid()!=1000 or r.get('uid')!=1000 or r.get('boot_id')!=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        or r.get('pid_namespace')!=os.readlink('/proc/self/ns/pid') or r.get('network_namespace')!=os.readlink('/proc/self/ns/net')
        or os.environ.get('ROS_DOMAIN_ID')!='185'):raise ValueError('same SIM container identity differs')
    if not r.get('old_adapter60_retired') or not r.get('bridge24_projector61_preserved'):raise ValueError('planned replacement proof required')
    old=r['old_adapter_identity']
    try:live=identity(old['pid'])
    except (FileNotFoundError,ProcessLookupError):live=None
    if live is not None:raise ValueError('old adapter PID still live/reused; refuse duplicate publishers')
    for expected in r['retained_source_identities']:
        if identity(expected['pid'])!=expected:raise ValueError('retained raw bridge/projector identity differs')
    if len(r['retained_source_identities'])!=2:raise ValueError('exact bridge24/projector61 required')
    rid=r.get('release_id','')
    if len(rid)!=32 or any(x not in '0123456789abcdef' for x in rid):raise ValueError('unique replacement release ID required')
    fd=os.open(args.root_release.resolve().parent/(rid+'.imu_adapter_once.json'),os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
    with os.fdopen(fd,'w') as stream:
        stream.write(json.dumps(dict(scope=SCOPE,release_sha256=sha(args.root_release),sources_sha256=r['sources_sha256']))+'\n')
        stream.flush();os.fsync(stream.fileno())
    import adapter_with_gate
    adapter_with_gate.run(args.runtime_config)
    return 0

if __name__=='__main__':raise SystemExit(main())

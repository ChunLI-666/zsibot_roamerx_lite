#!/usr/bin/env python3
"""Exec an unchanged navigation command with optional explicit CPU affinity.

Usage: run_budget [--cpu-list 4-7] -- ros2 launch robot_navigo ...
Without --cpu-list, no affinity query/set or audit output is performed. This
entry is opt-in; it does not change ordinary launch files or the environment.
"""
import argparse
import hashlib
from pathlib import Path
import platform
import json
import os
import re
import sys


def parse_cpu_list(text, allowed):
    """Accept a nonempty explicit Linux CPU list within the inherited mask."""
    if not isinstance(text, str) or not text or not allowed:
        raise ValueError('CPU list and inherited allowed mask must be nonempty')
    maximum = max(allowed)
    requested = set()
    for item in text.split(','):
        match = re.fullmatch(r'([0-9]+)(?:-([0-9]+))?', item)
        if match is None:
            raise ValueError('Invalid CPU-list token: ' + item)
        first = int(match[1])
        last = int(match[2]) if match[2] is not None else first
        # Check the endpoints before expanding, including very large ranges.
        if first > last or last > maximum:
            raise ValueError('CPU range is descending or exceeds the inherited mask')
        values = set(range(first, last + 1))
        if requested & values or not values <= allowed:
            raise ValueError('CPU list overlaps or requests CPUs outside the inherited mask')
        requested.update(values)
    return requested


# All checks are opt-in. Default path performs no metadata/profile query.
def require(value,message):
    if not value:raise ValueError(message)
def file_sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

# This profile is opt-in and currently validated only on CI6 x86/Jazzy with
# rmw_fastrtps_cpp8.4.4 and Fast DDS2.14.6. It changes participant transport,
# not ROS endpoint QoS, sampled headers, source pacing, or the native algorithm.
DDS_UDP_XML_SHA='78e6281cc011d0b164647b539e41074666e684bffca30aed45814d4221359a23'
def dds_udp_profile_path():
    here=Path(__file__).absolute()
    if here.parent.name=='robot_navigo'and here.parent.parent.name=='lib':
        return here.parent/'config/performance/ci6_udp_loopback.xml'
    require(here.parent.name=='performance'and here.parent.parent.name=='scripts',
            'Unsupported budget-entry layout for the fixed DDS profile')
    return here.parents[2]/'config/performance/ci6_udp_loopback.xml'
def dds_middleware_identity():
    import re
    import xml.etree.ElementTree as ET
    from ament_index_python.packages import get_package_prefix
    from rclpy.utilities import get_rmw_implementation_identifier
    try:
        implementation=get_rmw_implementation_identifier()
        require(implementation=='rmw_fastrtps_cpp','DDS profile requires rmw_fastrtps_cpp')
        prefix=Path(get_package_prefix('rmw_fastrtps_cpp'))
        rmw=prefix/'share/rmw_fastrtps_cpp/package.xml'
        version=ET.parse(rmw).getroot().findtext('version')
        fast=prefix/'share/fastrtps/cmake/fastrtps-config-version.cmake'
        match=re.findall(r'^set\(PACKAGE_VERSION "([^"]+)"\)',fast.read_text(),re.MULTILINE)
        require(version=='8.4.4'and match==['2.14.6'],
                'DDS profile is validated only for rmw_fastrtps_cpp8.4.4/Fast DDS2.14.6')
    except (RuntimeError,ET.ParseError)as error:
        raise ValueError('Cannot verify DDS middleware identity: '+str(error))from error
    return dict(rmw_identifier_in_budget_process=implementation,rmw_version=version,
        FastDDS_version=match[0],rmw_package=dict(path=str(rmw),sha256=file_sha(rmw)),
        FastDDS_version_file=dict(path=str(fast),sha256=file_sha(fast)),
        scope='Budget-process implementation selection and installed metadata; actual native transport still requires runtime proof.')
def dds_udp_environment(env):
    require(sys.platform.startswith('linux')and platform.machine()=='x86_64'and env.get('ROS_DISTRO')=='jazzy',
            'DDS UDP loopback profile is validated only on CI6 Linux x86_64/Jazzy')
    for name in('FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE',
                'RMW_FASTRTPS_USE_QOS_FROM_XML','RMW_FASTRTPS_PUBLICATION_MODE',
                'ROS_STATIC_PEERS','FASTDDS_ENVIRONMENT_FILE','FASTDDS_BUILTIN_TRANSPORTS'):
        require(name not in env,'Conflicting middleware override: '+name)
    require(env.get('RMW_IMPLEMENTATION')in(None,'','rmw_fastrtps_cpp'),'Conflicting RMW implementation')
    require(env.get('ROS_LOCALHOST_ONLY')in(None,'0','1')and
            env.get('ROS_AUTOMATIC_DISCOVERY_RANGE')in(None,'LOCALHOST','SYSTEM_DEFAULT'),
            'Conflicting discovery scope for the CI6 loopback profile')
    for name in('DEFAULT_FASTRTPS_PROFILES.xml','DEFAULT_FASTDDS_PROFILES.xml'):
        require(not Path(name).exists(),'Implicit CWD DDS profile forbidden: '+name)
    profile=dds_udp_profile_path()
    require(profile.is_file()and file_sha(profile)==DDS_UDP_XML_SHA,'Fixed installed DDS XML missing or changed')
    identity=dds_middleware_identity();result=dict(env)
    result.pop('ROS_LOCALHOST_ONLY',None)
    result.update(ROS_AUTOMATIC_DISCOVERY_RANGE='SYSTEM_DEFAULT',FASTRTPS_DEFAULT_PROFILES_FILE=str(profile))
    keys=('ROS_LOCALHOST_ONLY','ROS_AUTOMATIC_DISCOVERY_RANGE','FASTRTPS_DEFAULT_PROFILES_FILE')
    require({k:v for k,v in env.items()if k not in keys}=={k:v for k,v in result.items()if k not in keys},
            'DDS profile changed a nontransport environment variable')
    return result,dict(schema='robot_navigo.dds_udp_loopback.v1',enabled=True,
        XML=dict(path=str(profile),sha256=DDS_UDP_XML_SHA),middleware=identity,
        environment_before={k:env.get(k)for k in keys},environment_effective={k:result.get(k)for k in keys},
        RMW_environment_unchanged=env.get('RMW_IMPLEMENTATION')==result.get('RMW_IMPLEMENTATION'),
        endpoint_QoS_and_algorithm_unchanged=True,actual_native_transport_proof_required=True,
        scope='Explicit CI6 participant-only UDPv4 loopback; no ARM, accuracy, frequency or CPU acceptance inferred.')

class OnceFlag(argparse.Action):
    def __call__(self,parser,namespace,value,option_string=None):
        marker='_seen_'+self.dest
        if getattr(namespace,marker,False):parser.error('Duplicate wrapper option: '+option_string)
        setattr(namespace,marker,True);setattr(namespace,self.dest,True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--cpu-list', action='append', metavar='LIST')
    parser.add_argument('--dds-udp-loopback-profile', action=OnceFlag, nargs=0, default=False)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    command = args.command
    if command and command[0] == '--':
        command = command[1:]
    if not command or not command[0]:
        raise ValueError('An original navigation command is required after --')
    if args.dds_udp_loopback_profile:
        transport_env, transport_proof = dds_udp_environment(os.environ)
    if args.cpu_list is not None:
        if len(args.cpu_list) != 1:
            raise ValueError('Specify --cpu-list exactly once')
        allowed_before = os.sched_getaffinity(0)
        requested = parse_cpu_list(args.cpu_list[0], allowed_before)
        os.sched_setaffinity(0, requested)
        actual = os.sched_getaffinity(0)
        if actual != requested:
            raise ValueError('Actual CPU affinity differs from the requested mask')
        proof = dict(schema=1, enabled=True, pid=os.getpid(),
                     requested_cpus=sorted(requested),
                     allowed_before=sorted(allowed_before),
                     actual_cpus=sorted(actual), command=command)
        print('[NAV_CPU_BUDGET] ' + json.dumps(proof, sort_keys=True),
              file=sys.stderr, flush=True)
    # Replace this process: retain PID/descriptors/environment/CWD without an
    # extra child, process group, or signal-forwarding proxy.
    if args.dds_udp_loopback_profile:
        transport_proof.update(pid=os.getpid(), command=command, cwd=os.getcwd(),
                               scope='Explicit CI6 Nav launch tree participant UDPv4 loopback, including standalone smoother; native child environment/owned sockets require runtime verification. No CPU, precision or reliability acceptance inferred.')
        print('[NAV_DDS_UDP_LOOPBACK] ' + json.dumps(transport_proof, sort_keys=True),
              file=sys.stderr, flush=True)
        os.execvpe(command[0], command, transport_env)
        return 0
    os.execvp(command[0], command)
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (ValueError, OSError) as error:
        print('run_budget: ' + str(error), file=sys.stderr)
        raise SystemExit(2)

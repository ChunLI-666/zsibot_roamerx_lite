"""Versioned, hash-bound inputs for the private Matrix simulation workflow.

Candidate acceptance binds software, not permission. Source checks establish
live process/lease identity; the native client still checks every fresh input.
"""
import hashlib
import json
import math
import os
from pathlib import Path
import re
import time

HASH=re.compile(r'^[0-9a-f]{64}$')
COMMIT=re.compile(r'^[0-9a-f]{40}$')


def file_sha(path):
    with Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


def pin(path):
    path=Path(path).resolve(strict=True)
    return dict(path=str(path),sha256=file_sha(path))


def checked_file(reference):
    path=Path(reference['path']);expected=reference['sha256']
    if not path.is_absolute() or not HASH.fullmatch(expected) or file_sha(path)!=expected:
        raise ValueError('Input hash/path differs: '+str(path))
    return path


def checked_json(reference):
    path=Path(reference['path']);raw=path.read_bytes()
    if not path.is_absolute() or not HASH.fullmatch(reference['sha256']) or hashlib.sha256(raw).hexdigest()!=reference['sha256']:
        raise ValueError('JSON input differs: '+str(path))
    return json.loads(raw)


def candidate_binding(candidate):
    value=dict(variant=candidate['variant'],commit=candidate['commit'],
        source_files={row['name']:row['sha256'] for row in candidate['source_files']},
        build_receipts=sorted(row['sha256'] for row in candidate['build_receipts']))
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':')).encode()).hexdigest()


def candidate_check(path):
    candidate=json.loads(Path(path).read_text())
    if (candidate.get('schema')!='matrix.simulation.candidate.v1' or candidate.get('status')!='ACCEPTED'
            or candidate.get('variant') not in ('official','segmented') or not COMMIT.fullmatch(candidate.get('commit',''))):
        raise ValueError('Accepted primary candidate with full commit required')
    scope=candidate.get('scope',{})
    if scope.get('environment')!='matrix_private_simulation' or scope.get('ros_domain_id')!=185:
        raise ValueError('Candidate scope differs from the private simulation')
    for key in ('source_files','build_receipts','code_reviews','software_tests'):
        if not isinstance(candidate.get(key),list) or not candidate[key]:raise ValueError('Missing candidate evidence: '+key)
        for reference in candidate[key]:checked_file(reference)
    names=set()
    for reference in candidate['source_files']:
        name=Path(reference['name'])
        if name.is_absolute() or '..' in name.parts or str(name) in names or not name.parts:
            raise ValueError('Unique repository-relative source names required')
        names.add(str(name))
    # Typed code/test receipt references are not replaced by a ready/verified flag.
    binding=candidate_binding(candidate)
    for key,schema,field,value in (
            ('code_reviews','matrix.simulation.code_review.v1','decision','APPROVED'),
            ('software_tests','matrix.simulation.software_tests.v1','status','PASS')):
        for reference in candidate[key]:
            receipt=checked_json(reference)
            if (receipt.get('schema')!=schema or receipt.get(field)!=value
                    or receipt.get('candidate_binding_sha256')!=binding):
                raise ValueError('Candidate evidence does not accept these exact sources/builds: '+key)
    if candidate.get('client_request_idempotency_tested') is not True:
        raise ValueError('Durable client idempotency evidence required')
    return candidate


def entrypoint_check(plan,candidate):
    accepted={str(Path(row['path']).resolve()):row['sha256'] for row in candidate['source_files']}
    entries=candidate['entrypoints'];workflow=Path(__file__).resolve().parent
    for name in ('workflow.py','workflow_inputs.py','workflow_driver.py','workflow_worker.py','mission_ledger.py'):
        path=str(workflow/name)
        if entries.get(name)!=path or accepted.get(path)!=file_sha(path):
            raise ValueError('Executing workflow dependency is not the accepted source: '+name)
    client=Path(entries['mission_client']).resolve(strict=True)
    if client!=workflow.parent/'goal_pilot_source/mission_resume.py':
        raise ValueError('Canonical mission client from the same candidate source tree required')
    for path in client.parent.glob('*.py'):
        if accepted.get(str(path.resolve()))!=file_sha(path):raise ValueError('Mission dependency is not accepted: '+str(path))
    for stage in plan['stages']:
        for reference in stage.get('files',[]):
            path=Path(reference['path']).resolve()
            if path.suffix=='.py' and accepted.get(str(path))!=reference['sha256']:
                raise ValueError('Stage code is outside the accepted source table: '+str(path))
        if stage['kind']=='client':
            prefix=stage['client']['prefix']
            if (len(prefix) not in (2,3) or Path(prefix[-1]).resolve()!=client
                    or (len(prefix)==3 and prefix[1]!='-B')
                    or not re.fullmatch(r'python3(?:\.\d+)?',Path(prefix[0]).name)):
                raise ValueError('Client stage must invoke the accepted canonical mission client')


def process_check(expected):
    pid=expected['pid']
    if type(pid) is not int or pid<=0:raise ValueError('Invalid process id')
    proc=Path('/proc')/str(pid);stat=(proc/'stat').read_text().rsplit(')',1)[1].split()
    raw=(proc/'cmdline').read_bytes()
    actual=dict(pid=pid,starttime_ticks=int(stat[19]),cmdline_sha256=hashlib.sha256(raw).hexdigest(),
        boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        pid_namespace=os.readlink(proc/'ns/pid'),network_namespace=os.readlink(proc/'ns/net'))
    # Older actor receipts lack a per-process netns. Still verify the actual
    # process belongs to this private context; source_check binds that context.
    expected=dict(expected,network_namespace=expected.get('network_namespace',os.readlink('/proc/self/ns/net')))
    if stat[0] in ('Z','X') or not raw or any(actual[key]!=expected.get(key) for key in actual):
        raise ValueError('Live process identity differs: '+str(pid))
    return actual


def release_check(source,release,localization_plan,localization_started):
    native=source['native'];role=source['roles']['localization']
    if (release.get('runtime_config_sha256')!=native['runtime_config']['sha256']
            or release.get('localization_plan_sha256')!=role['plan']['sha256']
            or release.get('localization_started_sha256')!=role['started']['sha256']):
        raise ValueError('Localization release does not belong to this actor/config')
    child=localization_started['child']
    keys=('pid','starttime_ticks','cmdline_sha256','boot_id','pid_namespace')
    if any(native['process'].get(key)!=child.get(key) for key in keys):
        raise ValueError('Native process is not the owned localization child')
    if (str(Path(localization_plan['argv'][0]).resolve())!=str(Path(native['binary']['path']).resolve())
            or native['runtime_config']['path'] not in localization_plan['argv']):
        raise ValueError('Localization plan does not launch the declared Native/config')
    inputs={(row['path'],row['sha256']) for row in localization_plan['inputs']}
    if any((native[key]['path'],native[key]['sha256']) not in inputs for key in ('binary','runtime_config')):
        raise ValueError('Localization inputs do not pin Native/config')
    if native['library']['sha256'] not in {row['sha256'] for row in localization_plan['inputs']}:
        raise ValueError('Localization plan does not pin the loaded Native library')
    capture=checked_json(release['stationary_capture']);fit=checked_json(release['stationary_scanfit'])
    pairs=fit.get('pairs',[])
    if (capture.get('schema')!='terra.source11.stationary_capture.v1'
            or fit.get('schema')!='terra.source.stationary_native_scanfit.v1'
            or fit.get('stationary_capture_sha256')!=release['stationary_capture']['sha256']
            or type(fit.get('valid_pair_count')) is not int or fit['valid_pair_count']<3
            or fit['valid_pair_count']!=len(pairs) or fit.get('rejected_pairs')!=[]
            or fit.get('status_values')!=[1] or fit.get('valid_values')!=[True]
            or capture.get('status_values')!=[1] or capture.get('valid_values')!=[True]):
        raise ValueError('Stationary Native/capture evidence is not the supported accepted proof')
    for pair in pairs:
        fraction=pair.get('fraction_within_0_1m')
        if isinstance(fraction,bool) or not isinstance(fraction,(int,float)) or not math.isfinite(fraction) or not .9<=fraction<=1:
            raise ValueError('Stationary scanfit did not meet its existing .9 requirement')
        matching=[row for row in capture['pairs'] if all(
            row[kind].get('header_ns')==pair.get(kind+'_header_ns') and
            row[kind].get('cdr_sha256')==pair.get(kind+'_sha256') for kind in ('cloud','pose','imu'))]
        if len(matching)!=1 or any(matching[0][kind].get('received_unix_ns',0)<localization_started['started_ns']
                for kind in ('cloud','pose','imu')):
            raise ValueError('Scanfit pair is not this actor generation\'s original capture')
    ndt=checked_file(release['ndt_confirmation']).read_text().splitlines()
    if not any('confirm: 2/2' in line for line in ndt) or not any('INITIALIZING -> ACTIVE' in line for line in ndt):
        raise ValueError('Native NDT 2/2 and ACTIVE confirmation required')
    # Bind the bounded snapshot to the actual actor log without reading a
    # growing log to an unbounded EOF or copying all of it into memory.
    pending={line for line in ndt if line.strip()};log=Path(localization_plan['output'])/'actor.log'
    with log.open('rb') as stream:
        remaining=os.fstat(stream.fileno()).st_size;tail=b''
        while pending and remaining>0:
            chunk=stream.read(min(1024*1024,remaining));remaining-=len(chunk)
            if not chunk:break
            data=tail+chunk
            pending={line for line in pending if line.encode() not in data}
            tail=data[-max((len(line.encode()) for line in pending),default=0):] if pending else b''
    if pending:raise ValueError('NDT snapshot is not from the owned Native actor log')


def python_actor_check(argv,inputs,interpreter):
    if not argv or not re.fullmatch(r'python3(?:\.\d+)?',Path(argv[0]).name):
        raise ValueError('Missing native executable hash or supported Python actor')
    checked_file(interpreter)
    if file_sha(argv[0])!=interpreter['sha256']:raise ValueError('Python interpreter binding differs')
    args=list(argv[1:])
    while args and args[0] in ('-B','-u'):args.pop(0)
    if not args or not Path(args[0]).is_absolute() or not args[0].endswith('.py'):
        raise ValueError('Python actor must directly execute its pinned script')
    table={row['path']:row['sha256'] for row in inputs}
    if table.get(args[0])!=file_sha(args[0]):raise ValueError('Python actor script is not pinned by its plan')


def source_check(path,minimum_lease_sec=1.,*,live=True,expected_sha256=None):
    raw=Path(path).read_bytes()
    if expected_sha256 is not None and hashlib.sha256(raw).hexdigest()!=expected_sha256:
        raise ValueError('Source generation changed within prepared run')
    source=json.loads(raw)
    if source.get('schema')!='matrix.simulation.source.v1' or not source.get('source_id') or source.get('ros_domain_id')!=185:
        raise ValueError('Unsupported source manifest')
    if not math.isfinite(minimum_lease_sec) or minimum_lease_sec<0:raise ValueError('Invalid minimum lease')
    for key in ('map_identity','config'):checked_file(source[key])
    native=source['native']
    for key in ('binary','library','runtime_config'):checked_file(native[key])
    release=checked_json(source['localization_release'])
    if (release.get('schema')!='matrix.simulation.localization_release.v1'
            or release.get('source_id')!=source['source_id'] or release.get('GT_used') is not False
            or release.get('map_identity_sha256')!=source['map_identity']['sha256']
            or release.get('config_sha256')!=source['config']['sha256']
            or release.get('native_binary_sha256')!=native['binary']['sha256']
            or release.get('native_library_sha256')!=native['library']['sha256']
            or release.get('fresh_scanfit_pass') is not True or release.get('ndt_confirm_2_of_2') is not True):
        raise ValueError('Localization release/source/config/map binding differs')
    for key in ('stationary_capture','stationary_scanfit','ndt_confirmation'):checked_file(release[key])
    roles=source.get('roles',{})
    if set(roles)!={'sensor','localization','floor','command'}:raise ValueError('Four explicit source roles required')
    now=time.clock_gettime(time.CLOCK_BOOTTIME);actors={};command_pids=set();plans={};starts={}
    for role,spec in roles.items():
        plan=checked_json(spec['plan']);started=checked_json(spec['started']);plans[role]=plan;starts[role]=started
        if (plan.get('schema')!='terra.owned_actor.v1' or started.get('schema')!='terra.owned_actor.started.v1'
                or started.get('plan_sha256')!=spec['plan']['sha256'] or started.get('role')!=plan.get('role')
                or started['child'].get('argv')!=plan.get('argv')
                or any(plan.get(key)!=source.get(key) for key in ('boot_id','pid_namespace','network_namespace'))
                or any(started[part].get('boot_id')!=source['boot_id'] or
                    started[part].get('pid_namespace')!=source['pid_namespace'] for part in ('owner','child'))):
            raise ValueError('Typed actor plan/identity binding differs: '+role)
        runtime=plan.get('max_runtime_sec')
        if (isinstance(runtime,bool) or not isinstance(runtime,(int,float)) or not math.isfinite(runtime) or runtime<=0
                or type(started.get('started_ns')) is not int or started['started_ns']<=0
                or started['child'].get('ppid')!=started['owner'].get('pid')):
            raise ValueError('Finite actor lifetime/birth relationship required: '+role)
        for part in ('owner','child'):
            argv=started[part].get('argv',[])
            if (not argv or any(not isinstance(arg,str) or '\0' in arg for arg in argv)
                    or hashlib.sha256(b'\0'.join(arg.encode() for arg in argv)+b'\0').hexdigest()!=started[part].get('cmdline_sha256')):
                raise ValueError('Actor argv digest differs: '+role)
        if not isinstance(plan.get('argv'),list) or not plan['argv']:raise ValueError('Actor argv required: '+role)
        python_actor='executable_sha256' not in plan
        if python_actor:python_actor_check(plan['argv'],plan.get('inputs',[]),spec['interpreter'])
        elif file_sha(plan['argv'][0])!=plan['executable_sha256']:raise ValueError('Actor executable differs: '+role)
        if plan.get('relay_argv'):
            if not isinstance(started.get('relay'),dict) or started['relay'].get('argv')!=plan['relay_argv']:
                raise ValueError('Relay argv differs')
            python_actor_check(plan['relay_argv'],plan.get('inputs',[]),spec['interpreter'])
        for reference in plan.get('inputs',[]):checked_file(reference)
        output=Path(plan['output'])
        if (not output.is_absolute() or Path(spec['started']['path'])!=output/'started.json'
                or Path(spec['result_path'])!=output/'result.json'):
            raise ValueError('Actor output/receipt paths differ: '+role)
        owner_argv=started['owner'].get('argv',[])
        try:
            if (owner_argv[owner_argv.index('--plan')+1]!=spec['plan']['path'] or
                    owner_argv[owner_argv.index('--plan-sha256')+1]!=spec['plan']['sha256']):
                raise ValueError('Actor owner argv differs: '+role)
        except (ValueError,IndexError) as error:raise ValueError('Actor owner does not bind its plan: '+role) from error
        result=Path(spec['result_path'])
        if not result.is_absolute():raise ValueError('Actor result path must be absolute')
        if not live:continue
        if result.exists():raise ValueError('Source actor already closed: '+role)
        deadline=started.get('deadline_boottime_sec')
        if (isinstance(deadline,bool) or not isinstance(deadline,(int,float)) or not math.isfinite(deadline)
                or deadline-now<minimum_lease_sec):raise ValueError('Insufficient finite source lease: '+role)
        row={key:process_check(started[key]) for key in ('owner','child')}
        if python_actor and file_sha(Path('/proc')/str(row['child']['pid'])/'exe')!=spec['interpreter']['sha256']:
            raise ValueError('Live Python actor interpreter differs: '+role)
        if role=='command':
            row['relay']=process_check(started['relay']);command_pids={row['child']['pid'],row['relay']['pid']}
            if file_sha(Path('/proc')/str(row['relay']['pid'])/'exe')!=spec['interpreter']['sha256']:
                raise ValueError('Live command relay interpreter differs')
        row['deadline_boottime_sec']=deadline;actors[role]=row
    release_check(source,release,plans['localization'],starts['localization'])
    if live:
        if os.environ.get('ROS_DOMAIN_ID')!='185':raise ValueError('Private domain185 required')
        if (source['boot_id']!=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
                or source['pid_namespace']!=os.readlink('/proc/self/ns/pid')
                or source['network_namespace']!=os.readlink('/proc/self/ns/net')):
            raise ValueError('Source process context differs')
        for expected in source.get('retained_processes',[]):process_check(expected)
        actual=process_check(native['process']);proc=Path('/proc')/str(actual['pid'])
        if file_sha(proc/'exe')!=native['binary']['sha256']:raise ValueError('Loaded Native binary differs')
        maps=(proc/'maps').read_text().splitlines();library=str(Path(native['library']['path']).resolve(strict=True))
        if not any(line.endswith(' '+library) for line in maps):raise ValueError('Native library is not mapped')
        argv=[arg.decode() for arg in (proc/'cmdline').read_bytes().split(b'\0') if arg]
        if native['runtime_config']['path'] not in argv:raise ValueError('Native runtime config is not in its argv')
        found=set()
        for proc in Path('/proc').iterdir():
            if not proc.name.isdecimal():continue
            try:argv=[x.decode(errors='replace') for x in (proc/'cmdline').read_bytes().split(b'\0') if x]
            except OSError:continue
            if any(Path(arg).name in ('sim_command_adapter.py','sim_udp_relay.py') for arg in argv):found.add(int(proc.name))
        if found!=command_pids:raise ValueError('Command adapter/relay ownership is not exclusive')
    return dict(schema='matrix.simulation.source_check.v1',source_id=source['source_id'],ok=True,live=live,
        source_manifest_sha256=hashlib.sha256(raw).hexdigest(),checked_unix_ns=time.time_ns(),actors=actors,
        boundary='Process/lease and frozen-input admission; native READY and original sensor checks remain mandatory.')


def route_check(route_path,policy_path,map_path):
    # Reuse the actual client route contract; no second visit/goal rule here.
    import sys
    client=Path(__file__).resolve().parent.parent/'goal_pilot_source'
    if str(client) not in sys.path:sys.path.insert(0,str(client))
    from waypoint_contract import validate_route
    route=json.loads(Path(route_path).read_text());policy=json.loads(Path(policy_path).read_text())
    validate_route(route)
    if route['mission_budget_sec']!=policy['mission_budget_sec']:raise ValueError('Route/policy budget differs')
    if (policy.get('max_parent_dispatches')!=10 or policy.get('max_attempts_per_index')!=6
            or policy.get('max_parent_dispatches_per_index',3)!=3 or policy.get('bt_retries')!=1):
        raise ValueError('Experiment parent/child policy differs')
    # Map manifest is an identity artifact; actual load/hash checks are a pinned
    # map stage and the normal Native launcher, not a workflow success boolean.
    if not isinstance(json.loads(Path(map_path).read_text()),dict):raise ValueError('Map identity object required')
    return route,policy

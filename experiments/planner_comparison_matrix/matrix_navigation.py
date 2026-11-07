#!/usr/bin/env python3
"""Prepare/start only navigation and evidence capture against an already healthy Matrix+Lightning stack.

No fake plant, localization status publisher, TF publisher, sensor bridge, SDK adapter or goal client.
"""
import argparse,copy,hashlib,json,math,os,pathlib,shutil,signal,subprocess,sys,time,threading
import yaml
from static_layer_snapshot import EXCLUDED_LAYERS,capture as capture_static_layers,audit_recorded as audit_static_layers
from recording_deadline import arm as arm_deadline,runtime_seconds,process_identity
BASE=pathlib.Path(__file__).resolve().parent
NAV=BASE.parents[1]
def sha(path):
    with pathlib.Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()
def write(path,value,compact=False):
    path=pathlib.Path(path);temporary=path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value,separators=(',',':'))+'\n' if compact else json.dumps(value,indent=2)+'\n');temporary.replace(path)



class LiveReadyStorage:
    """Only the live READY transport uses a run-owned same-namespace tmpfs slot."""
    def __init__(self,output,deadline_manifest,root=pathlib.Path('/dev/shm')):
        self.output=pathlib.Path(output).resolve();self.root=pathlib.Path(root).resolve()
        self.launcher=deadline_manifest['launcher'];self.run_id=deadline_manifest['run_id']
        if len(self.run_id)!=32 or any(c not in '0123456789abcdef' for c in self.run_id):raise ValueError('Invalid READY run UUID')
        if self.launcher!=process_identity(os.getpid()):raise ValueError('READY launcher PID/birth/boot identity mismatch')
        self.owner=dict(schema='matrix.live_ready.owner.v1',run_id=self.run_id,launcher=self.launcher,
          output_directory=str(self.output),uid=os.getuid(),mount_namespace=os.readlink('/proc/self/ns/mnt'),
          pid_namespace=os.readlink('/proc/self/ns/pid'))
        self.owner_raw=(json.dumps(self.owner,sort_keys=True,separators=(',',':'))+'\n').encode()
        self.slot=self.root/('matrix-ready-'+self.run_id+'-'+self.launcher['boot_id']+'-'+str(self.launcher['pid'])+'-'+str(self.launcher['starttime_ticks']))
        self.live=self.slot/'ready.json';self.owner_path=self.slot/'owner.json';self.path=self.output/'ready.json'
        self.created=False;self.linked=False;self.archived=False;self.cleaned=False;self.slot_identity=None;self.files={}
        self.storage=dict(schema='matrix.live_ready.storage.v1',owner=self.owner,owner_sha256=hashlib.sha256(self.owner_raw).hexdigest(),
          live_slot=str(self.slot),live_file=str(self.live),case_file=str(self.path),live_storage='tmpfs',owned_slot_removed=False,
          namespace_boundary='Live absolute /dev/shm target resolves only in this container mount namespace; host prelaunch readers must read the case path inside the container until materialized.',
          policy='Only READY and owner in RAM. Original checked timestamp,500ms/50ms lease and physical predicates unchanged; disk events/timing/recording unchanged.')
    def start(self):
        if self.path.exists() or self.path.is_symlink():raise FileExistsError('Case READY already exists')
        mount=None
        for line in pathlib.Path('/proc/self/mountinfo').read_text().splitlines():
            before,after=line.split(' - ',1);point=pathlib.Path(before.split()[4])
            if self.root.is_relative_to(point) and (mount is None or len(str(point))>len(mount[0])):mount=(str(point),after.split()[0])
        if mount is None or mount[1]!='tmpfs':raise RuntimeError('Live READY root must be tmpfs')
        usage=shutil.disk_usage(self.root)
        if usage.free<4*1024*1024:raise RuntimeError('Live READY tmpfs needs4MiB free before business startup')
        self.storage['startup_tmpfs']=dict(mount_point=mount[0],filesystem=mount[1],total_bytes=usage.total,free_bytes=usage.free)
        self.slot.mkdir(mode=0o700);self.created=True
        st=self.slot.stat();self.slot_identity=(st.st_dev,st.st_ino,st.st_uid)
        try:
            os.chmod(self.slot,0o700)
            with self.owner_path.open('xb') as stream:
                value=os.fstat(stream.fileno());self.files['owner.json']=(value.st_dev,value.st_ino);stream.write(self.owner_raw)
            os.chmod(self.owner_path,0o600)
            self.publish(dict(ready=False,reason='launcher_starting',time_ns=time.time_ns()))
            # Preserve exact ownership on disk before any business process exists.
            with (self.output/'ready_storage_owner.json').open('xb') as stream:stream.write(self.owner_raw)
            write(self.output/'ready_storage.json',self.storage)
            self.path.symlink_to(self.live);self.linked=True
        except BaseException:
            # No runtime child can exist yet. Publish an ordinary false case file
            # before removing only this startup's inode-bound private files.
            self._rollback_start()
            raise
    def _rollback_start(self):
        if self.path.exists() or self.path.is_symlink():raise RuntimeError('Unexpected case READY during partial startup; no cleanup attempted')
        write(self.path,dict(ready=False,reason='live_ready_startup_failed',stopped_at_ns=time.time_ns()))
        self.archive_raw=self.path.read_bytes();self.archived=True
        if not (self.output/'ready_storage_owner.json').exists():
            with (self.output/'ready_storage_owner.json').open('xb') as stream:stream.write(self.owner_raw)
        self._remove_slot_files()
        self.storage.update(archived=True,startup_failed=True,owned_slot_removed=True,
          archive_sha256=hashlib.sha256(self.archive_raw).hexdigest(),archive_bytes=len(self.archive_raw),case_file_is_regular=True)
        write(self.output/'ready_storage.json',self.storage)
    def _remove_slot_files(self):
        st=self.slot.lstat()
        if self.slot.is_symlink() or (st.st_dev,st.st_ino,st.st_uid)!=self.slot_identity or process_identity(os.getpid())!=self.launcher:
            raise RuntimeError('READY slot/launcher identity changed; no cleanup attempted')
        if {p.name for p in self.slot.iterdir()}!=set(self.files):raise RuntimeError('Unexpected READY slot files; no cleanup attempted')
        for name,identity in self.files.items():
            item=self.slot/name;value=item.lstat()
            if item.is_symlink() or (value.st_dev,value.st_ino)!=identity:raise RuntimeError('READY file identity changed; no cleanup attempted')
        for name in self.files:(self.slot/name).unlink()
        self.slot.rmdir();self.cleaned=True
    def _owned(self):
        if not self.created:raise RuntimeError('READY slot has not been created')
        st=self.slot.lstat()
        if self.slot.is_symlink() or (st.st_dev,st.st_ino,st.st_uid)!=self.slot_identity or st.st_mode&0o777!=0o700:
            raise RuntimeError('READY slot identity/mode changed')
        if self.owner_path.is_symlink() or self.owner_path.read_bytes()!=self.owner_raw or process_identity(os.getpid())!=self.launcher:
            raise RuntimeError('READY owner identity changed')
    def publish(self,value,compact=False):
        if self.archived:raise RuntimeError('Live READY is already archived')
        self._owned()
        if self.linked and (not self.path.is_symlink() or os.readlink(self.path)!=str(self.live)):
            raise RuntimeError('Case READY link ownership changed')
        raw=(json.dumps(value,separators=(',',':'))+'\n' if compact else json.dumps(value,indent=2)+'\n').encode()
        temporary=self.slot/'ready.tmp';created_temporary=False
        try:
            with temporary.open('xb') as stream:
                created_temporary=True;value=os.fstat(stream.fileno());temporary_identity=(value.st_dev,value.st_ino);stream.write(raw)
            os.chmod(temporary,0o600);temporary.replace(self.live);self.files['ready.json']=temporary_identity
        except BaseException:
            # This private temp belongs to this exact write, never another slot.
            if created_temporary and temporary.exists() and not temporary.is_symlink():
                value=temporary.lstat()
                if (value.st_dev,value.st_ino)==temporary_identity:temporary.unlink()
            raise
    def stop_false(self,stopped_at_ns=None):
        if self.archived:
            if self.path.is_symlink() or self.path.read_bytes()!=self.archive_raw:raise RuntimeError('Archived READY changed')
            return
        if not self.created:
            if self.path.exists() or self.path.is_symlink():raise RuntimeError('Unowned case READY; refusing overwrite')
            write(self.path,dict(ready=False,reason='live_ready_not_started',stopped_at_ns=time.time_ns()))
            self.archive_raw=self.path.read_bytes();self.archived=True
            return
        self._owned()
        try:value=json.loads(self.live.read_bytes())
        except (OSError,ValueError):value={}
        if value.get('ready') is False and 'stopped_at_ns' in value:return
        value.update(ready=False,stopped_at_ns=time.time_ns() if stopped_at_ns is None else stopped_at_ns)
        self.publish(value)
    def materialize(self):
        """Caller proves controls/recorder closed; preserve identical final false bytes."""
        if self.archived:return
        self._owned();raw=self.live.read_bytes()
        if json.loads(raw).get('ready') is not False:raise RuntimeError('Cannot archive a true READY')
        if self.linked and (not self.path.is_symlink() or os.readlink(self.path)!=str(self.live)):
            raise RuntimeError('Case READY link ownership changed')
        temporary=self.output/'ready.materialize.tmp';created_temporary=False
        try:
            with temporary.open('xb') as stream:
                created_temporary=True;value=os.fstat(stream.fileno());temporary_identity=(value.st_dev,value.st_ino);stream.write(raw)
            temporary.replace(self.path)
        except BaseException:
            if created_temporary and temporary.exists() and not temporary.is_symlink():
                value=temporary.lstat()
                if (value.st_dev,value.st_ino)==temporary_identity:temporary.unlink()
            raise
        self.archive_raw=raw;self.archived=True
        self.storage.update(archived=True,archive_sha256=hashlib.sha256(raw).hexdigest(),archive_bytes=len(raw),
          archive_identical_to_final_live_bytes=True,case_file_is_regular=not self.path.is_symlink())
        write(self.output/'ready_storage.json',self.storage)
        self._owned();self._remove_slot_files()
        self.storage['owned_slot_removed']=True;write(self.output/'ready_storage.json',self.storage)


def phase_mark(timing,name,started):
    finished=(time.monotonic_ns(),time.thread_time_ns())
    timing[name]=dict(wall_ns=finished[0]-started[0],thread_cpu_ns=finished[1]-started[1])
    return finished

def next_heartbeat_deadline(previous,now,period=.05):
    """Absolute cadence; skip missed slots, never stack another fixed sleep."""
    deadline=previous+period
    if deadline<=now:deadline+=(int((now-deadline)//period)+1)*period
    return deadline

class OwnershipAuditWorker:
    """One full audit at a time; one immutable latest result, no queued backlog.

    The worker never writes READY, pumps ROS callbacks, or changes checked_unix_ns.
    Every consumer still enforces the original 500ms/50ms metadata age bounds.
    """
    def __init__(self,audit,seed,period=.2):
        self.audit=audit;self.period=period;self.lock=threading.Lock();self.stop_event=threading.Event()
        self.result=seed;self.error=None;self.sequence=0;self.running=False;self.timing={}
        self.thread=threading.Thread(target=self._work,name='matrix-ownership-audit',daemon=True)
        self.thread.start()
    def _work(self):
        deadline=time.monotonic()
        while not self.stop_event.is_set():
            started=(time.monotonic_ns(),time.thread_time_ns());started_unix=time.time_ns()
            with self.lock:self.running=True
            try:result=self.audit();error=None
            except Exception as exc:result=None;error=type(exc).__name__+': '+str(exc)
            completed=time.time_ns();timing={};phase_mark(timing,'full_audit',started)
            with self.lock:
                self.sequence+=1;self.running=False;self.error=error
                self.timing=dict(started_unix_ns=started_unix,completed_unix_ns=completed,**timing)
                if result is not None:self.result=result
            deadline=next_heartbeat_deadline(deadline,time.monotonic(),self.period)
            self.stop_event.wait(max(0.,deadline-time.monotonic()))
    def state(self):
        with self.lock:return dict(sequence=self.sequence,running=self.running,error=self.error,timing=dict(self.timing))
    def snapshot(self,now_ns):
        with self.lock:result=self.result;error=self.error;sequence=self.sequence
        if error is not None:raise RuntimeError('Ownership audit failed: '+error)
        checked=result.get('checked_unix_ns',0)
        if type(checked) is not int or not -50_000_000<=now_ns-checked<=500_000_000:
            raise RuntimeError('Ownership metadata observation stale or future; original audit lease not renewed')
        return result,sequence
    def request_stop(self):self.stop_event.set()
    def join(self,timeout=1.):
        self.thread.join(timeout);return not self.thread.is_alive()


def scan_observation(latest,topic,begin,end,reason):
    """Read-only source cache and clock bounds, not a new health predicate."""
    item=latest.get(topic)
    if item is None:return None
    received,message=item
    stamp=message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
    return dict(topic=topic,header_ns=stamp,frame_id=message.header.frame_id,ranges=len(message.ranges),
      receive_steady_seconds=received,validation_clock_begin_ns=begin[0],validation_clock_end_ns=end[0],
      validation_steady_begin_ns=begin[1],validation_steady_end_ns=end[1],
      signed_header_age_ns_bounds=[begin[0]-stamp,end[0]-stamp],
      signed_receive_age_sec_bounds=[begin[1]/1e9-received,end[1]/1e9-received],failure_reason=reason,
      boundary='Call bounds only, not exact internal validation now. source_health stops at first error; later producer guards may not have been evaluated.')


def source_snapshot(path,template,with_waypoints=False):
    """Validate a deployment source manifest without inspecting a Git checkout."""
    def unique_object(pairs):
        result={}
        for key,value in pairs:
            if key in result:raise ValueError('Duplicate source snapshot key: '+key)
            result[key]=value
        return result
    def hex_digest(value,length):
        return isinstance(value,str) and len(value)==length and all(c in '0123456789abcdef' for c in value)
    raw=pathlib.Path(path).read_bytes()
    data=json.loads(raw,object_pairs_hook=unique_object)
    if not isinstance(data,dict) or set(data)!={'schema_version','origin','files_sha256'}:
        raise ValueError('Source snapshot requires schema_version, origin and files_sha256')
    if type(data['schema_version']) is not int or data['schema_version']!=1:
        raise ValueError('Unsupported source snapshot schema_version')
    origin=data['origin']
    if not isinstance(origin,dict) or set(origin)!={'kind','canonical_repository','commit','branch','status'}:
        raise ValueError('Source snapshot origin requires canonical Git observation')
    if origin['kind']!='canonical_checkout_snapshot' or not hex_digest(origin['commit'],40):
        raise ValueError('Invalid source snapshot origin kind or commit')
    if not isinstance(origin['canonical_repository'],str) or not pathlib.PurePosixPath(origin['canonical_repository']).is_absolute():
        raise ValueError('Source snapshot canonical_repository must be an absolute origin path')
    if not isinstance(origin['branch'],str) or not origin['branch'] or not isinstance(origin['status'],str):
        raise ValueError('Source snapshot branch and status must preserve the canonical observation')
    files=data['files_sha256']
    if not isinstance(files,dict) or not files:raise ValueError('Source snapshot requires source file hashes')
    root=NAV.resolve()
    # The wrapper executes before this process; the other files are consumed here.
    required=[pathlib.Path(__file__),BASE/'static_layer_snapshot.py',BASE/'recording_deadline.py',BASE/'run_matrix_navigation.sh',
      template,NAV/'src/navigation/src/robot_navigo/scripts/nav_safety_gate.py']
    if with_waypoints:
        required += [BASE/'goal_pilot_source'/name for name in
          ['waypoint_batch.py','waypoint_contract.py','pilot_scan_timing.py','message_info_contract.py']]
    required_names=set()
    for source in required:
        resolved=source.resolve(strict=True)
        if not resolved.is_relative_to(root):raise ValueError('Source input escapes deployment NAV: '+str(source))
        required_names.add(source.absolute().relative_to(root).as_posix())
    if not required_names<=files.keys():
        raise ValueError('Source snapshot missing required files: '+repr(sorted(required_names-files.keys())))
    for name,digest in files.items():
        relative=pathlib.PurePosixPath(name)
        if not name or relative.is_absolute() or '..' in relative.parts or '\\' in name or relative.as_posix()!=name:
            raise ValueError('Source snapshot file must be a normalized relative NAV path: '+name)
        resolved=(root/name).resolve(strict=True)
        if not resolved.is_relative_to(root) or not resolved.is_file():
            raise ValueError('Source snapshot file escapes deployment NAV or is not a file: '+name)
        if not hex_digest(digest,64) or sha(resolved)!=digest:raise ValueError('Source snapshot hash mismatch: '+name)
    return data,raw

def native_bt_self_client_valid(proof):
    """Verify the owned native self-client and its sealed, observed topic ingress."""
    try:
        run_id=proof['run_id'];topic='/experiment/native_waypoint_private/run_'+run_id+'/goal_pose'
        if len(run_id)!=32 or any(char not in '0123456789abcdef' for char in run_id):return False
        if proof['private_goal_topic']!=topic or proof['private_goal_publishers']!=[]:return False
        if proof['first_private_goal_publisher_violation'] is not None:return False
        if type(proof['received_private_goal_messages']) is not int or proof['received_private_goal_messages']!=0:return False
        if not 0<proof['subscription_created_steady_ns']<proof['bt_launch_steady_ns']:return False
        pose_subscriptions=[(name,types) for name,types in proof['bt_subscriptions'] if 'geometry_msgs/msg/PoseStamped' in types or name in (topic,'/goal_pose')]
        if pose_subscriptions!=[(topic,['geometry_msgs/msg/PoseStamped'])]:return False
        owners=[item for item in proof['private_goal_subscribers'] if item['name']=='bt_navigator' and item['namespace']=='/']
        if len(owners)!=1 or owners[0]['type']!='geometry_msgs/msg/PoseStamped':return False
        initial=proof['initial_process_identity'];current=proof['current_process_identity'];plan=proof['owned_process_plan']
        if current!=initial or plan['role']!='bt_navigator' or plan['run_id']!=run_id or plan['pid']!=current['pid']:return False
        if not current['pid']>0 or not current['starttime_ticks']>0 or not current['boot_id'] or len(current['cmdline_sha256'])!=64:return False
        if current['argv']!=plan['executed_argv'] or current['argv'][0]!='/opt/ros/jazzy/lib/nav2_bt_navigator/bt_navigator':return False
        if [arg for arg in current['argv'] if arg.startswith('goal_pose:=')]!=['goal_pose:='+topic]:return False
        required={'/opt/ros/jazzy/lib/nav2_bt_navigator/bt_navigator','/opt/ros/jazzy/lib/libnav2_navigate_to_pose_navigator.so'}
        if {item['path'] for item in proof['artifacts']}!=required or len(proof['artifacts'])!=2:return False
        for item in proof['artifacts']:
            if item['path'] not in proof['current_mapped_files']:return False
            if len(item['expected_sha256'])!=64 or item['observed_sha256']!=item['expected_sha256']:return False
            if item['initial_stat']!=item['current_stat'] or set(item['initial_stat'])!={'dev','ino','size','mtime_ns','ctime_ns'}:return False
        return True
    except (KeyError,TypeError,ValueError,AttributeError,IndexError):return False

def waypoint_service_ownership(graph,action_classifier,native_self_client=None):
    """Separate action-feedback observers from actual NavigateToPose callers.

    Jazzy's action-client graph infers clients from feedback subscriptions. Any
    of the three real service-client endpoints is conservatively a caller here;
    the follower and sealed native BT self-client must each expose all three
    with exact, unambiguous types. Only verified sealing removes the latter
    from the legacy business-sender list, never from the actual graph evidence.
    """
    expected={
      '/navigate_to_pose/_action/send_goal':'nav2_msgs/action/NavigateToPose_SendGoal',
      '/navigate_to_pose/_action/get_result':'nav2_msgs/action/NavigateToPose_GetResult',
      '/navigate_to_pose/_action/cancel_goal':'action_msgs/srv/CancelGoal'}
    normalized=[];raw_actions=[];service_graph=[];callers=[];identities={};complete=True
    sealed_bt=native_bt_self_client_valid(native_self_client)
    for entry in graph:
        identity=dict(name=entry['name'],namespace=entry['namespace'])
        key=(entry['name'],entry['namespace']);identities[key]=identities.get(key,0)+1
        clients=entry.get('service_clients')
        if not isinstance(clients,(list,tuple)):
            complete=False;clients=[]
        endpoints={}
        for name,types in clients:
            if name in expected:endpoints.setdefault(name,[]).append(list(types))
        exact=set(endpoints)==set(expected) and all(endpoints[name]==[[kind]] for name,kind in expected.items())
        if endpoints:callers.append(dict(**identity,endpoints=endpoints,complete_exact_three=exact))
        exclude=sealed_bt and exact and identity==dict(name='bt_navigator',namespace='/')
        normalized.append(dict(**identity,clients=[('/navigate_to_pose',['nav2_msgs/action/NavigateToPose'])] if endpoints and not exclude else [],
          servers=entry['servers']))
        raw_actions.append(dict(**identity,clients=entry['clients'],servers=entry['servers']))
        service_graph.append(dict(**identity,service_clients=clients))
    duplicates=[dict(name=name,namespace=namespace,count=count) for (name,namespace),count in identities.items() if count>1]
    follower=[item for item in callers if item['name']=='waypoint_follower' and item['namespace']=='/']
    bt=[item for item in callers if item['name']=='bt_navigator' and item['namespace']=='/']
    sealed_bt=sealed_bt and len(bt)==1 and bt[0]['complete_exact_three']
    follower_complete=len(follower)==1 and follower[0]['complete_exact_three']
    result=action_classifier(normalized)
    result['exclusive_nav_client']=result['exclusive_nav_client'] and complete and follower_complete and sealed_bt and not duplicates
    result.update(raw_action_graph=raw_actions,actual_service_client_graph=service_graph,
      navigate_to_pose_service_client_evidence=callers,expected_navigate_to_pose_client_services=expected,
      waypoint_follower_complete_service_client=follower_complete,service_client_graph_complete=complete,
      duplicate_node_namespaces=duplicates,
      actual_navigate_to_pose_clients=[dict(name=item['name'],namespace=item['namespace']) for item in callers],
      controlled_native_self_client=dict(verified=sealed_bt,evidence=native_self_client),
      client_classification_boundary='Actual service-client endpoints identify every caller/canceller, including partial/wrong types. Native follower and bt_navigator must each have all three exact service types. Only the owned, artifact-bound bt self-client with run-UUID private topic ingress, observed zero publishers/messages and exact subscription ownership is excluded from the legacy navigate_to_pose_clients business-sender list. Raw action graph and both actual callers remain recorded. This is monitored cooperative isolation, not an ACL. Existing action-server role checks are unchanged.')
    return result

def lifecycle_artifacts(install,receipt_path):
    """Bind an optional existing lifecycle implementation to its exact build."""
    if bool(install)!=bool(receipt_path):raise ValueError('Lifecycle install and build receipt must be supplied together')
    if not install:return None
    prefix=pathlib.Path(install).resolve();receipt=pathlib.Path(receipt_path).resolve()
    data=json.loads(receipt.read_text())
    if data.get('schema')!='matrix.lifecycle.build_receipt.v1' or data.get('install_prefix')!=str(prefix):
        raise ValueError('Lifecycle build receipt schema or prefix differs')
    files=data.get('artifact_sha256',{})
    required={'lib/nav2_lifecycle_manager/lifecycle_manager','lib/libnav2_lifecycle_manager_core.so'}
    if not required<=files.keys():raise ValueError('Lifecycle receipt lacks executable/core library')
    for name,digest in files.items():
        relative=pathlib.PurePosixPath(name);path=(prefix/name).resolve()
        if relative.is_absolute() or '..' in relative.parts or relative.as_posix()!=name or not path.is_relative_to(prefix):
            raise ValueError('Lifecycle artifact escapes prefix')
        if not path.is_file() or sha(path)!=digest:raise ValueError('Lifecycle artifact differs: '+name)
    return dict(install_prefix=str(prefix),receipt_path=str(receipt),receipt_sha256=sha(receipt),
      source_manifest=data['source_manifest'],artifact_sha256=files)

def navigation_tree(variant):
    """A single bounded controller recovery retains the same external goal."""
    plain='<Sequence><ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/><FollowPath path="{path}" controller_id="FollowPath" goal_checker_id="Checker" progress_checker_id="Progress"/></Sequence>'
    if variant=='segmented':
        # Reset the error before ComputePath so planner failures cannot reuse a
        # previous goal's controller error and spuriously enter recovery.
        execute='<Sequence><SetBlackboard output_key="follow_path_error_code" value="0"/><ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased" error_code_id="{compute_path_error_code}"/><FollowPath path="{path}" controller_id="FollowPath" goal_checker_id="Checker" progress_checker_id="Progress" error_code_id="{follow_path_error_code}"/></Sequence>'
        recover='<Sequence><WouldAControllerRecoveryHelp error_code="{follow_path_error_code}"/><Delay delay_msec="1000"><AlwaysSuccess/></Delay></Sequence>'
        plain='<RecoveryNode number_of_retries="1" name="StoppedOriginalGoalReplan">'+execute+recover+'</RecoveryNode>'
    return '<root BTCPP_format="4" main_tree_to_execute="MainTree"><BehaviorTree ID="MainTree">'+plain+'</BehaviorTree></root>\n'

def prepare(args):
    template=pathlib.Path(args.template).absolute()
    snapshot,snapshot_raw=source_snapshot(args.source_snapshot,template,args.with_waypoints) if args.source_snapshot else (None,None)
    out=pathlib.Path(args.output).resolve();out.mkdir(parents=True,exist_ok=False)
    footprint=json.loads(args.footprint)
    if len(footprint)<3 or any(len(p)!=2 or any(not math.isfinite(v) for v in p) for p in footprint):raise ValueError('finite polygon required')
    crosses=[]
    for a,b,c in zip(footprint,footprint[1:]+footprint[:1],footprint[2:]+footprint[:2]):
        crosses.append((b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0]))
    if not (all(x>0 for x in crosses) or all(x<0 for x in crosses)):raise ValueError('strictly convex footprint required')
    if not 0<=args.padding<=.2:raise ValueError('invalid padding')
    if not 0<args.xy_tolerance<=.25 or not 0<args.yaw_tolerance<=.25:raise ValueError('invalid goal tolerance')
    map_yaml=pathlib.Path(args.map).resolve();map_data=yaml.safe_load(map_yaml.read_text())
    if abs(float(map_data['resolution'])-.01)>1e-8:raise ValueError('this comparison requires 1cm geometry map')
    map_image=(map_yaml.parent/map_data['image']).resolve()
    template=template.resolve()
    d=yaml.safe_load(template.read_text());rpp=copy.deepcopy(d['controller_server']['ros__parameters']['FollowPath'].get('rpp',d['controller_server']['ros__parameters']['FollowPath']))
    rpp.pop('plugin',None)
    if args.rotate_threshold is not None:rpp['rotate_to_heading_min_angle']=args.rotate_threshold
    selected_path=args.segmented_install if args.variant=='segmented' else args.official_install
    if not selected_path:raise ValueError('Provide an explicit --segmented-install or --official-install for the selected variant')
    selected_install=pathlib.Path(selected_path).resolve()
    required_plugin={'official':'official_lattice_rpp_adapter::CheckedLattice','hybrid':'official_lattice_rpp_adapter::CheckedHybrid','segmented':'robot_feasible_navigation::SafeLatticePlanner'}[args.variant]
    manifest=dict(scope='Matrix navigation experiment only; not physical robot acceptance',stage='prepared',execution_requested=args.run,variant=args.variant,
        selected_install=str(selected_install),map_yaml=str(map_yaml),map_yaml_sha256=sha(map_yaml),map_image=str(map_image),map_image_sha256=sha(map_image),
        profile_template=str(template),profile_template_sha256=sha(template),footprint=footprint,padding=args.padding,
        rotate_to_heading_min_angle=rpp['rotate_to_heading_min_angle'],
        odom_topic=args.odom_topic,scan_topic=args.scan_topic,camera_topic=args.camera_topic,camera_info_topic=args.camera_info_topic,
        global_costmap=dict(rolling_window=True,width_m=20,height_m=20,resolution=.01,publish_frequency=.5),
        local_costmap=dict(rolling_window=True,width_m=6,height_m=6,resolution=.01,publish_frequency=1.),
        goal_tolerances=dict(xy=args.xy_tolerance,yaw=args.yaw_tolerance,stateful=False),goal_distance_from_start_limit_m=8.,
        command_chain=['/cmd_vel_nav','/cmd_vel','/cmd_vel_safe'],goal_client='Separate authorized agent must record UUID, original goal, result/cancel and settled errors',
        build_receipt=str(pathlib.Path(args.build_receipt).resolve()) if args.build_receipt else None,
        source_repository_observation=None if snapshot else dict(head=subprocess.check_output(['git','-C',str(NAV),'rev-parse','HEAD'],text=True).strip(),
          status=subprocess.check_output(['git','-C',str(NAV),'status','--short'],text=True)),
        caution='Repository observation is not a substitute for the selected artifact build receipt')
    if snapshot:
        frozen=out/'deployment_source_snapshot.json';frozen.write_bytes(snapshot_raw)
        manifest['deployment_source_snapshot']=dict(path=str(frozen),sha256=sha(frozen),origin=snapshot['origin'],
          validated_files_sha256=snapshot['files_sha256'],deployment_nav=str(NAV),
          note='Supplied canonical source observation and verified deployment bytes; no deployment Git checkout was inspected. This does not replace the selected artifact build receipt.')
    if args.build_receipt:
        receipt=pathlib.Path(args.build_receipt).resolve();shutil.copy2(receipt,out/'source_build_receipt.json');manifest['build_receipt_sha256']=sha(receipt)
    manifest['lifecycle_manager_runtime']=lifecycle_artifacts(args.lifecycle_install,args.lifecycle_build_receipt)
    if manifest['lifecycle_manager_runtime']:
        shutil.copy2(args.lifecycle_build_receipt,out/'lifecycle_build_receipt.json')
    for node in ['controller_server','velocity_smoother','bt_navigator']:d[node]['ros__parameters']['odom_topic']=args.odom_topic
    d['map_server']['ros__parameters']['yaml_filename']=str(map_yaml)
    for name,size,frequency in [('global_costmap',20,.5),('local_costmap',6,1.)]:
        p=d[name][name]['ros__parameters'];p.update(footprint=json.dumps(footprint),footprint_padding=args.padding,
          rolling_window=True,width=size,height=size,resolution=.01,publish_frequency=frequency,
          update_frequency=2. if name=='global_costmap' else 10.,always_send_full_costmap=True)
        # Preserve the original map's unknown-space hard boundary: with only
        # inflate_unknown=false, stock inflation may replace unknown by253.
        p['inflation_layer'].update(inflate_unknown=False,inflate_around_unknown=True)
        p['obstacle_layer']=dict(plugin='nav2_costmap_2d::ObstacleLayer',enabled=True,footprint_clearing_enabled=False,observation_sources='scan',
          scan=dict(topic=args.scan_topic,data_type='LaserScan',marking=True,clearing=True,inf_is_valid=False,
            observation_persistence=0.,expected_update_rate=.3,min_obstacle_height=0.,max_obstacle_height=2.,
            raytrace_max_range=8.5,raytrace_min_range=.0,obstacle_max_range=8.,obstacle_min_range=.1))
    planner=d['planner_server']['ros__parameters']['GridBased'];planner.update(plugin=required_plugin,tolerance=0.,max_planning_time=2.,rotation_penalty=1.)
    controller=d['controller_server']['ros__parameters'];controller['Checker'].update(stateful=False,xy_goal_tolerance=args.xy_tolerance,yaw_goal_tolerance=args.yaw_tolerance,controller_name='FollowPath')
    controller['Checker'].update(trans_stopped_velocity=.01,rot_stopped_velocity=.01)
    if args.variant=='segmented':
        planner.update(cost_penalty=0.,terminal_goal_binding=True,terminal_global_resolution=.01)
        planner['lattice_filepath']=str(selected_install/'share/robot_feasible_navigation/params/stop_turn_1cm_h16.json')
        controller['Checker']['plugin']='nav2_controller::StoppedGoalChecker'
        controller['FollowPath']=dict(rpp,plugin='nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController')
        controller['segments']=dict(xy_tolerance=args.xy_tolerance,yaw_tolerance=args.yaw_tolerance,stopped_linear_velocity=.01,stopped_angular_velocity=.01,
          same_xy_epsilon=.0001,progress_search_distance=.25,odom_timeout=.3,rotation_anchor_wait_timeout=15.,command_sweep_horizon=1.,
          terminal_goal_binding=True,terminal_global_resolution=.01,terminal_global_extent=20.)
    else:
        controller['Checker']['plugin']='official_lattice_rpp_adapter::FeasibleGoalChecker'
        controller['FollowPath']=dict(plugin='official_lattice_rpp_adapter::FeasibleRPP',rpp=rpp,odom_timeout=.3,sweep_horizon=1.,
          completion_path_length=max(.08,args.xy_tolerance),allow_static_path_transform=False,handler=dict(enforce_path_rotation=True,enforce_path_inversion=True,
          inversion_xy_tolerance=args.xy_tolerance,inversion_yaw_tolerance=args.yaw_tolerance,minimum_rotation_angle=.05,prune_distance=2.,max_robot_pose_search_dist=3.))
        if args.variant=='hybrid':
            planner.pop('lattice_filepath',None);planner.update(motion_model_for_search='DUBIN',angle_quantization_bins=72,
              minimum_turning_radius=.5,analytic_expansion_max_length=3.,analytic_expansion_ratio=3.5)
            manifest['control_only']='Hybrid is auxiliary control; primary comparison is segmented versus official Lattice.'
        else:planner['lattice_filepath']=str(selected_install/'share/official_lattice_rpp_adapter/params/stop_turn_1cm_h16_small_radius.json')
    # Freeze the actual primitive input instead of depending on subsequent canonical branch switches.
    if 'lattice_filepath' in planner:
        source=pathlib.Path(planner['lattice_filepath']);shutil.copy2(source,out/'lattice.json');planner['lattice_filepath']=str(out/'lattice.json');manifest['lattice_sha256']=sha(out/'lattice.json')
    bt=out/'navigate.xml';bt.write_text(navigation_tree(args.variant))
    manifest['controller_recovery']=dict(enabled=args.variant=='segmented',maximum_retries=1 if args.variant=='segmented' else 0,
      policy='Controller recoverable failure only; stopped zero, builtin Delay1s, fresh full ComputePathToPose then new FollowPath to unchanged original goal; no waypoint success or goal UUID resend',
      wait_implementation='BehaviorTree.CPP Delay/AlwaysSuccess; no behavior_server or motion recovery action',
      plugins=['nav2_recovery_node_bt_node','nav2_would_a_controller_recovery_help_condition_bt_node'] if args.variant=='segmented' else [])
    d['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml']=str(bt)
    manifest['with_waypoints']=args.with_waypoints
    if args.with_waypoints:
        # Official Jazzy 1.3.13: one native batch, internally one NavigateToPose
        # per leg. The existing single-goal BT and terminal binding stay intact.
        sys.path.insert(0,str(BASE/'goal_pilot_source'))
        from waypoint_contract import WAYPOINT_PARAMETERS
        import xml.etree.ElementTree as ET
        waypoint_version=ET.parse('/opt/ros/jazzy/share/nav2_waypoint_follower/package.xml').getroot().findtext('version')
        if waypoint_version!='1.3.13':raise RuntimeError('Native waypoint integration is reviewed only for Jazzy 1.3.13')
        if ET.parse('/opt/ros/jazzy/share/nav2_bt_navigator/package.xml').getroot().findtext('version')!='1.3.13':
            raise RuntimeError('Native NavigateToPose self-client isolation is reviewed only for Jazzy 1.3.13')
        d['waypoint_follower']={'ros__parameters':copy.deepcopy(WAYPOINT_PARAMETERS)}
        d['lifecycle_manager']['ros__parameters']['node_names'].append('waypoint_follower')
        source_files=[BASE/'goal_pilot_source'/name for name in
          ['waypoint_batch.py','waypoint_contract.py','pilot_scan_timing.py','message_info_contract.py']]
        underlay_files=[pathlib.Path('/opt/ros/jazzy')/name for name in
          ['lib/nav2_waypoint_follower/waypoint_follower','lib/libwaypoint_follower_core.so','lib/libwait_at_waypoint.so',
           'share/nav2_waypoint_follower/package.xml','share/nav2_msgs/action/FollowWaypoints.action','share/nav2_msgs/msg/MissedWaypoint.msg',
           'lib/nav2_bt_navigator/bt_navigator','lib/libnav2_navigate_to_pose_navigator.so','share/nav2_bt_navigator/package.xml']]
        manifest['waypoint_contract']=dict(package_version=waypoint_version,parameters=WAYPOINT_PARAMETERS,number_of_loops=0,goal_index=0,
          underlay_files_sha256={str(path):sha(path) for path in underlay_files},
          client_sources_sha256={str(path.relative_to(NAV)):sha(path) for path in source_files},
          semantics='One native FollowWaypoints request; internal sequential NavigateToPose requests, no retry or skip after failure. Feedback is not visit proof.',
          child_cancellation_boundary='Upstream Jazzy 1.3.13 waypoint_follower uses async_cancel_all_goals on its NavigateToPose client when its own batch is canceled. The independent client cancels only its own FollowWaypoints UUID; this isolated experiment requires exclusive NavigateToPose ownership.',
          child_goal_timestamp_boundary='Upstream follower refreshes each requested child goal header to dispatch time. Original route values and native source pose sample stamps remain separately recorded.',
          native_self_client_policy='Jazzy NavigateToPose has a real self-client. Only with-waypoints remaps its sole goal_pose ingress to the fixed recording run UUID private topic, observes it before BT starts, and permits the exact artifact/owned-process-bound self-client only while actual subscription ownership, zero publishers and zero lifetime received messages remain verified. Actual service callers are recorded separately from legacy business-sender compatibility fields.')
    params=out/'params.yaml';params.write_text(yaml.safe_dump(d,sort_keys=False));manifest['params_sha256']=sha(params)
    shutil.copy2(__file__,out/'launcher_snapshot.py');manifest['launcher_sha256']=sha(__file__)
    helper=BASE/'static_layer_snapshot.py';shutil.copy2(helper,out/helper.name)
    manifest['static_layer_snapshot_helper_sha256']=sha(helper)
    helper=BASE/'recording_deadline.py';shutil.copy2(helper,out/helper.name)
    manifest['recording_deadline_helper_sha256']=sha(helper)
    manifest['recording_deadline_policy']=dict(max_runtime_sec=args.max_runtime_sec,clock='CLOCK_BOOTTIME from launcher process start',
      detached=True,signal='one launcher SIGINT; after 15s grace exact registered child SIGINT fallback',armed_before='ROS setup and all runtime children')
    manifest['recording_policy']=dict(excluded_continuous_topics_regex=EXCLUDED_LAYERS,
      replacement='One original-CDR snapshot of each frozen static layer under /experiment/static_layer_snapshot; map and master costmaps remain continuous')
    write(out/'runtime_manifest.json',manifest);write(out/'invocation.json',dict(argv=sys.argv,cwd=os.getcwd()))
    return out,selected_install,manifest

def run(args,out,selected_install,manifest,deadline_guard,ready_storage):
    if os.environ.get('ROS_DOMAIN_ID')!='185':raise RuntimeError('Matrix navigation requires the isolated domain185 environment from simulation_setup')
    if not args.build_receipt:raise RuntimeError('--run requires a concrete selected-artifact --build-receipt')
    if not args.message_install:raise RuntimeError('--run requires an isolated custom-message --message-install for complete recording')
    if lifecycle_artifacts(args.lifecycle_install,args.lifecycle_build_receipt)!=manifest['lifecycle_manager_runtime']:
        raise RuntimeError('Lifecycle artifacts changed after preparation')
    if args.source_snapshot:
        # Recheck the frozen manifest immediately before runtime setup, including
        # files copied later (safety gate); a preparation result is not a lease.
        frozen=out/'deployment_source_snapshot.json'
        if sha(frozen)!=manifest['deployment_source_snapshot']['sha256']:
            raise RuntimeError('Deployment source snapshot changed after preparation')
        source_snapshot(frozen,pathlib.Path(args.template).absolute(),args.with_waypoints)
    if args.with_waypoints:
        for path,digest in manifest['waypoint_contract']['underlay_files_sha256'].items():
            if sha(path)!=digest:raise RuntimeError('Native waypoint underlay changed after preparation: '+path)
        for name,digest in manifest['waypoint_contract']['client_sources_sha256'].items():
            if sha(NAV/name)!=digest:raise RuntimeError('Native waypoint client source changed after preparation: '+name)
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from rclpy.time import Time
    from rclpy.serialization import serialize_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CompressedImage,CameraInfo,LaserScan
    from std_msgs.msg import UInt8,Bool
    from nav2_msgs.msg import Costmap
    from nav2_msgs.srv import GetCostmap
    from lifecycle_msgs.srv import GetState
    from rcl_interfaces.srv import ListParameters,GetParameters
    from tf2_ros import Buffer,TransformListener
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy,DurabilityPolicy
    import cv2,numpy as np
    stopped=False;shutdown_started=False;static_snapshots=[];stop_signals=[]
    ownership_worker=None;producer_timing_stream=None
    def request_stop(signum,*unused):
        nonlocal stopped;stopped=True;stop_signals.append(signum)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT,request_stop);signal.signal(signal.SIGTERM,request_stop)
    node=Node('matrix_navigation_evidence_observer');latest={};counts={};procs=[];logs=[]
    event_stream=(out/'events.jsonl').open('w')
    def event(kind,**kw):
        event_stream.write(json.dumps(dict(time_ns=node.get_clock().now().nanoseconds,steady=time.monotonic(),kind=kind,**kw))+'\n');event_stream.flush()
    def receive(topic,msg):latest[topic]=(time.monotonic(),msg);counts[topic]=counts.get(topic,0)+1
    subscriptions=[];startup_costmap_observers={}
    costmap_evidence_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for topic,kind in [(args.odom_topic,Odometry),('/lightning/loc_status',UInt8),('/lightning/pose_valid',Bool),(args.scan_topic,LaserScan),(args.camera_topic,CompressedImage),(args.camera_info_topic,CameraInfo),
                       ('/global_costmap/costmap_raw',Costmap),('/local_costmap/costmap_raw',Costmap)]:
        qos=costmap_evidence_qos if topic in ('/global_costmap/costmap_raw','/local_costmap/costmap_raw') else qos_profile_sensor_data
        subscription=node.create_subscription(kind,topic,lambda m,t=topic:receive(t,m),qos)
        subscriptions.append(subscription)
        if topic in ('/global_costmap/costmap_raw','/local_costmap/costmap_raw'):
            startup_costmap_observers[topic]=subscription
    tf=Buffer();listener=TransformListener(tf,node)
    action_status={};bt_binding=None;bt_launch_steady_ns=None;private_goal_messages=0;first_private_goal=None;first_private_publisher_violation=None
    ownership_observation_lock=threading.Lock()
    if args.with_waypoints:
        from action_msgs.msg import GoalStatusArray
        from nav2_msgs.action import NavigateToPose
        from geometry_msgs.msg import PoseStamped
        from rclpy.qos import qos_profile_action_status_default
        from rclpy.action.graph import get_action_client_names_and_types_by_node,get_action_server_names_and_types_by_node
        from waypoint_contract import exclusive_graph
        private_goal_topic='/experiment/native_waypoint_private/run_'+deadline_guard.manifest['run_id']+'/goal_pose'
        def private_goal_received(message):
            nonlocal private_goal_messages,first_private_goal
            with ownership_observation_lock:
                private_goal_messages+=1
                if first_private_goal is None:
                    first_private_goal=dict(received_steady_ns=time.monotonic_ns(),message=message_to_ordereddict(message))
        subscriptions.append(node.create_subscription(PoseStamped,private_goal_topic,private_goal_received,QoSProfile(depth=10)))
        private_subscription_steady_ns=time.monotonic_ns()
        manifest['native_bt_self_client_ingress']=dict(run_id=deadline_guard.manifest['run_id'],private_goal_topic=private_goal_topic,
          subscription_created_steady_ns=private_subscription_steady_ns,policy='No observed publisher or received message is permitted for this entire run; first publisher violation and message count never reset.')
        write(out/'runtime_manifest.json',manifest)
        def artifact_stat(path):
            value=pathlib.Path(path).stat()
            return dict(dev=value.st_dev,ino=value.st_ino,size=value.st_size,mtime_ns=value.st_mtime_ns,ctime_ns=value.st_ctime_ns)
        def status_receive(action,message):
            observed=dict(received_unix_ns=time.time_ns(),
              active=[dict(uuid=bytes(status.goal_info.goal_id.uuid).hex(),status=int(status.status),
                accepted_stamp_ns=status.goal_info.stamp.sec*1_000_000_000+status.goal_info.stamp.nanosec)
                for status in message.status_list if status.status in (0,1,2,3)],
              all_statuses=[dict(uuid=bytes(status.goal_info.goal_id.uuid).hex(),status=int(status.status)) for status in message.status_list])
            with ownership_observation_lock:action_status[action]=observed
        for action in ['/navigate_to_pose','/follow_waypoints']:
            subscriptions.append(node.create_subscription(GoalStatusArray,action+'/_action/status',
              lambda message,a=action:status_receive(a,message),qos_profile_action_status_default))
        child_feedback={}
        def child_feedback_receive(message):
            observed=dict(received_unix_ns=time.time_ns(),goal_uuid=bytes(message.goal_id.uuid).hex(),
              feedback=message_to_ordereddict(message.feedback))
            with ownership_observation_lock:child_feedback.update(observed)
        subscriptions.append(node.create_subscription(NavigateToPose.Impl.FeedbackMessage,
          '/navigate_to_pose/_action/feedback',child_feedback_receive,QoSProfile(depth=10)))
        def waypoint_ownership():
            nonlocal first_private_publisher_violation
            timing={};phase=(time.monotonic_ns(),time.thread_time_ns())
            graph=[]
            for name,namespace in node.get_node_names_and_namespaces():
                graph.append(dict(name=name,namespace=namespace,
                  clients=get_action_client_names_and_types_by_node(node,name,namespace),
                  servers=get_action_server_names_and_types_by_node(node,name,namespace),
                  service_clients=node.get_client_names_and_types_by_node(name,namespace)))
            phase=phase_mark(timing,'all_node_graph_queries',phase)
            bt=dict(bt_binding) if bt_binding is not None else {}
            if bt_binding is not None:
                pid=bt_binding['owned_process_plan']['pid']
                mapped=pathlib.Path(f'/proc/{pid}/maps').read_text()
                private_publishers=[dict(name=info.node_name,namespace=info.node_namespace,type=info.topic_type)
                    for info in node.get_publishers_info_by_topic(private_goal_topic)]
                with ownership_observation_lock:
                    if private_publishers and first_private_publisher_violation is None:
                        first_private_publisher_violation=dict(observed_unix_ns=time.time_ns(),publishers=private_publishers)
                    observed_private_messages=private_goal_messages;observed_first_private=copy.deepcopy(first_private_goal)
                    observed_publisher_violation=copy.deepcopy(first_private_publisher_violation)
                bt.update(current_process_identity=process_identity(pid),
                  current_mapped_files=sorted({line.split()[-1] for line in mapped.splitlines() if len(line.split())>=6}),
                  artifacts=[dict(item,current_stat=artifact_stat(item['path'])) for item in bt_binding['artifacts']],
                  bt_subscriptions=node.get_subscriber_names_and_types_by_node('bt_navigator','/'),
                  private_goal_publishers=private_publishers,first_private_goal_publisher_violation=observed_publisher_violation,
                  private_goal_subscribers=[dict(name=info.node_name,namespace=info.node_namespace,type=info.topic_type)
                    for info in node.get_subscriptions_info_by_topic(private_goal_topic)],
                  received_private_goal_messages=observed_private_messages,first_received_private_goal=observed_first_private)
            phase=phase_mark(timing,'native_process_maps_stat_and_ingress',phase)
            ownership=waypoint_service_ownership(graph,exclusive_graph,bt)
            phase=phase_mark(timing,'ownership_classification',phase)
            publishers={action:[dict(name=info.node_name,namespace=info.node_namespace)
              for info in node.get_publishers_info_by_topic(action+'/_action/status')]
              for action in ['/navigate_to_pose','/follow_waypoints']}
            with ownership_observation_lock:
                observed_action_status=copy.deepcopy(action_status);observed_child_feedback=copy.deepcopy(child_feedback)
            ownership.update(checked_unix_ns=time.time_ns(),status_publishers=publishers,
              status_publishers_match=publishers['/navigate_to_pose']==[dict(name='bt_navigator',namespace='/')] and
                publishers['/follow_waypoints']==[dict(name='waypoint_follower',namespace='/')],
              navigate_to_pose_active=observed_action_status.get('/navigate_to_pose',{}).get('active',[]),
              follow_waypoints_active=observed_action_status.get('/follow_waypoints',{}).get('active',[]),
              observed_status=observed_action_status,
              latest_child_feedback=observed_child_feedback,
              status_boundary='Reliable transient-local subscription created before new owned servers. No sample means no accepted transition observed; status is not a periodic health heartbeat. Child UUID is observed, parent association is contextual under exclusive client ownership.')
            phase_mark(timing,'status_observation_snapshot',phase)
            ownership['producer_audit_timing']=timing
            return ownership
    def source_health():
        now=node.get_clock().now().nanoseconds/1e9
        for topic,limit in [(args.odom_topic,.3),(args.scan_topic,.3),('/lightning/loc_status',.2),('/lightning/pose_valid',.2),(args.camera_topic,.5)]:
            if topic not in latest:raise RuntimeError('Missing source '+topic)
            received,msg=latest[topic]
            if time.monotonic()-received>limit:raise RuntimeError('Stale receive '+topic)
            if hasattr(msg,'header'):
                age=now-msg.header.stamp.sec-msg.header.stamp.nanosec/1e9
                if not -.05<=age<=limit:raise RuntimeError('Stale/future stamp '+topic)
        if latest['/lightning/loc_status'][1].data!=1 or not latest['/lightning/pose_valid'][1].data:
            raise RuntimeError('Localization is not NORMAL with valid pose')
        for target,source in [('odom','base_link'),('map','odom')]:
            transform=tf.lookup_transform(target,source,Time());age=now-transform.header.stamp.sec-transform.header.stamp.nanosec/1e9
            if not -.05<=age<=.3:raise RuntimeError('Stale/future actual TF '+source+' -> '+target)
    def pump(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end and rclpy.ok():
            if not shutdown_started and not stopped:deadline_guard.check_live()
            # Jazzy caches ready callbacks by timeout kwargs. A changing tail timeout
            # restarts at the first subscription and can starve later sources.
            # Fixed 2ms may exceed this wall window by one idle wait; callbacks
            # can still overrun. Original health/deadline checks run afterwards.
            rclpy.spin_once(node,timeout_sec=.002)
    def wait(future,timeout=3.,operation=None):
        started=time.monotonic();end=started+timeout;outcome=None
        def rpc_event(kind,**kw):
            if operation:event(kind,operation=operation,elapsed_sec=time.monotonic()-started,timeout_sec=timeout,**kw)
        rpc_event('rpc_begin')
        try:
            while not future.done() and time.monotonic()<end and not stopped:rclpy.spin_once(node,timeout_sec=.02)
            if not future.done():
                future.cancel();outcome='rpc_stopped' if stopped else 'rpc_timeout'
                rpc_event(outcome,cancel_requested=True)
                raise TimeoutError((operation or 'service response')+(' stopped' if stopped else ' timeout'))
            response=future.result()
        except Exception as e:
            if not future.done():future.cancel()
            if operation and outcome is None:
                rpc_event('rpc_error',error=repr(e))
                raise RuntimeError(operation+': '+repr(e)) from e
            raise
        rpc_event('rpc_complete')
        return response
    runtime=out/'runtime_install';shutil.copytree(selected_install,runtime)
    receipt=json.loads((out/'source_build_receipt.json').read_text())
    commit=receipt.get('repository_commit','')
    if len(commit)!=40 or any(c not in '0123456789abcdef' for c in commit):raise RuntimeError('Build receipt requires full repository_commit SHA')
    if receipt.get('source_files_match_commit') is not True:raise RuntimeError('Build receipt must prove selected source files match its commit')
    declared=receipt.get('artifact_sha256',{})
    required=['lib/librfn_goal_cell_binding.so','lib/libsegment_controller_core.so','lib/libsafe_lattice_planner.so','lib/robot_feasible_navigation/segment_controller_server'] if args.variant=='segmented' else ['lib/libofficial_lattice_rpp_adapter.so']
    for rel in required:
        if rel not in declared:raise RuntimeError('Missing required artifact in build receipt: '+rel)
    for rel,digest in declared.items():
        artifact=(runtime/rel).resolve()
        if not artifact.is_relative_to(runtime.resolve()) or not artifact.is_file() or sha(artifact)!=digest:raise RuntimeError('Selected runtime does not match concrete build receipt: '+rel)
    env=os.environ.copy();env['AMENT_PREFIX_PATH']=str(runtime)+':/opt/ros/jazzy';env['LD_LIBRARY_PATH']=str(runtime/'lib')+':/opt/ros/jazzy/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu'
    lifecycle_prefix=pathlib.Path(args.lifecycle_install).resolve() if args.lifecycle_install else None
    lifecycle_env=env.copy()
    if lifecycle_prefix:lifecycle_env['LD_LIBRARY_PATH']=str(lifecycle_prefix/'lib')+':'+env['LD_LIBRARY_PATH']
    message_runtime=out/'message_runtime';shutil.copytree(pathlib.Path(args.message_install).resolve(),message_runtime)
    recorder_env=env.copy()
    recorder_env['AMENT_PREFIX_PATH']=str(message_runtime)+':'+env['AMENT_PREFIX_PATH']
    recorder_env['LD_LIBRARY_PATH']=str(message_runtime/'lib')+':'+env['LD_LIBRARY_PATH']
    recorder_env['PYTHONPATH']=str(message_runtime/'lib/python3.12/site-packages')+':'+env.get('PYTHONPATH','')
    recorder_env['PYTHONDONTWRITEBYTECODE']='1'
    recorder_name='matrix_recorder_'+str(os.getpid());manifest['recorder_node_name']=recorder_name
    custom_types=['livox_ros_driver2/msg/CustomMsg','robots_dog_msgs/msg/LivoxBridgeDebug','robot_debug_msgs/msg/BuildInfo','lightning/msg/LightningDebug']
    verify_code='from rosidl_runtime_py.utilities import get_message\nfrom rclpy.type_support import check_for_type_support\nimport sys\nfor name in sys.argv[1:]:\n cls=get_message(name);check_for_type_support(cls);print(name)\n'
    checked=subprocess.run(['/usr/bin/python3','-c',verify_code,*custom_types],env=recorder_env,text=True,capture_output=True,check=True)
    manifest['message_runtime']=dict(source=str(pathlib.Path(args.message_install).resolve()),required_types=custom_types,
      import_check_stdout=checked.stdout,files_sha256={str(p.relative_to(message_runtime)):sha(p) for p in message_runtime.rglob('*') if p.is_file()})
    plugins='\n'.join(p.read_text() for p in (runtime/'share').rglob('*.xml') if 'plugin' in p.name)
    if manifest['variant']=='hybrid' and 'official_lattice_rpp_adapter::CheckedHybrid' not in plugins:raise RuntimeError('CheckedHybrid not installed; refusing an unguarded fallback')
    manifest['environment']={k:env.get(k) for k in ['ROS_DOMAIN_ID','ROS_AUTOMATIC_DISCOVERY_RANGE','RMW_IMPLEMENTATION','FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE','AMENT_PREFIX_PATH','LD_LIBRARY_PATH']}
    manifest['runtime_install_files']={str(p.relative_to(runtime)):sha(p) for p in runtime.rglob('*') if p.is_file()}
    write(out/'runtime_manifest.json',manifest)
    def launch(name,cmd,process_env=None):
        if stopped:raise InterruptedError('Stop requested before starting '+name)
        deadline_guard.check_live()
        owned_cmd,process_env=deadline_guard.owned_command(name,cmd,env if process_env is None else process_env)
        log=(out/(name+'.log')).open('w');logs.append(log);deadline_guard.runtime_children_started=True
        proc=subprocess.Popen(owned_cmd,env=process_env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);procs.append((name,proc))
        deadline_guard.wait_for_child(name,proc.pid);event('process_started',name=name,pid=proc.pid,argv=cmd,deadline_pidfd_registered=True)
    try:
        pump(2.)
        for topic in ['/cmd_vel_nav','/cmd_vel','/cmd_vel_safe']:
            if node.get_publishers_info_by_topic(topic):raise RuntimeError('Existing velocity publisher requires owner cleanup before this run: '+topic)
        # Recorder covers preflight source data, all lifecycle transitions, full maps, camera and actions.
        launch('recorder',['/opt/ros/jazzy/bin/ros2','bag','record','--node-name',recorder_name,'-a','--include-hidden-topics','--exclude-regex',EXCLUDED_LAYERS,'-s','mcap','--storage-preset-profile','zstd_fast','--max-cache-size','268435456','-o',str(out/'bag')],recorder_env)
        deadline=time.monotonic()+args.preflight_timeout;preflight={}
        while time.monotonic()<deadline and not stopped:
            pump(.1)
            try:
                now=node.get_clock().now().nanoseconds/1e9
                for topic in [args.odom_topic,args.camera_topic,args.camera_info_topic,'/lightning/loc_status']:
                    if topic not in latest:raise ValueError('missing '+topic)
                    received,msg=latest[topic]
                    receipt_limit=.2 if topic=='/lightning/loc_status' else .3 if topic==args.odom_topic else .5
                    if topic!=args.camera_info_topic and time.monotonic()-received>receipt_limit:raise ValueError('stale receive '+topic)
                    if hasattr(msg,'header') and topic!=args.camera_info_topic:
                        age=now-msg.header.stamp.sec-msg.header.stamp.nanosec/1e9
                        if not -.05<=age<=(.3 if topic==args.odom_topic else .5):raise ValueError('stale/future header '+topic)
                if latest['/lightning/loc_status'][1].data!=1:raise ValueError('localization is not NORMAL')
                for target,source in [('odom','base_link'),('map','odom')]:
                    t=tf.lookup_transform(target,source,Time());age=now-t.header.stamp.sec-t.header.stamp.nanosec/1e9
                    if not -.05<=age<=.3:raise ValueError('stale/future actual TF '+source+' -> '+target)
                if counts.get(args.odom_topic,0)<3 or counts.get(args.camera_topic,0)<3:raise ValueError('waiting for source samples')
                image=latest[args.camera_topic][1];decoded=cv2.imdecode(np.frombuffer(bytes(image.data),np.uint8),cv2.IMREAD_COLOR)
                if decoded is None:raise ValueError('front camera cannot decode')
                info=latest[args.camera_info_topic][1]
                if decoded.shape[1]!=info.width or decoded.shape[0]!=info.height or info.k[0]<=0 or info.k[4]<=0:raise ValueError('camera calibration does not match decoded frame')
                (out/'front_camera_first.jpg').write_bytes(bytes(image.data));write(out/'front_camera_info.json',message_to_ordereddict(info))
                preflight=dict(ok=True,counts=counts.copy(),decoded_camera_size=[decoded.shape[1],decoded.shape[0]],note='Observed real sources; this observer publishes no sensor/status/TF data')
                break
            except Exception as e:preflight=dict(ok=False,reason=str(e))
        write(out/'preflight.json',preflight)
        if not preflight.get('ok'):raise RuntimeError('preflight failed: '+str(preflight))
        gate=out/'nav_safety_gate.py';shutil.copy2(NAV/'src/navigation/src/robot_navigo/scripts/nav_safety_gate.py',gate)
        manifest['gate_source_sha256']=sha(gate);write(out/'runtime_manifest.json',manifest)
        common=['--ros-args','--params-file',str(out/'params.yaml')]
        launch('nav_safety_gate',['/usr/bin/python3',str(gate)]+common)
        servers=[('nav2_map_server','map_server','map_server',[]),('nav2_planner','planner_server','planner_server',[]),
          ('nav2_controller','controller_server','controller_server',['-r','cmd_vel:=cmd_vel_nav']),
          ('nav2_velocity_smoother','velocity_smoother','velocity_smoother',['-r','cmd_vel:=cmd_vel_nav','-r','cmd_vel_smoothed:=cmd_vel']),
          ('nav2_bt_navigator','bt_navigator','bt_navigator',['-r','goal_pose:='+private_goal_topic] if args.with_waypoints else [])]
        if args.with_waypoints:servers.append(('nav2_waypoint_follower','waypoint_follower','waypoint_follower',[]))
        servers.append(('nav2_lifecycle_manager','lifecycle_manager','lifecycle_manager',[]))
        for package,exe,name,remaps in servers:
            exe_path=str(runtime/'lib/robot_feasible_navigation/segment_controller_server') if name=='controller_server' and args.variant=='segmented' else '/opt/ros/jazzy/lib/'+package+'/'+exe
            if name=='lifecycle_manager' and lifecycle_prefix:exe_path=str(lifecycle_prefix/'lib/nav2_lifecycle_manager/lifecycle_manager')
            if name=='bt_navigator' and args.with_waypoints:bt_launch_steady_ns=time.monotonic_ns()
            launch(name,[exe_path]+common+remaps,lifecycle_env if name=='lifecycle_manager' else None)
            if name=='bt_navigator' and args.with_waypoints:
                pid=dict(procs)[name].pid
                bt_binding=dict(run_id=deadline_guard.manifest['run_id'],private_goal_topic=private_goal_topic,
                  subscription_created_steady_ns=private_subscription_steady_ns,bt_launch_steady_ns=bt_launch_steady_ns,
                  initial_process_identity=process_identity(pid),owned_process_plan=dict(role=name,pid=pid,
                    run_id=deadline_guard.manifest['run_id'],executed_argv=deadline_guard.children[name]['executed_argv']))
        write(out/'processes.json',{name:p.pid for name,p in procs})
        for name in ['map_server','planner_server','controller_server','velocity_smoother','bt_navigator']+(['waypoint_follower'] if args.with_waypoints else []):
            service='/'+name+'/get_state';client=node.create_client(GetState,service);deadline=time.monotonic()+90;attempts=0
            while time.monotonic()<deadline and not stopped and rclpy.ok():
                pump(min(.1,max(0.,deadline-time.monotonic())))
                if stopped or not rclpy.ok() or not client.service_is_ready():continue
                if time.monotonic()>=deadline:continue
                attempts+=1
                try:
                    future=client.call_async(GetState.Request())
                    state=wait(future,timeout=min(3.,max(0.,deadline-time.monotonic())),operation=service)
                except TimeoutError:
                    remaining=deadline-time.monotonic()
                    if stopped or not rclpy.ok() or remaining<=0:raise
                    event('lifecycle_poll_retry',node=name,service=service,remaining_sec=remaining,attempts=attempts)
                    continue
                if not stopped and rclpy.ok() and time.monotonic()<deadline and state.current_state.id==3:break
            else:raise RuntimeError(name+' did not activate')
        params={}
        for name in ['map_server','planner_server','controller_server','local_costmap/local_costmap','global_costmap/global_costmap','velocity_smoother','bt_navigator','nav_safety_gate']+(['waypoint_follower'] if args.with_waypoints else []):
            lc=node.create_client(ListParameters,'/'+name+'/list_parameters');gc=node.create_client(GetParameters,'/'+name+'/get_parameters')
            if not lc.wait_for_service(timeout_sec=2.):raise RuntimeError('parameter service unavailable '+name)
            names=wait(lc.call_async(ListParameters.Request()),timeout=10.,operation='/'+name+'/list_parameters').result.names;req=GetParameters.Request();req.names=names
            params[name]={n:message_to_ordereddict(v) for n,v in zip(names,wait(gc.call_async(req),timeout=10.,operation='/'+name+'/get_parameters').values)}
        write(out/'runtime_parameters.json',params)
        if args.with_waypoints:
            expected=manifest['waypoint_contract']['parameters']
            fields={1:'bool_value',2:'integer_value',3:'double_value',4:'string_value'}
            for key,value in expected.items():
                observed=params['waypoint_follower'].get(key,{})
                expected_type=1 if type(value) is bool else 2 if type(value) is int else 3 if type(value) is float else 4
                if observed.get('type')!=expected_type or observed.get(fields[expected_type])!=value:
                    raise RuntimeError('Native waypoint effective parameter mismatch: '+key)
        static_snapshots=capture_static_layers(node,out,recorder_name,pump,lambda:stopped)
        actual_maps={}
        for name,size in [('global_costmap',2000),('local_costmap',600)]:
            client=node.create_client(GetCostmap,'/'+name+'/get_costmap')
            if not client.wait_for_service(timeout_sec=3.):raise RuntimeError('missing actual costmap service')
            msg=wait(client.call_async(GetCostmap.Request())).map
            if abs(msg.metadata.resolution-.01)>1e-6 or msg.metadata.size_x!=size or msg.metadata.size_y!=size:raise RuntimeError('actual rolling geometry mismatch '+name)
            (out/(name+'_initial.cdr')).write_bytes(serialize_message(msg));actual_maps[name]=message_to_ordereddict(msg.metadata)
        deadline=time.monotonic()+6.
        while time.monotonic()<deadline and any(counts.get('/'+n+'/costmap_raw',0)==0 for n in ['global_costmap','local_costmap']):pump(.1)
        if any(counts.get('/'+n+'/costmap_raw',0)==0 for n in ['global_costmap','local_costmap']):raise RuntimeError('full raw costmap publication not observed')
        # These observer payloads were used only to prove startup publication.
        # GetCostmap already saved exact initial CDR; the recorder remains subscribed.
        startup_raw_counts={topic:counts[topic] for topic in startup_costmap_observers}
        for topic,subscription in startup_costmap_observers.items():
            node.destroy_subscription(subscription);subscriptions.remove(subscription);latest.pop(topic,None)
        manifest['costmap_observer_scope']=dict(mode='startup_only',retained_startup_counts=startup_raw_counts,
          initial_rpc_cdr_preserved=True,recorder_continuous_costmap_topics_unchanged=True)
        event('startup_costmap_observers_detached',retained_startup_counts=startup_raw_counts,
          boundary='Only evidence-observer subscriptions removed; recorder and costmap publishers unchanged')
        identities={}
        for name,p in procs:
            maps=pathlib.Path(f'/proc/{p.pid}/maps').read_text()
            libraries=set(line.split()[-1] for line in maps.splitlines() if len(line.split())>=6 and
              ('/opt/ros/jazzy/lib/' in line or str(runtime/'lib') in line or
               (lifecycle_prefix and str(lifecycle_prefix/'lib') in line)))
            identities[name]=dict(pid=p.pid,cmdline=pathlib.Path(f'/proc/{p.pid}/cmdline').read_bytes().replace(b'\0',b' ').decode(),maps=maps,
              selected_libraries_sha256={library:sha(library) for library in libraries if pathlib.Path(library).is_file()})
        write(out/'runtime_identities.json',identities)
        if args.with_waypoints:
            bt_binding['artifacts']=[]
            for path in ['/opt/ros/jazzy/lib/nav2_bt_navigator/bt_navigator','/opt/ros/jazzy/lib/libnav2_navigate_to_pose_navigator.so']:
                before=artifact_stat(path);observed=sha(path);after=artifact_stat(path)
                expected=manifest['waypoint_contract']['underlay_files_sha256'][path]
                if before!=after or observed!=expected:raise RuntimeError('Native BT self-client artifact identity changed: '+path)
                bt_binding['artifacts'].append(dict(path=path,expected_sha256=expected,observed_sha256=observed,initial_stat=after))
            write(out/'native_bt_self_client_binding.json',bt_binding)
            manifest['native_bt_self_client_binding_sha256']=sha(out/'native_bt_self_client_binding.json')
            write(out/'runtime_manifest.json',manifest)
        recording_topics=[args.camera_topic,args.camera_info_topic,args.odom_topic,args.scan_topic,
          '/livox/lidar','/livox/imu','/lightning_bridge/debug','/lightning/debug','/debug/build_info',
          '/tf','/tf_static','/global_costmap/costmap_raw','/local_costmap/costmap_raw']
        recording_subscriptions={topic:[info.node_name for info in node.get_subscriptions_info_by_topic(topic)] for topic in recording_topics}
        write(out/'recording_subscriptions.json',recording_subscriptions)
        missing=[topic for topic,names in recording_subscriptions.items() if recorder_name not in names]
        if missing:raise RuntimeError('Recorder subscriptions missing before READY: '+repr(missing))
        # Startup may take time. Recheck the current source values immediately before
        # authorizing a separate goal pilot, rather than reusing an earlier preflight.
        pump(.2);ready_now=node.get_clock().now().nanoseconds/1e9
        for topic in [args.odom_topic,'/lightning/loc_status',args.camera_topic,args.camera_info_topic]:
            if topic not in latest:raise RuntimeError('Source disappeared before ready: '+topic)
            received,msg=latest[topic];limit=.2 if topic=='/lightning/loc_status' else .3 if topic==args.odom_topic else .5
            if topic!=args.camera_info_topic and time.monotonic()-received>limit:raise RuntimeError('Stale source before ready: '+topic)
            if topic in [args.odom_topic,args.camera_topic]:
                age=ready_now-msg.header.stamp.sec-msg.header.stamp.nanosec/1e9
                if not -.05<=age<=limit:raise RuntimeError('Stale/future source stamp before ready: '+topic)
        if latest['/lightning/loc_status'][1].data!=1:raise RuntimeError('Localization left NORMAL before ready')
        for target,source in [('odom','base_link'),('map','odom')]:
            transform=tf.lookup_transform(target,source,Time());age=ready_now-transform.header.stamp.sec-transform.header.stamp.nanosec/1e9
            if not -.05<=age<=.3:raise RuntimeError('Stale/future actual TF before ready: '+source+' -> '+target)
        camera=latest[args.camera_topic][1];info=latest[args.camera_info_topic][1]
        decoded=cv2.imdecode(np.frombuffer(bytes(camera.data),np.uint8),cv2.IMREAD_COLOR)
        if decoded is None or decoded.shape[1]!=info.width or decoded.shape[0]!=info.height:raise RuntimeError('Front camera invalid before ready')
        source_health()
        if args.with_waypoints:
            ownership=waypoint_ownership()
            if not ownership['exclusive_nav_client'] or not ownership['status_publishers_match']:
                raise RuntimeError('Native waypoint exclusive action graph not established: '+repr(ownership))
            if ownership['navigate_to_pose_active'] or ownership['follow_waypoints_active']:
                raise RuntimeError('Unexpected action goal already active before native batch readiness')
            write(out/'waypoint_ownership_initial.json',ownership)
        readiness=dict(ready=True,time_ns=node.get_clock().now().nanoseconds,variant=args.variant,actual_costmaps=actual_maps,
          params_sha256=sha(out/'params.yaml'),build_receipt_sha256=manifest['build_receipt_sha256'],source_counts=counts,
          action_server='/navigate_to_pose',goal_tolerances=manifest['goal_tolerances'],goal_client_authorization='Wait for root instruction; this file alone does not dispatch navigation')
        if args.with_waypoints:
            readiness.update(waypoint_action_server='/follow_waypoints',waypoint_contract=manifest['waypoint_contract'],
              waypoint_ownership=ownership,mission_deadline=dict(run_id=deadline_guard.manifest['run_id'],
              manifest_sha256=deadline_guard.digest,launcher_boot_id=deadline_guard.manifest['launcher']['boot_id'],
              deadline_boottime_sec=deadline_guard.manifest['deadline_boottime_sec']))
        # Graph/process auditing cannot starve callback pumping or the READY heartbeat.
        # Every audit is still complete; its original checked timestamp is authoritative.
        if args.with_waypoints:ownership_worker=OwnershipAuditWorker(waypoint_ownership,ownership)
        producer_timing_stream=(out/'ready_producer_timing.jsonl').open('w',buffering=65536)
        manifest['ready_producer']=dict(heartbeat_period_sec=.05,ownership_audit_period_sec=.2,
          ready_lease_ns=500_000_000,ownership_lease_ns=500_000_000,
          policy='Absolute heartbeat; complete worker audit retains checked timestamp; READY expiry never exceeds that original audit expiry; source_health last before publication',
          timing='Wall/thread CPU per phase; callback-pump, audit, health and atomic-write timings; no physical input limits changed',
          callback_wait_policy='Fixed .002s per spin, including overrun cycles, preserves Jazzy ready iterator fairness; at most one extra idle wait beyond a window, single callbacks remain nonpreemptible')
        write(out/'runtime_manifest.json',manifest)
        health_started=(node.get_clock().now().nanoseconds,time.monotonic_ns())
        pump(.02);source_health()
        readiness['producer_scan_observation']=scan_observation(latest,args.scan_topic,health_started,
          (node.get_clock().now().nanoseconds,time.monotonic_ns()),None)
        readiness['time_ns']=node.get_clock().now().nanoseconds
        readiness['valid_until_ns']=readiness['time_ns']+500_000_000
        if args.with_waypoints:readiness['valid_until_ns']=min(readiness['valid_until_ns'],ownership['checked_unix_ns']+500_000_000)
        ready_storage.publish(readiness,compact=True)
        event('navigation_ready');print(str(out/'ready.json'),flush=True)
        heartbeat_deadline=time.monotonic();last_write_finished_ns=time.monotonic_ns()
        previous_write_timing=None;previous_audit_sequence=None;last_timing_flush=time.monotonic()
        while not stopped:
            if not shutdown_started:deadline_guard.check_live()
            cycle_started=(time.monotonic_ns(),time.thread_time_ns());phase=cycle_started;timing={}
            remaining=heartbeat_deadline-time.monotonic()
            if remaining>0.:pump(remaining)
            else:rclpy.spin_once(node,timeout_sec=.002)
            phase=phase_mark(timing,'callback_pump',phase)
            dead=[(n,p.returncode) for n,p in procs if p.poll() is not None]
            if dead:raise RuntimeError('owned process exited '+repr(dead))
            previous=readiness['ready'];audit_sequence=None;health_started=None
            readiness['producer_scan_observation']=None
            try:
                if args.with_waypoints:
                    observed,audit_sequence=ownership_worker.snapshot(node.get_clock().now().nanoseconds)
                    # A current received private goal is fatal immediately, even if
                    # the latest full worker observation preceded that callback.
                    with ownership_observation_lock:
                        ingress_violated=private_goal_messages>0 or first_private_publisher_violation is not None
                    if ingress_violated:raise RuntimeError('Native BT private goal ingress violated')
                    readiness['waypoint_ownership']=observed
                    if not observed['exclusive_nav_client'] or not observed['status_publishers_match']:
                        raise RuntimeError('Native waypoint action ownership changed; shared NavigateToPose is unsupported')
                phase=phase_mark(timing,'ownership_snapshot',phase)
                # This recheck is AFTER graph work, immediately before publication.
                health_started=(node.get_clock().now().nanoseconds,time.monotonic_ns())
                source_health();phase=phase_mark(timing,'source_health',phase)
                readiness.update(ready=True,reason=None)
            except Exception as e:readiness.update(ready=False,reason=str(e))
            if health_started is not None:
                readiness['producer_scan_observation']=scan_observation(latest,args.scan_topic,health_started,
                  (node.get_clock().now().nanoseconds,time.monotonic_ns()),readiness.get('reason'))
            issued=node.get_clock().now().nanoseconds
            expiry=issued+500_000_000
            if args.with_waypoints and readiness['ready']:
                expiry=min(expiry,readiness['waypoint_ownership']['checked_unix_ns']+500_000_000)
            readiness.update(time_ns=issued,valid_until_ns=expiry)
            audit_state=ownership_worker.state() if ownership_worker is not None else None
            readiness['producer_timing']=dict(heartbeat_started_steady_ns=cycle_started[0],phases=timing,
              previous_atomic_write=previous_write_timing,ownership_audit=audit_state,
              ownership_audit_sequence_reused=audit_sequence is not None and audit_sequence==previous_audit_sequence,
              ownership_checked_age_ns=issued-readiness['waypoint_ownership']['checked_unix_ns'] if args.with_waypoints else None)
            phase=(time.monotonic_ns(),time.thread_time_ns())
            ready_storage.publish(readiness,compact=True)
            finished=phase_mark(timing,'atomic_ready_write',phase)
            timing_record=dict(issued_unix_ns=issued,ready=readiness['ready'],reason=readiness.get('reason'),
              phases=timing,audit=audit_state,audit_sequence=audit_sequence,
              reused_audit=audit_sequence is not None and audit_sequence==previous_audit_sequence,
              heartbeat_gap_ns=finished[0]-last_write_finished_ns,cycle_wall_ns=finished[0]-cycle_started[0],
              cycle_thread_cpu_ns=finished[1]-cycle_started[1])
            producer_timing_stream.write(json.dumps(timing_record,separators=(',',':'))+'\n')
            if time.monotonic()-last_timing_flush>=1.:
                producer_timing_stream.flush();last_timing_flush=time.monotonic()
            previous_write_timing=timing['atomic_ready_write'];previous_audit_sequence=audit_sequence
            last_write_finished_ns=finished[0]
            heartbeat_deadline=next_heartbeat_deadline(heartbeat_deadline,time.monotonic())
            if readiness['ready']!=previous:event('readiness_changed',ready=readiness['ready'],reason=readiness['reason'])
    except Exception as e:
        try:ready_storage.stop_false()
        except Exception:pass  # Owned process cleanup below must still run.
        write(out/'failed.json',dict(error=repr(e)));event('failure',error=repr(e));raise
    finally:
        shutdown_started=True
        deadline_guard.begin_shutdown('runtime_shutdown_started')
        shutdown_ready_error=None
        try:ready_storage.stop_false(node.get_clock().now().nanoseconds)
        except Exception as error:shutdown_ready_error=repr(error)
        if ownership_worker is not None:ownership_worker.request_stop()
        event('shutdown_started',requested_signals=stop_signals,ready_storage_error=shutdown_ready_error)
        # Stop command-producing servers first; keep smoother/gate/recorder alive for the zero tail.
        for name,p in procs:
            if name in ['waypoint_follower','bt_navigator','controller_server'] and p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        event('command_producers_stop_requested')
        pump(1.)
        for name,p in reversed(procs):
            if name!='recorder' and p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        event('smoother_gate_stop_requested',command_producer_zero_tail_sec=1.)
        pump(.5)
        for name,p in procs:
            if name=='recorder' and p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        event('recorder_stop_requested',additional_zero_tail_sec=.5)
        for name,p in procs:
            try:p.wait(timeout=30. if name=='recorder' else 8.)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=3.)
        deadline_guard.recording_closed('all_owned_processes_waited')
        ready_storage.stop_false()
        ready_storage.materialize()
        event('owned_processes_closed',returncodes={name:p.returncode for name,p in procs})
        for log in logs:log.close()
        metadata=out/'bag/metadata.yaml'
        if metadata.exists():
            event('recording_audit_started',note='Recorder is already closed; MCAP decoding and hashing can take time')
            md=yaml.safe_load(metadata.read_text())['rosbag2_bagfile_information'];topic_counts={x['topic_metadata']['name']:x['message_count'] for x in md['topics_with_message_count']}
            try:snapshot_audit=audit_static_layers(out/'bag',static_snapshots)
            except Exception as error:snapshot_audit=dict(passed=False,error=repr(error))
            write(out/'static_layer_recording_audit.json',snapshot_audit)
            required=[args.camera_topic,args.camera_info_topic,args.odom_topic,args.scan_topic,'/livox/lidar','/livox/imu','/lightning_bridge/debug','/lightning/debug','/debug/build_info','/tf','/tf_static','/map','/global_costmap/costmap_raw','/local_costmap/costmap_raw','/cmd_vel_nav','/cmd_vel','/cmd_vel_safe','/simulation/command_adapter/policy_cmd']
            required += [snapshot['recorded_topic'] for snapshot in static_snapshots]
            write(out/'final_audit.json',dict(message_count=md['message_count'],topic_counts=topic_counts,required_topics_nonempty={t:topic_counts.get(t,0)>0 for t in required},
              static_layer_exactly_once_cdr_verified=snapshot_audit['passed'],mcap_sha256={p.name:sha(p) for p in (out/'bag').glob('*.mcap')},note='Sequential decode and per-case action/results remain required before acceptance.'))
            event('recording_audit_completed')
        if ownership_worker is not None:event('ownership_audit_worker_closed',joined=ownership_worker.join())
        if producer_timing_stream is not None:producer_timing_stream.close()
        event('stopped',observed_counts=counts);event_stream.close();node.destroy_node();rclpy.shutdown()

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--map',required=True);p.add_argument('--footprint',required=True,help='JSON list of body-frame [x,y] vertices')
    p.add_argument('--padding',required=True,type=float);p.add_argument('--variant',required=True,choices=['official','segmented','hybrid']);p.add_argument('--output',required=True)
    p.add_argument('--odom-topic',default='/odom/current_pose');p.add_argument('--scan-topic',default='/laser_scan');p.add_argument('--camera-topic',default='/camera/front/image/compressed');p.add_argument('--camera-info-topic',default='/camera/front/camera_info')
    p.add_argument('--template',default=str(BASE/'matrix_profile.yaml'))
    p.add_argument('--source-snapshot',help='Validated source manifest for a Git-free frozen deployment; defaults to observing canonical Git')
    p.add_argument('--official-install');p.add_argument('--segmented-install')
    p.add_argument('--rotate-threshold',type=float,help='Explicit RPP heading threshold override; otherwise frozen template value')
    p.add_argument('--xy-tolerance',type=float,default=.15);p.add_argument('--yaw-tolerance',type=float,default=.08)
    p.add_argument('--build-receipt');p.add_argument('--message-install');p.add_argument('--preflight-timeout',type=float,default=30.);p.add_argument('--run',action='store_true')
    p.add_argument('--lifecycle-install',help='Optional isolated lifecycle executable/core pair, shared by all variants')
    p.add_argument('--lifecycle-build-receipt',help='Exact lifecycle artifact receipt; required with --lifecycle-install')
    p.add_argument('--max-runtime-sec',type=runtime_seconds,default=600.,help='Fixed deadline from launcher process start, in (0,7200] seconds; default 600')
    p.add_argument('--with-waypoints',action='store_true',help='Activate the native Jazzy FollowWaypoints server for an independently authorized ordered batch client')
    args=p.parse_args();out,install,manifest=prepare(args)
    if args.run:
        deadline_guard=None;ready_storage=None
        try:
            deadline_guard=arm_deadline(out,args.max_runtime_sec)
            ready_storage=LiveReadyStorage(out,deadline_guard.manifest);ready_storage.start()
            manifest['recording_deadline']=deadline_guard.manifest;manifest['ready_storage']=ready_storage.storage
            write(out/'runtime_manifest.json',manifest)
            run(args,out,install,manifest,deadline_guard,ready_storage)
        except Exception as error:
            if ready_storage is not None:
                try:ready_storage.stop_false()
                except Exception:pass  # Preserve the original failure and reach deadline cleanup.
            elif not (out/'ready.json').exists() and not (out/'ready.json').is_symlink():
                write(out/'ready.json',dict(ready=False,stopped_at_ns=time.time_ns()))
            if not (out/'failed.json').exists():write(out/'failed.json',dict(error=repr(error),stage='receipt_or_runtime_setup'))
            raise
        finally:
            if deadline_guard is not None:
                if not deadline_guard.runtime_children_started:
                    deadline_guard.recording_closed('launcher_exiting_before_any_runtime_children')
                if deadline_guard.closing:
                    if ready_storage is not None and ready_storage.created:
                        ready_storage.stop_false();ready_storage.materialize()
                    deadline_guard.process.wait(timeout=2.)
    else:print(str(out/'runtime_manifest.json'))
if __name__=='__main__':main()

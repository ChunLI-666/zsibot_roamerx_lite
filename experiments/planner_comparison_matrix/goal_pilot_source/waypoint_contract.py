"""Non-ROS contracts for one ordered native FollowWaypoints experiment."""
from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import threading
import time


ACTION = '/follow_waypoints'


class ReadyReadCapture:
    """Path-like single read for the unchanged frozen READY validator.

    The bytes and fd identity describe the same open file even across rename.
    Call bounds contain the frozen validator's unknown internal wall-time read;
    they are never presented as that exact timestamp or used in decisions.
    """
    def __init__(self, path):
        self.path = path
        self.raw = None
        self.reads = 0
        self.timing = {}
        self.fd_identity = {}
        self.capture_errors = []

    def _identity(self, stream, stage):
        try:
            value = os.fstat(stream.fileno())
            self.fd_identity[stage] = dict(dev=value.st_dev, ino=value.st_ino,
                size=value.st_size, mtime_ns=value.st_mtime_ns, ctime_ns=value.st_ctime_ns)
        except Exception as exc:
            # Optional metadata must never replace read/validation behavior.
            self.capture_errors.append(dict(stage=stage, type=type(exc).__name__, message=str(exc)))

    def read_bytes(self):
        self.reads += 1
        self.timing.update(read_begin_unix_ns=time.time_ns(), read_begin_steady_ns=time.monotonic_ns())
        try:
            # Path.read_bytes() uses this same open/read operation. No second read.
            with self.path.open('rb') as stream:
                self._identity(stream, 'before_read')
                self.raw = stream.read()
                self._identity(stream, 'after_read')
            return self.raw
        finally:
            self.timing.update(read_end_unix_ns=time.time_ns(), read_end_steady_ns=time.monotonic_ns())

    def failure(self, exc, call_bounds):
        return dict(raw_bytes=self.raw, metadata=dict(
            schema='matrix_native_ready_first_failed_read_v1',
            exact_failed_read_bytes_retained=self.raw is not None,
            byte_count=None if self.raw is None else len(self.raw),
            sha256=None if self.raw is None else hashlib.sha256(self.raw).hexdigest(),
            raw_file=None if self.raw is None else 'ready_read_failure.raw',
            exception_type=type(exc).__name__, exception_message=str(exc),
            read_calls=self.reads, fd_identity=self.fd_identity, read_timing=self.timing,
            frozen_call_bounds=call_bounds, capture_errors=self.capture_errors,
            frozen_internal_now_ns=None,
            timing_boundary='Original frozen read_ready clock is unchanged. Call begin/end bound its internal now only when decode reached that check; exact internal now and decode end are unavailable.',
            cache_boundary='These are the first failed call bytes; latest_ready remains a separate last-passing cache. No lease, ownership or physical-input predicate is changed.'))



def producer_scan_confirmation_failure(raw, variant, binding, native_binding, loaded_sources,
        own_uuid, parent_sent_ns, now_ns, scan_topic):
    """Read-only revalidation of a current strict not-ready scan failure.

    Frozen read_ready rejects not_ready before metadata. Never alter that document
    or feed a fabricated ready=True to it. Recheck the original metadata/bindings
    before the opt-in bounded confirmation may wait for a NEW strict READY.
    """
    ready=json.loads(raw)
    reason=ready.get('reason')
    if ready.get('ready') is not False or reason not in (
            'Stale/future stamp '+scan_topic,'Stale receive '+scan_topic):
        raise ValueError('Not a sole producer same-scan aging failure')
    if binding is None or native_binding is None or own_uuid is None or parent_sent_ns is None:
        raise ValueError('Producer scan confirmation requires the original dispatched binding/UUID')
    issued,expiry=ready.get('time_ns',0),ready.get('valid_until_ns',0)
    if not isinstance(issued,int) or not isinstance(expiry,int) or not 0<expiry-now_ns<=550_000_000 or not -50_000_000<=now_ns-issued<=500_000_000:
        raise ValueError('launcher_lease_missing_expired_or_future')
    immutable={k:ready.get(k) for k in ['variant','action_server','goal_tolerances','params_sha256','build_receipt_sha256']}
    if ready.get('variant')!=variant or ready.get('action_server')!='/navigate_to_pose' or immutable!=binding:
        raise ValueError('launcher_runtime_binding_changed')
    if ready.get('goal_tolerances')!=dict(xy=.15,yaw=.08,stateful=False):
        raise ValueError('Original reviewed goal tolerances changed')
    for key in ['params_sha256','build_receipt_sha256']:
        value=ready.get(key)
        if not isinstance(value,str) or len(value)!=64 or any(c not in '0123456789abcdef' for c in value):
            raise ValueError('missing_runtime_binding:'+key)
    native=waypoint_binding(ready,native_binding)
    if native['waypoint_contract']['client_sources_sha256']!=loaded_sources:
        raise ValueError('Loaded batch client source binding changed')
    validate_ownership(ready.get('waypoint_ownership',{}),now_ns,own_uuid,parent_sent_ns)
    observed=ready.get('producer_scan_observation',{})
    if not isinstance(observed,dict) or observed.get('topic')!=scan_topic or observed.get('frame_id')!='base_link' or observed.get('ranges',0)<=0 or observed.get('failure_reason')!=reason:
        raise ValueError('Producer failed scan context missing or mismatched')
    stamp=observed.get('header_ns');begin=observed.get('validation_clock_begin_ns');end=observed.get('validation_clock_end_ns')
    steady_begin=observed.get('validation_steady_begin_ns');steady_end=observed.get('validation_steady_end_ns');received=observed.get('receive_steady_seconds')
    if any(type(value) is not int or value<=0 for value in [stamp,begin,end,steady_begin,steady_end]) or begin>end or steady_begin>steady_end or not -50_000_000<=now_ns-end<=500_000_000:
        raise ValueError('Producer failed scan clock bounds invalid or stale')
    if stamp>begin+50_000_000:
        raise ValueError('Producer future scan is never eligible for aging confirmation')
    if type(received) not in (int,float) or not math.isfinite(received) or received<=0 or received>steady_end/1e9:
        raise ValueError('Producer scan receive time invalid')
    if reason=='Stale/future stamp '+scan_topic and begin-stamp<=300_000_000:
        raise ValueError('Producer old header is not established by its clock bounds')
    if reason=='Stale receive '+scan_topic and steady_begin/1e9-received<=.3:
        raise ValueError('Producer stale receive is not established by its clock bounds')
    return dict(ready=ready,sha256=hashlib.sha256(raw).hexdigest(),original_stamp_ns=stamp,
                issued_ns=issued,producer_scan_observation=observed)


def producer_scan_confirmation_outcome(issues,snapshot,original,ready,elapsed_ns,budget_ns,pending_valid):
    if not 0<=elapsed_ns<=budget_ns:return 'expired'
    if issues:return 'pending' if pending_valid else 'fatal'
    observation=ready.get('producer_scan_observation',{})
    own=snapshot.get('last',{}).get('scan',{})
    stamp=observation.get('header_ns',0);begin=observation.get('validation_clock_begin_ns',0);end=observation.get('validation_clock_end_ns',0)
    if (any(type(value) is not int or value<=0 for value in [stamp,begin,end]) or begin>end
            or not -50_000_000<=begin-stamp or end-stamp>300_000_000):return 'fatal'
    if (ready.get('ready') is True and ready.get('time_ns',0)>original['issued_ns']
            and observation.get('topic')==original['producer_scan_observation']['topic']
            and observation.get('frame_id')=='base_link' and observation.get('ranges',0)>0
            and stamp>original['original_stamp_ns'] and own.get('stamp_ns',0)>original['original_stamp_ns']):
        return 'recovered'
    return 'pending'


def validate_scan_aging_confirmation(seconds):
    """Opt-in callback confirmation, never a replacement scan-age threshold."""
    if (isinstance(seconds, bool) or not isinstance(seconds, (int, float))
            or not math.isfinite(seconds) or not 0. <= seconds <= .05):
        raise ValueError('--scan-aging-confirm-sec must be finite in [0,.05]; 0 retains strict cancellation')


def scan_confirmation_payload_valid(message):
    """Additional opt-in evidence; +inf is a normal no-return LaserScan bin."""
    scalars = [message.angle_min, message.angle_max, message.angle_increment,
               message.time_increment, message.scan_time, message.range_min, message.range_max]
    return (all(math.isfinite(value) for value in scalars)
            and message.angle_increment > 0. and message.angle_max >= message.angle_min
            and message.time_increment >= 0. and message.scan_time >= 0.
            and 0. <= message.range_min < message.range_max
            and bool(message.ranges)
            and all(math.isfinite(value) or value == math.inf for value in message.ranges))


def scan_aging_confirmation_eligible(issues, snapshot):
    """Only one old-header issue may wait; every other original guard stays fatal."""
    if set(issues) != {'scan:invalid_header_age'}:
        return False
    scan = snapshot.get('last', {}).get('scan', {})
    age, received = scan.get('header_age_s'), scan.get('receive_age_s')
    return (scan.get('frame_id') == 'base_link' and type(scan.get('stamp_ns')) is int
            and scan['stamp_ns'] > 0 and scan.get('ranges', 0) > 0
            and scan.get('confirmation_payload_valid') is True
            and isinstance(age, (int, float)) and math.isfinite(age) and age > .3
            and isinstance(received, (int, float)) and math.isfinite(received) and 0. <= received <= .3)


def scan_aging_confirmation_outcome(issues, snapshot, original_stamp_ns, elapsed_ns, budget_ns):
    """Use the original header; a late callback or a nonadvancing stamp cannot recover."""
    if not 0 <= elapsed_ns <= budget_ns:
        return 'expired'
    scan = snapshot.get('last', {}).get('scan', {})
    if issues:
        return 'pending' if scan_aging_confirmation_eligible(issues, snapshot) else 'fatal'
    age, received = scan.get('header_age_s'), scan.get('receive_age_s')
    fresh = (type(scan.get('stamp_ns')) is int and scan['stamp_ns'] > original_stamp_ns
             and scan.get('frame_id') == 'base_link' and scan.get('ranges', 0) > 0
             and scan.get('confirmation_payload_valid') is True
             and isinstance(age, (int, float)) and math.isfinite(age) and -.05 <= age <= .3
             and isinstance(received, (int, float)) and math.isfinite(received) and 0. <= received <= .3)
    return 'recovered' if fresh else 'fatal'


WAYPOINT_PARAMETERS = {
    'use_sim_time': False,
    'stop_on_failure': True,
    'loop_rate': 20,
    'action_server_result_timeout': 900.0,
    'global_frame_id': 'map',
    'waypoint_task_executor_plugin': 'wait_at_waypoint',
    'wait_at_waypoint.plugin': 'nav2_waypoint_follower::WaitAtWaypoint',
    'wait_at_waypoint.enabled': True,
    'wait_at_waypoint.waypoint_pause_duration': 1000,
}


def evidence_json(value, **kwargs):
    """Encode invalid numeric observations explicitly, after safety decisions.

    Never apply this to live caches or decision inputs. A tagged object retains
    the fact and location of NaN/Infinity in valid JSON instead of discarding an
    entire first-invalid/final report or silently turning an invalid value zero.
    """
    def encode(item):
        if isinstance(item, float) and not math.isfinite(item):
            return {'nonfinite_float': 'NaN' if math.isnan(item) else '+Infinity' if item > 0 else '-Infinity'}
        if isinstance(item, dict):
            return {key: encode(child) for key, child in item.items()}
        if isinstance(item, (list, tuple)):
            return [encode(child) for child in item]
        return item
    return json.dumps(encode(value), ensure_ascii=False, allow_nan=False, **kwargs)


def finite_velocity(value):
    return all(type(value.get(field)) in (int, float) and math.isfinite(value[field]) for field in ['vx', 'vy', 'wz'])


def strict_stopped(snapshot):
    """Fresh finite measured and commanded stop, using the original limits."""
    issues = []
    for key, linear_limit, angular_limit in [('odom', .03, .05), ('/cmd_vel_safe', .01, .01)]:
        value = snapshot.get('last', {}).get(key, {})
        finite = finite_velocity(value)
        if not finite:
            issues.append(key+':nonfinite_or_missing_stop_velocity')
        elif math.hypot(value['vx'], value['vy']) > linear_limit or abs(value['wz']) > angular_limit:
            issues.append(key+':not_stopped')
        if value.get('receive_age_s', math.inf) > .3:
            issues.append(key+':stale_stop_receive')
        if key == 'odom' and (value.get('frame_id') != 'odom' or value.get('child_frame_id') != 'base_link'
                             or not -.05 <= value.get('header_age_s', math.inf) <= .3):
            issues.append(key+':invalid_stop_frame_or_header_age')
    return not issues, issues


def validate_settle_budget(seconds):
    if isinstance(seconds, bool) or not isinstance(seconds, (int, float)) or not math.isfinite(seconds) or not .3 <= seconds <= 8.:
        raise ValueError('Stop observation must be in [.3,8] seconds to fit the fixed 30s shutdown reserve')


def angular_difference(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


def validate_route(document):
    """Preserve route order, including repeated XY with different headings."""
    if not isinstance(document, dict) or type(document.get('schema_version')) is not int or document.get('schema_version') != 1 or document.get('frame_id') != 'map':
        raise ValueError('Route requires schema_version=1 and frame_id=map')
    if document.get('executable', True) is not True:
        raise ValueError('Route is an unexecutable design document')
    budget = document.get('mission_budget_sec')
    if isinstance(budget, bool) or not isinstance(budget, (int, float)) or not math.isfinite(budget) or not 0 < budget <= 3300.:
        raise ValueError('Executable route requires a preregistered mission_budget_sec in (0,3300]')
    entries = document.get('waypoints')
    if not isinstance(entries, list) or not 2 <= len(entries) <= 256:
        raise ValueError('Route requires 2..256 ordered waypoints')
    identifiers = set()
    result = []
    for index, entry in enumerate(entries):
        if not isinstance(entry, dict):
            raise ValueError('Waypoint must be an object')
        identifier, pose = entry.get('id'), entry.get('pose')
        if not isinstance(identifier, str) or not identifier or identifier in identifiers:
            raise ValueError('Waypoint IDs must be nonempty and unique')
        if not isinstance(pose, list) or len(pose) != 3 or any(
                isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) for value in pose):
            raise ValueError('Waypoint pose must be finite [map_x,map_y,yaw_rad]')
        if entry.get('require_stopped') is not True:
            raise ValueError('Every waypoint requires measured and commanded stop')
        tags = entry.get('tags', [])
        if not isinstance(tags, list) or not all(isinstance(tag, str) for tag in tags):
            raise ValueError('Waypoint tags must be strings')
        if index and math.hypot(pose[0]-result[-1]['pose'][0], pose[1]-result[-1]['pose'][1]) >= 8.:
            raise ValueError('Every leg must be shorter than 8m; this alone does not prove rolling-window path clearance')
        identifiers.add(identifier)
        result.append(dict(entry, index=index, pose=[float(value) for value in pose], tags=list(tags)))
    return result


def initial_leg_limit(requested, ready, simulation_scan_age):
    """Simulation-only request envelope; server body/path checks stay authoritative.

    Original route legs still require <8 m. A failed preceding target can leave
    the actual start slightly farther away without leaving the 20 m rolling map.
    """
    if isinstance(requested, bool) or requested not in (8., 8.5):
        raise ValueError('Initial leg limit must be 8 or simulation-only 8.5 m')
    if requested == 8.:
        return requested
    metadata = ready.get('actual_costmaps', {}).get('global_costmap', {})
    resolution = metadata.get('resolution')
    sizes = [metadata.get('size_x'), metadata.get('size_y')]
    if (simulation_scan_age != .5 or isinstance(resolution, bool)
            or not isinstance(resolution, (int, float)) or not math.isfinite(resolution)
            or resolution <= 0 or any(type(size) is not int or size <= 0 for size in sizes)
            or any(size * resolution < 20. - 1e-6 for size in sizes)):
        raise ValueError('8.5 m initial request requires explicit simulation profile and >=20m global window')
    return requested


def waypoint_binding(ready, previous=None):
    """Augment the unchanged single-goal lease checks with native-batch identity."""
    contract = ready.get('waypoint_contract', {})
    if ready.get('waypoint_action_server') != ACTION or contract.get('package_version') != '1.3.13' or contract.get('parameters') != WAYPOINT_PARAMETERS:
        raise ValueError('Native waypoint action or effective stop/pause policy mismatch')
    if contract.get('number_of_loops') != 0 or contract.get('goal_index') != 0:
        raise ValueError('Batch must start at index 0 and run once without retry')
    binding = {key: ready.get(key) for key in ['waypoint_action_server', 'waypoint_contract', 'mission_deadline']}
    deadline = ready.get('mission_deadline', {})
    if not all(key in deadline for key in ['run_id', 'manifest_sha256', 'launcher_boot_id', 'deadline_boottime_sec']):
        raise ValueError('Missing fixed launcher deadline identity')
    for key in ['underlay_files_sha256', 'client_sources_sha256']:
        hashes = contract.get(key)
        if not isinstance(hashes, dict) or not hashes or any(
                not isinstance(value, str) or len(value) != 64 or any(c not in '0123456789abcdef' for c in value)
                for value in hashes.values()):
            raise ValueError('Missing native waypoint identity: '+key)
    if previous is not None and previous != binding:
        raise ValueError('Native waypoint runtime binding changed')
    return binding


def mission_budget(ready, seconds, boottime, boot_id, reserve=30.):
    deadline = ready['mission_deadline']
    value = deadline['deadline_boottime_sec']
    if (deadline['launcher_boot_id'] != boot_id or isinstance(value, bool)
            or not isinstance(value, (int, float)) or not math.isfinite(value)):
        raise ValueError('Launcher deadline clock/boot binding mismatch')
    remaining = value-boottime
    if remaining < seconds+reserve:
        raise ValueError('Insufficient fixed recording deadline for preregistered mission plus 30s shutdown reserve')
    return dict(remaining_sec=remaining, mission_budget_sec=seconds, shutdown_reserve_sec=reserve,
                deadline_boottime_sec=value, checked_boottime_sec=boottime)


def exclusive_graph(nodes):
    """Exact action graph roles, including duplicate node names and alien clients."""
    clients, pose_servers, batch_servers = [], [], []
    for node in nodes:
        identity = dict(name=node['name'], namespace=node['namespace'])
        for action, types in node['clients']:
            if action == '/navigate_to_pose' and 'nav2_msgs/action/NavigateToPose' in types:
                clients.append(identity)
        for action, types in node['servers']:
            if action == '/navigate_to_pose' and 'nav2_msgs/action/NavigateToPose' in types:
                pose_servers.append(identity)
            if action == ACTION and 'nav2_msgs/action/FollowWaypoints' in types:
                batch_servers.append(identity)
    follower = [dict(name='waypoint_follower', namespace='/')]
    navigator = [dict(name='bt_navigator', namespace='/')]
    return dict(exclusive_nav_client=clients == follower and pose_servers == navigator and batch_servers == follower,
                navigate_to_pose_clients=clients, navigate_to_pose_servers=pose_servers, follow_waypoints_servers=batch_servers)


def validate_ownership(ownership, now_ns, own_uuid=None, parent_sent_ns=None):
    if ownership.get('exclusive_nav_client') is not True or ownership.get('status_publishers_match') is not True:
        raise ValueError('NavigateToPose endpoint is not exclusively owned by the native follower')
    # This is launcher metadata, governed by the same 500ms lease as read_ready().
    # Sensor/header, TF, scan, command and stop guards retain their 300ms limits.
    if not -.05 <= (now_ns-ownership.get('checked_unix_ns', 0))/1e9 <= .5:
        raise ValueError('Waypoint action ownership observation is stale or future')
    pose = ownership.get('navigate_to_pose_active', [])
    batch = ownership.get('follow_waypoints_active', [])
    if own_uuid is None:
        if pose or batch:
            raise ValueError('An action goal is already active before this batch')
    elif any(item['uuid'] != own_uuid for item in batch) or len(batch) > 1 or len(pose) > 1:
        raise ValueError('Foreign batch or concurrent NavigateToPose goal observed')
    elif parent_sent_ns is not None and any(item.get('accepted_stamp_ns', 0) < parent_sent_ns-50_000_000 for item in pose):
        raise ValueError('Active NavigateToPose predates this batch; child association is unproven')


def ownership_metadata_lease_deviation(ready, now_ns, seen_check_ns):
    """Record each >300ms metadata observation once, apart from physical inputs."""
    ownership = ready.get('waypoint_ownership', {})
    checked_ns = ownership.get('checked_unix_ns', 0)
    age_ns = now_ns-checked_ns
    if age_ns <= 300_000_000 or checked_ns in seen_check_ns:
        return None
    seen_check_ns.add(checked_ns)
    return dict(classification='launcher_ownership_metadata_lease_latency',
                ownership_validation_unix_ns=now_ns, checked_unix_ns=checked_ns,
                ownership_age_ns=age_ns, original_max_age_ns=300_000_000,
                effective_max_age_ns=500_000_000, original_300ms_boundary_exceeded=True,
                within_effective_metadata_lease=age_ns <= 500_000_000,
                physical_input_classification_affected=False, latest_ready=ready)


class OwnedGoalIdentity:
    """A cancellation target can never become zero/all-goals or another UUID."""
    def __init__(self, value):
        self.value = bytes(value)
        if len(self.value) != 16 or not any(self.value):
            raise ValueError('A nonzero 16-byte owned Action UUID is required')

    def verify(self, value):
        if bytes(value) != self.value:
            raise ValueError('Action response UUID differs from the owned batch')


class AsyncEvidenceWriter:
    """Bounded nonblocking callback queue; serialization and I/O stay in one worker.

    Overflow and worker failures are permanent, explicit evidence gaps. They do
    not raise in ROS callbacks or replace the caller's UUID cancellation path.
    """
    def __init__(self, output, capacity=4096, publish_event=None, stream_factory=None):
        self.output = Path(output)
        self.queue = queue.Queue(maxsize=capacity)
        self.publish_event = publish_event
        self.stream_factory = stream_factory or (lambda path: path.open('a', buffering=65536))
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.submitted = self.persisted = self.dropped = 0
        self.first_gap = self.worker_error = None
        self.stream_counts = Counter()
        self.thread = threading.Thread(target=self._work, name='waypoint-evidence-writer', daemon=True)
        self.thread.start()

    def submit(self, stream, record):
        if stream not in ('events', 'scan_trace', 'state_trace'):
            raise ValueError('Unknown evidence stream')
        with self.lock:
            self.submitted += 1
            sequence = self.submitted
            rejected = self.stop.is_set() or self.worker_error is not None
        if not rejected:
            try:
                self.queue.put_nowait((sequence, stream, record))
                return True
            except queue.Full:
                pass
        with self.lock:
            self.dropped += 1
            if self.first_gap is None:
                self.first_gap = dict(sequence=sequence, stream=stream, steady_ns=time.monotonic_ns(),
                                      reason='worker_failed_or_closed' if rejected else 'queue_full')
        return False

    def _work(self):
        streams = {}
        flushed = time.monotonic()
        try:
            while not self.stop.is_set() or not self.queue.empty():
                try:
                    sequence, name, record = self.queue.get(timeout=.1)
                except queue.Empty:
                    continue
                if name not in streams:
                    streams[name] = self.stream_factory(self.output/(name+'.jsonl'))
                line = evidence_json(dict(sequence=sequence, **record))
                streams[name].write(line+'\n')
                if name == 'events' and self.publish_event is not None:
                    self.publish_event(line)
                with self.lock:
                    self.persisted += 1
                    self.stream_counts[name] += 1
                if time.monotonic()-flushed >= .2:
                    for stream in streams.values():
                        stream.flush()
                    flushed = time.monotonic()
        except BaseException as exc:
            with self.lock:
                self.worker_error = dict(type=type(exc).__name__, message=str(exc)[:512])
        finally:
            for stream in streams.values():
                try:
                    stream.close()
                except BaseException as exc:
                    with self.lock:
                        self.worker_error = dict(type=type(exc).__name__, message=str(exc)[:512])

    def close(self, timeout=5.):
        self.stop.set()
        self.thread.join(timeout)
        return self.summary()

    def summary(self):
        with self.lock:
            return dict(capacity=self.queue.maxsize, submitted=self.submitted, persisted=self.persisted,
                        dropped=self.dropped, pending=self.queue.qsize(), first_gap=self.first_gap,
                        worker_error=self.worker_error, writer_alive=self.thread.is_alive(),
                        streams=dict(self.stream_counts),
                        evidence_complete=(self.dropped == 0 and self.worker_error is None
                                           and self.persisted == self.submitted and not self.thread.is_alive()))


class AdvancingStop:
    """Post-zero raw-Odom dwell; rereading one cache entry earns no time."""
    def __init__(self):
        self.reset()

    def reset(self):
        self.zero_stamp = self.first_stamp = self.first_steady = None
        self.last_stamp = self.last_steady = self.last_receive = None

    def observe(self, snapshot, steady_ns, healthy):
        odom = snapshot.get('last', {}).get('odom', {})
        stamp, receive = odom.get('stamp_ns'), odom.get('received_unix_ns')
        stopped = healthy and strict_stopped(snapshot)[0]
        if not stopped or type(stamp) is not int or stamp <= 0:
            self.reset(); return False
        if self.zero_stamp is None:
            self.zero_stamp = self.last_stamp = stamp
            self.last_steady, self.last_receive = steady_ns, receive
            return False
        if stamp == self.last_stamp and receive == self.last_receive:
            return (self.first_stamp is not None and steady_ns-self.last_steady <= 300_000_000
                    and self.last_stamp-self.first_stamp >= 300_000_000
                    and self.last_steady-self.first_steady >= 300_000_000)
        sequence = (stamp > self.last_stamp and stamp-self.last_stamp <= 300_000_000
                    and 0 <= steady_ns-self.last_steady <= 300_000_000)
        self.last_stamp, self.last_steady, self.last_receive = stamp, steady_ns, receive
        if not sequence or stamp <= self.zero_stamp:
            self.first_stamp = self.first_steady = None
            return False
        if self.first_stamp is None:
            self.first_stamp, self.first_steady = stamp, steady_ns
        return stamp-self.first_stamp >= 300_000_000 and steady_ns-self.first_steady >= 300_000_000


class OrderedVisits:
    """Independent original-map checks. Feedback selects a leg; it proves no visit."""
    def __init__(self, route, xy_tolerance=.15, yaw_tolerance=.08, dwell_seconds=.3,
                 start_index=0, accepted_visits=None, advancing_stop=False):
        self.route = route
        self.xy_tolerance, self.yaw_tolerance = xy_tolerance, yaw_tolerance
        self.dwell_ns = int(dwell_seconds*1e9)
        self.active = None
        self.visits = [dict(index=i, id=entry['id'], original_goal_map=entry['pose'], tags=entry['tags'],
                            entered=None, first_within=None, passed=None, last_sample=None,
                            phase='not_entered', motion=dict(samples=0, distance_m=0., signed_yaw_rad=0.,
                            absolute_yaw_rad=0., max_radius_from_entry_m=0., measured_reverse_samples=0,
                            commanded_reverse_samples=0, forward_samples=0, max_forward_bearing_error_rad=0.))
                       for i, entry in enumerate(route)]
        self.violation = None
        self.dwell_start = None
        self.dwell_start_stamp = None
        self.start_index = start_index
        self.advancing_stop = AdvancingStop() if advancing_stop else None
        if type(start_index) is not int or not 0 <= start_index < len(route):
            raise ValueError('Invalid absolute start index')
        if len(accepted_visits or []) != start_index:
            raise ValueError('Resume requires every preceding accepted visit')
        for index, previous in enumerate(accepted_visits or []):
            if (previous.get('index') != index or previous.get('id') != route[index]['id']
                    or previous.get('original_goal_map') != route[index]['pose'] or not previous.get('passed')):
                raise ValueError('Accepted prefix identity differs from route')
            self.visits[index] = previous

    def feedback(self, index, steady_ns, unix_ns):
        if type(index) is not int or not 0 <= index < len(self.route):
            self.violation = 'native_feedback_index_out_of_range'
        elif self.active is None and index != self.start_index:
            self.violation = 'native_feedback_skipped_first_waypoint'
        elif self.active is not None and index != self.active:
            if index != self.active+1 or self.visits[self.active]['passed'] is None:
                self.violation = 'native_feedback_advanced_without_ordered_stopped_visit'
        if self.violation:
            return dict(kind='waypoint_order_violation', index=index, reason=self.violation)
        if index == self.active:
            return None
        self.active = index
        self.dwell_start = self.dwell_start_stamp = None
        if self.advancing_stop is not None:
            self.advancing_stop.reset()
        visit = self.visits[index]
        visit['entered'] = dict(steady_ns=steady_ns, unix_ns=unix_ns, basis='native current_waypoint feedback; child dispatch time unavailable')
        visit['phase'] = 'approach'
        return dict(kind='waypoint_entered', index=index, id=visit['id'], original_goal_map=visit['original_goal_map'],
                    entered=visit['entered'])

    @staticmethod
    def fresh(value, stamped=True):
        return (value.get('receive_age_s', math.inf) <= .3
                and (not stamped or -.05 <= value.get('header_age_s', math.inf) <= .3))

    def observe(self, snapshot, steady_ns, unix_ns, healthy):
        if self.active is None or self.violation:
            return None
        visit = self.visits[self.active]
        values = snapshot['last']
        pose, odom, command = (values.get(key, {}) for key in ['pose', 'odom', '/cmd_vel_safe'])
        goal = visit['original_goal_map']
        pose_ok = (pose.get('frame_id') == 'map' and self.fresh(pose)
                   and all(math.isfinite(pose.get(key, math.nan)) for key in ['x', 'y', 'yaw']))
        if pose_ok and pose.get('stamp_ns') != (visit['last_sample'] or {}).get('stamp_ns'):
            motion = visit['motion']
            previous = visit['last_sample']
            if previous is None:
                visit['entry_pose'] = dict(pose)
            else:
                motion['distance_m'] += math.hypot(pose['x']-previous['x'], pose['y']-previous['y'])
                delta = angular_difference(pose['yaw'], previous['yaw'])
                motion['signed_yaw_rad'] += delta
                motion['absolute_yaw_rad'] += abs(delta)
            origin = visit['entry_pose']
            motion['max_radius_from_entry_m'] = max(motion['max_radius_from_entry_m'], math.hypot(pose['x']-origin['x'], pose['y']-origin['y']))
            motion['samples'] += 1
            if self.fresh(odom):
                motion['measured_reverse_samples'] += int(odom.get('vx', 0.) < -.03)
                if odom.get('vx', 0.) > .03:
                    bearing = math.atan2(goal[1]-pose['y'], goal[0]-pose['x'])
                    motion['forward_samples'] += 1
                    motion['max_forward_bearing_error_rad'] = max(motion['max_forward_bearing_error_rad'], abs(angular_difference(pose['yaw'], bearing)))
            if self.fresh(command, stamped=False):
                motion['commanded_reverse_samples'] += int(command.get('vx', 0.) < -.01)
            visit['last_sample'] = dict(pose)
        within = (pose_ok and math.hypot(pose['x']-goal[0], pose['y']-goal[1]) <= self.xy_tolerance
                  and abs(angular_difference(pose['yaw'], goal[2])) <= self.yaw_tolerance)
        stopped = (self.fresh(odom) and odom.get('frame_id') == 'odom' and odom.get('child_frame_id') == 'base_link'
                   and math.hypot(odom.get('vx', math.inf), odom.get('vy', math.inf)) <= .03
                   and abs(odom.get('wz', math.inf)) <= .05 and self.fresh(command, stamped=False)
                   and math.hypot(command.get('vx', math.inf), command.get('vy', math.inf)) <= .01
                   and abs(command.get('wz', math.inf)) <= .01)
        if within and visit['first_within'] is None:
            visit['first_within'] = dict(steady_ns=steady_ns, unix_ns=unix_ns, pose=dict(pose))
        if visit['passed'] is not None:
            return None
        visit['phase'] = 'stopped_within_tolerance' if within and stopped else 'within_tolerance' if within else 'approach'
        if not (healthy and within and stopped):
            self.dwell_start = self.dwell_start_stamp = None
            if self.advancing_stop is not None:
                self.advancing_stop.reset()
            return None
        advanced = self.advancing_stop.observe(snapshot, steady_ns, healthy) if self.advancing_stop is not None else True
        if self.dwell_start is None:
            self.dwell_start, self.dwell_start_stamp = steady_ns, pose.get('stamp_ns')
        if not advanced or steady_ns-self.dwell_start < self.dwell_ns or pose.get('stamp_ns') == self.dwell_start_stamp:
            return None
        visit['passed'] = dict(steady_ns=steady_ns, unix_ns=unix_ns, dwell_start_steady_ns=self.dwell_start,
                               continuous_stopped_s=(steady_ns-self.dwell_start)/1e9,
                               xy_error_m=math.hypot(pose['x']-goal[0], pose['y']-goal[1]),
                               yaw_error_rad=abs(angular_difference(pose['yaw'], goal[2])), snapshot=snapshot)
        if self.advancing_stop is not None:
            visit['passed']['advancing_odom_dwell'] = dict(first_stamp_ns=self.advancing_stop.first_stamp,
                last_stamp_ns=self.advancing_stop.last_stamp, zero_baseline_stamp_ns=self.advancing_stop.zero_stamp)
        return dict(kind='waypoint_independently_visited', index=self.active, id=visit['id'], passed=visit['passed'])

    def summary(self):
        return dict(all_visited=all(visit['passed'] is not None for visit in self.visits), violation=self.violation,
                    xy_tolerance_m=self.xy_tolerance, yaw_tolerance_rad=self.yaw_tolerance,
                    continuous_stopped_seconds=self.dwell_ns/1e9, visits=self.visits,
                    evidence_boundary='Native feedback only chooses the expected leg. Actual original-map pose and fresh measured/commanded stop prove each ordered visit. Motion counters are observations; rotation sweep and camera/path acceptance require closed-MCAP analysis.')


# Explicit simulation-only scan age profile. No sensor/clock/metadata rewriting.
def simulation_scan_latency_profile(max_age_sec=.3):
    if type(max_age_sec) not in (int,float) or max_age_sec not in (.3,.5):
        raise ValueError('simulation scan max age must be .3 or .5')
    return dict(schema='matrix.simulation_scan_latency.v1',
        mode='strict_300ms' if max_age_sec==.3 else 'simulation_500ms',
        operational_max_age_sec=max_age_sec,strict_max_age_sec=.3,future_tolerance_sec=.05,
        original_header_and_receive_timestamps=True,applies_only_to='scan_header_and_receive_age')


def validate_simulation_scan_latency_profile(ready,max_age_sec=.3):
    expected=simulation_scan_latency_profile(max_age_sec)
    actual=ready.get('simulation_scan_latency_profile')
    # Historical strict fixtures remain valid; every opted-in read is exact-bound.
    if actual is None and max_age_sec==.3:return expected
    if actual!=expected or actual.get('original_header_and_receive_timestamps') is not True:
        raise ValueError('simulation_scan_latency_profile_mismatch')
    return expected


def operational_scan_age_issues(strict_issues,snapshot,max_age_sec=.3):
    simulation_scan_latency_profile(max_age_sec)
    if max_age_sec==.3:return list(strict_issues)
    scan=snapshot.get('last',{}).get('scan',{})
    receive,header=scan.get('receive_age_s'),scan.get('header_age_s')
    fresh=(type(receive) in (int,float) and type(header) in (int,float)
        and math.isfinite(receive) and math.isfinite(header) and 0<=receive<=max_age_sec
        and -.05<=header<=max_age_sec and type(scan.get('stamp_ns')) is int and scan['stamp_ns']>0
        and scan.get('frame_id')=='base_link' and scan.get('ranges',0)>0
        and scan.get('confirmation_payload_valid') is True)
    if not fresh:
        return list(strict_issues)+['scan:simulation_operational_age_or_payload_invalid']
    allowed={'scan:missing_or_stale_receive','scan:invalid_header_age','scan:producer_strict_300ms_age_exceeded'}
    return [issue for issue in strict_issues if issue not in allowed]


def producer_scan_signed_age_bounds(ready, *, clock_ns=None, steady_ns=None):
    obs=ready.get('producer_scan_observation')
    if not isinstance(obs,dict):raise ValueError('scan:producer_observation_missing_or_invalid')
    stamp=obs.get('header_ns');begin=obs.get('validation_clock_begin_ns');end=obs.get('validation_clock_end_ns')
    sb=obs.get('validation_steady_begin_ns');se=obs.get('validation_steady_end_ns');received=obs.get('receive_steady_seconds')
    if (any(type(v) is not int or v<=0 for v in [stamp,begin,end,sb,se]) or begin>end or sb>se
            or type(received) not in (int,float) or not math.isfinite(received) or received<=0
            or obs.get('frame_id')!='base_link' or obs.get('ranges',0)<=0):
        raise ValueError('scan:producer_observation_missing_or_invalid')
    # Optional decision clocks re-age the same original sample without rewriting
    # its header, receipt time or captured producer validation evidence.
    if clock_ns is not None:begin=end=clock_ns
    if steady_ns is not None:sb=se=steady_ns
    return dict(header_age_bounds_s=[(begin-stamp)/1e9,(end-stamp)/1e9],
        receive_age_bounds_s=[sb/1e9-received,se/1e9-received],original_observation=obs)


def strict_simulation_scan_deviations(snapshot,ready,*,producer_bounds=None):
    deviations=[];scan=snapshot.get('last',{}).get('scan',{})
    own={k:scan.get(k) for k in ['stamp_ns','receive_age_s','header_age_s','received_unix_ns','received_steady']}
    if any(type(own.get(k)) in (int,float) and math.isfinite(own[k]) and own[k]>.3
           for k in ['receive_age_s','header_age_s']):
        deviations.append(dict(source='consumer_original_scan',original_scan=own))
    if ready is not None or producer_bounds is not None:
        bounds=producer_scan_signed_age_bounds(ready) if producer_bounds is None else producer_bounds
        if any(age>.3 for key in ['header_age_bounds_s','receive_age_bounds_s'] for age in bounds[key]):
            deviations.append(dict(source='producer_original_scan_bounds',**bounds))
    return deviations


def record_scan_timing_health(pilot, require_stopped, issues, snapshot, entered_unix_ns, entered_steady_ns,
                              producer_evidence_ready=None, producer_evidence_context=None, *, producer_bounds=None):
    """Record original strict evidence before returning operational health.

    Both full health and stop decisions share the same permanent fault/count/
    deduplication rules. Optional bounds describe this decision's original sample.
    """
    strict_issues=list(issues);deviations=[]
    if pilot.simulation_scan_max_age_sec>.3:
        deviations=strict_simulation_scan_deviations(snapshot,producer_evidence_ready,producer_bounds=producer_bounds)
        if any(d['source']=='producer_original_scan_bounds' for d in deviations):
            strict_issues.append('scan:producer_strict_300ms_age_exceeded')
    # The immutable original .3 check and original snapshot feed strict evidence.
    pilot.scan_timing_record_check(require_stopped,strict_issues,snapshot,entered_unix_ns,entered_steady_ns)
    if deviations:
        if pilot.strict_scan_latency_first_fault is None:
            pilot.strict_scan_latency_first_fault=dict(checked_unix_ns=entered_unix_ns,checked_steady_ns=entered_steady_ns,
                original_strict_issues=list(strict_issues),original_snapshot=snapshot,deviations=deviations,
                producer_evidence_context=producer_evidence_context,
                failed_READY_raw_sha256=None if getattr(pilot,'current_ready_read_failure',None) is None else pilot.current_ready_read_failure['metadata']['sha256'],
                latest_validated_READY_sha256=getattr(pilot,'latest_ready_sha',None),strict_timing_status='FAIL')
        pilot.invalidate_artifact(['scan:strict_300ms_latency_violation'])
        for deviation in deviations:
            stamp=(deviation.get('original_scan',{}).get('stamp_ns') if deviation['source']=='consumer_original_scan'
                else deviation['original_observation'].get('header_ns'))
            key=(deviation['source'],stamp)
            if key not in pilot.strict_scan_latency_seen:
                pilot.strict_scan_latency_seen.add(key);pilot.strict_scan_latency_count+=1
                pilot.emit('strict_scan_latency_deviation',strict_timing_status='FAIL',strict_timing_accepted=False,
                    deviation=deviation,original_check_unix_ns=entered_unix_ns)
    operational_issues=operational_scan_age_issues(strict_issues,snapshot,pilot.simulation_scan_max_age_sec)
    return not operational_issues,operational_issues,snapshot

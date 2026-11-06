"""Non-ROS contracts for one ordered native FollowWaypoints experiment."""
from collections import Counter
import json
import math
from pathlib import Path
import queue
import threading
import time


ACTION = '/follow_waypoints'
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
    if not -.05 <= (now_ns-ownership.get('checked_unix_ns', 0))/1e9 <= .3:
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


class OrderedVisits:
    """Independent original-map checks. Feedback selects a leg; it proves no visit."""
    def __init__(self, route, xy_tolerance=.15, yaw_tolerance=.08, dwell_seconds=.3):
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

    def feedback(self, index, steady_ns, unix_ns):
        if type(index) is not int or not 0 <= index < len(self.route):
            self.violation = 'native_feedback_index_out_of_range'
        elif self.active is None and index != 0:
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
            return None
        if self.dwell_start is None:
            self.dwell_start, self.dwell_start_stamp = steady_ns, pose.get('stamp_ns')
        if steady_ns-self.dwell_start < self.dwell_ns or pose.get('stamp_ns') == self.dwell_start_stamp:
            return None
        visit['passed'] = dict(steady_ns=steady_ns, unix_ns=unix_ns, dwell_start_steady_ns=self.dwell_start,
                               continuous_stopped_s=(steady_ns-self.dwell_start)/1e9,
                               xy_error_m=math.hypot(pose['x']-goal[0], pose['y']-goal[1]),
                               yaw_error_rad=abs(angular_difference(pose['yaw'], goal[2])), snapshot=snapshot)
        return dict(kind='waypoint_independently_visited', index=self.active, id=visit['id'], passed=visit['passed'])

    def summary(self):
        return dict(all_visited=all(visit['passed'] is not None for visit in self.visits), violation=self.violation,
                    xy_tolerance_m=self.xy_tolerance, yaw_tolerance_rad=self.yaw_tolerance,
                    continuous_stopped_seconds=self.dwell_ns/1e9, visits=self.visits,
                    evidence_boundary='Native feedback only chooses the expected leg. Actual original-map pose and fresh measured/commanded stop prove each ordered visit. Motion counters are observations; rotation sweep and camera/path acceptance require closed-MCAP analysis.')

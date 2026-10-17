"""Independent wire observer, action client, and optional sensor-only SE2 plant."""
import math
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path as RosPath, OccupancyGrid
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt8, Bool
from rosgraph_msgs.msg import Clock as ClockMessage
from builtin_interfaces.msg import Time as TimeMessage
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from unique_identifier_msgs.msg import UUID
from lifecycle_msgs.srv import GetState
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message

from common import ACTION_NAMES, write_json, cadence, new_navigation_goal_uuid
from world import Scene, integrate, error_to_goal


def stamp(seconds):
    result = TimeMessage()
    result.sec, result.nanosec = divmod(round(seconds * 1e9), 1_000_000_000)
    return result


def pose_into(value, pose):
    value.position.x, value.position.y = float(pose[0]), float(pose[1])
    value.orientation.z, value.orientation.w = math.sin(pose[2] / 2), math.cos(pose[2] / 2)


def yaw(rotation):
    return math.atan2(2 * (rotation.w * rotation.z + rotation.x * rotation.y),
                      1 - 2 * (rotation.y ** 2 + rotation.z ** 2))


class ObservedTopicGoal:
    """A real BT-created goal, adopted from status; never sends an Action goal."""
    def __init__(self, node, goal_uuid):
        self.goal_id = UUID(uuid=list(goal_uuid))
        self.accepted = True
        self.cancel_client = node.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.result_client = node.create_client(NavigateToPose.Impl.GetResultService,
                                                '/navigate_to_pose/_action/get_result')

    def cancel_goal_async(self):
        request = CancelGoal.Request()
        request.goal_info.goal_id = self.goal_id
        return self.cancel_client.call_async(request)

    def get_result_async(self):
        request = NavigateToPose.Impl.GetResultService.Request()
        request.goal_id = self.goal_id
        return self.result_client.call_async(request)


class Monitor(Node):
    def __init__(self, args, plant=False):
        super().__init__('minimal_debug_validation', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.args, self.plant = args, plant
        self.started = time.monotonic()
        self.events, self.latest, self.counts, self.types = [], {}, {}, {}
        self.subscriptions_kept = []
        self.stream = (args.output / 'events.jsonl').open('w', buffering=1)
        self.navigator = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.goal_input = getattr(args, 'goal_input', 'action')
        goal_topic = '/goal_pose' if self.goal_input == 'topic' else '/validation/goal_pose'
        self.goal_pub = self.create_publisher(PoseStamped, goal_topic, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.safe = [0., 0., 0.]
        self.last_safe_ns = 0
        self.pose = list(getattr(args, 'initial', [2., 4., 0.]))
        self.initial = self.pose.copy()
        self.plant_ns = time.monotonic_ns()
        self.sim_time = 1000.
        self.publish_tf, self.publish_scan = True, True
        self.scan_count = 0
        topics = [('/cmd_vel_safe', Twist), ('/cmd_vel_nav', Twist), ('/cmd_vel', Twist),
            ('/odom/current_pose', Odometry), ('/laser_scan', LaserScan), ('/plan', RosPath),
            ('/lightning/loc_status', UInt8), ('/lightning/pose_valid', Bool),
            ('/nav_safety_gate/gate_status', UInt8), ('/map', OccupancyGrid)]
        for topic, cls in topics:
            qos = 100
            if topic == '/laser_scan':
                qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
            if topic == '/map':
                qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.subscriptions_kept.append(self.create_subscription(cls, topic, self.callback(topic), qos))
        for endpoint in ACTION_NAMES:
            topic = '/' + endpoint + '/_action/status'
            qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.subscriptions_kept.append(self.create_subscription(GoalStatusArray, topic, self.callback(topic), qos))
        self.dynamic_topics = set()
        self.discovery = self.create_timer(.2, self.discover,
            clock=Clock(clock_type=ClockType.STEADY_TIME))
        if plant:
            self.scene = Scene(args.map)
            self.clock_pub = self.create_publisher(ClockMessage, '/clock', 10)
            self.odom_pub = self.create_publisher(Odometry, '/odom/current_pose', 10)
            self.scan_pub = self.create_publisher(LaserScan, '/laser_scan', 10)
            self.loc_pub = self.create_publisher(UInt8, '/lightning/loc_status', 10)
            self.valid_pub = self.create_publisher(Bool, '/lightning/pose_valid', 10)
            self.tf_pub = TransformBroadcaster(self)
            self.tick_timer = self.create_timer(.02, self.tick,
                clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.record('monitor_start', dict(plant=plant, pose=self.pose, hardware_io=False))

    def record(self, event, data, observation=None):
        import json
        row = dict(wall_elapsed_sec=time.monotonic() - self.started,
                   monotonic_ns=time.monotonic_ns(), event=event, data=data)
        if observation:
            row['dds_observation'] = observation
        self.events.append(row)
        self.latest[event] = row
        self.counts[event] = self.counts.get(event, 0) + 1
        self.stream.write(json.dumps(row, separators=(',', ':')) + '\n')

    def discover(self):
        for topic, names in self.get_topic_names_and_types():
            if topic not in ('/debug/observations', '/debug/view', '/debug/build_info'):
                continue
            if len(names) != 1:
                raise RuntimeError('Ambiguous debug schema at ' + topic)
            if topic in self.dynamic_topics:
                continue
            cls = get_message(names[0])
            qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL if topic == '/debug/build_info'
                else DurabilityPolicy.VOLATILE)
            self.subscriptions_kept.append(self.create_subscription(cls, topic, self.callback(topic), qos))
            self.dynamic_topics.add(topic)
            self.types[topic] = names[0]
            self.record('schema_discovered', dict(topic=topic, datatype=names[0]))

    def callback(self, topic):
        def receive(message, info):
            if topic == '/cmd_vel_safe':
                now = time.monotonic_ns()
                if self.plant:
                    self.advance(now)
                self.safe = [message.linear.x, message.linear.y, message.angular.z]
                if not all(math.isfinite(value) for value in self.safe):
                    raise AssertionError('Nonfinite safe command')
                self.last_safe_ns = now
                data = dict(velocity=self.safe.copy(), plant_pose=self.pose.copy() if self.plant else None)
            elif topic == '/plan':
                data = dict(frame=message.header.frame_id, poses=len(message.poses),
                    length_m=sum(math.hypot(b.pose.position.x-a.pose.position.x,
                        b.pose.position.y-a.pose.position.y)
                        for a, b in zip(message.poses, message.poses[1:])),
                    end=message_to_ordereddict(message.poses[-1]) if message.poses else None)
            elif topic == '/map':
                data = dict(frame=message.header.frame_id, width=message.info.width, height=message.info.height)
            elif topic == '/laser_scan':
                data = dict(header=message_to_ordereddict(message.header), bins=len(message.ranges))
            else:
                data = message_to_ordereddict(message)
            def metadata(name):
                return info.get(name, 0) if isinstance(info, dict) else getattr(info, name, 0)
            observation = dict(source_timestamp_ns=int(metadata('source_timestamp')),
                               received_timestamp_ns=int(metadata('received_timestamp')),
                               message_info_kind=type(info).__name__)
            self.record(topic, data, observation)
        return receive

    def advance(self, now_ns):
        boundary = max(self.plant_ns, min(now_ns, self.last_safe_ns + 350_000_000))
        if boundary > self.plant_ns and self.last_safe_ns:
            self.pose = integrate(self.pose, self.safe, (boundary - self.plant_ns) / 1e9)
        self.sim_time += (now_ns - self.plant_ns) / 1e9
        self.plant_ns = now_ns

    def tick(self):
        self.advance(time.monotonic_ns())
        source = stamp(self.sim_time)
        clock = ClockMessage()
        clock.clock = source
        self.clock_pub.publish(clock)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = source, 'odom', 'base_link'
        pose_into(odom.pose.pose, self.pose)
        if time.monotonic_ns() - self.last_safe_ns < 350_000_000:
            odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z = self.safe
        self.odom_pub.publish(odom)
        if self.publish_tf:
            transforms = []
            for parent, child, position in [('map', 'odom', [0., 0., 0.]), ('odom', 'base_link', self.pose)]:
                transform = TransformStamped()
                transform.header.stamp, transform.header.frame_id, transform.child_frame_id = source, parent, child
                transform.transform.translation.x, transform.transform.translation.y = float(position[0]), float(position[1])
                transform.transform.rotation.z, transform.transform.rotation.w = math.sin(position[2] / 2), math.cos(position[2] / 2)
                transforms.append(transform)
            self.tf_pub.sendTransform(transforms)
        self.loc_pub.publish(UInt8(data=1))
        self.valid_pub.publish(Bool(data=True))
        self.scan_count += 1
        if self.publish_scan and self.scan_count % 5 == 0:
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = source, 'base_link'
            scan.angle_min, scan.angle_increment = -math.pi, 2 * math.pi / 360
            scan.angle_max = scan.angle_min + 359 * scan.angle_increment
            scan.range_min, scan.range_max, scan.scan_time = .01, 8., .1
            scan.ranges = self.scene.scan(self.pose)
            self.scan_pub.publish(scan)

    def spin_until(self, predicate, seconds, label, processes=None, allow=()):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.02)
            if predicate():
                return
            if processes:
                processes.check(allow=allow)
        raise TimeoutError(label)

    def spin_for(self, seconds, processes=None, allow=()):
        until = time.monotonic() + seconds
        self.spin_until(lambda: time.monotonic() >= until, seconds + 1., 'observation window', processes, allow)

    def watch_raw_sources(self, sources, types):
        """Audit serialized input only; never republish or alter a sensor sample."""
        from replay_support import CdrStreamDigest
        self.raw_sources = {topic: CdrStreamDigest() for topic in sources}
        def callback(name):
            def receive(data):
                self.raw_sources[name].add(data)
            return receive
        for topic in sources:
            qos = QoSProfile(depth=2000, reliability=ReliabilityPolicy.RELIABLE)
            self.subscriptions_kept.append(self.create_subscription(get_message(types[topic]),
                topic, callback(topic), qos, raw=True))

    def raw_source_report(self):
        return {topic: value.snapshot() for topic, value in self.raw_sources.items()}

    def resume_source(self, sources, processes):
        from rosbag2_interfaces.srv import Resume
        client = self.create_client(Resume, '/rosbag2_player/resume')
        self.spin_until(lambda: client.service_is_ready() and all(
            self.get_publishers_info_by_topic(topic) for topic in sources),
            15., 'paused source publisher discovery and resume service', processes)
        future = client.call_async(Resume.Request())
        self.spin_until(future.done, 10., 'resume complete original sensor source', processes)
        if future.exception():
            raise future.exception()
        self.record('full_source_resumed', dict(sim_time_ns=self.get_clock().now().nanoseconds))

    def until_source_time(self, target_ns, rate, processes):
        remaining = max(0., (target_ns - self.get_clock().now().nanoseconds) / 1e9 / rate)
        self.spin_until(lambda: self.get_clock().now().nanoseconds >= target_ns,
                        remaining + 30., 'source clock reaches ' + str(target_ns), processes)

    def sources_ready(self):
        now = time.monotonic_ns()
        for topic in ('/odom/current_pose', '/laser_scan', '/lightning/loc_status', '/lightning/pose_valid', '/map'):
            if topic not in self.latest:
                return False
            if topic != '/map' and now - self.latest[topic]['monotonic_ns'] > 700_000_000:
                return False
        ros_now = self.get_clock().now().nanoseconds / 1e9
        for topic in ('/odom/current_pose', '/laser_scan'):
            source = self.latest[topic]['data']['header']['stamp']
            age = ros_now - source['sec'] - source['nanosec'] / 1e9
            if not 0 <= age < .7:
                return False
        if self.latest['/lightning/loc_status']['data']['data'] != 1:
            return False
        if not self.latest['/lightning/pose_valid']['data']['data']:
            return False
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            source = transform.header.stamp
            age = ros_now - source.sec - source.nanosec / 1e9
            return 0 <= age < .7
        except Exception:
            return False

    def recorder_ready(self, processes):
        """Observe recorder subscriptions while a real clock is already running."""
        goal_topic = '/goal_pose' if self.goal_input == 'topic' else '/validation/goal_pose'
        def ready():
            return self.get_clock().now().nanoseconds > 0 and all(
                any(info.node_name.startswith('rosbag2_recorder')
                    for info in self.get_subscriptions_info_by_topic(topic))
                for topic in ('/clock', goal_topic))
        self.spin_until(ready, 15., 'recorder clock and goal subscriptions', processes)
        self.spin_for(.2, processes)
        self.record('recorder_ready', dict(sim_time_ns=self.get_clock().now().nanoseconds,
            log_uses_sim_time=True, pre_clock_messages_not_recorded=True))

    def readiness(self, processes):
        self.spin_until(self.sources_ready, self.args.ready_timeout,
                        'current localization, odom, scan, map and map->base_link TF', processes)
        names = ('map_server', 'planner_server', 'controller_server', 'smoother_server',
                 'bt_navigator', 'behavior_server', 'velocity_smoother')
        clients = {name: self.create_client(GetState, '/' + name + '/get_state') for name in names}
        pending, active = {}, set()
        def all_ready():
            for name, client in clients.items():
                if name in active:
                    continue
                if name in pending:
                    if pending[name].done():
                        response = pending.pop(name).result()
                        if response and response.current_state.id == 3:
                            active.add(name)
                elif client.service_is_ready():
                    pending[name] = client.call_async(GetState.Request())
            return len(active) == len(names) and self.navigator.server_is_ready() and self.sources_ready()
        self.spin_until(all_ready, self.args.ready_timeout, 'active lifecycle nodes and navigation Action', processes)
        self.record('stack_ready', dict(lifecycle_active=sorted(active)))

    def send_goal(self, goal, processes):
        request = NavigateToPose.Goal()
        request.pose.header.frame_id = 'map'
        request.pose.header.stamp = self.get_clock().now().to_msg()
        pose_into(request.pose.pose, goal)
        if self.goal_input == 'topic':
            self.spin_until(lambda: any(info.node_name == 'bt_navigator'
                for info in self.get_subscriptions_info_by_topic('/goal_pose')),
                10., 'real BT navigator subscription to /goal_pose', processes)
            known = {tuple(item['goal_info']['goal_id']['uuid']) for row in self.events
                     if row['event'] == '/navigate_to_pose/_action/status'
                     for item in row['data']['status_list']}
            boundary = time.monotonic_ns()
            self.goal_pub.publish(request.pose)
            self.record('goal_request', message_to_ordereddict(request.pose))
            self.record('goal_topic_published', dict(topic='/goal_pose', frame_id='map',
                action_send_goal_used=False))
            self.spin_until(lambda: new_navigation_goal_uuid(self.events, known, boundary) is not None,
                10., 'unique NavigateToPose UUID created by BT goal topic callback', processes)
            handle = ObservedTopicGoal(self, new_navigation_goal_uuid(self.events, known, boundary))
            self.spin_until(lambda: handle.result_client.service_is_ready()
                and handle.cancel_client.service_is_ready(), 5., 'observed goal result/cancel services', processes)
        else:
            self.goal_pub.publish(request.pose)
            self.record('goal_request', message_to_ordereddict(request.pose))
            future = self.navigator.send_goal_async(request)
            self.spin_until(future.done, 10., 'NavigateToPose acknowledgement', processes)
            handle = future.result()
        if not handle or not handle.accepted:
            raise RuntimeError('Actual NavigateToPose rejected the validation request')
        self.record('goal_accepted', dict(goal_uuid=[int(value) for value in handle.goal_id.uuid],
            goal=goal, input_transport=self.goal_input))
        return handle, handle.get_result_async()

    def cancel(self, handle, result, processes, allow=()):
        if result.done():
            return result.result().status
        self.record('cancel_requested', dict(goal_uuid=[int(value) for value in handle.goal_id.uuid]))
        future = handle.cancel_goal_async()
        self.spin_until(future.done, 10., 'cancel acknowledgement', processes, allow)
        self.record('cancel_acknowledged', message_to_ordereddict(future.result()))
        self.spin_until(result.done, 15., 'terminal result after controlled cancel', processes, allow)
        return result.result().status

    def stopped_tail(self, seconds, processes, allow=()):
        start = time.monotonic_ns()
        self.spin_for(max(seconds, 1.), processes, allow)
        rows = [row for row in self.events if row['event'] == '/cmd_vel_safe'
                and row['monotonic_ns'] > start + 700_000_000]
        if not rows or any(max(map(abs, row['data']['velocity'])) > 1e-6 for row in rows):
            raise AssertionError('Safe output absent or nonzero after terminal settling window')
        self.record('stopped_tail_verified', dict(samples=len(rows)))

    def lifecycle_shutdown(self, processes):
        """Use the stack's standard shutdown before stopping its clock/processes."""
        client = self.create_client(ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes')
        self.spin_until(client.service_is_ready, 5., 'lifecycle manager shutdown service', processes)
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.SHUTDOWN
        future = client.call_async(request)
        self.spin_until(future.done, 30., 'standard lifecycle shutdown response', processes)
        response = future.result()
        self.record('lifecycle_shutdown_response', dict(success=bool(response and response.success)))
        if not response or not response.success:
            raise AssertionError('Standard lifecycle shutdown returned failure')
        names = ('map_server', 'controller_server', 'smoother_server', 'planner_server',
                 'behavior_server', 'velocity_smoother', 'bt_navigator', 'waypoint_follower')
        pending, final = {}, set()
        clients = {name: self.create_client(GetState, '/' + name + '/get_state') for name in names}
        def finalized():
            for name, node_client in clients.items():
                if name in final:
                    continue
                if name in pending:
                    if pending[name].done():
                        answer = pending.pop(name).result()
                        if answer and answer.current_state.id == 4:
                            final.add(name)
                elif node_client.service_is_ready():
                    pending[name] = node_client.call_async(GetState.Request())
            return len(final) == len(names)
        self.spin_until(finalized, 15., 'all managed lifecycle nodes finalized', processes)
        self.record('lifecycle_finalized', dict(nodes=sorted(final)))

    def report(self):
        observations = [row['data'] for row in self.events if row['event'] == '/debug/observations']
        checkers = [sample for row in observations for sample in row.get('checker', [])]
        branches = [sample for row in observations for sample in row.get('branch', [])]
        status = {name: [item['status'] for row in self.events
            if row['event'] == '/' + name + '/_action/status' for item in row['data']['status_list']]
            for name in ACTION_NAMES}
        nonzero = sum(row['event'] == '/cmd_vel_safe' and
            max(map(abs, row['data']['velocity'])) > 1e-6 for row in self.events if row['event'] == '/cmd_vel_safe')
        result = dict(counts=self.counts, datatypes=self.types, action_statuses=status,
            nonzero_safe_commands=nonzero, checker_samples=checkers, branch_samples=branches,
            algorithm_truth=dict(source='actual StatusObservation records; no checker/TF/costmap algorithm called by observer',
                raw_checker_evaluations=sum(bool(sample.get('evaluated')) for sample in checkers),
                raw_true_checker_returns=sum(bool(sample.get('result_valid')) and bool(sample.get('result')) for sample in checkers),
                actual_plan_messages=self.counts.get('/plan', 0),
                compute_action_observed=bool(status['compute_path_to_pose']),
                smoother_action_observed=bool(status['smooth_path']),
                follow_action_observed=bool(status['follow_path'])))
        result['control_output_cadence'] = {}
        for topic in ('/cmd_vel_nav', '/cmd_vel', '/cmd_vel_safe'):
            rows = [row for row in self.events if row['event'] == topic]
            source = [row.get('dds_observation', {}).get('source_timestamp_ns', 0) for row in rows]
            result['control_output_cadence'][topic] = dict(
                publisher_dds_source=cadence([value for value in source if value > 0]),
                subscriber_callback=cadence([row['monotonic_ns'] for row in rows]),
                missing_dds_source_timestamps=sum(value <= 0 for value in source),
                meaning='command publication/receipt intervals; not algorithm compute duration')
            accepted = [row['monotonic_ns'] for row in self.events if row['event'] == 'goal_accepted']
            terminal = [row['monotonic_ns'] for row in self.events if row['event'] == 'action_terminal']
            active_rows = [row for row in rows if accepted and terminal
                           and accepted[0] <= row['monotonic_ns'] <= terminal[-1]]
            active_source = [row.get('dds_observation', {}).get('source_timestamp_ns', 0) for row in active_rows]
            result['control_output_cadence'][topic]['active_navigation_goal'] = dict(
                publisher_dds_source=cadence([value for value in active_source if value > 0]),
                subscriber_callback=cadence([row['monotonic_ns'] for row in active_rows]),
                selection='callback observed between actual accepted and terminal navigation events; excludes idle/shutdown gap')
        write_json(self.args.output / 'control_timing_report.json', result['control_output_cadence'])
        import re
        metadata = [row['data'] for row in self.events if row['event'] == '/debug/build_info']
        stamps = [('BuildInfo/' + row.get('component', ''), row.get('build', {})) for row in metadata]
        for index, observation in enumerate(observations):
            stamps += [('observation/' + str(index) + '/' + kind, observation.get(kind, {}))
                       for kind in ('source_build', 'reporter_build')]
        errors = []
        for label, build in stamps:
            own = build.get('repository_id') in ('navigation', 'lightning', 'yz_robot_ctrl')
            if own and not build.get('known'):
                errors.append(label + ': own build unknown')
            if build.get('known') and not re.fullmatch(r'[0-9a-f]{40}', build.get('git_commit', '')):
                errors.append(label + ': missing full git commit')
            if own and build.get('known') and (build.get('dirty_state') not in (1, 2)
                or not build.get('artifact_id') or not re.fullmatch(r'[0-9a-f]{64}', build.get('build_id', ''))):
                errors.append(label + ': missing dirty/artifact/build fingerprint')
        result['build_metadata'] = dict(valid=bool(metadata) and not errors,
            metadata_messages=len(metadata), stamps_checked=len(stamps), errors=errors,
            own_repositories=sorted({stamp.get('repository_id') for _, stamp in stamps if stamp.get('known')}),
            native_action_unknown_binding='Allowed; no inference from runtime source HEAD')
        write_json(self.args.output / 'build_metadata_report.json', result['build_metadata'])
        view_errors, reasons, valid_views, terminal_views = [], {}, 0, 0
        for row in self.events:
            if row['event'] != '/debug/view':
                continue
            view = row['data']
            reason = view.get('current_checker_reason', '')
            reasons[reason] = reasons.get(reason, 0) + 1
            following = [state for state in view.get('actions', [])
                         if state['endpoint'].rstrip('/').endswith('/follow_path') and state['value'] == 2]
            if not following:
                terminal_views += 1
            if not view.get('current_checker_valid'):
                continue
            valid_views += 1
            entries = view.get('current_checker', [])
            if len(following) != 1 or len(entries) != 1:
                view_errors.append('valid checker without a unique observed executing FollowPath')
                continue
            item = entries[0]
            if not item['goal_id_valid'] or item['goal_id'] != following[0]['goal_id']:
                view_errors.append('checker UUID does not match observed executing FollowPath')
            samples = item.get('checker', [])
            if len(samples) != 1 or not samples[0]['evaluated'] or not samples[0]['result_valid']:
                view_errors.append('unevaluated checker is marked current')
            revisions = [entry['path_revision'] for entry in view.get('observations', [])
                if entry['source_instance'] == item['source_instance'] and entry['goal_id_valid']
                and entry['goal_id'] == item['goal_id'] and (entry.get('checker') or
                    (entry.get('branch') and entry['branch'][0]['site'] == 'path_update'))]
            if revisions and item['path_revision'] != max(revisions):
                view_errors.append('checker from an older path revision is marked current')
        result['view_contract'] = dict(valid=not view_errors, errors=sorted(set(view_errors)),
            valid_current_checker_views=valid_views, no_executing_follow_views=terminal_views,
            invalidation_reasons=reasons)
        write_json(self.args.output / 'view_contract_report.json', result['view_contract'])
        write_json(self.args.output / 'observations_report.json', result)
        return result

    def finish(self):
        if self.plant:
            self.advance(time.monotonic_ns())
            self.record('plant_finished', dict(pose=self.pose, displacement_m=math.hypot(
                self.pose[0] - self.initial[0], self.pose[1] - self.initial[1])))
        self.stream.close()
        self.destroy_node()

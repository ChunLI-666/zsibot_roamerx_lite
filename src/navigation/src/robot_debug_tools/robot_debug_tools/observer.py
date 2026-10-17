"""Passive native Action/sideband observer. No TF, costmap, algorithms or services."""
import time
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from action_msgs.msg import GoalStatusArray
from robot_debug_msgs.msg import BuildInfo, BuildStamp, DebugView, StatusObservation
from .build import load_build_stamp
from .model import ObservationStore


DEFAULT_ACTIONS = ['/navigate_to_pose', '/navigate_through_poses',
                   '/compute_path_to_pose', '/compute_path_through_poses',
                   '/smooth_path', '/follow_path', '/backup', '/spin',
                   '/wait', '/drive_on_heading']


class DebugObserver(Node):
    def __init__(self):
        super().__init__('debug_observer')
        self.declare_parameter('action_endpoints', DEFAULT_ACTIONS)
        self.model = ObservationStore()
        self.metadata = {}
        self.build = load_build_stamp('robot_debug_tools')
        self.last_ros = None
        self.output = self.create_publisher(DebugView, '/debug/view', 10)
        self.create_subscription(StatusObservation, '/debug/observations', self._observation, 64)
        meta_qos = QoSProfile(depth=64, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(BuildInfo, '/debug/build_info', self._metadata, meta_qos)
        status_qos = QoSProfile(depth=64, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_topics = {}
        self.action_subscriptions = []
        for endpoint in self.get_parameter('action_endpoints').value:
            endpoint = self.resolve_topic_name(endpoint).rstrip('/')
            topic = endpoint + '/_action/status'
            self.status_topics[endpoint] = topic
            self.action_subscriptions.append(self.create_subscription(
                GoalStatusArray, topic,
                lambda msg, info, e=endpoint: self._status(e, msg, info), status_qos))
        self.create_timer(1.0, self._publish,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _metadata(self, msg, info):
        key = (msg.component, msg.source_instance)
        self.metadata[key] = (msg, bytes(info['publisher_gid']) if info.get('publisher_gid') is not None else None, time.monotonic())
        if len(self.metadata) > 128:
            self.metadata.pop(next(iter(self.metadata)))

    @staticmethod
    def _fqn(publisher):
        return '/' + '/'.join((publisher.node_namespace.strip('/'), publisher.node_name)).strip('/')

    def _binding(self, endpoint, publisher_gid):
        try:
            # A unique graph name cannot prove which process emitted a queued
            # sample across a fast restart. Jazzy lacks publisher_gid in MessageInfo;
            # keep native Action source binding unknown rather than guess.
            if publisher_gid is None:
                return BuildStamp(), False
            publishers = self.get_publishers_info_by_topic(self.status_topics[endpoint])
            if publisher_gid is not None:
                publishers = [p for p in publishers
                              if bytes(p.endpoint_gid) == bytes(publisher_gid)]
            if len(publishers) != 1:
                return BuildStamp(), False
            owner = publishers[0]
            fqn = self._fqn(owner)
            if sum('/' + '/'.join((ns.strip('/'), name)).strip('/') == fqn
                   for name, ns in self.get_node_names_and_namespaces()) != 1:
                return BuildStamp(), False
            meta_owners = [p for p in self.get_publishers_info_by_topic('/debug/build_info')
                           if self._fqn(p) == fqn]
            if len(meta_owners) != 1:
                return BuildStamp(), False
            matches = []
            for msg, gid, received in self.metadata.values():
                if msg.node_fqn != fqn:
                    continue
                endpoints = [self.resolve_topic_name(e).rstrip('/') for e in msg.endpoints
                             if not e.startswith(('udp:', 'sdk:'))]
                gid_matches = gid is not None and gid == bytes(meta_owners[0].endpoint_gid)
                if (gid_matches and msg.node_fqn == fqn and endpoint in endpoints
                        and time.monotonic() - received < 6.0):
                    matches.append(msg)
            if len(matches) == 1:
                return matches[0].build, matches[0].build.known
        except Exception:
            pass
        return BuildStamp(), False

    def _clock_check(self):
        now = self.get_clock().now()
        if self.last_ros is not None and now.nanoseconds < self.last_ros:
            self.model.clear()
        self.last_ros = now.nanoseconds
        return now.to_msg()

    def _status(self, endpoint, msg, info):
        observed = self._clock_check()
        build, valid = self._binding(endpoint, info.get('publisher_gid'))
        for status in msg.status_list:
            self.model.action(endpoint, status, observed, build, valid)
        # Empty lists carry no native IDLE or fabricated terminal.
        self._publish()

    def _observation(self, msg):
        self._clock_check()
        self.model.observation(msg)
        if msg.branch or msg.gate or msg.execution:
            self._publish()

    def _publish(self):
        view = DebugView()
        view.header.stamp = self._clock_check()
        view.reporter_build = self.build
        view.actions = list(self.model.actions.values())
        view.observations = list(self.model.observations.values())
        checker, reason = self.model.current_checker()
        view.current_checker_valid = checker is not None
        view.current_checker_reason = reason
        if checker is not None:
            view.current_checker = [checker]
        view.observer_dropped_records = self.model.dropped
        self.output.publish(view)


def main(args=None):
    rclpy.init(args=args)
    node = DebugObserver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

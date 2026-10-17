"""Bounded producer-local handoff; ROS publication happens in an independent worker."""
from collections import deque
import threading
import uuid
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from robot_debug_msgs.msg import BuildInfo


class DiagnosticPublisher:
    def __init__(self, node, component, build, endpoints):
        self.node, self.component, self.build = node, component, build
        self.instance = uuid.uuid4().hex
        if not node.has_parameter('enable_debug_observations'):
            node.declare_parameter('enable_debug_observations', True)
        self.enabled = bool(node.get_parameter('enable_debug_observations').value)
        self._queue, self._lock = deque(), threading.Lock()
        self.dropped = 0
        self._ticks = 0
        self.output = node.create_publisher(
            __import__('robot_debug_msgs.msg', fromlist=['StatusObservation']).StatusObservation,
            '/debug/observations', 64)
        self.build_info_publisher = node.create_publisher(
            BuildInfo, '/debug/build_info', QoSProfile(depth=16,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._sources = {}
        self.register_source(build, self.instance, component, endpoints,
                             node.get_fully_qualified_name())
        self._stop = threading.Event()
        self._worker = threading.Thread(target=self._run, daemon=True,
                                        name='debug-sideband-' + component)
        self._worker.start()
        node.context.on_shutdown(self.close)

    def register_source(self, build, instance, component, endpoints, node_fqn=''):
        info = BuildInfo(build=build, source_instance=instance, component=component,
                         node_fqn=node_fqn, endpoints=list(endpoints))
        self._sources[(component, instance)] = info

    def record(self, observation):
        if not self.enabled:
            return
        if not self._lock.acquire(blocking=False):
            self.dropped += 1
            return
        try:
            if len(self._queue) >= 128:
                self.dropped += 1
                return
            if not observation.source_build.repository_id:
                # Explicit unknown external source must set repository_id to preserve it.
                observation.source_build = self.build
            observation.reporter_build = self.build
            if not observation.source_instance:
                observation.source_instance = self.instance
            if not observation.component:
                observation.component = self.component
            if observation.header.stamp.sec == 0 and observation.header.stamp.nanosec == 0:
                observation.header.stamp = self.node.get_clock().now().to_msg()
            self._queue.append(observation)
        except Exception:
            self.dropped += 1
        finally:
            self._lock.release()

    def _flush(self):
        with self._lock:
            pending, self._queue = self._queue, deque()
        for observation in pending:
            try:
                observation.dropped_records += self.dropped
                self.output.publish(observation)
            except Exception:
                self.dropped += 1
        if self._ticks % 20 == 0:
            for info in list(self._sources.values()):
                try:
                    info.header.stamp = self.node.get_clock().now().to_msg()
                    self.build_info_publisher.publish(info)
                except Exception:
                    self.dropped += 1
        self._ticks += 1

    def _run(self):
        while not self._stop.is_set():
            try:
                self._flush()
            except Exception:
                self.dropped += 1
            self._stop.wait(0.1)

    def close(self):
        self._stop.set()
        if self._worker is not threading.current_thread():
            self._worker.join(timeout=0.5)

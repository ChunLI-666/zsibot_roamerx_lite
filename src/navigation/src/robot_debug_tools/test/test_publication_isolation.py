"""Diagnostic congestion/errors must not enter producer control callbacks."""
import threading
import time
from types import SimpleNamespace
from robot_debug_msgs.msg import BuildStamp, StatusObservation
from robot_debug_tools.publisher import DiagnosticPublisher


class FakeClock:
    def now(self):
        return SimpleNamespace(to_msg=lambda: __import__('builtin_interfaces.msg',
                              fromlist=['Time']).Time(sec=1))


class FakePublisher:
    def __init__(self):
        self.started = threading.Event()
        self.release = threading.Event()
        self.slow = False
        self.fail = False
        self.records = []

    def publish(self, value):
        self.started.set()
        if self.slow:
            assert self.release.wait(3)
        if self.fail:
            raise RuntimeError('diagnostic publish failed')
        self.records.append(value)


class FakeNode:
    def __init__(self, enabled=True):
        self.enabled = enabled
        self.publishers = []
        self.context = SimpleNamespace(on_shutdown=lambda callback: None)

    def has_parameter(self, name):
        return True

    def get_parameter(self, name):
        return SimpleNamespace(value=self.enabled)

    def create_publisher(self, *args):
        result = FakePublisher()
        self.publishers.append(result)
        return result

    def get_fully_qualified_name(self):
        return '/test'

    def get_clock(self):
        return FakeClock()


def test_slow_publication_does_not_block_producer_and_queue_is_bounded():
    node = FakeNode()
    publisher = DiagnosticPublisher(node, 'test', BuildStamp(), [])
    publisher.output.slow = True
    try:
        publisher.record(StatusObservation())
        assert publisher.output.started.wait(1)
        start = time.monotonic()
        for _ in range(1000):
            publisher.record(StatusObservation())
        assert time.monotonic() - start < 0.5
        assert len(publisher._queue) <= 128
        assert publisher.dropped > 0
    finally:
        publisher.output.release.set()
        publisher.close()


def test_publish_errors_stay_in_diagnostic_worker():
    publisher = DiagnosticPublisher(FakeNode(), 'test', BuildStamp(), [])
    try:
        publisher.output.fail = True
        publisher.record(StatusObservation())
        assert publisher.output.started.wait(1)
        deadline = time.monotonic() + 1
        while publisher.dropped == 0 and time.monotonic() < deadline:
            time.sleep(0.005)
        assert publisher.dropped > 0
        assert publisher._worker.is_alive()
    finally:
        publisher.close()


def test_disabled_sampling_preserves_build_metadata():
    node = FakeNode(enabled=False)
    publisher = DiagnosticPublisher(node, 'test', BuildStamp(), [])
    try:
        publisher.record(StatusObservation())
        assert not publisher._queue
        assert node.publishers[1].started.wait(1)
        assert not publisher.output.records
    finally:
        publisher.close()

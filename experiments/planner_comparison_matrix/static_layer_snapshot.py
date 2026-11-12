"""Capture frozen-map diagnostic layers once without continuously requesting them.

The primary map and master costmaps remain continuously recorded. Each diagnostic
snapshot preserves its original serialized message, including its sampling stamp.
"""
import hashlib
import json
import time


EXCLUDED_LAYERS = r'^/(global|local)_costmap/static_layer(_raw)?$'


def audit_recorded(bag, snapshots):
    import rosbag2_py
    expected = {item['recorded_topic']: item['cdr_sha256'] for item in snapshots}
    observed = {topic: [] for topic in expected}
    if not expected:
        return dict(passed=False, reason='No static snapshots captured before startup ended', topics={})
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(expected)))
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        observed[topic].append(hashlib.sha256(raw).hexdigest())
    rows = {topic: dict(count=len(values), expected_sha256=expected[topic],
                       recorded_sha256=values, passed=values == [expected[topic]])
            for topic, values in observed.items()}
    return dict(passed=all(row['passed'] for row in rows.values()), topics=rows)


def capture(node, output, recorder_name, pump, stopped, timeout=15.):
    from nav_msgs.msg import OccupancyGrid
    from nav2_msgs.msg import Costmap
    from rclpy.duration import Duration
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    snapshots = []
    for namespace in ['global_costmap', 'local_costmap']:
        for suffix, message_type in [('static_layer', OccupancyGrid),
                                     ('static_layer_raw', Costmap)]:
            source = '/' + namespace + '/' + suffix
            topic = '/experiment/static_layer_snapshot' + source
            received = []

            def accept(raw):
                if not received:
                    received.append(bytes(raw))

            subscription = node.create_subscription(message_type, source, accept, qos, raw=True)
            try:
                deadline = time.monotonic() + timeout
                while not received and time.monotonic() < deadline and not stopped():
                    pump(.02)
                if not received:
                    raise RuntimeError('No original static-layer snapshot: ' + source)
            finally:
                node.destroy_subscription(subscription)

            raw = received[0]
            filename = namespace + '_' + suffix + '.cdr'
            (output / filename).write_bytes(raw)
            publisher = node.create_publisher(message_type, topic, qos)
            try:
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline and not stopped():
                    names = [info.node_name for info in node.get_subscriptions_info_by_topic(topic)]
                    if recorder_name in names:
                        break
                    pump(.02)
                else:
                    raise RuntimeError('Recorder did not subscribe to snapshot: ' + topic)
                if stopped():
                    raise RuntimeError('Stopped before publishing snapshot')
                publisher.publish(raw)
                if not publisher.wait_for_all_acked(Duration(seconds=timeout)):
                    raise RuntimeError('Snapshot acknowledgement timed out: ' + topic)
            finally:
                node.destroy_publisher(publisher)
            snapshots.append(dict(source_topic=source, recorded_topic=topic, file=filename,
                                  cdr_sha256=hashlib.sha256(raw).hexdigest(), bytes=len(raw)))
    (output / 'static_layer_snapshots.json').write_text(json.dumps(dict(
        policy='Frozen map only; recreate the run when loading another map. Master costmaps and map remain fully recorded.',
        excluded_continuous_topics_regex=EXCLUDED_LAYERS, snapshots=snapshots), indent=2) + '\n')
    return snapshots

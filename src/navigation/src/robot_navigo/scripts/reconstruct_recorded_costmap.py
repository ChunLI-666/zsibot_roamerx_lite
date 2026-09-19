#!/usr/bin/env python3
"""Read-only reconstruction of recorded full costmaps plus rectangular updates.

Outputs a frozen historical context, not a simulation of future perception.
ROS imports are deferred so numeric reconstruction can be tested separately.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import yaml


def occupancy_to_cost(values):
    """Conservative inverse of Nav2's lossy costmap publication translation."""
    values = np.asarray(values)
    if not np.isfinite(values).all() or np.any(values != np.floor(values)):
        raise ValueError('Occupancy values must be finite integers')
    if np.any((values < -1) | (values > 100)):
        raise ValueError('Occupancy value outside [-1, 100]')
    table = np.zeros(102, dtype=np.uint8)  # index = occupancy + 1
    table[0] = 255
    for cost in range(1, 253):
        occupancy = 1 + (97 * (cost - 1)) // 251
        table[occupancy + 1] = cost  # largest preimage; never turns inflation lethal
    table[100], table[101] = 253, 254
    return table[values.astype(np.int64) + 1]


class RecordedGrid:
    def __init__(self):
        self.meta = None
        self.cells = None
        self.full_time_ns = None
        self.last_time_ns = None
        self.source_stamp_ns = None
        self.latest_stamped_source_ns = 0
        self.applied_updates = 0
        self.rejected_updates = []
        self.unstamped_updates = 0

    def full(self, meta, values, source_stamp_ns, received_ns):
        width, height = int(meta['width']), int(meta['height'])
        if width <= 0 or height <= 0 or not meta['frame']:
            raise ValueError('Invalid grid dimensions/frame')
        if not all(math.isfinite(meta[key]) for key in ('resolution', 'origin_x', 'origin_y')):
            raise ValueError('Invalid grid geometry')
        if meta['resolution'] <= 0 or len(values) != width * height:
            raise ValueError('Invalid resolution or data size')
        occupancy_to_cost(values)  # validate before accepting any state
        self.meta = dict(meta)
        self.cells = np.asarray(values, dtype=np.int16).reshape(height, width).copy()
        self.full_time_ns = received_ns
        self.last_time_ns = received_ns
        self.source_stamp_ns = source_stamp_ns
        self.latest_stamped_source_ns = source_stamp_ns
        self.applied_updates = 0
        self.rejected_updates = []
        self.unstamped_updates = 0

    def update(self, frame, x, y, width, height, values, source_stamp_ns, received_ns):
        reason = None
        if self.cells is None:
            reason = 'update_without_full_grid'
        elif frame != self.meta['frame']:
            reason = 'frame_mismatch'
        elif source_stamp_ns > 0 and source_stamp_ns < self.latest_stamped_source_ns:
            reason = 'source_time_regression'
        elif width <= 0 or height <= 0 or x < 0 or y < 0 or (
                x + width > self.meta['width'] or y + height > self.meta['height']):
            reason = 'update_out_of_bounds'
        elif len(values) != width * height:
            reason = 'invalid_update_size'
        if reason:
            self.rejected_updates.append(dict(reason=reason, received_ns=received_ns))
            return False
        occupancy_to_cost(values)
        self.cells[y:y + height, x:x + width] = np.asarray(values).reshape(height, width)
        self.source_stamp_ns = source_stamp_ns
        if source_stamp_ns > 0:
            self.latest_stamped_source_ns = source_stamp_ns
        if source_stamp_ns == 0:
            # This repository's Costmap2DPublisher deliberately writes Time()
            # in update headers. Receipt order is the available ordering evidence.
            self.unstamped_updates += 1
        self.last_time_ns = received_ns
        self.applied_updates += 1
        return True

    def write(self, output, name, target_ns, bag):
        if self.cells is None or self.last_time_ns > target_ns:
            raise ValueError('No prior full grid for snapshot')
        output.mkdir(parents=True, exist_ok=True)
        raw = occupancy_to_cost(self.cells).tobytes(order='C')
        path = output / (name + '.costmap')
        path.write_bytes(raw)
        spec = {key: self.meta[key] for key in
                ('width', 'height', 'resolution', 'origin_x', 'origin_y')}
        spec['data'] = str(path.resolve())
        result = dict(
            name=name, bag=str(bag), target_receipt_time_ns=target_ns,
            frame=self.meta['frame'], map=spec, sha256=hashlib.sha256(raw).hexdigest(),
            full_receipt_time_ns=self.full_time_ns, last_receipt_time_ns=self.last_time_ns,
            last_source_time_ns=self.source_stamp_ns,
            last_receipt_age_sec=(target_ns - self.last_time_ns) / 1e9,
            full_age_sec=(target_ns - self.full_time_ns) / 1e9,
            applied_updates=self.applied_updates,
            unstamped_updates=self.unstamped_updates,
            rejected_updates=[dict(item) for item in self.rejected_updates],
            cost_counts={str(int(cost)): int(count) for cost, count in
                         zip(*np.unique(np.frombuffer(raw, dtype=np.uint8), return_counts=True))},
            limitations=[
                'Frozen recorded costmap, not future obstacle perception or a complete navigation replay',
                'Recorded inflation and self-clearing used the historical footprint',
                'OccupancyGrid loses cost precision; inverse uses maximum cost within each published bucket',
                'Finite rolling window; exiting it must be treated as unknown/out-of-bounds',
                'Updates have no map generation ID; zero-stamped updates use bag receipt order',
                'Known nonzero regressing update source times are rejected; frame and bounds must match'])
        (output / (name + '.json')).write_text(json.dumps(result, indent=2))
        (output / (name + '.yaml')).write_text(yaml.safe_dump(result))
        return result


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--snapshot', action='append', required=True,
                        help='name:bag_receipt_time_nanoseconds')
    parser.add_argument('--topic', default='/local_costmap/costmap')
    args = parser.parse_args()
    targets = sorted((int(value.rsplit(':', 1)[1]), value.rsplit(':', 1)[0])
                     for value in args.snapshot)
    if len({name for _, name in targets}) != len(targets) or any(
            not name or Path(name).name != name or name in ('.', '..') for _, name in targets):
        parser.error('Snapshot names must be unique file names')
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import OccupancyGrid
    from map_msgs.msg import OccupancyGridUpdate

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[args.topic, args.topic + '_updates']))
    state = RecordedGrid()
    index = 0
    results = []
    # Unindexed MCAP may return storage order. Sort the selected messages by
    # receipt time before choosing snapshots; never consume a future update.
    records = []
    while reader.has_next():
        topic, serialized, received = reader.read_next()
        if received <= targets[-1][0]:
            records.append((received, topic, serialized))
    records.sort(key=lambda record: record[0])
    source_hash = hashlib.sha256()
    for received, topic, serialized in records:
        topic_bytes = topic.encode('utf-8')
        source_hash.update(received.to_bytes(8, 'little', signed=True))
        source_hash.update(len(topic_bytes).to_bytes(4, 'little'))
        source_hash.update(topic_bytes)
        source_hash.update(len(serialized).to_bytes(8, 'little'))
        source_hash.update(serialized)
    for received, topic, serialized in records:
        while index < len(targets) and targets[index][0] < received:
            target, name = targets[index]
            results.append(state.write(args.output, name, target, args.bag))
            index += 1
        if index == len(targets):
            break
        if topic == args.topic:
            message = deserialize_message(serialized, OccupancyGrid)
            q = message.info.origin.orientation
            # Harness uses axis-aligned maps; reject unsupported origins.
            if not all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w)) or (
                    abs(q.x) + abs(q.y) + abs(q.z) > 1e-6 or abs(abs(q.w) - 1) > 1e-6):
                raise ValueError('Rotated/invalid occupancy-grid origin is unsupported')
            state.full(dict(width=message.info.width, height=message.info.height,
                            resolution=message.info.resolution,
                            origin_x=message.info.origin.position.x,
                            origin_y=message.info.origin.position.y, frame=message.header.frame_id),
                       message.data, stamp_ns(message.header.stamp), received)
        else:
            message = deserialize_message(serialized, OccupancyGridUpdate)
            state.update(message.header.frame_id, message.x, message.y,
                         message.width, message.height, message.data,
                         stamp_ns(message.header.stamp), received)
    while index < len(targets):
        target, name = targets[index]
        results.append(state.write(args.output, name, target, args.bag))
        index += 1
    for result in results:
        result['input_provenance'] = dict(
            selected_stream_sha256=source_hash.hexdigest(), selected_message_count=len(records),
            selected_through_receipt_ns=targets[-1][0], topics=[args.topic, args.topic + '_updates'],
            reconstruction_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
        (args.output / (result['name'] + '.json')).write_text(json.dumps(result, indent=2))
        (args.output / (result['name'] + '.yaml')).write_text(yaml.safe_dump(result))
    (args.output / 'manifest.json').write_text(json.dumps(results, indent=2))
    for result in results:
        print(result['name'], result['frame'], result['map']['width'], result['map']['height'],
              'age_sec', result['last_receipt_age_sec'], 'updates', result['applied_updates'],
              'rejected', len(result['rejected_updates']))


if __name__ == '__main__':
    main()

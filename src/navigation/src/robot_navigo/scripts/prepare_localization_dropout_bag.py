#!/usr/bin/env python3
"""Copy a sensor-only bag prefix and optionally omit a LiDAR receipt-time interval.

Original CDR payloads (including every message header/point time) and receipt
timestamps are preserved byte for byte. Never writes to the input bag.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path


def stream_digest(records):
    digest = hashlib.sha256()
    for stamp, topic, payload in records:
        name = topic.encode('utf8')
        digest.update(stamp.to_bytes(8, 'little', signed=True))
        digest.update(len(name).to_bytes(4, 'little'))
        digest.update(name)
        digest.update(len(payload).to_bytes(8, 'little'))
        digest.update(payload)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=45.)
    parser.add_argument('--drop-start', type=float, default=8.)
    parser.add_argument('--drop-duration', type=float, default=4.5)
    parser.add_argument('--lidar-topic', default='/livox/lidar')
    parser.add_argument('--imu-topic', default='/livox/imu')
    args = parser.parse_args()
    if args.output.resolve().is_relative_to(args.input.resolve()):
        parser.error('Output must be outside the source bag directory')
    if args.output.exists():
        parser.error('Output must be a new directory; existing evidence is never overwritten')
    if not all(math.isfinite(value) and value >= 0 for value in
               (args.duration, args.drop_start, args.drop_duration)) or args.duration <= 0:
        parser.error('Durations must be finite and nonnegative, prefix duration positive')
    if args.drop_start + args.drop_duration > args.duration:
        parser.error('Drop interval must be inside the prefix')
    import rosbag2_py
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.input), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    wanted = {args.lidar_topic, args.imu_topic}
    metadata = {topic.name: topic for topic in reader.get_all_topics_and_types() if topic.name in wanted}
    if set(metadata) != wanted:
        parser.error('Both configured sensor topics must be present')
    reader.set_filter(rosbag2_py.StorageFilter(topics=sorted(wanted)))
    records = []
    # Read the filtered stream before sorting: unindexed MCAP need not provide
    # receive-time order. Retaining raw CDR avoids reserialization differences.
    while reader.has_next():
        topic, payload, received = reader.read_next()
        records.append((received, topic, payload))
    records.sort(key=lambda item: item[0])
    if not records:
        parser.error('Empty sensor stream')
    start = records[0][0]
    end = start + round(args.duration * 1e9)
    prefix = [record for record in records if record[0] <= end]
    drop_start = start + round(args.drop_start * 1e9)
    drop_end = drop_start + round(args.drop_duration * 1e9)
    kept, dropped = [], []
    for record in prefix:
        destination = dropped if record[1] == args.lidar_topic and drop_start <= record[0] < drop_end else kept
        destination.append(record)
    if args.drop_duration and not dropped:
        parser.error('Requested injection removed no LiDAR messages')
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(args.output), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    for topic in metadata.values():
        writer.create_topic(topic)
    for received, topic, payload in kept:
        writer.write(topic, payload, received)
    del writer  # Flush MCAP index and rosbag metadata before readback.
    check = rosbag2_py.SequentialReader()
    check.open(rosbag2_py.StorageOptions(uri=str(args.output), storage_id='mcap'),
               rosbag2_py.ConverterOptions('', ''))
    readback = []
    while check.has_next():
        topic, payload, received = check.read_next()
        readback.append((received, topic, payload))
    readback.sort(key=lambda item: item[0])
    if len(readback) != len(kept) or stream_digest(readback) != stream_digest(kept):
        raise RuntimeError('Readback differs from selected original CDR messages/timestamps')
    manifest = dict(source_bag=str(args.input.resolve()), output_bag=str(args.output.resolve()),
                    prefix_duration_sec=args.duration, first_receipt_ns=start,
                    last_selected_receipt_ns=prefix[-1][0],
                    actual_selected_duration_sec=(prefix[-1][0] - start) / 1e9,
                    configured_topics=dict(lidar=args.lidar_topic, imu=args.imu_topic),
                    drop_receipt_interval_ns=[drop_start, drop_end],
                    source_prefix_counts=dict(Counter(row[1] for row in prefix)),
                    kept_counts=dict(Counter(row[1] for row in kept)),
                    dropped_counts=dict(Counter(row[1] for row in dropped)),
                    source_prefix_stream_sha256=stream_digest(prefix),
                    kept_stream_sha256=stream_digest(kept), readback_identical=True,
                    script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                    limitations=['Injection is defined in bag receipt time, not rewritten sensor time',
                                 'Only IMU and LiDAR are copied; no reference pose, TF, commands or goals are replayed'])
    (args.output / 'dropout_manifest.json').write_text(json.dumps(manifest, indent=2))
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()

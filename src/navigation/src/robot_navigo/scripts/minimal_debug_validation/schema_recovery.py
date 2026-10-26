"""Lossless Action-schema companion, following recover_action_channels.py.

The original recorder output is never rewritten. All copied CDR, log/publish
times and sequence numbers are compared after writing; native ROS classes
decode every recovered record. Definitions come from the active ament prefix.
"""
from collections import Counter, defaultdict
from dataclasses import dataclass
import importlib
import json
import os
from pathlib import Path
import re
import struct

from common import mcap_summary_schemas, sha256, write_json, message_stamp_ns, timestamp_domain_valid


MAGIC = b'\x89MCAP0\r\n'
GENERATOR_SOURCE_PATH = Path(__file__).resolve()
GENERATOR_SOURCE_SHA256 = sha256(GENERATOR_SOURCE_PATH)


def u16(value):
    return struct.pack('<H', value)


def u32(value):
    return struct.pack('<I', value)


def u64(value):
    return struct.pack('<Q', value)


def string(value):
    data = value.encode()
    return u32(len(data)) + data


def record(opcode, data):
    return struct.pack('<BQ', opcode, len(data)) + data


def read_string(data, offset):
    size = struct.unpack_from('<I', data, offset)[0]
    return data[offset + 4:offset + 4 + size].decode(), offset + 4 + size


@dataclass(frozen=True)
class Message:
    topic: str
    data: bytes
    log: int
    publish: int
    sequence: int


def messages(path, selected):
    _, channels = mcap_summary_schemas(path)
    ids = {identity for identity, item in channels.items() if item['topic'] in selected}
    def unpack(payload):
        at = 0
        while at + 9 <= len(payload):
            opcode, size = struct.unpack_from('<BQ', payload, at)
            start, end = at + 9, at + 9 + size
            if end > len(payload):
                raise ValueError('Truncated MCAP chunk record')
            if opcode == 5:
                identity, sequence, log, publish = struct.unpack_from('<HIQQ', payload, start)
                if identity in ids:
                    yield Message(channels[identity]['topic'], payload[start + 22:end], log, publish, sequence)
            at = end
    with Path(path).open('rb') as stream:
        assert stream.read(8) == MAGIC
        total = Path(path).stat().st_size
        while stream.tell() + 9 <= total - 8:
            header = stream.read(9)
            opcode, size = struct.unpack('<BQ', header)
            if opcode not in (5, 6):
                stream.seek(size, 1)
                continue
            body = stream.read(size)
            if opcode == 5:
                yield from unpack(header + body)
                continue
            compression, offset = read_string(body, 28)
            payload_size = struct.unpack_from('<Q', body, offset)[0]
            if compression:
                raise ValueError('Schema recovery expects the runner uncompressed MCAP preset: ' + compression)
            yield from unpack(body[offset + 8:offset + 8 + payload_size])


class Definitions:
    def __init__(self):
        self.sources, self.bodies = set(), {}
        self.class_checks = {}

    def body(self, name):
        from ament_index_python.packages import get_package_share_directory
        if name in self.bodies:
            return self.bodies[name]
        package, kind = name.split('/')
        share = Path(get_package_share_directory(package))
        if kind.endswith('_FeedbackMessage'):
            action = kind[:-len('_FeedbackMessage')]
            body = 'unique_identifier_msgs/UUID goal_id\n' + package + '/' + action + '_Feedback feedback\n'
        elif kind.endswith('_Feedback'):
            action = kind[:-len('_Feedback')]
            path = share / 'action' / (action + '.action')
            self.sources.add(path)
            idl = path.with_suffix('.idl')
            self.sources.add(idl)
            body = path.read_text().split('---')[2]
            empty_feedback = not any(line.split('#', 1)[0].strip() for line in body.splitlines())
            if empty_feedback:
                # Verify the actual generated representation of empty feedback.
                if 'structure_needs_at_least_one_member' not in idl.read_text():
                    raise ValueError('Unknown empty-feedback IDL representation: ' + str(idl))
                body = 'uint8 structure_needs_at_least_one_member\n'
            fields = {}
            for line in body.splitlines():
                tokens = line.split('#', 1)[0].strip().split()
                if len(tokens) >= 2 and '=' not in tokens[1]:
                    fields[tokens[1]] = {'float32': 'float', 'float64': 'double'}.get(tokens[0], tokens[0])
            generated = getattr(importlib.import_module(package + '.action'), action).Feedback.get_fields_and_field_types()
            expected = {} if empty_feedback else fields
            if list(expected.items()) != list(generated.items()):
                raise ValueError('Installed .action feedback differs from generated class: ' + name)
            self.class_checks[name] = dict(generated_fields=generated, empty_idl_dummy=empty_feedback)
        else:
            path = share / 'msg' / (kind + '.msg')
            self.sources.add(path)
            body = path.read_text()
        self.bodies[name] = body
        return body

    def full(self, type_name):
        package, _, kind = type_name.split('/')
        root = package + '/' + kind
        dependencies = {}
        def visit(name):
            body = self.body(name)
            for line in body.splitlines():
                line = line.split('#', 1)[0].strip()
                if not line:
                    continue
                token = re.split(r'\[|<=', line.split()[0])[0]
                if '/' in token:
                    dependency = token
                elif token == 'Header':
                    dependency = 'std_msgs/Header'
                elif token[0].isupper():
                    dependency = name.split('/')[0] + '/' + token
                else:
                    continue
                if dependency != root and dependency not in dependencies:
                    dependencies[dependency] = self.body(dependency)
                    visit(dependency)
            return body
        body = visit(root)
        return body.rstrip() + '\n' + ''.join('\n' + '=' * 80 + '\nMSG: ' + name + '\n' + value.rstrip() + '\n'
            for name, value in dependencies.items())


def write_companion(path, rows, types, definitions, library='minimal-debug lossless Action schemas'):
    names, topics = sorted(set(types.values())), sorted(types)
    sid, cid = {name: i + 1 for i, name in enumerate(names)}, {topic: i + 1 for i, topic in enumerate(topics)}
    schemas = [record(3, u16(sid[name]) + string(name) + string('ros2msg') + string(definitions.full(name))) for name in names]
    channels = [record(4, u16(cid[topic]) + u16(sid[types[topic]]) + string(topic) + string('cdr') + u32(0)) for topic in topics]
    rows = sorted(rows, key=lambda row: (row.log, row.topic, row.sequence))
    payload, indexes = bytearray(), defaultdict(list)
    for row in rows:
        indexes[cid[row.topic]].append((row.log, len(payload)))
        payload += record(5, u16(cid[row.topic]) + u32(row.sequence) + u64(row.log) + u64(row.publish) + row.data)
    start, end = min(row.log for row in rows), max(row.log for row in rows)
    with Path(path).open('wb') as stream:
        stream.write(MAGIC + record(1, string('ros2') + string(library)))
        for value in schemas + channels:
            stream.write(value)
        chunk_position = stream.tell()
        chunk = record(6, u64(start) + u64(end) + u64(len(payload)) + u32(0) + string('') + u64(len(payload)) + payload)
        stream.write(chunk)
        index_start, offsets = stream.tell(), {}
        for identity, entries in sorted(indexes.items()):
            offsets[identity] = stream.tell()
            stream.write(record(7, u16(identity) + u32(len(entries) * 16) + b''.join(u64(t) + u64(o) for t, o in entries)))
        index_length = stream.tell() - index_start
        stream.write(record(15, u32(0)))
        summary_start = stream.tell()
        for value in schemas + channels:
            stream.write(value)
        counts = Counter(row.topic for row in rows)
        stats = u64(len(rows)) + u16(len(schemas)) + u32(len(channels)) + u32(0) + u32(0) + u32(1) + u64(start) + u64(end)
        stats += u32(len(channels) * 10) + b''.join(u16(cid[topic]) + u64(counts[topic]) for topic in topics)
        stream.write(record(11, stats))
        index = u64(start) + u64(end) + u64(chunk_position) + u64(len(chunk)) + u32(len(offsets) * 10)
        index += b''.join(u16(identity) + u64(offset) for identity, offset in sorted(offsets.items()))
        index += u64(index_length) + string('') + u64(len(payload)) + u64(len(payload))
        stream.write(record(8, index))
        stream.write(record(2, u64(summary_start) + u64(0) + u32(0)) + MAGIC)


def native_class(name):
    if '/action/' in name:
        package, _, kind = name.split('/')
        return getattr(importlib.import_module(package + '.action'),
                       kind[:-len('_FeedbackMessage')]).Impl.FeedbackMessage
    from rosidl_runtime_py.utilities import get_message
    return get_message(name)


class UnsupportedHeaderRepresentation(ValueError):
    """The native decoder must handle this representation instead."""


def cdr_header_stamps(data):
    """Read a leading ROS Header without constructing a LiDAR point array."""
    if len(data) < 16:
        raise ValueError('Truncated CDR Header')
    if data[0] != 0 or data[1] not in (0, 1):
        raise UnsupportedHeaderRepresentation('Unsupported CDR Header endianness/representation')
    endian = '<' if data[1] == 1 else '>'
    seconds, nanoseconds, frame_size = struct.unpack_from(endian + 'iII', data, 4)
    if nanoseconds >= 1_000_000_000:
        raise ValueError('Invalid CDR Header timestamp nanoseconds')
    if frame_size < 1 or 16 + frame_size > len(data):
        raise ValueError('Truncated CDR Header frame_id')
    frame = data[16:16 + frame_size]
    if frame[-1] != 0:
        raise ValueError('Unterminated CDR Header frame_id')
    bytes(frame[:-1]).decode('utf-8')
    return [seconds * 1_000_000_000 + nanoseconds] if seconds or nanoseconds else []


def delivery_message_stamps(data, type_name, cls, header_validated_types, deserialize):
    """Keep full native decoding except for a validated CustomMsg Header path."""
    header_type = 'livox_ros_driver2/msg/CustomMsg'
    if type_name == header_type and type_name in header_validated_types:
        try:
            return None, cdr_header_stamps(data), True
        except UnsupportedHeaderRepresentation:
            pass  # Preserve native support for representations outside this fast path.
    decoded = deserialize(data, cls)
    stamps = message_stamp_ns(decoded)
    if type_name == header_type:
        if not 0 <= decoded.header.stamp.nanosec < 1_000_000_000:
            raise ValueError('Invalid native CustomMsg Header timestamp nanoseconds')
        try:
            serialized_stamps = cdr_header_stamps(data)
        except UnsupportedHeaderRepresentation:
            return decoded, stamps, False
        if stamps != serialized_stamps:
            raise AssertionError('CustomMsg native and serialized Header stamps differ')
        header_validated_types.add(type_name)
    return decoded, stamps, False


def self_contained(output):
    """Create a complete single-file viewing ROSbag; leave recorder originals intact."""
    import copy
    import yaml
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    output = Path(output)
    types, raw_definitions, rows, sources = {}, {}, [], []
    generated = Definitions()
    for source in sorted((output / 'bag').glob('*.mcap')):
        schemas, channels = mcap_summary_schemas(source, include_data=True)
        selected = {}
        for channel in channels.values():
            schema = schemas[channel['schema_id']]
            topic, name = channel['topic'], schema['name']
            if topic in types and types[topic] != name:
                raise ValueError('Conflicting type definitions for ' + topic)
            types[topic] = selected[topic] = name
            if schema['encoding'] and schema['data_size']:
                if schema['encoding'] != 'ros2msg':
                    raise ValueError('Unexpected source schema encoding: ' + schema['encoding'])
                previous = raw_definitions.setdefault(name, schema['data'])
                if previous != schema['data']:
                    raise ValueError('Conflicting complete schemas for ' + name)
            elif '/_action/' not in topic:
                raise ValueError('Only missing Action schemas can be reconstructed: ' + topic)
        rows.extend(messages(source, selected))
        sources.append(dict(path=str(source), sha256=sha256(source)))
    if not rows:
        raise ValueError('Cannot deliver an empty ROSbag')
    present = {row.topic for row in rows}
    types = {topic: name for topic, name in types.items() if topic in present}
    missing = set(types.values()) - set(raw_definitions)
    for name in sorted(missing):
        raw_definitions[name] = generated.full(name)

    class CompleteDefinitions:
        def full(self, name):
            return raw_definitions[name]

    destination = output / 'viewing_bag'
    destination.mkdir()
    path = destination / 'viewing.mcap'
    write_companion(path, rows, types, CompleteDefinitions(),
                    library='minimal-debug lossless self-contained ROSbag schemas')
    restored = list(messages(path, types))
    if Counter(restored) != Counter(rows):
        raise AssertionError('Self-contained ROSbag changed original topic/CDR/log/publish/sequence')
    schemas, channels = mcap_summary_schemas(path)
    if any(not schemas[c['schema_id']]['encoding'] or not schemas[c['schema_id']]['data_size']
           or c['message_encoding'] != 'cdr' for c in channels.values()):
        raise AssertionError('Delivered MCAP has an incomplete schema or non-CDR channel')
    classes = {name: native_class(name) for name in set(types.values())}
    source_domain_path = output / 'source_time_domain.json'
    source_domain = json.loads(source_domain_path.read_text()) if source_domain_path.is_file() else {}
    source_window_ns = source_domain.get('source_window_ns')
    timing, domain_errors, goals = {}, [], []
    header_validated_types = set()
    native_decoded, header_audited = Counter(), Counter()
    for row in restored:
        type_name = types[row.topic]
        decoded, sources_ns, header_only = delivery_message_stamps(
            row.data, type_name, classes[type_name], header_validated_types, deserialize_message)
        if header_only:
            header_audited[row.topic] += 1
        else:
            native_decoded[type_name] += 1
        if sources_ns:
            entry = timing.setdefault(row.topic, dict(samples=0, min_log_ns=row.log,
                max_log_ns=row.log, min_source_ns=min(sources_ns), max_source_ns=max(sources_ns),
                maximum_absolute_difference_sec=0.))
            entry['samples'] += len(sources_ns)
            entry['min_log_ns'], entry['max_log_ns'] = min(entry['min_log_ns'], row.log), max(entry['max_log_ns'], row.log)
            entry['min_source_ns'] = min(entry['min_source_ns'], *sources_ns)
            entry['max_source_ns'] = max(entry['max_source_ns'], *sources_ns)
            entry['maximum_absolute_difference_sec'] = max(entry['maximum_absolute_difference_sec'],
                *(abs(row.log-value)/1e9 for value in sources_ns))
            for value in sources_ns:
                if not timestamp_domain_valid(row.log, value,
                        source_window_ns=source_window_ns) and len(domain_errors) < 20:
                    domain_errors.append(dict(topic=row.topic, log_ns=row.log, source_ns=value))
        if row.topic in ('/goal_pose', '/validation/goal_pose'):
            goals.append(dict(topic=row.topic, frame_id=decoded.header.frame_id,
                              log_ns=row.log, stamp_ns=sources_ns))
    domain_valid = bool(timing.get('/clock')) and not domain_errors
    domain_report = dict(valid=domain_valid, topics=timing, errors=domain_errors,
        bound_sec=None if source_window_ns else 300, source_window_ns=source_window_ns,
        meaning='exact original source time domain, including cached headers' if source_window_ns
            else 'same short-trial time domain; not network or algorithm latency',
        source_timestamps_edited=False, log_timestamps_edited=False)
    write_json(output / 'time_domain_report.json', domain_report)
    if not domain_valid:
        raise AssertionError('Recorded log_time and message/TF stamps are not in the same simulated time domain')
    if any(goal['frame_id'] != 'map' for goal in goals):
        raise AssertionError('A validation goal was published outside map frame')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    independently_read = Counter()
    while reader.has_next():
        topic, payload, log = reader.read_next()
        independently_read[(topic, bytes(payload), log)] += 1
    if independently_read != Counter((r.topic, r.data, r.log) for r in rows):
        raise AssertionError('Independent reader differs from complete delivered MCAP')
    if any(not item.encoding or not item.encoded_message_definition
           for item in reader.get_all_message_definitions()):
        raise AssertionError('Independent reader found incomplete delivered schemas')
    reader.close()
    # Keep a standard ROSbag directory as well as the directly openable MCAP.
    metadata = copy.deepcopy(yaml.safe_load((output / 'bag/metadata.yaml').read_text()))
    info = metadata['rosbag2_bagfile_information']
    start, end = min(r.log for r in rows), max(r.log for r in rows)
    counts = Counter(r.topic for r in rows)
    topic_metadata = {entry['topic_metadata']['name']: entry['topic_metadata']
                      for entry in info['topics_with_message_count']}
    info.update(duration=dict(nanoseconds=end-start), starting_time=dict(nanoseconds_since_epoch=start),
        message_count=len(rows), relative_file_paths=[path.name],
        topics_with_message_count=[dict(topic_metadata=topic_metadata[topic], message_count=counts[topic])
                                   for topic in sorted(counts)],
        files=[dict(path=path.name, starting_time=dict(nanoseconds_since_epoch=start),
                    duration=dict(nanoseconds=end-start), message_count=len(rows))])
    (destination / 'metadata.yaml').write_text(yaml.safe_dump(metadata, sort_keys=False))
    if any(sha256(item['path']) != item['sha256'] for item in sources):
        raise AssertionError('Recorder original was modified while creating delivery bag')
    result = dict(success=True, directory=str(destination), mcap=str(path), sha256=sha256(path),
        metadata_sha256=sha256(destination / 'metadata.yaml'), original_files_modified=False,
        sources=sources, preserved=['topic', 'CDR', 'log_time', 'publish_time', 'sequence'],
        raw_records_identical=True, native_decoded_records=sum(native_decoded.values()),
        native_decoded_by_type=dict(native_decoded),
        cdr_header_audited_records=sum(header_audited.values()),
        cdr_header_audited_by_topic=dict(header_audited),
        cdr_header_native_validated_types=sorted(header_validated_types),
        validated_records=sum(native_decoded.values()) + sum(header_audited.values()),
        validation_scope='Every record retains original CDR/time/sequence and independent reader comparison; '
            'CustomMsg is natively decoded first, then every remaining serialized Header is time-domain audited '
            'without constructing point objects; all other records retain full native decoding',
        time_domain_valid=domain_valid, goal_messages=goals,
        independent_reader_records=sum(independently_read.values()),
        topics={topic: dict(type=types[topic], message_count=counts[topic]) for topic in sorted(counts)},
        generated_schema_types=sorted(missing), generated_feedback_class_checks=generated.class_checks,
        schema_sources=[dict(path=str(p), sha256=sha256(p)) for p in sorted(generated.sources)],
        generator_sha256=GENERATOR_SOURCE_SHA256, generator_source_path=str(GENERATOR_SOURCE_PATH),
        timestamp_audit_method='native_decode_all_other_records; '
            'CustomMsg_native_first_then_supported_CDR_Header_v1_with_native_fallback',
        generator_scope='Offline postprocess module identity at import; lazy import may differ from '
            'the session-start runtime manifest. Native runtime CPU/quality and original CDR are unchanged.',
        ros_distro=os.environ.get('ROS_DISTRO'))
    write_json(output / 'delivery_bag_report.json', result)
    return result


def recover(output):
    from rclpy.serialization import deserialize_message, serialize_message
    from rosidl_runtime_py.utilities import get_message
    output = Path(output)
    types, rows, sources = {}, [], []
    for path in sorted((output / 'bag').glob('*.mcap')):
        schemas, channels = mcap_summary_schemas(path)
        selected = {}
        for channel in channels.values():
            schema = schemas.get(channel['schema_id'], {})
            if not schema.get('encoding') or not schema.get('data_size'):
                if '/_action/' not in channel['topic']:
                    raise ValueError('Non-Action schema missing; cannot silently repair: ' + channel['topic'])
                selected[channel['topic']] = schema['name']
        types.update(selected)
        rows.extend(messages(path, selected))
        sources.append(dict(path=str(path), sha256=sha256(path)))
    # Channels with no messages need no companion topic.
    present = {row.topic for row in rows}
    types = {topic: value for topic, value in types.items() if topic in present}
    result = dict(needed=bool(rows), original_files_modified=False, sources=sources,
                  ros_distro=os.environ.get('ROS_DISTRO'),
                  topics=dict(Counter(row.topic for row in rows)))
    if rows:
        definitions = Definitions()
        path = output / 'action_schemas.mcap'
        write_companion(path, rows, types, definitions)
        restored = list(messages(path, types))
        if Counter(rows) != Counter(restored):
            raise AssertionError('Action CDR/time/sequence changed during schema recovery')
        for row in restored:
            name = types[row.topic]
            if '/action/' in name:
                package, _, kind = name.split('/')
                cls = getattr(importlib.import_module(package + '.action'), kind[:-len('_FeedbackMessage')]).Impl.FeedbackMessage
            else:
                cls = get_message(name)
            decoded = deserialize_message(row.data, cls)
            if deserialize_message(serialize_message(decoded), cls) != decoded:
                raise AssertionError('Native ROS semantic decode failed')
        import rosbag2_py
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                    rosbag2_py.ConverterOptions('', ''))
        independently_read = Counter()
        while reader.has_next():
            topic, payload, log = reader.read_next()
            independently_read[(topic, bytes(payload), log)] += 1
        if independently_read != Counter((row.topic, row.data, row.log) for row in rows):
            raise AssertionError('Independent rosbag2 reader differs from Action source records')
        if any(not item.encoding or not item.encoded_message_definition
               for item in reader.get_all_message_definitions()):
            raise AssertionError('Independent reader found incomplete companion schemas')
        reader.close()
        result.update(companion=str(path), companion_sha256=sha256(path),
            preserved=['CDR', 'log_time', 'publish_time', 'sequence', 'topic'],
            raw_records_identical=True, native_ros_decode_count=len(restored),
            independent_rosbag2_read_count=sum(independently_read.values()),
            schema_sources=[dict(path=str(source), sha256=sha256(source)) for source in sorted(definitions.sources)],
            schema_generator_sha256=sha256(__file__),
            generated_feedback_class_checks=definitions.class_checks,
            foxglove='Open original bag MCAPs plus action_schemas.mcap. /debug/view is the primary state timeline. Do not rosbag-play the companion with the original: Action traffic would be duplicated.')
    write_json(output / 'schema_recovery.json', result)
    return result

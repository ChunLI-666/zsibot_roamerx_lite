"""Read-only SHA + MCAP summary/schema/count audit. Never decodes message CDR.

Record layout: https://mcap.dev/spec (Footer, Schema, Channel, Statistics).
This checks summaries, not every compressed chunk, CRC or image payload.
"""
import hashlib
from pathlib import Path
import struct

MAGIC = b'\x89MCAP0\r\n'


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def summary(path):
    path = Path(path)
    result = dict(path=str(path), bytes=path.stat().st_size, sha256=sha(path),
                  audit='summary_only_no_chunk_CRC_or_CDR_decode', schemas={}, channels={})
    with path.open('rb') as stream:
        if stream.read(8) != MAGIC or result['bytes'] < 45:
            raise ValueError('missing MCAP initial magic or truncated file')
        stream.seek(-37, 2)
        footer = stream.read(37)
        if footer[0] != 2 or struct.unpack_from('<Q', footer, 1)[0] != 20 or footer[-8:] != MAGIC:
            raise ValueError('missing final MCAP footer/magic')
        start, offsets, crc = struct.unpack_from('<QQI', footer, 9)
        result.update(summary_start=start, summary_offset_start=offsets, summary_crc_present=bool(crc))
        if start == 0:
            result['summary_available'] = False
            return result
        end = result['bytes'] - 37
        if not 8 <= start < end or (offsets and not start <= offsets < end):
            raise ValueError('MCAP summary offsets outside file')
        stream.seek(start)
        while stream.tell() < end:
            header = stream.read(9)
            if len(header) != 9:
                raise ValueError('truncated summary record')
            opcode, length = struct.unpack('<BQ', header)
            if length > 16*1024*1024 or stream.tell() + length > end:
                raise ValueError('summary record exceeds bounded section')
            data = stream.read(length)
            def string(position):
                size = struct.unpack_from('<I', data, position)[0]
                stop = position + 4 + size
                if stop > len(data):
                    raise ValueError('truncated summary string')
                return data[position+4:stop].decode('utf-8'), stop
            if opcode == 3:
                identifier = struct.unpack_from('<H', data)[0]
                name, position = string(2)
                encoding, position = string(position)
                size = struct.unpack_from('<I', data, position)[0]
                raw = data[position+4:position+4+size]
                if len(raw) != size:
                    raise ValueError('truncated schema')
                result['schemas'][str(identifier)] = dict(name=name, encoding=encoding, bytes=size,
                                                         sha256=hashlib.sha256(raw).hexdigest())
            elif opcode == 4:
                identifier, schema_id = struct.unpack_from('<HH', data)
                topic, position = string(4)
                encoding, _ = string(position)
                result['channels'][str(identifier)] = dict(topic=topic, schema_id=schema_id, message_encoding=encoding)
            elif opcode == 11:
                values = struct.unpack_from('<QHIIIIQQI', data)
                size = values[-1]
                if size % 10 or len(data) < 46 + size:
                    raise ValueError('invalid channel statistics map')
                counts = dict(struct.iter_unpack('<HQ', data[46:46+size]))
                result['statistics'] = dict(message_count=values[0], schema_count=values[1],
                    channel_count=values[2], attachment_count=values[3], metadata_count=values[4],
                    chunk_count=values[5], message_start_time_ns=values[6], message_end_time_ns=values[7],
                    channel_message_counts={str(key):value for key,value in counts.items()})
        if stream.tell() != end:
            raise ValueError('summary ended outside footer')
        result['summary_available'] = True
    return result


def audit(bag, required_topics):
    import yaml  # Jazzy's existing PyYAML; no installation or ROS context.
    bag = Path(bag)
    metadata = bag / 'metadata.yaml'
    if not metadata.is_file():
        return dict(complete=False, reason='metadata_missing', mcap_files=[str(p) for p in bag.glob('*.mcap')])
    raw = yaml.safe_load(metadata.read_text())['rosbag2_bagfile_information']
    counts = {entry['topic_metadata']['name']:entry['message_count'] for entry in raw['topics_with_message_count']}
    result = dict(metadata_sha256=sha(metadata), metadata=raw, topic_counts=counts, files=[],
                  missing_or_empty_required_topics=sorted(t for t in required_topics if counts.get(t, 0) <= 0),
                  audit='metadata_and_MCAP_summary_only_no_payload_decode')
    paths = [bag / name for name in raw['relative_file_paths']]
    if not paths or any(p.resolve().parent != bag.resolve() or p.suffix != '.mcap' for p in paths):
        raise ValueError('metadata contains unexpected bag member paths')
    result['files'] = [summary(path) for path in paths]
    summary_counts = {}
    invalid_required_schemas = []
    for entry in result['files']:
        for identifier, count in entry.get('statistics', {}).get('channel_message_counts', {}).items():
            topic = entry['channels'][identifier]['topic']
            summary_counts[topic] = summary_counts.get(topic, 0) + count
        for channel in entry['channels'].values():
            schema = entry['schemas'].get(str(channel['schema_id']), {})
            if channel['topic'] in required_topics and (channel['message_encoding'] != 'cdr' or not schema.get('bytes')):
                invalid_required_schemas.append(channel['topic'])
    result['summary_topic_counts'] = summary_counts
    # A discovered topic may have zero messages and no entry in MCAP's statistics map.
    result['summary_counts_match_metadata'] = ({key:value for key,value in summary_counts.items() if value}
                                               == {key:value for key,value in counts.items() if value})
    result['invalid_required_schema_topics'] = sorted(set(invalid_required_schemas))
    result['total_counts_consistent'] = (raw['message_count'] == sum(counts.values())
        == sum(entry.get('statistics', {}).get('message_count', -1) for entry in result['files']))
    result['complete'] = (not result['missing_or_empty_required_topics'] and result['summary_counts_match_metadata']
                          and not invalid_required_schemas and result['total_counts_consistent']
                          and all(entry.get('statistics') for entry in result['files']))
    return result

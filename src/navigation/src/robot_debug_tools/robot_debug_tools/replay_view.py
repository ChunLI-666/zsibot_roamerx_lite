"""Read an indexed MCAP and write a standalone, offline diagnostics report."""

import argparse
from decimal import Decimal, InvalidOperation
import hashlib
from html import escape
import json
import math
from pathlib import Path
import struct

from .replay_projection import UNKNOWN, project


TOPIC_TYPES = {
    '/debug/observations': 'robot_debug_msgs/msg/StatusObservation',
    '/debug/view': 'robot_debug_msgs/msg/DebugView',
    '/debug/build_info': 'robot_debug_msgs/msg/BuildInfo',
    '/behavior_tree_log': 'nav2_msgs/msg/BehaviorTreeLog',
    '/lightning/debug': 'lightning/msg/LightningDebug',
    '/lightning/loc_status': 'std_msgs/msg/UInt8',
    '/nav_safety_gate/gate_status': 'std_msgs/msg/UInt8',
    '/goal_pose': 'geometry_msgs/msg/PoseStamped',
    '/cmd_vel_nav': 'geometry_msgs/msg/Twist',
    '/cmd_vel': 'geometry_msgs/msg/Twist',
    '/cmd_vel_safe': 'geometry_msgs/msg/Twist',
}
CDR_SCHEMA = 'ros2msg'
NANOSECONDS = 10**9


def topic_type(topic):
    if topic.endswith('/_action/status'):
        return 'action_msgs/msg/GoalStatusArray'
    return TOPIC_TYPES.get(topic)


def seconds(text):
    try:
        value = Decimal(text)
        if not value.is_finite() or not 0 <= value <= Decimal('18446744073'):
            raise ValueError
        ns = value * NANOSECONDS
        if ns != ns.to_integral_value():
            raise ValueError
        return int(ns)
    except (InvalidOperation, ValueError) as exc:
        raise argparse.ArgumentTypeError('expected nonnegative seconds, with up to nanosecond resolution') from exc


def as_data(value):
    if isinstance(value, bytes):
        return list(value)
    if isinstance(value, (tuple, list)):
        return [as_data(item) for item in value]
    if hasattr(value, '__slots__'):
        return {name: as_data(getattr(value, name)) for name in value.__slots__}
    return value


def json_value(value):
    if isinstance(value, dict):
        return {key: json_value(item) for key, item in value.items()}
    if isinstance(value, list):
        return [json_value(item) for item in value]
    if type(value) is int and abs(value) >= 2**53:
        return str(value)
    if isinstance(value, float) and not math.isfinite(value):
        return {'nonfinite_float': str(value)}
    return value


def schema_definitions(reader, path):
    with path.open('rb') as stream:
        summary = reader(stream).get_summary()
        if summary is None:
            raise ValueError(f'no MCAP summary in schema companion {path}')
    values = {}
    for schema in summary.schemas.values():
        if schema.encoding == CDR_SCHEMA and schema.data:
            previous = values.setdefault(schema.name, schema)
            if previous.data != schema.data:
                raise ValueError(f'ambiguous companion schema {schema.name}')
    return values


def read_window(args):
    from mcap.exceptions import McapError
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    definitions = schema_definitions(make_reader, args.schemas_from) if args.schemas_from else {}
    # Separate decoder caches since schema IDs are local to each MCAP.
    native, supplemental = DecoderFactory(), DecoderFactory()
    with args.bag.open('rb') as stream:
        reader = make_reader(stream)
        summary = reader.get_summary()
        if not summary or not summary.statistics or not summary.chunk_indexes:
            raise ValueError('expected an indexed MCAP with message statistics')
        origin = summary.statistics.message_start_time
        topics = {channel.topic for channel in summary.channels.values() if topic_type(channel.topic)}
        if args.topic:
            topics.intersection_update(args.topic)
        if not topics:
            return [], False
        messages = reader.iter_messages(topics=topics, start_time=origin + args.start,
                                        end_time=origin + args.start + args.duration)
        records = []
        for schema, channel, msg in messages:
            if len(records) == args.limit:
                return records, True
            full_schema = schema
            decoder = native
            schema_source = 'Embedded'
            if schema is not None and not schema.data and schema.name in definitions:
                full_schema = definitions[schema.name]
                decoder = supplemental
                schema_source = str(args.schemas_from)
            data, error = None, ''
            try:
                decode = decoder.decoder_for(channel.message_encoding, full_schema)
                if decode is not None:
                    data = as_data(decode(msg.data))
                else:
                    error = 'Missing or unsupported schema/encoding'
            except (ValueError, TypeError, AttributeError, IndexError, KeyError, OverflowError,
                    RuntimeError, McapError, struct.error) as exc:
                error = f'Decode failed: {exc}'
            if isinstance(data, dict) and full_schema.name == topic_type(channel.topic):
                payload = project(channel.topic, data)
            elif data is not None:
                payload = {'phase': 'Raw message', 'title': 'Unexpected message type',
                           'facts': {'Expected type': topic_type(channel.topic), 'Source stamp': UNKNOWN}}
            else:
                payload = {'phase': 'Decode unavailable', 'title': error, 'facts': {'Source stamp': UNKNOWN}}
            seconds_part, ns_part = divmod(msg.log_time - origin, NANOSECONDS)
            records.append({
                'relative': f'{seconds_part}.{ns_part:09d}', 'topic': channel.topic, **payload, 'raw': data,
                'locator': {
                    'Topic': channel.topic, 'Log time (ns)': str(msg.log_time),
                    'Publish time (ns)': str(msg.publish_time), 'Channel ID': channel.id,
                    'Schema ID': channel.schema_id, 'Schema type': schema.name if schema else UNKNOWN,
                    'Schema source': schema_source if data is not None else UNKNOWN,
                    'Sequence': msg.sequence, 'CDR bytes': len(msg.data),
                    'CDR SHA-256': hashlib.sha256(msg.data).hexdigest(),
                },
            })
    return records, False


PAGE = """<!doctype html>
<html lang="en">
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Navigation debug replay</title>
<style>
*{box-sizing:border-box}
body{margin:0;background:#edf2f3;color:#16242c;font:16px/1.5 system-ui,sans-serif}
main{width:min(1200px,calc(100% - 24px));margin:28px auto}
h1,h2{line-height:1.2} p,dd,summary{overflow-wrap:anywhere}
details{margin:.6rem 0}
summary{cursor:pointer;background:#dceced;padding:.65rem .8rem;border-radius:.35rem}
time{font-variant-numeric:tabular-nums;font-weight:650}
.event{border:1px solid #b6c9cb;border-radius:.4rem;background:white}
.compare{display:grid;grid-template-columns:minmax(0,1fr) minmax(0,1fr);gap:1rem;padding:.8rem}
dt{font-weight:600;color:#36565e} dd{margin:0 0 .45rem}
pre{margin:0;max-height:42rem;overflow:auto;background:#172e36;color:#edf2f3;padding:.7rem;border-radius:.3rem}
@media(max-width:720px){.compare{grid-template-columns:minmax(0,1fr)}}
</style>
<main>
<h1>Navigation debug replay</h1>
"""


def value_html(value):
    if isinstance(value, dict):
        return fields_html(value)
    if isinstance(value, list):
        if not value:
            return 'None reported'
        items = '<ol>' + ''.join(f'<li>{value_html(item)}</li>' for item in value) + '</ol>'
        if len(value) > 3:
            return f'<details><summary>{len(value)} recorded entries</summary>{items}</details>'
        return items
    if isinstance(value, bool):
        return 'true' if value else 'false'
    return escape(str(value))


def fields_html(data):
    fields = (f'<dt>{escape(key)}</dt><dd>{value_html(value)}</dd>' for key, value in data.items())
    return '<dl>' + ''.join(fields) + '</dl>'


def write_report(output, source, records, truncated):
    count = f'{len(records)} recorded messages' + (' · limit reached, more records available' if truncated else '')
    with output.open('x', encoding='utf-8') as stream:
        stream.write(PAGE)
        stream.write(f'<p>{escape(str(source))}</p><p>{count}</p>')
        stream.write('<p>Ordered by MCAP log time. Publish time, source stamps and watchdog durations '
                     'are displayed separately. Each entry represents one recorded message.</p>')
        for index, event in enumerate(records, 1):
            heading = ' · '.join(escape(str(event[key])) for key in ('phase', 'title', 'topic'))
            opening = ' open' if index == 1 else ''
            raw = escape(json.dumps(json_value(event['raw']), ensure_ascii=False, allow_nan=False, indent=2))
            stream.write(f'<details class="event" id="event-{index}"{opening}><summary>'
                         f'<time>{event["relative"]} s</time> · {heading}</summary><div class="compare">'
                         f'<section><h2>Recorded facts</h2>{fields_html(json_value(event["facts"]))}')
            if 'builds' in event:
                stream.write(f'<details><summary>Build identity</summary>{fields_html(event["builds"])}</details>')
            stream.write(f'<details class="locator"><summary>Original record locator</summary>'
                         f'{fields_html(event["locator"])}</details></section>'
                         f'<section><h2>Raw decoded message</h2><pre tabindex="0">{raw}</pre></section>'
                         '</div></details>\n')
        stream.write('</main></html>\n')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path, help='indexed input .mcap; use one file per split')
    parser.add_argument('output', type=Path, help='new standalone HTML file')
    parser.add_argument('--start', type=seconds, default=0, help='seconds from first recording log time')
    parser.add_argument('--duration', type=seconds, default=5 * NANOSECONDS, help='window length in seconds')
    parser.add_argument('--limit', type=int, default=500, help='maximum records; default 500, maximum 5000')
    parser.add_argument('--topic', action='append', help='restrict to an exact supported topic, repeatable')
    parser.add_argument('--schemas-from', type=Path, help='use only missing schemas from this companion MCAP')
    args = parser.parse_args(argv)
    if not 1 <= args.limit <= 5000 or args.duration == 0:
        parser.error('require positive duration and 1–5000 records')
    if args.topic and any(not topic_type(topic) for topic in args.topic):
        parser.error('unsupported --topic')
    try:
        from mcap.exceptions import McapError
    except ImportError as exc:
        parser.exit(1, f'{exc}. Install the offline Python dependencies mcap and mcap-ros2-support.\n')
    try:
        records, truncated = read_window(args)
        write_report(args.output, args.bag.resolve(), records, truncated)
    except ImportError as exc:
        parser.exit(1, f'{exc}. Install the offline Python dependencies mcap and mcap-ros2-support.\n')
    except (OSError, ValueError, McapError, struct.error) as exc:
        parser.exit(1, f'{exc}\n')

"""Full-source playback evidence and scheduling; no navigation policy."""
import hashlib
import math
import struct


class CdrStreamDigest:
    """Hash ordered original serialized samples without retaining point arrays."""
    def __init__(self):
        self.digest = hashlib.sha256()
        self.count = 0
        self.first_header_ns = self.last_header_ns = None

    def add(self, data):
        data = bytes(data)
        self.digest.update(struct.pack('<Q', len(data)))
        self.digest.update(data)
        endian = '<' if data[1] & 1 else '>'
        seconds, nanoseconds = struct.unpack_from(endian + 'iI', data, 4)
        timestamp = seconds * 1_000_000_000 + nanoseconds
        if self.count == 0:
            self.first_header_ns = timestamp
        self.last_header_ns = timestamp
        self.count += 1

    def snapshot(self):
        return dict(count=self.count, first_header_ns=self.first_header_ns,
                    last_header_ns=self.last_header_ns,
                    ordered_cdr_sha256=self.digest.hexdigest())


def source_evidence(bag, sensors):
    """Read only the selected original messages; no ROS node or timestamp edits."""
    import rosbag2_py
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=sensors))
    digests = {topic: CdrStreamDigest() for topic in sensors}
    bounds = {}
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        digests[topic].add(data)
        bounds.setdefault(topic, [timestamp, timestamp])[1] = timestamp
    result = {}
    for topic, digest in digests.items():
        if not digest.count:
            raise ValueError('Original source topic is empty: ' + topic)
        result[topic] = dict(**digest.snapshot(), first_log_ns=bounds[topic][0],
                             last_log_ns=bounds[topic][1])
    return result


def coverage_report(expected, actual):
    errors = []
    for topic, source in expected.items():
        observed = actual.get(topic, {})
        for key in ('count', 'first_header_ns', 'last_header_ns', 'ordered_cdr_sha256'):
            if observed.get(key) != source[key]:
                errors.append(dict(topic=topic, field=key, expected=source[key],
                                   actual=observed.get(key)))
    return dict(valid=not errors, expected=expected, received=actual, errors=errors,
                meaning='Every raw sensor CDR received in original per-topic order, including first and last')


def full_schedule(metadata, goal_seconds, teardown_seconds, tail_seconds):
    start = metadata['starting_time']['nanoseconds_since_epoch']
    duration = metadata['duration']['nanoseconds']
    if not all(math.isfinite(v) for v in (goal_seconds, teardown_seconds, tail_seconds)):
        raise ValueError('Playback schedule must be finite')
    if goal_seconds < 0 or teardown_seconds < tail_seconds + 10:
        raise ValueError('Reserve at least tail + 10 seconds before EOF for navigation cleanup')
    end = start + duration
    teardown = end - round(teardown_seconds * 1e9)
    goal = start + round(goal_seconds * 1e9)
    if goal >= teardown:
        raise ValueError('Goal must precede the teardown window')
    return dict(start_ns=start, end_ns=end, duration_seconds=duration / 1e9,
                goal_ns=goal, teardown_ns=teardown,
                teardown_before_eof_seconds=teardown_seconds)


def validate_playback_options(args):
    if args.rate <= 0 or not math.isfinite(args.rate) or args.start_offset < 0:
        raise ValueError('Invalid playback bounds')
    if args.full_bag_mode:
        if args.duration is not None or args.start_offset != 0:
            raise ValueError('Full bag mode forbids --duration and nonzero --start-offset')
    elif (args.duration is not None and args.duration <= 0) or args.observe <= 0:
        raise ValueError('Invalid playback/observation duration')


def player_command(args, sensors):
    command = ['ros2', 'bag', 'play', str(args.bag), '--clock', '100',
        '--disable-keyboard-controls', '--rate', str(args.rate)]
    if args.full_bag_mode:
        # The paused player publishes its real initial /clock. Establish all
        # recorder/auditor subscriptions before resuming the first raw sample.
        command += ['--start-paused', '--wait-for-all-acked', '10000']
    else:
        command += ['--start-offset', str(args.start_offset), '--playback-duration',
                    str(args.duration if args.duration is not None else 120.)]
    return command + ['--topics', *sensors]


def require_natural_eof(process):
    code = process.poll()
    if code is None:
        raise RuntimeError('Source player has not reached EOF')
    if code != 0:
        raise RuntimeError('Source player failed before natural EOF: ' + str(code))
    return code


def finish_original_source(monitor, processes, player, schedule, expected, rate):
    """Finalize navigation with live source, then consume its complete tail."""
    monitor.lifecycle_shutdown(processes)
    monitor.record('full_source_navigation_finalized', dict(
        sim_time_ns=monitor.get_clock().now().nanoseconds,
        player_still_running=player.poll() is None))
    if player.poll() is not None:
        raise AssertionError('Original source ended before navigation teardown completed')
    remaining = max(0., (schedule['end_ns']-monitor.get_clock().now().nanoseconds)/1e9/rate)
    monitor.spin_until(lambda: player.poll() is not None, remaining+30.,
        'complete original source natural EOF', processes, allow=('player',))
    require_natural_eof(player)
    monitor.spin_until(lambda: coverage_report(expected, monitor.raw_source_report())['valid'],
        10., 'drain complete final serialized raw samples', processes, allow=('player',))
    monitor.spin_for(2., processes, allow=('player',))
    coverage = coverage_report(expected, monitor.raw_source_report())
    coverage.update(player_returncode=player.returncode, natural_eof=True,
        original_start_ns=schedule['start_ns'], original_end_ns=schedule['end_ns'],
        last_clock_ns=monitor.get_clock().now().nanoseconds,
        navigation_finalized_before_eof=True, timestamps_edited=False, synthetic_clock=False)
    return coverage

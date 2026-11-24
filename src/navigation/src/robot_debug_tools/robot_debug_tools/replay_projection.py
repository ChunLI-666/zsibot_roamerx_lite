"""Display recorded facts without maintaining a second navigation state machine."""

import math

UNKNOWN = 'Unknown'
ACTION = {0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING',
          4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
LOCATION = {0: 'UNKNOWN', 1: 'NORMAL', 2: 'DEGRADED', 3: 'LOST'}
GATE = {**LOCATION, 4: 'EMERGENCY_STOP', 5: 'CMD_TIMEOUT', 6: 'INVALID_CMD'}


def field(data, key):
    value = data.get(key)
    return value if value is not None and value != '' else UNKNOWN


def valid(data, flag, key):
    return field(data, key) if data.get(flag) is True else UNKNOWN


def stamp(value):
    if not isinstance(value, dict):
        return UNKNOWN
    sec, ns = value.get('sec'), value.get('nanosec')
    if type(sec) is not int or type(ns) is not int or not 0 <= ns < 10**9:
        return UNKNOWN
    # ROS time zero is used for uninitialized headers, including BuildInfo at startup.
    if (sec, ns) == (0, 0):
        return '0.000000000 (unset or clock origin)'
    whole, fractional = divmod(abs(sec * 10**9 + ns), 10**9)
    return f'{"-" if sec < 0 else ""}{whole}.{fractional:09d}'


def uuid(value):
    octets = value.get('uuid') if isinstance(value, dict) else None
    if isinstance(octets, list) and len(octets) == 16 and all(type(v) is int and 0 <= v <= 255 for v in octets):
        return bytes(octets).hex()
    return UNKNOWN


def code(names, value):
    return f'{names.get(value, UNKNOWN)} ({value})' if type(value) is int else UNKNOWN


def header(data):
    msg = data.get('header', {})
    return {'ROS source stamp': stamp(msg.get('stamp')), 'Frame': field(msg, 'frame_id')}


def build(value):
    return value if isinstance(value, dict) and value.get('known') is True else UNKNOWN


def action_fields(item, cached=False):
    if cached:
        return {
            'Endpoint': field(item, 'endpoint'), 'UUID': uuid(item.get('goal_id')),
            'Recorded status': code(ACTION, item.get('value')),
            'Goal acceptance (ROS)': stamp(item.get('goal_accepted_stamp')),
            'Observer saw at (ROS)': stamp(item.get('observed_at')),
            'Action source build': build(item.get('source_build'))
            if item.get('source_binding_valid') is True else UNKNOWN,
        }
    info = item.get('goal_info', {})
    return {
        'UUID': uuid(info.get('goal_id')), 'Native status': code(ACTION, item.get('status')),
        'Goal acceptance (ROS)': stamp(info.get('stamp')),
    }


def observe(data):
    identity = {
        **header(data), 'Component': field(data, 'component'), 'Endpoint': field(data, 'endpoint'),
        'Source instance': field(data, 'source_instance'),
        'UUID': uuid(data.get('goal_id')) if data.get('goal_id_valid') is True else UNKNOWN,
        'Path revision (recorded)': field(data, 'path_revision'),
        'Dropped records (producer counter)': field(data, 'dropped_records'),
    }
    details = []
    for sample in data.get('gate', []):
        details.append((field(sample, 'actual_branch'), {
            'Gate status': code(GATE, sample.get('gate_status')),
            'Input is zero': valid(sample, 'input_zero_valid', 'input_is_zero'),
            'Output is zero': field(sample, 'output_is_zero'),
            'Command elapsed (steady ms)': valid(sample, 'cmd_elapsed_valid', 'cmd_elapsed_ms'),
            'Localization heartbeat elapsed (steady ms)': valid(sample, 'loc_elapsed_valid', 'loc_elapsed_ms'),
            'Physical execution': UNKNOWN,
        }))
    for sample in data.get('branch', []):
        details.append((f"{field(sample, 'site')}: {field(sample, 'branch')}", {
            'Native boolean return': valid(sample, 'value_valid', 'value'),
            'Original error': sample.get('original_error') or 'No error text recorded',
        }))
    for sample in data.get('checker', []):
        result = valid(sample, 'result_valid', 'result') if sample.get('evaluated') is True else UNKNOWN
        details.append((f"{field(sample, 'checker_id')}: {result}", {
            'Evaluated': field(sample, 'evaluated'), 'Native checker return': result,
            'Path endpoint XY distance (m)': valid(sample, 'geometry_valid', 'path_end_xy_distance_m'),
            'Path endpoint yaw error (rad)': valid(sample, 'geometry_valid', 'path_end_yaw_error_rad'),
            'Geometry frame': valid(sample, 'geometry_valid', 'frame_id'),
            'Robot pose source stamp': stamp(sample.get('robot_stamp')),
            'Path endpoint source stamp': stamp(sample.get('goal_stamp')),
        }))
    for sample in data.get('transport', []):
        details.append((f"Transport: {field(sample, 'operation')}", {
            'Attempted': field(sample, 'attempted'), 'Send OK': valid(sample, 'result_valid', 'send_ok'),
            'Reply seen': field(sample, 'reply_seen'),
            'Reply receipt stamp': stamp(sample.get('reply_receive_stamp')) if sample.get('reply_seen') is True else UNKNOWN,
            'Physical execution': UNKNOWN,
        }))
    for sample in data.get('execution', []):
        details.append((f"Execution: {field(sample, 'event')}", {
            'Command accepted': valid(sample, 'command_accepted_valid', 'command_accepted'),
            'Wrapper called': valid(sample, 'wrapper_called_valid', 'wrapper_called'),
            'Deduplicated': valid(sample, 'dedup_valid', 'dedup'),
            'Wrapper return': valid(sample, 'return_valid', 'wrapper_raw_return'), 'Physical execution': UNKNOWN,
        }))
    return {
        'phase': identity['Component'], 'title': details[0][0] if details else 'No supported observation fields',
        'facts': {**identity, 'Native observations': [{'Reason / branch': label, **facts} for label, facts in details]},
        'builds': {'Source build': build(data.get('source_build')),
                   'Reporter build': build(data.get('reporter_build'))},
    }


def project(topic, data):
    """Summarize exactly one decoded message. No previous messages are consulted."""
    if topic == '/debug/observations':
        return observe(data)
    if topic == '/debug/view':
        actions = [action_fields(item, cached=True) for item in data.get('actions', [])]
        return {
            'phase': 'Observer', 'title': 'Previously observed state',
            'facts': {
                **header(data), 'Action observations': actions,
                'Current checker validity': field(data, 'current_checker_valid'),
                'Checker reason': field(data, 'current_checker_reason'),
                'Observer dropped records': field(data, 'observer_dropped_records'),
            },
            'builds': {'Reporter build': build(data.get('reporter_build'))},
        }
    if topic == '/debug/build_info':
        return {'phase': 'Build', 'title': field(data, 'component'),
                'facts': {**header(data), 'Source instance': field(data, 'source_instance'),
                          'Node': field(data, 'node_fqn'), 'Endpoints': field(data, 'endpoints')},
                'builds': {'Artifact build': build(data.get('build'))}}
    if topic.endswith('/_action/status'):
        actions = [action_fields(item) for item in data.get('status_list', [])]
        statuses = ', '.join(dict.fromkeys(action['Native status'] for action in actions))
        return {'phase': 'Action', 'title': statuses or 'No goals reported',
                'facts': {'Endpoint': topic.removesuffix('/_action/status'), 'Goal states': actions,
                          'State change source stamp': UNKNOWN, 'Action source build': UNKNOWN,
                          'Service result': 'Not in the status message'}}
    if topic == '/behavior_tree_log':
        events = [{
            'Node': field(item, 'node_name'), 'Previous': field(item, 'previous_status'),
            'Current': field(item, 'current_status'), 'BT event stamp (separate clock)': stamp(item.get('timestamp')),
        } for item in data.get('event_log', [])]
        changed = [event for event in events if event['Current'] in ('SUCCESS', 'FAILURE')]
        title = ', '.join(f"{item['Node']}: {item['Current']}" for item in changed[:3])
        return {'phase': 'Behavior tree', 'title': title or 'Node transitions',
                'facts': {'ROS source stamp': stamp(data.get('timestamp')), 'Action UUID': UNKNOWN,
                          'Transitions in recorded order': events}}
    if topic == '/lightning/debug':
        return {'phase': 'Localization', 'title': code(LOCATION, data.get('loc_status')),
                'facts': {**header(data), 'Native reason': field(data, 'loc_state_reason'),
                          'Match source time (s)': field(data, 'loc_stamp'),
                          'LIO source time (s)': field(data, 'lio_stamp'),
                          'IMU source time (s)': field(data, 'imu_stamp'),
                          'Match score': field(data, 'loc_score'), 'Schema version': field(data, 'schema_version'),
                          'Debug sequence': field(data, 'debug_sequence'), 'Source instance': UNKNOWN}}
    if topic in ('/lightning/loc_status', '/nav_safety_gate/gate_status'):
        gate = topic == '/nav_safety_gate/gate_status'
        return {'phase': 'Safety gate' if gate else 'Localization',
                'title': code(GATE if gate else LOCATION, data.get('data')),
                'facts': {'Native code': field(data, 'data'), 'ROS source stamp': UNKNOWN}}
    if topic == '/goal_pose':
        pose = data.get('pose', {})
        return {'phase': 'Goal', 'title': 'Published goal pose',
                'facts': {**header(data), 'Position (m)': field(pose, 'position'),
                          'Orientation (quaternion)': field(pose, 'orientation'), 'UUID': UNKNOWN}}
    linear, angular = data.get('linear', {}), data.get('angular', {})
    values = [group.get(axis) for group in (linear, angular) for axis in ('x', 'y', 'z')]
    finite = all(type(v) in (int, float) and math.isfinite(v) for v in values)
    is_zero = all(v == 0 for v in values) if finite else UNKNOWN
    return {'phase': 'Velocity command', 'title': 'Zero output' if is_zero is True else
            'Nonzero output' if is_zero is False else 'Output validity unknown',
            'facts': {'Linear (m/s)': linear or UNKNOWN, 'Angular (rad/s)': angular or UNKNOWN,
                      'ROS source stamp': UNKNOWN, 'Physical execution': UNKNOWN}}

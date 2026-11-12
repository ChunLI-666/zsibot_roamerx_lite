"""Pure adapter for full_scene_mapping.py result.json; no ROS or process control."""
import math
import time

SCOPE = 'MAPPING_ACQUISITION_ONLY'
MODE = 'controlled_static_sim_scene'
SUCCESS = 'controlled_static_sim_raw_acquisition_completed'


def assess(value, controller, identity, release_sha256, recorder_ready_sha256, bound_steady_ns):
    """Only a bound, exited controller's complete measured-stop result earns a tail."""
    expected = dict(root_release_sha256=release_sha256,
                    route_sha256=controller['route_sha256'], oracle_sha256=controller['oracle_sha256'],
                    sources_sha256=controller['sources_sha256'], output_directory=controller['output_directory'],
                    case_id=controller['case_id'], recording_run_id=controller['recording_run_id'],
                    recorder_ready_sha256=recorder_ready_sha256)
    if (value.get('scope') != SCOPE or value.get('mode') != MODE
            or value.get('run_id') != controller['run_id']
            or value.get('owner_process_identity') != identity
            or value.get('runtime_binding') != expected
            or value.get('formal_navigation_acceptance') is not False):
        return False, 'result_identity_or_runtime_binding_mismatch'
    if value.get('classification') != SUCCESS:
        return False, 'controller_reported_' + str(value.get('classification'))
    writer = value.get('writer', {})
    if (value.get('stop_verified') is not True or value.get('relay_closed') is not True
            or value.get('relay_close_errors') != [] or value.get('relay_returncode') != 0
            or writer.get('evidence_complete') is not True or writer.get('dropped') != 0
            or writer.get('worker_error') is not None or writer.get('writer_alive') is not False):
        return False, 'result_stop_relay_or_evidence_incomplete'
    stop = value.get('final_stop_observation', {})
    truth = stop.get('truth', {})
    xyz, quaternion, twist = truth.get('xyz', []), truth.get('q', []), truth.get('twist', [])
    finite = lambda data: all(type(v) in (int, float) and math.isfinite(v) for v in data)
    checked = stop.get('checked_steady_ns')
    if (stop.get('command') != [0., 0.] or not finite([stop.get('continuous_seconds')])
            or stop['continuous_seconds'] < .3 or type(checked) is not int
            or not bound_steady_ns <= checked <= time.monotonic_ns() + 50_000_000
            or truth.get('frame_id') != 'map' or truth.get('child_frame_id') != 'BASE_LINK'
            or type(truth.get('stamp_ns')) is not int or truth['stamp_ns'] <= 0
            or len(xyz) != 3 or len(quaternion) != 4 or len(twist) != 6
            or not finite(xyz + quaternion + twist)
            or abs(sum(v*v for v in quaternion) - 1.) > 1e-3
            or math.sqrt(sum(v*v for v in twist[:3])) > .03
            or math.sqrt(sum(v*v for v in twist[3:])) > .05):
        return False, 'final_original_truth_measured_stop_invalid'
    checked_unix = stop.get('checked_unix_ns')
    received = truth.get('received_steady')
    if type(checked_unix) is not int or not finite([received]):
        return False, 'final_original_truth_stop_freshness_missing'
    header_age = (checked_unix - truth['stamp_ns']) / 1e9
    receipt_age = checked / 1e9 - received
    if not -.05 <= header_age <= .3 or not 0. <= receipt_age <= .3:
        return False, 'final_original_truth_stop_stale_or_future'
    return True, 'bound_complete_mapping_and_measured_stop'

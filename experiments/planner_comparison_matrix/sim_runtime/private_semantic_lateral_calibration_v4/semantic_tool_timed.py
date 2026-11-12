#!/usr/bin/env python3
"""Default-HOLD, direct bounded semantic lateral backend for a reviewed parent.

No ROS, relay, controller spawn, stand button or runtime-config operation.
The parent contract/deadline modules are imported only after explicit --execute.
This source is not an executable release or a completed parent integration.
"""
import argparse
import json
import math
from pathlib import Path
import signal
import time

SCOPE = 'MAPPING_SEMANTIC_LATERAL_BOUNDED_AMPLITUDE_SUSTAINED_CALIBRATION_ONLY'
TARGET = ('127.0.0.1', 7447)
PRIVATE_TARGET = ('192.168.1.219', 7447)
PERIOD_NS = 50_000_000
ALLOWED_MAGNITUDES = (.2, .4, .6)
MIN_ACTIVE_NS = 1_500_000_000
MAX_ACTIVE_NS = 4_000_000_000
MAX_MOTION_PACKETS = 80
FINAL_ZERO_PACKETS = 5
SOCKET_TIMEOUT_SEC = .05


class CanaryHold(ValueError):
    pass


class CanaryInterrupted(Exception):
    def __init__(self, number):
        self.number = number
        super().__init__('canary interrupted by signal ' + str(number))


def frame(sign, neutral=False):
    if type(sign) not in (int, float) or not math.isfinite(sign) or abs(sign) not in ALLOWED_MAGNITUDES:
        raise CanaryHold('only one selected signed semantic magnitude from0.2/0.4/0.6')
    return dict(version=1, action='walk', forward=0., lateral=0. if neutral else sign, yaw=0.)


def permitted(command, sign):
    if (not isinstance(command, dict) or set(command) != {'version', 'action', 'forward', 'lateral', 'yaw'}
            or type(command['version']) is not int or command['version'] != 1 or command['action'] != 'walk'
            or any(type(command[k]) not in (int, float) or not math.isfinite(command[k]) for k in ('forward', 'lateral', 'yaw'))
            or command['forward'] != 0 or command['yaw'] != 0 or command['lateral'] not in (0., sign)):
        raise CanaryHold('only fixed version1/walk zero-forward/yaw and selected lateral or zero')
    frame(sign)  # Validate the selected sign too; no bool, nonfinite or arbitrary magnitude.


def selected_target(value):
    if (not isinstance(value,(tuple,list)) or len(value)!=2 or type(value[0]) is not str
            or type(value[1]) is not int or tuple(value) not in (TARGET,PRIVATE_TARGET)):
        raise CanaryHold('only exact reviewed legacy/private IPv4:7447 target')
    return tuple(value)


def send_frame(sock, command, sign, stream, clock, target=TARGET):
    target=selected_target(target)
    permitted(command, sign)
    payload = json.dumps(command, separators=(',', ':'), allow_nan=False).encode()
    before_unix, before_steady = clock.time_ns(), clock.monotonic_ns()
    count, error = None, None
    try:
        count = sock.sendto(payload, target)
        if count != len(payload):
            raise CanaryHold('short UDP send')
    except BaseException as exc:
        error = exc
    after_steady, after_unix = clock.monotonic_ns(), clock.time_ns()
    import hashlib
    row = dict(event='actual_socket_sendto_interval_not_actuation' if error is None else 'socket_sendto_failed_or_short_not_actuation',
        send_before_unix_ns=before_unix, send_after_unix_ns=after_unix,
        send_before_steady_ns=before_steady, send_after_steady_ns=after_steady,
        target=list(target), bytes=count, payload_sha256=hashlib.sha256(payload).hexdigest(),
        command=command, send_success=error is None, actuation_proven=False)
    if error is not None:
        row.update(error_type=type(error).__name__, error=str(error))
    stream.write(json.dumps(row, allow_nan=False) + '\n')
    stream.flush()
    if error is not None:
        raise error
    return row


def prove_parent(case, release, ready, output, contract, deadline, clock=time):
    contract.host_proof(release, case, now_ns=clock.time_ns())
    for role in ('parent_identity', 'recorder_identity', 'helper_identity'):
        identity = ready[role]
        if deadline.process_identity(identity['pid']) != identity:
            raise CanaryHold('bound parent/recorder/helper identity changed: ' + role)
    lease = contract.read(output/'observation_lease.json')
    now, steady = clock.time_ns(), clock.monotonic_ns()
    if (lease.get('release_id') != release['release_id'] or lease.get('parent_identity') != ready['parent_identity']
            or not 0 <= (steady - lease['written_steady_ns'])/1e9 <= .15
            or not isinstance(lease.get('anchor'), list) or len(lease['anchor']) != 3):
        raise CanaryHold('fresh matching parent lease <=150ms required')
    contract.check_live(lease['samples'], lease['anchor'], now, steady)
    if clock.clock_gettime(time.CLOCK_BOOTTIME) >= ready['deadline_boottime_sec'] - 10.:
        raise CanaryHold('insufficient capped recorder time')


def run_probe(sign, duration_sec, socket_factory, prove, stream, clock=time, signal_api=signal, target=TARGET):
    """One socket. All clocks/sockets/proofs/signals can be replaced by pure fakes."""
    frame(sign)
    target=selected_target(target)
    if type(duration_sec) not in (int, float) or not math.isfinite(duration_sec) or not MIN_ACTIVE_NS/1e9 <= duration_sec <= MAX_ACTIVE_NS/1e9:
        raise CanaryHold('active duration must be finite in [1.5,4] seconds')
    duration_ns = round(duration_sec*1e9)
    if duration_ns <= 0:
        raise CanaryHold('active duration below nanosecond resolution')
    state = dict(stopping=False, signals=[])
    result = dict(scope=SCOPE, direct_bounded_backend=True, relay_watchdog_sec=None,
        engine_timeout_sec_observed_in_prior_evidence=.5, hard_plant_guarantee=False,
        sign=sign, requested_active_sec=duration_sec, maximum_motion_packets=MAX_MOTION_PACKETS,
        motion_packets_sent=0, per_motion_proof_checks=0,
        final_zero_attempts=0, final_zero_logged_successes=0, close_returned=False,
        errors=[], completed=False)
    sock, previous = None, {}
    def interrupted(number, unused):
        state['signals'].append(number)
        if not state['stopping']:
            raise CanaryInterrupted(number)
    try:
        for number in (signal_api.SIGINT, signal_api.SIGTERM):
            previous[number] = signal_api.signal(number, interrupted)
        prove()  # No socket is created if the initial proof fails.
        sock = socket_factory()
        sock.settimeout(SOCKET_TIMEOUT_SEC)
        start = clock.monotonic_ns()
        end, due = start + duration_ns, start
        result.update(active_start_steady_ns=start, active_deadline_steady_ns=end)
        while result['motion_packets_sent'] < min(MAX_MOTION_PACKETS, math.ceil(duration_ns/PERIOD_NS)):
            now = clock.monotonic_ns()
            if now >= end:
                break
            if now < due:
                clock.sleep((min(due, end) - now) / 1e9)
            if clock.monotonic_ns() >= end:
                break
            prove()
            result['per_motion_proof_checks'] += 1
            if clock.monotonic_ns() >= end:
                break  # A slow proof never permits a late nonzero send.
            row = send_frame(sock, frame(sign), sign, stream, clock,target)
            result['motion_packets_sent'] += 1
            if row['send_after_steady_ns'] > end:
                raise CanaryHold('nonzero send syscall crossed active deadline')
            due = row['send_after_steady_ns'] + PERIOD_NS  # No catch-up bursts.
        remaining = end - clock.monotonic_ns()
        if remaining > 0:
            clock.sleep(remaining/1e9)
    except BaseException as exc:
        result['errors'].append(dict(stage='active', type=type(exc).__name__, message=str(exc)))
    finally:
        state['stopping'] = True  # Repeated handled signals cannot abort zero cleanup.
        if sock is not None:
            for index in range(FINAL_ZERO_PACKETS):
                result['final_zero_attempts'] += 1
                try:
                    send_frame(sock, frame(sign, neutral=True), sign, stream, clock,target)
                    result['final_zero_logged_successes'] += 1
                except BaseException as exc:
                    result['errors'].append(dict(stage='zero', index=index, type=type(exc).__name__, message=str(exc)))
                if index + 1 < FINAL_ZERO_PACKETS:
                    try:
                        clock.sleep(PERIOD_NS/1e9)
                    except BaseException as exc:
                        result['errors'].append(dict(stage='zero_wait', type=type(exc).__name__, message=str(exc)))
            try:
                sock.close()
                result['close_returned'] = True
            except BaseException as exc:
                result['errors'].append(dict(stage='close', type=type(exc).__name__, message=str(exc)))
        for number, handler in previous.items():
            signal_api.signal(number, handler)
    result['signals_observed'] = state['signals']
    result['completed'] = (not result['errors'] and result['motion_packets_sent'] > 0
        and result['final_zero_logged_successes'] == FINAL_ZERO_PACKETS and result['close_returned'])
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--case', type=Path)
    parser.add_argument('--root-release', type=Path)
    parser.add_argument('--ready', type=Path)
    parser.add_argument('--ready-sha256')
    parser.add_argument('--probe-index', type=int, choices=[0, 1])
    args = parser.parse_args(argv)
    if not args.execute:
        print(json.dumps(dict(state='ROOT_HOLD', scope=SCOPE, no_socket_created=True,
            no_parent_contract_imported=True, backend_only_not_integrated=True)))
        return 0
    if any(getattr(args, key) is None for key in ('case', 'root_release', 'ready', 'ready_sha256', 'probe_index')):
        parser.error('explicit case/release/ready hash and probe index required')
    import lateral_contract as contract
    import recording_deadline as deadline
    if (contract.SCOPE != SCOPE or Path(__file__).name not in contract.PACKAGE
            or contract.HERE.resolve() != Path(__file__).resolve().parent):
        raise CanaryHold('backend requires an independently frozen semantic parent/contract package')
    case, release = contract.read(args.case), contract.read(args.root_release)
    output = Path(release['output_directory'])
    if case.get('scope') != SCOPE or release.get('scope') != SCOPE:
        raise CanaryHold('semantic canary scope required')
    contract.validate(case, release, args.case, output)
    target=contract.command_endpoint(case,release)
    contract.patch_check(case)  # Original .89 body +1m engineering stop reserve, no new geometry.
    contract.probe_parameters(case)  # Exact paired sign/magnitude/duration case binding.
    if case.get('max_runtime_sec') != 60.:
        raise CanaryHold('only the reviewed bounded-amplitude paired case/60s-recorder')
    ready = contract.read(args.ready)
    if (args.ready != output/'recorder_ready.json' or contract.sha(args.ready) != args.ready_sha256
            or ready.get('release_id') != release['release_id'] or ready.get('mcap_magic_verified') is not True
            or ready.get('deadline_ack_verified') is not True):
        raise CanaryHold('bound complete recorder readiness missing')
    if not (output/f'probe_{args.probe_index}.planned.json').is_file():
        raise CanaryHold('parent probe plan absent')
    prove = lambda: prove_parent(case, release, ready, output, contract, deadline)
    prove()
    contract.claim(output/f'probe_{args.probe_index}.tool_once.json', dict(release_id=release['release_id'],
        probe=args.probe_index, identity=deadline.process_identity(__import__('os').getpid()), claimed_unix_ns=time.time_ns(),
        backend='direct_semantic_no_relay_watchdog'))
    def socket_factory():
        import socket
        return socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    with (output/f'probe_{args.probe_index}_send.jsonl').open('x') as stream:
        result = run_probe(case['probes'][args.probe_index], case['duration_sec'], socket_factory, prove, stream,target=target)
    contract.claim(output/f'probe_{args.probe_index}.backend_result.json', result)
    print(json.dumps(result, allow_nan=False))
    return 0 if result['completed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())

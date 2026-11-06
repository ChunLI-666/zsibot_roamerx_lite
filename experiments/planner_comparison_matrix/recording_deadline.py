#!/usr/bin/env python3
"""Detached, Linux pidfd-bound deadline for one experiment launcher; no ROS imports."""

import argparse
import hashlib
import json
import math
import os
import pathlib
import select
import signal
import subprocess
import sys
import time
import uuid


MAX_RUNTIME_SEC = 3600.0
MANIFEST = 'recording_deadline.json'
ARMED = 'recording_deadline_armed.json'
SHUTDOWN = 'recording_deadline_shutdown.json'
EVENTS = 'recording_deadline_events.jsonl'
CHILDREN = 'recording_deadline_children.json'
CHILDREN_ARMED = 'recording_deadline_children_armed.json'
OWNED_NAMES = ('recorder', 'nav_safety_gate', 'map_server', 'planner_server',
               'controller_server', 'velocity_smoother', 'bt_navigator', 'lifecycle_manager')
RUN_ID_ENV = 'MATRIX_RECORDING_RUN_ID'
OUTPUT_ENV = 'MATRIX_RECORDING_OUTPUT'
ROLE_ENV = 'MATRIX_RECORDING_ROLE'
LAUNCH_ENV = 'MATRIX_RECORDING_LAUNCH_ID'


def runtime_seconds(value):
    seconds = float(value)
    if not math.isfinite(seconds) or not 0 < seconds <= MAX_RUNTIME_SEC:
        raise argparse.ArgumentTypeError('max runtime must be finite and in (0, 3600] seconds')
    return seconds


def atomic_json(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2) + '\n')
    temporary.replace(path)


def process_identity(pid):
    process = pathlib.Path('/proc') / str(pid)
    # comm may contain spaces and closing parentheses; fields after its final ')' are stable.
    fields = (process / 'stat').read_text().rsplit(')', 1)[1].split()
    command = (process / 'cmdline').read_bytes()
    if not command or fields[0] == 'Z':
        raise ProcessLookupError('launcher has exited')
    return dict(pid=pid, starttime_ticks=int(fields[19]),
                cmdline_sha256=hashlib.sha256(command).hexdigest(),
                argv=[part.decode(errors='backslashreplace') for part in command.rstrip(b'\0').split(b'\0')],
                boot_id=pathlib.Path('/proc/sys/kernel/random/boot_id').read_text().strip())


def command_argv(argv):
    """Predict Linux exec argv, accepting native executables or one direct ELF shebang."""
    if not argv or any(not isinstance(arg, str) or '\0' in arg for arg in argv):
        raise ValueError('owned command requires NUL-free string argv')
    program = pathlib.Path(argv[0])
    if not program.is_absolute():
        raise ValueError('owned executable must be an absolute path')
    with program.open('rb') as stream:
        first = stream.readline(256)
    if first.startswith(b'#!'):
        if not first.endswith(b'\n'):
            raise ValueError('ambiguous or overlong executable shebang')
        parts = first[2:].strip().split(maxsplit=1)
        interpreter = pathlib.Path(os.fsdecode(parts[0]))
        if not interpreter.is_absolute():
            raise ValueError('owned script interpreter must be absolute')
        with interpreter.open('rb') as stream:
            header = stream.read(4)
        if header != b'\x7fELF':
            raise ValueError('owned script needs a direct ELF interpreter, without env/nested scripts')
        if interpreter.name == 'env':
            raise ValueError('env shebang is not an exact executable contract')
        return [os.fsdecode(part) for part in parts] + list(argv)
    if not first.startswith(b'\x7fELF'):
        raise ValueError('owned command must be ELF or have a supported direct shebang')
    return list(argv)


def identity_after_exec(identity, argv):
    command = b'\0'.join(os.fsencode(arg) for arg in argv) + b'\0'
    return dict(identity, argv=[part.decode(errors='backslashreplace') for part in command.rstrip(b'\0').split(b'\0')],
                cmdline_sha256=hashlib.sha256(command).hexdigest())


def child_path(output, name, kind):
    if name not in OWNED_NAMES:
        raise ValueError('child role is outside the exact navigation/recorder allowlist')
    return output / ('recording_deadline_' + name + '_' + kind + '.json')


def check_child_binding(identity, run_id, output, role=None, launch_id=None):
    if process_identity(identity['pid']) != identity:
        raise ValueError('owned process PID/starttime/cmdline/boot identity mismatch')
    environment = (pathlib.Path('/proc') / str(identity['pid']) / 'environ').read_bytes().split(b'\0')
    if (os.fsencode(RUN_ID_ENV + '=' + run_id) not in environment
            or os.fsencode(OUTPUT_ENV + '=' + str(output)) not in environment):
        raise ValueError('owned process does not carry this run/output identity')
    for key, value in ((ROLE_ENV, role), (LAUNCH_ENV, launch_id)):
        if value is not None and os.fsencode(key + '=' + value) not in environment:
            raise ValueError('owned process role/launch identity mismatch')


class DeadlineGuard:
    def __init__(self, output, manifest, digest, process):
        self.output, self.manifest, self.digest, self.process = output, manifest, digest, process
        self.closing = False
        self.runtime_children_started = False
        self.children = {}

    def check_live(self):
        if not self.closing and self.process.poll() is not None:
            raise RuntimeError('Independent recording deadline guard exited; refusing to continue')

    def shutdown_stage(self, stage, reason):
        self.closing = stage == 'recording_closed'
        atomic_json(self.output / SHUTDOWN, dict(run_id=self.manifest['run_id'], manifest_sha256=self.digest,
                                               stage=stage, reason=reason, time_ns=time.time_ns(),
                                               boottime_sec=time.clock_gettime(time.CLOCK_BOOTTIME)))

    def begin_shutdown(self, reason):
        self.shutdown_stage('shutdown_started', reason)

    def recording_closed(self, reason):
        self.shutdown_stage('recording_closed', reason)

    def owned_command(self, name, argv, environment):
        if name not in OWNED_NAMES or name in self.children:
            raise ValueError('Unexpected or repeated owned child: ' + name)
        plan = dict(run_id=self.manifest['run_id'], manifest_sha256=self.digest,
                    output_directory=str(self.output), role=name, launch_id=uuid.uuid4().hex,
                    argv=list(argv), executed_argv=command_argv(argv))
        path = child_path(self.output, name, 'plan')
        command = [sys.executable, '-I', str(self.output / pathlib.Path(__file__).name), '--owned-exec', str(path)]
        plan['wrapper_argv'] = command
        atomic_json(path, plan)
        self.children[name] = plan
        environment = dict(environment, **{RUN_ID_ENV: plan['run_id'], OUTPUT_ENV: str(self.output),
                                          ROLE_ENV: name, LAUNCH_ENV: plan['launch_id']})
        return command, environment

    def wait_for_child(self, name, pid):
        plan = self.children[name]
        until = time.monotonic() + 3.0
        while time.monotonic() < until:
            self.check_live()
            acknowledgement = self.output / CHILDREN_ARMED
            if acknowledgement.exists():
                observed = json.loads(acknowledgement.read_text())
                if (observed.get('run_id') == self.manifest['run_id'] and observed.get('manifest_sha256') == self.digest
                        and observed.get('registrations', {}).get(name, {}).get('launch_id') == plan['launch_id']
                        and observed['registrations'][name]['identity']['pid'] == pid
                        and name in observed.get('exec_verified', [])):
                    return
            time.sleep(.02)
        raise TimeoutError('Independent recording deadline did not retain child pidfd: ' + name)


def arm(output, seconds, shutdown_grace_sec=15.0):
    """Arm before any runtime child; fail closed unless the detached helper acknowledges."""
    seconds = runtime_seconds(seconds)
    if not math.isfinite(shutdown_grace_sec) or not 0 < shutdown_grace_sec <= 15.0:
        raise ValueError('shutdown grace must be in (0, 15] seconds')
    output = pathlib.Path(output).resolve()
    identity = process_identity(os.getpid())
    tick_rate = os.sysconf('SC_CLK_TCK')
    started = identity['starttime_ticks'] / tick_rate
    manifest = dict(schema_version=1, run_id=uuid.uuid4().hex, output_directory=str(output),
                    launcher=identity, clock='CLOCK_BOOTTIME; /proc starttime includes launcher startup',
                    clock_ticks_per_second=tick_rate, launcher_started_boottime_sec=started,
                    max_runtime_sec=seconds, deadline_boottime_sec=started + seconds,
                    created_time_ns=time.time_ns(), signal='SIGINT', launcher_signal_limit=1,
                    shutdown_grace_sec=shutdown_grace_sec, owned_child_signal_limit=1,
                    policy='Launcher SIGINT, then bounded grace and exact registered-owned-child SIGINT fallback; no process groups or escalation')
    manifest_path = output / MANIFEST
    atomic_json(manifest_path, manifest)
    digest = hashlib.sha256(manifest_path.read_bytes()).hexdigest()
    # Execute the per-run frozen helper with an isolated Python import path and no terminal/pipes.
    process = subprocess.Popen([sys.executable, '-I', str(output / pathlib.Path(__file__).name),
                                '--watch', str(manifest_path), '--manifest-sha256', digest,
                                '--run-id', manifest['run_id']], cwd='/', stdin=subprocess.DEVNULL,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                               env={'PATH':'/usr/bin:/bin', 'LANG':'C.UTF-8'},
                               start_new_session=True, close_fds=True)
    guard = DeadlineGuard(output, manifest, digest, process)
    try:
        until = time.monotonic() + 5.0
        while time.monotonic() < until:
            guard.check_live()
            acknowledgement = output / ARMED
            if acknowledgement.exists():
                observed = json.loads(acknowledgement.read_text())
                if (observed['run_id'] != manifest['run_id'] or observed['manifest_sha256'] != digest
                        or observed['watchdog']['pid'] != process.pid):
                    raise RuntimeError('Recording deadline acknowledgement identity mismatch')
                return guard
            time.sleep(0.02)
        raise TimeoutError('Independent recording deadline did not acknowledge arming')
    except BaseException:
        guard.recording_closed('arming_failed_before_runtime')
        process.wait(timeout=2.0)
        raise


def owned_exec(plan_path):
    """Self-register the dormant wrapper; exec only after independent pidfd acknowledgement."""
    output = plan_path.parent.resolve()
    raw = plan_path.read_bytes()
    plan = json.loads(raw)
    name = plan['role']
    if plan_path != child_path(output, name, 'plan') or plan['output_directory'] != str(output):
        raise ValueError('owned exec plan path/output identity mismatch')
    manifest_path = output / MANIFEST
    manifest_raw = manifest_path.read_bytes()
    if hashlib.sha256(manifest_raw).hexdigest() != plan['manifest_sha256']:
        raise ValueError('owned exec run manifest changed')
    manifest = json.loads(manifest_raw)
    if manifest['run_id'] != plan['run_id'] or command_argv(plan['argv']) != plan['executed_argv']:
        raise ValueError('owned exec run/command contract mismatch')
    identity = process_identity(os.getpid())
    if identity_after_exec(identity, plan['wrapper_argv']) != identity:
        raise ValueError('owned exec wrapper command identity mismatch')
    check_child_binding(identity, plan['run_id'], output, name, plan['launch_id'])
    registration = dict(run_id=plan['run_id'], manifest_sha256=plan['manifest_sha256'],
                        role=name, launch_id=plan['launch_id'], plan_sha256=hashlib.sha256(raw).hexdigest(),
                        wrapper_identity=identity, identity=identity_after_exec(identity, plan['executed_argv']))
    armed = json.loads((output / ARMED).read_text())
    watchdog_fd = os.pidfd_open(armed['watchdog']['pid'])
    try:
        if (armed['run_id'] != plan['run_id'] or armed['manifest_sha256'] != plan['manifest_sha256']
                or process_identity(armed['watchdog']['pid']) != armed['watchdog']):
            raise ValueError('owned exec watchdog identity mismatch')
        atomic_json(child_path(output, name, 'registration'), registration)
        while time.clock_gettime(time.CLOCK_BOOTTIME) < manifest['deadline_boottime_sec']:
            if select.select([watchdog_fd], [], [], 0)[0]:
                return 2  # Guard has exited: the real executable must never start.
            if manifest_path.read_bytes() != manifest_raw or plan_path.read_bytes() != raw:
                raise ValueError('owned exec immutable run/plan changed')
            acknowledgement = output / CHILDREN_ARMED
            if acknowledgement.exists():
                observed = json.loads(acknowledgement.read_text())
                if (observed.get('run_id') == plan['run_id']
                        and observed.get('manifest_sha256') == plan['manifest_sha256']
                        and observed.get('registrations', {}).get(name) == registration):
                    if command_argv(plan['argv']) != plan['executed_argv']:
                        raise ValueError('owned executable shebang changed before exec')
                    if (time.clock_gettime(time.CLOCK_BOOTTIME) >= manifest['deadline_boottime_sec']
                            or select.select([watchdog_fd], [], [], 0)[0]):
                        return 2
                    # Same PID/starttime/run binding across exec; watchdog retains the pidfd.
                    os.execve(plan['argv'][0], plan['argv'], os.environ)
            time.sleep(.02)
        return 2  # A dormant wrapper cannot start a new producer after the fixed deadline.
    finally:
        os.close(watchdog_fd)


def watch(manifest_path, digest, run_id):
    output = manifest_path.parent.resolve()
    with (output / EVENTS).open('a', buffering=1) as events:
        def event(kind, **details):
            events.write(json.dumps(dict(kind=kind, run_id=run_id, time_ns=time.time_ns(),
                                         boottime_sec=time.clock_gettime(time.CLOCK_BOOTTIME),
                                         **details)) + '\n')

        descriptor = None
        signal_sent = False
        children = {}
        try:
            raw = manifest_path.read_bytes()
            if hashlib.sha256(raw).hexdigest() != digest:
                raise ValueError('run manifest hash mismatch')
            manifest = json.loads(raw)
            if manifest['run_id'] != run_id or manifest['output_directory'] != str(output):
                raise ValueError('run identity mismatch')
            runtime_seconds(manifest['max_runtime_sec'])
            expected = manifest['launcher']
            if expected['pid'] <= 1 or expected['pid'] == os.getpid():
                raise ValueError('invalid launcher PID')
            if not callable(getattr(signal, 'pidfd_send_signal', None)):
                raise RuntimeError('Python/Linux pidfd_send_signal support is required')
            # Open before checking /proc; keep the descriptor until the single signal is sent.
            # Reuse after the check can never redirect pidfd_send_signal to a different process.
            descriptor = os.pidfd_open(expected['pid'])
            signal.pidfd_send_signal(descriptor, 0)  # Validate kernel support and permission before arming.
            poller = select.poll()
            poller.register(descriptor, select.POLLIN)
            registry_raw = {}
            exec_verified = set()
            observed_stage = None

            def exited(fd):
                return bool(select.select([fd], [], [], 0)[0])

            def live_children():
                return [name for name, (fd, _) in children.items() if not exited(fd)]

            def register_children():
                changed = False
                for name in OWNED_NAMES:
                    path = child_path(output, name, 'registration')
                    if not path.exists():
                        continue
                    current = path.read_bytes()
                    if current == registry_raw.get(name):
                        continue
                    registry_raw[name] = current
                    child_fd = None
                    try:
                        registration = json.loads(current)
                        if (registration['run_id'] != run_id or registration['manifest_sha256'] != digest
                                or registration['role'] != name):
                            raise ValueError('owned registration run/role/manifest identity mismatch')
                        if name in children:
                            if children[name][1] != registration:
                                raise ValueError('registered identity cannot be replaced')
                            continue
                        plan_raw = child_path(output, name, 'plan').read_bytes()
                        if hashlib.sha256(plan_raw).hexdigest() != registration['plan_sha256']:
                            raise ValueError('owned command plan hash mismatch')
                        plan = json.loads(plan_raw)
                        if (plan['run_id'] != run_id or plan['manifest_sha256'] != digest
                                or plan['role'] != name or plan['output_directory'] != str(output)
                                or plan['launch_id'] != registration['launch_id']
                                or command_argv(plan['argv']) != plan['executed_argv']):
                            raise ValueError('owned command plan/exec contract mismatch')
                        identity = registration['wrapper_identity']
                        if identity['pid'] <= 1 or identity['pid'] in (expected['pid'], os.getpid()):
                            raise ValueError('invalid owned child PID')
                        if (identity_after_exec(identity, plan['wrapper_argv']) != identity
                                or identity_after_exec(identity, plan['executed_argv']) != registration['identity']):
                            raise ValueError('owned wrapper/final command contract mismatch')
                        child_fd = os.pidfd_open(identity['pid'])
                        signal.pidfd_send_signal(child_fd, 0)
                        check_child_binding(identity, run_id, output, name, plan['launch_id'])
                        children[name] = (child_fd, registration)
                        child_fd = None
                        changed = True
                        event('child_pidfd_retained', name=name, wrapper_identity=identity,
                              planned_exec_identity=registration['identity'])
                    except Exception as error:
                        event('child_registration_rejected', name=name, error=repr(error))
                    finally:
                        if child_fd is not None:
                            os.close(child_fd)
                for name, (child_fd, registration) in children.items():
                    if name in exec_verified or exited(child_fd):
                        continue
                    try:
                        check_child_binding(registration['identity'], run_id, output, name, registration['launch_id'])
                        exec_verified.add(name)
                        changed = True
                        event('child_exec_identity_verified', name=name, identity=registration['identity'])
                    except (FileNotFoundError, ProcessLookupError, ValueError):
                        pass  # The acknowledged wrapper may still be pending its controlled exec.
                if changed:
                    records = {name:registration for name, (_, registration) in children.items()}
                    registry = dict(run_id=run_id, manifest_sha256=digest, registrations=records,
                                    exec_verified=sorted(exec_verified))
                    atomic_json(output / CHILDREN, registry)
                    atomic_json(output / CHILDREN_ARMED, registry)

            def state():
                nonlocal observed_stage
                if manifest_path.read_bytes() != raw:
                    event('identity_mismatch', reason='run_manifest_changed')
                    return 'rejected'
                marker = output / SHUTDOWN
                if marker.exists():
                    shutdown = json.loads(marker.read_text())
                    if shutdown.get('run_id') != run_id or shutdown.get('manifest_sha256') != digest:
                        event('identity_mismatch', reason='shutdown_marker_run_identity_changed')
                        return 'rejected'
                    stage = shutdown.get('stage')
                    if stage not in ('shutdown_started', 'recording_closed'):
                        raise ValueError('unknown shutdown stage')
                    if stage != observed_stage:
                        event('shutdown_observed', stage=stage, reason=shutdown['reason'],
                              live_owned_children=live_children())
                        observed_stage = stage
                    if stage == 'recording_closed' and not live_children():
                        return 'closed'
                if poller.poll(0):
                    return 'exited'
                try:
                    observed = process_identity(expected['pid'])
                except (FileNotFoundError, ProcessLookupError):
                    return 'exited'
                if observed != expected:
                    event('identity_mismatch', reason='launcher_pid_starttime_cmdline_or_boot_changed')
                    return 'rejected'
                return 'live'

            def fallback(reason):
                register_children()
                event('fallback_started', reason=reason, live_owned_children=live_children())

                def stop_phase(names, phase):
                    event('fallback_phase_started', phase=phase)
                    for name in names:
                        if name not in children:
                            continue
                        child_fd, registration = children[name]
                        if exited(child_fd):
                            continue
                        try:
                            # Recheck every stable identity and the initial run binding at send time.
                            if hashlib.sha256(child_path(output, name, 'plan').read_bytes()).hexdigest() != registration['plan_sha256']:
                                raise ValueError('owned command plan changed before signal')
                            observed = process_identity(registration['identity']['pid'])
                            if observed not in (registration['wrapper_identity'], registration['identity']):
                                raise ValueError('owned process changed outside controlled wrapper-to-exec command contract')
                            check_child_binding(observed, run_id, output, name, registration['launch_id'])
                            signal.pidfd_send_signal(child_fd, signal.SIGINT)
                            event('fallback_child_sigint_sent', name=name, launcher_pid=expected['pid'],
                                  child_pid=observed['pid'], phase=phase, signal_count=1)
                        except ProcessLookupError:
                            event('fallback_child_already_exited', name=name)
                        except Exception as error:
                            event('fallback_child_identity_rejected', name=name, error=repr(error))

                stop_phase(('bt_navigator', 'controller_server'), 'command_producers')
                time.sleep(1.0)
                stop_phase(tuple(name for name in reversed(OWNED_NAMES)
                                 if name not in ('recorder', 'bt_navigator', 'controller_server')), 'remaining_navigation')
                time.sleep(.5)
                stop_phase(('recorder',), 'recorder')
                until = time.monotonic() + 30.
                while live_children() and time.monotonic() < until:
                    time.sleep(.05)
                remaining = live_children()
                event('fallback_completed', all_registered_owned_processes_exited=not remaining,
                      remaining_owned_children=remaining,
                      note='Process exit only; bag metadata/readability audit is separate. No escalation or repeated SIGINT.')
                return 1 if remaining else 0

            register_children()
            if state() != 'live':
                return 0
            acknowledgement = dict(run_id=run_id, manifest_sha256=digest,
                                   watchdog=process_identity(os.getpid()), watchdog_session_id=os.getsid(0),
                                   watchdog_process_group_id=os.getpgrp())
            event('armed', launcher=expected, deadline_boottime_sec=manifest['deadline_boottime_sec'])
            atomic_json(output / ARMED, acknowledgement)
            grace_deadline = None
            while True:
                register_children()
                status = state()
                if status in ('closed', 'rejected'):
                    return 0 if status == 'closed' else 1
                if status == 'exited':
                    event('launcher_exited', reason='pidfd_readable', live_owned_children=live_children())
                    return fallback('launcher_exited_before_recording_closed') if live_children() else 0
                remaining = manifest['deadline_boottime_sec'] - time.clock_gettime(time.CLOCK_BOOTTIME)
                if not signal_sent and remaining <= 0:
                    # Recheck the run/cleanup identity immediately before the pidfd-bound signal.
                    if state() == 'live':
                        event('deadline_reached', reason='fixed_launcher_runtime_budget_exhausted',
                              max_runtime_sec=manifest['max_runtime_sec'])
                        signal.pidfd_send_signal(descriptor, signal.SIGINT)
                        signal_sent = True
                        event('sigint_sent', launcher_pid=expected['pid'], signal_count=1,
                              reason='fixed_launcher_deadline', shutdown_grace_sec=manifest.get('shutdown_grace_sec', 15.))
                        grace_deadline = time.clock_gettime(time.CLOCK_BOOTTIME) + manifest.get('shutdown_grace_sec', 15.)
                if signal_sent:
                    if not live_children():
                        event('registered_owned_processes_closed_after_sigint')
                        return 0
                    if time.clock_gettime(time.CLOCK_BOOTTIME) >= grace_deadline:
                        return fallback('launcher_shutdown_grace_exhausted')
                poller.poll(100)
        except ProcessLookupError:
            event('launcher_exited', reason='process_disappeared_before_signal')
            return fallback('launcher_exited_at_signal') if children else 0
        except Exception as error:
            event('guard_failed', error=repr(error), signal_sent=signal_sent)
            return 1
        finally:
            if descriptor is not None:
                os.close(descriptor)
            for child_fd, _ in children.values():
                os.close(child_fd)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--watch', type=pathlib.Path)
    mode.add_argument('--owned-exec', type=pathlib.Path)
    parser.add_argument('--manifest-sha256')
    parser.add_argument('--run-id')
    args = parser.parse_args()
    if args.owned_exec:
        return owned_exec(args.owned_exec)
    if not args.manifest_sha256 or not args.run_id:
        parser.error('--watch requires --manifest-sha256 and --run-id')
    return watch(args.watch, args.manifest_sha256, args.run_id)


if __name__ == '__main__':
    sys.exit(main())

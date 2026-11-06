#!/usr/bin/python3
"""Default-HOLD recorder-only parent for a separately released full-scene controller.

No ROS imports, Node, clock/TF/velocity publisher, controller spawn or controller
signal. The unchanged detached deadline helper registers only our recorder.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import select
import shutil
import signal
import stat
import subprocess
import sys
import time

import bag_summary
import mapping_result_adapter
import recording_deadline as deadline

HERE = Path(__file__).resolve().parent
SCOPE = 'RAW_RECORDING_ONLY'
TAIL_SEC = 20.
CLOSE_RESERVE_SEC = 15.
CLOSE_WAIT_SEC = 45.
PACKAGE = ('full_scene_recording_owner.py', 'mapping_result_adapter.py', 'bag_summary.py', 'recording_deadline.py')
REQUIRED_TOPICS = {'/front_lidar', '/front_lidar/imu', '/odom', '/front_camera/image/compressed',
    '/livox/lidar', '/livox/imu', '/laser_scan', '/camera/front/image/compressed', '/camera/front/camera_info',
    '/tf_static', '/clock', '/experiment/full_scene_mapping/events'}
REQUIRED_ARTIFACT_ROLES = {'network_environment', 'message_environment', 'scene_binding', 'sensor_config',
    'scene_inventory', 'route', 'oracle', 'controller_script', 'controller_contract', 'relay_script',
    'waypoint_contract', 'map_coverage'}
ENV_REQUIRED = dict(ROS_DOMAIN_ID='185', RMW_IMPLEMENTATION='rmw_fastrtps_cpp', ROS_LOCALHOST_ONLY='0',
                    ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
ENV_BINDING = tuple(ENV_REQUIRED) + ('AMENT_PREFIX_PATH', 'LD_LIBRARY_PATH', 'PYTHONPATH',
    'FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE')


class Hold(ValueError):
    pass


def sha(path):
    return bag_summary.sha(path)


def read_json(path):
    if Path(path).stat().st_size > 16*1024*1024:
        raise Hold('bounded JSON input exceeded')
    def unique(pairs):
        value = {}
        for key, item in pairs:
            if key in value:
                raise Hold('duplicate JSON key: ' + key)
            value[key] = item
        return value
    return json.loads(Path(path).read_text(), object_pairs_hook=unique,
                      parse_constant=lambda value: (_ for _ in ()).throw(Hold('nonfinite JSON: ' + value)))


def write(path, value):
    deadline.atomic_json(Path(path), value)


def uuid32(value):
    return isinstance(value, str) and len(value) == 32 and all(c in '0123456789abcdef' for c in value)


def claim_once(path, value):
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(descriptor, 'w') as stream:
        stream.write(json.dumps(value, indent=2) + '\n')
        stream.flush()
        os.fsync(stream.fileno())


def identity_live(expected, environment=None):
    if deadline.process_identity(expected['pid']) != expected:
        raise Hold('PID/starttime/command/boot identity changed')
    if environment:
        actual = (Path('/proc') / str(expected['pid']) / 'environ').read_bytes().split(b'\0')
        if any(os.fsencode(key + '=' + value) not in actual for key, value in environment.items()):
            raise Hold('bound controller run/output environment differs')


def export_file(root, relative, contents=False):
    """Read a regular export leaf through no-follow directory descriptors."""
    path = Path(relative) if isinstance(relative, str) else None
    if (path is None or not relative or path.is_absolute() or relative != path.as_posix()
            or any(part in ('', '.', '..') for part in relative.split('/')) or '\0' in relative):
        raise Hold('source export requires normalized relative paths without dot/escape components')
    descriptor = os.open(root, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
    try:
        for part in path.parts[:-1]:
            child = os.open(part, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=descriptor)
            os.close(descriptor)
            descriptor = child
        leaf = os.open(path.parts[-1], os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK, dir_fd=descriptor)
        with os.fdopen(leaf, 'rb') as stream:
            before = os.fstat(stream.fileno())
            if not stat.S_ISREG(before.st_mode) or (contents and before.st_size > 16*1024*1024):
                raise Hold('source export leaf must be a regular file and manifest bounded')
            digest, chunks = hashlib.sha256(), []
            for chunk in iter(lambda: stream.read(1024*1024), b''):
                digest.update(chunk)
                if contents:
                    chunks.append(chunk)
            after = os.fstat(stream.fileno())
            if (before.st_ino, before.st_size, before.st_mtime_ns) != (after.st_ino, after.st_size, after.st_mtime_ns):
                raise Hold('source export file changed while hashing')
            return dict(relative_path=relative, sha256=digest.hexdigest(), bytes=after.st_size), b''.join(chunks)
    finally:
        os.close(descriptor)


def verify_frozen_source_export(repository, artifacts):
    """Verify canonical-origin metadata and actual source bytes; never run Git."""
    def hexadecimal(value, length):
        return isinstance(value, str) and len(value) == length and all(c in '0123456789abcdefABCDEF' for c in value)
    origin_keys = {'canonical_repository_path', 'branch', 'head', 'status_porcelain', 'diff_sha256'}
    origin = repository.get('expected_origin')
    if (repository.get('kind') != 'frozen_source_export' or repository.get('root_reviewed') is not True
            or not isinstance(origin, dict) or set(origin) != origin_keys
            or not isinstance(origin['canonical_repository_path'], str)
            or not Path(origin['canonical_repository_path']).is_absolute()
            or '..' in Path(origin['canonical_repository_path']).parts
            or not isinstance(origin['branch'], str) or not origin['branch']
            or not hexadecimal(origin['head'], 40) or not isinstance(origin['status_porcelain'], str)
            or not hexadecimal(origin['diff_sha256'], 64)):
        raise Hold('root-reviewed complete canonical source-export origin required')
    root_value, manifest_value = repository.get('export_root'), repository.get('manifest_path')
    if (not isinstance(root_value, str) or not isinstance(manifest_value, str)
            or not Path(root_value).is_absolute() or not Path(manifest_value).is_absolute()
            or '..' in Path(root_value).parts or '..' in Path(manifest_value).parts
            or not hexadecimal(repository.get('manifest_sha256'), 64)):
        raise Hold('absolute source export root/manifest and manifest SHA required')
    root, manifest_path = Path(root_value), Path(manifest_value)
    if not root.is_dir() or any(parent.is_symlink() for parent in (root, *root.parents)):
        raise Hold('source export root/ancestors must be real directories')
    def walk_error(error):
        raise Hold('source export directory inventory unreadable: ' + str(error)) from error
    try:
        manifest_relative = manifest_path.relative_to(root).as_posix()
        inventory = set()
        for directory, dirs, files in os.walk(root, followlinks=False, onerror=walk_error):
            for name in dirs + files:
                item = Path(directory)/name
                if item.is_symlink() or not (item.is_dir() or item.is_file()):
                    raise Hold('source export symlink/special file rejected')
            inventory.update((Path(directory)/name).relative_to(root).as_posix() for name in files)
        manifest_verification, raw = export_file(root, manifest_relative, contents=True)
    except (OSError, ValueError) as exc:
        raise Hold('source export path/symlink/escape rejected: ' + str(exc)) from exc
    if (manifest_verification['sha256'] != repository['manifest_sha256']
            or not any(item.get('path') == str(manifest_path) and item.get('sha256') == repository['manifest_sha256']
                       for item in artifacts.values())):
        raise Hold('source export manifest must match complete artifact inventory and SHA')
    def unique(pairs):
        value = {}
        for key, item in pairs:
            if key in value:
                raise Hold('duplicate source export JSON key: ' + key)
            value[key] = item
        return value
    manifest = json.loads(raw, object_pairs_hook=unique,
                          parse_constant=lambda value: (_ for _ in ()).throw(Hold('nonfinite source export JSON')))
    if (not isinstance(manifest, dict) or set(manifest) != {'schema', 'origin', 'files_sha256'}
            or manifest['schema'] != 'canonical_git_source_export.v1' or manifest['origin'] != origin
            or not isinstance(manifest['files_sha256'], dict) or not manifest['files_sha256']):
        raise Hold('source export schema/origin/file inventory differs from root freeze')
    verifications = []
    for relative, digest in sorted(manifest['files_sha256'].items()):
        if not hexadecimal(digest, 64) or relative == manifest_relative:
            raise Hold('source export source SHA invalid or manifest self-listed')
        try:
            verification, _ = export_file(root, relative)
        except (OSError, ValueError) as exc:
            raise Hold('source export file path/symlink/escape rejected: ' + str(exc)) from exc
        if verification['sha256'] != digest:
            raise Hold('source export source bytes changed: ' + relative)
        verifications.append(verification)
    if inventory != set(manifest['files_sha256']) | {manifest_relative}:
        raise Hold('source export must contain exactly listed participating sources plus manifest')
    return dict(kind='frozen_source_export', export_root=str(root), manifest_path=str(manifest_path),
                manifest_sha256=manifest_verification['sha256'], origin=origin,
                observed_git_on_runtime_host=False, root_reviewed=True,
                origin_is_canonical_metadata_not_runtime_git_observation=True,
                verified_files=verifications, directory_inventory=sorted(inventory)), raw


def validate(config, output, release):
    if config.get('schema_version') != 1 or config.get('scope') != SCOPE or config.get('executable') is not True:
        raise Hold('default-HOLD config is not an executable recorder release')
    if not uuid32(config.get('recording_run_id')) or not config.get('case_id'):
        raise Hold('root case and unique recording run ID required')
    seconds = deadline.runtime_seconds(config['max_runtime_sec'])
    wait = config.get('controller_wait_sec')
    if type(wait) not in (float, int) or not math.isfinite(wait) or not 0 < wait <= 120:
        raise Hold('controller preparation wait must be finite in (0,120]')
    controller = config['controller']
    controller_scope, _ = mapping_result_adapter.expected_contract(controller)
    if not uuid32(controller.get('run_id')) or controller.get('case_id') != config['case_id'] or controller.get('recording_run_id') != config['recording_run_id']:
        raise Hold('controller run/case/recording binding required')
    deadline.runtime_seconds(controller['max_runtime_sec'])
    if seconds < wait + controller['max_runtime_sec'] + TAIL_SEC + CLOSE_RESERVE_SEC:
        raise Hold('recorder budget must cover preparation + whole controller + extra raw tail + close reserve')
    for key in ('output_directory', 'root_release_path'):
        if not Path(controller[key]).is_absolute():
            raise Hold('absolute controller paths required')
    if Path(controller['output_directory']) != output / 'controller':
        raise Hold('controller output must be this unique recording run/controller')
    if Path(controller['output_directory']).exists():
        raise Hold('controller output already exists')
    if not controller.get('expected_argv') or not controller.get('sources_sha256'):
        raise Hold('frozen exact controller argv and source hashes required')
    if not REQUIRED_TOPICS <= set(config.get('required_recorded_topics', [])):
        raise Hold('original/derived LiDAR IMU JPG Info clock debug topics required')
    if config.get('record_all_including_hidden') is not True or config.get('original_cdr_no_reencoding') is not True:
        raise Hold('all-topic original CDR recording contract required')
    if set(config['expected_environment']) != set(ENV_BINDING):
        raise Hold('complete explicit same-host/message-prefix environment binding required')
    if any(config['expected_environment'].get(k) != v for k, v in ENV_REQUIRED.items()):
        raise Hold('same-host SIM domain185 FastDDS only')
    if any(os.environ.get(k) != v for k, v in config['expected_environment'].items()):
        raise Hold('current environment differs from frozen original runtime environment')
    artifacts = config['artifacts']
    if not REQUIRED_ARTIFACT_ROLES <= set(artifacts) or not any(a.get('kind') == 'elf' for a in artifacts.values()):
        raise Hold('complete run script/ELF/scene/config/oracle/route inventory required')
    if config.get('artifact_inventory_complete_root_reviewed') is not True:
        raise Hold('root must review the complete participating script/ELF/scene inventory')
    for name, item in artifacts.items():
        if not name.replace('_', '').isalnum() or not Path(item['path']).is_absolute() or sha(item['path']) != item['sha256']:
            raise Hold('frozen artifact missing or changed: ' + name)
    if controller['route_sha256'] != artifacts['route']['sha256'] or controller['oracle_sha256'] != artifacts['oracle']['sha256']:
        raise Hold('controller route/oracle differs from raw run freeze')
    if not config.get('source_processes') or not config.get('repositories'):
        raise Hold('exact source identities and canonical source commit/dirty expectations required')
    for repository in config['repositories']:
        if repository.get('kind') == 'frozen_source_export':
            verify_frozen_source_export(repository, artifacts)
        elif repository.get('kind', 'git') != 'git':
            raise Hold('unsupported source provenance kind')
    for item in config['source_processes']:
        identity_live(item['identity'])
    if (release.get('scope') != SCOPE or release.get('allow_execute') is not True
            or release.get('recording_run_id') != config['recording_run_id']
            or release.get('case_id') != config['case_id'] or release.get('output_directory') != str(output)
            or release.get('boot_id') != Path('/proc/sys/kernel/random/boot_id').read_text().strip()
            or release.get('network_namespace') != os.readlink('/proc/self/ns/net')
            or release.get('package_sha256') != {name:sha(HERE/name) for name in PACKAGE}):
        raise Hold('explicit recorder-only root release identity/package binding required')


def freeze(config, output, config_path, release_path):
    for name in PACKAGE:
        shutil.copyfile(HERE/name, output/name)
    shutil.copyfile(config_path, output/'recording_config.frozen.json')
    shutil.copyfile(release_path, output/'root_recording_release.frozen.json')
    destination = output/'frozen_inputs'
    destination.mkdir()
    entries = {}
    for index, (role, item) in enumerate(sorted(config['artifacts'].items())):
        copied = destination / (str(index).zfill(3) + '_' + role + '_' + Path(item['path']).name)
        shutil.copyfile(item['path'], copied)
        if sha(copied) != item['sha256'] or sha(item['path']) != item['sha256']:
            raise Hold('artifact changed during freeze: ' + role)
        entries[role] = dict(item, frozen_path=str(copied.relative_to(output)))
    # Auto-freeze the exact Python ELF and ros2 script in addition to the reviewed dependency inventory.
    for role, path in [('parent_python_elf', Path(sys.executable).resolve()), ('ros2_cli', Path('/opt/ros/jazzy/bin/ros2'))]:
        copied = destination / role
        shutil.copyfile(path, copied)
        entries[role] = dict(path=str(path), sha256=sha(copied), frozen_path=str(copied.relative_to(output)))
    write(output/'frozen_inputs_manifest.json', dict(artifacts=entries, package_sha256={name:sha(output/name) for name in PACKAGE},
        environment=config['expected_environment'], source_processes=config['source_processes'],
        scene_instance_id=config['scene_instance_id'], no_cdr_reencoding=True))


def snapshot_repositories(config, output):
    result = []
    for index, repository in enumerate(config['repositories']):
        if repository.get('kind') == 'frozen_source_export':
            verification, raw = verify_frozen_source_export(repository, config['artifacts'])
            filename = 'repository_' + str(index) + '.source_export.manifest.json'
            (output/filename).write_bytes(raw)
            if sha(output/filename) != verification['manifest_sha256']:
                raise Hold('source export manifest changed while freezing evidence')
            result.append(dict(verification, manifest_frozen_file=filename))
            continue
        if repository.get('kind', 'git') != 'git':
            raise Hold('unsupported source provenance kind')
        path = Path(repository['path'])
        def git(*args):
            return subprocess.run(['/usr/bin/git', '--no-optional-locks', '-C', str(path), *args], check=True,
                                  stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10).stdout
        head = git('rev-parse', 'HEAD').decode().strip()
        status = git('status', '--porcelain=v1', '--untracked-files=all').decode()
        diff = git('diff', '--binary', 'HEAD', '--')
        digest = hashlib.sha256(diff).hexdigest()
        if (head != repository['expected_head'] or status != repository['expected_status_porcelain']
                or digest != repository['expected_diff_sha256']):
            raise Hold('source commit/dirty state differs from root freeze: ' + str(path))
        filename = 'repository_' + str(index) + '.diff'
        (output/filename).write_bytes(diff)
        result.append(dict(path=str(path), head=head, status_porcelain=status, diff_sha256=digest, diff_file=filename))
    write(output/'repositories_at_recording_start.json', result)


def recorder_command(config, output):
    return ['/opt/ros/jazzy/bin/ros2', 'bag', 'record', '--node-name', 'full_scene_raw_recorder_' + config['recording_run_id'],
            '-a', '--include-hidden-topics', '-s', 'mcap', '--storage-preset-profile', 'zstd_fast',
            '--max-cache-size', '268435456', '-o', str(output/'bag')]


def bind_controller(config, output, ready_sha):
    path = output/'controller_binding.json'
    if not path.exists():
        return None
    value = read_json(path)
    controller = config['controller']
    controller_scope, _ = mapping_result_adapter.expected_contract(controller)
    if any(value.get(key) != controller[key] for key in ('run_id', 'case_id', 'recording_run_id', 'output_directory')):
        raise Hold('controller binding run/output/case differs')
    if value.get('scope') != controller_scope or value.get('recorder_ready_sha256') != ready_sha:
        raise Hold('controller binding scope/recorder readiness differs')
    release_path = Path(controller['root_release_path'])
    digest = sha(release_path)
    if value.get('root_release_sha256') != digest or value.get('root_release_path') != str(release_path):
        raise Hold('controller release hash/path differs')
    release = read_json(release_path)
    expected = dict(scope=controller_scope, mode=mapping_result_adapter.MODE, allow_execute=True,
        release_id=controller['run_id'], case_id=controller['case_id'], recording_run_id=controller['recording_run_id'],
        route_sha256=controller['route_sha256'], oracle_sha256=controller['oracle_sha256'],
        sources_sha256=controller['sources_sha256'], output_directory=controller['output_directory'],
        max_runtime_sec=controller['max_runtime_sec'], recorder_ready_path=str(output/'recorder_ready.json'),
        recorder_ready_sha256=ready_sha, scene_instance_id=config['scene_instance_id'])
    if any(release.get(key) != item for key, item in expected.items()):
        raise Hold('post-recorder root controller release differs from frozen acquisition')
    identity = value['identity']
    if identity['argv'] != controller['expected_argv']:
        raise Hold('exact controller argv differs')
    descriptor = os.pidfd_open(identity['pid'])  # Retained before identity/env recheck; never used for a signal.
    try:
        identity_live(identity, dict(MAPPING_OWNER_RUN_ID=controller['run_id'], MAPPING_OWNER_OUTPUT=controller['output_directory']))
        if select.select([descriptor], [], [], 0)[0]:
            raise Hold('controller exited before exact identity could be bound')
        shutil.copyfile(release_path, output/'root_controller_release.frozen.json')
        shutil.copyfile(path, output/'controller_binding.frozen.json')
        if sha(output/'root_controller_release.frozen.json') != digest or read_json(output/'controller_binding.frozen.json') != value:
            raise Hold('controller binding/release changed during freeze')
        bound_steady_ns = time.monotonic_ns()
        write(output/'controller_binding_ack.json', dict(identity=identity, root_release_sha256=digest,
            recorder_ready_sha256=ready_sha, boottime_sec=time.clock_gettime(time.CLOCK_BOOTTIME),
            bound_steady_ns=bound_steady_ns,
            observation_only_no_controller_signal=True))
        return descriptor, identity, digest, path.read_bytes(), release_path.read_bytes(), bound_steady_ns
    except BaseException:
        os.close(descriptor)
        raise


def run(config_path, output, release_path):
    output = output.resolve()
    config, release = read_json(config_path), read_json(release_path)
    validate(config, output, release)
    if release.get('config_sha256') != sha(config_path):
        raise Hold('root recorder release config SHA mismatch')
    output.mkdir(parents=True, exist_ok=False)
    guard = recorder = recorder_fd = controller_fd = recorder_birth = None
    stopped = []
    result = dict(scope=SCOPE, recording_run_id=config['recording_run_id'], case_id=config['case_id'],
                  classification='HOLD_preparation_incomplete', controller_signals_sent=0, recorder_sigint_sent=False)
    previous = {number:signal.signal(number, lambda n, f: stopped.append(n)) for number in (signal.SIGINT, signal.SIGTERM)}
    try:
        freeze(config, output, config_path, release_path)
        if (sha(output/'recording_config.frozen.json') != release['config_sha256']
                or read_json(output/'root_recording_release.frozen.json') != release
                or sha(release_path) != sha(output/'root_recording_release.frozen.json')):
            raise Hold('root recorder config/release changed during freeze')
        marker = release_path.resolve().parent/(config['recording_run_id'] + '.recording_once.json')
        claim_once(marker, dict(scope=SCOPE, config_sha256=sha(config_path), release_sha256=sha(release_path),
            output_directory=str(output), launcher=deadline.process_identity(os.getpid()), claimed_unix_ns=time.time_ns()))
        # This marker precedes the first Popen, including the read-only Git snapshot subprocesses.
        snapshot_repositories(config, output)
        if stopped:
            raise Hold('parent interrupted before recorder')
        guard = deadline.arm(output, config['max_runtime_sec'])
        remaining = guard.manifest['deadline_boottime_sec'] - time.clock_gettime(time.CLOCK_BOOTTIME)
        required = config['controller_wait_sec'] + config['controller']['max_runtime_sec'] + TAIL_SEC + CLOSE_RESERVE_SEC
        if remaining < required:
            raise Hold('freeze/startup consumed the reserved recorder budget')
        argv, environment = guard.owned_command('recorder', recorder_command(config, output), dict(os.environ))
        with (output/'recorder.log').open('x') as log:
            recorder = subprocess.Popen(argv, cwd='/', env=environment, stdin=subprocess.DEVNULL, stdout=log,
                                        stderr=subprocess.STDOUT, start_new_session=True, close_fds=True)
        recorder_fd = os.pidfd_open(recorder.pid)
        # Exec can transiently expose an empty cmdline. Retain the pidfd first
        # and bind stable birth fields without interpreting argv until ACK.
        birth_fields = (Path('/proc')/str(recorder.pid)/'stat').read_text().rsplit(')', 1)[1].split()
        recorder_birth = dict(pid=recorder.pid, starttime_ticks=int(birth_fields[19]),
                              boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip())
        guard.wait_for_child('recorder', recorder.pid)
        registration = read_json(output/'recording_deadline_recorder_registration.json')
        if any(registration['wrapper_identity'][key] != recorder_birth[key]
               for key in ('pid', 'starttime_ticks', 'boot_id')):
            raise Hold('owned recorder birth differs from retained child after ACK')
        recorder_birth = registration['wrapper_identity']
        recorder_identity = registration['identity']
        if recorder_identity != deadline.identity_after_exec(recorder_birth, guard.children['recorder']['executed_argv']):
            raise Hold('owned recorder registration exec identity differs from plan')
        identity_live(recorder_identity)
        storage_until = time.monotonic() + 10.
        storage_files = []
        while time.monotonic() < storage_until and not stopped:
            guard.check_live()
            if recorder.poll() is not None:
                raise Hold('owned recorder exited before MCAP storage opened')
            for path in (output/'bag').glob('*.mcap'):
                with path.open('rb') as stream:
                    if stream.read(8) == bag_summary.MAGIC:
                        storage_files.append(dict(path=str(path), bytes=path.stat().st_size))
            if storage_files:
                break
            time.sleep(.05)
        if not storage_files or stopped:
            raise Hold('MCAP storage did not open within bounded preparation')
        for item in config['source_processes']:
            identity_live(item['identity'])
        remaining = guard.manifest['deadline_boottime_sec'] - time.clock_gettime(time.CLOCK_BOOTTIME)
        if remaining < required:
            raise Hold('recorder initialization consumed the reserved whole-controller/tail budget')
        ready = dict(schema_version=1, scope=SCOPE, recording_run_id=config['recording_run_id'], case_id=config['case_id'],
            scene_instance_id=config['scene_instance_id'], recorder_process_identity=recorder_identity,
            node_name='full_scene_raw_recorder_' + config['recording_run_id'], node_namespace='/',
            recorder_node_name='full_scene_raw_recorder_' + config['recording_run_id'], recorder_node_namespace='/',
            required_recorded_topics=config['required_recorded_topics'], source_processes=config['source_processes'],
            frozen_inputs_manifest_sha256=sha(output/'frozen_inputs_manifest.json'),
            repositories_snapshot_sha256=sha(output/'repositories_at_recording_start.json'),
            config_sha256=sha(config_path), package_sha256=release['package_sha256'],
            storage_open_header_verified=storage_files,
            deadline_manifest_sha256=guard.digest, deadline_run_id=guard.manifest['run_id'],
            deadline_boottime_sec=guard.manifest['deadline_boottime_sec'],
            recorder_deadline_steady_ns=time.monotonic_ns() + int(remaining*1e9),
            ready_unix_ns=time.time_ns(), ready_boottime_sec=time.clock_gettime(time.CLOCK_BOOTTIME),
            ready_condition='live recorder; retained deadline/parent pidfds; source identities and frozen inputs bound',
            graph_subscription_verified=False, authorizes_controller_or_motion=False)
        write(output/'recorder_ready.json', ready)
        ready_sha = sha(output/'recorder_ready.json')
        result.update(recorder_identity=recorder_identity, recorder_ready_sha256=ready_sha)
        waiting_until = time.clock_gettime(time.CLOCK_BOOTTIME) + config['controller_wait_sec']
        bound = None
        tail_until = None
        while not stopped:
            guard.check_live()
            if sha(output/'recorder_ready.json') != ready_sha:
                raise Hold('immutable recorder readiness receipt changed')
            if recorder.poll() is not None:
                raise Hold('owned recorder exited before completion')
            for item in config['source_processes']:
                identity_live(item['identity'])
            now = time.clock_gettime(time.CLOCK_BOOTTIME)
            if bound is None:
                bound = bind_controller(config, output, ready_sha)
                if bound:
                    controller_fd = bound[0]
                    result['controller_identity'] = bound[1]
                elif now >= waiting_until:
                    result['classification'] = 'CLOSED_no_controller_within_preparation_wait'
                    break
            elif tail_until is not None:
                if now >= tail_until:
                    result['classification'] = 'CLOSED_mapping_success_with_extra_stationary_raw_tail'
                    result['extra_tail_completed_boottime_sec'] = now
                    break
            else:
                if ((output/'controller_binding.json').read_bytes() != bound[3]
                        or Path(config['controller']['root_release_path']).read_bytes() != bound[4]):
                    raise Hold('bound controller/release metadata changed')
                if select.select([controller_fd], [], [], 0)[0]:
                    result['controller_exit_observed_boottime_sec'] = now
                    path = Path(config['controller']['output_directory'])/'result.json'
                    if not path.exists():
                        result['classification'] = 'CLOSED_controller_exited_result_missing'
                        break
                    payload = read_json(path)
                    shutil.copyfile(path, output/'controller_result.frozen.json')
                    success, reason = mapping_result_adapter.assess(payload, config['controller'], bound[1], bound[2], ready_sha, bound[5])
                    result.update(controller_result_sha256=sha(path), controller_success=success, controller_assessment=reason)
                    if not success:
                        result['classification'] = 'CLOSED_controller_failed_or_unbound_result'
                        break
                    tail_until = now + TAIL_SEC
                    result.update(extra_tail_seconds=TAIL_SEC, extra_tail_started_boottime_sec=now,
                        extra_tail_basis='controller exited after measured zero+stopped+relay EOF/exit; source identities remain checked; parent publishes nothing')
                else:
                    try:
                        identity_live(bound[1], dict(MAPPING_OWNER_RUN_ID=config['controller']['run_id'],
                                                   MAPPING_OWNER_OUTPUT=config['controller']['output_directory']))
                    except (ProcessLookupError, FileNotFoundError):
                        # Exit can occur between the pidfd readiness check and
                        # /proc reads. Only the retained exact pidfd can confirm
                        # that transition; identity mismatches still fail.
                        if not select.select([controller_fd], [], [], .05)[0]:
                            raise
            time.sleep(.05)
        if stopped:
            result.update(classification='CLOSED_parent_interrupted', parent_signals=stopped)
    except BaseException as exc:
        result.update(classification='HOLD_recording_owner_failure', error_type=type(exc).__name__, error=str(exc))
    finally:
        if guard is not None:
            guard.begin_shutdown(result['classification'])
        if recorder is not None:
            try:
                if recorder.poll() is None and recorder_fd is not None:
                    plan = guard.children['recorder']
                    observed = deadline.process_identity(recorder.pid)
                    if (recorder_birth is None or any(observed[key] != recorder_birth[key] for key in ('pid', 'starttime_ticks', 'boot_id'))
                            or observed['argv'] not in (plan['wrapper_argv'], plan['executed_argv'])):
                        raise Hold('exact owned recorder wrapper/exec identity changed before close')
                    deadline.check_child_binding(observed, guard.manifest['run_id'], output, 'recorder', plan['launch_id'])
                    signal.pidfd_send_signal(recorder_fd, signal.SIGINT)
                    result['recorder_sigint_sent'] = True
                recorder.wait(timeout=CLOSE_WAIT_SEC)
            except BaseException as exc:
                result['close_error'] = dict(type=type(exc).__name__, message=str(exc))
            result['recorder_returncode'] = recorder.poll()
        all_exited = recorder is None or recorder.poll() is not None
        if guard is not None and all_exited:
            guard.recording_closed(result['classification'])
            try:
                guard.process.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                result['guard_exit_pending'] = True
        result.update(recorder_exited=all_exited, finished_unix_ns=time.time_ns(),
                      helper_sha256=sha(output/'recording_deadline.py') if (output/'recording_deadline.py').exists() else None)
        # Disarm only after process exit, then allow slow streaming SHA/summary without deadline signals.
        if all_exited and recorder is not None:
            try:
                result['bag_audit'] = bag_summary.audit(output/'bag', config['required_recorded_topics'])
            except BaseException as exc:
                result['bag_audit'] = dict(complete=False, error_type=type(exc).__name__, error=str(exc))
        result['recording_evidence_complete'] = bool(all_exited and result.get('bag_audit', {}).get('complete'))
        for descriptor in (recorder_fd, controller_fd):
            if descriptor is not None:
                os.close(descriptor)
        write(output/'recording_result.json', result)
        for number, handler in previous.items():
            signal.signal(number, handler)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--root-release', type=Path)
    args = parser.parse_args()
    if not args.execute:
        print(json.dumps(dict(scope=SCOPE, classification='HOLD_PREPARED_ONLY_NO_PROCESS',
                              config_sha256=sha(args.config), output=str(args.output.resolve()),
                              package_sha256={name:sha(HERE/name) for name in PACKAGE})))
        return 0
    if args.root_release is None:
        parser.error('--execute needs a new explicit recorder-only root release')
    result = run(args.config.resolve(), args.output, args.root_release.resolve())
    print(json.dumps(dict(classification=result['classification'], output=str(args.output.resolve()),
                         recorder_exited=result['recorder_exited'])))
    normal = result['classification'] == 'CLOSED_no_controller_within_preparation_wait'
    accepted = (result['classification'] == 'CLOSED_mapping_success_with_extra_stationary_raw_tail'
                and result['recording_evidence_complete'])
    return 0 if result['recorder_exited'] and (normal or accepted) else 2


if __name__ == '__main__':
    raise SystemExit(main())

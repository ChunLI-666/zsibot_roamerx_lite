"""Isolated process management and artifacts shared by the two validation runners."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shlex
import signal
import struct
import subprocess
import sys
import time
import uuid

import yaml


HERE = Path(__file__).resolve().parent
NAV_SOURCE = HERE.parents[1]
WORKSPACE = NAV_SOURCE.parents[6]
ACTION_NAMES = ('navigate_to_pose', 'compute_path_to_pose', 'smooth_path',
                'follow_path', 'spin', 'backup', 'wait')
RECORD_TOPICS = ['/clock', '/tf', '/tf_static', '/map', '/laser_scan',
    '/lightning/pose', '/odom/current_pose', '/lightning/loc_status',
    '/lightning/pose_valid', '/lightning/debug', '/lightning_bridge/debug',
    '/behavior_tree_log', '/plan', '/trajectories', '/transformed_global_plan',
    '/cmd_vel_nav', '/cmd_vel', '/cmd_vel_safe',
    '/nav_safety_gate/gate_status', '/debug/observations', '/debug/view',
    '/debug/build_info', '/goal_pose', '/validation/goal_pose',
    '/global_costmap/costmap', '/global_costmap/costmap_updates',
    '/local_costmap/costmap', '/local_costmap/costmap_updates',
    '/global_costmap/published_footprint', '/local_costmap/published_footprint']
RECORD_TOPICS += ['/' + name + '/_action/' + kind for name in ACTION_NAMES
                  for kind in ('status', 'feedback')]


def write_json(path, data):
    Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def isolate_recovery_seed(config, output, source_root=WORKSPACE):
    """Redirect runtime writes while preserving any configured recovery FP seed."""
    group = config.setdefault('lidar_loc', {})
    source = Path(group.get('recover_pose_path', './data/recover_pose.txt'))
    if not source.is_absolute():
        source = Path(source_root) / source
    destination = Path(output) / 'recover_pose.txt'
    result = dict(source=str(source.resolve()), existed=source.is_file(),
                  runtime=str(destination), source_sha256=None)
    if source.is_file():
        destination.write_bytes(source.read_bytes())
        result['source_sha256'] = sha256(source)
    group['recover_pose_path'] = str(destination)
    return result


def arguments(description):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument('--install-prefix', type=Path, required=True,
                        help='Overlay root containing setup.bash; built artifacts under test')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--config', type=Path,
                        help='Lightning YAML for replay; unmodified navigation parameters for closed loop')
    parser.add_argument('--nav-params', type=Path, default=NAV_SOURCE / 'params/navigo_params.yaml')
    parser.add_argument('--map', type=Path,
                        help='map.yaml or its directory; closed loop defaults to generated open map')
    parser.add_argument('--domain', type=int,
                        help='Isolated ROS domain (default: random 100..220)')
    parser.add_argument('--ready-timeout', type=float, default=120.)
    parser.add_argument('--timeout', type=float, default=90., help='Goal observation wall timeout')
    parser.add_argument('--tail', type=float, default=5., help='Recorded wall seconds after terminal action')
    parser.add_argument('--goal', nargs=3, type=float, metavar=('X', 'Y', 'YAW'))
    parser.add_argument('--goal-input', choices=('action', 'topic'), default='action',
                        help='Replay defaults to the real /goal_pose BT input; closed loop uses Action')
    parser.add_argument('--no-composition', action='store_true')
    parser.add_argument('--observer-off', action='store_true',
                        help='Do not start debug_observer (overhead/control comparison only)')
    parser.add_argument('--observations-off', action='store_true',
                        help='Set producer enable_debug_observations=false; algorithm parameters unchanged')
    return parser


def require_default_environment():
    """Acceptance must exercise the fixed library, not a lifetime workaround."""
    temporary = {name: os.environ[name] for name in ('LD_PRELOAD', 'MAGICK_THREAD_LIMIT')
                 if os.environ.get(name)}
    if temporary:
        raise RuntimeError('Default-environment validation rejects temporary overrides: ' + str(temporary))


def new_navigation_goal_uuid(events, known, since_ns):
    """Adopt only the single new UUID actually emitted by the topic-driven BT."""
    found = set()
    for row in events:
        if row['event'] != '/navigate_to_pose/_action/status' or row['monotonic_ns'] < since_ns:
            continue
        for item in row['data']['status_list']:
            value = tuple(int(v) for v in item['goal_info']['goal_id']['uuid'])
            if value not in known and any(value):
                found.add(value)
    if len(found) > 1:
        raise RuntimeError('Ambiguous newly accepted navigation goals; cannot adopt or cancel safely')
    return next(iter(found)) if found else None


def message_stamp_ns(message):
    """Source timestamps only; never replace them with receipt or wall time."""
    values = []
    if hasattr(message, 'header'):
        values.append(message.header.stamp)
    if hasattr(message, 'transforms'):
        values += [item.header.stamp for item in message.transforms]
    if hasattr(message, 'clock'):
        values.append(message.clock)
    return [value.sec * 1_000_000_000 + value.nanosec for value in values
            if value.sec or value.nanosec]


def timestamp_domain_valid(log_ns, source_ns, maximum_difference_sec=300, source_window_ns=None):
    # This is a time-domain guard, not a claim about network/algorithm latency.
    # The bound allows latched/static messages across a complete short trial.
    if source_window_ns is not None:
        lower, upper = source_window_ns
        return lower <= log_ns <= upper and lower <= source_ns <= upper
    return abs(log_ns - source_ns) <= maximum_difference_sec * 1_000_000_000


def selected_library_identity(actual, install_prefix, library_name):
    expected = Path(install_prefix) / 'lib' / library_name
    actual = Path(actual)
    present = actual.is_file() and expected.is_file()
    return dict(actual=str(actual.resolve()), expected=str(expected.resolve()),
        sha256=sha256(actual) if actual.is_file() else None,
        selected_overlay_sha256=sha256(expected) if expected.is_file() else None,
        valid=present and actual.resolve() == expected.resolve() and sha256(actual) == sha256(expected))


def lifecycle_core_identity(actual, install_prefix):
    return selected_library_identity(actual, install_prefix, 'libnav2_lifecycle_manager_core.so')


def bootstrap(args):
    """Re-exec after sourcing only the selected overlay; isolate ROS discovery."""
    require_default_environment()
    args.install_prefix = args.install_prefix.resolve()
    args.output = args.output.resolve()
    setup = args.install_prefix / 'setup.bash'
    if not setup.is_file():
        raise ValueError('Missing overlay setup: ' + str(setup))
    if args.domain is None:
        args.domain = 100 + uuid.uuid4().int % 121
    if not 0 <= args.domain <= 232:
        raise ValueError('ROS domain must be in 0..232')
    for name in ('ready_timeout', 'timeout', 'tail'):
        if not getattr(args, name) > 0:
            raise ValueError(name + ' must be positive')
    marker = str(args.install_prefix) + ':' + str(args.domain)
    if os.environ.get('MINIMAL_VALIDATION_ENV') != marker:
        distro = os.environ.get('ROS_DISTRO', 'jazzy')
        command = 'set -e; source ' + shlex.quote('/opt/ros/' + distro + '/setup.bash')
        command += '; source ' + shlex.quote(str(setup))
        argv = list(sys.argv)
        if '--domain' not in argv:
            argv += ['--domain', str(args.domain)]
        # A user's remote discovery server/static-peer XML must not turn a
        # local verification domain into a hardware transport.
        peer_variables = ('CYCLONEDDS_URI', 'FASTDDS_DEFAULT_PROFILES_FILE',
                          'FASTRTPS_DEFAULT_PROFILES_FILE', 'ROS_STATIC_PEERS',
                          'ROS_DISCOVERY_SERVER')
        command += '; unset ' + ' '.join(peer_variables)
        command += '; export ROS_LOCALHOST_ONLY=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST'
        command += '; exec /usr/bin/python3 ' + ' '.join(shlex.quote(value) for value in argv)
        env = os.environ.copy()
        for name in peer_variables:
            env.pop(name, None)
        env.update(MINIMAL_VALIDATION_ENV=marker, ROS_DOMAIN_ID=str(args.domain),
                   ROS_LOCALHOST_ONLY='1', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                   ROS_LOG_DIR=str(args.output / 'ros_logs'))
        pangolin = WORKSPACE / 'src/lightning-lm/thirdparty/Pangolin-0.9.3/build'
        if pangolin.is_dir():
            env['LD_LIBRARY_PATH'] = str(pangolin) + ':' + env.get('LD_LIBRARY_PATH', '')
        os.execvpe('/bin/bash', ['bash', '-c', command], env)
    if args.output.exists():
        raise ValueError('Use a new output directory: ' + str(args.output))
    args.output.mkdir(parents=True)
    if args.map:
        args.map = args.map.resolve()
        if args.map.is_dir():
            args.map /= 'map.yaml'
        if not args.map.is_file():
            raise ValueError('Map does not exist: ' + str(args.map))


def prepare_navigation(args, use_sim_time=True):
    """Copy the approved ordinary profile; only time/map/deployment settings vary."""
    source = args.config if getattr(args, 'closed_loop', False) and args.config else args.nav_params
    data = yaml.safe_load(source.read_text())
    def adjust(value):
        if isinstance(value, dict):
            for key, child in value.items():
                if key == 'use_sim_time':
                    value[key] = use_sim_time
                elif key == 'enable_epoch_contract':
                    if child:
                        raise ValueError('Epoch profile is outside this validation scope')
                    value[key] = False
                else:
                    adjust(child)
    adjust(data)
    for name in ('controller_server', 'planner_server', 'bt_navigator',
                 'velocity_smoother'):
        data.setdefault(name, {}).setdefault('ros__parameters', {})['enable_epoch_contract'] = False
    gate = data.setdefault('nav_safety_gate', {}).setdefault('ros__parameters', {})
    gate.update(use_sim_time=use_sim_time, enable_epoch_contract=False)
    if getattr(args, 'observations_off', False):
        for name in ('map_server', 'planner_server', 'controller_server',
                     'behavior_server', 'bt_navigator', 'velocity_smoother', 'nav_safety_gate'):
            data[name]['ros__parameters']['enable_debug_observations'] = False
    bt = data['bt_navigator']['ros__parameters']
    for key in ('default_nav_to_pose_bt_xml', 'default_nav_through_poses_bt_xml'):
        if 'epoch' in str(bt.get(key, '')).lower():
            raise ValueError('Epoch BT is outside this validation scope')
    data['map_server']['ros__parameters']['yaml_filename'] = str(args.map)
    path = args.output / 'navigation_params.yaml'
    path.write_text(yaml.safe_dump(data, sort_keys=False))
    write_json(args.output / 'profile.json', dict(source=str(source.resolve()),
        source_sha256=sha256(source), effective_sha256=sha256(path),
        modifications=['use_sim_time=true', 'map YAML path', 'enable_epoch_contract=false'],
        producer_observations_enabled=not getattr(args, 'observations_off', False),
        controller=data['controller_server']['ros__parameters'],
        scope='ordinary production MPPI and goal checker; no algorithm overlay'))
    return path, data


def runtime_manifest(args, mode):
    from ament_index_python.packages import get_package_prefix
    critical = ('navigo_util', 'navigo_core', 'navigo_costmap_2d', 'navigo_behavior_tree',
        'navigo_bt_navigator', 'navigo_mppi_controller', 'navigo_path_controller',
        'navigo_path_planner', 'navigo_navfn_planner', 'navigo_map_server',
        'navigo_behaviors', 'navigo_velocity_optimizer', 'navigo_waypoint_follower',
        'robot_navigo', 'robot_debug_msgs', 'robot_debug_tools', 'nav2_lifecycle_manager', 'bondcpp')
    if mode == 'replay':
        critical += ('lightning',)
    prefixes = {package: str(Path(get_package_prefix(package)).resolve()) for package in critical}
    mixed = {package: prefix for package, prefix in prefixes.items()
             if not Path(prefix).is_relative_to(args.install_prefix)}
    if mixed:
        raise RuntimeError('Critical packages come from a different build overlay: ' + str(mixed))
    targets = [('robot_navigo', 'lib/robot_navigo/livox_scan_projector'),
        ('robot_navigo', 'lib/robot_navigo/nav_safety_gate.py'),
        ('navigo_path_controller', 'lib/navigo_path_controller/controller_server'),
        ('navigo_path_planner', 'lib/navigo_path_planner/planner_server'),
        ('navigo_bt_navigator', 'lib/navigo_bt_navigator/bt_navigator'),
        ('nav2_smoother', 'lib/nav2_smoother/smoother_server'),
        ('nav2_lifecycle_manager', 'lib/libnav2_lifecycle_manager_core.so'),
        ('bondcpp', 'lib/libbondcpp.so')]
    if mode == 'replay':
        targets += [('lightning', 'lib/lightning/run_loc_online')]
    if not args.observer_off:
        targets += [('robot_debug_tools', 'lib/robot_debug_tools/debug_observer')]
    files, libraries = [], {}
    for package, relative in targets:
        path = Path(get_package_prefix(package)) / relative
        if not path.is_file():
            raise RuntimeError('Missing runtime target: ' + str(path))
        files.append(dict(package=package, path=str(path), resolved=str(path.resolve()), sha256=sha256(path)))
        with path.open('rb') as stream:
            elf = stream.read(4) == b'\x7fELF'
        if elf:
            env = os.environ.copy()
            if package == 'nav2_smoother':
                env['LD_LIBRARY_PATH'] = '/opt/ros/' + os.environ.get('ROS_DISTRO', 'jazzy') + '/lib:' + env.get('LD_LIBRARY_PATH', '')
            listing = subprocess.check_output(['ldd', str(path)], text=True, env=env)
            libraries[str(path)] = listing
            if 'not found' in listing:
                raise RuntimeError('Missing runtime library: ' + str(path))
            for line in listing.splitlines():
                if '=>' not in line:
                    continue
                candidate = Path(line.split('=>', 1)[1].strip().split(' ', 1)[0])
                if candidate.is_file() and str(WORKSPACE) in str(candidate.resolve()):
                    files.append(dict(path=str(candidate), resolved=str(candidate.resolve()), sha256=sha256(candidate)))
    for path in HERE.glob('*.py'):
        files.append(dict(path=str(path), sha256=sha256(path)))
    result = dict(mode=mode, ros_distro=os.environ.get('ROS_DISTRO'),
        ros_domain_id=args.domain, install_prefix=str(args.install_prefix), package_prefixes=prefixes,
        files=files, libraries=libraries,
        environment={name: os.environ.get(name) for name in ('MAGICK_THREAD_LIMIT', 'LD_PRELOAD',
            'RMW_IMPLEMENTATION', 'ROS_LOCALHOST_ONLY', 'ROS_AUTOMATIC_DISCOVERY_RANGE')},
        temporary_environment_overrides_absent=not any(os.environ.get(name)
            for name in ('LD_PRELOAD', 'MAGICK_THREAD_LIMIT')),
        goal_input=getattr(args, 'goal_input', 'action'),
        composition_enabled=not args.no_composition,
        hardware_io=False, transport='ROS localhost; no LCM, UDP adapter or SDK process',
        limitations=['SE2 plant excludes dynamics/actuator/SDK' if mode == 'closed_loop'
                     else 'Recorded motion is open loop: arrival is N/A'])
    write_json(args.output / 'runtime_manifest.json', result)
    return result


class Processes:
    """Own only process groups started by this instance; never global pkill."""
    def __init__(self, output):
        self.output = Path(output)
        self.items = []
        self.stages = {}
        self.pump = None

    def stage(self, name, status, **detail):
        entry = self.stages.setdefault(name, dict(started_monotonic_ns=time.monotonic_ns()))
        entry.update(status=status, monotonic_ns=time.monotonic_ns(), **detail)
        write_json(self.output / 'stage_manifest.json', self.stages)

    def start(self, name, command, env=None):
        stream = (self.output / (name + '.log')).open('w')
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                   start_new_session=True, env=env, cwd=self.output)
        self.items.append((name, process, stream))
        self.stage(name, 'RUNNING', pid=process.pid, command=list(map(str, command)))
        return process

    def check(self, allow=()):
        for name, process, _ in self.items:
            code = process.poll()
            if code is not None and name not in allow:
                raise RuntimeError(f'{name} exited prematurely: {code}')

    def loaded_libraries(self, install_prefix):
        """Freeze actual mapped plugin DSOs, not only pre-launch ldd guesses."""
        listing = subprocess.check_output(['ps', '-eo', 'pid=,ppid='], text=True)
        graph = [tuple(map(int, line.split())) for line in listing.splitlines() if line.strip()]
        owned = {process.pid for _, process, _ in self.items}
        while True:
            expanded = owned | {pid for pid, parent in graph if parent in owned}
            if expanded == owned:
                break
            owned = expanded
        selected = {path.name: path for path in (Path(install_prefix) / 'lib').glob('*.so*') if path.is_file()}
        hashes, mapped, mismatches, lifecycle_cores, bonds = {}, {}, [], {}, {}
        for pid in sorted(owned):
            path = Path('/proc') / str(pid) / 'maps'
            try:
                lines = path.read_text().splitlines()
            except (FileNotFoundError, ProcessLookupError):
                continue
            for line in lines:
                fields = line.split(None, 5)
                if len(fields) < 6 or '.so' not in fields[5]:
                    continue
                library = Path(fields[5])
                is_lifecycle_core = library.name.startswith('libnav2_lifecycle_manager_core.so')
                is_bondcpp = library.name.startswith('libbondcpp.so')
                if not str(library).startswith(str(WORKSPACE)) and not (is_lifecycle_core or is_bondcpp):
                    continue
                if is_lifecycle_core:
                    identity = lifecycle_core_identity(library, install_prefix)
                    entry = lifecycle_cores.setdefault(str(library.resolve()), identity)
                    if pid not in entry.setdefault('pids', []):
                        entry['pids'].append(pid)
                    if not identity['valid']:
                        mismatches.append(str(library))
                if is_bondcpp:
                    bonds.setdefault(pid, set()).add(str(library))
                if not library.is_file():
                    continue
                if str(library) not in hashes:
                    hashes[str(library)] = sha256(library)
                item = mapped.setdefault(str(library), dict(path=str(library),
                    resolved=str(library.resolve()), sha256=hashes[str(library)], pids=[]))
                if pid not in item['pids']:
                    item['pids'].append(pid)
                expected = selected.get(library.name)
                if '__rosidl_' in library.name:
                    item['scope'] = 'message type support; fixed underlay allowed, not a navigation algorithm DSO'
                elif is_bondcpp:
                    item['scope'] = 'fixed overlay required in manager container only; isolated upstream smoother may keep /opt'
                elif expected is not None:
                    key = str(expected)
                    if key not in hashes:
                        hashes[key] = sha256(expected)
                    item['selected_overlay_sha256'] = hashes[key]
                    if hashes[str(library)] != hashes[key]:
                        mismatches.append(str(library))
        manager_bonds = []
        for pid in {pid for item in lifecycle_cores.values() for pid in item['pids']}:
            actual = bonds.get(pid, set())
            if len(actual) != 1:
                mismatches.append('manager PID ' + str(pid) + ': expected exactly one bondcpp library')
            for path in actual:
                identity = selected_library_identity(path, install_prefix, 'libbondcpp.so')
                manager_bonds.append(dict(pid=pid, **identity))
                if not identity['valid']:
                    mismatches.append(path)
        result = dict(owned_pids=sorted(owned), libraries=list(mapped.values()),
                      mismatched_overlay_libraries=sorted(set(mismatches)),
                      lifecycle_cores=list(lifecycle_cores.values()),
                      manager_bondcpp=manager_bonds)
        write_json(self.output / 'loaded_libraries.json', result)
        if len(lifecycle_cores) != 1:
            raise RuntimeError('Expected exactly one actually mapped fixed LifecycleManager core: ' + str(lifecycle_cores))
        if mismatches:
            raise RuntimeError('Actual plugin DSOs differ from selected overlay: ' + str(sorted(set(mismatches))))
        return result

    def stop(self, name):
        for item_name, process, stream in self.items:
            if item_name != name:
                continue
            sent = []
            for sig, timeout in ((signal.SIGINT, 12.), (signal.SIGTERM, 4.), (signal.SIGKILL, 2.)):
                process.poll()
                try:
                    os.killpg(process.pid, 0)
                except ProcessLookupError:
                    break
                # ros2 launch relays SIGINT to its children itself. Signalling
                # its complete group as well delivers duplicate interrupts.
                if name == 'navigation' and sig == signal.SIGINT:
                    process.send_signal(sig)
                else:
                    os.killpg(process.pid, sig)
                sent.append(sig.name)
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    process.poll()
                    try:
                        os.killpg(process.pid, 0)
                    except ProcessLookupError:
                        break
                    if self.pump:
                        self.pump()
                    else:
                        time.sleep(.02)
            process.poll()
            stream.flush()
            try:
                os.killpg(process.pid, 0)
                alive = True
            except ProcessLookupError:
                alive = False
            audit = child_exit_audit(self.output / (name + '.log'), 'SIGINT' in sent)
            self.stage(name, 'STOPPED' if not alive else 'CLEANUP_FAILED',
                       returncode=process.returncode, signals=sent, group_alive=alive,
                       child_failures=audit['failures'],
                       normal_log_process_signal_exits=audit['normal_log_process_signal_exits'])

    def close(self):
        # Keep the replay/plant clock and sensors live while lifecycle nodes
        # shut down. Stopping /clock first can strand their sim-time waits.
        preferred = ['navigation', 'scan_projection', 'gate', 'observer',
                     'localization', 'player', 'recorder']
        names = [name for name in preferred if any(row[0] == name for row in self.items)]
        names += [row[0] for row in reversed(self.items) if row[0] not in names]
        for name in names:
            self.stop(name)
        for _, _, stream in self.items:
            stream.close()

    def cleanup_valid(self):
        return all(not entry.get('group_alive') and not
            (set(entry.get('signals', [])) & {'SIGTERM', 'SIGKILL'})
            and not entry.get('child_failures')
            for entry in self.stages.values() if 'group_alive' in entry)


def child_exit_audit(path, deliberate_sigint=False):
    """Allow only the non-ROS tail's expected SIGINT during observed cleanup."""
    pattern = re.compile(r'process has died|failed to terminate|SIGSEGV|abort due to signal')
    failures, normal, shutdown_seen = [], [], False
    tail_sigint = re.compile(r'\[log_aggregator-\d+\]: process has died .*exit code (-2|130),')
    for line in Path(path).read_text(errors='replace').splitlines():
        if 'user interrupted with ctrl-c (SIGINT)' in line:
            shutdown_seen = True
        if not pattern.search(line):
            continue
        if deliberate_sigint and shutdown_seen and tail_sigint.search(line):
            normal.append(line)
        else:
            failures.append(line)
    return dict(failures=failures[-8:], normal_log_process_signal_exits=normal)


def child_failures(path):
    """A launch leader exiting zero does not prove its native children did."""
    return child_exit_audit(path)['failures']


def start_stack(processes, args, params, scan=False):
    if not args.observer_off:
        processes.start('observer', ['ros2', 'run', 'robot_debug_tools', 'debug_observer',
            '--ros-args', '-p', 'use_sim_time:=true'])
    processes.start('navigation', ['ros2', 'launch', 'robot_navigo', 'lightning_nav_bringup.launch.py',
        'map:=' + str(args.map), 'params_file:=' + str(params), 'use_sim_time:=true',
        'autostart:=true', 'use_composition:=' + str(not args.no_composition),
        'enable_laserscan:=' + str(scan).lower(),
        'log_base_dir:=' + str(args.output / 'nav_logs')])


def cadence(timestamps):
    """Report wire publication/receipt intervals, without calling them compute time."""
    intervals = [(later - earlier) / 1e9 for earlier, later in zip(timestamps, timestamps[1:])]
    positive = sorted(value for value in intervals if value > 0 and math.isfinite(value))
    if not positive:
        return dict(samples=len(timestamps), intervals=0, nonpositive_intervals=len(intervals))
    def percentile(fraction):
        index = (len(positive) - 1) * fraction
        low, high = math.floor(index), math.ceil(index)
        return positive[low] + (positive[high] - positive[low]) * (index - low)
    return dict(samples=len(timestamps), intervals=len(positive),
        nonpositive_intervals=len(intervals) - len(positive), mean_sec=sum(positive) / len(positive),
        median_sec=percentile(.5), p95_sec=percentile(.95), max_sec=max(positive))


def start_recording(processes, args):
    overrides = args.output / 'record_qos.yaml'
    durable = dict(reliability='reliable', durability='transient_local', history='keep_last', depth=50)
    qos = {topic: durable for topic in ['/tf_static', '/map', '/debug/build_info',
                                      '/global_costmap/costmap', '/local_costmap/costmap']}
    qos.update({'/' + name + '/_action/status': durable for name in ACTION_NAMES})
    overrides.write_text(yaml.safe_dump(qos))
    processes.start('recorder', ['ros2', 'bag', 'record', '-s', 'mcap',
        '--use-sim-time', '--include-hidden-topics', '--qos-profile-overrides-path', str(overrides),
        '-o', str(args.output / 'bag'), *RECORD_TOPICS, *getattr(args, 'record_topics_extra', [])])


def mcap_summary_schemas(path, include_data=False):
    """Read repeated Schema/Channel summary records without a Python MCAP dependency.

    rosbag2's MCAP writer repeats these records in the uncompressed summary.
    Chunks are skipped here; rosbag2_py independently decodes their messages.
    Files with no readable summary fail the schema check rather than passing
    based merely on rosbag2's type names.
    """
    schemas, channels = {}, {}
    def string(data, offset):
        size = struct.unpack_from('<I', data, offset)[0]
        end = offset + 4 + size
        if end > len(data):
            raise ValueError('Truncated MCAP string')
        return data[offset + 4:end], end
    with Path(path).open('rb') as stream:
        total = Path(path).stat().st_size
        if stream.read(8) != b'\x89MCAP0\r\n':
            raise ValueError('Invalid MCAP magic')
        while stream.tell() + 9 <= total - 8:
            opcode, size = struct.unpack('<BQ', stream.read(9))
            if stream.tell() + size > total - 8:
                raise ValueError('Truncated MCAP record')
            if opcode not in (3, 4):
                stream.seek(size, 1)
                continue
            if size > 32 * 1024 * 1024:
                raise ValueError('Unexpectedly large MCAP schema/channel record')
            data = stream.read(size)
            if opcode == 3:
                identity = struct.unpack_from('<H', data)[0]
                name, offset = string(data, 2)
                encoding, offset = string(data, offset)
                payload, _ = string(data, offset)
                schemas[identity] = dict(name=name.decode(), encoding=encoding.decode(), data_size=len(payload))
                if include_data:
                    schemas[identity]['data'] = payload.decode()
            else:
                identity, schema = struct.unpack_from('<HH', data)
                topic, offset = string(data, 4)
                encoding, _ = string(data, offset)
                channels[identity] = dict(topic=topic.decode(), schema_id=schema, message_encoding=encoding.decode())
    return schemas, channels


def audit_bag(output):
    """Inspect the recorded MCAP schemas and payload counts after recorder flush."""
    result = dict(files=[], topics={}, schema_errors=[], native_decoded_types={})
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        for path in sorted((Path(output) / 'bag').glob('*.mcap')):
            result['files'].append(dict(path=str(path), size=path.stat().st_size))
            schemas, channels = mcap_summary_schemas(path)
            reader = rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                        rosbag2_py.ConverterOptions('', ''))
            topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
            seen = set()
            while reader.has_next():
                topic, payload, _ = reader.read_next()
                entry = result['topics'].setdefault(topic, dict(count=0, type=topic_types[topic]))
                entry['count'] += 1
                if not payload:
                    result['schema_errors'].append(topic + ':empty_payload')
                seen.add(topic)
                type_name = topic_types[topic]
                if type_name not in result['native_decoded_types']:
                    if '/action/' in type_name:
                        import importlib
                        package, _, kind = type_name.split('/')
                        cls = getattr(importlib.import_module(package + '.action'), kind[:-len('_FeedbackMessage')]).Impl.FeedbackMessage
                    else:
                        cls = get_message(type_name)
                    deserialize_message(payload, cls)
                    result['native_decoded_types'][type_name] = dict(first_topic=topic, success=True)
            for topic in seen:
                matching = [channel for channel in channels.values() if channel['topic'] == topic]
                if not matching or any(not schemas.get(channel['schema_id'], {}).get('encoding')
                    or not schemas.get(channel['schema_id'], {}).get('data_size')
                    or channel['message_encoding'] != 'cdr' for channel in matching):
                    result['schema_errors'].append(topic)
        result['schema_errors'] = sorted(set(result['schema_errors']))
        if result['schema_errors']:
            from schema_recovery import recover
            restored = recover(output)
            covered = set(restored.get('topics', {}))
            result['raw_schema_errors'] = result['schema_errors']
            result['schema_errors'] = [topic for topic in result['schema_errors'] if topic not in covered]
            result['recovery'] = restored
        result['success'] = bool(result['files']) and not result['schema_errors']
        if result['success']:
            from schema_recovery import self_contained
            result['self_contained'] = self_contained(output)
    except Exception as error:
        result.update(success=False, error=str(error))
    write_json(Path(output) / 'schema_report.json', result)
    return result

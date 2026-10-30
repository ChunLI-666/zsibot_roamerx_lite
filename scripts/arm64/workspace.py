#!/usr/bin/env python3
"""Bounded native build of the paired ARM release repositories.

No checkout, source deletion, package installation, service operation, SDK or
UDP command is performed. Build caches are mutable; manifests name the actual
artifacts and their original build inputs. A host-validation prefix is never
accepted as an ARM build.
"""
from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import platform
import shlex
import shutil
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
NAV_REPO = HERE.parents[1]
SCHEMA = 1
NAV_PACKAGES = (
    'robots_dog_msgs', 'robot_debug_msgs', 'robot_debug_tools', 'navigo_epoch_msgs',
    'bondcpp', 'nav2_lifecycle_manager', 'navigo_util', 'navigo_core',
    'navigo_costmap_2d', 'navigo_map_server', 'navigo_navfn_planner',
    'navigo_path_planner', 'navigo_path_controller', 'navigo_mppi_controller',
    'navigo_behavior_tree', 'navigo_bt_navigator', 'navigo_behaviors',
    'navigo_velocity_optimizer', 'robot_navigo',
)
COMPILE_SUFFIXES = {'.c', '.cc', '.cpp', '.cxx', '.h', '.hh', '.hpp', '.hxx',
                    '.cmake', '.msg', '.srv', '.action', '.idl', '.in'}
IGNORED_PARTS = {'build', 'install', 'log', '.git', '__pycache__', '.pytest_cache',
                 'doc', 'docs', 'test', 'tests', 'regression'}


def digest(data):
    return hashlib.sha256(data).hexdigest()


def hash_json(value):
    return digest(json.dumps(value, sort_keys=True, separators=(',', ':')).encode())


def run_text(command, **kwargs):
    return subprocess.check_output(command, text=True, **kwargs).strip()


def git(repo, *args):
    return run_text(['git', '-C', str(repo), *args])


def atomic_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + '\n')
    temporary.replace(path)


def locate_workspace(explicit=None):
    if explicit:
        workspace = Path(explicit).expanduser().resolve()
        if not (workspace / 'src').is_dir():
            raise ValueError(f'Workspace must have a src directory: {workspace}')
        return workspace
    for candidate in NAV_REPO.parents:
        if (candidate / 'src').is_dir() and NAV_REPO.is_relative_to(candidate / 'src'):
            return candidate
    raise ValueError('Cannot locate workspace; pass --workspace /absolute/path')


def locate_localization(workspace, explicit=None):
    candidates = [Path(explicit).expanduser().resolve()] if explicit else [
        workspace / 'src' / 'LocMap', workspace / 'src' / 'lightning-lm']
    valid = [p for p in candidates if (p / 'package.xml').is_file()
             and ET.parse(p / 'package.xml').findtext('name') == 'lightning']
    if len(valid) != 1:
        raise ValueError('Specify exactly one LocMap checkout with --localization-repo')
    return valid[0]


def clean_environment(prefix=None):
    underlay = Path('/opt/ros/jazzy/setup.bash')
    if not underlay.is_file():
        raise ValueError('ROS 2 Jazzy underlay is required at /opt/ros/jazzy')
    # Do not inherit arbitrary nested ROS overlays, LD_PRELOAD or CPU policies.
    base = {k: os.environ[k] for k in ('HOME', 'USER', 'LANG', 'LC_ALL', 'TERM') if k in os.environ}
    base['PATH'] = '/usr/local/bin:/usr/bin:/bin'
    command = ['bash', '--noprofile', '--norc', '-c',
               'set -e; source "$1" >&2; if [ -n "$2" ]; then source "$2" >&2; fi; env -0',
               'arm-workspace', str(underlay), str(prefix / 'setup.bash') if prefix else '']
    raw = subprocess.check_output(command, env=base)
    result = dict(item.decode().split('=', 1) for item in raw.split(b'\0') if b'=' in item)
    result['ROBOT_NAVIGO_ARM_RUNTIME'] = '1'
    result['LIGHTNING_USE_EXTERNAL_LIVOX'] = '1'
    result['PYTHONDONTWRITEBYTECODE'] = '1'
    return result


def source_paths(repo):
    raw = subprocess.check_output(['git', '-C', str(repo), 'ls-files', '-z', '--cached',
                                   '--others', '--exclude-standard'])
    return sorted({Path(p.decode()) for p in raw.split(b'\0') if p})


def discover_packages(loc):
    packages = {}
    for relative in source_paths(NAV_REPO):
        if relative.name != 'package.xml' or relative.parts[0] != 'src':
            continue
        path = NAV_REPO / relative
        name = ET.parse(path).findtext('name')
        if name in NAV_PACKAGES:
            if name in packages:
                raise ValueError(f'Duplicate package {name}')
            packages[name] = path.parent
    missing = set(NAV_PACKAGES) - packages.keys()
    if missing:
        raise ValueError(f'Missing required navigation sources: {sorted(missing)}')
    packages['livox_ros_driver2'] = HERE / 'livox_interfaces'
    packages['lightning'] = loc
    return packages


def semantic_message(path):
    return [' '.join(line.split('#', 1)[0].split()) for line in path.read_text().splitlines()
            if line.split('#', 1)[0].strip()]


def check_livox(workspace, loc):
    source = loc / 'thirdparty' / 'livox_ros_driver'
    for name in ('CustomPoint', 'CustomMsg'):
        path = source / 'msg' / (name + '.msg')
        if not path.is_file():
            raise ValueError(f'Missing Livox interface {path}')
        driver = workspace / 'src' / 'livox_ros_driver2' / 'msg' / path.name
        if driver.is_file() and semantic_message(driver) != semantic_message(path):
            raise ValueError(f'Hardware driver and LocMap disagree on {path.name}')
    return source


def package_inputs(name, path, loc, repo_files):
    repo = loc if name == 'lightning' else NAV_REPO
    rel = path.relative_to(repo)
    compilation, runtime = {}, {}
    for filename in repo_files[repo]:
        if not filename.is_relative_to(rel):
            continue
        sub = filename.relative_to(rel)
        if not sub.parts or any(p in IGNORED_PARTS for p in sub.parts):
            continue
        full = repo / filename
        if not full.is_file():
            continue
        value = file_identity(full)
        is_compile = (sub.suffix in COMPILE_SUFFIXES or sub.name in ('CMakeLists.txt', 'package.xml')
                      or name == 'robot_debug_tools' and sub.suffix == '.py')
        (compilation if is_compile else runtime)[str(sub)] = value
    if name == 'livox_ros_driver2':
        for f in ('CustomPoint.msg', 'CustomMsg.msg'):
            compilation[f'canonical/{f}'] = digest(
                (loc / 'thirdparty/livox_ros_driver/msg' / f).read_bytes())
    return {'compile': hash_json(compilation), 'runtime': hash_json(runtime)}


def dependency_order(packages, scope):
    # Runtime plugins are explicitly present in NAV_PACKAGES. package.xml alone
    # cannot discover MPPI/Navfn/BT dynamic loading from parameters and XML.
    from catkin_pkg.package import parse_package
    graph = {}
    for name, path in packages.items():
        package = parse_package(str(path / 'package.xml'))
        package.evaluate_conditions(dict(os.environ, ROS_VERSION='2', ROS_DISTRO='jazzy',
                                         ROBOT_NAVIGO_ARM_RUNTIME='1', LIGHTNING_USE_EXTERNAL_LIVOX='1'))
        dependencies = set()
        for kind in ('build_depends', 'buildtool_depends', 'build_export_depends', 'exec_depends'):
            for dep in getattr(package, kind):
                if dep.evaluated_condition is not False and dep.name in packages and dep.name != name:
                    dependencies.add(dep.name)
        graph[name] = dependencies
    graph['lightning'].update(('robot_debug_msgs', 'robot_debug_tools', 'livox_ros_driver2'))
    graph['robot_navigo'].add('livox_ros_driver2')
    wanted = set(packages) if scope == 'all' else (
        set(NAV_PACKAGES) | {'livox_ros_driver2'} if scope == 'navigation' else {'lightning'})
    def add_dependencies(name):
        for dep in graph[name]:
            if dep not in wanted:
                wanted.add(dep)
                add_dependencies(dep)
    for name in list(wanted):
        add_dependencies(name)
    order = []
    while wanted:
        ready = sorted(name for name in wanted if graph[name].issubset(order))
        if not ready:
            raise ValueError(f'Cyclic local dependencies: {sorted(wanted)}')
        order.extend(ready)
        wanted.difference_update(ready)
    return order, graph


def toolchain_identity(env, architecture):
    # dpkg versions cover the packaged ROS/compiler/math dependency set. Local
    # /usr/local dependencies require an explicit rebuild when changed.
    return {'architecture': architecture, 'ros': 'jazzy',
            'compiler': run_text(['c++', '--version'], env=env).splitlines()[0],
            'cmake': run_text(['cmake', '--version'], env=env).splitlines()[0],
            'ninja': run_text(['ninja', '--version'], env=env),
            'packages_sha256': digest(subprocess.check_output(
                ['dpkg-query', '-W', '-f=${Package} ${Version}\n'], env=env)),
            'profile': {'localization_only': True, 'ui': False, 'external_livox': True,
                        'navigation_runtime': True, 'build_type': 'Release', 'testing': False}}


def process_memory(root_pid):
    """Conservative sum of descendant RSS, including shared pages per process."""
    rows = {}
    for proc in Path('/proc').glob('[0-9]*'):
        try:
            fields = (proc / 'stat').read_text().rsplit(')', 1)[1].split()
            rows[int(proc.name)] = (int(fields[1]), int(fields[21]) * os.sysconf('SC_PAGE_SIZE'))
        except (OSError, ValueError, IndexError):
            continue
    selected = {root_pid}
    changed = True
    while changed:
        children = {pid for pid, (parent, _) in rows.items() if parent in selected}
        changed = not children.issubset(selected)
        selected.update(children)
    return sum(rows[pid][1] for pid in selected if pid in rows)


def measured_run(command, env, cwd, log_path):
    started = time.monotonic()
    peak = 0
    print('+ ' + shlex.join(map(str, command)), flush=True)
    with log_path.open('wb') as log:
        process = subprocess.Popen(command, env=env, cwd=cwd, stdout=log,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        try:
            while process.poll() is None:
                peak = max(peak, process_memory(process.pid))
                time.sleep(0.25)
        except BaseException:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=10)
            raise
    row = {'command': list(map(str, command)), 'returncode': process.returncode,
           'wall_seconds': time.monotonic() - started, 'peak_descendant_rss_sum_bytes': peak,
           'memory_measurement': '250ms samples; RSS sum includes shared pages multiple times',
           'log': str(log_path)}
    if process.returncode:
        print(log_path.read_text(errors='replace')[-10000:], file=sys.stderr)
    return row


def file_identity(path):
    return {'sha256': digest(path.read_bytes()) if path.is_file() else None,
            'mode': path.stat().st_mode & 0o777 if path.exists() else None,
            'symlink': os.readlink(path) if path.is_symlink() else None}


def artifact_files(prefix):
    return {str(p.relative_to(prefix)): file_identity(p) for p in sorted(prefix.rglob('*'))
            if (p.is_file() or p.is_symlink()) and '__pycache__' not in p.parts
            and not p.name.endswith('.pyc')}



def environment_files(prefix):
    return {p.name: file_identity(p) for p in sorted(prefix.iterdir())
            if p.is_file() or p.is_symlink()} if prefix.is_dir() else {}


def artifacts_match(prefix, expected):
    return bool(expected) and prefix.is_dir() and artifact_files(prefix) == expected


def source_identity(repo):
    return {'path': str(repo), 'branch': git(repo, 'branch', '--show-current'),
            'commit': git(repo, 'rev-parse', 'HEAD'),
            'dirty': bool(git(repo, 'status', '--porcelain=v1'))}


def prepare(args):
    workspace = locate_workspace(args.workspace)
    loc = locate_localization(workspace, args.localization_repo)
    architecture = platform.machine()
    if architecture not in ('aarch64', 'arm64') and not args.host_validation:
        raise ValueError('This entry builds native ARM64. Use --host-validation only for isolated host tests.')
    label = f'{architecture}-jazzy-runtime'
    root = Path(args.output).expanduser().resolve() if args.output else workspace / '.arm64' / label
    if (root == workspace or root.is_relative_to(workspace / 'src')
            or root.is_relative_to(NAV_REPO) or root.is_relative_to(loc)
            or any(root.is_relative_to(workspace / d) for d in ('build', 'install', 'log'))):
        raise ValueError('Output must be an independent build directory')
    if root.exists() and any(root.iterdir()) and not (root / '.arm64-build-owner.json').is_file():
        raise ValueError('Nonempty output directory is not owned by this ARM build tool')
    for child in ('build', 'install', 'runs'):
        path = root / child
        if path.is_symlink() or not path.resolve().is_relative_to(root):
            raise ValueError(f'Output subdirectory must remain inside its owned root: {path}')
    return workspace, loc, root, architecture


def remove_previous_install(root, name):
    prefix = root / 'install' / name
    if prefix.is_symlink() or not prefix.resolve().is_relative_to(root.resolve()):
        raise ValueError(f'Package prefix must not be a symlink: {prefix}')
    manifest = root / 'build' / name / 'install_manifest.txt'
    if manifest.is_file():
        for entry in manifest.read_text().splitlines():
            path = Path(os.path.abspath(entry))
            if (not path.is_relative_to(prefix.absolute())
                    or not path.parent.resolve().is_relative_to(prefix.resolve())):
                raise ValueError(f'Previous CMake install escaped package prefix: {path}')
            if path.is_file() or path.is_symlink():
                path.unlink()


def build(args):
    workspace, loc, root, architecture = prepare(args)
    env = clean_environment()
    for tool in ('colcon', 'cmake', 'ninja', 'c++'):
        if not shutil.which(tool, path=env['PATH']):
            raise ValueError(f'Missing build dependency {tool}; see doc/arm64_deployment.md')
    livox = check_livox(workspace, loc)
    packages = discover_packages(loc)
    order, graph = dependency_order(packages, args.scope)
    identity = toolchain_identity(env, architecture)
    state_path = root / 'state.json'
    old_bytes = state_path.read_bytes() if state_path.is_file() else None
    old = json.loads(old_bytes) if old_bytes is not None else {}
    if old and old.get('toolchain') != identity:
        raise ValueError('Toolchain/dependency set changed; choose a new --output directory for a cold build.')
    repo_files = {repo: source_paths(repo) for repo in (NAV_REPO, loc)}
    inputs = {n: package_inputs(n, packages[n], loc, repo_files) for n in order}
    rules_digest = digest(Path(__file__).read_bytes())
    signatures, actions = {}, {}
    for name in order:
        signatures[name] = hash_json({'input': inputs[name]['compile'], 'toolchain': identity,
                                      'build_rules': rules_digest,
                                      'dependencies': {d: signatures[d] for d in sorted(graph[name])}})
        cached = old.get('packages', {}).get(name, {})
        if (cached.get('signature') != signatures[name] or args.rebuild
                or not artifacts_match(root / 'install' / name, cached.get('artifacts'))):
            actions[name] = 'build'
        elif cached.get('runtime') != inputs[name]['runtime']:
            actions[name] = 'install'
        else:
            actions[name] = 'reuse'
    plan = {'schema': SCHEMA, 'scope': args.scope, 'output': str(root),
            'build_rules_sha256': rules_digest,
            'host_validation': architecture not in ('aarch64', 'arm64'),
            'repositories': [source_identity(loc), source_identity(NAV_REPO)],
            'toolchain': identity, 'packages': actions, 'jobs': args.jobs,
            'ccache': shutil.which('ccache', path=env['PATH']),
            'hardware_driver': 'external; this overlay provides message types only'}
    print(json.dumps(plan, indent=2), flush=True)
    if args.plan:
        return 0
    root.mkdir(parents=True, exist_ok=True)
    lock_path = workspace / '.arm64-build.lock'
    with lock_path.open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise ValueError('Another localization/navigation workspace build holds the global lock')
        if (state_path.read_bytes() if state_path.is_file() else None) != old_bytes:
            raise ValueError('Build state changed during planning; retry for a fresh plan')
        atomic_json(root / '.arm64-build-owner.json', {'schema': SCHEMA, 'architecture': architecture})
        # A failed/interrupted invocation never replaces the last successful state.
        run_dir = root / 'runs' / time.strftime('%Y%m%d_%H%M%S')
        run_dir.mkdir(parents=True, exist_ok=False)
        report = dict(plan, success=False, stages=[])
        atomic_json(run_dir / 'report.json', report)
        atomic_json(root / 'manifest.json', report)
        # Remove only files owned by the previous CMake install. Preserve
        # colcon-generated package environment/ordering metadata on install-only.
        for name in order:
            if actions[name] == 'build':
                remove_previous_install(root, name)
        env.update(CMAKE_BUILD_PARALLEL_LEVEL=str(args.jobs), MAKEFLAGS=f'-j{args.jobs}',
                   CCACHE_DIR=str(workspace / '.arm64' / 'ccache' / architecture))
        selected = [n for n in order if actions[n] == 'build']
        if selected:
            command = ['colcon', '--log-base', str(run_dir / 'colcon'), 'build',
                       '--base-paths', *map(str, packages.values()),
                       '--build-base', str(root / 'build'), '--install-base', str(root / 'install'),
                       '--executor', 'sequential', '--packages-select', *selected,
                       '--cmake-force-configure',
                       '--event-handlers', 'console_direct+', '--cmake-args', '-GNinja',
                       '-DCMAKE_BUILD_TYPE=Release', '-DBUILD_TESTING=OFF',
                       '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON',
                       '-DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF',
                       '-DCMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY=OFF',
                       '-DROBOT_NAVIGO_ARM_RUNTIME=ON', '-DLIGHTNING_BOARD_BUILD=ON',
                       '-DLIGHTNING_LOCALIZATION_ONLY=ON', '-DLIGHTNING_WITH_UI=OFF',
                       '-DLIGHTNING_BUILD_DEV_TOOLS=OFF', '-DLIGHTNING_USE_EXTERNAL_LIVOX=ON',
                       f'-DLIVOX_INTERFACE_SOURCE={livox}',
                       f'-DCMAKE_JOB_POOLS=compile={args.jobs};link=1',
                       '-DCMAKE_JOB_POOL_COMPILE=compile', '-DCMAKE_JOB_POOL_LINK=link']
            if plan['ccache']:
                command.extend([f'-DCMAKE_C_COMPILER_LAUNCHER={plan["ccache"]}',
                                f'-DCMAKE_CXX_COMPILER_LAUNCHER={plan["ccache"]}'])
            if (root / 'install/setup.bash').is_file():
                cached_env = clean_environment(root / 'install')
                cached_env.update({k: env[k] for k in ('CMAKE_BUILD_PARALLEL_LEVEL', 'MAKEFLAGS', 'CCACHE_DIR')})
            else:
                cached_env = env
            row = measured_run(command, cached_env, workspace, run_dir / 'build.log')
            report['stages'].append(row)
            atomic_json(run_dir / 'report.json', report)
            if row['returncode']:
                return row['returncode']
        for name in order:
            if actions[name] == 'install':
                remove_previous_install(root, name)
                row = measured_run(['cmake', '--install', str(root / 'build' / name)], env,
                                   workspace, run_dir / (name + '_install.log'))
                report['stages'].append(row)
                if row['returncode']:
                    atomic_json(run_dir / 'report.json', report)
                    return row['returncode']
        # Detect edits made by another task while the compiler was running.
        current_files = {repo: source_paths(repo) for repo in (NAV_REPO, loc)}
        if (digest(Path(__file__).read_bytes()) != rules_digest
                or [source_identity(loc), source_identity(NAV_REPO)] != plan['repositories']
                or any(package_inputs(n, packages[n], loc, current_files) != inputs[n] for n in order)):
            raise ValueError('Source inputs changed during build; no successful state recorded. Re-run after edits finish.')
        state = {'schema': SCHEMA, 'toolchain': identity, 'packages': dict(old.get('packages', {}))}
        for name in order:
            previous = old.get('packages', {}).get(name, {})
            state['packages'][name] = {'signature': signatures[name], 'runtime': inputs[name]['runtime'],
                'dependencies': {d: signatures[d] for d in sorted(graph[name])},
                'artifacts': artifact_files(root / 'install' / name),
                'built_from': source_identity(loc if name == 'lightning' else NAV_REPO)
                    if actions[name] == 'build' else previous['built_from']}
        state['environment_files'] = environment_files(root / 'install')
        report['success'] = True
        report['state_sha256'] = hash_json(state)
        report['artifact_build_sources'] = {n: state['packages'][n]['built_from'] for n in order}
        atomic_json(state_path, state)
        atomic_json(run_dir / 'report.json', report)
        atomic_json(root / 'manifest.json', report)
        print(f'Build complete: {root / "manifest.json"}', flush=True)
    return 0


def verified_state(root, architecture, names=None):
    try:
        manifest = json.loads((root / 'manifest.json').read_text())
        state = json.loads((root / 'state.json').read_text())
    except (OSError, ValueError) as error:
        raise ValueError('No verified successful workspace build') from error
    if (not manifest.get('success') or manifest.get('state_sha256') != hash_json(state)
            or state.get('toolchain', {}).get('architecture') != architecture):
        raise ValueError('Build manifest/state/architecture mismatch; rebuild before launch')
    if state.get('environment_files') != environment_files(root / 'install'):
        raise ValueError('Workspace environment scripts changed; rebuild before launch')
    for name in names if names is not None else state['packages']:
        item = state['packages'].get(name)
        if not item or not artifacts_match(root / 'install' / name, item['artifacts']):
            raise ValueError(f'Package artifacts unavailable or changed: {name}')
        for dep, signature in item.get('dependencies', {}).items():
            if state['packages'].get(dep, {}).get('signature') != signature:
                raise ValueError(f'{name} was built against an older {dep}; rebuild the full workspace')
    return state


def check(args):
    _, loc, root, architecture = prepare(args)
    verified_state(root, architecture)
    env = clean_environment(root / 'install')
    packages = discover_packages(loc)
    order, _ = dependency_order(packages, args.scope)
    state = json.loads((root / 'state.json').read_text())
    failures, rows = [], []
    for name in order:
        prefix = root / 'install' / name
        actual = run_text(['ros2', 'pkg', 'prefix', name], env=env)
        okay = Path(actual).resolve() == prefix.resolve() and artifacts_match(
            prefix, state.get('packages', {}).get(name, {}).get('artifacts'))
        rows.append({'package': name, 'prefix': actual, 'matches': okay})
        if not okay:
            failures.append(f'{name}: prefix or artifact mismatch')
    for path in (root / 'install').rglob('*'):
        if not path.is_file() or path.suffix in ('.a', '.pyc'):
            continue
        with path.open('rb') as stream:
            magic = stream.read(4)
        if magic != b'\x7fELF':
            continue
        header = run_text(['readelf', '-h', str(path)])
        expected = 'AArch64' if architecture in ('aarch64', 'arm64') else 'X86-64'
        if expected not in header:
            failures.append(f'Wrong ELF architecture: {path}')
        linked = subprocess.run(['ldd', str(path)], env=env, capture_output=True, text=True)
        if 'not found' in linked.stdout:
            failures.append(f'Missing library: {path}: {linked.stdout}')
    # Required runtime plugin package is deliberately supplied by the underlay.
    smoother = run_text(['ros2', 'pkg', 'prefix', 'nav2_smoother'], env=env)
    if not Path(smoother).is_relative_to('/opt/ros/jazzy'):
        failures.append('nav2_smoother must resolve to the Jazzy underlay')
    result = {'success': not failures, 'packages': rows, 'failures': failures,
              'host_validation': architecture not in ('aarch64', 'arm64'),
              'scope': 'static artifact/prefix/library verification; no hardware or motion'}
    atomic_json(root / 'check.json', result)
    print(json.dumps(result, indent=2))
    return int(bool(failures))


def start(args):
    workspace, loc, root, architecture = prepare(args)
    lock = (workspace / '.arm64-build.lock').open('a')
    try:
        fcntl.flock(lock, fcntl.LOCK_SH | fcntl.LOCK_NB)
    except BlockingIOError:
        raise ValueError('Workspace is being built; wait before launching')
    state = verified_state(root, architecture)
    required = ({'lightning', 'robot_debug_msgs', 'robot_debug_tools', 'livox_ros_driver2'}
                if args.command == 'localization' else set(NAV_PACKAGES) | {'livox_ros_driver2'})
    if not required.issubset(state['packages']):
        raise ValueError(f'Missing runtime packages: {sorted(required - state["packages"].keys())}')
    os.set_inheritable(lock.fileno(), True)  # Keep shared lock through ros2 foreground process.
    env = clean_environment(root / 'install')
    # Keep user-selected ROS networking for a real launch, unlike build isolation.
    for key in ('ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'ROS_AUTOMATIC_DISCOVERY_RANGE',
                'ROS_STATIC_PEERS', 'RMW_IMPLEMENTATION', 'CYCLONEDDS_URI',
                'FASTRTPS_DEFAULT_PROFILES_FILE'):
        if key in os.environ:
            env[key] = os.environ[key]
    if args.command == 'localization':
        config = Path(args.config).expanduser().resolve()
        if not config.is_file():
            raise ValueError(f'Missing configuration: {config}')
        command = ['ros2', 'run', 'lightning', 'run_loc_online', '--config', str(config),
                   '--ros-args', '-p', f'use_sim_time:={str(args.sim_time).lower()}']
    else:
        map_file = Path(args.map).expanduser().resolve()
        if not map_file.is_file():
            raise ValueError(f'Missing navigation map: {map_file}')
        command = ['ros2', 'launch', 'robot_navigo', 'lightning_nav_bringup.launch.py',
                   f'map:={map_file}', f'use_sim_time:={str(args.sim_time).lower()}',
                   'enable_waypoint_follower:=false', 'enable_livox_converter:=false']
        if args.params:
            params = Path(args.params).expanduser().resolve()
            if not params.is_file():
                raise ValueError(f'Missing navigation parameters: {params}')
            command.append(f'params_file:={params}')
    command.extend(args.ros_args)
    print(shlex.join(command), flush=True)
    if args.dry_run:
        return 0
    # Preserve caller CWD: relative paths inside a site YAML keep their original semantics.
    os.execvpe(command[0], command, env)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('build', 'check', 'localization', 'navigation'))
    parser.add_argument('--workspace')
    parser.add_argument('--localization-repo')
    parser.add_argument('--output', help='Persistent profile-specific output; never an existing general install')
    parser.add_argument('--host-validation', action='store_true', help='Explicit x86 test-only build/run, never ARM acceptance')
    parser.add_argument('--scope', choices=('all', 'localization', 'navigation'), default='all')
    parser.add_argument('--jobs', type=int, default=2, help='Total compiler job budget; packages run sequentially')
    parser.add_argument('--plan', action='store_true', help='Show build/reuse plan without writing or compiling')
    parser.add_argument('--rebuild', action='store_true', help='Visit every selected package without deleting build caches')
    parser.add_argument('--config', help='Explicit site Lightning YAML for localization')
    parser.add_argument('--map', help='Explicit 2D map YAML for navigation')
    parser.add_argument('--params', help='Explicit site navigation parameters; default is the installed reviewed YAML')
    parser.add_argument('--sim-time', action='store_true', help='Use /clock only for replay/simulation')
    parser.add_argument('--dry-run', action='store_true', help='Print a launch command without starting ROS processes')
    values = list(sys.argv[1:] if argv is None else argv)
    split = values.index('--') if '--' in values else len(values)
    args = parser.parse_args(values[:split])
    args.ros_args = values[split + 1:]
    try:
        if args.jobs < 1 or args.jobs > 8:
            raise ValueError('--jobs must be between 1 and 8; choose from measured memory budget')
        if args.command == 'localization' and not args.config:
            raise ValueError('--config is required')
        if args.command == 'navigation' and not args.map:
            raise ValueError('--map is required')
        if args.ros_args and args.command not in ('localization', 'navigation'):
            raise ValueError('Extra ROS arguments apply only to launch commands')
        return {'build': build, 'check': check, 'localization': start, 'navigation': start}[args.command](args)
    except (ValueError, OSError, subprocess.CalledProcessError) as error:
        print(f'ARM workspace: {error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())

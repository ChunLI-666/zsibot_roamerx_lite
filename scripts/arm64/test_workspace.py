"""Offline contract tests: no compiler, ROS process, hardware or network is used."""
import argparse
import importlib.util
import json
import itertools
from pathlib import Path

import pytest


SPEC = importlib.util.spec_from_file_location('arm_workspace_under_test',
                                           Path(__file__).with_name('workspace.py'))
workspace = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(workspace)
# Retain the actual classifier for planning tests whose compiler fixture mocks
# package_inputs; classification itself must still read real source paths.
ACTUAL_PACKAGE_INPUTS = workspace.package_inputs


def package(directory, name, dependencies=''):
    path = directory / name
    path.mkdir(parents=True)
    (path / 'package.xml').write_text(
        f'<package format="3"><name>{name}</name><version>1.0.0</version>'
        '<description>Offline fixture</description>'
        '<maintainer email="fixture@example.test">Fixture</maintainer>'
        f'<license>Apache-2.0</license>{dependencies}</package>')
    return path


@pytest.fixture
def graph_packages(tmp_path, monkeypatch):
    dependencies = {
        'robot_debug_msgs': '', 'robot_debug_tools': '<depend>robot_debug_msgs</depend>',
        'livox_ros_driver2': '', 'lightning': '', 'robot_navigo': '',
        'messages': '', 'core': '<depend>messages</depend>',
        'controller': '<depend>core</depend>',
    }
    monkeypatch.setattr(workspace, 'NAV_PACKAGES', tuple(
        name for name in dependencies if name not in ('lightning', 'livox_ros_driver2')))
    return {name: package(tmp_path, name, deps) for name, deps in dependencies.items()}


def test_localization_scope_only_adds_required_interfaces(graph_packages):
    order, graph = workspace.dependency_order(graph_packages, 'localization')
    assert set(order) == {'lightning', 'robot_debug_msgs', 'robot_debug_tools', 'livox_ros_driver2'}
    assert order.index('robot_debug_msgs') < order.index('robot_debug_tools') < order.index('lightning')
    assert graph['lightning'] == {'robot_debug_msgs', 'robot_debug_tools', 'livox_ros_driver2'}


def test_navigation_scope_covers_explicit_plugins_and_orders_dependencies(graph_packages):
    order, graph = workspace.dependency_order(graph_packages, 'navigation')
    assert set(order) == set(graph_packages) - {'lightning'}
    for name in order:
        assert all(order.index(dep) < order.index(name) for dep in graph[name])


def test_conditional_development_dependency_is_not_added(graph_packages):
    path = graph_packages['robot_navigo'] / 'package.xml'
    path.write_text(path.read_text().replace('</package>',
        '<exec_depend condition="$ROBOT_NAVIGO_ARM_RUNTIME != \'1\'">lightning</exec_depend>'
        '</package>'))
    order, graph = workspace.dependency_order(graph_packages, 'navigation')
    assert 'lightning' not in graph['robot_navigo']
    assert 'lightning' not in order


def test_dependency_cycle_is_rejected(graph_packages):
    path = graph_packages['core'] / 'package.xml'
    path.write_text(path.read_text().replace('</package>', '<depend>controller</depend></package>'))
    with pytest.raises(ValueError, match='Cyclic'):
        workspace.dependency_order(graph_packages, 'all')


@pytest.fixture
def input_fixture(tmp_path, monkeypatch):
    nav, loc = tmp_path / 'navigation', tmp_path / 'LocMap'
    nav.mkdir()
    loc.mkdir()
    monkeypatch.setattr(workspace, 'NAV_REPO', nav)

    def snapshot(name, relative, content, mode=None):
        package_root = loc if name == 'lightning' else nav / name
        path = package_root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
        if mode is not None:
            path.chmod(mode)
        files = {repo: [f.relative_to(repo) for f in repo.rglob('*') if f.is_file()]
                 for repo in (nav, loc)}
        return ACTUAL_PACKAGE_INPUTS(name, package_root, loc, files)
    return snapshot


@pytest.mark.parametrize('name,relative', [
    ('controller', 'src/controller.cpp'), ('controller', 'include/controller.hpp'),
    ('messages', 'msg/State.msg'), ('controller', 'CMakeLists.txt'),
    ('controller', 'package.xml'), ('robot_debug_tools', 'scripts/generate_build_stamp.py'),
    ('robot_navigo', 'scripts/nav_safety_gate.py'),
    ('lightning', 'scripts/run_loc_online_entry.py'),
])
def test_compile_inputs_invalidate_build_signature(input_fixture, name, relative):
    first = input_fixture(name, relative, 'first')
    second = input_fixture(name, relative, 'second')
    assert first['compile'] != second['compile']


@pytest.mark.parametrize('name,relative', [
    ('robot_navigo', 'params/site.yaml'), ('robot_navigo', 'launch/entry.launch.py'),
    ('robot_navigo', 'scripts/helper'), ('robot_navigo', 'scripts/other.py'),
    ('lightning', 'config/site.yaml'), ('lightning', 'launch/entry.launch.py'),
    ('lightning', 'scripts/other.py'),
])
def test_runtime_only_changes_do_not_invalidate_compilation(input_fixture, name, relative):
    first = input_fixture(name, relative, 'first')
    second = input_fixture(name, relative, 'second')
    assert first['compile'] == second['compile']
    assert first['runtime'] != second['runtime']


@pytest.mark.parametrize('relative', ['docs/guide.md', 'test/check.cpp', 'build/generated.cpp'])
def test_non_runtime_material_does_not_invalidate_package(input_fixture, relative):
    first = input_fixture('controller', relative, 'first')
    assert input_fixture('controller', relative, 'second') == first


def test_executable_source_mode_is_part_of_runtime_identity(input_fixture):
    first = input_fixture('robot_navigo', 'scripts/helper', '#!/bin/sh\nexit 0\n', 0o644)
    second = input_fixture('robot_navigo', 'scripts/helper', '#!/bin/sh\nexit 0\n', 0o755)
    assert first['runtime'] != second['runtime']


def test_build_options_after_command_are_parsed(monkeypatch):
    calls = []
    monkeypatch.setattr(workspace, 'build', lambda args: calls.append(args) or 0)
    assert workspace.main(['build', '--jobs', '1', '--scope', 'localization', '--plan']) == 0
    assert calls[0].jobs == 1 and calls[0].scope == 'localization' and calls[0].plan
    assert not calls[0].ros_args


def test_launch_forwards_only_explicit_remaining_arguments(monkeypatch):
    calls = []
    monkeypatch.setattr(workspace, 'start', lambda args: calls.append(args) or 0)
    assert workspace.main(['navigation', '--map', '/tmp/map with spaces.yaml', '--dry-run',
                           '--', 'namespace:=test']) == 0
    assert calls[0].map == '/tmp/map with spaces.yaml'
    assert calls[0].dry_run and calls[0].ros_args == ['namespace:=test']


def test_unknown_build_option_is_rejected(monkeypatch):
    monkeypatch.setattr(workspace, 'build', lambda args: pytest.fail('must not build on typo'))
    with pytest.raises(SystemExit) as error:
        workspace.main(['build', '--jbos', '1'])
    assert error.value.code == 2


@pytest.mark.parametrize('jobs', ['0', '9'])
def test_out_of_budget_jobs_never_invoke_build(monkeypatch, jobs):
    monkeypatch.setattr(workspace, 'build', lambda args: pytest.fail('invalid job budget'))
    assert workspace.main(['build', '--jobs', jobs]) == 2


def test_artifact_contents_and_executable_mode_are_checked(tmp_path):
    prefix = tmp_path / 'install'
    prefix.mkdir()
    script = prefix / 'run'
    script.write_text('#!/bin/sh\nexit 0\n')
    script.chmod(0o755)
    expected = workspace.artifact_files(prefix)
    assert workspace.artifacts_match(prefix, expected)
    script.chmod(0o644)
    assert not workspace.artifacts_match(prefix, expected)
    script.chmod(0o755)
    script.write_text('#!/bin/sh\nexit 1\n')
    assert not workspace.artifacts_match(prefix, expected)


def test_artifact_symlink_target_is_checked_even_when_bytes_match(tmp_path):
    prefix = tmp_path / 'install'
    prefix.mkdir()
    for name in ('original', 'replacement'):
        (prefix / name).write_text('same bytes')
    link = prefix / 'current'
    link.symlink_to('original')
    expected = workspace.artifact_files(prefix)
    link.unlink()
    link.symlink_to('replacement')
    assert not workspace.artifacts_match(prefix, expected)


@pytest.mark.parametrize('manifest', [None, {'success': False}, {'success': True}])
def test_start_rejects_missing_failed_or_unverified_manifest(tmp_path, monkeypatch, manifest):
    if manifest is not None:
        (tmp_path / 'manifest.json').write_text(json.dumps(manifest))
    monkeypatch.setattr(workspace, 'prepare', lambda args: (tmp_path, tmp_path, tmp_path, 'aarch64'))
    monkeypatch.setattr(workspace, 'clean_environment',
                        lambda *args: pytest.fail('unverified artifacts must not be sourced'))
    args = argparse.Namespace(command='localization', dry_run=True, config='/tmp/config.yaml',
                              ros_args=[], sim_time=False, scope='all')
    with pytest.raises(ValueError):
        workspace.start(args)


@pytest.fixture
def cached_build(tmp_path, monkeypatch):
    """Exercise planning/state transitions with the compiler fully replaced."""
    nav, loc, output = tmp_path / 'src/navigation', tmp_path / 'src/LocMap', tmp_path / 'output'
    nav.mkdir(parents=True)
    loc.mkdir()
    monkeypatch.setattr(workspace, 'NAV_REPO', nav)
    names = ['robot_debug_msgs', 'lightning', 'robot_navigo']
    packages = {name: (loc if name == 'lightning' else nav / name) for name in names}
    graph = {'robot_debug_msgs': set(), 'lightning': {'robot_debug_msgs'},
             'robot_navigo': {'robot_debug_msgs'}}
    inputs = {name: {'compile': workspace.hash_json([name, 'compile']),
                     'runtime': workspace.hash_json([name, 'runtime'])} for name in names}
    source_version = {'commit': '1' * 40}
    monkeypatch.setattr(workspace, 'prepare', lambda args: (tmp_path, loc, output, 'aarch64'))
    monkeypatch.setattr(workspace, 'clean_environment', lambda *args: {'PATH': '/usr/bin'})
    monkeypatch.setattr(workspace.shutil, 'which', lambda tool, **kwargs: '/usr/bin/' + tool)
    monkeypatch.setattr(workspace, 'check_livox', lambda *args: loc / 'thirdparty/livox_ros_driver')
    monkeypatch.setattr(workspace, 'discover_packages', lambda *args: packages)
    monkeypatch.setattr(workspace, 'dependency_order', lambda *args: (names, graph))
    monkeypatch.setattr(workspace, 'toolchain_identity', lambda *args: {'architecture': 'aarch64'})
    monkeypatch.setattr(workspace, 'source_paths', lambda *args: [])
    monkeypatch.setattr(workspace, 'package_inputs', lambda name, *args: dict(inputs[name]))
    monkeypatch.setattr(workspace, 'source_identity', lambda repo: {
        'path': str(repo), 'branch': 'release/arm64-fixture',
        'commit': source_version['commit'], 'dirty': False})
    sequence = itertools.count()
    monkeypatch.setattr(workspace.time, 'strftime', lambda *args: 'run_' + str(next(sequence)))

    def fake_compiler(command, env, cwd, log_path):
        assert command[0] == 'colcon', 'fixture expects a selected-package build'
        selected = list(itertools.takewhile(
            lambda value: not value.startswith('--'),
            command[command.index('--packages-select') + 1:]))
        for name in selected:
            prefix = output / 'install' / name
            prefix.mkdir(parents=True, exist_ok=True)
            (prefix / 'artifact').write_text('fixture binary for ' + name + inputs[name]['compile'])
        for name in ('setup.bash', 'local_setup.bash', '_local_setup_util_sh.py'):
            (output / 'install' / name).write_text('# offline environment fixture\n')
        return {'returncode': 0, 'command': command, 'wall_seconds': 0}

    monkeypatch.setattr(workspace, 'measured_run', fake_compiler)
    # Any missed subprocess stub is a test failure, never a real compiler launch.
    monkeypatch.setattr(workspace.subprocess, 'Popen',
                        lambda *args, **kwargs: pytest.fail('external process forbidden in fixture'))
    args = argparse.Namespace(scope='all', jobs=1, rebuild=False, plan=False)
    assert workspace.build(args) == 0
    return args, names, inputs, source_version, output


def planned_actions(cached_build, capsys):
    args = cached_build[0]
    args.plan = True
    capsys.readouterr()
    assert workspace.build(args) == 0
    return json.loads(capsys.readouterr().out)['packages']


def test_unchanged_inputs_reuse_all_packages(cached_build, capsys):
    assert set(planned_actions(cached_build, capsys).values()) == {'reuse'}


def test_interface_change_invalidates_transitive_consumers(cached_build, capsys):
    cached_build[2]['robot_debug_msgs']['compile'] = 'changed message schema'
    assert set(planned_actions(cached_build, capsys).values()) == {'build'}


def test_leaf_source_change_does_not_rebuild_other_modules(cached_build, capsys):
    cached_build[2]['lightning']['compile'] = 'changed localization implementation'
    assert planned_actions(cached_build, capsys) == {
        'robot_debug_msgs': 'reuse', 'lightning': 'build', 'robot_navigo': 'reuse'}


def test_configuration_change_only_reinstalls_runtime(cached_build, capsys):
    cached_build[2]['robot_navigo']['runtime'] = 'changed site config'
    assert planned_actions(cached_build, capsys) == {
        'robot_debug_msgs': 'reuse', 'lightning': 'reuse', 'robot_navigo': 'install'}


@pytest.mark.parametrize('name,relative,action', [
    ('robot_navigo', 'scripts/nav_safety_gate.py', 'build'),
    ('lightning', 'scripts/run_loc_online_entry.py', 'build'),
    ('robot_navigo', 'params/site.yaml', 'install'),
    ('robot_navigo', 'launch/entry.launch.py', 'install'),
    ('lightning', 'config/site.yaml', 'install'),
    ('lightning', 'launch/entry.launch.py', 'install'),
])
def test_real_source_change_plans_only_required_package_action(
        cached_build, capsys, name, relative, action):
    args, names, inputs, _, _ = cached_build
    packages = workspace.discover_packages(None)
    loc, nav = packages['lightning'], workspace.NAV_REPO
    source = packages[name] / relative
    source.parent.mkdir(parents=True, exist_ok=True)
    source.write_text('original execution source or resource')

    def actual_inputs():
        files = {repo: [f.relative_to(repo) for f in repo.rglob('*') if f.is_file()]
                 for repo in (nav, loc)}
        return ACTUAL_PACKAGE_INPUTS(name, packages[name], loc, files)

    inputs[name] = actual_inputs()
    assert workspace.build(args) == 0  # Fixture compiler only, freezes the real input hashes.
    source.write_text('updated execution source or resource')
    inputs[name] = actual_inputs()
    assert planned_actions(cached_build, capsys) == {
        package_name: action if package_name == name else 'reuse' for package_name in names}


def test_reused_binary_keeps_original_build_commit(cached_build, capsys):
    args, names, inputs, source_version, output = cached_build
    original = json.loads((output / 'state.json').read_text())
    source_version['commit'] = '2' * 40
    assert workspace.build(args) == 0
    state = json.loads((output / 'state.json').read_text())
    assert all(state['packages'][name]['built_from'] == original['packages'][name]['built_from']
               for name in names)
    manifest = json.loads((output / 'manifest.json').read_text())
    assert all(row['commit'] == '2' * 40 for row in manifest['repositories'])
    assert all(row['commit'] == '1' * 40 for row in manifest['artifact_build_sources'].values())


def test_partial_interface_update_rejects_older_installed_consumer(cached_build, monkeypatch):
    args, names, inputs, _, output = cached_build
    graph = {'robot_debug_msgs': set(), 'lightning': {'robot_debug_msgs'},
             'robot_navigo': {'robot_debug_msgs'}}
    monkeypatch.setattr(workspace, 'dependency_order', lambda packages, scope: (
        [name for name in names if scope == 'all' or name != 'lightning'], graph))
    initial = workspace.verified_state(output, 'aarch64')
    inputs['robot_debug_msgs']['compile'] = 'new interface requiring consumer rebuild'
    args.scope = 'navigation'
    assert workspace.build(args) == 0
    mixed = json.loads((output / 'state.json').read_text())
    assert mixed['packages']['lightning'] == initial['packages']['lightning']
    # Every individual artifact is intact; the package dependency contracts are
    # incompatible. Hash verification alone must not declare the overlay valid.
    assert all(workspace.artifacts_match(output / 'install' / name, row['artifacts'])
               for name, row in mixed['packages'].items())
    with pytest.raises(ValueError, match='lightning.*older robot_debug_msgs'):
        workspace.verified_state(output, 'aarch64')
    args.scope = 'all'
    assert workspace.build(args) == 0
    assert workspace.verified_state(output, 'aarch64')['packages']['lightning'] != initial['packages']['lightning']


def test_head_change_during_build_rejects_success_even_with_same_source_bytes(cached_build, monkeypatch):
    args, _, _, source_version, output = cached_build
    before = (output / 'state.json').read_bytes()
    compiler = workspace.measured_run

    def change_head(*values):
        result = compiler(*values)
        source_version['commit'] = '2' * 40
        return result

    monkeypatch.setattr(workspace, 'measured_run', change_head)
    args.rebuild = True
    with pytest.raises(ValueError, match='Source inputs changed during build'):
        workspace.build(args)
    assert (output / 'state.json').read_bytes() == before
    assert not json.loads((output / 'manifest.json').read_text())['success']
    with pytest.raises(ValueError, match='manifest/state/architecture mismatch'):
        workspace.verified_state(output, 'aarch64')


def test_failed_build_cannot_reuse_previous_success_manifest(cached_build, monkeypatch):
    args, _, inputs, _, output = cached_build
    before = (output / 'state.json').read_bytes()
    inputs['lightning']['compile'] = 'new source whose compilation fails'

    def failed_compiler(command, env, cwd, log_path):
        prefix = output / 'install/lightning'
        prefix.mkdir(parents=True, exist_ok=True)
        (prefix / 'artifact').write_text('incomplete replacement')
        return {'returncode': 17, 'command': command, 'wall_seconds': 0}

    monkeypatch.setattr(workspace, 'measured_run', failed_compiler)
    assert workspace.build(args) == 17
    assert (output / 'state.json').read_bytes() == before
    assert not json.loads((output / 'manifest.json').read_text())['success']
    with pytest.raises(ValueError, match='manifest/state/architecture mismatch'):
        workspace.verified_state(output, 'aarch64')


@pytest.mark.parametrize('damage', ['state', 'architecture', 'artifact'])
def test_success_manifest_does_not_accept_modified_state_or_artifacts(cached_build, damage):
    _, _, _, _, output = cached_build
    assert workspace.verified_state(output, 'aarch64')
    if damage == 'state':
        state_path = output / 'state.json'
        state = json.loads(state_path.read_text())
        state['packages']['lightning']['built_from']['commit'] = '3' * 40
        state_path.write_text(json.dumps(state))
    elif damage == 'artifact':
        (output / 'install/lightning/artifact').write_text('altered binary')
    with pytest.raises(ValueError):
        workspace.verified_state(output, 'x86_64' if damage == 'architecture' else 'aarch64')


def test_install_refresh_removes_old_cmake_outputs_but_keeps_colcon_metadata(tmp_path):
    prefix = tmp_path / 'install/fixture'
    old_config = prefix / 'share/fixture/params/removed.yaml'
    metadata = prefix / 'share/colcon-core/packages/fixture'
    package_hook = prefix / 'share/fixture/package.dsv'
    for path in (old_config, metadata, package_hook):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(path.name)
    manifest = tmp_path / 'build/fixture/install_manifest.txt'
    manifest.parent.mkdir(parents=True)
    manifest.write_text(str(old_config) + '\n')
    workspace.remove_previous_install(tmp_path, 'fixture')
    assert not old_config.exists()
    assert metadata.read_text() == 'fixture'
    assert package_hook.read_text() == 'package.dsv'


def test_install_refresh_rejects_manifest_entry_outside_owned_prefix(tmp_path):
    outside = tmp_path / 'unrelated_site_config.yaml'
    outside.write_text('must survive')
    manifest = tmp_path / 'build/fixture/install_manifest.txt'
    manifest.parent.mkdir(parents=True)
    manifest.write_text(str(outside) + '\n')
    with pytest.raises(ValueError, match='escaped package prefix'):
        workspace.remove_previous_install(tmp_path, 'fixture')
    assert outside.read_text() == 'must survive'


@pytest.mark.parametrize('name', ['setup.bash', 'local_setup.bash', '_local_setup_util_sh.py'])
def test_modified_workspace_environment_is_rejected_before_sourcing(cached_build, monkeypatch, name):
    _, _, _, _, output = cached_build
    before = workspace.verified_state(output, 'aarch64')
    assert name in before['environment_files']
    (output / 'install' / name).write_text('# changed environment can select different binaries\n')
    # Package files remain correct, so this must use the independent top-level
    # environment fingerprint rather than only per-package artifact hashes.
    assert all(workspace.artifacts_match(output / 'install' / package_name, item['artifacts'])
               for package_name, item in before['packages'].items())
    monkeypatch.setattr(workspace, 'clean_environment',
                        lambda *args: pytest.fail('altered environment must not be sourced'))
    args = argparse.Namespace(command='localization', dry_run=True, config='/tmp/config.yaml',
                              ros_args=[], sim_time=False, scope='all')
    with pytest.raises(ValueError, match='Workspace environment scripts changed'):
        workspace.start(args)


@pytest.mark.parametrize('command,target', [('localization', 'lightning'), ('navigation', 'robot_navigo')])
def test_start_requires_target_module_even_when_other_installed_packages_verify(
        cached_build, monkeypatch, command, target):
    _, _, _, _, output = cached_build
    state = workspace.verified_state(output, 'aarch64')
    monkeypatch.setattr(workspace, 'NAV_PACKAGES', (
        'robot_debug_msgs', 'robot_debug_tools', 'robot_navigo'))
    # Complete the minimal fixture's interface set; all required packages except
    # the deliberately omitted target remain present and internally consistent.
    for name in ('robot_debug_tools', 'livox_ros_driver2'):
        prefix = output / 'install' / name
        prefix.mkdir(parents=True)
        (prefix / 'artifact').write_text('fixture interface ' + name)
        state['packages'][name] = {
            'signature': name, 'dependencies': {},
            'artifacts': workspace.artifact_files(prefix),
        }
    state['packages'].pop(target)
    workspace.atomic_json(output / 'state.json', state)
    manifest = json.loads((output / 'manifest.json').read_text())
    manifest['state_sha256'] = workspace.hash_json(state)
    workspace.atomic_json(output / 'manifest.json', manifest)
    assert workspace.verified_state(output, 'aarch64')
    monkeypatch.setattr(workspace, 'clean_environment',
                        lambda *args: pytest.fail('incomplete runtime must not be sourced'))
    args = argparse.Namespace(command=command, dry_run=True, config='/tmp/config.yaml',
                              map='/tmp/map.yaml', ros_args=[], sim_time=False, scope='all')
    with pytest.raises(ValueError, match='Missing runtime packages:.*' + target):
        workspace.start(args)


def test_runtime_only_dependency_remains_available_while_consumer_rebuilds(cached_build, monkeypatch):
    args, _, inputs, _, output = cached_build
    dependency = output / 'install/robot_debug_msgs/artifact'
    manifest = output / 'build/robot_debug_msgs/install_manifest.txt'
    manifest.parent.mkdir(parents=True)
    manifest.write_text(str(dependency) + '\n')
    inputs['robot_debug_msgs']['runtime'] = 'changed installed non-compile resource'
    inputs['lightning']['compile'] = 'consumer implementation update'
    compiler = workspace.measured_run

    def compile_or_install(command, env, cwd, log_path):
        if command[0] == 'colcon':
            # The real consumer configure needs dependency Config.cmake, headers
            # and libraries, even if that dependency only needs reinstallation.
            assert dependency.is_file(), 'runtime-only dependency was removed before consumer build'
            return compiler(command, env, cwd, log_path)
        assert command[:2] == ['cmake', '--install']
        assert Path(command[2]).name == 'robot_debug_msgs'
        dependency.write_text('reinstalled unchanged dependency compilation')
        return {'returncode': 0, 'command': command, 'wall_seconds': 0}

    monkeypatch.setattr(workspace, 'measured_run', compile_or_install)
    assert workspace.build(args) == 0
    assert workspace.verified_state(output, 'aarch64')

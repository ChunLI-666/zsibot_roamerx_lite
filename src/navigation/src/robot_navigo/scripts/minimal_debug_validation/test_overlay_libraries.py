"""Merged/isolated artifact identity and headless environment, without ROS."""
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

import common
import replay


def package_index(monkeypatch, prefixes):
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', SimpleNamespace(
        get_package_prefix=lambda name: str(prefixes[name]),
        get_packages_with_prefixes=lambda: prefixes.copy()))


def library(prefix, name, contents=b'fixed library'):
    path = prefix / 'lib' / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(contents)
    return path


@pytest.mark.parametrize('isolated', [False, True])
@pytest.mark.parametrize('package,name', [
    ('nav2_lifecycle_manager', 'libnav2_lifecycle_manager_core.so'),
    ('bondcpp', 'libbondcpp.so')])
def test_identity_uses_exact_index_prefix_and_rejects_identical_foreign_copy(
        monkeypatch, tmp_path, isolated, package, name):
    root = tmp_path / 'install'
    prefix = root / package if isolated else root
    package_index(monkeypatch, {package: prefix})
    expected = library(prefix, name)
    foreign = library(tmp_path / 'old_install', name, expected.read_bytes())
    result = common.selected_library_identity(expected, root, name)
    assert result['valid'] and result['expected'] == str(expected)
    assert not common.selected_library_identity(foreign, root, name)['valid']


def test_shadowed_package_index_does_not_fall_back_to_existing_library(monkeypatch, tmp_path):
    root = tmp_path / 'install'
    expected = library(root / 'bondcpp', 'libbondcpp.so')
    package_index(monkeypatch, {'bondcpp': tmp_path / 'old_install/bondcpp'})
    with pytest.raises(RuntimeError, match='outside the selected'):
        common.selected_library_identity(expected, root, expected.name)


@pytest.mark.parametrize('isolated', [False, True])
@pytest.mark.parametrize('foreign_copy', [False, True])
def test_mapped_core_bond_and_plugin_require_selected_artifacts(
        monkeypatch, tmp_path, isolated, foreign_copy):
    workspace = tmp_path / 'workspace'
    root = workspace / 'artifacts/install'
    packages = ('nav2_lifecycle_manager', 'bondcpp', 'navigo_costmap_2d')
    prefixes = {name: root / name if isolated else root for name in packages}
    package_index(monkeypatch, prefixes)
    core = library(prefixes['nav2_lifecycle_manager'], 'libnav2_lifecycle_manager_core.so')
    bond = library(prefixes['bondcpp'], 'libbondcpp.so')
    plugin = library(prefixes['navigo_costmap_2d'], 'libnav2_costmap_2d_core.so')
    if foreign_copy:
        # Equal bytes cannot turn a stale prefix into the selected artifact.
        plugin = library(workspace / 'old_install', plugin.name, plugin.read_bytes())
    upstream_bond = library(tmp_path / 'underlay', 'libbondcpp.so', b'upstream smoother bond')
    maps = {100: [core, bond, plugin], 200: [upstream_bond]}
    original_read = Path.read_text
    def read_text(path, *args, **kwargs):
        if path.parent.parent == Path('/proc') and path.name == 'maps':
            return '\n'.join('0000-1000 r-xp 0000 00:00 0 ' + str(item)
                             for item in maps[int(path.parent.name)])
        return original_read(path, *args, **kwargs)
    monkeypatch.setattr(Path, 'read_text', read_text)
    monkeypatch.setattr(common, 'WORKSPACE', workspace)
    monkeypatch.setattr(common.subprocess, 'check_output', lambda *a, **k: '100 1\n200 100\n')
    processes = common.Processes(tmp_path)
    processes.items = [('navigation', SimpleNamespace(pid=100), None)]
    if foreign_copy:
        with pytest.raises(RuntimeError, match='DSOs differ'):
            processes.loaded_libraries(root)
    else:
        report = processes.loaded_libraries(root)
        assert not report['mismatched_overlay_libraries']
        assert report['lifecycle_cores'][0]['valid']
        assert report['manager_bondcpp'][0]['valid']
        assert report['manager_bondcpp'][0]['pid'] == 100
        observed = next(row for row in report['libraries'] if row['path'] == str(plugin))
        assert observed['selected_overlay_path'] == str(plugin)


@pytest.mark.parametrize('arm_runtime', [False, True])
def test_bootstrap_only_injects_pangolin_for_ordinary_profile(monkeypatch, tmp_path, arm_runtime):
    workspace = tmp_path / 'workspace'
    pangolin = workspace / 'src/lightning-lm/thirdparty/Pangolin-0.9.3/build'
    pangolin.mkdir(parents=True)
    install = tmp_path / 'install'
    install.mkdir()
    (install / 'setup.bash').write_text('# mock setup\n')
    args = common.arguments('test').parse_args([
        '--install-prefix', str(install), '--output', str(tmp_path / 'output'), '--domain', '123',
        *(['--arm-runtime'] if arm_runtime else [])])
    monkeypatch.setattr(common, 'WORKSPACE', workspace)
    for name in ('LD_PRELOAD', 'MAGICK_THREAD_LIMIT', 'MINIMAL_VALIDATION_ENV'):
        monkeypatch.delenv(name, raising=False)
    monkeypatch.setenv('LOCNAV_PROFILE', '0')
    monkeypatch.setenv('LOCNAV_STAGE_WITNESS', '0')
    monkeypatch.setenv('LD_LIBRARY_PATH', '/selected/lib')
    class ReexecCaptured(Exception):
        pass
    captured = {}
    def execvpe(binary, argv, env):
        captured.update(env)
        raise ReexecCaptured()
    monkeypatch.setattr(common.os, 'execvpe', execvpe)
    with pytest.raises(ReexecCaptured):
        common.bootstrap(args)
    expected = '/selected/lib' if arm_runtime else str(pangolin) + ':/selected/lib'
    assert captured['LD_LIBRARY_PATH'] == expected


@pytest.mark.parametrize('arm_runtime', [False, True])
def test_replay_only_injects_pangolin_for_ordinary_profile(monkeypatch, tmp_path, arm_runtime):
    # Match the pre-existing legacy fallback path calculation exactly.
    script = tmp_path / 'a/b/c/d/e/f/g/h/replay.py'
    monkeypatch.setattr(replay, '__file__', str(script))
    pangolin = script.resolve().parents[8] / 'lightning-lm/thirdparty/Pangolin-0.9.3/build'
    pangolin.mkdir(parents=True)
    monkeypatch.setenv('LD_LIBRARY_PATH', '/selected/lib')
    env = replay.localization_environment(SimpleNamespace(arm_runtime=arm_runtime))
    expected = '/selected/lib' if arm_runtime else str(pangolin) + ':/selected/lib'
    assert env['LD_LIBRARY_PATH'] == expected

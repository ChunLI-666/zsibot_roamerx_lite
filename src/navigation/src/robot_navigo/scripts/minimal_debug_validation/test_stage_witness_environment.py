"""Environment/provenance mocks only; no ROS, native compilation or replay."""
import os
import sys
from types import SimpleNamespace

import pytest

import common


@pytest.mark.parametrize('value', [None, '0', '1'])
def test_stage_witness_guard_allows_explicit_diagnostics(monkeypatch, value):
    monkeypatch.delenv('LOCNAV_STAGE_WITNESS', raising=False)
    if value is not None:
        monkeypatch.setenv('LOCNAV_STAGE_WITNESS', value)
    monkeypatch.setenv('LOCNAV_PROFILE', '0')
    monkeypatch.delenv('LD_PRELOAD', raising=False)
    monkeypatch.delenv('MAGICK_THREAD_LIMIT', raising=False)
    common.require_default_environment()
    assert os.environ.get('LOCNAV_STAGE_WITNESS') == value


@pytest.mark.parametrize('value', ['', 'true', 'yes', '2', '-1', ' 1', '1 '])
def test_stage_witness_invalid_value_is_rejected_before_start(monkeypatch, value):
    monkeypatch.setenv('LOCNAV_PROFILE', '0')
    monkeypatch.setenv('LOCNAV_STAGE_WITNESS', value)
    with pytest.raises(RuntimeError, match='LOCNAV_STAGE_WITNESS must be 0 or 1'):
        common.require_default_environment()


@pytest.mark.parametrize('variable', ['LD_PRELOAD', 'MAGICK_THREAD_LIMIT'])
def test_stage_diagnostic_does_not_bypass_workaround_rejection(monkeypatch, variable):
    monkeypatch.setenv('LOCNAV_PROFILE', '0')
    monkeypatch.setenv('LOCNAV_STAGE_WITNESS', '1')
    monkeypatch.setenv(variable, 'unapproved')
    with pytest.raises(RuntimeError, match='temporary overrides'):
        common.require_default_environment()


@pytest.mark.parametrize('value', [None, '0', '1'])
@pytest.mark.parametrize('profile', [False, True])
def test_manifest_preserves_raw_stage_value_and_separates_default_environment(
        monkeypatch, tmp_path, value, profile):
    overlay = tmp_path / 'overlay'
    overlay.mkdir()
    packages = SimpleNamespace(get_package_prefix=lambda name: str(overlay))
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', packages)
    for relative in ('lib/robot_navigo/livox_scan_projector',
                     'lib/robot_navigo/nav_safety_gate.py',
                     'lib/navigo_path_controller/controller_server',
                     'lib/navigo_path_planner/planner_server',
                     'lib/navigo_bt_navigator/bt_navigator',
                     'lib/nav2_smoother/smoother_server',
                     'lib/libnav2_lifecycle_manager_core.so', 'lib/libbondcpp.so'):
        path = overlay / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('non-ELF mocked runtime identity')
    monkeypatch.delenv('LOCNAV_STAGE_WITNESS', raising=False)
    if value is not None:
        monkeypatch.setenv('LOCNAV_STAGE_WITNESS', value)
    monkeypatch.setenv('LOCNAV_PROFILE', '1' if profile else '0')
    monkeypatch.setenv('LOCNAV_PROFILE_BACKEND', 'resources')
    monkeypatch.delenv('LD_PRELOAD', raising=False)
    monkeypatch.delenv('MAGICK_THREAD_LIMIT', raising=False)
    args = SimpleNamespace(install_prefix=overlay, output=tmp_path, domain=123,
                           observer_off=True, no_composition=False)
    report = common.runtime_manifest(args, 'closed_loop')
    enabled = value == '1'
    assert report['environment']['LOCNAV_STAGE_WITNESS'] == value
    assert report['stage_witness_enabled'] is enabled
    assert report['profiling_enabled'] is profile
    assert report['default_environment'] is (not profile and not enabled)
    assert report['temporary_environment_overrides_absent']

"""Offline deployment contract checks; importing the launch starts no nodes."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

PACKAGE=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('forward_test_launch',PACKAGE/'launch/rockdog_forward_test.launch.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def profile():
    return module.build_profile(PACKAGE/'params/navigo_params.yaml',PACKAGE)


def test_forward_profile_preserves_global_planner_and_critic_weights():
    base=yaml.safe_load((PACKAGE/'params/navigo_params.yaml').read_text())
    result=profile()
    assert result['planner_server']==base['planner_server']
    original=base['controller_server']['ros__parameters']['FollowPath']
    current=result['controller_server']['ros__parameters']['FollowPath']
    for critic in original['critics']:
        assert current[critic]==original[critic]
    assert current['forward_alignment']['enabled'] and current['vx_min']==0 and current['vy_max']==0


def test_stopped_goal_progress_and_executable_rotation_are_compatible():
    params=profile()['controller_server']['ros__parameters']
    goal=params['general_goal_checker'];motion=params['FollowPath']['forward_alignment']
    assert goal['plugin']=='navigo_path_controller::StoppedGoalChecker'
    assert goal['stateful'] is False
    assert goal['trans_stopped_velocity']==goal['rot_stopped_velocity']==.01
    assert motion['angular_acceleration']/params['controller_frequency'] >= motion['min_angular_velocity']
    progress=params['progress_checker']
    assert progress['plugin']=='navigo_path_controller::PoseProgressChecker'
    assert progress['required_movement_angle']/motion['min_angular_velocity'] < progress['movement_time_allowance']


def test_no_motion_recovery_and_no_negative_smoother_output():
    result=profile();bt=result['bt_navigator']['ros__parameters']
    assert result['behavior_server']['ros__parameters']['behavior_plugins']==['wait']
    for key in ('default_nav_to_pose_bt_xml','default_nav_through_poses_bt_xml'):
        tags={e.tag for e in ET.parse(bt[key]).iter()}
        assert 'FollowPath' in tags
        assert not tags & {'BackUp','Spin','DriveOnHeading'}
    smooth=result['velocity_smoother']['ros__parameters']
    assert smooth['min_velocity'][0]==0
    assert smooth['min_velocity'][1]==smooth['max_velocity'][1]==0
    assert smooth['velocity_timeout']==.25


def test_model_footprint_applies_identically_to_both_maps():
    result=profile();model=yaml.safe_load((PACKAGE/'params/zsl1_model_envelope.yaml').read_text())
    maps=[result[name][name]['ros__parameters'] for name in ('local_costmap','global_costmap')]
    for m in maps:
        assert yaml.safe_load(m['footprint'])==model['footprint']
        assert m['footprint_padding']==model['footprint_padding']
        assert m['inflation_layer']['inflation_radius']==.5


def test_late_overlay_overrides_reverse_in_custom_base(tmp_path):
    base=yaml.safe_load((PACKAGE/'params/navigo_params.yaml').read_text())
    base['controller_server']['ros__parameters']['FollowPath']['vx_min']=-1.
    base['velocity_smoother']['ros__parameters']['min_velocity'][0]=-1.
    path=tmp_path/'custom.yaml';path.write_text(yaml.safe_dump(base))
    result=module.build_profile(path,PACKAGE)
    assert result['controller_server']['ros__parameters']['FollowPath']['vx_min']==0
    assert result['velocity_smoother']['ros__parameters']['min_velocity'][0]==0


def test_custom_base_cannot_select_incompatible_checker_or_prediction_limits(tmp_path):
    base=yaml.safe_load((PACKAGE/'params/navigo_params.yaml').read_text())
    cp=base['controller_server']['ros__parameters']
    cp['goal_checker_plugins']=['other_checker']
    cp['controller_plugins']=['other_controller']
    cp['FollowPath'].update(vx_max=1.,wz_max=2.,motion_model='Ackermann',iteration_count=3)
    path=tmp_path/'custom.yaml';path.write_text(yaml.safe_dump(base))
    result=module.build_profile(path,PACKAGE)['controller_server']['ros__parameters']
    assert result['goal_checker_plugins']==['general_goal_checker']
    assert result['controller_plugins']==['FollowPath']
    assert result['FollowPath']['vx_max']==.15
    assert result['FollowPath']['wz_max']==.1
    assert result['FollowPath']['motion_model']=='Omni'
    assert result['FollowPath']['iteration_count']==1


def test_field_test_forces_complete_non_epoch_chain_even_on_epoch_base(tmp_path):
    base=yaml.safe_load((PACKAGE/'params/navigo_params.yaml').read_text())
    nodes=('bt_navigator','controller_server','velocity_smoother','nav_safety_gate')
    for name in nodes:
        base.setdefault(name,{}).setdefault('ros__parameters',{})['enable_epoch_contract']=True
    base['controller_server']['ros__parameters']['enable_epoch_backup']=True
    base['bt_navigator']['ros__parameters']['enable_epoch_backup_recovery']=True
    path=tmp_path/'epoch.yaml';path.write_text(yaml.safe_dump(base))
    result=module.build_profile(path,PACKAGE)
    for name in nodes:
        assert result[name]['ros__parameters']['enable_epoch_contract'] is False
    assert result['controller_server']['ros__parameters']['enable_epoch_backup'] is False
    assert result['bt_navigator']['ros__parameters']['enable_epoch_backup_recovery'] is False

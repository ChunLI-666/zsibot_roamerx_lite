"""Offline AST/byte/installed-API evidence; creates no ROS context or process."""
import ast
import hashlib
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent
BASE=next(p for p in HERE.parents if p.name=='planner_comparison_20261105')
CAL=BASE/'full_scene_lateral_calibration_v1/review_source_v2'
IMU=BASE/'sim_imu_monotonic_gate_v1/review_source_v1'
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def definitions(path):return {n.name:n for n in ast.parse(path.read_text()).body if isinstance(n,(ast.FunctionDef,ast.ClassDef))}
def dump(node):return ast.dump(node,include_attributes=False)
def main():
    checks={}
    for name in ['stat_birth','child','interrupt']:checks['owned_'+name+'_AST_exact']=dump(definitions(CAL/'lateral_once.py')[name])==dump(definitions(HERE/'owned_recorder.py')[name])
    for name in ['sha','read','finite','wrap','rpy','claim','host_proof','StableStop']:checks['contract_'+name+'_AST_exact']=dump(definitions(CAL/'lateral_contract.py')[name])==dump(definitions(HERE/'readiness_contract.py')[name])
    checks['original_imu_converter_function_AST_exact']=dump(definitions(IMU/'matrix_sensor_contract.py')['imu_to_local'])==dump(definitions(HERE/'imu_converter.py')['imu_to_local'])
    for name,original in [('recording_deadline.py',CAL/'recording_deadline.py'),('bag_summary.py',CAL/'bag_summary.py'),('imu_gate.py',IMU/'imu_gate.py'),('sensor_config.json',CAL/'sensor_config.json')]:checks['exact_bytes_'+name]=sha(HERE/name)==sha(original)
    release=json.loads((HERE/'root_release.HOLD.json').read_text());config=json.loads((HERE/'config.HOLD.json').read_text())
    checks['exact_nine_runtime_source_hashes']=len(release['package_sha256'])==9 and all(sha(HERE/name)==value for name,value in release['package_sha256'].items())
    checks['HOLD_case_release_graph_binding']=config['executable'] is False and release['allow_execute'] is False and config['root_expected_source_node_names_reviewed'] is False
    checks['fixed_discovery3_data10_launcher45_fallback60']=config['discovery_sec']==3. and config['duration_sec']==10. and config['launcher_deadline_sec']==45. and config['max_runtime_sec']==60.
    new_sources=['readiness_capture.py','readiness_contract.py','readiness_validator.py','readiness_observer.py','owned_recorder.py','imu_converter.py']
    nodes=[node for name in new_sources for node in ast.walk(ast.parse((HERE/name).read_text()))]
    checks['no_application_publishers_clients_actions_sockets']=not any(isinstance(n,ast.Attribute) and n.attr in ['create_publisher','publish','create_client','create_service','send_goal','send_goal_async','socket','sendto','kill','killpg'] for n in nodes)
    observer=(HERE/'readiness_observer.py').read_text()
    checks['standard_ROS_message_imports_only']=all(s not in observer for s in ['livox_ros_driver2','cv2','PIL','image_transport'])
    checks['no_callback_publisher_gid_attribute_assumption']='info.publisher_gid' not in observer and 'isinstance(info,dict)' in observer
    primary=(HERE/'primary_rclpy_subscription_7.1.12.cpp').read_text()
    checks['versioned_primary_callback_dict_has_no_gid']='pytaken_msg, py::dict(' in primary and '"publication_sequence_number"_a' in primary and '"reception_sequence_number"_a' in primary and 'publisher_gid' not in primary
    owner=definitions(HERE/'readiness_capture.py')['run'];calls=[n for n in ast.walk(owner) if isinstance(n,ast.Call)]
    owned=[n for n in calls if isinstance(n.func,ast.Attribute) and n.func.attr=='owned_command']
    checks['sole_owned_role_recorder']=len(owned)==1 and ast.literal_eval(owned[0].args[0])=='recorder'
    sends=[n for n in calls if isinstance(n.func,ast.Name) and n.func.id=='interrupt']
    checks['only_parent_signal_target_owned_recorder']=len(sends)==1 and isinstance(sends[0].args[0],ast.Name) and sends[0].args[0].id=='recorder'
    text=(HERE/'readiness_capture.py').read_text()
    checks['arm_ACK_magic_before_observer']=text.index('guard=deadline.arm(output,45.)')<text.index("guard.wait_for_child('recorder'")<text.index('if not opened:raise')<text.index('observer=Observer(')
    checks['shutdown_signal_closed_audit_order']=text.index('guard.begin_shutdown(')<text.index('interrupt(recorder,')<text.index('guard.recording_closed(')<text.index('bag_summary.audit(')
    checks['never_motion_authority']='ready_for_motion=False' in text and 'no_motion_authority=True' in text
    installed={}
    for suffix in ['share/rclpy/package.xml','lib/python3.12/site-packages/rclpy/subscription.py','lib/python3.12/site-packages/rclpy/executors.py','lib/python3.12/site-packages/rclpy/topic_endpoint_info.py']:
        path=Path('/opt/ros/jazzy')/suffix
        installed[str(path)]=dict(sha256=sha(path))
    checks['actual_installed_rclpy7.1.12']='<version>7.1.12</version>' in Path('/opt/ros/jazzy/share/rclpy/package.xml').read_text()
    report=dict(state='PASS' if all(checks.values()) else 'FAIL',checks=checks,check_count=len(checks),installed_primary_API_files=installed,
        primary_reference='https://github.com/ros2/rclpy/blob/7.1.12/rclpy/src/rclpy/subscription.cpp#L182',primary_sha256=sha(HERE/'primary_rclpy_subscription_7.1.12.cpp'),
        no_ROS_Node_runtime_probe=True,per_message_GID_not_available_in_this_API=True)
    (HERE/'source_audit_v1.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    return 0 if all(checks.values()) else 1
if __name__=='__main__':raise SystemExit(main())

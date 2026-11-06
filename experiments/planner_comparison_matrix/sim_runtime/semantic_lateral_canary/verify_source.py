"""Offline source-delta verification. Imports no runtime package or ROS modules."""
import ast
import difflib
import hashlib
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent
EVIDENCE=next(path for path in HERE.parents if path.name=='planner_comparison_20261105')
CAL=EVIDENCE/'full_scene_lateral_calibration_v1/review_source_v2'
IMU=EVIDENCE/'sim_imu_monotonic_gate_v1/review_source_v1'
BACKEND=EVIDENCE/'sim_semantic_lateral_canary_v1/review_source_v1'

def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def tree(path):return ast.parse(path.read_text())
def dump(node):return ast.dump(node,include_attributes=False)
def definitions(path):return {node.name:node for node in tree(path).body if isinstance(node,(ast.FunctionDef,ast.ClassDef))}
def nested(function,name):return next(node for node in function.body if isinstance(node,ast.FunctionDef) and node.name==name)

def main():
    checks={};old=definitions(CAL/'lateral_contract.py');new=definitions(HERE/'lateral_contract.py')
    for name in ['Hold','sha','read','finite','wrap','body_xy','rpy','claim','package_hashes','host_proof','patch_check','check_live','StableStop']:
        checks['unchanged_contract_'+name]=dump(old[name])==dump(new[name])
    original=old['validate']
    for statement in original.body:
        if isinstance(statement,ast.If) and isinstance(statement.test,ast.BoolOp):
            statement.test.values=[value for value in statement.test.values if "matrix_mc_udp_test.py" not in ast.unparse(value)]
    checks['validate_only_unused_MC_sha_clause_removed']=dump(original)==dump(new['validate'])
    old=definitions(CAL/'lateral_once.py');new=definitions(HERE/'lateral_once.py')
    for name in ['stat_birth','child','interrupt','main']:
        checks['unchanged_parent_'+name]=dump(old[name])==dump(new[name])
    for name in ['watch','wait_stop']:
        checks['unchanged_parent_nested_'+name]=dump(nested(old['run'],name))==dump(nested(new['run'],name))
    old_metric=ast.get_source_segment((CAL/'lateral_once.py').read_text(),old['metrics'])
    expected_metric=old_metric.replace('def metrics(history,trace_path):','def metrics(history,trace_path,sign):').replace("sends=[json.loads(line) for line in trace_path.read_text().splitlines()]",'sends=semantic_sends(trace_path,sign)').replace("['LX']","['lateral']").replace("sign=-float(first['command']['lateral']);",'')
    checks['metric_GT_stop_gap_body_preserved_after_trace_sign_adaptation']=expected_metric==ast.get_source_segment((HERE/'lateral_once.py').read_text(),new['metrics'])
    checks['parent_no_legacy_Observer_class']='Observer' not in new
    checks['Observer_class_identical_to_gate_freeze']=dump(definitions(IMU/'calibration_observer.py')['Observer'])==dump(definitions(HERE/'calibration_observer.py')['Observer'])
    checks['Observer_only_active_contract_import_changed']=(IMU/'calibration_observer.py').read_text().replace('import calibration_contract_frozen as c','import lateral_contract as c')==(HERE/'calibration_observer.py').read_text()
    for name,base in [('semantic_tool_timed.py',BACKEND),('imu_gate.py',IMU),('recording_deadline.py',CAL),('bag_summary.py',CAL)]:
        checks['unchanged_bytes_'+name]=digest(HERE/name)==digest(base/name)
    for name in ['acquisition_oracle.npz','sensor_config.json']:
        checks['unchanged_material_'+name]=digest(HERE/name)==digest(CAL/name)
    old_host=(CAL/'host_preflight.py').read_text()
    checks['host_only_semantic_forbidden_name_added']=old_host.replace("'matrix_mc_udp_test.py','official_tool_timed.py'","'matrix_mc_udp_test.py','official_tool_timed.py','semantic_tool_timed.py'")==(HERE/'host_preflight.py').read_text()
    contract=tree(HERE/'lateral_contract.py')
    scope=next(node.value.value for node in contract.body if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='SCOPE' for t in node.targets))
    checks['scope_semantic_only']=scope=='MAPPING_SEMANTIC_LATERAL_CANARY_ONLY'
    case=json.loads((HERE/'case.HOLD.json').read_text());release=json.loads((HERE/'root_release.HOLD.json').read_text())
    checks['prepared_pair_is_fixed_positive_then_negative']=case['probes']==[.2,-.2]
    checks['prepared_case_release_host_are_HOLD']=(case['executable'] is False and release['allow_execute'] is False and json.loads((HERE/'host_input.HOLD.json').read_text())['root_inputs_reviewed'] is False)
    expected_names={'lateral_once.py','lateral_contract.py','semantic_tool_timed.py','host_preflight.py','calibration_observer.py','imu_gate.py','recording_deadline.py','bag_summary.py'}
    checks['release_exact_eight_source_hashes']=set(release['package_sha256'])==expected_names and all(digest(HERE/name)==value for name,value in release['package_sha256'].items())
    for name in expected_names:
        tree(HERE/name)
    checks['all_eight_runtime_sources_parse']=True
    diff=''
    for name in ['lateral_once.py','lateral_contract.py','host_preflight.py']:
        diff+=''.join(difflib.unified_diff((CAL/name).read_text().splitlines(True),(HERE/name).read_text().splitlines(True),fromfile='immutable_cal_v2/'+name,tofile='combined_v1/'+name))
    diff+=''.join(difflib.unified_diff((IMU/'calibration_observer.py').read_text().splitlines(True),(HERE/'calibration_observer.py').read_text().splitlines(True),fromfile='immutable_gate_v1/calibration_observer.py',tofile='combined_v1/calibration_observer.py'))
    (HERE/'source_delta_v1.diff').write_text(diff)
    report=dict(state='PASS' if all(checks.values()) else 'FAIL',checks=checks,check_count=len(checks),no_runtime_imports=True,no_ROS_Node_socket_Popen_SSH=True)
    (HERE/'source_audit_v1.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    return 0 if all(checks.values()) else 1

if __name__=='__main__':raise SystemExit(main())

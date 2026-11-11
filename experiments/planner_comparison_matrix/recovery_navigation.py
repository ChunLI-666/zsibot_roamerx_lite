"""Opt-in recovery runtime configuration, artifacts and publisher ownership."""
import hashlib
import copy
import json
from pathlib import Path
import uuid
from xml.sax.saxutils import quoteattr

ERROR_CODE_NAMES = ['follow_path_error_code', 'compute_path_error_code', 'recovery_error_code']


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def artifacts(prefix, receipt_path):
    prefix=Path(prefix).resolve(); receipt=Path(receipt_path).resolve()
    data=json.loads(receipt.read_text())
    if (data.get('schema')!='matrix.recovery.build_receipt.v1' or data.get('install_prefix')!=str(prefix)
            or data.get('source_files_match_commit') is not True):
        raise ValueError('Recovery install requires commit-bound build receipt')
    files=data.get('artifact_sha256',{})
    required={'lib/libmatrix_recovery_nodes.so','lib/matrix_recovery_bt/zero_handoff_gate',
              'lib/libmatrix_recovery_bt__rosidl_typesupport_cpp.so'}
    if not required<=files.keys():raise ValueError('Recovery build receipt lacks required artifacts')
    for name,digest in files.items():
        path=(prefix/name).resolve()
        if not path.is_relative_to(prefix) or not path.is_file() or sha(path)!=digest:
            raise ValueError('Recovery artifact differs: '+name)
    return dict(install_prefix=str(prefix),receipt_path=str(receipt),receipt_sha256=sha(receipt),
                artifact_sha256=files,source_sha256=data['source_sha256'],repository_commit=data['repository_commit'])


def prepare(args, selected_install, footprint, scan_age):
    if not getattr(args,'with_mission_recovery',False):return None
    if not args.with_waypoints or args.variant not in ('official','segmented'):
        raise ValueError('Mission recovery requires native waypoints and a primary variant')
    if not getattr(args,'recovery_build_receipt',None):raise ValueError('--recovery-build-receipt required')
    result=artifacts(getattr(args,'recovery_install',None) or selected_install,args.recovery_build_receipt)
    result.update(schema='matrix.recovery.runtime.v1',run_id=uuid.uuid4().hex,
        footprint=footprint,padding=args.padding,scan_topic=args.scan_topic,odom_topic=args.odom_topic,
        scan_max_age=scan_age,bt_retries=1,health_stable_sec=1.,stop_timeout_sec=30.)
    # ROS topic tokens cannot start with a digit. Keep the original run identity
    # for parameters/services/evidence; only its topic token needs a safe prefix.
    result['raw_topic']='/matrix/run_'+result['run_id']+'/controller_cmd_vel'
    return result


def tree(recovery):
    attrs=' '.join(key+'='+quoteattr(str(value)) for key,value in dict(
        run_id=recovery['run_id'],footprint=json.dumps(recovery['footprint']),padding=recovery['padding'],
        odom_topic=recovery['odom_topic'],scan_topic=recovery['scan_topic'],scan_max_age=recovery['scan_max_age'],
        timeout=recovery['stop_timeout_sec'],health_stable=recovery['health_stable_sec']).items())
    execute=('<Sequence><StoppedReplanScope><Sequence><BeginStoppedReplanAttempt controller_error_code="{follow_path_error_code}" '
        'planner_error_code="{compute_path_error_code}" recovery_error_code="{recovery_error_code}"/>'
        '<WaitForStoppedAndHealthy '+attrs+' error_code_id="{recovery_error_code}"/>'
        '<ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased" use_start="false" error_code_id="{compute_path_error_code}"/>'
        '<ValidateStoppedPlanStart '+attrs+' path="{path}" error_code_id="{recovery_error_code}"/>'
        '</Sequence></StoppedReplanScope>'
        '<FollowPath path="{path}" controller_id="FollowPath" goal_checker_id="Checker" progress_checker_id="Progress" error_code_id="{follow_path_error_code}"/>'
        '</Sequence>')
    recover=('<Fallback><WouldAControllerRecoveryHelp error_code="{follow_path_error_code}"/>'
             '<WouldAPlannerRecoveryHelp error_code="{compute_path_error_code}"/>'
             '<WouldAStoppedPlanRecoveryHelp error_code="{recovery_error_code}"/></Fallback>')
    return ('<root BTCPP_format="4" main_tree_to_execute="MainTree"><BehaviorTree ID="MainTree">'
            '<RecoveryNode number_of_retries="'+str(recovery['bt_retries'])+'" name="MeasuredStoppedOriginalGoalReplan">'
            +execute+recover+'</RecoveryNode></BehaviorTree></root>\n')


def graph(node, recovery):
    def publishers(topic):
        return [dict(name=i.node_name,namespace=i.node_namespace,gid=list(i.endpoint_gid))
                for i in node.get_publishers_info_by_topic(topic)]
    output=publishers('/cmd_vel_nav');raw=publishers(recovery['raw_topic'])
    valid=(len(output)==1 and output[0]['name']=='matrix_zero_handoff' and output[0]['namespace']=='/'
           and len(raw)==1 and raw[0]['name']=='controller_server' and raw[0]['namespace']=='/')
    return dict(verified=valid,run_id=recovery['run_id'],raw_topic=recovery['raw_topic'],
                output_publishers=output,controller_publishers=raw)


def runtime_identity(manifest, parameters, identities, receipt, out, runtime):
    """Bind stable mission identity to typed readback and actually loaded bytes."""
    import xml.etree.ElementTree as ET
    import yaml
    def digest(value):
        return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode()).hexdigest()
    recovery=manifest['recovery']; commit=receipt['repository_commit']
    actual_errors=parameters.get('bt_navigator',{}).get('error_code_names',{})
    if actual_errors.get('type')!=9 or actual_errors.get('string_array_value')!=ERROR_CODE_NAMES:
        raise RuntimeError('Actual recovery error_code_names mismatch')
    origin=manifest.get('deployment_source_snapshot',{}).get('origin')
    source=origin or manifest['source_repository_observation']
    if (source.get('commit',source.get('head'))!=commit or source.get('status') or
            recovery['repository_commit']!=commit):
        raise RuntimeError('Mission runtime/source/build commit mismatch or dirty source')
    loaded={p:h for item in identities.values() for p,h in item['selected_libraries_sha256'].items()}
    required=['lib/libmatrix_recovery_nodes.so']+(['lib/libsegment_controller_core.so','lib/libsafe_lattice_planner.so']
        if manifest['variant']=='segmented' else ['lib/libofficial_lattice_rpp_adapter.so'])
    declarations={**receipt['artifact_sha256'],**recovery['artifact_sha256']}
    for rel in required:
        if loaded.get(str(runtime/rel))!=declarations.get(rel):
            raise RuntimeError('Mission required DSO not actually loaded: '+rel)
    exe=Path('/proc')/str(identities['matrix_zero_handoff']['pid'])/'exe'
    if sha(exe)!=declarations['lib/matrix_recovery_bt/zero_handoff_gate']:
        raise RuntimeError('Recovery gate executable mismatch')
    map_path=Path(parameters['map_server']['yaml_filename']['string_value'])
    if sha(map_path)!=manifest['map_yaml_sha256'] or sha(manifest['map_image'])!=manifest['map_image_sha256']:
        raise RuntimeError('Actual map source changed')
    map_doc=yaml.safe_load(map_path.read_text());map_doc['image']=manifest['map_image_sha256']
    map_hash=digest(map_doc)
    # Exclude only run identity and replace file locations with their verified content.
    config=copy.deepcopy(parameters)
    for key in ('run_id','raw_topic'):config['matrix_zero_handoff'].pop(key,None)
    config['map_server']['yaml_filename']['string_value']=map_hash
    for group in config.values():
        for key,value in group.items():
            if value.get('type')==4 and ('lattice_filepath' in key or key=='default_nav_to_pose_bt_xml'):
                path=Path(value['string_value'])
                if key=='default_nav_to_pose_bt_xml':
                    if path.resolve()!=(out/'navigate.xml').resolve():raise RuntimeError('Actual BT path mismatch')
                    tree=ET.fromstring(path.read_text())
                    for item in tree.iter():
                        if 'run_id' in item.attrib:item.attrib['run_id']='<run>'
                    value['string_value']=hashlib.sha256(ET.tostring(tree)).hexdigest()
                else:value['string_value']=sha(path)
    acceptance=dict(original_xy=.15,original_yaw=.08,stop_linear=.03,stop_angular=.05,
                    command_linear=.01,command_angular=.01,advancing_dwell_sec=.3)
    return dict(variant=manifest['variant'],source_sha=commit,map_contract_sha256=map_hash,
        footprint_sha256=digest(dict(footprint=manifest['footprint'],padding=manifest['padding'])),
        acceptance_policy_sha256=digest(acceptance),effective_config_sha256=digest(config),
        verified=True,runtime_parameters_sha256=sha(out/'runtime_parameters.json'),
        runtime_identities_sha256=sha(out/'runtime_identities.json'))

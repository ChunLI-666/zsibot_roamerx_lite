"""Actual Costmap2DROS loading regression; enable with CONTROLLER_FEEDBACK_ROOT."""
import hashlib
import os
from pathlib import Path
import resource
import subprocess
import pytest
import yaml
from run_cases import verify_footprint


def test_scoring_rejects_radius_fallback_even_with_matching_file_hash(tmp_path):
    source = tmp_path/'footprint.yaml'
    source.write_text(yaml.safe_dump(dict(footprint=[[-.3,-.15],[.3,-.15],[.3,.15],[-.3,.15]], footprint_padding=.01)))
    case = dict(footprint_file=str(source),footprint_sha256='same-hash')
    assert not verify_footprint(dict(source_sha256='same-hash',effective_polygon=[[.1,0]]*16),case)
    assert verify_footprint(dict(source_sha256='same-hash',effective_polygon=[[-.31,-.16],[.31,-.16],[.31,.16],[-.31,.16]]),case)


@pytest.mark.skipif('CONTROLLER_FEEDBACK_ROOT' not in os.environ, reason='requires built ROS harness')
def test_block_flow_and_actual_costmap_radius_fallback(tmp_path):
    root = Path(os.environ['CONTROLLER_FEEDBACK_ROOT']).resolve()
    original = yaml.safe_load((root/'zsl1_model/cases/warehouse_final_yaw_seed42.yaml').read_text())
    geometry = yaml.safe_load(Path(original['footprint_file']).read_text())
    records = []
    for label, flow, force_fallback in [('block',False,False),('flow',True,False),('fallback',False,True)]:
        source = tmp_path/(label+'_geometry.yaml')
        source.write_text(yaml.safe_dump(geometry,default_flow_style=flow))
        case = dict(original,footprint_file=str(source),footprint_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),steps=1)
        if force_fallback:
            case['test_only_footprint_parameter'] = yaml.safe_dump(geometry['footprint'],default_flow_style=False)
        path = tmp_path/(label+'.yaml'); path.write_text(yaml.safe_dump(case))
        output = tmp_path/(label+'.csv')
        def no_core():
            resource.setrlimit(resource.RLIMIT_CORE,(0,0))
        result = subprocess.run([str(root/'harness_build/controller_feedback'),str(path),str(root/'zsl1_model/cases/candidate.yaml'),str(output),'--ros-args','--log-level','warn'],
                                capture_output=True,text=True,timeout=30,preexec_fn=no_core,
                                env=dict(os.environ,ROS_DOMAIN_ID='182',ROS_LOCALHOST_ONLY='1',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST'))
        if force_fallback:
            assert result.returncode != 0
            assert 'Costmap footprint fallback' in result.stderr
            assert not output.exists()
        else:
            assert result.returncode == 1, result.stderr
            record = yaml.safe_load(Path(str(output)+'.footprint.yaml').read_text())
            assert len(record['effective_polygon']) == 4
            assert verify_footprint(record,case)
            records.append(record['effective_polygon'])
    assert records[0] == records[1]

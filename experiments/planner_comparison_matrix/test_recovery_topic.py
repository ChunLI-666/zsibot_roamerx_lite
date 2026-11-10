"""Deterministic UUID topic generation and stable effective-config identity."""
import json
import os
from pathlib import Path
import re
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock
import uuid
from rclpy.validate_full_topic_name import validate_full_topic_name
from rclpy.exceptions import InvalidTopicNameException

import recovery_navigation as recovery


def prepared(run_id,variant='segmented'):
    args=SimpleNamespace(with_mission_recovery=True,with_waypoints=True,variant=variant,
        recovery_build_receipt='test-receipt',recovery_install='test-install',padding=.15,
        scan_topic='/laser_scan',odom_topic='/odom/current_pose')
    with mock.patch.object(recovery,'artifacts',return_value={'repository_commit':'a'*40}), \
            mock.patch.object(recovery.uuid,'uuid4',return_value=uuid.UUID(hex=run_id)):
        return recovery.prepare(args,'test-install',[[-.6,-.5],[.6,-.5],[.6,.5],[-.6,.5]],.5)


class RecoveryTopicTests(unittest.TestCase):
    def test_every_hex_leading_digit_and_letter_has_ros_safe_token(self):
        for variant in ('segmented','official'):
            for first in '0123456789abcdef':
                value=first+'1'*31
                with self.subTest(variant=variant,first=first):
                    config=prepared(value,variant)
                    self.assertEqual(config['run_id'],value)
                    self.assertEqual(config['raw_topic'],'/matrix/run_'+value+'/controller_cmd_vel')
                    validate_full_topic_name(config['raw_topic'])
                    self.assertTrue(all(re.fullmatch('[A-Za-z_][A-Za-z0-9_]*',token)
                        for token in config['raw_topic'].split('/')[1:]))
                    self.assertIn('run_id="'+value+'"',recovery.tree(config))

    def test_a14_exact_uuid_is_retained_as_run_identity(self):
        value='9538272517ff455a9234193281fe775f';config=prepared(value)
        self.assertEqual(config['run_id'],value)
        self.assertEqual(config['raw_topic'],'/matrix/run_9538272517ff455a9234193281fe775f/controller_cmd_vel')
        with self.assertRaises(InvalidTopicNameException):
            validate_full_topic_name('/matrix/'+value+'/controller_cmd_vel')
        other=prepared('a538272517ff455a9234193281fe775f')
        self.assertNotEqual(config['raw_topic'],other['raw_topic'])
        self.assertNotEqual(config['run_id'],other['run_id'])

    def test_topic_prefix_and_run_uuid_do_not_change_effective_configuration(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);runtime=root/'runtime';runtime.mkdir()
            image=root/'map.pgm';image.write_bytes(b'P5\n1 1\n255\n\xff')
            map_path=root/'map.yaml';map_path.write_text('image: map.pgm\nresolution: 0.01\norigin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
            loaded={}
            for name in ('lib/libmatrix_recovery_nodes.so','lib/libsegment_controller_core.so','lib/libsafe_lattice_planner.so'):
                path=runtime/name;path.parent.mkdir(exist_ok=True);path.write_bytes(b'identity-fixture')
                loaded[str(path)]=recovery.sha(path)
            declared={str(Path(p).relative_to(runtime)):h for p,h in loaded.items()}
            declared['lib/matrix_recovery_bt/zero_handoff_gate']=recovery.sha('/proc/self/exe')
            identities=dict(bt_navigator={'selected_libraries_sha256':loaded},
                matrix_zero_handoff={'pid':os.getpid(),'selected_libraries_sha256':{}})
            (root/'runtime_identities.json').write_text(json.dumps(identities))
            receipt=dict(repository_commit='a'*40,artifact_sha256=declared)
            def identity(value,legacy=False,floor=.4):
                config=prepared(value)
                config['artifact_sha256']=declared
                if legacy:config['raw_topic']='/matrix/'+value+'/controller_cmd_vel'
                (root/'navigate.xml').write_text(recovery.tree(config))
                params=dict(map_server={'yaml_filename':dict(type=4,string_value=str(map_path))},
                    bt_navigator={'default_nav_to_pose_bt_xml':dict(type=4,string_value=str(root/'navigate.xml')),
                        'error_code_names':dict(type=9,string_array_value=list(recovery.ERROR_CODE_NAMES))},
                    matrix_zero_handoff={k:dict(type=4,string_value=config[k]) for k in ('run_id','raw_topic')},
                    controller_server={'FollowPath.rpp.desired_linear_vel':dict(type=3,double_value=floor)})
                (root/'runtime_parameters.json').write_text(json.dumps(params))
                manifest=dict(recovery=config,variant='segmented',source_repository_observation=dict(head='a'*40,status=''),
                    map_yaml_sha256=recovery.sha(map_path),map_image=str(image),map_image_sha256=recovery.sha(image),
                    footprint=config['footprint'],padding=config['padding'])
                return recovery.runtime_identity(manifest,params,identities,receipt,root,runtime)
            prior=identity('a'+'1'*31,legacy=True)
            for value in ('9538272517ff455a9234193281fe775f','a'+'1'*31):
                current=identity(value)
                for key in ('effective_config_sha256','map_contract_sha256','footprint_sha256','acceptance_policy_sha256','source_sha'):
                    self.assertEqual(current[key],prior[key],key)
            self.assertNotEqual(identity('a'+'1'*31,floor=.3)['effective_config_sha256'],prior['effective_config_sha256'])


if __name__=='__main__':unittest.main()

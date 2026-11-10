"""Real sealed A12 evidence through the normal offline CLI, never dispatch ROS."""
import contextlib
import io
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
import uuid
import yaml

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE/'goal_pilot_source'))
from mission_resume import main
from mission_state import MissionStore,Hold,digest

class SealedA12(unittest.TestCase):
    def test_real_prefix_closure_identity_and_negative_controls(self):
        workspace=Path(os.environ.get('MATRIX_EVIDENCE_ROOT',str(HERE.parents[4])))
        base=workspace/'analysis_outputs/planner_comparison_20261105'
        old=base/'native_A12_Source8_F011_lifecycle_HOLD_20261109_solmax_v1'
        source=old/'execution_A12_formal41_v1/result.remote.raw.json'
        if not source.exists():self.skipTest('Sealed workspace A12 evidence unavailable')
        route=old/'route.EXECUTABLE.json'
        closure=old/'execution_lifecycle_once/exact_closure.response.json'
        self.assertEqual(digest(source.read_bytes()),'ecba652955f034160e86b29008bb2f900f1064695edd5a48feb279a46ed0eb9d')
        self.assertEqual(digest(route.read_bytes()),'31b040497957718552500a1ff8c2c7278019687746eb1bd5cec1103fb1719446')
        closed=json.loads(closure.read_text());argv=next(r['target']['argv'] for r in closed['records'] if r['role']=='navigation')
        map_path=Path(argv[argv.index('--map')+1]);map_doc=yaml.safe_load(map_path.read_text())
        image=(map_path.parent/map_doc['image']).resolve();map_doc['image']=digest(image.read_bytes())
        identity=dict(variant='segmented',source_sha='dafbddf06066b8987bd3128a9d1479d800ac9d4f',
            map_contract_sha256=digest(map_doc),effective_config_sha256='a'*64,
            footprint_sha256=digest(dict(footprint=json.loads(argv[argv.index('--footprint')+1]),padding=float(argv[argv.index('--padding')+1]))),
            acceptance_policy_sha256=digest(dict(original_xy=.15,original_yaw=.08,stop_linear=.03,stop_angular=.05,
                command_linear=.01,command_angular=.01,advancing_dwell_sec=.3)))
        # effective_config is a test-only sentinel; this import has no ROS entry.
        # A subsequent real resume requires exact actual READY identity and fails it.
        for negative in (None,'other_uuid','other_run','other_map','unrelated_closure'):
            with self.subTest(negative=negative),tempfile.TemporaryDirectory() as directory:
                root=Path(directory);copy=json.loads(closure.read_text())
                if negative=='other_uuid':copy['parent_UUID']=str(uuid.uuid4())
                if negative=='other_run':copy['run_id']='other'
                if negative=='unrelated_closure':copy={'unrelated':True}
                (root/'closure.json').write_text(json.dumps(copy))
                manifest=dict(schema='matrix.mission.closed_attempt.v1',result_path=str(source),result_sha256=digest(source.read_bytes()),
                    historical_bt_retries=1,authorized_new_source_sha=identity['source_sha'],
                    closure=dict(path=str(root/'closure.json'),sha256=digest((root/'closure.json').read_bytes())),
                    original_map=dict(yaml_path=str(map_path),yaml_sha256=digest(map_path.read_bytes()),image_path=str(image),image_sha256=digest(image.read_bytes())),
                    **{k:identity[k] for k in ('map_contract_sha256','footprint_sha256','acceptance_policy_sha256')})
                manifest['evidence_files']=[manifest['closure']]
                if negative=='other_map':manifest['map_contract_sha256']='b'*64
                for name,value in [('manifest',manifest),('identity',identity),('policy',dict(mission_budget_sec=1800,max_parent_dispatches=3,max_attempts_per_index=6,bt_retries=1))]:
                    (root/(name+'.json')).write_text(json.dumps(value))
                args=['import-closed-attempt','--execute','--route',str(route),'--identity',str(root/'identity.json'),
                    '--policy',str(root/'policy.json'),'--closed-attempt-manifest',str(root/'manifest.json'),
                    '--store',str(root/'mission.db'),'--resource-lock',str(root/'lock'),'--output',str(root/'out'),'--request-id',str(uuid.uuid4())]
                if negative:
                    with self.assertRaises(Hold),contextlib.redirect_stdout(io.StringIO()):main(args)
                    continue
                with contextlib.redirect_stdout(io.StringIO()):self.assertEqual(main(args),0)
                state=MissionStore.inspect(root/'mission.db')
                self.assertEqual(state['progress']['accepted_prefix_count'],4)
                self.assertEqual(state['progress']['next_index'],4)
                self.assertEqual(state['current_target']['id'],'F015')
                self.assertEqual(state['budgets']['per_index_reserved'][:6],[2,2,2,2,2,0])
                self.assertAlmostEqual(state['budgets']['historical_debited_sec'],140.783957763)
                self.assertFalse(state['result_flags']['operational_complete'])
                self.assertTrue(all(r['legacy_stop_proof'] for r in state['progress']['accepted_receipts']))

if __name__=='__main__':unittest.main()

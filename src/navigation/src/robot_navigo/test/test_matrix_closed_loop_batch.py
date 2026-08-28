#!/usr/bin/env python3

import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import yaml


SCRIPT = Path(__file__).parents[1] / 'scripts' / 'matrix_closed_loop_batch.py'
SUITE = Path(__file__).parents[1] / 'regression' / 'matrix_closed_loop_suite.yaml'
SPEC = importlib.util.spec_from_file_location('matrix_closed_loop_batch', SCRIPT)
BATCH = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = BATCH
SPEC.loader.exec_module(BATCH)


class MatrixClosedLoopBatchTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.route = self.root / 'route.json'
        self.route.write_text(json.dumps({
            'schema': 'robot_navigo.map_coverage',
            'frame_id': 'map',
            'return_to_start': True,
            'metadata': {
                'planned_coverage_pct': 99.0,
                'map_fingerprint_sha256': 'a' * 64,
                'route_safety_check': 'all segments sampled inside the safe reachable domain',
            },
            'waypoints': [
                {'name': str(index), 'x': index, 'y': index % 2, 'yaw': 0.0}
                for index in range(8)
            ],
        }), encoding='utf-8')

    def tearDown(self):
        self.temporary.cleanup()

    def fake_command(self):
        code = (
            "from pathlib import Path; import json; "
            "p=Path(r'{case_output}'); p.mkdir(parents=True, exist_ok=True); "
            "(p/'result.json').write_text(json.dumps({{"
            "'pass': True, 'action': {{'status': 'SUCCEEDED'}}, "
            "'route': {{'waypoint_results': [{{'within_tolerance': True}}]}}, "
            "'acceptance': {{}}, 'relative_trajectory': {{'position_rmse_m': 0.1}}, "
            "'ground_truth_physical_motion': {{'path_length_m': 3.0}}, "
            "'topic_stats': {{}}}}))"
        )
        return [sys.executable, '-c', code]

    def manifest(self, modes=None, scenes=None):
        if modes is None:
            modes = [
                {'name': 'gt_baseline', 'command': self.fake_command()},
                {'name': 'lightning_formal', 'command': self.fake_command()},
            ]
        if scenes is None:
            scenes = [{
                'name': 'warehouse', 'scene_id': 1, 'available': True, 'enabled': True,
                'cases': [{
                    'name': 'coverage', 'coverage': 'full_map_loop', 'min_waypoints': 8,
                    'route': str(self.route), 'modes': modes,
                }],
            }]
        return {
            'schema': BATCH.SCHEMA,
            'schema_version': BATCH.SCHEMA_VERSION,
            'output_root': str(self.root / 'outputs'),
            'variables': {'workspace': str(self.root)},
            'scenes': scenes,
        }

    def test_manifest_requires_explicit_modes_and_full_coverage_route(self):
        invalid = self.manifest(modes=[{
            'name': 'gt_baseline', 'command': self.fake_command(),
        }])
        self.assertTrue(BATCH.validate_manifest(invalid) == [])
        invalid['scenes'][0]['cases'][0]['coverage'] = 'single_goal'
        self.assertTrue(BATCH.validate_manifest(invalid))
        invalid['scenes'][0]['cases'][0]['coverage'] = 'full_map_loop'
        self.route.write_text(json.dumps({
            'frame_id': 'map', 'return_to_start': False, 'waypoints': []
        }), encoding='utf-8')
        with self.assertRaises(BATCH.ManifestError):
            BATCH.collect_cases(invalid, self.root / 'bad', self.root, set(), set(), False, True, None)

    def test_cases_are_serial_and_modes_are_not_fallbacks(self):
        manifest = self.manifest()
        run_dir = self.root / 'run'
        rows = BATCH.collect_cases(
            manifest, run_dir, self.root, set(), set(), False, False, 10.0)
        self.assertEqual(['gt_baseline', 'lightning_formal'], [row['mode'] for row in rows])
        self.assertEqual(['PASS', 'PASS'], [row['status'] for row in rows])
        self.assertNotEqual(
            rows[0]['input_signature'], rows[1]['input_signature'],
            'GT and Lightning cases must have independent signatures')
        for row in rows:
            status = json.loads((run_dir / 'warehouse/coverage' / row['mode'] /
                                 'case_status.json').read_text())
            self.assertEqual('PASS', status['status'])
            self.assertEqual('UNAVAILABLE', status['contact_detection']['status'])

    def test_resume_reuses_matching_final_status(self):
        manifest = self.manifest(modes=[{
            'name': 'gt_baseline', 'command': self.fake_command(),
        }])
        run_dir = self.root / 'resume'
        first = BATCH.collect_cases(manifest, run_dir, self.root, set(), set(), False, False, 10)
        result = run_dir / 'warehouse/coverage/gt_baseline/result.json'
        result.unlink()
        second = BATCH.collect_cases(manifest, run_dir, self.root, set(), set(), True, False, 10)
        self.assertEqual('PASS', first[0]['status'])
        self.assertTrue(second[0]['resumed'])
        self.assertFalse(result.exists(), 'resume must not rerun or recreate runner artifacts')

    def test_unavailable_scene_and_html_are_explicit(self):
        scene = {
            'name': 'house', 'scene_id': 6, 'available': False, 'enabled': False,
            'reason': 'map missing <contact>',
        }
        manifest = self.manifest(scenes=[scene])
        rows = BATCH.collect_cases(manifest, self.root / 'unavailable', self.root,
                                   set(), set(), False, False, None)
        self.assertEqual('UNAVAILABLE', rows[0]['status'])
        summary = BATCH.aggregate(rows)
        report = BATCH.html_report(summary)
        self.assertIn('UNAVAILABLE', report)
        self.assertIn('&lt;contact&gt;', report)
        self.assertIn('UNAVAILABLE', report)

    def test_repository_suite_has_two_explicit_modes_and_route(self):
        manifest = BATCH.load_manifest(SUITE)
        warehouse = next(scene for scene in manifest['scenes'] if scene['name'] == 'warehouse')
        case = warehouse['cases'][0]
        self.assertEqual('central_smoke', case['coverage'])
        self.assertEqual(
            {'gt_baseline', 'lightning_formal'},
            {mode['name'] for mode in case['modes']})
        self.assertEqual(8, case['min_waypoints'])
        per_goal_timeout = manifest['variables']['per_goal_timeout_sec']
        self.assertGreaterEqual(per_goal_timeout, 180)
        for mode in case['modes']:
            timeout_index = mode['command'].index('--timeout')
            self.assertEqual(
                '{per_goal_timeout_sec}', mode['command'][timeout_index + 1],
                'batch timeout and per-goal NavigateToPose timeout must be explicit')

    def test_timeout_writes_final_status_and_stops_owned_group(self):
        slow = {
            'name': 'gt_baseline',
            'command': [sys.executable, '-c', 'import time; time.sleep(5)'],
        }
        manifest = self.manifest(modes=[slow])
        rows = BATCH.collect_cases(
            manifest, self.root / 'timeout', self.root, set(), set(), False, False, 0.05)
        self.assertEqual('TIMEOUT', rows[0]['status'])
        status = json.loads((self.root / 'timeout/warehouse/coverage/gt_baseline/'
                             'case_status.json').read_text())
        self.assertEqual('TIMEOUT', status['status'])

    def test_orchestrator_uses_flock_and_process_groups_without_global_kill(self):
        source = SCRIPT.read_text(encoding='utf-8')
        self.assertIn('fcntl.flock', source)
        self.assertIn('start_new_session=True', source)
        self.assertIn('os.killpg', source)
        self.assertNotIn('pkill', source)

    def test_package_root_supports_source_and_install_layouts(self):
        source_script = self.root / 'source/robot_navigo/scripts/tool.py'
        source_script.parent.mkdir(parents=True)
        (source_script.parent.parent / 'routes').mkdir(parents=True)
        (source_script.parent.parent / 'regression').mkdir()
        source_script.touch()
        self.assertEqual(source_script.parent.parent, BATCH.package_root_for_script(source_script))

        installed = self.root / 'install/robot_navigo/lib/robot_navigo/tool.py'
        installed.parent.mkdir(parents=True)
        installed.touch()
        share = self.root / 'install/robot_navigo/share/robot_navigo'
        (share / 'routes').mkdir(parents=True)
        (share / 'regression').mkdir()
        self.assertEqual(share, BATCH.package_root_for_script(installed))

    def test_legacy_waypoint_results_count_succeeded_actions(self):
        metrics = BATCH.mode_result_metrics({
            'route': {'waypoint_results': [
                {'action': {'status': 'SUCCEEDED'}},
                {'action': {'status': 'TIMEOUT'}},
            ]},
        })
        self.assertEqual(metrics['waypoints_completed'], 1)
        self.assertEqual(metrics['waypoints_total'], 2)


if __name__ == '__main__':
    unittest.main()

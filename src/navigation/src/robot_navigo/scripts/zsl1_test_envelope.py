#!/usr/bin/env python3
"""Geometry unit checks and official-model regressions (no ROS or network)."""
import copy
import json
import math
from pathlib import Path
import struct
import tempfile
import unittest

import numpy as np
import yaml
from zsl1_merge_params import merge_experiment
from zsl1_envelope import Model, checked_costmap_padding, generate, hull, import_home, outside_count, rotation, stl_vertices, transform


class GeometryTest(unittest.TestCase):
    def test_axis_angle_and_urdf_transform_order(self):
        np.testing.assert_allclose(rotation([0, 0, 1], math.pi / 2) @ [1, 0, 0], [0, 1, 0], atol=1e-12)
        matrix = transform([1, 2, 3], [math.pi / 2, 0, math.pi / 2])
        np.testing.assert_allclose(matrix @ [0, 1, 0, 1], [1, 2, 4, 1], atol=1e-12)
        with self.assertRaises(ValueError): rotation([0, 0, 0], 1)

    def test_binary_ascii_and_nonfinite_mesh(self):
        with tempfile.TemporaryDirectory() as directory:
            binary, ascii_path = Path(directory) / 'binary.stl', Path(directory) / 'ascii.stl'
            binary.write_bytes(bytes(80) + struct.pack('<I12fH', 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0))
            ascii_path.write_text('solid s\n facet normal 0 0 1\n outer loop\n vertex 0 0 0\n vertex 1 0 0\n vertex 0 1 0\n endloop\n endfacet\nendsolid s\n')
            np.testing.assert_equal(stl_vertices(binary), stl_vertices(ascii_path))
            ascii_path.write_text(ascii_path.read_text().replace('vertex 0 0 0', 'vertex nan 0 0'))
            with self.assertRaises(ValueError): stl_vertices(ascii_path)

    def test_convex_containment_detects_an_outside_vertex(self):
        points = np.array([[-2, -1, 0], [1, -1, 0], [1, 2, 0], [-2, 2, 0], [0, 0, 0]])
        polygon = hull(points)
        self.assertEqual(outside_count(points, polygon)[0], 0)
        self.assertEqual(outside_count(np.array([[2, 0, 0]]), polygon)[0], 1)
        self.assertGreater(np.sum(polygon[:, 0] * np.roll(polygon[:, 1], -1) - polygon[:, 1] * np.roll(polygon[:, 0], -1)), 0)

    def test_costmap_sign_padding_rejects_a_convex_input_that_becomes_concave(self):
        polygon = np.array([[-.831969313, .665288295], [-.538715582, -.895957398], [.752968462, -.882863930],
            [.574196615, -.521261114], [-.099321267, .592648541], [-.216761999, .780548704]])
        self.assertEqual(outside_count(np.column_stack([polygon, np.zeros(len(polygon))]), hull(polygon))[0], 0)
        with self.assertRaisesRegex(ValueError, 'nonconvex'):
            checked_costmap_padding(polygon, .1)
        rectangle = np.array([[-.36, -.19], [.28, -.19], [.28, .19], [-.36, .19]])
        padded = checked_costmap_padding(rectangle, .01)
        np.testing.assert_allclose(padded, [[-.37, -.2], [.29, -.2], [.29, .2], [-.37, .2]])


class OfficialModelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        relative = Path('src/zsibot/zsibot_model/zsl-1/urdf/ZSL-1.urdf')
        cls.workspace = next((p for p in Path(__file__).resolve().parents if (p / relative).exists()), None)
        if cls.workspace is None:
            raise unittest.SkipTest('Official ZSL-1 model checkout is required for integration tests')
        cls.model = Model(cls.workspace / relative)
        cls.params = Path(__file__).resolve().parent.parent / 'params'
        cls.posture = cls.params / 'zsl1_model_postures.yaml'
        cls.official = cls.params / 'zsl1_official_source_manifest.json'
        cls.matrix = cls.workspace / 'src/zsibot/matrix/src/robot_mujoco/jszr_robots/xgb/xgb.xml'
        cls.doc = yaml.safe_load(cls.posture.read_text())
        cls.joints = cls.doc['samples'][0]['joints']

    def test_source_home_and_all_17_meshes_correspond(self):
        imported = import_home(self.model, self.matrix)
        self.assertEqual(len(self.model.meshes), 17)
        self.assertEqual(imported['samples'][0]['joints'], self.joints)
        self.assertEqual(len(imported['source']['matching_assets']), 17)

    def test_foot_forward_kinematics_against_independent_planar_formula(self):
        poses = self.model.poses(self.joints)
        for leg, x, y in [('FL', .17449, .159412), ('FR', .17449, -.159399),
                          ('RL', -.17449, .159412), ('RR', -.17449, -.1594)]:
            expected = [x - .2 * math.sin(.9) - .21366 * math.sin(-.9), y,
                        -.2 * math.cos(.9) - .21366 * math.cos(-.9)]
            np.testing.assert_allclose(poses[leg + '_FOOT_LINK'][:3, 3], expected, atol=1e-9)

    def test_missing_nonfinite_and_zero_knee_are_rejected(self):
        joints = dict(self.joints)
        del joints['FL_KNEE_JOINT']
        with self.assertRaises(ValueError): self.model.poses(joints)
        for value in (0, float('nan'), -4):
            joints = dict(self.joints, FL_KNEE_JOINT=value)
            with self.assertRaises(ValueError): self.model.poses(joints)

    def test_full_vertex_coverage_and_asymmetric_origin_are_preserved(self):
        result, overlay = generate(self.model, self.posture, self.official)
        self.assertEqual(result['metadata']['coverage']['outside_vertices'], 0)
        self.assertEqual(len(result['footprint']), 4)
        self.assertLess(result['bounds']['minimum_xy'][0], -.35)
        self.assertLess(result['bounds']['maximum_xy'][0], .30)
        self.assertGreater(result['bounds']['width_m'], .37)
        self.assertFalse(result['hardware_deployment_ready'])
        self.assertFalse(result['motion_envelope_verified'])
        for name in ('global_costmap', 'local_costmap'):
            params = overlay[name][name]['ros__parameters']
            self.assertEqual(json.loads(params['footprint']), result['footprint'])
            self.assertEqual(params['footprint_padding'], result['footprint_padding'])
            self.assertGreater(params['inflation_layer']['inflation_radius'], result['bounds']['padded_circumscribed_radius_m'])

    def test_untrusted_source_and_unimplemented_payload_cannot_pass(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            bad = json.loads(self.official.read_text())
            bad['files'][0]['remote_sha256'] = '0' * 64
            (path / 'official.json').write_text(json.dumps(bad))
            with self.assertRaises(ValueError): generate(self.model, self.posture, path / 'official.json')
            doc = copy.deepcopy(self.doc)
            doc['payloads'] = [{'name': 'unmodelled_camera'}]
            (path / 'postures.yaml').write_text(yaml.safe_dump(doc))
            with self.assertRaises(ValueError): generate(self.model, path / 'postures.yaml', self.official)

    def test_complete_experiment_merge_preserves_plugins_and_rejects_split_geometry(self):
        read = lambda name: yaml.safe_load((self.params / name).read_text())
        args = [read(name) for name in ('navigo_params.yaml', 'forward_alignment_experiment.yaml',
                'zsl1_model_footprint_overlay.yaml', 'zsl1_model_envelope.yaml')]
        result = merge_experiment(*args)
        self.assertEqual(result['local_costmap']['local_costmap']['ros__parameters']['plugins'],
            args[0]['local_costmap']['local_costmap']['ros__parameters']['plugins'])
        args[2]['local_costmap']['local_costmap']['ros__parameters']['footprint_padding'] = .02
        with self.assertRaises(ValueError): merge_experiment(*args)

    def test_matrix_axis_mismatch_is_detected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'different.xml'
            path.write_text(self.matrix.read_text().replace('axis="1 0 0"', 'axis="-1 0 0"', 1))
            with self.assertRaisesRegex(ValueError, 'kinematics differ'):
                import_home(self.model, path)

    def test_synthetic_postures_are_covered_but_not_added_to_the_declared_envelope(self):
        # These seeded random postures validate FK/projection; they are NOT gait
        # evidence and are never included in the published home-model footprint.
        rng = np.random.default_rng(20260910)
        for _ in range(3):
            joints = {name: float(rng.uniform(*joint['limit'])) for name, joint in self.model.joints.items() if joint['kind'] == 'revolute'}
            clouds = self.model.sample(joints, transform([.1, -.2, 0], [.1, -.1, .3]))
            points = np.concatenate(list(clouds.values()))
            self.assertEqual(outside_count(points, hull(points))[0], 0)
        self.assertEqual(len(self.doc['samples']), 1)


if __name__ == '__main__':
    unittest.main()

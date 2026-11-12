#!/usr/bin/env python3
"""Independent schema, grid, connectivity and swept-sample contracts."""

import hashlib
import importlib.util
import json
import math
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
ARTIFACT = ROOT / "params" / "stop_turn_1cm_h16.json"


def angular_delta(a, b):
    return math.atan2(math.sin(b - a), math.cos(b - a))


class PrimitiveContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.data = json.loads(ARTIFACT.read_text())
        cls.meta = cls.data["lattice_metadata"]
        cls.angles = cls.meta["heading_angles"]
        cls.motions = cls.data["primitives"]

    def test_schema_and_counts(self):
        self.assertEqual(set(self.data), {"version", "date_generated", "lattice_metadata", "primitives"})
        self.assertEqual(set(self.meta), {"motion_model", "turning_radius", "grid_resolution",
                         "stopping_threshold", "num_of_headings", "heading_angles", "number_of_trajectories"})
        self.assertEqual(self.meta["motion_model"], "diff")
        self.assertEqual(self.meta["grid_resolution"], 0.01)
        self.assertGreater(self.meta["turning_radius"], 0)
        self.assertEqual(self.meta["num_of_headings"], 16)
        self.assertEqual(self.meta["number_of_trajectories"], 48)
        self.assertEqual(len(self.motions), 48)
        fields = {"trajectory_id", "start_angle_index", "end_angle_index", "left_turn",
                  "trajectory_radius", "trajectory_length", "arc_length", "straight_length", "poses"}
        for i, motion in enumerate(self.motions):
            self.assertEqual(set(motion), fields)
            self.assertEqual(motion["trajectory_id"], i)
            self.assertEqual(motion["start_angle_index"], i // 3)
            self.assertIsInstance(motion["left_turn"], bool)
            for pose in motion["poses"]:
                self.assertEqual(len(pose), 3)
                self.assertTrue(all(math.isfinite(x) for x in pose))
                self.assertGreaterEqual(pose[2], 0)
                self.assertLess(pose[2], math.tau)

    def test_nonuniform_integer_grid_headings_and_endpoints(self):
        self.assertEqual(len(self.angles), 16)
        self.assertEqual(self.angles, sorted(self.angles))
        gaps = [angular_delta(self.angles[i], self.angles[(i + 1) % 16]) for i in range(16)]
        self.assertTrue(all(g > 0 for g in gaps))
        self.assertGreater(max(gaps) - min(gaps), 0.1)
        for i in range(16):
            motion = self.motions[3 * i]
            x, y, yaw = motion["poses"][-1]
            self.assertAlmostEqual(x / 0.01, round(x / 0.01), places=12)
            self.assertAlmostEqual(y / 0.01, round(y / 0.01), places=12)
            self.assertAlmostEqual(angular_delta(math.atan2(y, x), yaw), 0, places=12)
            self.assertGreater(x * math.cos(yaw) + y * math.sin(yaw), 0)
            self.assertEqual(motion["end_angle_index"], i)
            self.assertAlmostEqual(motion["trajectory_length"], math.hypot(x, y), places=12)
            self.assertEqual(motion["trajectory_length"], motion["straight_length"])
            self.assertEqual(motion["arc_length"], 0)
            self.assertTrue(all(p[2] == self.angles[i] for p in motion["poses"]))

    def test_rotation_direction_wrap_and_dense_samples(self):
        padded_corner_radius = math.hypot(0.67 / 2, 0.37 / 2)
        for i in range(16):
            for slot, sign in ((1, 1), (2, -1)):
                motion = self.motions[3 * i + slot]
                end = (i + sign) % 16
                self.assertEqual(motion["end_angle_index"], end)
                self.assertEqual(motion["left_turn"], sign > 0)
                self.assertEqual(motion["poses"][0], [0, 0, self.angles[i]])
                self.assertEqual(motion["poses"][-1], [0, 0, self.angles[end]])
                for field in ("trajectory_radius", "trajectory_length", "arc_length", "straight_length"):
                    self.assertEqual(motion[field], 0)
                deltas = []
                for a, b in zip(motion["poses"], motion["poses"][1:]):
                    self.assertEqual(a[:2], [0, 0])
                    self.assertEqual(b[:2], [0, 0])
                    delta = angular_delta(a[2], b[2])
                    self.assertGreater(sign * delta, 0)
                    self.assertLessEqual(abs(delta), 0.0025 + 1e-12)
                    self.assertLess(padded_corner_radius * abs(delta), 0.00125)
                    deltas.append(delta)
                self.assertAlmostEqual(sum(deltas), angular_delta(self.angles[i], self.angles[end]), places=12)

    def test_translation_samples_are_forward_dense_and_collinear(self):
        for motion in self.motions[::3]:
            yaw = motion["poses"][0][2]
            for a, b in zip(motion["poses"], motion["poses"][1:]):
                dx, dy = b[0] - a[0], b[1] - a[1]
                self.assertGreater(dx * math.cos(yaw) + dy * math.sin(yaw), 0)
                self.assertAlmostEqual(dx * math.sin(yaw) - dy * math.cos(yaw), 0, places=12)
                self.assertLessEqual(math.hypot(dx, dy), 0.001 + 1e-12)

    def test_headings_and_grid_are_connected_without_reverse(self):
        # Adjacent rotations reach every heading; four unit cardinal translations
        # at those reachable headings generate all integer XY lattice cells.
        visited = {0}
        pending = [0]
        while pending:
            i = pending.pop()
            for motion in self.motions[3 * i + 1:3 * i + 3]:
                end = motion["end_angle_index"]
                if end not in visited:
                    visited.add(end)
                    pending.append(end)
        self.assertEqual(visited, set(range(16)))
        endpoints = {(round(m["poses"][-1][0] / 0.01), round(m["poses"][-1][1] / 0.01))
                     for m in self.motions[::3]}
        self.assertTrue({(1, 0), (-1, 0), (0, 1), (0, -1)} <= endpoints)

    def test_generated_bytes_and_provenance_are_reproducible(self):
        path = ROOT / "tools" / "generate_stop_turn_primitives.py"
        spec = importlib.util.spec_from_file_location("stop_turn_generator", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.assertEqual(module.encoded(module.generate()), ARTIFACT.read_bytes())
        provenance = json.loads(ARTIFACT.with_suffix(".provenance.json").read_text())
        self.assertEqual(provenance["sha256"], hashlib.sha256(ARTIFACT.read_bytes()).hexdigest())
        self.assertEqual(provenance["generator_sha256"], hashlib.sha256(path.read_bytes()).hexdigest())
        self.assertFalse(provenance["reverse_or_lateral_displacement"])


if __name__ == "__main__":
    unittest.main()

#!/usr/bin/env python3

import importlib.util
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "scripts" / "matrix_ground_truth_baseline.py"
SPEC = importlib.util.spec_from_file_location("matrix_ground_truth_baseline", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class MatrixGroundTruthBaselineTest(unittest.TestCase):
    def test_load_and_compose_fixed_map_alignment(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "alignment.json"
            path.write_text(json.dumps({
                "map_T_ground_truth": {"x": 1.0, "y": 2.0, "yaw": math.pi / 2.0},
            }), encoding="utf-8")
            transform = MODULE.load_map_to_ground_truth(path)
            x, y, yaw = MODULE.compose_planar(transform, (2.0, 0.0, 0.25))
            self.assertAlmostEqual(x, 1.0)
            self.assertAlmostEqual(y, 4.0)
            self.assertAlmostEqual(yaw, math.pi / 2.0 + 0.25)

    def test_missing_alignment_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "alignment.json"
            path.write_text("{}", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "map_T_ground_truth"):
                MODULE.load_map_to_ground_truth(path)

    def test_publish_rate_gate_handles_first_reset_and_period(self):
        self.assertTrue(MODULE.publish_due(None, 100, 20))
        self.assertFalse(MODULE.publish_due(100, 119, 20))
        self.assertTrue(MODULE.publish_due(100, 120, 20))
        self.assertTrue(MODULE.publish_due(100, 90, 20))


if __name__ == "__main__":
    unittest.main()

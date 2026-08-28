#!/usr/bin/env python3

import importlib.util
import sys
import unittest
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "scripts" / "build_matrix_reference_map.py"
SPEC = importlib.util.spec_from_file_location("build_matrix_reference_map", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class MatrixReferenceMapTest(unittest.TestCase):
    def test_unknown_pixel_stays_between_ros_trinary_thresholds(self):
        occupancy = 1.0 - MODULE.UNKNOWN_PIXEL / 255.0
        self.assertGreater(occupancy, 0.25)
        self.assertLess(occupancy, 0.65)


if __name__ == "__main__":
    unittest.main()

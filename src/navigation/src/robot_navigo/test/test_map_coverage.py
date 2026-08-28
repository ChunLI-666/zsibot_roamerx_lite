#!/usr/bin/env python3

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
from PIL import Image


SCRIPT = Path(__file__).parents[1] / "scripts" / "map_coverage.py"
SPEC = importlib.util.spec_from_file_location("map_coverage", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class MapCoverageTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        # Cartesian map: free room, an occupied vertical wall with a doorway,
        # and an unknown strip that must never count as valid coverage.
        cells = np.full((20, 30), 254, dtype=np.uint8)
        cells[5:15, 14] = 0
        cells[9:11, 14] = 254
        cells[0:3, :] = 128
        self.image = self.root / "map.pgm"
        Image.fromarray(np.flipud(cells), mode="L").save(self.image)
        self.yaml = self.root / "map.yaml"
        self.yaml.write_text(
            "image: map.pgm\nmode: trinary\nresolution: 0.1\n"
            "origin: [0.0, 0.0, 0.0]\nnegate: 0\n"
            "occupied_thresh: 0.65\nfree_thresh: 0.25\n",
            encoding="utf-8",
        )

    def tearDown(self):
        self.tmp.cleanup()

    def test_unknown_is_not_free_and_start_domain_is_connected(self):
        grid = MODULE.load_grid_map(self.yaml)
        self.assertTrue(grid.unknown[1, 1])
        domain, meta = MODULE.safe_reachable_domain(grid, (0.5, 0.8), 0.0)
        self.assertGreater(meta["reachable_free_cells"], 0)
        self.assertFalse(domain[1, 1])

    def test_route_segments_are_free_and_stay_in_start_component(self):
        grid = MODULE.load_grid_map(self.yaml)
        waypoints, meta, domain = MODULE.generate_route(
            grid, (0.8, 0.8), 0.0, 0.5, 0.4, 0.3
        )
        self.assertGreater(len(waypoints), 2)
        points = [(item["x"], item["y"]) for item in waypoints]
        self.assertEqual(points[0], (0.8, 0.8))
        self.assertTrue(all(MODULE.segment_is_free(grid, domain, a, b) for a, b in zip(points, points[1:])))
        self.assertGreater(meta["planned_coverage_cells"], 0)

    def test_safety_erosion_rejects_start_too_close_to_wall(self):
        grid = MODULE.load_grid_map(self.yaml)
        with self.assertRaisesRegex(ValueError, "unsafe_after_erosion"):
            MODULE.safe_reachable_domain(grid, (1.3, 1.0), 0.2)

    def test_evaluation_excludes_unknown_and_reports_uncovered_regions(self):
        grid = MODULE.load_grid_map(self.yaml)
        trajectory = [(0.8, 0.8), (0.8, 1.8), (2.5, 1.8)]
        metadata, domain, covered, uncovered = MODULE.evaluate_trajectory(
            grid, trajectory, (0.8, 0.8), 0.0, 0.0
        )
        self.assertEqual(metadata["unknown"], 0)
        self.assertEqual(metadata["valid_cells"], int(np.count_nonzero(domain)))
        self.assertEqual(metadata["covered_cells"], int(np.count_nonzero(covered)))
        self.assertEqual(metadata["uncovered_cells"], int(np.count_nonzero(uncovered)))
        self.assertTrue(metadata["uncovered_regions"])
        self.assertLess(metadata["coverage_pct"], 100.0)

        unknown_metadata, _, _, _ = MODULE.evaluate_trajectory(
            grid, [(0.8, 0.8), (0.8, 0.1)], (0.8, 0.8), 0.0, 0.0
        )
        self.assertGreater(unknown_metadata["unknown"], 0)

    def test_csv_evaluation_and_cli_outputs(self):
        csv_path = self.root / "odom.csv"
        csv_path.write_text("stamp,x,y\n0,0.8,0.8\n1,1.8,0.8\n", encoding="utf-8")
        output = self.root / "coverage.json"
        preview = self.root / "coverage.png"
        code = MODULE.main([
            "evaluate", "--map-yaml", str(self.yaml), "--trajectory-csv", str(csv_path),
            "--start-x", "0.8", "--start-y", "0.8", "--output", str(output),
            "--preview", str(preview), "--coverage-radius", "0.2",
        ])
        self.assertEqual(code, 0)
        self.assertTrue(preview.is_file())
        payload = json.loads(output.read_text(encoding="utf-8"))
        self.assertEqual(payload["schema"], MODULE.SCHEMA)
        self.assertEqual(payload["metadata"]["unknown_counts_as"], "not valid and never covered")

        route_output = self.root / "route.json"
        route_preview = self.root / "route.png"
        code = MODULE.main([
            "generate", "--map-yaml", str(self.yaml), "--start-x", "0.8", "--start-y", "0.8",
            "--output", str(route_output), "--preview", str(route_preview),
        ])
        self.assertEqual(code, 0)
        route_payload = json.loads(route_output.read_text(encoding="utf-8"))
        self.assertNotIn("handler", route_payload["parameters"])
        self.assertTrue(route_payload["return_to_start"])
        self.assertEqual(route_payload["frame_id"], "map")


if __name__ == "__main__":
    unittest.main()

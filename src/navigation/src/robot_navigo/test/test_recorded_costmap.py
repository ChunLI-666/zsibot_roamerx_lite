import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

SPEC = importlib.util.spec_from_file_location('reconstruct_recorded_costmap',
    Path(__file__).resolve().parents[1] / 'scripts/reconstruct_recorded_costmap.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class ReconstructionTest(unittest.TestCase):
    def test_every_cost_is_conservatively_reconstructed_without_new_lethal(self):
        raw = np.arange(256)
        occupancy = np.array([0] + [1 + (97 * (c - 1)) // 251 for c in range(1, 253)] +
                             [99, 100, -1])
        recovered = MODULE.occupancy_to_cost(occupancy)
        self.assertTrue(np.all(recovered >= raw))
        self.assertTrue(np.all(recovered[1:253] < 253))
        np.testing.assert_array_equal(recovered[[0, 253, 254, 255]], [0, 253, 254, 255])

    def test_invalid_occupancy_rejected(self):
        for values in ([-2], [101], [float('nan')], [0.5]):
            with self.assertRaises(ValueError):
                MODULE.occupancy_to_cost(values)

    def grid(self):
        state = MODULE.RecordedGrid()
        state.full(dict(width=4, height=3, resolution=.05, origin_x=-1.,
                        origin_y=-2., frame='odom'), [0] * 12, 100, 110)
        return state

    def test_patch_orientation_and_unknown_preserved(self):
        state = self.grid()
        self.assertTrue(state.update('odom', 1, 1, 2, 1, [100, -1], 120, 130))
        np.testing.assert_array_equal(state.cells, [[0, 0, 0, 0], [0, 100, -1, 0], [0, 0, 0, 0]])
        with tempfile.TemporaryDirectory() as tmp:
            result = state.write(Path(tmp), 'sample', 140, '/bag')
            raw = np.fromfile(result['map']['data'], dtype=np.uint8).reshape(3, 4)
            np.testing.assert_array_equal(raw[1], [0, 254, 255, 0])
            self.assertEqual(result['last_receipt_age_sec'], 1e-8)
            json.loads((Path(tmp) / 'sample.json').read_text())

    def test_invalid_patch_does_not_mutate_state(self):
        state = self.grid()
        before = state.cells.copy()
        for patch in [('map', 0, 0, 1, 1, [100], 120, 130),
                      ('odom', 0, 0, 1, 1, [100], 90, 130),
                      ('odom', 3, 0, 2, 1, [100, 100], 120, 130)]:
            self.assertFalse(state.update(*patch))
            np.testing.assert_array_equal(state.cells, before)
        self.assertEqual(state.last_time_ns, 110)

    def test_full_grid_replaces_rolling_origin_and_update_history(self):
        state = self.grid()
        state.update('odom', 0, 0, 1, 1, [100], 120, 130)
        meta = dict(state.meta, origin_x=5.)
        state.full(meta, [0] * 12, 140, 150)
        self.assertEqual(state.meta['origin_x'], 5.)
        self.assertEqual(state.applied_updates, 0)
        self.assertFalse(np.any(state.cells))

    def test_zero_stamped_publisher_updates_use_receipt_order(self):
        state = self.grid()
        self.assertTrue(state.update('odom', 0, 0, 1, 1, [100], 0, 130))
        self.assertEqual(state.unstamped_updates, 1)
        self.assertEqual(state.last_time_ns, 130)
        self.assertEqual(state.cells[0, 0], 100)
        self.assertFalse(state.update('odom', 0, 0, 1, 1, [0], 90, 140))

    def test_snapshot_rejections_are_immutable(self):
        state = self.grid()
        with tempfile.TemporaryDirectory() as tmp:
            result = state.write(Path(tmp), 'sample', 115, '/bag')
            state.update('wrong_frame', 0, 0, 1, 1, [100], 120, 130)
            self.assertEqual(result['rejected_updates'], [])


if __name__ == '__main__':
    unittest.main()

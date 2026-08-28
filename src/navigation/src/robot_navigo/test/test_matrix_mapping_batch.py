#!/usr/bin/env python3
import fcntl
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest


SCRIPT = Path(__file__).parents[1] / "scripts" / "matrix_mapping_batch.py"
SPEC = importlib.util.spec_from_file_location("matrix_mapping_batch", SCRIPT)
BATCH = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = BATCH
SPEC.loader.exec_module(BATCH)


class MatrixMappingBatchTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.eval_bag = self.root / "later_localization_bag"
        self.eval_bag.mkdir()
        self.mapping_bag = self.root / "mapping_bag"
        self.coverage = self.root / "coverage.json"
        self.map_dir = self.root / "map"

    def tearDown(self):
        self.tmp.cleanup()

    def manifest(self, available=True):
        collect = [sys.executable, "-c", (
            "from pathlib import Path; import json; "
            f"Path(r'{self.mapping_bag}').mkdir(parents=True, exist_ok=True); "
            f"Path(r'{self.coverage}').write_text(json.dumps({{"
            f"'schema':'robot_navigo.map_coverage','metadata':{{'coverage_pct':100,"
            f"'uncovered_cells':0,'map_fingerprint_sha256':'{'a' * 64}',"
            f"'trajectory_source':r'{self.mapping_bag}'}}}}))"
        )]
        offline = [
            sys.executable, "-c",
            f"from pathlib import Path; assert Path(__import__('sys').argv[1]).exists(); "
            f"Path(r'{self.map_dir}').mkdir(parents=True, exist_ok=True)",
            "{mapping_bag}",
        ]
        return {"schema": BATCH.SCHEMA, "schema_version": 1, "variables": {}, "scenes": [{
            "name": "warehouse", "scene_id": 1, "available": available, "enabled": available,
            "reason": "fixture unavailable" if not available else None, "ros_domain_id": 43,
            "mapping_bag": str(self.mapping_bag), "localization_bag": str(self.eval_bag),
            "mapping_dataset_id": "warehouse_mapping_run_001",
            "localization_dataset_id": "warehouse_eval_run_002",
            "coverage_metadata": str(self.coverage),
            "collect_mapping_bag": {"command": collect, "required_outputs": [str(self.mapping_bag)]},
            "offline_lightning_mapping": {"command": offline, "required_evidence": [str(self.mapping_bag)], "required_outputs": [str(self.map_dir)]},
        }]}

    def execute_scene(self, manifest=None, **kwargs):
        return BATCH.run_scene((manifest or self.manifest())["scenes"][0], self.root / "run", {}, **kwargs)

    def test_manifest_validation_and_gt_boundary(self):
        self.assertEqual([], BATCH.validate_manifest(self.manifest()))
        bad = self.manifest()
        bad["scenes"][0]["offline_lightning_mapping"]["command"].append("--gt-topic")
        self.assertTrue(any("must not use GT" in x for x in BATCH.validate_manifest(bad)))
        bad = self.manifest()
        bad["scenes"][0]["collect_mapping_bag"]["command"].append("pkill matrix")
        self.assertTrue(any("pkill" in x for x in BATCH.validate_manifest(bad)))

        bad = self.manifest()
        bad["scenes"][0]["name"] = []
        bad["scenes"][0]["scene_id"] = []
        bad["scenes"][0]["ros_domain_id"] = []
        errors = BATCH.validate_manifest(bad)
        self.assertTrue(any("name" in item for item in errors))
        self.assertTrue(any("scene_id" in item for item in errors))
        self.assertTrue(any("ros_domain_id" in item for item in errors))

        templated = self.manifest()
        templated["scenes"][0]["collect_mapping_bag"]["required_outputs"] = [
            "{mapping_bag}"]
        self.assertEqual([], BATCH.validate_manifest(templated))

    def test_nested_manifest_templates_are_fully_resolved(self):
        context = {
            "workspace": str(self.root),
            "mapping_bag": "{workspace}/mapping_bag",
        }
        self.assertEqual(
            str(self.mapping_bag),
            BATCH.resolve_fully("{mapping_bag}", context),
        )

    def test_bag_independence_contract_blocks_same_dataset(self):
        manifest = self.manifest()
        manifest["scenes"][0]["localization_bag"] = str(self.mapping_bag)
        rows = self.execute_scene(manifest, resume=False, dry_run=False, timeout_override=3)
        self.assertEqual(["UNAVAILABLE", "UNAVAILABLE"], [r["status"] for r in rows])
        self.assertIn("must differ", rows[0]["reason"])
        manifest = self.manifest()
        manifest["scenes"][0]["localization_dataset_id"] = "warehouse_mapping_run_001"
        self.assertTrue(any("dataset IDs" in x for x in BATCH.validate_manifest(manifest)))

    def test_coverage_gate_precedes_offline_mapping(self):
        manifest = self.manifest()
        manifest["scenes"][0]["collect_mapping_bag"]["command"][-1] = (
            f"from pathlib import Path; Path(r'{self.mapping_bag}').mkdir(parents=True, exist_ok=True); "
            f"Path(r'{self.coverage}').write_text('{{\"schema\":\"robot_navigo.map_coverage\","
            f"\"metadata\":{{\"coverage_pct\":99,\"uncovered_cells\":1,"
            f"\"map_fingerprint_sha256\":\"{'a' * 64}\","
            f"\"trajectory_source\":\"{self.mapping_bag}\"}}}}')")
        rows = self.execute_scene(manifest, resume=False, dry_run=False, timeout_override=3)
        self.assertEqual("PASS", rows[0]["status"])
        self.assertEqual("UNAVAILABLE", rows[1]["status"])
        self.assertFalse(self.map_dir.exists())

    def test_phases_are_strictly_serial(self):
        order = self.root / "order.txt"
        manifest = self.manifest()
        manifest["scenes"][0]["collect_mapping_bag"]["command"][-1] += f"; Path(r'{order}').write_text('collect\\n')"
        manifest["scenes"][0]["offline_lightning_mapping"]["command"][-2] += f"; Path(r'{order}').open('a').write('offline\\n')"
        rows = self.execute_scene(manifest, resume=False, dry_run=False, timeout_override=3)
        self.assertEqual(["PASS", "PASS"], [r["status"] for r in rows])
        self.assertEqual(["collect", "offline"], order.read_text().splitlines())

    def test_resume_and_dry_run_are_auditable(self):
        rows = self.execute_scene(resume=False, dry_run=True, timeout_override=3)
        self.assertEqual(["SKIPPED", "SKIPPED"], [r["status"] for r in rows])
        self.assertFalse(self.mapping_bag.exists())
        first = self.execute_scene(resume=False, dry_run=False, timeout_override=3)
        self.assertEqual(["PASS", "PASS"], [r["status"] for r in first])
        self.map_dir.rmdir()
        second = self.execute_scene(resume=True, dry_run=False, timeout_override=3)
        self.assertTrue(second[0]["resumed"])
        self.assertFalse(second[1].get("resumed", False))
        self.assertTrue(self.map_dir.exists())

    def test_dry_run_summary_is_not_reported_as_pass(self):
        suite = self.root / "suite.yaml"
        suite.write_text(__import__("yaml").safe_dump(self.manifest()), encoding="utf-8")
        output = self.root / "batch"
        self.assertEqual(0, BATCH.main([
            "--suite", str(suite), "--output", str(output), "--dry-run"]))
        summary = json.loads((output / "batch_summary.json").read_text())
        self.assertTrue(summary["dry_run"])
        self.assertFalse(summary["pass"])

    def test_missing_evidence_and_unavailable_scene_never_pass(self):
        manifest = self.manifest(available=False)
        rows = self.execute_scene(manifest, resume=False, dry_run=False, timeout_override=3)
        self.assertEqual(["UNAVAILABLE", "UNAVAILABLE"], [r["status"] for r in rows])
        manifest = self.manifest()
        manifest["scenes"][0]["offline_lightning_mapping"]["required_evidence"] = [str(self.root / "missing")]
        rows = self.execute_scene(manifest, resume=False, dry_run=False, timeout_override=3)
        self.assertEqual("UNAVAILABLE", rows[1]["status"])

    def test_lock_atomic_state_and_owned_process_cleanup_boundary(self):
        lock = self.root / "lock"
        with lock.open("a+") as stream:
            fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            with self.assertRaises(BATCH.ManifestError):
                with BATCH.batch_lock(lock):
                    pass
        rows = self.execute_scene(resume=False, dry_run=False, timeout_override=3)
        self.assertEqual("PASS", rows[0]["status"])
        self.assertFalse(list((self.root / "run").rglob("*.tmp")))
        source = SCRIPT.read_text(encoding="utf-8")
        self.assertIn("fcntl.flock", source)
        self.assertIn("start_new_session=True", source)
        self.assertIn("os.killpg(process.pid", source)
        self.assertIn("command_contains_global_cleanup", source)

        process = __import__("subprocess").Popen(
            [sys.executable, "-c", "import time; time.sleep(30)"],
            start_new_session=True,
        )
        BATCH.terminate_process_group(process, grace_sec=0.1)
        self.assertIsNotNone(process.poll())


if __name__ == "__main__":
    unittest.main()

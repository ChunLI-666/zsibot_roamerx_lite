#!/usr/bin/env python3

import importlib.util
import json
import os
import pathlib
import sys
import tempfile
import textwrap
import unittest
from contextlib import redirect_stdout
from io import StringIO

import yaml


SCRIPT = pathlib.Path(__file__).parents[1] / "scripts" / "matrix_localization_batch.py"
SUITE = pathlib.Path(__file__).parents[1] / "regression" / "matrix_localization_suite.yaml"
SPEC = importlib.util.spec_from_file_location("matrix_localization_batch", SCRIPT)
BATCH = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = BATCH
SPEC.loader.exec_module(BATCH)


class MatrixLocalizationBatchTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = pathlib.Path(self.temporary.name)
        self.input_file = self.root / "input.txt"
        self.input_file.write_text("v1\n", encoding="utf-8")

    def tearDown(self):
        self.temporary.cleanup()

    def manifest(self, scenes=None):
        if scenes is None:
            scenes = [self.scene("warehouse", 1)]
        return {
            "schema": BATCH.SCHEMA,
            "schema_version": BATCH.SCHEMA_VERSION,
            "name": "unit",
            "output_root": str(self.root / "outputs"),
            "defaults": {"timeout_sec": 2},
            "variables": {},
            "scenes": scenes,
        }

    def scene(self, name, scene_id, command=None, available=True, enabled=True, reason=None):
        if not available:
            return {
                "name": name,
                "scene_id": scene_id,
                "available": False,
                "enabled": False,
                "reason": reason or "test fixture is unavailable",
            }
        command = command or [
            sys.executable,
            "-c",
            (
                "from pathlib import Path; import json; "
                "p=Path(r'{stage_output}'); p.mkdir(parents=True, exist_ok=True); "
                "(p/'metric.json').write_text(json.dumps({'passed': True}))"
            ),
        ]
        base = {
            "signature_inputs": [str(self.input_file)],
            "commands": [{"name": "fixture", "argv": command}],
            "required_outputs": ["{stage_output}/metric.json"],
            "metrics": [{
                "name": "passed",
                "file": "{stage_output}/metric.json",
                "path": "passed",
                "op": "eq",
                "value": True,
            }],
        }
        return {
            "name": name,
            "scene_id": scene_id,
            "available": True,
            "enabled": enabled,
            "inputs": {},
            "stages": {
                "map": dict(base),
                "init": {**base, "depends_on": ["map"]},
                "tracking": {**base, "depends_on": ["map", "init"]},
                "report": {
                    "depends_on": ["map", "init", "tracking"],
                    "signature_inputs": [str(self.input_file)],
                    "commands": [],
                },
            },
        }

    def test_manifest_validation_requires_reasons_and_all_enabled_stages(self):
        valid = self.manifest([
            self.scene("warehouse", 1),
            self.scene("house", 6, available=False, reason="no map"),
        ])
        self.assertEqual([], BATCH.validate_manifest(valid))

        invalid = self.manifest([{
            "name": "house",
            "scene_id": 6,
            "available": False,
            "enabled": False,
        }])
        self.assertTrue(any("reason" in error for error in BATCH.validate_manifest(invalid)))

    def test_run_writes_atomic_status_and_all_report_formats(self):
        run_dir = self.root / "run"
        rows = BATCH.execute_suite(
            self.manifest(), {}, run_dir, ["map", "init", "tracking", "report"]
        )
        self.assertEqual(["PASS"] * 4, [row["status"] for row in rows])
        status = json.loads((run_dir / "warehouse/map/stage_status.json").read_text())
        self.assertEqual("PASS", status["status"])
        self.assertTrue(status["input_signature"])
        self.assertEqual("runtime", status["signature_inputs"][-1]["kind"])
        self.assertTrue((run_dir / "batch_summary.json").is_file())
        self.assertTrue((run_dir / "batch_summary.csv").is_file())
        self.assertTrue((run_dir / "batch_metrics.csv").is_file())
        self.assertTrue((run_dir / "LOCALIZATION_REPORT.md").is_file())
        report = (run_dir / "LOCALIZATION_REPORT.md").read_text(encoding="utf-8")
        self.assertIn("## 指标明细", report)
        self.assertIn("| warehouse | map | - | passed | True |", report)
        self.assertFalse(list(run_dir.rglob("*.tmp")))

    def test_resume_reuses_only_matching_input_signature(self):
        counter = self.root / "counter.txt"
        command = [
            sys.executable,
            "-c",
            textwrap.dedent(
                f"""
                from pathlib import Path
                import json
                counter = Path(r'{counter}')
                count = int(counter.read_text()) + 1 if counter.exists() else 1
                counter.write_text(str(count))
                output = Path(r'{{stage_output}}')
                output.mkdir(parents=True, exist_ok=True)
                (output / 'metric.json').write_text(json.dumps({{'passed': True}}))
                """
            ),
        ]
        manifest = self.manifest([self.scene("warehouse", 1, command=command)])
        run_dir = self.root / "resume"
        first = BATCH.execute_suite(manifest, {}, run_dir, ["map"])
        second = BATCH.execute_suite(manifest, {}, run_dir, ["map"], resume=True)
        self.assertEqual("PASS", first[0]["status"])
        self.assertTrue(second[0]["resumed"])
        self.assertEqual("1", counter.read_text())

        self.input_file.write_text("v2\n", encoding="utf-8")
        third = BATCH.execute_suite(manifest, {}, run_dir, ["map"], resume=True)
        self.assertFalse(third[0]["resumed"])
        self.assertEqual("2", counter.read_text())
        self.assertEqual(1, len(list((run_dir / "warehouse/.stale").glob("map_*"))))

    def test_directory_signature_detects_content_change_with_preserved_metadata(self):
        directory = self.root / "dataset"
        directory.mkdir()
        data = directory / "data.mcap"
        data.write_bytes(b"first")
        original = data.stat()
        first = BATCH.path_signature(directory)

        data.write_bytes(b"other")
        os.utime(data, ns=(original.st_atime_ns, original.st_mtime_ns))
        second = BATCH.path_signature(directory)
        self.assertNotEqual(first["sha256"], second["sha256"])

    def test_resume_does_not_reuse_metric_failure(self):
        counter = self.root / "metric_counter.txt"
        command = [
            sys.executable,
            "-c",
            (
                "from pathlib import Path; import json; "
                f"c=Path(r'{counter}'); n=int(c.read_text())+1 if c.exists() else 1; c.write_text(str(n)); "
                "p=Path(r'{stage_output}'); p.mkdir(parents=True, exist_ok=True); "
                "(p/'metric.json').write_text(json.dumps({'passed': False}))"
            ),
        ]
        manifest = self.manifest([self.scene("warehouse", 1, command=command)])
        run_dir = self.root / "metric_resume"
        first = BATCH.execute_suite(manifest, {}, run_dir, ["map"])
        second = BATCH.execute_suite(manifest, {}, run_dir, ["map"], resume=True)
        self.assertEqual("METRIC_FAIL", first[0]["status"])
        self.assertEqual("METRIC_FAIL", second[0]["status"])
        self.assertFalse(second[0].get("resumed", False))
        self.assertEqual("2", counter.read_text())

    def test_metric_failure_is_distinct_from_infrastructure_failure(self):
        command = [
            sys.executable,
            "-c",
            (
                "from pathlib import Path; import json; "
                "p=Path(r'{stage_output}'); p.mkdir(parents=True, exist_ok=True); "
                "(p/'metric.json').write_text(json.dumps({'passed': False}))"
            ),
        ]
        manifest = self.manifest([self.scene("warehouse", 1, command=command)])
        rows = BATCH.execute_suite(manifest, {}, self.root / "metric", ["map"])
        self.assertEqual("METRIC_FAIL", rows[0]["status"])

        broken = self.manifest([self.scene("warehouse", 1, command=[sys.executable, "-c", "raise SystemExit(3)"])])
        rows = BATCH.execute_suite(broken, {}, self.root / "infra", ["map"])
        self.assertEqual("INFRA_FAIL", rows[0]["status"])

    def test_timeout_and_keep_going(self):
        slow = self.scene("slow", 1, command=[sys.executable, "-c", "import time; time.sleep(5)"])
        fast = self.scene("fast", 2)
        manifest = self.manifest([slow, fast])
        rows = BATCH.execute_suite(
            manifest,
            {},
            self.root / "timeout",
            ["map"],
            keep_going=True,
            timeout_override=0.05,
        )
        self.assertEqual(["TIMEOUT", "PASS"], [row["status"] for row in rows])

    def test_unavailable_scene_is_reported_as_skipped(self):
        unavailable = self.scene("house", 6, available=False, reason="mapping bag missing")
        rows = BATCH.execute_suite(
            self.manifest([unavailable]), {}, self.root / "unavailable", ["map", "init"]
        )
        self.assertEqual(["SKIPPED", "SKIPPED"], [row["status"] for row in rows])
        self.assertIn("mapping bag missing", rows[0]["reason"])
        report = (self.root / "unavailable/LOCALIZATION_REPORT.md").read_text(encoding="utf-8")
        self.assertIn("## 不可用场景", report)
        self.assertIn("mapping bag missing", report)

    def test_init_cases_continue_after_metric_failure_and_are_reported(self):
        manifest = self.manifest()
        cases = []
        for index, passed in enumerate((True, False, True)):
            cases.append({
                "name": f"offset_{index}",
                "variables": {"expected_offset": str(index * 10)},
                "commands": [{
                    "name": "case_fixture",
                    "argv": [
                        sys.executable,
                        "-c",
                        (
                            "from pathlib import Path; import json; "
                            "p=Path(r'{case_output}'); p.mkdir(parents=True, exist_ok=True); "
                            f"(p/'metric.json').write_text(json.dumps(dict(passed={passed!r}, "
                            "offset=int('{expected_offset}'))))"
                        ),
                    ],
                }],
                "required_outputs": ["{case_output}/metric.json"],
                "metrics": [
                    {
                        "name": "passed",
                        "file": "{case_output}/metric.json",
                        "path": "passed",
                        "op": "eq",
                        "value": True,
                    },
                    {
                        "name": "offset",
                        "file": "{case_output}/metric.json",
                        "path": "offset",
                        "op": "report",
                    },
                ],
            })
        manifest["scenes"][0]["stages"]["init"] = {
            "depends_on": ["map"],
            "continue_on_failure": True,
            "signature_inputs": [str(self.input_file)],
            "cases": cases,
        }
        manifest["scenes"][0]["stages"]["tracking"]["depends_on"] = ["map"]
        manifest["scenes"][0]["stages"]["report"]["depends_on"] = []
        manifest["scenes"][0]["stages"]["report"]["always_run"] = True
        self.assertEqual([], BATCH.validate_manifest(manifest))
        run_dir = self.root / "cases"
        rows = BATCH.execute_suite(manifest, {}, run_dir, ["map", "init", "tracking", "report"])

        self.assertEqual("METRIC_FAIL", rows[1]["status"])
        self.assertEqual(
            ["PASS", "METRIC_FAIL", "PASS"],
            [case["status"] for case in rows[1]["cases"]],
        )
        self.assertEqual("PASS", rows[2]["status"])
        self.assertEqual("PASS", rows[3]["status"])
        self.assertTrue((run_dir / "warehouse/init/cases/offset_2/metric.json").is_file())
        self.assertTrue((run_dir / "warehouse/init/cases/offset_2/case_status.json").is_file())
        report = (run_dir / "LOCALIZATION_REPORT.md").read_text(encoding="utf-8")
        self.assertIn("## 分 Case 结果", report)
        self.assertIn("| warehouse | init | offset_2 | PASS |", report)
        self.assertIn("| warehouse | init | offset_1 | passed | False |", report)
        self.assertFalse(list(run_dir.rglob("*.tmp")))

    def test_init_case_validation_requires_three_unique_path_safe_names(self):
        manifest = self.manifest()
        manifest["scenes"][0]["stages"]["init"] = {
            "cases": [
                {"name": "same", "commands": [{"argv": ["true"]}]},
                {"name": "same", "commands": [{"argv": ["true"]}]},
            ]
        }
        errors = BATCH.validate_manifest(manifest)
        self.assertTrue(any("at least three" in error for error in errors))
        self.assertTrue(any("duplicate case" in error for error in errors))

    def test_optional_report_metric_keeps_missing_value_without_failing(self):
        metric_file = self.root / "optional.json"
        metric_file.write_text('{"passed": true}\n', encoding="utf-8")
        metrics = BATCH.evaluate_metrics([{
            "name": "missing_detail",
            "file": str(metric_file),
            "path": "metrics.detail",
            "optional": True,
            "op": "report",
        }], {})
        self.assertIsNone(metrics[0]["actual"])
        self.assertTrue(metrics[0]["passed"])

    def test_repository_suite_uses_frozen_map_independent_bag_and_required_evaluators(self):
        suite = yaml.safe_load(SUITE.read_text(encoding="utf-8"))
        warehouse = suite["scenes"][0]
        variables = suite["variables"]
        self.assertNotEqual(
            variables["warehouse_mapping_bag"], variables["warehouse_localization_bag"]
        )
        map_stage = warehouse["stages"]["map"]
        provenance_argv = next(
            command["argv"] for command in map_stage["commands"]
            if command["name"] == "verify_archived_map_provenance"
        )
        self.assertIn("sha256sum --check SHA256SUMS", provenance_argv[-1])
        alignment_argv = next(
            command["argv"] for command in map_stage["commands"]
            if command["name"] == "fixed_map_ground_truth_alignment"
        )
        self.assertNotIn("--evaluation-bag", alignment_argv)
        self.assertIn("--source-bag", alignment_argv)
        self.assertIn("--allow-empty-ground-truth-child-frame", alignment_argv)
        self.assertIn("{warehouse_map}/SHA256SUMS", map_stage["required_outputs"])

        init_cases = warehouse["stages"]["init"]["cases"]
        self.assertEqual(["0", "120", "240"], [
            case["variables"]["start_offset_sec"] for case in init_cases
        ])
        for case in init_cases:
            evaluator = next(
                command for command in case["commands"]
                if command["name"] == "absolute_localization_accuracy"
            )
            self.assertIn("--source-bag", evaluator["argv"])
            self.assertIn("--calibration-bag", evaluator["argv"])
        tracking_evaluator = next(
            command for command in warehouse["stages"]["tracking"]["commands"]
            if command["name"] == "absolute_localization_accuracy"
        )
        self.assertIn("--source-bag", tracking_evaluator["argv"])
        self.assertIn("--calibration-bag", tracking_evaluator["argv"])

    def test_missing_signature_input_is_invalid_input(self):
        manifest = self.manifest()
        manifest["scenes"][0]["stages"]["map"]["signature_inputs"] = [
            str(self.root / "missing.bag")
        ]
        rows = BATCH.execute_suite(manifest, {}, self.root / "invalid", ["map"])
        self.assertEqual("INVALID_INPUT", rows[0]["status"])

    def test_validate_and_report_cli_commands(self):
        manifest = self.manifest()
        suite = self.root / "suite.yaml"
        suite.write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
        output = StringIO()
        with redirect_stdout(output):
            self.assertEqual(0, BATCH.main(["validate", "--suite", str(suite)]))

        run_dir = self.root / "cli_report"
        BATCH.execute_suite(manifest, {}, run_dir, ["map"])
        with redirect_stdout(output):
            self.assertEqual(
                0,
                BATCH.main([
                    "report", "--suite", str(suite), "--run-dir", str(run_dir)
                ]),
            )
        self.assertTrue((run_dir / "LOCALIZATION_REPORT.md").is_file())


if __name__ == "__main__":
    unittest.main()

#!/usr/bin/env python3
import json
import os
from pathlib import Path
import subprocess
import tempfile
import textwrap
import unittest


SCRIPT = Path(__file__).parents[1] / "scripts" / "matrix_mapping_collect.sh"


class MatrixMappingCollectTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.matrix_root = self.root / "matrix"
        self.workspace = self.root / "workspace"
        self.matrix_root.mkdir()
        self.workspace.mkdir()

        self.map_dir = self.root / "reference map"
        self.map_dir.mkdir()
        self.map_image = self.map_dir / "map.pgm"
        self.map_image.write_bytes(b"P5\n2 2\n255\n" + bytes((254, 254, 0, 128)))
        self.map_yaml = self.map_dir / "map.yaml"
        self.map_yaml.write_text(textwrap.dedent("""\
            image: map.pgm
            mode: trinary
            resolution: 0.05
            origin: [0.0, 0.0, 0.0]
            negate: 0
            occupied_thresh: 0.65
            free_thresh: 0.25
        """), encoding="utf-8")
        self.route = self.root / "coverage route.json"
        self.route.write_text(json.dumps({
            "frame_id": "map",
            "return_to_start": True,
            "waypoints": [{"x": 0.0, "y": 0.0, "yaw": 0.0}],
        }), encoding="utf-8")
        self.alignment = self.root / "alignment.json"
        self.alignment.write_text(json.dumps({
            "map_T_ground_truth": {"x": 0.0, "y": 0.0, "yaw": 0.0},
        }), encoding="utf-8")

        self.runner_args = self.root / "runner_args.txt"
        self.coverage_args = self.root / "coverage_args.txt"
        self.fake_runner = self.root / "fake_runner.sh"
        self.fake_runner.write_text(textwrap.dedent("""\
            #!/usr/bin/env bash
            set -euo pipefail
            printf '%s\n' "$@" > "$FAKE_RUNNER_ARGS"
            [[ "${FAKE_RUNNER_FAIL:-0}" == "0" ]] || exit 17
            result_dir=""
            while [[ $# -gt 0 ]]; do
              if [[ "$1" == "--result-dir" ]]; then
                result_dir=$2
                shift 2
              else
                shift
              fi
            done
            [[ -n "$result_dir" ]]
            mkdir -p "$result_dir"
            if [[ "${FAKE_RUNNER_SKIP_BAG:-0}" == "0" ]]; then
              mkdir -p "$result_dir/closed_loop_bag"
              printf '%s\n' \
                'rosbag2_bagfile_information:' \
                '  storage_identifier: mcap' \
                > "$result_dir/closed_loop_bag/metadata.yaml"
              : > "$result_dir/closed_loop_bag/data_0.mcap"
            fi
        """), encoding="utf-8")

        self.fake_coverage = self.root / "fake_map_coverage.py"
        self.fake_coverage.write_text(textwrap.dedent("""\
            #!/usr/bin/env python3
            import json
            import os
            from pathlib import Path
            import sys

            Path(os.environ["FAKE_COVERAGE_ARGS"]).write_text(
                "\\n".join(sys.argv[1:]) + "\\n", encoding="utf-8")
            args = sys.argv[1:]
            def value(flag):
                return args[args.index(flag) + 1]
            coverage = float(os.environ.get("FAKE_COVERAGE_PCT", "100"))
            uncovered = int(os.environ.get("FAKE_UNCOVERED_CELLS", "0"))
            output = Path(value("--output"))
            preview = Path(value("--preview"))
            output.write_text(json.dumps({
                "schema": "robot_navigo.map_coverage",
                "schema_version": 1,
                "metadata": {
                    "coverage_pct": coverage,
                    "uncovered_cells": uncovered,
                },
            }), encoding="utf-8")
            preview.write_bytes(b"fake png")
        """), encoding="utf-8")

        self.fake_ros2 = self.root / "fake_ros2.sh"
        self.fake_ros2.write_text(textwrap.dedent("""\
            #!/usr/bin/env bash
            set -euo pipefail
            [[ "$1" == "bag" ]]
            if [[ "$2" == "convert" ]]; then
              [[ "$3" == "-i" && -d "$4" && "$5" == "-o" && -f "$6" ]]
              output=$(python3 -c \
                'import sys,yaml; print(yaml.safe_load(open(sys.argv[1], encoding="utf-8"))["output_bags"][0]["uri"])' \
                "$6")
              mkdir -p "$output"
              : > "$output/data_0.mcap"
              printf '%s\n' \
                'rosbag2_bagfile_information:' \
                '  storage_identifier: mcap' \
                '  topics_with_message_count:' \
                '  - topic_metadata:' \
                '      name: /livox/lidar' \
                '      type: sensor_msgs/msg/PointCloud2' \
                '    message_count: 100' \
                '  - topic_metadata:' \
                '      name: /imu/data_raw' \
                '      type: sensor_msgs/msg/Imu' \
                '    message_count: 2000' \
                > "$output/metadata.yaml"
              exit 0
            fi
            [[ "$2" == "info" && -d "$3" ]]
            printf 'Files: %s/data_0.mcap\n' "$3"
            printf 'Topic: /livox/lidar | Type: sensor_msgs/msg/PointCloud2 | Count: 100\n'
            printf 'Topic: /imu/data_raw | Type: sensor_msgs/msg/Imu | Count: 2000\n'
            if [[ "$3" == */diagnostic_bag ]]; then
              printf 'Topic: /odom/current_pose | Type: nav_msgs/msg/Odometry | Count: 2000\n'
              if [[ "${FAKE_OMIT_GT_TOPIC:-0}" == "0" ]]; then
                printf 'Topic: /odom/mujoco_odom | Type: nav_msgs/msg/Odometry | Count: 2000\n'
              fi
            fi
        """), encoding="utf-8")
        self.fake_runner.chmod(0o755)
        self.fake_coverage.chmod(0o755)
        self.fake_ros2.chmod(0o755)

        self.env = os.environ.copy()
        self.env.update({
            "MATRIX_MAPPING_RUNNER": str(self.fake_runner),
            "MATRIX_MAP_COVERAGE_TOOL": str(self.fake_coverage),
            "ROS2_BIN": str(self.fake_ros2),
            "FAKE_RUNNER_ARGS": str(self.runner_args),
            "FAKE_COVERAGE_ARGS": str(self.coverage_args),
        })

    def tearDown(self):
        self.tmp.cleanup()

    def command(self, result_dir, *extra):
        return [
            "bash", str(SCRIPT),
            "--matrix-root", str(self.matrix_root),
            "--workspace", str(self.workspace),
            "--reference-map", str(self.map_yaml),
            "--gt-alignment", str(self.alignment),
            "--route", str(self.route),
            "--result-dir", str(result_dir),
            "--scene", "3",
            "--headless",
            "--per-goal-timeout", "123",
            "--minimum-coverage-pct", "99",
            *extra,
        ]

    def run_collect(self, result_dir, *extra, env=None):
        return subprocess.run(
            self.command(result_dir, *extra),
            text=True,
            capture_output=True,
            env=env or self.env,
            timeout=15,
        )

    @staticmethod
    def option_value(arguments, option):
        return arguments[arguments.index(option) + 1]

    def test_success_moves_bag_evaluates_coverage_and_records_provenance(self):
        result = self.root / "successful result"
        result.mkdir()
        completed = self.run_collect(result)
        self.assertEqual(0, completed.returncode, completed.stderr)

        mapping_bag = result / "mapping_bag"
        diagnostic_bag = result / "diagnostic_bag"
        self.assertTrue((mapping_bag / "metadata.yaml").is_file())
        self.assertTrue((diagnostic_bag / "metadata.yaml").is_file())
        self.assertFalse((result / "closed_loop_bag").exists())
        self.assertTrue((result / "bag_info.txt").is_file())
        self.assertTrue((result / "mapping_bag_info.txt").is_file())
        self.assertTrue((result / "diagnostic_bag_info.txt").is_file())
        self.assertIn(
            "metadata.yaml",
            (result / "mapping_bag_SHA256SUMS").read_text(encoding="utf-8"),
        )
        self.assertTrue((result / "coverage_metadata.json").is_file())
        self.assertTrue((result / "coverage_preview.png").is_file())
        self.assertTrue((result / "provenance/route.json").is_file())
        self.assertTrue((result / "provenance/reference_map/map.yaml").is_file())
        self.assertTrue((result / "provenance/reference_map/map.pgm").is_file())
        checksums = (result / "input_SHA256SUMS").read_text(encoding="utf-8")
        self.assertIn("provenance/route.json", checksums)
        self.assertIn("provenance/reference_map/map.yaml", checksums)
        self.assertIn("provenance/reference_map/map.pgm", checksums)
        self.assertIn("provenance/mapping_bag_convert.yaml", checksums)

        runner = self.runner_args.read_text(encoding="utf-8").splitlines()
        self.assertEqual("mapping_collect", self.option_value(runner, "--mode"))
        self.assertEqual(str(self.map_yaml.resolve()), self.option_value(runner, "--map"))
        self.assertEqual("3", self.option_value(runner, "--scene"))
        self.assertEqual("123", self.option_value(runner, "--timeout"))
        self.assertEqual("5", self.option_value(runner, "--pre-roll-sec"))
        self.assertEqual("1", self.option_value(runner, "--post-roll-sec"))
        self.assertIn("--headless", runner)

        coverage = self.coverage_args.read_text(encoding="utf-8").splitlines()
        self.assertEqual("evaluate", coverage[0])
        self.assertEqual("/odom/current_pose", self.option_value(coverage, "--topic"))
        self.assertEqual(
            str(diagnostic_bag), self.option_value(coverage, "--trajectory-bag")
        )
        self.assertEqual("0.0", self.option_value(coverage, "--start-x"))
        self.assertEqual("0.0", self.option_value(coverage, "--start-y"))
        self.assertEqual("0.35", self.option_value(coverage, "--safety-radius"))
        self.assertEqual("0.75", self.option_value(coverage, "--coverage-radius"))
        self.assertEqual("mcap", self.option_value(coverage, "--storage-id"))

    def test_custom_existing_empty_mapping_bag_is_replaced_exactly(self):
        result = self.root / "custom result"
        destination = self.root / "external mapping bag"
        destination.mkdir()
        completed = self.run_collect(result, "--mapping-bag", str(destination))
        self.assertEqual(0, completed.returncode, completed.stderr)
        self.assertTrue((destination / "metadata.yaml").is_file())
        self.assertTrue((result / "diagnostic_bag/metadata.yaml").is_file())
        self.assertFalse((result / "mapping_bag").exists())
        self.assertFalse((result / "closed_loop_bag").exists())

    def test_nonempty_result_or_destination_fails_before_runner(self):
        result = self.root / "nonempty result"
        result.mkdir()
        (result / "keep.txt").write_text("do not overwrite", encoding="utf-8")
        completed = self.run_collect(result)
        self.assertNotEqual(0, completed.returncode)
        self.assertIn("result directory must be empty", completed.stderr)
        self.assertFalse(self.runner_args.exists())

        result = self.root / "new result"
        destination = self.root / "nonempty destination"
        destination.mkdir()
        (destination / "keep.txt").write_text("do not overwrite", encoding="utf-8")
        completed = self.run_collect(result, "--mapping-bag", str(destination))
        self.assertNotEqual(0, completed.returncode)
        self.assertIn("mapping bag destination must be empty", completed.stderr)
        self.assertFalse(result.exists())

    def test_missing_runner_bag_fails_without_running_coverage(self):
        result = self.root / "missing bag result"
        env = self.env | {"FAKE_RUNNER_SKIP_BAG": "1"}
        completed = self.run_collect(result, env=env)
        self.assertNotEqual(0, completed.returncode)
        self.assertIn("runner did not produce bag", completed.stderr)
        self.assertFalse(self.coverage_args.exists())

    def test_missing_required_bag_topic_fails_before_coverage(self):
        result = self.root / "missing topic result"
        env = self.env | {"FAKE_OMIT_GT_TOPIC": "1"}
        completed = self.run_collect(result, env=env)
        self.assertNotEqual(0, completed.returncode)
        self.assertIn("diagnostic bag is missing required topic: /odom/mujoco_odom", completed.stderr)
        self.assertTrue((result / "diagnostic_bag/metadata.yaml").is_file())
        self.assertFalse((result / "mapping_bag").exists())
        self.assertFalse(self.coverage_args.exists())

    def test_coverage_below_threshold_fails_closed_but_retains_bag(self):
        result = self.root / "low coverage result"
        env = self.env | {
            "FAKE_COVERAGE_PCT": "98.5",
            "FAKE_UNCOVERED_CELLS": "10",
        }
        completed = self.run_collect(result, env=env)
        self.assertNotEqual(0, completed.returncode)
        self.assertIn("actual coverage 98.500000% is below 99.000000%", completed.stderr)
        self.assertTrue((result / "mapping_bag/metadata.yaml").is_file())
        self.assertTrue((result / "diagnostic_bag/metadata.yaml").is_file())
        self.assertTrue((result / "coverage_metadata.json").is_file())
        self.assertTrue((result / "input_SHA256SUMS").is_file())
        self.assertTrue((result / "provenance/route.json").is_file())


if __name__ == "__main__":
    unittest.main()

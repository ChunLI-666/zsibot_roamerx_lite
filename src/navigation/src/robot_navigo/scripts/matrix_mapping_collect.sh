#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: matrix_mapping_collect.sh --matrix-root DIR --workspace DIR \
  --reference-map FILE --gt-alignment FILE --route FILE --result-dir DIR [options]

Run the Matrix GT/reference-map navigation collector, archive its mapping bag,
split the complete diagnostic recording into a LiDAR/IMU-only mapping bag, and
evaluate actual map-frame trajectory coverage. Lightning localization is never
started by this wrapper.

Required:
  --matrix-root DIR           Matrix repository
  --workspace DIR             Built colcon workspace
  --reference-map FILE        Reference map.yaml used only by GT navigation
  --gt-alignment FILE         Fixed map_T_ground_truth JSON
  --route FILE                Map-frame coverage route JSON
  --result-dir DIR            New or empty result directory

Options:
  --mapping-bag DIR           Final bag path (default: RESULT_DIR/mapping_bag)
  --scene ID                  Matrix scene ID (default: 1)
  --headless                  Run Matrix with off-screen rendering
  --per-goal-timeout SEC      Runner timeout for each route goal (default: 180)
  --pre-roll-sec SEC          Static sensor recording before motion (default: 5)
  --post-roll-sec SEC         Sensor recording after motion stops (default: 1)
  --minimum-coverage-pct PCT  Required actual coverage in (0, 100] (default: 100)
EOF
}

fail() {
  printf '[FAIL] %s\n' "$*" >&2
  exit 1
}

require_empty_or_new_directory() {
  local path=$1 label=$2
  if [[ -e "$path" ]]; then
    [[ -d "$path" ]] || fail "$label exists but is not a directory: $path"
    [[ -z "$(find "$path" -mindepth 1 -maxdepth 1 -print -quit)" ]] || \
      fail "$label must be empty: $path"
  fi
}

MATRIX_ROOT=""
WORKSPACE=""
REFERENCE_MAP=""
GT_ALIGNMENT=""
ROUTE_FILE=""
RESULT_DIR=""
MAPPING_BAG=""
SCENE_ID=1
HEADLESS=0
PER_GOAL_TIMEOUT=180
PRE_ROLL_SEC=5
POST_ROLL_SEC=1
MINIMUM_COVERAGE_PCT=100

while [[ $# -gt 0 ]]; do
  case "$1" in
    --matrix-root) MATRIX_ROOT=${2:-}; shift 2 ;;
    --workspace) WORKSPACE=${2:-}; shift 2 ;;
    --reference-map) REFERENCE_MAP=${2:-}; shift 2 ;;
    --gt-alignment) GT_ALIGNMENT=${2:-}; shift 2 ;;
    --route) ROUTE_FILE=${2:-}; shift 2 ;;
    --result-dir) RESULT_DIR=${2:-}; shift 2 ;;
    --mapping-bag) MAPPING_BAG=${2:-}; shift 2 ;;
    --scene) SCENE_ID=${2:-}; shift 2 ;;
    --headless) HEADLESS=1; shift ;;
    --per-goal-timeout) PER_GOAL_TIMEOUT=${2:-}; shift 2 ;;
    --pre-roll-sec) PRE_ROLL_SEC=${2:-}; shift 2 ;;
    --post-roll-sec) POST_ROLL_SEC=${2:-}; shift 2 ;;
    --minimum-coverage-pct) MINIMUM_COVERAGE_PCT=${2:-}; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) printf '[FAIL] unknown option: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -n "$MATRIX_ROOT" ]] || { usage >&2; fail '--matrix-root is required'; }
[[ -n "$WORKSPACE" ]] || { usage >&2; fail '--workspace is required'; }
[[ -n "$REFERENCE_MAP" ]] || { usage >&2; fail '--reference-map is required'; }
[[ -n "$GT_ALIGNMENT" ]] || { usage >&2; fail '--gt-alignment is required'; }
[[ -n "$ROUTE_FILE" ]] || { usage >&2; fail '--route is required'; }
[[ -n "$RESULT_DIR" ]] || { usage >&2; fail '--result-dir is required'; }

[[ -d "$MATRIX_ROOT" ]] || fail "Matrix repository not found: $MATRIX_ROOT"
[[ -d "$WORKSPACE" ]] || fail "workspace not found: $WORKSPACE"
[[ -f "$REFERENCE_MAP" ]] || fail "reference map not found: $REFERENCE_MAP"
[[ -f "$GT_ALIGNMENT" ]] || fail "GT alignment not found: $GT_ALIGNMENT"
[[ -f "$ROUTE_FILE" ]] || fail "coverage route not found: $ROUTE_FILE"

MATRIX_ROOT=$(realpath "$MATRIX_ROOT")
WORKSPACE=$(realpath "$WORKSPACE")
REFERENCE_MAP=$(realpath "$REFERENCE_MAP")
GT_ALIGNMENT=$(realpath "$GT_ALIGNMENT")
ROUTE_FILE=$(realpath "$ROUTE_FILE")
RESULT_DIR=$(realpath -m "$RESULT_DIR")
if [[ -z "$MAPPING_BAG" ]]; then
  MAPPING_BAG="$RESULT_DIR/mapping_bag"
fi
MAPPING_BAG=$(realpath -m "$MAPPING_BAG")
SOURCE_BAG="$RESULT_DIR/closed_loop_bag"
DIAGNOSTIC_BAG="$RESULT_DIR/diagnostic_bag"

python3 - "$SCENE_ID" "$PER_GOAL_TIMEOUT" "$PRE_ROLL_SEC" \
  "$POST_ROLL_SEC" "$MINIMUM_COVERAGE_PCT" <<'PY'
import math
import sys

scene_text, timeout_text, pre_text, post_text, coverage_text = sys.argv[1:]
try:
    scene = int(scene_text)
    timeout = float(timeout_text)
    pre_roll = float(pre_text)
    post_roll = float(post_text)
    coverage = float(coverage_text)
except ValueError as exc:
    raise SystemExit(f"invalid numeric option: {exc}")
if str(scene) != scene_text.strip() or scene < 0:
    raise SystemExit("--scene must be a non-negative integer")
if not math.isfinite(timeout) or timeout <= 0.0:
    raise SystemExit("--per-goal-timeout must be finite and positive")
if not math.isfinite(pre_roll) or pre_roll < 0.0:
    raise SystemExit("--pre-roll-sec must be finite and non-negative")
if not math.isfinite(post_roll) or post_roll < 0.0:
    raise SystemExit("--post-roll-sec must be finite and non-negative")
if not math.isfinite(coverage) or not 0.0 < coverage <= 100.0:
    raise SystemExit("--minimum-coverage-pct must be in (0, 100]")
PY

python3 - "$RESULT_DIR" "$MAPPING_BAG" "$SOURCE_BAG" "$DIAGNOSTIC_BAG" <<'PY'
import pathlib
import sys

result, destination, source, diagnostic = (
    pathlib.Path(value) for value in sys.argv[1:]
)
if destination == result or result.is_relative_to(destination):
    raise SystemExit("mapping bag must not be the result directory or its ancestor")
if destination == source or destination.is_relative_to(source):
    raise SystemExit("mapping bag must not be closed_loop_bag or one of its descendants")
if destination == diagnostic or destination.is_relative_to(diagnostic):
    raise SystemExit("mapping bag must not be diagnostic_bag or one of its descendants")
PY

require_empty_or_new_directory "$RESULT_DIR" 'result directory'
require_empty_or_new_directory "$MAPPING_BAG" 'mapping bag destination'
mkdir -p "$RESULT_DIR"

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
SELF_PATH=$(realpath "${BASH_SOURCE[0]}")
RUNNER=${MATRIX_MAPPING_RUNNER:-"$SCRIPT_DIR/matrix_closed_loop_run.sh"}
COVERAGE_TOOL=${MATRIX_MAP_COVERAGE_TOOL:-"$SCRIPT_DIR/map_coverage.py"}
ROS2_BIN=${ROS2_BIN:-ros2}
[[ -f "$RUNNER" ]] || fail "Matrix runner not found: $RUNNER"
[[ -f "$COVERAGE_TOOL" ]] || fail "map coverage tool not found: $COVERAGE_TOOL"
command -v "$ROS2_BIN" >/dev/null 2>&1 || fail "ros2 command not found: $ROS2_BIN"
RUNNER=$(realpath "$RUNNER")
COVERAGE_TOOL=$(realpath "$COVERAGE_TOOL")

MAP_IMAGE=$(python3 - "$REFERENCE_MAP" <<'PY'
from pathlib import Path
import sys
import yaml

source = Path(sys.argv[1])
try:
    document = yaml.safe_load(source.read_text(encoding="utf-8"))
except (OSError, yaml.YAMLError) as exc:
    raise SystemExit(f"cannot read reference map YAML: {exc}")
if not isinstance(document, dict) or not isinstance(document.get("image"), str):
    raise SystemExit("reference map YAML has no image path")
image = Path(document["image"]).expanduser()
if not image.is_absolute():
    image = source.parent / image
image = image.resolve()
if not image.is_file():
    raise SystemExit(f"reference map image not found: {image}")
print(image)
PY
)

RUNNER_ARGS=(
  --mode mapping_collect
  --matrix-root "$MATRIX_ROOT"
  --workspace "$WORKSPACE"
  --map "$REFERENCE_MAP"
  --gt-alignment "$GT_ALIGNMENT"
  --route "$ROUTE_FILE"
  --result-dir "$RESULT_DIR"
  --scene "$SCENE_ID"
  --timeout "$PER_GOAL_TIMEOUT"
  --pre-roll-sec "$PRE_ROLL_SEC"
  --post-roll-sec "$POST_ROLL_SEC"
)
if [[ "$HEADLESS" == "1" ]]; then
  RUNNER_ARGS+=(--headless)
fi

printf '[RUN] collecting Matrix mapping data in GT/reference-map mode\n'
bash "$RUNNER" "${RUNNER_ARGS[@]}"

[[ -d "$SOURCE_BAG" ]] || fail "runner did not produce bag: $SOURCE_BAG"
[[ -f "$SOURCE_BAG/metadata.yaml" ]] || \
  fail "runner bag has no metadata.yaml: $SOURCE_BAG"

DIAGNOSTIC_STAGE="${DIAGNOSTIC_BAG}.partial.$$"
[[ ! -e "$DIAGNOSTIC_STAGE" ]] || \
  fail "temporary diagnostic bag path exists: $DIAGNOSTIC_STAGE"
mv -- "$SOURCE_BAG" "$DIAGNOSTIC_STAGE"
[[ -f "$DIAGNOSTIC_STAGE/metadata.yaml" ]] || \
  fail "moved diagnostic bag lost metadata; retained at $DIAGNOSTIC_STAGE"
mv -- "$DIAGNOSTIC_STAGE" "$DIAGNOSTIC_BAG"

"$ROS2_BIN" bag info "$DIAGNOSTIC_BAG" > "$RESULT_DIR/diagnostic_bag_info.txt"
[[ -s "$RESULT_DIR/diagnostic_bag_info.txt" ]] || \
  fail 'ros2 bag info produced no diagnostic output'
for required_topic in \
  /livox/lidar /imu/data_raw /odom/current_pose /odom/mujoco_odom; do
  grep -Fq "Topic: $required_topic |" "$RESULT_DIR/diagnostic_bag_info.txt" || \
    fail "diagnostic bag is missing required topic: $required_topic"
done

STORAGE_ID=$(python3 - "$DIAGNOSTIC_BAG/metadata.yaml" <<'PY'
from pathlib import Path
import sys
import yaml

source = Path(sys.argv[1])
try:
    document = yaml.safe_load(source.read_text(encoding="utf-8"))
    storage = document["rosbag2_bagfile_information"]["storage_identifier"]
except (OSError, KeyError, TypeError, yaml.YAMLError) as exc:
    raise SystemExit(f"cannot determine rosbag storage_identifier: {exc}")
if not isinstance(storage, str) or not storage:
    raise SystemExit("rosbag storage_identifier is empty")
print(storage)
PY
)

require_empty_or_new_directory "$MAPPING_BAG" 'mapping bag destination'
mkdir -p "$(dirname "$MAPPING_BAG")"
if [[ -d "$MAPPING_BAG" ]]; then
  rmdir "$MAPPING_BAG"
fi
MAPPING_STAGE="${MAPPING_BAG}.partial.$$"
[[ ! -e "$MAPPING_STAGE" ]] || \
  fail "temporary mapping bag path exists: $MAPPING_STAGE"
CONVERT_OPTIONS="$RESULT_DIR/mapping_bag_convert.yaml"
python3 - "$CONVERT_OPTIONS" "$MAPPING_STAGE" "$STORAGE_ID" <<'PY'
from pathlib import Path
import sys
import yaml

output, uri, storage_id = sys.argv[1:]
document = {
    "output_bags": [{
        "uri": uri,
        "storage_id": storage_id,
        "all": False,
        "topics": ["/livox/lidar", "/imu/data_raw"],
    }]
}
Path(output).write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
PY
if ! "$ROS2_BIN" bag convert -i "$DIAGNOSTIC_BAG" -o "$CONVERT_OPTIONS" \
    > "$RESULT_DIR/bag_convert.log" 2>&1; then
  fail "failed to create LiDAR/IMU mapping bag; partial output retained at $MAPPING_STAGE"
fi
[[ -f "$MAPPING_STAGE/metadata.yaml" ]] || \
  fail "converted mapping bag has no metadata; retained at $MAPPING_STAGE"
mv -- "$MAPPING_STAGE" "$MAPPING_BAG"

"$ROS2_BIN" bag info "$MAPPING_BAG" > "$RESULT_DIR/mapping_bag_info.txt"
[[ -s "$RESULT_DIR/mapping_bag_info.txt" ]] || \
  fail 'ros2 bag info produced no mapping output'
cp -- "$RESULT_DIR/mapping_bag_info.txt" "$RESULT_DIR/bag_info.txt"
python3 - "$MAPPING_BAG/metadata.yaml" <<'PY'
from pathlib import Path
import sys
import yaml

document = yaml.safe_load(Path(sys.argv[1]).read_text(encoding="utf-8"))
entries = document["rosbag2_bagfile_information"]["topics_with_message_count"]
topics = {
    item["topic_metadata"]["name"]: (
        item["topic_metadata"]["type"], int(item["message_count"])
    )
    for item in entries
}
expected = {
    "/livox/lidar": "sensor_msgs/msg/PointCloud2",
    "/imu/data_raw": "sensor_msgs/msg/Imu",
}
if set(topics) != set(expected):
    raise SystemExit(
        f"mapping bag topic set is {sorted(topics)}, expected {sorted(expected)}"
    )
for name, msg_type in expected.items():
    actual_type, count = topics[name]
    if actual_type != msg_type or count <= 0:
        raise SystemExit(
            f"mapping bag topic {name} has type={actual_type}, count={count}"
        )
PY
(
  cd "$MAPPING_BAG"
  find . -type f -print0 | sort -z | xargs -0 sha256sum
) > "$RESULT_DIR/mapping_bag_SHA256SUMS"
[[ -s "$RESULT_DIR/mapping_bag_SHA256SUMS" ]] || \
  fail 'mapping bag SHA256 manifest was not generated'

PROVENANCE_DIR="$RESULT_DIR/provenance"
mkdir -p "$PROVENANCE_DIR/reference_map" "$PROVENANCE_DIR/sources"
cp -- "$REFERENCE_MAP" "$PROVENANCE_DIR/reference_map/$(basename "$REFERENCE_MAP")"
cp -- "$MAP_IMAGE" "$PROVENANCE_DIR/reference_map/$(basename "$MAP_IMAGE")"
cp -- "$ROUTE_FILE" "$PROVENANCE_DIR/route.json"
cp -- "$GT_ALIGNMENT" "$PROVENANCE_DIR/gt_alignment.json"
cp -- "$CONVERT_OPTIONS" "$PROVENANCE_DIR/mapping_bag_convert.yaml"
cp -- "$SELF_PATH" "$PROVENANCE_DIR/sources/matrix_mapping_collect.sh"
cp -- "$RUNNER" "$PROVENANCE_DIR/sources/matrix_closed_loop_run.sh"
cp -- "$COVERAGE_TOOL" "$PROVENANCE_DIR/sources/map_coverage.py"

{
  printf 'role\toriginal_path\tarchived_path\n'
  printf 'reference_map\t%s\t%s\n' "$REFERENCE_MAP" \
    "$PROVENANCE_DIR/reference_map/$(basename "$REFERENCE_MAP")"
  printf 'reference_image\t%s\t%s\n' "$MAP_IMAGE" \
    "$PROVENANCE_DIR/reference_map/$(basename "$MAP_IMAGE")"
  printf 'route\t%s\t%s\n' "$ROUTE_FILE" "$PROVENANCE_DIR/route.json"
  printf 'gt_alignment\t%s\t%s\n' "$GT_ALIGNMENT" "$PROVENANCE_DIR/gt_alignment.json"
  printf 'bag_conversion\t%s\t%s\n' "$CONVERT_OPTIONS" \
    "$PROVENANCE_DIR/mapping_bag_convert.yaml"
  printf 'collector\t%s\t%s\n' "$SELF_PATH" \
    "$PROVENANCE_DIR/sources/matrix_mapping_collect.sh"
  printf 'runner\t%s\t%s\n' "$RUNNER" \
    "$PROVENANCE_DIR/sources/matrix_closed_loop_run.sh"
  printf 'coverage_tool\t%s\t%s\n' "$COVERAGE_TOOL" \
    "$PROVENANCE_DIR/sources/map_coverage.py"
} > "$RESULT_DIR/source_files.tsv"

(
  cd "$RESULT_DIR"
  sha256sum \
    "provenance/reference_map/$(basename "$REFERENCE_MAP")" \
    "provenance/reference_map/$(basename "$MAP_IMAGE")" \
    provenance/route.json provenance/gt_alignment.json \
    provenance/mapping_bag_convert.yaml \
    provenance/sources/matrix_mapping_collect.sh \
    provenance/sources/matrix_closed_loop_run.sh \
    provenance/sources/map_coverage.py
) > "$RESULT_DIR/input_SHA256SUMS"

COVERAGE_METADATA="$RESULT_DIR/coverage_metadata.json"
COVERAGE_PREVIEW="$RESULT_DIR/coverage_preview.png"
python3 "$COVERAGE_TOOL" evaluate \
  --map-yaml "$REFERENCE_MAP" \
  --trajectory-bag "$DIAGNOSTIC_BAG" \
  --topic /odom/current_pose \
  --storage-id "$STORAGE_ID" \
  --start-x 0.0 --start-y 0.0 \
  --safety-radius 0.35 --coverage-radius 0.75 \
  --output "$COVERAGE_METADATA" --preview "$COVERAGE_PREVIEW"
[[ -s "$COVERAGE_METADATA" ]] || fail 'coverage metadata was not generated'
[[ -s "$COVERAGE_PREVIEW" ]] || fail 'coverage preview was not generated'

python3 - "$COVERAGE_METADATA" "$MINIMUM_COVERAGE_PCT" <<'PY'
import json
import math
from pathlib import Path
import sys

source = Path(sys.argv[1])
threshold = float(sys.argv[2])
try:
    document = json.loads(source.read_text(encoding="utf-8"))
    metadata = document["metadata"]
    coverage = float(metadata["coverage_pct"])
except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
    raise SystemExit(f"invalid coverage metadata: {exc}")
if document.get("schema") != "robot_navigo.map_coverage" or not math.isfinite(coverage):
    raise SystemExit("coverage metadata has an invalid schema or percentage")
if coverage + 1e-9 < threshold:
    raise SystemExit(f"actual coverage {coverage:.6f}% is below {threshold:.6f}%")
if threshold >= 100.0 and metadata.get("uncovered_cells") != 0:
    raise SystemExit("100% coverage requires uncovered_cells=0")
print(f"[PASS] actual trajectory coverage: {coverage:.6f}%")
PY

printf '[PASS] mapping bag: %s\n' "$MAPPING_BAG"
printf '[PASS] diagnostic bag: %s\n' "$DIAGNOSTIC_BAG"
printf '[PASS] coverage evidence: %s\n' "$COVERAGE_METADATA"
printf '[PASS] provenance: %s\n' "$PROVENANCE_DIR"

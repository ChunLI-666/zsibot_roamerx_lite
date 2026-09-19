# Offline MPPI cost audit

This diagnostic links the project's real MPPI library and critic plugins. It does not publish actuator commands. It supports one optimization iteration, Omni motion and disabled forward alignment. It evaluates frozen recorded contexts, not a closed-loop simulation or recovery of historical optimizer state.

The completed experiment used baseline `c34631c`, historical config `6942974`, ROS Jazzy from the existing workspace, a private Release MPPI build and an x86 AVX2 diagnostic build. `CMakeLists.txt` currently assumes AVX2/FMA; adapt those flags before using other hardware. Runtime libraries and inputs are hashed before/after each matrix.

## Reproduce

Use the canonical checkout and a **new** output directory. Source the installed ROS environment and workspace dependencies first. The commands below use task-specific shell variables; do not reuse an existing experiment output.

```bash
source /opt/ros/jazzy/setup.bash
source /home/charles/project/colcon_ws/install/setup.bash
nav_repo=/home/charles/project/colcon_ws/src/zsibot/zsibot_roamerx_lite
audit_src="$nav_repo/src/navigation/src/robot_navigo/scripts/mppi_cost_audit"
audit_root=/home/charles/project/colcon_ws/analysis_outputs/mppi_cost_audit_new
mkdir "$audit_root"
cmake -S "$nav_repo/src/navigation/src/navigo_mppi_controller" -B "$audit_root/mppi_build" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$audit_root/install" -DBUILD_TESTING=OFF
cmake --build "$audit_root/mppi_build" -j2
cmake --install "$audit_root/mppi_build"
cmake -S "$audit_src" -B "$audit_root/probe_build" \
  -Dnavigo_mppi_controller_DIR="$audit_root/install/share/navigo_mppi_controller/cmake"
cmake --build "$audit_root/probe_build" -j2
python3 "$audit_src/prepare_audit.py" --workspace /home/charles/project/colcon_ws --output "$audit_root/inputs"
python3 "$audit_src/run_audit.py" --root "$audit_root"
python3 "$audit_src/analyze_audit.py" --root "$audit_root"
python3 "$audit_src/validate_native.py" --root "$audit_root"
```

Input preparation needs the existing extracted debug/plan JSON and original final 0829 bag at the paths in `prepare_audit.py`. It invokes the repository's costmap reconstruction utility, preserves recorded timestamps and validates frame/goal consistency. Data paths are local to this workspace. Python requires numpy, PyYAML, matplotlib and the existing reconstruction utility's ROS/MCAP dependencies.

## Outputs and interpretation

- `inputs/protocol.json`: nine contexts, parameter origin, timestamps and limitations. `params_weight*.yaml` contains the only three weight variants.
- `runs.json`, per-case `case.yaml`, `process.log`: invocation and exit evidence.
- `trace.csv`: 500 candidate rows with each native cumulative critic increment, regularization, total, softmax weight and trajectory summary.
- `trace.yaml`: final diagnostic command, full/local goal distances, batch reference index, failure/skipped flags.
- `results.json`, `fixed_cost_comparison.png`: validated summary and fixed alternative plot.
- `native_validation/results.json`: 18 comparisons to the unmodified `evalControl` call.
- `identity_before/after.json`: input/binary/dependency immutability evidence.

`mean_control_vx<0` classifies a candidate for aggregate weight mass; it is not the probability of executing a reverse command. `first_control_vx` is the output-index candidate control **before** final averaging/constraints/SG filtering. `best_forward` includes zero-mean candidates. Only `diagnostic_command` with `command_valid=true` is a sampled-batch output; fixed alternatives are never treated as control decisions. No downstream velocity smoother, SDK or dynamics is executed here.

Two nominal assumptions are provided: all-zero and constant clipped measured velocity. Neither restores historical warm state; SG history starts at zero. All-cost checks use native float32 increments, including ULP effects after large collision costs. A failed critic marks later terms skipped, not successfully evaluated zero. The tool does not implement fallback retries; failed batches cannot support output comparisons.

The InflationLayer object supplies real metadata to CostCritic but is not activated; recorded grids are loaded afterward and not reinflated. Historical actual parameters, internal map timing, nominal/noise/filter states are unknown. Full installed debug goal is closer to the controller contract than a raw planner endpoint, but its TF timestamp may differ from the optimizer's.

The matrix runner, native validator and C++ trace writer refuse existing output destinations. The analyzer intentionally refreshes summaries/plots. Do not run Python with `-O`: its validation assertions must remain enabled. Full candidate timestep arrays are not exported; equal weight-pair summaries are weaker than full trajectory equivalence.

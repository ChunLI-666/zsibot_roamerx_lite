# Official path-interface local experiment

This directory is an isolated x86_64/Jazzy experiment. It is not a production
controller server and does not deploy or publish robot velocity commands.
The robot and the main session's source/build/install are not modified.

The frozen official controller revision is recorded in
`evidence/runtime_source_integrity.json`. New official interfaces are exercised
with Jazzy node/publisher compatibility wrappers. A separate helper patch retains
the entire in-place rotation run before releasing subsequent path segments;
`evidence/complete_rotation_constraint.patch` records that behavior change.

Run the synthetic controller/action fixtures with:

```bash
bash run_isolated.sh complete-rotation
```

The global-planner tests use the installed official Jazzy Smac plugins, a frozen
1 cm stop/turn control set from the main experiment, and identical padded
rectangular footprints in the planner and controller. The control set's origin
is recorded in `inputs/global_planner/manifest.json`; it is not claimed to be an
unmodified official generator output. Diagnostic metadata variants are separate
files. The copied old real map is a static test input, not the current board map.

`evidence/global_planner_comparison.json` retains the original global-planner
comparison. `evidence/ab_chain_result.json` contains the initial planner-to-new-
controller scenarios and negative controls. The final tighter-tolerance results
are in `evidence/ab_chain_tight_xy_result.json`. Simulated motion is ideal
constant-twist integration with dense footprint sweeps, not quadruped dynamics.

Current verified findings:

- The recorded near-goal case can rotate in place and finish with a very short
  path instead of the Hybrid loop.
- The main stop/turn control set still loops in another near-goal case when its
  Dubins distance estimate uses the old 0.5 m radius. With the same actions and
  exact search tolerance, using a 0.01 m radius in the distance-estimate metadata
  removes that loop. This is not setting Hybrid's physical curve radius to zero.
- Lattice goal-heading quantization can violate the original requested yaw
  tolerance. An explicit adapter test appends that yaw at the planner's terminal
  XY; raw planner output and adapted output are reported separately.
- A real planner path through the synthetic L corridor is collision-free, but
  the initial controller tuning starts rotating too early and rejects the turn.
  This negative result is retained in `evidence/baseline_xy4cm/`. Consistent
  1 cm controller XY / 2 cm handler XY tolerances pass this counterexample.
- Final adapted results pass all five scenarios at both tested turn speeds.
  The original requested pose, not only the discretized path endpoint, is
  checked. Cancel/new-goal and actual legacy action round trips also pass.

Reproduce the planner/controller suite with:

```bash
bash run_planner_chain_isolated.sh
```

The local result supports an isolated native ARM build/software test. This
fixture is not a production ControllerServer. Actual BT requests and its
velocity output into the existing smoother/safety-gate chain still need to be
connected and jointly tested. Those existing downstream modules need not be
rewritten. Production progress checking, actual TF/odom, robot execution and
quadruped dynamics are outside this fixture. No board action is performed.

The user requested preservation on a separate feature branch without switching
the canonical checkout. This experiment is stored below `experiments/` on that
branch only; the main branch, working files, and real index are not used to stage
or check out the snapshot. No push or board deployment is included.

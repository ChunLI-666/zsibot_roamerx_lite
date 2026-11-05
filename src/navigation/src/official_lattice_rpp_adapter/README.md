# Official Lattice, feasible path handling and RPP experiment

This opt-in Jazzy package composes the installed Smac State Lattice and Regulated
Pure Pursuit plugins. It adapts the frozen upstream feasible path handler to the
Jazzy lifecycle ABI; it is not a native newer Nav2 release. Source identities and
compatibility changes are in `upstream_provenance.json`.

`CheckedLattice` validates complete filled-footprint paths and preserves the
original requested endpoint. `FeasibleRPP` retains ordered rotation boundaries,
checks actual odometry/TF freshness, and checks the remaining rotation and command
sweep at the actual body pose. It must run with the matching `FeasibleGoalChecker`
so an intermediate visit to the final position cannot finish an unconsumed route.
Use non-stateful RPP and goal checking, no reverse, and the installed 1 cm / 16
heading primitive file. `CheckedHybrid` provides a separate DUBIN control group;
its same-position rotation shortcut does not establish native DUBIN turn-in-place
capability.

Jazzy 1.3.13 Lattice returns one pose containing the exact start position and
goal orientation when both requests occupy the same continuous-map cell.
The adapter recognizes that verbatim output before float restoration only when
the XY displacement is strictly below the path rotation equivalence of 0.1 mm.
It retains the original start, checks the complete filled rotation and tiny
terminal connection, and appends the exact original goal. Larger same-cell
displacements and other malformed endpoints still fail; the general 1 nm goal
XY check remains. This compatibility does not request a bearing turn to correct
localization noise or change controller goal tolerances.

`COLCON_IGNORE` deliberately keeps this experiment out of the product build.
Build with a clean Jazzy environment, a separate CMake build directory and install
prefix, then run CTest. The sibling `robot_feasible_navigation` supplies the shared
filled-footprint sweep header; its exact commit is part of the build identity.
Runtime evidence must identify both this library and the installed Nav2 libraries.

The November 5 x86 tests use real Nav2 Actions and an explicitly ideal velocity
plant. They cover ordered turns, loop completion, sparse paths, preemption and
fail-closed behavior for stale inputs and obstacles. Those results do not establish
Matrix dynamics, leg clearance, physical braking distance or hardware acceptance.

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

The separate `terminal_goal_binding` mode defaults to false. Enable it as a
paired CheckedLattice/FeasibleRPP profile only, with frozen matching global and
local grid resolution. The planner proves the official endpoint belongs to the
requested goal cell, retains every checked raw SE2 pose and terminal rotation,
then appends the exact original goal for the standard server goal checker.
That final goal-region edge is a conservative filled-space certificate, not a
translation for RPP to execute. The controller separates it only in this explicit
mode. It enters actual-origin terminal rotation only after ordered mandatory
prefix progress and while the actual body is inside the original goal circle.
Every remaining heading is consumed in order; leaving the circle fails rather
than pursuing a lateral quantization correction. Original checker tolerances are
unchanged. RPP's internal terminal XY trigger is narrowed by the endpoint residual
so stopping near the lattice endpoint cannot imply arrival outside the original
goal region. A new plan, reset or cancellation invalidates prior completion.
The opt-in mode also uses ordered heading checkpoints for active nonterminal
rotations, including signed half-turns and crossings of the angle wrap boundary.
Winding and direction reversals retain their full heading sequence; an entire
run's matching final pose cannot authorize skipping its checkpoints. An actual
shortest-angle command opposing the next ordered checkpoint also fails, with
only a float-roundoff allowance. Plain-mode behavior remains unchanged.
Consecutive numerically co-located samples must stay within the same 0.1 mm
anchor region over the entire run. Many individually tiny translated steps cannot
disguise a curved loop as a single in-place rotation.

In this explicit mode, completion only permits a fresh goal check. The checker
reads the latest actual body TF directly in the immutable original goal frame,
with the controller's existing TF age, future, regression and steady-progress
limits, then uses the unchanged non-stateful SimpleGoalChecker tolerances. It
does not reuse the server's historical transformation of a stamped path endpoint.
Success also revalidates the same immutable plan, completion epoch and generation;
a reset, cancellation or replacement during that check invalidates its result.
The server's reset after setPlan revokes completion while retaining the new goal
binding. Controller reset, cleanup and the beginning of any new setPlan clear it.
Default-off mode retains the server's existing pose/goal check.

Intermediate anchors use the declared inversion XY tolerance and ordered heading
checkpoints. This does not guarantee execution of every translational sample or
the complete path arc length: short legs whose anchor circles overlap may require
all headings while allowing no translation. The final straight also permits
goal-region truncation. Validation must report these positional semantics apart
from exact original-goal XY/yaw acceptance.

`COLCON_IGNORE` deliberately keeps this experiment out of the product build.
Build with a clean Jazzy environment, a separate CMake build directory and install
prefix, then run CTest. The sibling `robot_feasible_navigation` supplies the shared
filled-footprint sweep header; its exact commit is part of the build identity.
Runtime evidence must identify both this library and the installed Nav2 libraries.

The November 5 x86 tests use real Nav2 Actions and an explicitly ideal velocity
plant. They cover ordered turns, loop completion, sparse paths, preemption and
fail-closed behavior for stale inputs and obstacles. Those results do not establish
Matrix dynamics, leg clearance, physical braking distance or hardware acceptance.

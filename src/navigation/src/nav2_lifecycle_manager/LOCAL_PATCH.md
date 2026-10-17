# Lifecycle callback lifetime overlay

This is the official ROS Jazzy `nav2_lifecycle_manager` 1.3.13 source package,
vendored into the canonical navigation repository. The source archive digest and
all original file digests are recorded in `upstream_provenance.json`. Its original
header and package.xml, before local changes, match the locally installed 1.3.13
binary package byte for byte. The current package.xml adds only the explicit
`class_loader` test dependency for the new component unload probe; production
dependencies are unchanged. Apache-2.0 headers are retained; LICENSE contains
the full license text.

The production changes are limited to callback lifetime and shutdown ownership.
A private RAII registration retains the Context and returned pre-shutdown handle;
the manager destructor resets it before other members begin destruction. If the
constructor throws after registration, member unwinding removes it as well.

The pre-shutdown callback only logs its invocation. It does not join the manager
worker or destroy its client maps while Context still needs to notify nested
client executors of shutdown. The existing destructor first joins the worker, then stops the bond timer and
releases node names, clients and bonds, before the Node base is destroyed. Timer
ownership is not mutated concurrently with the worker from a Context callback.
No join runs inside a Context shutdown callback or its callback-list mutex.

Lifecycle clients borrow their manager through an empty-owner shared_ptr alias
instead of retaining an owning shared_from_this reference. This breaks the
manager -> client -> parent manager cycle so destruction is reachable. Current
clients are confined to node_map_, are never returned or published, and are only
used by this manager's worker; that worker is joined before clients or the parent
Node are destroyed. The alias does not create a control block or replace the
manager's actual shared_from_this owner. Future changes must preserve this
non-escaping, manager-owned client lifetime contract. Bond construction also uses
the borrowed Node alias, so an activation result arriving while the destructor
joins its worker cannot reacquire shared_from_this or transfer final ownership
to that worker. Bond still retains the same Node interfaces and protocol settings.

Without this removal, Context can retain the callback after node destruction
and component dlclose, then jump to its unloaded std::function manager while
Context itself is destroyed. See the workspace native_shutdown_rootcause.md
for the precise core evidence.

Package name, executable, component class, production dependencies and default launch
remain upstream-compatible. This package intentionally overrides the installed
package through a normal colcon overlay; it does not modify /opt. No preload,
RTLD_NODELETE shim or library pin is part of the fix. No navigation algorithm,
transition policy, service request/result, timeout, retry or bond protocol semantics
are changed. Only shutdown-time timer/resource destruction order and parent ownership
are changed.

Two existing timer creation sites also check the manager Node's own Context:
the 200 ms bond check timer and the 1 s respawn timer. If Context has already
closed, no new timer is created. If it closes between the check and construction,
only RCLError from that timer creation is suppressed, and only while the same
Context is invalid; errors under a still-valid Context are rethrown. The existing
hard reset is outside that catch. After respawn state queries, a closed Context
also prevents a fresh startup sequence. Live-Context recovery policy, timers,
timeouts, transitions and retries remain unchanged.

Added regressions cover manager destruction before Context shutdown, deferred
manager cleanup after Context shutdown, constructor exceptions, and real
component factory loads and unloads with a Context that remains alive. The unload
probe does not link the manager library and checks /proc/self/maps to ensure the
library is actually gone before shutting down/destroying Context. A populated
mode also initializes real manager-owned clients, handshakes with the manager
worker, and proves that these clients do not keep the component or DSO alive.

Six further child-process regressions use the real default Context and the
actual manager service worker: initialized idle-client ownership/destruction,
startup with an intentionally unanswered lifecycle request, and a shutdown
service request blocked the same way while Context shuts down. A real paired
bond handshake followed by Context shutdown exercises active cleanup. A finite
held ACTIVATE reply is released only after the destructor has begun joining,
covering Bond construction without an owning shared_from_this reacquisition. Their external
test deadlines detect a deadlock; no production timeout or retry is changed.
A further real paired-Bond case begins hard reset during a fixture's pre-shutdown
callback, holds its DEACTIVATE response, and lets Context shutdown interrupt that
wait. The exact previous manager/Bond binaries reproduce SIGABRT in the 1 s
respawn timer creation; a parent gdb captured checkBondConnections ->
create_wall_timer -> TimerBase. The fixed test requires proof that reset began
inside pre-shutdown, then verifies normal Context shutdown and worker destruction.
These tests do not extend support to hot-unloading a manager whose Context is
still valid while an RPC never responds; no production wait limit is changed.
The exact first-stage registration-only source and library are retained in the
workspace analysis output for a before/after control.

For GNU builds the core target uses `-fno-gnu-unique`, matching the installed
library's absence of GNU_UNIQUE symbols. GNU_UNIQUE symbols can make glibc keep a
DSO mapped implicitly and conceal the defect. This option permits real unloading;
it does not pin the library or change navigation behavior.

When updating the vendored upstream version, reapply the registration lifetime
patch and unload build option, refresh provenance, and run these tests plus default navigation
replay. Keep the package in the normal build package list and verify package
index/library resolution from the intended overlay before running navigation.

ABI note: the private registration member changes `LifecycleManager` class layout.
Build/install its core, component factory and standalone main together from this
overlay; do not replace only the shared object for an external consumer that
allocates this class using an old header. The default component factory is built
inside the same core library. `LifecycleManagerClient` and its header are unchanged.

The manager shutdown fix requires the sibling bondcpp 4.2.0 overlay: it safely
releases an active Bond after its Node Context closes without creating a final
notification timer. Keep both packages in the normal build and verify that the
local bondcpp library is actually resolved. Its public headers and ABI are
unchanged; alive-Context heartbeat/protocol/timeout behavior remains upstream.

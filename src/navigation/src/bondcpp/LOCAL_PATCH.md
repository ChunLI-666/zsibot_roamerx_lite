# Context-safe Bond resource destruction

This is the exact ROS Jazzy bondcpp 4.2.0 source package (4.2.0-1noble),
matching the installed 4.2.0 binary package headers and package.xml byte for byte.
Original BSD-3-Clause license and headers are retained. Source archive and file
hashes are in upstream_provenance.json.

Only the destructor notification path changes: after the Bond Node's own Context
has closed, do not call breakBond, which can create a dead-publication timer.
Existing timer/subscription resources are still cancelled and released. While
the Context is valid, the original break/wait/protocol behavior remains intact.
A close racing with the first validity check can raise RCLError; that error is
ignored only if the same Context has become invalid. For a still-valid Context,
terminate as the original noexcept destructor would, preserving error visibility.
Other exception types are not caught. No heartbeat, timeout or bond state-machine
behavior is changed while running. No public header, class layout or ABI changes.

Two child-process tests form a real paired Bond, then test destruction after its
Node-specific Context closes and three concurrent Context-close/destructor
rounds. The global Context remains valid to catch the old global-ok shortcut.
GNU builds disable GNU_UNIQUE symbols so implicit NODELETE cannot hide unload
faults. This normal colcon overlay does not modify /opt or require a library pin.

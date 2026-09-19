"""Wall-time ideal velocity plant with an explicit command watchdog.

No actuator inertia or quadruped gait is claimed. Moving intervals are subdivided
for geometry auditing, never clipped or discarded after a delayed callback.
"""
import math
from scene import integrate

COMMAND_WATCHDOG_NS = 350_000_000


def advance_interval(pose, command, start_ns, end_ns, receipt_ns,
                     blocked=False, watchdog_ns=COMMAND_WATCHDOG_NS, step_ns=20_000_000):
    if end_ns < start_ns or watchdog_ns <= 0 or step_ns <= 0:
        raise ValueError('Invalid plant time interval')
    if len(pose) != 3 or len(command) != 3 or not all(math.isfinite(v) for v in [*pose, *command]):
        raise ValueError('Plant pose and command must be finite three-vectors')
    expiry_ns = receipt_ns + watchdog_ns
    boundaries = sorted({start_ns, end_ns, *[v for v in (receipt_ns, expiry_ns) if start_ns < v < end_ns]})
    current = list(pose)
    segments = []
    for begin, end in zip(boundaries, boundaries[1:]):
        velocity = list(command) if not blocked and receipt_ns <= begin < expiry_ns else [0., 0., 0.]
        cursor = begin
        while cursor < end:
            stop = min(end, cursor + step_ns) if any(velocity) else end
            dt = (stop-cursor)/1e9
            after = integrate(current, velocity, dt)
            segments.append(dict(before=current.copy(), after=after.copy(), executed=velocity.copy(),
                                 dt=dt, interval_start_ns=cursor, interval_end_ns=stop))
            current = after
            cursor = stop
    executed = list(command) if not blocked and receipt_ns <= end_ns < expiry_ns else [0., 0., 0.]
    return current, executed, segments

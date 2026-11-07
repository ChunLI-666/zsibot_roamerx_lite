Independent Matrix goal pilot source

Copy these files into an analysis output directory before running. The pilot intentionally restricts per-attempt output to descendants of its own directory, saves its source and invocation with every attempt, and never overwrites an attempt. Default mode observes only. --execute requires current domain185 inputs, a fresh launcher READY lease and one explicit absolute or body-relative goal.

The original pilot SHA256 15967872f575b3a0106aaf41766bff0c3cd7a72219208921351d9826cadd9405 was used for the first two Matrix navigation attempts on November 5. Both were canceled for scan freshness loss and remain invalid-input cases; their original per-attempt sources and the earlier analysis pilot are preserved. This revised source adds explicit continuous preflight and has offline mocked-clock/guard coverage; it has not yet dispatched a live goal. Goal requests/results and exact settled original-goal errors are published as String events for the recorder and also saved locally.

--stable-preflight-seconds defaults to 0, retaining the original first-ready behavior. A positive value requires that uninterrupted healthy interval after Action-server discovery and before resolving the body-relative goal or sending a request. Every spin rechecks the unchanged source ages, native TF, front camera, launcher lease and runtime identity, event subscriber, measured stop and commanded stop through execution_ready(require_stopped=True). Any failed check resets the interval. The original immediate final check remains; its failure sends no goal and cannot recover through the old one-second first-ready wait.

For the reviewed same-host run, use --preflight-seconds 40 --stable-preflight-seconds 10. The 40-second budget starts after the original event-subscriber discovery gate (up to 5 seconds), and includes initial source readiness, Action discovery and the continuous interval. Failure to accumulate the interval within that budget returns invalid_preflight_not_sent with zero goals. Events record budget start, healthy-interval start, interruptions and final timing/counters; result.json includes the continuous-preflight summary when that phase was reached. All durations must be finite and nonnegative, the stable interval must not exceed the preflight budget, and both goal forms must contain finite numbers.

Offline evidence: analysis_outputs/planner_comparison_20261105/stable_preflight_tests (workspace-relative). These tests use the actual guard and run_case methods with a virtual clock and an ActionClient stand-in that captures a request before transport. They create no ROS node or graph and send no goal or UDP packet. Earlier guard probes and observe-only Node initialization evidence apply to their original source snapshots.

Optional long-route scan callback confirmation (2026-11-07)

--scan-aging-confirm-sec defaults to 0; the explicit maximum is .05 seconds.
After batch acceptance, a sole old scan-header finding may drain the existing
callbacks for that bounded interval. Recovery requires a strictly newer original
scan stamp and every original freshness/frame/finite/ownership check to pass.
Other invalid inputs remain fatal. The .3 second input limits and original
observer bytes are unchanged. Preflight remains strict. Each episode permanently
records a performance violation and strict timing FAIL, even after recovery;
continuous stopped dwell resets across it. This observation policy can collect
a complete functional route while exposing timing violations; it does not
convert the attempt to strict timing acceptance. Ownership diagnostics now retain
the exact failing stage, validation time and original READY snapshot.

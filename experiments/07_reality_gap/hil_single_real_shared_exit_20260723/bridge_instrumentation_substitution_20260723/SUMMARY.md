# Instrumented-bridge live substitution -- epuck5809 (2026-07-23)

**Classification: bounded, stationary, diagnostic-only live substitution.**
Not a formal HIL trial, not ground motion. Executed via the committed,
tested runner `run_bridge_instrumentation_substitution.sh --execute 5477`
(commit `238a2cd`), exactly as tested. Robot stationary throughout, user
monitoring. No guard, controller, virtual peer, `goal_navigator`,
Webots, or rosbag started; no `/cmd_vel` publisher at any point.

## Execution summary

- Original bridge PID **5477** stopped via `kill -INT`; confirmed exited
  before the instrumented bridge started.
- Exactly one bridge process verified present at every stage (before
  stop, after instrumented start, after rollback).
- Instrumented bridge ran for the full 130s capture window (PID 7881).
- Instrumented bridge stopped via `kill -INT` on its exact PID; its
  diagnostic CSV confirmed closed and non-empty before proceeding.
- Original, unmodified bridge restarted immediately, identical
  parameters, no diagnostic flags -- **new PID 7961**.
- Post-rollback health, all confirmed: `connected=true`,
  `validity_flags=7`, sensor topics present, `/cmd_vel Publisher count: 0`.
- Total wall time for the whole script: 2m31.320s.

## Evidence

`bridge_diagnostic_events.csv` (8114 rows), SHA-256-verified identical
between the native WSL path and the local (gitignored) Windows copy in
`raw_logs/` (manifest: `raw_logs/SHA256SUMS.txt`).

## Analysis

**GC instrumentation -- a clean negative result.** 13 real garbage
collections occurred during the 130s window (12x generation 0, 1x
generation 1). Every one was sub-millisecond: durations ranged
30-650 microseconds. **This rules out Python garbage collection as the
cause of either this run's timer irregularities or the ~0.9-1.0s
freezes captured in the prior `TARGETED_STATIONARY_DIAGNOSTIC`** -- no
GC cycle recorded here comes anywhere close to that duration.

**The ~32-33s periodicity recurred a fourth time, independently
confirmed, but at much smaller magnitude this run.** The 1Hz
`_publish_status` timer's normally-clean ~1.000s interval was split
into two uneven pieces at four points: elapsed ~16.4s/17.1s (0.38s +
0.72s), ~48.6s/49.1s (0.52s + 0.49s), ~80.9s/81.1s (0.76s + 0.27s), and
~113.1s/113.2s (0.98s + 0.11s). Gaps between these events: 32.2s,
32.0s, 32.3s -- the same period as every prior audit.

**Unlike the prior targeted diagnostic, this run found NO gap above
0.1s in the 0.02s `_publish_latest_state` timer, and NO gap above 0.15s
in TCP `state` payload reception** (verified down to a 0.03s threshold
for both -- the only >0.03s pattern found in `tcp_state_received` was
the normal ~0.10-0.11s inter-arrival interval itself, i.e. the Pi
server's real 10Hz `state_rate_hz`, not an anomaly). The underlying
sensor-topic freeze that dominated the prior targeted run's four events
(~0.9-1.0s gaps across every one of `/odom`/`/tof`/six IR topics) did
**not** clearly reproduce at that same severity in this 130s window --
only the much smaller 1Hz-timer jitter did.

**Process CPU-time deltas during the four perturbed status-timer
intervals** (e.g. elapsed 16.381s: 0.38s wall / 0.018s CPU; elapsed
17.104s: 0.72s wall / 0.038s CPU) show a CPU-consumption **rate**
roughly consistent with the process's normal ~0.05 CPU-s-per-wall-s
baseline, not a rate collapsing toward zero. This argues against a
hard, whole-process OS scheduling freeze for *this* occurrence (CPU
time kept accruing roughly in proportion to wall time throughout), and
is more consistent with jitter in when the single-threaded rclpy
executor services the 1Hz timer relative to the much more frequent
0.02s timer -- though this does not, by itself, explain why that
jitter recurs on a strict ~32-33s clock rather than at random.
Overall process CPU utilization across the whole 130s run was low
(7.24s CPU / 134.6s wall, ~5.4%), consistent with a normally
I/O-bound, mostly-idle bridge process.

## Direct evidence vs. unresolved

**Direct evidence (this run):** GC is not the cause (ruled out by
duration). The ~32-33s period is real and reproduced a fourth time.
This run's manifestation was materially milder than the prior run's,
and its CPU-time signature during the perturbed intervals does not
show the process going fully idle/frozen.

**Unresolved:** why the same ~32-33s external trigger sometimes
produces a ~0.9-1.0s freeze across every physical-input topic (prior
run) and sometimes only a ~30-100ms timer-ordering wobble (this run).
The external trigger itself is still not identified -- OS/WSL2
scheduling contention and Wi-Fi/network-stack-level periodic activity
remain the leading, un-instrumented candidates.

## Explicitly not done (per instruction)

No fix implemented. No threshold changed. No further live test started
automatically. Ground motion remains blocked pending resolution or an
explicit, separate risk acceptance decision.

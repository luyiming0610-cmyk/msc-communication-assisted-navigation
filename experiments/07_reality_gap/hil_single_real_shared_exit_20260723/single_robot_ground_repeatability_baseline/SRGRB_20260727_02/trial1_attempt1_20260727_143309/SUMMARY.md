# SRGRB_20260727_02 -- Trial 1, Attempt 1 -- RUN_ID 20260727_143309

**Classification: `INVALID`.** Reason: `POSE_READINESS_FLUSH_FRESHNESS_INCOMPATIBLE`.

**This is not a motion failure and not a safety failure.** No arm
command was ever issued, the robot was never placed on the ground, no
pulse occurred, and no motion of any kind was observed at any point.
The robot remained on its stand, wheels clear of the ground, for the
entire attempt. `GROUND_DIAGNOSTIC_LIVE_ZERO_STATE_CHECK_PASS` was
achieved cleanly (`validity_flags=7`, bridge connected, WSL CSV
growing zero-only, Pi verifier verdict `PASS` with zero nonzero
commands, guard sole `/cmd_vel` publisher with zero output). This is a
procedural/instrumentation `INVALID` outcome per the approved retry
policy, eligible for a retry attempt.

## What happened

1. Bring-up proceeded through the Pi driver, audited Pi server, WSL
   bridge, `state_publisher`, the command-evidence recorder (started
   correctly via `--output-root`, confirmed alive with its manifest
   and CSV agreeing exactly with the frozen root), and the guard
   (started `armed=False`). Sole publisher and zero output on
   `/cmd_vel` confirmed.
2. The Pi audit verifier and the combined `LIVE_ZERO_STATE_CHECK`
   both passed cleanly.
3. `hil_repeatability_pose_readiness.py` was run, as required before
   ground placement, and reported:
   ```
   REPEATABILITY_POSE_READINESS_BLOCKED
   REASONS=['LATEST_POSE_SAMPLE_STALE(age_s=1.078,max_s=1.0)']
   ```
4. The attempt was stopped before ground placement rather than
   re-running the identical check to obtain a timing-dependent
   `PASS`.

## Root cause

The recorder was started with `--flush-interval-s 1.0` (the same
value used by non-repeatability ground diagnostics), which exactly
equals this gate's own `max_pose_sample_staleness_s` (1.0 s).
`hil_command_evidence_recorder.py`'s `CommandEvidenceCsvWriter`
flushes its CSV to disk only when a write arrives at/after the flush
interval has elapsed (not on a background timer) -- so between two
consecutive flushes, the freshest sample actually readable on disk
stays fixed at the last flush's own row, and its age (as observed by a
check running near the end of that window) approaches the flush
interval itself even under perfectly healthy conditions. With zero
margin between the flush interval and the staleness threshold, the
ordinary, unavoidable overhead of actually running the check (this
gate's own CSV-parse time, Python interpreter/import startup, ordinary
clock/scheduling jitter -- none of them a sensor, bridge, or
connectivity fault) was enough to push the observed age (1.078s) over
the threshold. This is reproduced deterministically (not a real
sensor/bridge fault) in
`test_hil_repeatability_pose_readiness.py`'s
`FlushFreshnessIncompatibilityTest`, matching the observed magnitude
almost exactly.

## Shutdown

Exact-PID `kill -INT` only, never `pkill`. Reverse order, recorder
last via its manifest:

| Process | PID(s) | Confirmed stopped |
|---|---|---|
| Guard | 3039 | Yes -- `pgrep` empty |
| state_publisher | 2918 (wrapper) / 2919 (node) | Yes -- `pgrep` empty |
| WSL bridge | 2887 | Yes -- `pgrep` empty |
| Audited Pi server | 1340 | Yes -- `pgrep` empty |
| Pi driver | 1237 (wrapper) / 1238 (node) | Yes -- `pgrep` empty |
| WSL command-evidence recorder | 2953 | Yes -- `stop` via manifest, `status=STOPPED`, final CSV SHA-256-verified (13101 rows) |

Confirmed afterward: no related process remains on either machine;
`/cmd_vel` absent from `ros2 topic list`.

## Fix applied offline, after full shutdown

Repeatability trials must start the recorder with
`--flush-interval-s 0.2`
(`hil_repeatability_pose_readiness.RECOMMENDED_REPEATABILITY_FLUSH_INTERVAL_S`),
never `1.0`. This gate's own `max_pose_sample_staleness_s` safety rule
(1.0 s) is unchanged -- the fix is a parameter choice for how the
recorder is invoked, not a relaxation of the staleness rule, and not a
change to `state_publisher.py`, the protocol, controller, bridge, or
guard. See `SINGLE_ROBOT_GROUND_REPEATABILITY_BASELINE_SPEC.md`
section 11.A and section 6 for the full derivation, and
`test_hil_repeatability_pose_readiness.py`'s
`FlushFreshnessIncompatibilityTest`/`ProductionRepeatabilitySettingsTest`
for the deterministic reproduction and validation of the fix. No code
was edited while the physical stack was live; the fix was made only
after the shutdown recorded above was fully confirmed.

## Evidence

Raw evidence (WSL CSV, Pi JSONL, Pi verdict JSON, recorder manifest/log)
remains local to the WSL/Pi filesystems and gitignored, exactly as for
every prior run -- not committed. No `post_run_verification.json`
exists for this attempt (the attempt was stopped before ground
placement/pulse, per the classification above, not because evidence is
missing or broken -- `LIVE_ZERO_STATE_CHECK` evidence was healthy
throughout).

## Effect on the batch

Per the approved retry policy, this `INVALID` (procedural/
instrumentation) classification permits a retry: Trial 1 Attempt 2 is
prepared with a new `RUN_ID` and fresh evidence paths, using the fixed
flush-interval setting, only after this attempt's records were
committed and the session state was re-initialized -- see this
batch's `attempts_manifest.json`.

# First ground diagnostic, run 20260724_172233 -- epuck5809 (2026-07-24)

**Classification: `INCOMPLETE_DIAGNOSTIC` / `EXCLUDED`.**
**Reason: `COMBINED_GATE_ARGV_OFF_BY_ONE`.**

Never silently re-attempted; this record is permanent regardless of
any later, separate run. Run ID `20260724_172233` and its evidence
paths below must never be reused.

## What happened

The full pre-pulse supervised sequence (session confirmations,
pre-stack check, Pi driver, audited Pi server, WSL bridge,
state_publisher, WSL command-evidence recorder, Pi-side verifier,
verdict transfer and hash verification) completed and each step's own
verification passed. The run was stopped and excluded at the
live-zero-state combined-gate step, before the guard was ever started
and before any arm/pulse command, because
`run_ground_diagnostic_preflight.sh live-zero-state` crashed:
```
Traceback (most recent call last):
  File "<stdin>", line 6, in <module>
ValueError: not enough values to unpack (expected 20, got 19)
```
Root cause: the verdict-computation heredoc unpacked 20 positional
arguments via a Python slice, `sys.argv[1:20]`, which is end-exclusive
and therefore only yields 19 values. Fixed in commit `0fefe90`
(extracted to `../tools/hil_live_zero_state_verdict.py` with named
`--flags`, plus an end-to-end regression test that invokes it as a
real subprocess -- the missing coverage that let this bug reach a live
run).

## Record

- No pulse was ever issued.
- The guard was never started (`GUARD=NOT_STARTED` at the point of the
  crash).
- The robot never left its stand -- ground placement (runbook step 11)
  is gated on this exact check passing, which it never did.
- All processes that were running (driver, audited server, bridge,
  state_publisher, recorder) were stopped by their exact PID via
  `kill -INT`, never `pkill`, in the order: state_publisher, WSL
  bridge, audited Pi server, Pi driver, WSL recorder last.
- `/cmd_vel` confirmed absent from `ros2 topic list` after shutdown;
  no related process remained on either machine.
- The WSL recorder for this run was started directly (not via the
  manifest-generating wrapper script), so no `manifest.json` exists for
  it -- its exact PID was recorded and used directly instead.

## Process PIDs (all confirmed stopped)

| Process | PID(s) |
|---|---|
| Pi driver | 679 (wrapper) / 680 (node) |
| Audited Pi server | 800 |
| WSL bridge | 22587 |
| state_publisher | 22711 (wrapper) / 22712 (node) |
| WSL command-evidence recorder | 22776 |
| Guard | never started |

## Evidence (raw files preserved on disk only, never committed here)

| File | Path | SHA-256 |
|---|---|---|
| Pi command-audit JSONL | `/home/pi/real_robot_avoidance_v1/command_audit_20260724_172233.jsonl` | not re-hashed after shutdown; growing and zero-only as of the last Pi verifier run (see below) |
| Pi verifier verdict JSON | `/home/pi/real_robot_avoidance_v1/pi_audit_verdict_20260724_172233.json` | `fbb7fdf05b7d10ea20f99a2193e6f0d3e471260489c8e1d375397b0bee70c8c6` |
| WSL command-evidence CSV | `/home/eamon/epuck_comm_bags/first_ground_diagnostic_20260724_172233/command_evidence.csv` | not separately hashed for this record |
| Transferred WSL copy of the Pi verdict | `/home/eamon/epuck_comm_bags/first_ground_diagnostic_20260724_172233/pi_audit_verdict_20260724_172233.json` | `fbb7fdf05b7d10ea20f99a2193e6f0d3e471260489c8e1d375397b0bee70c8c6` (verified identical to the Pi-side copy) |

The last Pi verifier run before shutdown reported: `PI_AUDIT_VERDICT=PASS`,
`TOTAL_RECORDS=7954`, `MALFORMED_COUNT=0`, `NONZERO_RECEIVED_COUNT=0`,
`NONZERO_APPLIED_COUNT=0`, `GROWING=True`, `LATEST_ZERO_REASON=COMMANDED_ZERO`
-- consistent with the record above: no nonzero command was ever
observed on either evidence stream at any point in this run.

## Window/evidence table

| Window | Exact PID | Evidence path |
|---|---|---|
| Pi Window 1 -- physical driver | 679 / 680 | (none -- console log only) |
| Pi Window 2 -- audited command server | 800 | Pi JSONL: see above |
| Pi Window 3 -- Pi read-only verification | (not long-running) | Pi verifier verdict JSON: see above |
| WSL Window 1 -- TCP bridge | 22587 | (none -- console log only) |
| WSL Window 2 -- state publisher | 22711 / 22712 | (none -- console log only) |
| WSL Window 3 -- command-evidence recorder control | 22776 | CSV: see above (no manifest.json this run) |
| WSL Window 4 -- command guard | never started | (none) |
| WSL Window 5 -- read-only HIL verification | (not long-running) | crash traceback above |
| WSL Window 6 -- supervised motion command | never used | (none -- gate never passed) |

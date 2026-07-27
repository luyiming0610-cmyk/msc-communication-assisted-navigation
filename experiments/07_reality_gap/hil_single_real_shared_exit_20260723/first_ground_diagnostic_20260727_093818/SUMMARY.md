# First ground diagnostic, run 20260727_093818 -- epuck5809 (2026-07-27)

**Classification: `INCOMPLETE_DIAGNOSTIC` / `EXCLUDED`.**
**Reason: `UNPLANNED_PI_DISCONNECTION_AFTER_LIVE_GATE`.**

Never silently re-attempted; this record is permanent regardless of
any later, separate run. Run ID `20260727_093818` and its evidence
paths below must never be reused.

## What happened

The full pre-pulse supervised sequence completed and each step's own
verification passed, including the combined gate:
`GROUND_DIAGNOSTIC_LIVE_ZERO_STATE_CHECK_PASS` was achieved with
`validity_flags=7`, bridge connected, WSL CSV growing zero-only, and a
matching, hash-verified Pi verdict showing zero nonzero commands
across 14827 records. The robot was then moved to the ground (step 11)
per that PASS. Before `APPROVED_FOR_SINGLE_PULSE=YES` was ever
provided, and before any arm or pulse command was issued, the Pi was
accidentally disconnected. The robot was powered off and returned to
its stand. The guard remained DISARMED throughout.

**The earlier `LIVE_ZERO_STATE_CHECK_PASS` is explicitly recorded as
having been valid only up to the moment of disconnection -- it does
not authorize any motion after that point**, and none was attempted.

## Record

- `GROUND_DIAGNOSTIC_LIVE_ZERO_STATE_CHECK_PASS` was achieved before
  the disconnection; it does not apply afterward.
- No `APPROVED_FOR_SINGLE_PULSE=YES` was ever given.
- No arm command was ever issued.
- No pulse was ever issued.
- The guard remained DISARMED for its entire lifetime this run.
- The robot was moved to the ground per the PASS, then returned to its
  stand and powered off after the disconnection -- no motion command
  was involved in either transition.
- The Pi driver (PIDs `782`/`783`) and audited Pi server (PID `904`)
  are recorded as `DISCONNECTION_TERMINATED`, not gracefully stopped --
  the Pi was not contacted during shutdown, per instruction.
- WSL-side processes were stopped by exact PID via `kill -INT`, never
  `pkill`: guard (`1154`), state_publisher (`916`, wrapper `915`),
  WSL bridge (`837`), recorder (`1027`, stopped last).
- A read-only check afterward confirms no related WSL process remains
  and `/cmd_vel` is absent from the current ROS graph.

## Evidence status

| File | Path | Status |
|---|---|---|
| WSL command-evidence CSV | `/home/eamon/epuck_comm_bags/first_ground_diagnostic_20260727_093818/command_evidence.csv` | Present, 15986 lines. Closeout SHA-256: `821ce36b2ebafbe214e8f010802dad66d5d737307ef6cfbfeba0d9de408b71cc`. Final recorded rows show `/epuck_bridge/status` with `bridge_connected=False`, consistent with the Pi disconnection. |
| Transferred WSL copy of the Pi verdict | `/home/eamon/epuck_comm_bags/first_ground_diagnostic_20260727_093818/pi_audit_verdict_20260727_093818.json` | Present, SHA-256 `664b1cb0468312514f0b77638ccc7b64d40e6d0c2c9d8886bc9df88bec2a2db1` -- unchanged, matches the hash verified against the Pi-side copy at transfer time (before the disconnection). |
| Pi command-audit JSONL | `/home/pi/real_robot_avoidance_v1/command_audit_20260727_093818.jsonl` | Not re-checked after the disconnection -- the Pi was not contacted during this closeout, per instruction. Its state at the moment of disconnection is reflected only in the verdict above (`TOTAL_RECORDS=14827`, all zero). |
| Pi verifier verdict JSON (on the Pi) | `/home/pi/real_robot_avoidance_v1/pi_audit_verdict_20260727_093818.json` | Not re-checked after the disconnection, same reason. |

No controller, guard, bridge, parameter, threshold, protocol, or
experimental geometry was modified in response to this run.

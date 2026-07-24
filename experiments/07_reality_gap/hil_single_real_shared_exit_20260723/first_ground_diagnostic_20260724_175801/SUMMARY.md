# First ground diagnostic, run 20260724_175801 -- epuck5809 (2026-07-24)

**Classification: `INCOMPLETE_DIAGNOSTIC` / `EXCLUDED`.**
**Reason: `UNPLANNED_ROBOT_POWER_LOSS`.**

Never silently re-attempted; this record is permanent regardless of
any later, separate run. Run ID `20260724_175801` and its evidence
paths below must never be reused.

## What happened

The full pre-pulse supervised sequence (session confirmations,
pre-stack check, Pi driver, audited Pi server, WSL bridge,
state_publisher, WSL command-evidence recorder, guard start DISARMED)
completed and each step's own verification passed. The
`live-zero-state` combined gate was in progress (validity_flags,
bridge, guard identity, and `/cmd_vel` zero output had all been
reconfirmed) when the e-puck lost power unexpectedly. The Pi went down
mid-session; the operator closed all Pi and WSL terminal
windows/processes manually rather than continuing the exact-PID
shutdown sequence against a session whose Pi side had already gone
dark.

## Record

- No arm command was ever issued.
- No motion pulse was ever issued.
- The robot remained on its stand throughout -- ground placement
  (runbook step 11) is gated on the live-zero-state gate passing,
  which it never did.
- The Pi's loss of power was unexpected, not a planned shutdown.
- All user-visible Pi and WSL processes/windows were closed manually
  by the operator after the power loss. **This was not an exact-PID
  `kill -INT` shutdown** -- no exact-PID confirmation exists for the
  Pi driver (PIDs 1006/1007) or the audited Pi server (PID 1064) for
  this run; their termination is attributed to the power loss itself,
  not to a graceful, confirmed stop. The WSL-side guard (PID 26340),
  state_publisher (PIDs 25952/25953), and bridge (PID 25875) were
  likewise closed manually, not via a verified `kill -INT` sequence
  recorded here.
- A read-only, offline check afterward (WSL side only, no Pi contact)
  confirms no related WSL process remains and `/cmd_vel` is absent
  from the current ROS graph.

## Evidence status -- explicitly marked, not assumed complete

| File | Path | Status |
|---|---|---|
| WSL command-evidence CSV | `/home/eamon/epuck_comm_bags/first_ground_diagnostic_20260724_175801/command_evidence.csv` | Present, 11743 lines. Closeout SHA-256: `7dc31a6450a4970bff71284b81bfbd2d58fc63c84188faa24a2321a1c3df5903`. Its own recorder process was not stopped via its normal `kill -INT` shutdown path, so this hash reflects whatever was flushed to disk at the moment the process ended -- not a confirmed-clean close. The final recorded rows show `validity_flags=0` on `/epuck1/state` immediately before the file stops growing, consistent with the Pi losing power. |
| Transferred WSL copy of the Pi verdict | `/home/eamon/epuck_comm_bags/first_ground_diagnostic_20260724_175801/pi_audit_verdict_20260724_175801.json` | Present, SHA-256 `856a9ab0deec8bc8e1d8d20926ea5f3866a2f960a7130cf8a458ccb627fdc85e` -- unchanged from the last verified transfer before the power loss (matches the Pi-side hash confirmed at that time). This verdict reflects the Pi's state *before* the power loss, not after. |
| Pi command-audit JSONL | `/home/pi/real_robot_avoidance_v1/command_audit_20260724_175801.jsonl` | **Not re-checked.** The Pi was not contacted during this closeout, per instruction. Its final state (whether it closed cleanly, was truncated, or is otherwise affected by the power loss) is unknown and must not be assumed either way. |
| Pi verifier verdict JSON | `/home/pi/real_robot_avoidance_v1/pi_audit_verdict_20260724_175801.json` | **Not re-checked**, same reason. |

No controller, guard, bridge, parameter, threshold, protocol, or
experimental geometry was modified in response to this run.

## Recommended pre-run addition for the next attempt

- Battery sufficiently charged before bring-up.
- Power cable/battery seated securely.
- Pi remains reachable for a short stationary observation period
  before proceeding past bring-up.
- Robot stays on the stand during bring-up (already required).

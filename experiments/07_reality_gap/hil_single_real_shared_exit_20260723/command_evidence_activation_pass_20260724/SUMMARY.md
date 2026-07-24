# Suspended zero-motion command-evidence activation: PASS (2026-07-24)

**Classification: `SUSPENDED_ZERO_MOTION_COMMAND_EVIDENCE_PASS`.**

## Purpose

Suspended zero-motion validation of the full Pi-side + WSL-side
command-evidence chain designed and hardened following the two
2026-07-23 `UNEXPECTED_PHYSICAL_MOTION` incidents (see
`../safety_incident_unexpected_motion_20260723/SUMMARY.md` and
`../safety_incident_unexpected_motion_2_20260723/SUMMARY.md`) and the
2026-07-24 recorder fixes (commits `8ae1be955efc7bb44a9d99d0a796d3f9c36238b4`,
`a14fdf2ba983a8db994c82eeebd2a71ca89af16d`). **This is explicitly NOT a
ground navigation trial and NOT a formal performance trial.** The
robot remained on its stand, wheels suspended, for the entire session.
No controller, virtual peer, goal_navigator, or Webots was ever
started; no pytest/colcon test was run against the live stack.

This is the first attempt (after one earlier `INCOMPLETE_DIAGNOSTIC`/
`EXCLUDED` attempt, `../command_evidence_activation_20260724/SUMMARY.md`,
which stopped short specifically because the WSL CSV could not yet be
observed growing mid-session) that reached and completed every
intended verification step.

## Sequence executed

Pi driver -> audited Pi server (`command_audit_enabled:=true`, new
timestamped `command_audit_path`) -> WSL bridge -> state_publisher ->
WSL command-evidence recorder (confirmed active and topic-verified
*before* the guard) -> `hil_cmd_vel_guard.py`, started **DISARMED**,
with temporary diagnostic-only limits `max_linear_speed_mps=0.02`,
`max_angular_speed_rps=0.0` (never adopted as ground parameters).

## Verified during the session

- **`validity_flags=7`** confirmed on `/epuck1/state`.
- **Guard was the sole `/cmd_vel` publisher, `armed=False`, for the
  entire session** -- confirmed both from the guard's own log
  (`guarded_cmd_vel_publisher_count=1 guarded_publisher_is_self=True`)
  and independently via `ros2 topic info -v /cmd_vel` (sole publisher
  node: `hil_cmd_vel_guard`; the only two subscribers were
  `hil_command_evidence_recorder` and `wsl_epuck_tcp_bridge_sensors`,
  both expected/benign). Actual `/cmd_vel` output sampled directly:
  `linear.x=0.0, angular.z=0.0` on all fields.
- **No unexpected wheel motion or sound occurred at any point.**

## Command-evidence chain results

- **WSL CSV**: `command_evidence.csv`, **22,812 rows**, SHA-256
  `01648a13f172f8217664a195e8fa28100068d2f44a3b883c2cb6f4144a409eb5`.
  Parsed with Python's `csv` module (not a loose regex): **0 nonzero**
  `linear_x`/`angular_z` values across the entire session. Confirmed
  existing and growing (via two separate, later re-checks) while the
  recorder was still running -- proving the 2026-07-24 incremental-
  write/periodic-flush fix works under a real live session, not only
  synthetic tests.
- **Pi JSONL**: `command_audit_20260724_113236.jsonl`, **34,458
  records**, SHA-256
  `4a8212f2c77dbb3c26f8cf738b118dc07c6ec3fffb1dd7d6c2686bef61e69a3e`.
  Parsed with Python's `json` module (not a loose regex): **0
  nonzero** received or applied commands anywhere in the file.
  `zero_reason` counts:
  - `DISCONNECTED` = 3959 (before the bridge connected, and again
    after it disconnected during shutdown)
  - `NEVER_RECEIVED` = 1 (the single instant right after connection,
    before the bridge's own keep-alive stream began)
  - `COMMANDED_ZERO` = 21168 (the bulk of the session -- the bridge's
    own zero-value keep-alive stream, connected and fresh, genuinely
    zero)

Both raw files (`command_evidence.csv`, `command_audit_20260724_113236.jsonl`)
are preserved on disk in `raw_logs/` alongside this document, SHA-256
verified above, but **not committed to git** (gitignored, per this
project's established raw-evidence convention -- large/regenerable
capture files live on disk, never in the repo).

## Shutdown (exact-PID SIGINT only, in required order)

1. Guard (PID 727) -- re-confirmed `armed=False` and `/cmd_vel=0.0`
   immediately before stopping.
2. state_publisher actual node (PID 530); wrapper (PID 509) confirmed
   exited too.
3. WSL bridge (PID 467).
4. Audited Pi server (PID 803) -- JSONL confirmed closed with final
   records showing `DISCONNECTED`/zero.
5. Pi driver actual process (PID 666); wrapper (PID 665) confirmed
   exited too.
6. WSL command-evidence recorder (PID 660), stopped **last**, via its
   manifest -- `status=STOPPED`, CSV flushed and SHA-256-verified.

## Post-shutdown verification

- No physical/HIL process remained on either side.
- `/cmd_vel`: `Unknown topic '/cmd_vel'` -- did not exist at all.
- Both evidence files' SHA-256 independently recomputed after copy and
  confirmed to exactly match the values recorded above.

## Explicitly not done / not claimed

- **This is not a ground navigation trial.** No field geometry was
  used or needed; the robot never left its stand.
- **This is not a formal performance trial.** No task-completion
  metric, no encounter, no communication condition was exercised.
  The guard was never armed.
- Root cause of the two 2026-07-23 `UNEXPECTED_PHYSICAL_MOTION`
  incidents remains unsolved and is not addressed or retroactively
  explained by this pass -- this activation only proves the
  command-evidence chain itself now works for any *future* session.

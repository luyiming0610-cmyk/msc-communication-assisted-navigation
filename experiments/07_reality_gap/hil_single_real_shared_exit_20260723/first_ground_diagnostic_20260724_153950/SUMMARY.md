# First ground diagnostic, run 20260724_153950 -- epuck5809 (2026-07-24)

**Classification: `INCOMPLETE_DIAGNOSTIC` / `EXCLUDED`.** Not a `PASS`,
not a `FAIL` -- the live evidence gate blocked before the pulse step
was ever reached, for a tooling reason, not a safety violation. Never
silently re-attempted; this record is permanent regardless of any
later, separate run.

## What happened

The full pre-pulse supervised sequence (session confirmations,
pre-stack check, Pi driver, audited Pi server, WSL bridge,
state_publisher, WSL command-evidence recorder, guard start DISARMED)
completed and each step's own verification passed. The run was stopped
and excluded at the live-zero-state gate, before arming or any pulse
command, because that check (running from WSL) tried to read the Pi's
local command-audit JSONL by a plain filesystem path -- the Pi and WSL
machine share no filesystem, only a network connection, so the file
was never actually reachable from there. The check failed closed
correctly (a missing/unreachable file was never treated as proven
zero), but could not have passed with that design regardless of the
robot's actual state. See `../tools/hil_ground_diagnostic_phases.py`'s
module docstring and commit `661998b` for the structural fix made
after this run.

## Record

- No pulse was ever issued.
- The robot never moved on the ground.
- The guard remained disarmed for its entire lifetime this run.
- The WSL command-evidence recorder expired at its own `--duration-s
  600` timeout (raised to 3600 for future runs, see commit
  documenting the runbook update below) -- it was not killed early and
  did not fail; it simply reached its configured duration before the
  live gate was resolved.
- The live gate blocked specifically because WSL could not read a
  Pi-local path (`PI_JSONL_NOT_GROWING`, `PI_EVIDENCE_CONTAINS_NONZERO_COMMAND`
  from the old, since-replaced single-function check) -- not because
  any actual nonzero command was observed on either evidence stream.
- All processes (guard, state_publisher, WSL bridge, audited Pi
  server, Pi driver) were stopped by their exact PID via `kill -INT`,
  never `pkill`; the recorder had already exited on its own before
  shutdown began.

## Evidence (raw files preserved on disk only, never committed here)

| File | Path | SHA-256 |
|---|---|---|
| WSL command-evidence CSV | `/home/eamon/epuck_comm_bags/first_ground_diagnostic_20260724_153950/command_evidence.csv` | `b51cd085a23416fa2caabc4acd96eccd0191ffb4ca5e93713e68ed603865989b` |
| Pi command-audit JSONL | `/home/pi/real_robot_avoidance_v1/command_audit_20260724_153950.jsonl` | `65d01dc6ae7d03eba9c0f3f5687893ead26a9b65255af55b0d0c8b3e126e3f01` |

The Pi JSONL SHA-256 above was computed directly on the Pi
(`sha256sum` in the Pi terminal); it has not yet been copied off the
Pi as of this record. If it still needs copying for later analysis,
use one explicit, interactive `scp` (never non-interactive/automated):
```bash
scp pi@<PI_IP>:/home/pi/real_robot_avoidance_v1/command_audit_20260724_153950.jsonl /home/eamon/epuck_comm_bags/first_ground_diagnostic_20260724_153950/command_audit_20260724_153950.jsonl
```
Verify the copy's SHA-256 matches the value above before relying on it
for any analysis.

## Window/evidence table

The fixed window map (see `../first_ground_diagnostic/GROUND_DIAGNOSTIC_RUNBOOK.md`)
did not exist yet during this run; PIDs were tracked ad hoc in the
conversation transcript rather than in this fixed format. Recorded
here for continuity with future runs, which will fill in the full
table at the time of the attempt:

| Window | Exact PID | Evidence path |
|---|---|---|
| Pi driver | 651 (wrapper) / 652 (node) | (none -- console log only) |
| Audited command server | 771 | Pi JSONL: see above |
| WSL TCP bridge | 10691 | (none -- console log only) |
| WSL state_publisher | 10770 (wrapper) / 10771 (node) | (none -- console log only) |
| WSL command-evidence recorder | (exited on its own, `--duration-s 600`) | CSV: see above |
| WSL command guard | 11042 | (none -- console log only) |

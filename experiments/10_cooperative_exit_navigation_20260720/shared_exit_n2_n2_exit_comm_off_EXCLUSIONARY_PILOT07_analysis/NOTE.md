# shared_exit_n2_n2_exit_comm_off_EXCLUSIONARY_PILOT07 -- FAILED, orchestrator watchdog bug, NOT an exit-geometry or parking-conflict finding (preserved, not deleted)

**EXCLUSIONARY_DIAGNOSTIC. Not counted toward any formal or pilot statistic.**

## CLASSIFICATION: infrastructure bug, not EXCLUDED_EXIT_GEOMETRY_DIAGNOSTIC

First pilot on the analytically-redesigned exit geometry (revision 3:
elevated markers, re-derived parking zones, `max_runtime_s=102.0`).
`data_validity=INVALID`, `stop_reason=MAX_RUNTIME`,
`controller_crashed=true`. Robot A's raw `/epuck1/cmd_vel` was still
nonzero (`linear.x=0.01`) at the moment of the check, and neither
controller ever reached `COMPLETE`.

## Root cause (confirmed by reading run_shared_exit_n2_trial.sh, not guessed)

The orchestrator's own controller-stage wait loop
(`deadline=$((SECONDS + 90))`) was still hardcoded to 90 seconds from
the revision-2/2b era, when `max_runtime_s` was 68.0s. When
`max_runtime_s` was formula-recomputed to 102.0s in the revision-3
exit-geometry redesign, this orchestrator-level watchdog was not
updated in step, so the orchestrator killed the controller stage at
`t=90s` -- BEFORE either robot's own `cooperative_avoider` could reach
its own `max_runtime_s=102.0s` fallback, let alone genuinely complete.
This is a pure coordination/infrastructure bug (a stale hardcoded value
out of sync with a parameter it depends on), unrelated to exit
geometry, marker placement, or parking-zone conflict -- the trial
never ran long enough to exercise any of that redesigned geometry at
all.

## Fix

`run_shared_exit_n2_trial.sh`'s watchdog deadline is now derived from
`max_runtime_s + startup_hold_s + 15.0s settle buffer` instead of a
hardcoded constant, so it can never again silently fall out of sync
with the controller's own `max_runtime_s` parameter.

## Why this does not count against the "no PILOT08" rule

The instruction was: if PILOT07 fails again on exit geometry or
parking conflict, stop immediately. This failure was neither -- it is
an orchestration-level watchdog bug, diagnosed with certainty from the
script's own source, unrelated to the redesigned scene. The corrected
retry (same exit geometry, same `max_runtime_s=102.0` formula, only the
orchestrator's own watchdog fixed) is run under a new pilot number per
this project's evidence-preservation convention (never reuse or
overwrite a trial directory name), not because the geometry itself
needed another iteration.

## Disposition

- Native WSL bag + diag_logs preserved at
  `/home/eamon/epuck_comm_bags/shared_exit_n2_n2_exit_comm_off_EXCLUSIONARY_PILOT07`
  (+ `_diag_logs`) and a SHA-256-verified Windows copy under this
  directory's sibling `bags/` path (gitignored).
- Process cleanup confirmed clean after this pilot.

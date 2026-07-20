# cooperative_exit_n2_n2_comm_off_EXCLUSIONARY_PILOT01 -- FAILED, real orchestrator bug (preserved, not deleted)

**EXCLUSIONARY_DIAGNOSTIC. Not counted toward any formal or pilot statistic.**

## What happened

`run_n2_controllers.py`'s `main()` read the `N2_COMM_MODE` environment
variable and validated it against `("COMM_ON", "COMM_OFF")`, but
`run_cooperative_exit_n2_trial.sh` (the orchestrator) sets the SAME
variable to its own condition-name values, `N2_COMM_OFF`/`N2_COMM_ON`
(with the `N2_` prefix) -- a naming mismatch between the two new scripts
written in the same commit. The controller launch script exited
immediately with:

```
N2_COMM_MODE must be COMM_ON or COMM_OFF, got 'N2_COMM_OFF'
```

Confirmed via direct inspection of `controller.log` (the only line it
contains). No robot ever moved; Webots/state_publisher/bag recording all
started and shut down cleanly (process cleanup confirmed CLEAN
afterward), so this is a clean, harmless setup-time failure, not a
runtime safety event.

Also caught in the same pass: `trial_verdict.json`'s `data_validity`
field incorrectly stayed `"VALID"` despite `controller_crashed: true` --
a second bug (the orchestrator recorded the crash reason into
`INVALID_REASON` but never actually set `DATA_VALIDITY="INVALID"` in
that branch). Both bugs are fixed in the orchestrator/controller-launch
scripts (see the git history for this directory's sibling commit) before
any retry.

## STEM naming note

This attempt's directory stem, `cooperative_exit_n2_n2_comm_off_...`, has
a cosmetic double-`n2` redundancy (`STEM="cooperative_exit_n2_${COMM_MODE,,}_..."`
where `COMM_MODE` already starts with `N2_`). Fixed in the orchestrator
for all subsequent attempts (`cooperative_exit_n2_comm_off_...`). This
attempt's name is NOT retroactively renamed -- it is preserved exactly as
it was recorded, per the project's evidence-preservation rule.

## Disposition

- Native WSL bag + diag_logs preserved at
  `/home/eamon/epuck_comm_bags/cooperative_exit_n2_n2_comm_off_EXCLUSIONARY_PILOT01`
  (+ `_diag_logs`) and copied here (gitignored Windows copy) for
  completeness.
- Not rerun under this same name -- the corrected retry uses a new label,
  `EXCLUSIONARY_PILOT02`, per the project's no-directory-overwrite rule.
- `DATA_VALIDITY` for this attempt: `INVALID` (corrected manually here;
  the buggy `trial_verdict.json` inside `diag_logs/` is preserved
  unmodified as the original raw record).

# objective5_impairment_matrix_v1_condition_D_trial03_attempt01

**FORMAL_SIM**, Objective 5 impairment matrix, Condition D, Trial 03, attempt 01.
Automated sequential batch run (D Trials 02-05), authorized by the user, single wsl.exe invocation per trial (interop register+verify, then eamon-session orchestrator invocation). manual_observation.status=NOT_OBSERVED throughout this batch per explicit user instruction.

## Files (git-tracked)

`frozen_params.json`, `matrix_analysis.json`, `trial_verdict.json` (written by the orchestrator during the run);
`final_verdict.json`, `runtime_manifest.json`, `summary.md`, `README.md`, `SHA256SUMS` (written after the run, this pass).

## Raw data (NOT committed)

Preserved at native WSL `/home/eamon/epuck_comm_bags/objective5_impairment_matrix_v1_condition_D_trial03_attempt01` (+ `_diag_logs`) and a
SHA-256-verified Windows copy at
`experiments/05_objective5_impairment_matrix/bags/objective5_impairment_matrix_v1_condition_D_trial03_attempt01/` (gitignored).

## Result (five-axis, authoritative -- see `three_axis_verdict.json`)

`DATA_ARTIFACT_INTEGRITY=VALID`, `MANIPULATION_VALIDITY=VALID`,
`TASK_OUTCOME=SUCCESS`, `FORMAL_MEASUREMENT_VALIDITY=VALID`,
`FORMAL_BATCH_INCLUSION=INCLUDED`.

**RAZOR-THIN MARGIN**: only **+0.170mm** -- a THRESHOLD PASS, not robust
safety. Genuine, unadjusted data.

`DATA_VALIDITY=VALID`, `TASK_OUTCOME=SUCCESS`, `FAIL` (obsolete single-axis
strict-gate label, superseded -- see `final_verdict.json`'s
`trial_verdict_authoritative_note`).
`min_interrobot_distance_m=0.14016992055075728` vs `safety_radius_m=0.14`.

## Batch status

`TRIAL_PASS_PENDING_BATCH_COMPLETION`. Not the whole Condition D batch.

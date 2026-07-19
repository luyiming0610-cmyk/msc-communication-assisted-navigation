# objective5_impairment_matrix_v1_condition_D_trial01_attempt01

**FORMAL_SIM**, Objective 5 impairment matrix, Condition D, Trial 01, attempt 01.
Launched individually via the permanent run_objective5_matrix_from_windows.sh wrapper for direct user Webots observation (not part of an automated batch); Trial 01 of the Condition D formal n=5 batch, following the same manually-observed-first pattern as Condition A/B/C Trial 01.

## Files (git-tracked)

`frozen_params.json`, `matrix_analysis.json`, `trial_verdict.json` (written by the orchestrator during the run);
`final_verdict.json`, `runtime_manifest.json`, `summary.md`, `README.md`, `SHA256SUMS` (written after the run, this pass).

## Raw data (NOT committed)

Preserved at native WSL `/home/eamon/epuck_comm_bags/objective5_impairment_matrix_v1_condition_D_trial01_attempt01` (+ `_diag_logs`) and a
SHA-256-verified Windows copy at
`experiments/05_objective5_impairment_matrix/bags/objective5_impairment_matrix_v1_condition_D_trial01_attempt01/` (gitignored).

## Result

`DATA_VALIDITY=VALID`, `TASK_OUTCOME=SUCCESS`, automated strict-gate
label `FAIL` (see clarification in `summary.md`/`final_verdict.json`
-- the strict gate reuses A/B/C's zero-reordering criteria, but
nonzero `out_of_order_count` is Condition D's intended manipulation
check succeeding, not a defect).
`min_interrobot_distance_m=0.1444723957613864` vs `safety_radius_m=0.14`
(positive margin).

## Batch status

`TRIAL_PASS_PENDING_BATCH_COMPLETION`. Not the whole Condition D batch.

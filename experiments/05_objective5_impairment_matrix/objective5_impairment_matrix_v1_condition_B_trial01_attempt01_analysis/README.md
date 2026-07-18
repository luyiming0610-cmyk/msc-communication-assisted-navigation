# objective5_impairment_matrix_v1_condition_B_trial01_attempt01

**FORMAL_SIM**, Objective 5 impairment matrix, Condition B, Trial 01, attempt 01.
Trial 01 of a formal n=5 batch. Launched manually by the user via the
permanent Windows->WSL wrapper for individual real-time observation --
**not** part of an automated batch. *(Correction: this line originally
carried the Condition-A-batch template's default automated-batch text;
see `final_verdict.json`'s `revision_note`.)* `manual_observation.status
=PENDING` in `final_verdict.json`; Trials 02-05 will not auto-start
until the observation is received and confirmed.

See `objective5_condition_B_trial01_trigger_mechanism_audit.{json,md}`
for the offline trigger-mechanism audit: `PURE_COMMUNICATION_CPA_
AVOIDANCE`, `LOCAL_*` all 0.

## Files (git-tracked)

`frozen_params.json`, `matrix_analysis.json`, `trial_verdict.json` (written by the orchestrator during the run);
`final_verdict.json`, `runtime_manifest.json`, `summary.md`, `README.md`, `SHA256SUMS` (written after the run, this pass).

## Raw data (NOT committed)

Preserved at native WSL `/home/eamon/epuck_comm_bags/objective5_impairment_matrix_v1_condition_B_trial01_attempt01` (+ `_diag_logs`) and a
SHA-256-verified Windows copy at
`experiments/05_objective5_impairment_matrix/bags/objective5_impairment_matrix_v1_condition_B_trial01_attempt01/` (gitignored).

## Result

`DATA_VALIDITY=VALID`, `TASK_OUTCOME=SUCCESS`, `PASS`.
`min_interrobot_distance_m=0.14777153762172363` vs `safety_radius_m=0.14`.

## Batch status

`TRIAL_PASS_PENDING_BATCH_COMPLETION`. Not the whole Condition B batch.

# objective5_impairment_matrix_v1_condition_D_trial06_attempt01

**FORMAL_SIM**, Objective 5 impairment matrix, Condition D, Trial 06, attempt 01.
Automated run authorized by the user as the sole preregistered replacement for excluded D04 (rosbag-only single-message capture gap). manual_observation.status=NOT_OBSERVED per explicit user instruction. Uses trial_index=6, seeds resolved from the extended objective5_impairment_matrix_conditions.csv (4006/14006), not hand-typed.

## Files (git-tracked)

`frozen_params.json`, `matrix_analysis.json`, `trial_verdict.json` (written by the orchestrator during the run);
`final_verdict.json`, `runtime_manifest.json`, `summary.md`, `README.md`, `SHA256SUMS` (written after the run, this pass).

## Raw data (NOT committed)

Preserved at native WSL `/home/eamon/epuck_comm_bags/objective5_impairment_matrix_v1_condition_D_trial06_attempt01` (+ `_diag_logs`) and a
SHA-256-verified Windows copy at
`experiments/05_objective5_impairment_matrix/bags/objective5_impairment_matrix_v1_condition_D_trial06_attempt01/` (gitignored).

## Result (five-axis, authoritative -- see `three_axis_verdict.json`)

`DATA_ARTIFACT_INTEGRITY=VALID`, `MANIPULATION_VALIDITY=VALID`,
`TASK_OUTCOME=SUCCESS`, `FORMAL_MEASUREMENT_VALIDITY=VALID`,
`FORMAL_BATCH_INCLUSION=INCLUDED`. Sole preregistered replacement for
excluded D04 (see `runtime_manifest.json`'s `replacement_for` field).

`DATA_VALIDITY=VALID`, `TASK_OUTCOME=SUCCESS`, `FAIL` (obsolete single-axis
strict-gate label, superseded -- see `final_verdict.json`'s
`trial_verdict_authoritative_note`).
`min_interrobot_distance_m=0.14431858718680218` vs `safety_radius_m=0.14`
(margin +4.32mm).

## Batch status

`TRIAL_INCLUDED_BATCH_COMPLETE`. D06 is the fifth included trial and the
sole preregistered replacement for excluded D04. See
`../objective5_impairment_matrix_v1_condition_D_formal_batch_summary.{json,md}`
for the complete batch result (D01/D02/D03/D05/D06 included; D04 retained
and explicitly excluded as a measurement-chain attempt).

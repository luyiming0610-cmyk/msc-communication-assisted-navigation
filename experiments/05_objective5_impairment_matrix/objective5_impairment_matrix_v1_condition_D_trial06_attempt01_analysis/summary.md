# FORMAL_SIM -- Condition D, Trial 06, attempt 01

`objective5_impairment_matrix_v1_condition_D_trial06_attempt01` -- part of a formal n=5 batch for Condition D.
Automated run authorized by the user as the sole preregistered replacement for excluded D04 (rosbag-only single-message capture gap). manual_observation.status=NOT_OBSERVED per explicit user instruction. Uses trial_index=6, seeds resolved from the extended objective5_impairment_matrix_conditions.csv (4006/14006), not hand-typed.

## Verdict (five-axis, authoritative -- see `three_axis_verdict.json`)

`DATA_ARTIFACT_INTEGRITY=VALID`, `MANIPULATION_VALIDITY=VALID`,
`TASK_OUTCOME=SUCCESS`, `FORMAL_MEASUREMENT_VALIDITY=VALID`,
`FORMAL_BATCH_INCLUSION=INCLUDED`. **Sole preregistered replacement for
excluded D04** -- see `runtime_manifest.json`'s `replacement_for` field.

`DATA_VALIDITY=VALID`, `TASK_OUTCOME=SUCCESS`, `FAIL` (obsolete single-axis
strict-gate label, superseded -- see `final_verdict.json`'s
`trial_verdict_authoritative_note`).

## Run identity

- git commit: `fa4718260421706fac668c88a3d5a706f5a014da`
- orchestrator SHA-256: `20d2ef63a152a7d65632e4fd3414c9cd1cdaa2a449f58daf7eac1bd28110913b`
- network_impairment_relay.py SHA-256: `f5d408bc3379f79fa70628370b4dfb6d537c4d03a1968fe8dc75a691c3e6d5ff`
- network_impairment.py SHA-256: `253e0d960e9b587a3c5e60587ce7ac56c167fd6aba1c98f8b7b940e821210561`
- sequence_counter.py SHA-256: `57bb0699a444df644d75c4e834b5fd13b5f15a6283d7b1d276ec0b65674f1fd3`
- All four match Trial 01 exactly (verified before this trial started).

## Communication metrics -- LEGACY_METHOD_INVALID_UNDER_REORDERING

Values below come from the live `sequence_counter.py`'s adjacent-delta
accounting, WRONG under reordering. **Do not use for D-condition
missing/capture judgment** -- see the reorder-safe table below instead.

| | epuck1&rarr;epuck2 | epuck2&rarr;epuck1 |
|---|---|---|
| sample_count | 429 | 436 |
| missing/duplicate/out_of_order (LEGACY, invalid) | 204/0/96 | 203/0/94 |
| capture_ratio (LEGACY, invalid) | 1.0023364485981308 | 1.0 |
| mean/median/p95/p99/max age (s) | 0.1632634032564088/0.1599999999999966/0.29999999999999716/0.2999999999999997/0.3000000000000007 | 0.15972477063761362/0.1599999999999966/0.29999999999999716/0.3000000000000007/0.3000000000000007 |
| latency_measurement_status | VALID_AT_SIM_CLOCK_RESOLUTION | VALID_AT_SIM_CLOCK_RESOLUTION |
| mean bandwidth (bytes/s) | 712.7352226436997 | 711.832818313749 |

`p99_message_age_s` is a genuine finite value on both directions under
the strict formal-trial schema gate (`schema_problems=[]` both
directions). Latency/throughput are unaffected by the legacy-accounting bug.

## Communication metrics -- REORDER-SAFE (authoritative, see `reordering_delivery_audit.json`)

| | epuck1&rarr;epuck2 | epuck2&rarr;epuck1 |
|---|---|---|
| relay forwarded (unique) | 429 | 436 |
| relay dropped | 0 | 0 |
| **actual_missing_count** | **0** | **0** |
| out_of_order (relay forwarded stream, adjacent) | 96 | 94 |
| **aligned_window_forwarded_to_bag_capture_ratio** | **1.0** | **1.0** |

## Queue drain / rosbag / realtime factor

- `drain_duration_s=0.5302`, `queue_drained=True`.
- `ros2 bag info` succeeded (3204 messages, 7 topics, 52.305706294s duration).
- `preload_realtime_factor=0.955`, `full_load_realtime_factor=0.975`.

## Safety

`min_interrobot_distance_m=0.14431858718680218`, `safety_radius_m=0.14`, `safety_margin_m=0.00431858718680217`.

## Process cleanup / data integrity

- Post-run process check: **CLEAN**.
- Native WSL bag copied to `experiments/05_objective5_impairment_matrix/bags/objective5_impairment_matrix_v1_condition_D_trial06_attempt01/` (gitignored) -- SHA-256 verified identical to native WSL source (bag + diag_logs). See `SHA256SUMS`.

## Batch status

`TRIAL_INCLUDED_BATCH_COMPLETE`. D06 is the sole preregistered replacement
for excluded D04 and is the fifth included trial. The authoritative batch
result is `objective5_impairment_matrix_v1_condition_D_formal_batch_summary.{json,md}`:
Condition D is complete with D01/D02/D03/D05/D06 included and D04 retained
separately as `EXCLUDED_MEASUREMENT_CHAIN_ATTEMPT`.

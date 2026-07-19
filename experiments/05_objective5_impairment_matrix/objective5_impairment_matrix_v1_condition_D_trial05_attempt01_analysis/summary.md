# FORMAL_SIM -- Condition D, Trial 05, attempt 01

`objective5_impairment_matrix_v1_condition_D_trial05_attempt01` -- part of a formal n=5 batch for Condition D.
Automated sequential batch run (D Trials 02-05), authorized by the user, single wsl.exe invocation per trial (interop register+verify, then eamon-session orchestrator invocation). manual_observation.status=NOT_OBSERVED throughout this batch per explicit user instruction.

## Verdict (five-axis, authoritative -- see `three_axis_verdict.json`)

`DATA_ARTIFACT_INTEGRITY=VALID`, `MANIPULATION_VALIDITY=VALID`,
`TASK_OUTCOME=SUCCESS`, `FORMAL_MEASUREMENT_VALIDITY=VALID`,
`FORMAL_BATCH_INCLUSION=INCLUDED`. `min_interrobot_distance_m=0.14919158786608294`,
`safety_margin_m=0.009191587866082929` (**+9.19mm**).

`DATA_VALIDITY=VALID`, `TASK_OUTCOME=SUCCESS`, `FAIL` (obsolete single-axis
strict-gate label, superseded -- see `final_verdict.json`'s
`trial_verdict_authoritative_note`).

## Run identity

- git commit: `685edbc20402ecaaa93a07bc68af7943f399c8cd`
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
| sample_count | 438 | 445 |
| missing/duplicate/out_of_order (LEGACY, invalid) | 219/0/103 | 196/0/94 |
| capture_ratio (LEGACY, invalid) | 1.0 | 1.0022522522522523 |
| mean/median/p95/p99/max age (s) | 0.15287671232191657/0.1599999999999966/0.29999999999999716/0.3000000000000007/0.3000000000000007 | 0.1611235954988751/0.16000000000000014/0.29999999999999716/0.3000000000000007/0.3000000000000007 |
| latency_measurement_status | VALID_AT_SIM_CLOCK_RESOLUTION | VALID_AT_SIM_CLOCK_RESOLUTION |
| mean bandwidth (bytes/s) | 709.6440014188818 | 709.2258647019653 |

`p99_message_age_s` is a genuine finite value on both directions under
the strict formal-trial schema gate (`schema_problems=[]` both
directions). Latency/throughput are unaffected by the legacy-accounting bug.

## Communication metrics -- REORDER-SAFE (authoritative, see `reordering_delivery_audit.json`)

| | epuck1&rarr;epuck2 | epuck2&rarr;epuck1 |
|---|---|---|
| relay forwarded (unique) | 438 | 445 |
| relay dropped | 0 | 0 |
| **actual_missing_count** | **0** | **0** |
| out_of_order (adjacent/displaced, forwarded) | 103/105 | 94/95 |
| **aligned_window_forwarded_to_bag_capture_ratio** | **1.0** | **1.0** |

## Queue drain / rosbag / realtime factor

- `drain_duration_s=0.5302`, `queue_drained=True`.
- `ros2 bag info` succeeded (3198 messages, 7 topics, 54.273241194s duration).
- `preload_realtime_factor=1.004`, `full_load_realtime_factor=0.967`.

## Safety

`min_interrobot_distance_m=0.14919158786608294`, `safety_radius_m=0.14`, `safety_margin_m=0.009191587866082929`.

## Process cleanup / data integrity

- Post-run process check: **CLEAN**.
- Native WSL bag copied to `experiments/05_objective5_impairment_matrix/bags/objective5_impairment_matrix_v1_condition_D_trial05_attempt01/` (gitignored) -- SHA-256 verified identical to native WSL source (bag + diag_logs). See `SHA256SUMS`.

## Batch status

`TRIAL_PASS_PENDING_BATCH_COMPLETION`. This is one trial of the
Condition D formal n=5 batch.

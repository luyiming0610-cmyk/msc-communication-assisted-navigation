# FORMAL_SIM -- Condition A, Trial 02, attempt 01

`objective5_impairment_matrix_v1_condition_A_trial02_attempt01` -- part of a formal n=5 batch for Condition A
(zero impairment). Ran as part of an automated sequential batch under
explicit user authorization; not individually manually observed.

## Verdict

`DATA_VALIDITY=VALID`, `TASK_OUTCOME=SUCCESS`, `PASS`.

## Run identity

- git commit: `48824897885d854e280da40b610a0d5ce67e9162`
- orchestrator SHA-256: `20d2ef63a152a7d65632e4fd3414c9cd1cdaa2a449f58daf7eac1bd28110913b`
- network_impairment_relay.py SHA-256: `f5d408bc3379f79fa70628370b4dfb6d537c4d03a1968fe8dc75a691c3e6d5ff`
- network_impairment.py SHA-256: `253e0d960e9b587a3c5e60587ce7ac56c167fd6aba1c98f8b7b940e821210561`
- sequence_counter.py SHA-256: `57bb0699a444df644d75c4e834b5fd13b5f15a6283d7b1d276ec0b65674f1fd3`
- All four match Trial 01 exactly (verified before this trial started).

## Communication metrics (real analyzer, strict schema, legacy_replay=false)

| | epuck1&rarr;epuck2 | epuck2&rarr;epuck1 |
|---|---|---|
| sample_count | 443 | 461 |
| missing/duplicate/out_of_order | 0/0/0 | 0/0/0 |
| capture_ratio | 1.0 | 1.0 |
| mean/median/p95/p99/max age (s) | 4.514672686230152e-05/0.0/0.0/0.0/0.019999999999999574 | 0.0/0.0/0.0/0.0/0.0 |
| latency_measurement_status | RESOLUTION_LIMITED | RESOLUTION_LIMITED |
| mean bandwidth (bytes/s) | 706.8562307647508 | 707.2298945857104 |

`p99_message_age_s` is a genuine finite value on both directions under
the strict formal-trial schema gate (`schema_problems=[]` both
directions).

## Queue drain / rosbag / realtime factor

- `drain_duration_s=0.2302`, `queue_drained=True`.
- `ros2 bag info` succeeded (3298 messages, 7 topics, 55.111631421s duration).
- `preload_realtime_factor=0.965`, `full_load_realtime_factor=0.999`.

## Safety

`min_interrobot_distance_m=0.15064050840214138`, `safety_radius_m=0.14`, `safety_margin_m=0.010640508402141369`.

## Process cleanup / data integrity

- Post-run process check: **CLEAN**.
- Native WSL bag copied to `experiments/05_objective5_impairment_matrix/bags/objective5_impairment_matrix_v1_condition_A_trial02_attempt01/` (gitignored) -- SHA-256 verified identical to native WSL source (bag + diag_logs). See `SHA256SUMS`.

## Batch status

`TRIAL_PASS_PENDING_BATCH_COMPLETION`. This is one trial of the
Condition A formal n=5 batch.

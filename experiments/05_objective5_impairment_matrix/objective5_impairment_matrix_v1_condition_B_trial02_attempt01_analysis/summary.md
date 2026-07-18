# FORMAL_SIM -- Condition B, Trial 02, attempt 01

`objective5_impairment_matrix_v1_condition_B_trial02_attempt01` -- part of a formal n=5 batch for Condition B.
Automated sequential batch run (Trials 02-05), authorized by the user after Trial 01 manual observation and startup-sync audit; launched via the permanent run_objective5_matrix_from_windows.sh wrapper, behavioral-code SHA verified identical to B01 before the run.

## Verdict

`DATA_VALIDITY=VALID`, `TASK_OUTCOME=SUCCESS`, `PASS`.

## Run identity

- git commit: `14a0650655066075dc4911367b2cdebf58127f45`
- orchestrator SHA-256: `20d2ef63a152a7d65632e4fd3414c9cd1cdaa2a449f58daf7eac1bd28110913b`
- network_impairment_relay.py SHA-256: `f5d408bc3379f79fa70628370b4dfb6d537c4d03a1968fe8dc75a691c3e6d5ff`
- network_impairment.py SHA-256: `253e0d960e9b587a3c5e60587ce7ac56c167fd6aba1c98f8b7b940e821210561`
- sequence_counter.py SHA-256: `57bb0699a444df644d75c4e834b5fd13b5f15a6283d7b1d276ec0b65674f1fd3`
- All four match Trial 01 exactly (verified before this trial started).

## Communication metrics (real analyzer, strict schema, legacy_replay=false)

| | epuck1&rarr;epuck2 | epuck2&rarr;epuck1 |
|---|---|---|
| sample_count | 438 | 435 |
| missing/duplicate/out_of_order | 0/0/0 | 0/0/0 |
| capture_ratio | 1.0 | 1.0 |
| mean/median/p95/p99/max age (s) | 0.20890410958904104/0.20000000000000284/0.21999999999999886/0.21999999999999886/0.21999999999999886 | 0.20914942528735628/0.20000000000000284/0.21999999999999886/0.21999999999999886/0.21999999999999886 |
| latency_measurement_status | VALID_AT_SIM_CLOCK_RESOLUTION | VALID_AT_SIM_CLOCK_RESOLUTION |
| mean bandwidth (bytes/s) | 711.1063982144561 | 710.7790047973922 |

`p99_message_age_s` is a genuine finite value on both directions under
the strict formal-trial schema gate (`schema_problems=[]` both
directions).

## Queue drain / rosbag / realtime factor

- `drain_duration_s=0.4302`, `queue_drained=True`.
- `ros2 bag info` succeeded (3241 messages, 7 topics, 55.699798624s duration).
- `preload_realtime_factor=0.962`, `full_load_realtime_factor=0.948`.

## Safety

`min_interrobot_distance_m=0.14483920109118267`, `safety_radius_m=0.14`, `safety_margin_m=0.00483920109118266`.

## Process cleanup / data integrity

- Post-run process check: **CLEAN**.
- Native WSL bag copied to `experiments/05_objective5_impairment_matrix/bags/objective5_impairment_matrix_v1_condition_B_trial02_attempt01/` (gitignored) -- SHA-256 verified identical to native WSL source (bag + diag_logs). See `SHA256SUMS`.

## Batch status

`TRIAL_PASS_PENDING_BATCH_COMPLETION`. This is one trial of the
Condition B formal n=5 batch.

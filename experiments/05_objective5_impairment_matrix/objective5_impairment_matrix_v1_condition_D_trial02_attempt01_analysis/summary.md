# FORMAL_SIM -- Condition D, Trial 02, attempt 01

`objective5_impairment_matrix_v1_condition_D_trial02_attempt01` -- part of a formal n=5 batch for Condition D.
Automated sequential batch run (D Trials 02-05), authorized by the user, single wsl.exe invocation per trial (interop register+verify, then eamon-session orchestrator invocation). manual_observation.status=NOT_OBSERVED throughout this batch per explicit user instruction.

## Verdict

`DATA_VALIDITY=VALID`, `TASK_OUTCOME=SUCCESS`, `FAIL`.

## Run identity

- git commit: `c12b22a6f11983385e4a93a9e604bb6f7d3ef92a`
- orchestrator SHA-256: `20d2ef63a152a7d65632e4fd3414c9cd1cdaa2a449f58daf7eac1bd28110913b`
- network_impairment_relay.py SHA-256: `f5d408bc3379f79fa70628370b4dfb6d537c4d03a1968fe8dc75a691c3e6d5ff`
- network_impairment.py SHA-256: `253e0d960e9b587a3c5e60587ce7ac56c167fd6aba1c98f8b7b940e821210561`
- sequence_counter.py SHA-256: `57bb0699a444df644d75c4e834b5fd13b5f15a6283d7b1d276ec0b65674f1fd3`
- All four match Trial 01 exactly (verified before this trial started).

## Communication metrics (real analyzer, strict schema, legacy_replay=false)

| | epuck1&rarr;epuck2 | epuck2&rarr;epuck1 |
|---|---|---|
| sample_count | 430 | 419 |
| missing/duplicate/out_of_order | 197/0/91 | 180/0/83 |
| capture_ratio | 1.0 | 1.0 |
| mean/median/p95/p99/max age (s) | 0.1620930232441849/0.1599999999999966/0.29999999999999716/0.3000000000000007/0.3000000000000007 | 0.1578997613341276/0.1599999999999966/0.29999999999999716/0.30000000000000004/0.3000000000000007 |
| latency_measurement_status | VALID_AT_SIM_CLOCK_RESOLUTION | VALID_AT_SIM_CLOCK_RESOLUTION |
| mean bandwidth (bytes/s) | 708.6365071638157 | 707.2299479741904 |

`p99_message_age_s` is a genuine finite value on both directions under
the strict formal-trial schema gate (`schema_problems=[]` both
directions).

## Queue drain / rosbag / realtime factor

- `drain_duration_s=0.5302`, `queue_drained=True`.
- `ros2 bag info` succeeded (3206 messages, 7 topics, 52.163107966s duration).
- `preload_realtime_factor=0.959`, `full_load_realtime_factor=0.962`.

## Safety

`min_interrobot_distance_m=0.1516223239385512`, `safety_radius_m=0.14`, `safety_margin_m=0.011622323938551199`.

## Process cleanup / data integrity

- Post-run process check: **CLEAN**.
- Native WSL bag copied to `experiments/05_objective5_impairment_matrix/bags/objective5_impairment_matrix_v1_condition_D_trial02_attempt01/` (gitignored) -- SHA-256 verified identical to native WSL source (bag + diag_logs). See `SHA256SUMS`.

## Batch status

`TRIAL_PASS_PENDING_BATCH_COMPLETION`. This is one trial of the
Condition D formal n=5 batch.

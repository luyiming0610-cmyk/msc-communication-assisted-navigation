# FORMAL_SIM -- Condition A, Trial 01, attempt 01

`objective5_impairment_matrix_v1_condition_A_trial01_attempt01` -- the
first trial of the formal n=5 Condition A batch (zero impairment).
**This is one trial, not the whole batch.** Trials 02-05 have not run;
this file must not be read as Condition A batch completion.

## Verdict

`DATA_VALIDITY=VALID`, `TASK_OUTCOME=SUCCESS`, `PASS`.

## Run identity

- git commit: `2837d08609292dbfdab10b93fd68c610da25357f`
- orchestrator SHA-256: `20d2ef63a152a7d65632e4fd3414c9cd1cdaa2a449f58daf7eac1bd28110913b`
- network_impairment_relay.py SHA-256: `f5d408bc3379f79fa70628370b4dfb6d537c4d03a1968fe8dc75a691c3e6d5ff`
- network_impairment.py SHA-256: `253e0d960e9b587a3c5e60587ce7ac56c167fd6aba1c98f8b7b940e821210561`
- sequence_counter.py SHA-256: `57bb0699a444df644d75c4e834b5fd13b5f15a6283d7b1d276ec0b65674f1fd3`
- Launched manually by the user directly inside an already-registered
  eamon WSL session (`bash run_objective5_impairment_matrix_trial.sh A
  1`), not via a fresh assistant-issued `wsl.exe` call -- see
  `runtime_manifest.json`'s `launch_method` field for the exact
  correction.

## Communication metrics (real analyzer, strict schema, `legacy_replay=false`)

| | epuck1&rarr;epuck2 | epuck2&rarr;epuck1 |
|---|---|---|
| relay received/forwarded | 415/415 | 434/434 |
| independent_drop / outage_drop | 0/0 | 0/0 |
| sample_count | 415 | 434 |
| missing / duplicate / out_of_order | 0/0/0 | 0/0/0 |
| capture_ratio | 1.0 | 1.0 |
| mean / median / p95 / p99 / max message age (s) | 4.82e-05 / 0.0 / 0.0 / 0.0 / 0.02 | 1.84e-04 / 0.0 / 0.0 / 0.0 / 0.08 |
| latency_measurement_status | RESOLUTION_LIMITED | RESOLUTION_LIMITED |
| mean bandwidth (bytes/s) | 708.15 | 708.25 |

`p99_message_age_s` is a genuine finite value (`0.0`) on both
directions under the strict formal-trial schema gate -- not `null`,
confirming `schema_problems=[]` both directions in
`matrix_analysis.json`.

## Queue drain / rosbag / realtime factor

- `drain_duration_s=0.2302`, `queue_drained=true`,
  `pending_queue_depth=0` both directions.
- `metadata.yaml` present; `bag_record.log` has zero drop/warn/error
  lines; `ros2 bag info` succeeded (3207 messages, 8 topics, 52.61s
  duration) -- see `runtime_manifest.json`.
- `preload_realtime_factor=1.012`, `full_load_realtime_factor=0.963`
  (both within the 0.8-1.2 tolerance band).

## Safety

`min_interrobot_distance_m=0.1430942842844398`,
`safety_radius_m=0.14`, **`safety_margin_m≈0.003094`** (about 3.09mm).
**PASS, but the margin is small** -- retained explicitly here and must
be carried into any later batch-level summary, per instruction. This is
reported as an observed fact for this specific trial; it is not grounds
to alter the frozen geometry or controller.

## Manual observation (user, real-time)

Avoidance completed: **yes**. Collision: **no**. Spinning/visible
oscillation: **no**. Recovered-and-auto-stopped: **yes**. Visible
desynchronization: **no**. Other notes: none.

*Correction*: `recovered_and_auto_stopped` was originally reported as
"no" (a fill-in error, not a re-observation) and has been corrected to
"yes" here and in `final_verdict.json`/`README.md`. No automated
measurement field, and no other file, was touched by this correction.

## Process cleanup / data integrity

- Post-run process check (`webots`, `webots_ros2_driver`,
  `state_publisher`, `cooperative_avoider`, relay/counter launch
  processes, `sequence_counter`, `ros2 bag record`): **CLEAN**.
- Native WSL bag copied to
  `experiments/05_objective5_impairment_matrix/bags/objective5_impairment_matrix_v1_condition_A_trial01_attempt01/`
  (gitignored, never committed) -- full file listing + SHA-256 verified
  identical to the native WSL source both for the bag and the diag_logs
  directory (relay CSVs, counter JSON, controller/simulation/state
  logs). See `SHA256SUMS`.

## Batch status

`TRIAL_01_PASS_PENDING_BATCH_COMPLETION`. Registry/index/project_status
are updated to record this specific trial's result; none of them mark
Condition A as a completed batch. Trial 02 has not been started.

# FORMAL_SIM -- Condition B, Trial 01, attempt 01

`objective5_impairment_matrix_v1_condition_B_trial01_attempt01` -- Trial 01 of a formal n=5 batch for Condition B
(fixed 0.20s relay delay, zero jitter/loss, outage disabled). Launched
manually by the user via the permanent Windows->WSL wrapper
(`run_objective5_matrix_from_windows.sh`) for individual real-time
observation -- **NOT** part of an automated batch. *(Correction: this
paragraph originally carried the Condition-A-batch template's default
"automated sequential batch... not individually manually observed"
text, incorrect for this manually-launched trial; corrected here, see
`final_verdict.json`'s `revision_note` for the same correction applied
there.)* Trials 02-05 will not auto-start until the user's manual
observation is received and confirmed consistent with the automated
evidence below.

## Trigger mechanism (offline audit, no Webots run -- see `objective5_condition_B_trial01_trigger_mechanism_audit.{json,md}`)

`PURE_COMMUNICATION_CPA_AVOIDANCE`: `first_trigger_reason=PREDICTED_CPA`,
every `LOCAL_*` event counter is 0. `SAFE_STOP_STALE=4` is a benign
pre-encounter startup transient (ros_time~23-26s), well before the
first `AVOID_TURN` at ros_time=36.660s. The local ToF/IR safety layer
remained enabled throughout (never disabled) but did not engage.

## Verdict

`DATA_VALIDITY=VALID`, `TASK_OUTCOME=SUCCESS`, `PASS`.

## Run identity

- git commit: `3f30f708cac4f102f920925236e07c518356337f`
- orchestrator SHA-256: `20d2ef63a152a7d65632e4fd3414c9cd1cdaa2a449f58daf7eac1bd28110913b`
- network_impairment_relay.py SHA-256: `f5d408bc3379f79fa70628370b4dfb6d537c4d03a1968fe8dc75a691c3e6d5ff`
- network_impairment.py SHA-256: `253e0d960e9b587a3c5e60587ce7ac56c167fd6aba1c98f8b7b940e821210561`
- sequence_counter.py SHA-256: `57bb0699a444df644d75c4e834b5fd13b5f15a6283d7b1d276ec0b65674f1fd3`
- All four match Trial 01 exactly (verified before this trial started).

## Communication metrics (real analyzer, strict schema, legacy_replay=false)

| | epuck1&rarr;epuck2 | epuck2&rarr;epuck1 |
|---|---|---|
| sample_count | 444 | 443 |
| missing/duplicate/out_of_order | 0/0/0 | 0/0/0 |
| capture_ratio | 1.0 | 1.0 |
| mean/median/p95/p99/max age (s) | 0.20864864865090083/0.20000000000000284/0.21999999999999886/0.21999999999999886/0.21999999999999886 | 0.20916478555530468/0.20000000000000284/0.21999999999999886/0.21999999999999886/0.21999999999999886 |
| latency_measurement_status | VALID_AT_SIM_CLOCK_RESOLUTION | VALID_AT_SIM_CLOCK_RESOLUTION |
| mean bandwidth (bytes/s) | 708.7997883517908 | 709.3269245090428 |

`p99_message_age_s` is a genuine finite value on both directions under
the strict formal-trial schema gate (`schema_problems=[]` both
directions).

## Queue drain / rosbag / realtime factor

- `drain_duration_s=0.4302`, `queue_drained=True`.
- `ros2 bag info` succeeded (3278 messages, 7 topics, 55.497479486s duration).
- `preload_realtime_factor=0.955`, `full_load_realtime_factor=0.958`.

## Safety

`min_interrobot_distance_m=0.14777153762172363`, `safety_radius_m=0.14`, `safety_margin_m=0.007771537621723612`.

## Manual observation (user, real-time)

Avoidance completed: **yes**. Collision: **no**. Spinning/visible
oscillation: **no**. Recovered-and-auto-stopped: **yes**. Visible
desynchronization during avoidance: **no**. Later avoidance than
Condition A: not observed (user could not tell). **Startup asynchrony
observed: yes** -- the two robots' avoidance maneuvers were themselves
synchronized, but epuck1 (bottom-left) visibly began moving noticeably
earlier than epuck2 (top-right) at trial start. This observation is
preserved and is the subject of a separate offline startup-timing-sync
audit (`objective5_conditionB_trial01_startup_sync_audit.{json,md}`)
before Trial 02 may proceed.

`final_confirmed_verdict=PASS_PENDING_STARTUP_SYNC_AUDIT` in
`final_verdict.json`.

## Process cleanup / data integrity

- Post-run process check: **CLEAN**.
- Native WSL bag copied to `experiments/05_objective5_impairment_matrix/bags/objective5_impairment_matrix_v1_condition_B_trial01_attempt01/` (gitignored) -- SHA-256 verified identical to native WSL source (bag + diag_logs). See `SHA256SUMS`.

## Batch status

`TRIAL_PASS_PENDING_BATCH_COMPLETION`. This is one trial of the
Condition B formal n=5 batch.

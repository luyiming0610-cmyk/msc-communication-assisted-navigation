# FORMAL_SIM -- Condition D, Trial 01, attempt 01

`objective5_impairment_matrix_v1_condition_D_trial01_attempt01` -- part of a formal n=5 batch for Condition D.
Launched individually via the permanent run_objective5_matrix_from_windows.sh wrapper for direct user Webots observation (not part of an automated batch); Trial 01 of the Condition D formal n=5 batch, following the same manually-observed-first pattern as Condition A/B/C Trial 01.

## Verdict

`DATA_VALIDITY=VALID`, `TASK_OUTCOME=SUCCESS`, automated strict-gate label `FAIL`.

**Clarification**: the automated strict-gate label reuses the
zero-reordering criteria built for Conditions A/B/C
(`missing_count==0`, `out_of_order_count==0`). Condition D's frozen
design (`jitter_s=0.30` with `delay_s=0.15`) is intended to produce
genuine out-of-order delivery, and did: `out_of_order_count=89/92`
(both directions > 0), which is the condition-specific manipulation
check succeeding, not a defect. `capture_ratio` remains 1.0/1.0023
(every arrived message was captured), `independent_drop_count` and
`outage_drop_count` are 0 both directions, `min_interrobot_distance_m`
exceeds `safety_radius_m` with a positive margin, and process cleanup
is CLEAN. See `final_verdict.json`'s `trial_verdict_clarification_note`
for the full text. The condition-appropriate verdict awaits the user's
manual observation.

## Run identity

- git commit: `9ace19d5ffe9966be3fb7d0f192672a0fed05e0d`
- orchestrator SHA-256: `20d2ef63a152a7d65632e4fd3414c9cd1cdaa2a449f58daf7eac1bd28110913b`
- network_impairment_relay.py SHA-256: `f5d408bc3379f79fa70628370b4dfb6d537c4d03a1968fe8dc75a691c3e6d5ff`
- network_impairment.py SHA-256: `253e0d960e9b587a3c5e60587ce7ac56c167fd6aba1c98f8b7b940e821210561`
- sequence_counter.py SHA-256: `57bb0699a444df644d75c4e834b5fd13b5f15a6283d7b1d276ec0b65674f1fd3`
- All four match Trial 01 exactly (verified before this trial started).

## Communication metrics (real analyzer, strict schema, legacy_replay=false)

| | epuck1&rarr;epuck2 | epuck2&rarr;epuck1 |
|---|---|---|
| sample_count | 434 | 430 |
| missing/duplicate/out_of_order | 189/0/89 | 192/0/92 |
| capture_ratio | 1.0 | 1.0023310023310024 |
| mean/median/p95/p99/max age (s) | 0.16350230413824737/0.16000000000000014/0.29999999999999716/0.30000000000000016/0.3000000000000007 | 0.15855813953255696/0.16000000000000014/0.29999999999999716/0.3000000000000007/0.3000000000000007 |
| latency_measurement_status | VALID_AT_SIM_CLOCK_RESOLUTION | VALID_AT_SIM_CLOCK_RESOLUTION |
| mean bandwidth (bytes/s) | 708.5697965429898 | 709.1136118964199 |

`p99_message_age_s` is a genuine finite value on both directions under
the strict formal-trial schema gate (`schema_problems=[]` both
directions).

## Queue drain / rosbag / realtime factor

- `drain_duration_s=0.5302`, `queue_drained=True`.
- `ros2 bag info` succeeded (3234 messages, 7 topics, 53.292035088s duration).
- `preload_realtime_factor=0.96`, `full_load_realtime_factor=0.955`.

## Safety

`min_interrobot_distance_m=0.1444723957613864`, `safety_radius_m=0.14`, `safety_margin_m=0.0044723957613863885`.

## Process cleanup / data integrity

- Post-run process check: **CLEAN**.
- Native WSL bag copied to `experiments/05_objective5_impairment_matrix/bags/objective5_impairment_matrix_v1_condition_D_trial01_attempt01/` (gitignored) -- SHA-256 verified identical to native WSL source (bag + diag_logs). See `SHA256SUMS`.

## Batch status

`TRIAL_PASS_PENDING_BATCH_COMPLETION`. This is one trial of the
Condition D formal n=5 batch.

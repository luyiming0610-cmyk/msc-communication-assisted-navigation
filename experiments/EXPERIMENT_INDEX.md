# Experiment Index

Display-folder naming is governed by `NAMING_CONVENTION.md`. Historical
controller versions and bag/trial IDs shown below remain unchanged for
reproducibility even when their parent display folder has been renamed.

Human-readable index into `experiment_registry.csv` (machine-readable, one
row per experiment/batch) and `path_manifest.csv` (every logical path, both
Windows and WSL forms). This document does NOT move, rename, or duplicate
any original data — it only points at where things already are. Original
bag/log/analysis directories stay exactly where the pilot scripts already
write them; see the actual paths in the CSVs.

**Read `PROJECT_HANDOFF.md` (repo root) first** if you are new to this
project — it is the single entry point. This file is the detailed index it
points to.

## Categories

### 01_protocol_and_unit_tests
Message definition, protocol version, unit tests, build results.
- `src/epuck2_comm_interfaces/msg/EpuckState.msg` — frozen `PROTOCOL_VERSION=1`
- `src/epuck2_comm_interfaces/PROTOCOL_FREEZE_20260717.md` — the freeze manifest (fingerprint, rules)
- `src/epuck2_comm/test/` — 126 tests, all passing as of commit `03ce36c`
- Registry rows: `epuck2_comm_unit_tests`, `epuck_state_msg_protocol_freeze`

### 02_controller_regression
v1/v2/v3/v4 controller design, defect diagnosis, regression evidence. This
is controller-development evidence — **not** formal communication-performance
statistics, even though some of it lives inside `cooperative_avoidance_20260716/`.
- `experiments/3-1.局部避障锁存实验/` — controller_v2 pilot_a/pilot_a2
- `experiments/3-2.统一遭遇避障实验/` — controller_v3 pilot_a3 (forensic evidence that motivated v4)
- `experiments/3-3.全传感器避障实验/bags/*_static_box_{a,b,c,d}` — pilot_v4_a attempts 1-4 (3 fails, 1 pass)
- `experiments/3-3.全传感器避障实验/bags/*_static_box_fusion_{a,b2,b3}` — pilot_v4_b/b2/b3 (fail, partial, pass)
- `experiments/cooperative_avoidance_20260716/bags/combined_*` — the controller_v1 combined-scenario box-corner-runaway-turn defect that started the whole v1→v4 chain (see `combined_wood_moving_peer_README.md`'s 2026-07-17 entry)
- Registry rows: `controller_v2_*`, `controller_v3_*`, `v4_pilot_*`, `combined_wood_moving_peer_v1_defect_pilots`, `combined_wood_moving_peer_postfix_pilots`

### 03_phase4_task_validation
Static wooden block, pure CPA, combined avoidance — Objective 4's
task-specific validation vehicle.
- **Phase 1** (controller_v1): `experiments/fusion_static_neighbor_20260716/` — 5/5 formal (`local_static_long_trial_02..07`), 1 diagnostic (`local_static_trial_01`)
- **Phase 2/3** (controller_v1): `experiments/cooperative_avoidance_20260716/` — 45 formal trials across 9 batches (head-on centered/offset/crossing/ablation local-only/ablation fused), see `cooperative_avoidance_experiment_index_20260716.md` for the authoritative per-trial table
- **Phase 4** (controller_v4, **SEALED**): `experiments/3-3.全传感器避障实验/bags/*_combined_formal_trial0{1..5}` — **5/5 PASS**, manifest commit `e32560e`, batch summary commit `cbb1897`
  - **Binding limitation**: all 5/5 trials triggered via `PROXIMITY_FALLBACK`, none via `PREDICTED_CPA`. Naming rule: "staged local-obstacle avoidance followed by communication-assisted proximity/cooperative avoidance." Never describe this batch as preventing an otherwise-certain head-on collision.
  - Exclusionary pilots (not pooled with the formal 5/5): `head_on_cpa_pure_c1`, `combined_trial1`, `combined_trial2_timebasefix`
- Registry rows: `phase1_*`, `phase2_3_*`, `v4_pure_cpa_*`, `v4_combined_pilot_*`, `v4_phase4_formal_trial0{1..5}`, `phase4_formal_batch_5of5`

### 04_objective5_comm_baseline
Zero-delay/zero-loss communication baseline. Two formal results now
exist (latency-partial + latency-complete); the rest of this category
remains diagnostic-only.
- `experiments/3-3.全传感器避障实验/bags/*_comm_baseline_trial{1,2,3}` — FAIL, `/mnt/c` rosbag-write message loss (~40-55%), superseded by the native-path diagnostic
- `experiments/3-3.全传感器避障实验/bags/*_comm_baseline_native_trial0{1,2}` — PASS (comm-layer-only, no `cooperative_avoider`), aligned-window PDR=1.0, confirms root cause is `/mnt/c` I/O not the relay/transport
- `experiments/3-3.全传感器避障实验/bags/*_objective5_comm_baseline_zero_impairment_formal_trial01` — **PASS, FORMAL_SIM.** Genuine `cooperative_avoider` task completion under zero impairment, native WSL ext4 bag path. Aligned-window PDR=1.0 both robots; `sequence_gap_count`=`duplicate_count`=`out_of_order_count`=0 both robots; `sequence_counter` `complete=true` both robots; realtime factor 0.963 (preload) / 0.951 (full load); message rate ≈8.69 Hz/robot; mean bandwidth ≈696 bytes/s/robot; no bag/QoS drop-warn-error lines. **metric_coverage: PDR=VALID sequence_integrity=VALID throughput=VALID task_behavior=VALID latency=NOT_MEASURED.** Message age/latency is N/A, permanently (not re-analyzed/backfilled) — root cause was `analyze_comm_performance.py` mixing rosbag2's own wall-clock recording timestamp with `message.stamp` (sim time), NOT an unset stamp field (`state_publisher.py` sets it correctly). The first 3 attempts at this exact trial failed for orchestration-script reasons (WSL interop, then two distinct process-shutdown bugs in `run_objective5_comm_baseline_formal_trial.sh`), not communication-result reasons — kept, not deleted, in that experiment directory's README execution_attempts table.
- `experiments/3-3.全传感器避障实验/bags/*_objective5_timestamp_latency_validation_pilot01_condition_{a_delay0,b_delay025}` — PASS, PILOT (diagnostic-only, no `cooperative_avoider`). Validates the stamp/latency measurement chain before trial02_stamp: condition_a (delay=0) mean age ~microsecond-scale; condition_b (delay=0.25s) observed increment ~0.26s vs configured 0.25s, error ~0.01s. Never pooled with formal statistics.
- `experiments/3-3.全传感器避障实验/bags/*_objective5_comm_baseline_zero_impairment_formal_trial02_stamp` — **PASS, FORMAL_SIM.** Latency-complete companion to trial01 under `protocol_v1.1_stamp_semantics` (wire schema unchanged, still `PROTOCOL_VERSION=1`): `state_publisher.py` now gates publication on a valid ROS clock (`WAITING_FOR_CLOCK`), `sequence_counter.py` now computes live per-message latency (message.stamp vs its own receipt time, same clock domain). Same PDR/sequence/task results as trial01 (aligned-window PDR=1.0, 0 gaps/dup/oo, `complete=true`, realtime factor 0.996/1.003) plus **metric_coverage all five VALID**: live-counter mean/median/p95/max `message_age_s`=0.0 both robots (zero configured delay). **Does not replace trial01** — both remain separately registered with different metric_coverage.
- `src/epuck2_comm/epuck2_comm/{analyze_comm_performance,network_impairment,network_impairment_relay,sequence_counter,state_publisher}.py` — the tooling itself, 144/144 unit tests passing (10 sequence_counter reliability tests + this pass's stamp-semantics tests: state_publisher WAITING_FOR_CLOCK, sequence_counter live latency, relay CSV source_stamp/actual_release_time, analyze_comm_performance clock-domain-mismatch guard)
- **peer_timeout_s audit** (read-only, frozen controller untouched): freshness is judged by callback receipt time, not `msg.stamp`; a constant delay alone does not trigger `peer_timeout` (only jitter/loss can) — an earlier draft's "0.6s delay triggers timeout" claim is retracted.
- **network_impairment_relay audit**: scheduling uses ROS/sim time (not wall time), so delay semantics don't drift with realtime factor; jitter can cause reordering (release-time min-heap); drop/jitter fully reproducible via seeded `random.Random`.
- **Not yet done**: the delay/loss impairment matrix (A-G conditions) — design only, submitted for user confirmation, not run
- Registry rows: `comm_baseline_v1_trial{1,2,3}`, `comm_baseline_native_diag_trial0{1,2}`, `objective5_comm_baseline_zero_impairment_formal_trial01`, `objective5_timestamp_latency_validation_pilot01`, `objective5_comm_baseline_zero_impairment_formal_trial02_stamp`

### 05_objective5_impairment_matrix
Delay, loss, combined-impairment experiments (Conditions A-G). **Design
frozen, orchestrator built and exclusionary-pilot-verified. Condition
A's formal n=5 batch is COMPLETE: 5/5 PASS**
(`objective5_impairment_matrix_v1_condition_A_trial01..05_attempt01`,
see `objective5_impairment_matrix_v1_condition_A_formal_batch_summary.md`).
**Conditions B-G have not started and will not auto-start; each
remaining condition's Trial 01 awaits explicit user confirmation before
it runs.**
- Design: `objective5_impairment_matrix_design_v1.md` (revision 2+),
  `objective5_impairment_matrix_conditions.csv`,
  `objective5_impairment_matrix_analysis_plan.md` (revision 2+).
  Two-dimensional verdict (`DATA_VALIDITY` = VALID/INVALID,
  `TASK_OUTCOME` = SUCCESS/SAFE_DEGRADATION/UNSAFE_FAILURE/NOT_EVALUABLE
  — a simplified 4-category scheme, superseding an earlier 6-category
  draft never implemented in code) is real, tested code
  (`tools/matrix_verdict.py`), not design-doc prose. Final,
  non-overlapping per-trial/per-direction seed table for randomized
  Conditions D/E/F/G: trial 01-05 -> epuck1→epuck2
  `4001..4005`, epuck2→epuck1 `14001..14005` (10 distinct values, no
  reuse across trials/directions/conditions).
- Orchestrator: `tools/run_objective5_impairment_matrix_trial.sh`, one
  unified parameterized script for every condition (frozen CSV is the
  only parameter source, no hand-typed override), supports both formal
  trials and `--pilot LABEL` exclusionary runs in separate, non-colliding
  directories.
- Two exclusionary pilots (`EXCLUSIONARY_DIAGNOSTIC`, not part of the
  formal n=5): `objective5_matrix_v1_conditionA_exclusionary_pilot01`
  (zero-impairment orchestrator equivalence check; original run's
  DATA_VALIDITY was falsely INVALID due to a since-fixed JSON-parsing
  bug, corrected post-hoc in `trial_verdict_corrected.json` without a
  re-run: DATA_VALIDITY=VALID, TASK_OUTCOME=SUCCESS) and
  `objective5_matrix_v1_conditionF_exclusionary_pilot01/02/03`
  (bidirectional periodic-outage path; `pilot01`/`pilot02` failed on
  real, now-fixed orchestrator bugs — process-group leaks in the
  controller/sim/state_publisher launches, a `DATA_VALIDITY` field
  mislabeling bug, a `grep -c` false-positive, and a relay-status echo
  timeout race, each preserved with a `NOTE.md`; `pilot03` is the valid
  result: DATA_VALIDITY=VALID, TASK_OUTCOME=SUCCESS, both directions'
  `current_outage_index=5` and consistent `outage_drop_count` (30/30),
  `independent_drop_count=0/0`).
- **Formal Condition A batch, COMPLETE, 5/5 PASS**:
  `objective5_impairment_matrix_v1_condition_A_trial01..05_attempt01`.
  Trial 01 launched manually and individually observed by the user
  (`DATA_VALIDITY=VALID`, `TASK_OUTCOME=SUCCESS`,
  `min_interrobot_distance_m=0.1430942842844398`, safety margin
  ~3.09mm; manual observation confirmed avoidance completed, both
  robots recovered and auto-stopped normally, no collision/
  spinning/desync -- `recovered_and_auto_stopped` was originally
  recorded as "no", a fill-in error corrected to "yes", see
  `final_verdict.json`'s `manual_observation.correction_note`). Trials
  02-05 ran automatically under explicit user authorization (frozen
  Condition A config, no controller/relay/analyzer/geometry/pose/
  threshold changes, behavioral-code SHA-256 verified identical to
  Trial 01 before each run, no per-trial manual observation), each
  independently `PASS`:
  `DATA_VALIDITY=VALID`/`TASK_OUTCOME=SUCCESS`/`analyzer_ok=true`
  (strict schema, `legacy_replay=false`, p99 finite both directions)
  in all 5 trials, `capture_ratio=1.0` and 0 drops both directions in
  all 5 trials, `min_interrobot_distance_m` in
  `[0.14178534915907265, 0.15064050840214138]` (all > `safety_radius_m
  =0.14`; **tightest margin ~1.79mm, Trial 05** -- reported explicitly,
  not grounds to alter the frozen geometry/controller), process
  cleanup CLEAN after every run. Full cross-trial mean/stdev/min/max
  (message counts, capture ratio, latency mean/median/p95/p99/max,
  throughput, realtime factor, safety margin, task completion time) in
  `objective5_impairment_matrix_v1_condition_A_formal_batch_summary.md`.
  **The 0/near-0-second message age in all 5 trials reflects
  simulation-clock resolution under zero configured impairment, not
  real physical network delay** -- the correct baseline reference for
  Conditions B-G. Raw bag/diag_logs for every trial preserved outside
  git (native WSL + Windows copy, SHA-256-verified identical), see each
  trial's own `..._analysis/` directory.
- **Formal Condition B batch, COMPLETE, 5/5 PASS**:
  `objective5_impairment_matrix_v1_condition_B_trial01..05_attempt01`
  (fixed `delay_s=0.20`, `jitter_s=0.0`, `drop_probability=0.0`, outage
  disabled -- world/controller/thresholds/initial poses identical to
  Condition A; local ToF/IR safety layer enabled throughout, never
  engaged). Trial 01 launched manually via the permanent wrapper and
  individually observed (final PASS confirmed after an offline
  startup-sync audit classified the user-observed ~2.6s startup
  asynchrony as `ORCHESTRATOR_LAUNCH_TIMING_VARIANCE` -- present in
  Condition A too, direction not fixed, avoidance always synchronized);
  Trials 02-05 ran automatically under explicit authorization with
  behavioral-code SHA-256 verified identical to B01 before each run.
  All 5: `DATA_VALIDITY=VALID`/`TASK_OUTCOME=SUCCESS`/`analyzer_ok=true`
  (strict schema, p99 finite), `capture_ratio=1.0`, 0 drops both
  directions, `first_trigger_reason=PREDICTED_CPA` with every `LOCAL_*`
  counter 0 (`PURE_COMMUNICATION_CPA_AVOIDANCE` 5/5). Median message
  age = 0.200s exactly (the configured delay) in every trial;
  mean ~0.209s; p95/p99/max 0.22s. `min_interrobot_distance_m` in
  `[0.1418936528990241, 0.14777153762172363]` -- margin mean ~4.66mm,
  **tightest ~1.89mm (Trial 05)**, all positive. Per-trial
  TIMEBASE_INIT startup deltas recorded as covariates
  ([-2.64, -2.38, -0.02, -0.34, 0.00]s; AVOID_TURN/RECOVER deltas
  0.000s in all 5). Full cross-trial statistics + descriptive
  startup-delta relationship check (no significance claims, n=5) in
  `objective5_impairment_matrix_v1_condition_B_formal_batch_summary.{json,md}`.
  **Condition F precondition recorded in `project_status.json`**: the
  startup offset (up to ~2.66s, ~3.8x F's 0.7s outage duration) must be
  checked against actual outage-window timing before formal F begins.
- **Formal Condition C batch, COMPLETE, FAILED SAFETY GATE (4/5 SUCCESS)**:
  `objective5_impairment_matrix_v1_condition_C_trial01..05_attempt01`
  (fixed `delay_s=1.00`, `jitter_s=0.0`, `drop_probability=0.0`, outage
  disabled; the same frozen controller, world, thresholds and behavioral-code
  SHA-256 values as Conditions A/B). All 5 trials have `DATA_VALIDITY=VALID`,
  `capture_ratio=1.0`, zero relay drops, finite strict-schema latency fields,
  and median message age `1.000s` in both directions. Trials 01-04 have
  `TASK_OUTCOME=SUCCESS`; Trial 05 is the retained valid
  `TASK_OUTCOME=UNSAFE_FAILURE` because its minimum separation
  `0.1389086m` is below the frozen `0.1400000m` safety radius by about
  `1.091mm`. It was not rerun and no threshold/geometry/controller value was
  changed. All 5 first triggers are `PREDICTED_CPA`; every `LOCAL_*` count is
  zero. The user-observed S-shaped path is a reproducible batch-level effect:
  every robot in every trial made exactly 9 angular-command direction
  reversals (about five S-lobes) within one continuous `AVOID_PASS`, not
  repeated CPA encounters or IR/ToF fallback. Full evidence:
  `objective5_impairment_matrix_v1_condition_C_formal_batch_summary.{json,md}`.
- **Condition D (jitter/reordering), COMPLETE -- 5/5 INCLUDED
  (D01/D02/D03/D05/D06), D04 EXCLUDED (measurement-chain artifact)**:
  frozen `delay_s=0.15`, `jitter_s=0.30` peak-to-peak → per-message release
  delay `[0,0.30]s`, `drop_probability=0.0`, outage disabled, seeds
  `400N/1400N` per trial N; same frozen controller/world/thresholds/
  behavioral-code SHA-256 as A/B/C. All trials use the three-axis verdict
  (`DATA_VALIDITY` / `MANIPULATION_VALIDITY` / `TASK_OUTCOME`), computed via
  a new versioned set-based analyzer `tools/reorder_safe_delivery_analyzer.py`
  (v1, 13 unit tests) that replaces the live `sequence_counter.py`'s
  adjacent-delta accounting -- that accounting is wrong under reordering
  (bogus missing counts, impossible ratios >1) and its disputed
  `matrix_analysis.json` fields are annotated METHOD_INVALID in every D
  trial's analysis dir (preserved, not deleted). Analyzer output fields are
  named `aligned_window_forwarded_to_bag_capture_ratio` and
  `relay_received_to_forwarded_ratio` (renamed 2026-07-19 for accuracy, no
  value change). Every trial: `manual_observation.status=NOT_OBSERVED`.
  Verdict schema corrected 2026-07-19 to FIVE axes:
  `DATA_ARTIFACT_INTEGRITY` (schema/bag-mechanics), `MANIPULATION_VALIDITY`
  (relay-level: 0 drops + reordering genuinely induced, cross-checked
  against the independent online `sequence_counter` subscriber -- NOT gated
  on bag capture), `TASK_OUTCOME`, `FORMAL_MEASUREMENT_VALIDITY`
  (bag-recording-chain completeness specifically), `FORMAL_BATCH_INCLUSION`.
  - **D01**: `VALID/VALID/SUCCESS/VALID` → **INCLUDED**, margin +4.472mm.
    Corrected from an initial mis-derived `missing=189/192`/
    `capture_ratio=1.00233` (root cause: `sequence_counter.py` using
    first-ARRIVED sequence as minimum). `PREDICTED_CPA`, `LOCAL_*` all zero.
  - **D02**: `VALID/VALID/SUCCESS/VALID` → **INCLUDED**, margin +11.62mm.
  - **D03**: `VALID/VALID/SUCCESS/VALID` → **INCLUDED**, margin **+0.17mm**
    -- RAZOR-THIN, a threshold pass, explicitly NOT robust safety; genuine,
    preserved as-is, not rerun/adjusted.
  - **D04**: `DATA_ARTIFACT_INTEGRITY=VALID`, `MANIPULATION_VALIDITY=VALID`
    (relay: 0 drops, reordering genuinely induced, independently confirmed
    complete by `epuck2_counter.json`'s online subscriber:
    `received_count=unique_sequence_count=450`, matching the relay's full
    forwarded count), `TASK_OUTCOME=SUCCESS` (margin +3.099mm), but
    **`FORMAL_MEASUREMENT_VALIDITY=INVALID`**
    (`aligned_window_forwarded_to_bag_capture_ratio=0.9976958525345622`,
    sequence 17 epuck2→epuck1) → **`FORMAL_BATCH_INCLUSION=EXCLUDED`**.
    Precise wording: a **rosbag-only single-message capture gap** -- the
    message was forwarded by the relay and received live by the online
    counter, so this is NOT a relay-to-downstream loss, NOT a
    communication-manipulation failure, and NOT a task failure. Classified
    `EXCLUDED_MEASUREMENT_CHAIN_ATTEMPT`: preserved in full, NOT rerun, NOT
    counted toward the formal n=5. See
    `objective5_impairment_matrix_v1_condition_D_trial04_attempt01_analysis/STOP_CONDITION_REPORT.md`
    (correction section) and `three_axis_verdict.json`.
  - **D05**: `VALID/VALID/SUCCESS/VALID` → **INCLUDED**, margin +9.19mm.
  - **D06**: `VALID/VALID/SUCCESS/VALID` → **INCLUDED**, margin +4.32mm.
    **Sole preregistered replacement for D04.** Authorized by the user;
    `objective5_impairment_matrix_conditions.csv`'s Condition D seed lists
    extended by exactly one entry (`4006`/`14006`, following the existing
    base+index pattern, commit `fa47182`; CSV SHA-256 before
    `7d0f3110...`/after `f98a47b6...`, both recorded in D06's
    `runtime_manifest.json` alongside `replacement_for=D04`). `n_trials`
    for Condition D remains `5` -- D01, D02, D03, D05, D06 are the
    complete, final formal set.
  - **Condition D formal n=5 batch: COMPLETE.** Full cross-trial summary,
    comparison against A/B/C, and the explicitly-listed D04 exclusion:
    `objective5_impairment_matrix_v1_condition_D_formal_batch_summary.{json,md}`.
    Evidence per trial:
    `objective5_impairment_matrix_v1_condition_D_trial0{1..6}_attempt01_analysis/`.
- **Condition E independent-loss formal batch: `FINAL_BATCH_PASS` (5/5).**
  Five Webots trials used independent Bernoulli loss (`drop_probability=0.15`), zero delay/jitter, and fixed seeds. All datasets were `VALID`; all tasks were `SUCCESS`; all safety margins were positive (tightest 4.734mm); all first triggers were `PREDICTED_CPA` with `LOCAL_*` zero. Relay-authoritative mean drop fractions were 0.151646 (epuck1→epuck2) and 0.148138 (epuck2→epuck1). Total loss is taken from relay received/forwarded/drop counters; sequence gaps are explicitly boundary-censored and are not treated as total loss. Trial 01 was manually confirmed; Trials 02–05 were authorized automated continuations. Raw evidence: 70/70 files SHA-256 matched, 0 mismatch. Summary: `objective5_condition_E_formal_batch_summary.{json,md}`; per-trial evidence: `objective5_impairment_matrix_v1_condition_E_trial0{1..5}_attempt01_analysis/`. Execution commit: `540ad98cbeb3bbf79c3782ec6fe349d071d6f19a`.
- Registry rows:
  `objective5_matrix_v1_conditionA_exclusionary_pilot01`,
  `objective5_matrix_v1_conditionF_exclusionary_pilot01`,
  `objective5_matrix_v1_conditionF_exclusionary_pilot02`,
  `objective5_matrix_v1_conditionF_exclusionary_pilot03`,
  `objective5_impairment_matrix_v1_condition_A_trial01_attempt01`,
  `objective5_impairment_matrix_v1_condition_A_trial02_attempt01`,
  `objective5_impairment_matrix_v1_condition_A_trial03_attempt01`,
  `objective5_impairment_matrix_v1_condition_A_trial04_attempt01`,
  `objective5_impairment_matrix_v1_condition_A_trial05_attempt01`,
  `objective5_impairment_matrix_v1_condition_B_trial01_attempt01`,
  `objective5_impairment_matrix_v1_condition_B_trial02_attempt01`,
  `objective5_impairment_matrix_v1_condition_B_trial03_attempt01`,
  `objective5_impairment_matrix_v1_condition_B_trial04_attempt01`,
  `objective5_impairment_matrix_v1_condition_B_trial05_attempt01`,
  `objective5_impairment_matrix_v1_condition_C_trial01_attempt01`,
  `objective5_impairment_matrix_v1_condition_C_trial02_attempt01`,   `objective5_impairment_matrix_v1_condition_C_trial03_attempt01`,
   `objective5_impairment_matrix_v1_condition_C_trial04_attempt01`,
   `objective5_impairment_matrix_v1_condition_C_trial05_attempt01`,
   `objective5_impairment_matrix_v1_condition_E_trial01_attempt01`,
   `objective5_impairment_matrix_v1_condition_E_trial02_attempt01`,
   `objective5_impairment_matrix_v1_condition_E_trial03_attempt01`,
   `objective5_impairment_matrix_v1_condition_E_trial04_attempt01`,
   `objective5_impairment_matrix_v1_condition_E_trial05_attempt01`,
   `objective5_condition_E_formal_batch_20260721`.

### 06_physical_pipuck
Two physical e-puck2/Pi-puck units, Wi-Fi validation, disconnect/recovery,
physical avoidance demo. **First formal physical result now exists**
(see below); the rest of this category remains diagnostic/not-started.
- `experiments/06_physical_pipuck/single_device_bringup/physical_single_device_zero_impairment_baseline_v1_batch/` — **`physical_single_device_zero_impairment_baseline_v1`, FINAL_BATCH_PASS (5/5 FINAL_PASS)**. This is the first *formal* physical-hardware result of the whole project (everything physical before this — bridge/driver bringup, `physical_single_device_transport_diagnostic_pilot01_attempt01`, `physical_expanded_bridge_epuckstate_integration_pilot01_attempt01` — is `DIAGNOSTIC_PHYSICAL`, not formal). Stationary (no ground motion, no controller), single e-puck2 (#5809), expanded Pi-TCP-WSL bridge + `EpuckState.msg` protocol. Five trials (`trial01_attempt02`, `trial02_attempt02`, `trial03_attempt01`, `trial04_attempt01`, `trial05_attempt01`), each its own `physical_single_device_zero_impairment_baseline_v1_trial0N_attemptNN_analysis/` directory (`final_verdict.json`, `final_summary.md`, `runtime_manifest.json`, `pi_system_metrics_window.csv`) — all 5 are one continuous driver/Pi-expanded-server/WSL-bridge session (NOT 5 independent cold starts); only `state_publisher` was FRESH-restarted per trial, everything else REUSED. Each trial's real, measured, 4-source-overlap (rosbag + WSL bridge-status CSV + WSL system-metrics CSV + batch-level Pi system-metrics CSV) window spans 309.9-311.1s, yielding a centered 240.000s main window with >=34.9s buffer on both sides.
  - **Scope note (read before citing)**: Tier A (`APPLICATION_STATE_SEQUENCE_DELIVERY_RATIO`, 1.0 in all 5 trials, 0 missing/out_of_order) is a trial-start-vs-trial-end snapshot delta of the WSL bridge's own cumulative counters — Pi-application-level state-sequence receipt completeness, NOT IP/TCP packet loss. `duplicate_count` is `NOT_MEASURABLE` in every trial (the bridge's own code doesn't separately track duplicates from generic out-of-order arrivals) and is never reported as 0. Tier B (EpuckState bag capture) ratio is 1.0 in all 5 trials at an actual measured rate of ~8.88-8.94 Hz (not any configured/nominal rate). Tier C (raw sensor topics `/odom /scan /tof /ps0-7`) run ~9.2 Hz each, 0 stall; no PDR is claimed there (no source-side sequence exists). RTT (1Hz `/epuck_bridge/status` snapshot, explicitly not a full transaction census) shows a recurring tail — roughly 20-25% of samples >50ms, the same ~20-25% >100ms, 0% >200ms — consistently across all 5 separate measurement windows; this is reported as an observed repeatable pattern only, with **no root-cause attribution**. One-way Pi-to-WSL latency is **NOT reported/NOT measured** anywhere in this batch — no NTP/chrony clock-sync procedure has been verified between the Pi and WSL. `trial01_attempt01_short_window` and `trial02_attempt01_short_window` are **excluded** diagnostic evidence (a window-timing defect in an earlier orchestrator version, `run_baseline_v1_trial.sh`, since fixed by `run_baseline_v1_trial_v2.sh`) — not part of this n=5, and their raw data lives only outside the git tree (native WSL path `/home/eamon/epuck_comm_bags/`), not under a committed analysis directory.
  - Code: orchestrator `experiments/06_physical_pipuck/single_device_bringup/tools/run_baseline_v1_trial_v2.sh`, Tier-A delta computation `tools/compute_tier_a_delta.py`, final 4-source analysis `tools/run_final_trial_analysis.py`, Pi-metrics window slicing `tools/slice_pi_metrics.py`. Relevant recent work (see git log for exact hashes): the Tier-A snapshot-delta fix, a JSON-parsing fix for `ros2 topic echo`'s trailing `---` YAML marker, and the final 4-source analysis addition.
  - Raw data (rosbag `.db3`, raw CSVs, orchestrator logs, the batch-level Pi raw CSV) lives outside the git repo at native WSL path `/home/eamon/epuck_comm_bags/` — only the derived batch/trial evidence (verdict JSON, summaries, manifests, the per-trial sliced Pi-metrics CSV, SHA256SUMS) is git-tracked.
- Registry rows: `physical_single_device_zero_impairment_baseline_v1`, `physical_single_device_zero_impairment_baseline_v1_short_window_excluded`

### 07_reality_gap
Simulation-vs-physical PDR/latency/coordination-efficiency/failure-mode
comparison. **Not started** (depends on 06).

### 08_paper_ready_outputs
Currently empty — nothing has cleared the bar for "directly citable in the
dissertation with no further caveats" yet. Phase 4's 5/5 formal batch and
Phase 1-3's formal batches ARE citable but with the explicit limitations
recorded in `project_status.json`'s `known_limitations` and this index.
When summary tables/figures are produced for the dissertation, they belong
here as generated CSVs/figures that LINK to their evidence path — never as
copies of the original bags.

### 09_legacy_and_excluded
Old message format, failed pilots, artifact-contaminated data, explicitly
excluded runs. Nothing here is deleted.
- 5 pre-protocol-freeze bags in `cooperative_avoidance_20260716/`
  (`head_on_cpa_only_trial_02..06_postfix`) — cannot be reprocessed by
  current analyzers (`Fast CDR` deserialization exception against the
  current `EpuckState` message shape); original contemporaneous analysis
  outputs remain valid historical evidence
- `experiments/3-3.全传感器避障实验/bags/*_combined_formal_trial01_INCOMPLETE_no_controller_log/` — first Trial 01 attempt, controller log was never captured (manual-command redirection oversight), preserved not deleted, excluded from the formal batch
- Various `cooperative_avoidance_20260716` diagnostic/invalid/interrupted/timeout runs — see `cooperative_avoidance_20260716_diagnostics_and_invalid` registry row and the experiment's own index doc
- `communication_baseline_20260716/` — registry row `communication_baseline_20260716_stub`; contains exactly one file, an empty (0-byte) stub, no real experiment ever ran here (`artifact_missing`); unrelated to the current Objective 5 comm-baseline work despite the similar name

### 10_cooperative_exit_navigation_20260720 (Stage 0 retained; Stage 1 formal paired batch complete)

Supervisor-requested new task-level study, added 2026-07-20, scope
narrowed by the supervisor a second time (2026-07-20) to N2-only with a
real edge/corner exit and asymmetric exit-discovery information. Does
NOT modify, delete, or reinterpret any Objective 5 Condition A-D
evidence (frozen, unchanged). Full evidence chain is indexed in
`STAGE_CLASSIFICATION.md` — read that file first.

**Stage 0 — preparatory shared-goal mechanism validation (COMPLETE,
`PREPARATORY / EXCLUSIONARY / NOT_INCLUDED_IN_FORMAL_STATISTICS`)**:
the original central-rendezvous scenario. Not a research result, not
deleted, not rerun — it validated `task_completion_monitor.py`, the
goal-hold judgment logic, the `TASK_COMPLETE_GOAL`-replaces-
`max_runtime_s` stop path, and the full OFF/ON record+analyze chain, and
surfaced/fixed two real defects (start-pose-inside-goal-region; a
cmd_vel-verification DDS-teardown race). 8 pilot attempts total, 2 of
which (`N2_COMM_OFF_EXCLUSIONARY_PILOT06`, `N2_COMM_ON_EXCLUSIONARY_
PILOT02`) reached genuine, non-degenerate `TASK_OUTCOME=SUCCESS` — see
`STAGE_CLASSIFICATION.md`'s pilot inventory table for all 8 with
disposition and evidence links. Phase 1 (read-only architecture audit,
`architecture_audit_multi_robot_20260720.md`) and the Phase 2 design/
tooling that Stage 0 built on remain valid and are reused by Stage 1
(`task_completion_analyzer.py`, `multi_peer_risk.py`/
`multi_peer_extension_design_20260720.md` design for a future N3/N4,
not yet needed).

**Stage 1 — two-robot shared edge-exit study (FORMAL BATCH COMPLETE, `FINAL_BATCH_PASS`)**:
a real east-wall opening, asymmetric exit knowledge, and a deterministic Robot-B search route were held identical across five paired `N2_EXIT_COMM_OFF`/`N2_EXIT_COMM_ON` trials. All 10 runs completed successfully, settled in their assigned parking regions, and had zero observed collisions. Robot B's mean completion time fell from 94.184 +/- 3.029s to 88.184 +/- 2.298s. The mean paired makespan saving was 6.000 +/- 1.709s (6.345%), and all 5/5 pairs improved. These are descriptive n=5 simulation results, not broad statistical generalisation.

Mechanism audit passed in all five communication-enabled trials: Robot A physically entered/discovered the exit, then transmitted its first `GoalAnnouncement`, then Robot B changed from deterministic search to direct exit navigation. No premature switch was found. Authoritative derived evidence: `shared_exit_formal_batch_summary/batch_summary.json` and `shared_exit_formal_batch_summary/summary.md`. Raw evidence remains at `/home/eamon/epuck_comm_bags/` with a gitignored Windows copy at `bags/shared_exit_formal_20260721/`; all 175/175 copied files matched by SHA-256. Frozen execution commit: `049dcc496de7fd7a1c881eff221c701eef2cc564`.

Stage 1 is complete and must not be rerun merely to improve the result. A future N3/N4 extension remains separate and not started; it requires explicit authorization and the already documented multi-peer design work.

## Known open/unconfirmed items from this indexing pass

- `experiments/communication_baseline_20260716/` — purpose/contents not investigated this pass beyond confirming it has no `bags/` subdirectory. Needs a follow-up read.
- `fusion_static_neighbor_20260716/`'s Phase 1 trial count: directories on disk are `local_static_long_trial_02..07` (6 dirs), but `local_static_locked_batch_03_07_summary.md`'s name suggests trials 03-07 (5). This naming discrepancy was flagged, not resolved, during this indexing pass.
- The 53-bag `cooperative_avoidance_20260716/` and 2-bag `controller_v2_local_latch_20260717` / 1-bag `controller_v3_unified_encounter_20260717` directories were categorized from directory names and prior-session knowledge, not by freshly re-reading every individual `summary.json` this pass.

## Full raw inventory (for reference)

- 6 experiment directories under `experiments/` (bag directory count and valid-trial count are tracked separately, never conflated: 84 bag directories exist across 5 of the 6 directories -- `communication_baseline_20260716` has none, just one empty stub file; not every bag directory is a valid/complete trial, e.g. `combined_formal_trial01_INCOMPLETE_no_controller_log` has a valid `metadata.yaml` but no analysis output).
- Formal trials registered against already-existing, previously-generated batch summaries (Phase 1: 5, per `local_static_locked_batch_03_07_summary.md`'s filename, though the exact count is flagged OPEN against the 6 directories on disk; Phase 2/3: 45, per `cooperative_avoidance_experiment_index_20260716.md`; Phase 4: 5, individually read and SEALED this session) -- these counts come from reading existing batch-summary documents, NOT from re-verifying all 84 bag directories individually during any indexing pass.
- 38 git commits total as of the previous indexing pass's correction (`git rev-list --count HEAD` = 38, HEAD = `2558216`); this number drifts every commit -- always re-run `git rev-list --count HEAD` rather than trusting a number written in a document.
- See `path_manifest.csv` for the complete path-by-path breakdown with Windows/WSL forms, and `experiment_registry.csv`'s `verification_status`/`verification_basis` columns for what evidence backs each row.

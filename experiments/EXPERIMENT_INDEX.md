# Experiment Index

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
- `experiments/controller_v2_local_latch_20260717/` — controller_v2 pilot_a/pilot_a2
- `experiments/controller_v3_unified_encounter_20260717/` — controller_v3 pilot_a3 (forensic evidence that motivated v4)
- `experiments/controller_v4_full_sensor_bypass_20260717/bags/*_static_box_{a,b,c,d}` — pilot_v4_a attempts 1-4 (3 fails, 1 pass)
- `experiments/controller_v4_full_sensor_bypass_20260717/bags/*_static_box_fusion_{a,b2,b3}` — pilot_v4_b/b2/b3 (fail, partial, pass)
- `experiments/cooperative_avoidance_20260716/bags/combined_*` — the controller_v1 combined-scenario box-corner-runaway-turn defect that started the whole v1→v4 chain (see `combined_wood_moving_peer_README.md`'s 2026-07-17 entry)
- Registry rows: `controller_v2_*`, `controller_v3_*`, `v4_pilot_*`, `combined_wood_moving_peer_v1_defect_pilots`, `combined_wood_moving_peer_postfix_pilots`

### 03_phase4_task_validation
Static wooden block, pure CPA, combined avoidance — Objective 4's
task-specific validation vehicle.
- **Phase 1** (controller_v1): `experiments/fusion_static_neighbor_20260716/` — 5/5 formal (`local_static_long_trial_02..07`), 1 diagnostic (`local_static_trial_01`)
- **Phase 2/3** (controller_v1): `experiments/cooperative_avoidance_20260716/` — 45 formal trials across 9 batches (head-on centered/offset/crossing/ablation local-only/ablation fused), see `cooperative_avoidance_experiment_index_20260716.md` for the authoritative per-trial table
- **Phase 4** (controller_v4, **SEALED**): `experiments/controller_v4_full_sensor_bypass_20260717/bags/*_combined_formal_trial0{1..5}` — **5/5 PASS**, manifest commit `e32560e`, batch summary commit `cbb1897`
  - **Binding limitation**: all 5/5 trials triggered via `PROXIMITY_FALLBACK`, none via `PREDICTED_CPA`. Naming rule: "staged local-obstacle avoidance followed by communication-assisted proximity/cooperative avoidance." Never describe this batch as preventing an otherwise-certain head-on collision.
  - Exclusionary pilots (not pooled with the formal 5/5): `head_on_cpa_pure_c1`, `combined_trial1`, `combined_trial2_timebasefix`
- Registry rows: `phase1_*`, `phase2_3_*`, `v4_pure_cpa_*`, `v4_combined_pilot_*`, `v4_phase4_formal_trial0{1..5}`, `phase4_formal_batch_5of5`

### 04_objective5_comm_baseline
Zero-delay/zero-loss communication baseline. Two formal results now
exist (latency-partial + latency-complete); the rest of this category
remains diagnostic-only.
- `experiments/controller_v4_full_sensor_bypass_20260717/bags/*_comm_baseline_trial{1,2,3}` — FAIL, `/mnt/c` rosbag-write message loss (~40-55%), superseded by the native-path diagnostic
- `experiments/controller_v4_full_sensor_bypass_20260717/bags/*_comm_baseline_native_trial0{1,2}` — PASS (comm-layer-only, no `cooperative_avoider`), aligned-window PDR=1.0, confirms root cause is `/mnt/c` I/O not the relay/transport
- `experiments/controller_v4_full_sensor_bypass_20260717/bags/*_objective5_comm_baseline_zero_impairment_formal_trial01` — **PASS, FORMAL_SIM.** Genuine `cooperative_avoider` task completion under zero impairment, native WSL ext4 bag path. Aligned-window PDR=1.0 both robots; `sequence_gap_count`=`duplicate_count`=`out_of_order_count`=0 both robots; `sequence_counter` `complete=true` both robots; realtime factor 0.963 (preload) / 0.951 (full load); message rate ≈8.69 Hz/robot; mean bandwidth ≈696 bytes/s/robot; no bag/QoS drop-warn-error lines. **metric_coverage: PDR=VALID sequence_integrity=VALID throughput=VALID task_behavior=VALID latency=NOT_MEASURED.** Message age/latency is N/A, permanently (not re-analyzed/backfilled) — root cause was `analyze_comm_performance.py` mixing rosbag2's own wall-clock recording timestamp with `message.stamp` (sim time), NOT an unset stamp field (`state_publisher.py` sets it correctly). The first 3 attempts at this exact trial failed for orchestration-script reasons (WSL interop, then two distinct process-shutdown bugs in `run_objective5_comm_baseline_formal_trial.sh`), not communication-result reasons — kept, not deleted, in that experiment directory's README execution_attempts table.
- `experiments/controller_v4_full_sensor_bypass_20260717/bags/*_objective5_timestamp_latency_validation_pilot01_condition_{a_delay0,b_delay025}` — PASS, PILOT (diagnostic-only, no `cooperative_avoider`). Validates the stamp/latency measurement chain before trial02_stamp: condition_a (delay=0) mean age ~microsecond-scale; condition_b (delay=0.25s) observed increment ~0.26s vs configured 0.25s, error ~0.01s. Never pooled with formal statistics.
- `experiments/controller_v4_full_sensor_bypass_20260717/bags/*_objective5_comm_baseline_zero_impairment_formal_trial02_stamp` — **PASS, FORMAL_SIM.** Latency-complete companion to trial01 under `protocol_v1.1_stamp_semantics` (wire schema unchanged, still `PROTOCOL_VERSION=1`): `state_publisher.py` now gates publication on a valid ROS clock (`WAITING_FOR_CLOCK`), `sequence_counter.py` now computes live per-message latency (message.stamp vs its own receipt time, same clock domain). Same PDR/sequence/task results as trial01 (aligned-window PDR=1.0, 0 gaps/dup/oo, `complete=true`, realtime factor 0.996/1.003) plus **metric_coverage all five VALID**: live-counter mean/median/p95/max `message_age_s`=0.0 both robots (zero configured delay). **Does not replace trial01** — both remain separately registered with different metric_coverage.
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
  `objective5_impairment_matrix_v1_condition_B_trial05_attempt01`.

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
- `experiments/controller_v4_full_sensor_bypass_20260717/bags/*_combined_formal_trial01_INCOMPLETE_no_controller_log/` — first Trial 01 attempt, controller log was never captured (manual-command redirection oversight), preserved not deleted, excluded from the formal batch
- Various `cooperative_avoidance_20260716` diagnostic/invalid/interrupted/timeout runs — see `cooperative_avoidance_20260716_diagnostics_and_invalid` registry row and the experiment's own index doc
- `communication_baseline_20260716/` — registry row `communication_baseline_20260716_stub`; contains exactly one file, an empty (0-byte) stub, no real experiment ever ran here (`artifact_missing`); unrelated to the current Objective 5 comm-baseline work despite the similar name

## Known open/unconfirmed items from this indexing pass

- `experiments/communication_baseline_20260716/` — purpose/contents not investigated this pass beyond confirming it has no `bags/` subdirectory. Needs a follow-up read.
- `fusion_static_neighbor_20260716/`'s Phase 1 trial count: directories on disk are `local_static_long_trial_02..07` (6 dirs), but `local_static_locked_batch_03_07_summary.md`'s name suggests trials 03-07 (5). This naming discrepancy was flagged, not resolved, during this indexing pass.
- The 53-bag `cooperative_avoidance_20260716/` and 2-bag `controller_v2_local_latch_20260717` / 1-bag `controller_v3_unified_encounter_20260717` directories were categorized from directory names and prior-session knowledge, not by freshly re-reading every individual `summary.json` this pass.

## Full raw inventory (for reference)

- 6 experiment directories under `experiments/` (bag directory count and valid-trial count are tracked separately, never conflated: 84 bag directories exist across 5 of the 6 directories -- `communication_baseline_20260716` has none, just one empty stub file; not every bag directory is a valid/complete trial, e.g. `combined_formal_trial01_INCOMPLETE_no_controller_log` has a valid `metadata.yaml` but no analysis output).
- Formal trials registered against already-existing, previously-generated batch summaries (Phase 1: 5, per `local_static_locked_batch_03_07_summary.md`'s filename, though the exact count is flagged OPEN against the 6 directories on disk; Phase 2/3: 45, per `cooperative_avoidance_experiment_index_20260716.md`; Phase 4: 5, individually read and SEALED this session) -- these counts come from reading existing batch-summary documents, NOT from re-verifying all 84 bag directories individually during any indexing pass.
- 38 git commits total as of the previous indexing pass's correction (`git rev-list --count HEAD` = 38, HEAD = `2558216`); this number drifts every commit -- always re-run `git rev-list --count HEAD` rather than trusting a number written in a document.
- See `path_manifest.csv` for the complete path-by-path breakdown with Windows/WSL forms, and `experiment_registry.csv`'s `verification_status`/`verification_basis` columns for what evidence backs each row.

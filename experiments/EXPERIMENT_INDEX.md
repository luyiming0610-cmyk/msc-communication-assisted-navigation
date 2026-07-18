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
Zero-delay/zero-loss communication baseline. **Currently all diagnostic,
none formal yet.**
- `experiments/controller_v4_full_sensor_bypass_20260717/bags/*_comm_baseline_trial{1,2,3}` — FAIL, `/mnt/c` rosbag-write message loss (~40-55%), superseded by the native-path diagnostic
- `experiments/controller_v4_full_sensor_bypass_20260717/bags/*_comm_baseline_native_trial0{1,2}` — PASS (comm-layer-only, no `cooperative_avoider`), aligned-window PDR=1.0, confirms root cause is `/mnt/c` I/O not the relay/transport
- `src/epuck2_comm/epuck2_comm/{analyze_comm_performance,network_impairment,network_impairment_relay,sequence_counter}.py` — the tooling itself, 39 unit tests, all passing
- **Not yet done**: a full formal zero-impairment baseline with `cooperative_avoider` actually running, using the native-WSL-path workflow
- Registry rows: `comm_baseline_v1_trial{1,2,3}`, `comm_baseline_native_diag_trial0{1,2}`

### 05_objective5_impairment_matrix
Delay, loss, combined-impairment experiments. **Not started.** Blocked on
04 completing a full formal baseline first (see `project_status.json`'s
`blocked_items`).

### 06_physical_pipuck
Two physical e-puck2/Pi-puck units, Wi-Fi validation, disconnect/recovery,
physical avoidance demo. **Not started.**

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

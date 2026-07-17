# cooperative_avoidance_20260716

**Purpose**: Phase 2/3 formal task-validation batches (head-on/offset/
crossing/ablation CPA scenarios) under `controller_v1`, plus the combined
wooden-box + moving-peer scenario that surfaced the `controller_v1`
box-corner runaway-turn safety defect motivating the v2→v3→v4 controller
chain.

**Scenarios and versions covered** (53 bag directories total):
- `head_on_centered_realtime_formal_trial_0{1..5}`, `head_on_offset_040_realtime_formal_trial_0{1..5}`, `crossing_90_realtime_formal_trial_0{1..5}`, `ablation_local_only_head_on_realtime_formal_trial_0{1..5}`, `ablation_fused_head_on_realtime_formal_trial_0{1..5}` — **Phase 2/3 formal batches, 45 trials total, all controller_v1** (03_phase4_task_validation)
- `head_on_cpa_only_trial_01`, `head_on_trial_01`, `head_on_trial_02_recovery`, and various `_interrupted`/`_timeout`/`_invalid`/`_diagnostic` named directories — excluded diagnostics (09_legacy_and_excluded)
- `head_on_cpa_only_trial_02..06_postfix` — **pre-protocol-freeze bags, cannot be reprocessed by current analyzers** (old `EpuckState` wire shape); original analysis outputs remain valid (09_legacy_and_excluded)
- `combined_local_then_peer_realtime_trial_pilot_0{1..4}`, `combined_wood_moving_peer_realtime_trial_01`, `combined_wood_moving_peer_realtime_trial_pilot_0{1,2}` — controller_v1 combined-scenario defect evidence (02_controller_regression)
- `combined_wood_moving_peer_postfix_realtime_trial_pilot_fix_0{1..3}` — postfix attempts, superseded by controller_v4 (02_controller_regression)

**Config**: `config/head_on_centered_realtime/`, `config/head_on_lateral_offset_040/`, `config/crossing_90/`, `config/ablation_head_on_local_only/`, `config/ablation_head_on_fused/`, `config/head_on_cpa_only/`, `config/combined_wood_moving_peer/`

**PASS/FAIL criteria and actual results**: see
`cooperative_avoidance_experiment_index_20260716.md` (per-trial table,
authoritative) and `simulation_rate_integrity_audit_20260716.md` (realtime
factor gate; documents the earlier accelerated-Webots-factor issue and its
0.8-1.2 controlled-realtime fix, plus a 2026-07-17 addendum confirming the
`controller_v4_ros_time_consistency` fix and the discovered
`self.started_at` clock-init race — since fixed).

**Included in dissertation**: the 45 Phase 2/3 formal trials YES (see
`master_experiment_plan_20260716.md`'s Phase 2/3 sections — controller_v1
era, still valid because these protocols' runtimes never reached the
extended-tail window where the v1 defect manifests). The combined-scenario
`controller_v1` evidence is defect/regression evidence, not formal Phase 4
statistics — Phase 4's formal statistics live in
`controller_v4_full_sensor_bypass_20260717/` instead.

**Unique variable changed vs. predecessor**: this directory predates
`controller_v4`; see `combined_wood_moving_peer_README.md`'s 2026-07-17
entry for the full box-corner-runaway-turn root-cause account that started
the v2→v3→v4 chain.

**Git commits**: pre-dates this session's commit history in most cases
(baseline commit `e76adf3`, `controller_v1`); see per-trial detail in
`experiment_registry.csv`.

**Note**: per-bag verdicts in `experiment_registry.csv` for this directory
are aggregated/summarized from directory naming conventions and this
session's prior-context knowledge, not freshly re-verified against every
individual `summary.json` — flagged as a known limitation in
`project_status.json`.

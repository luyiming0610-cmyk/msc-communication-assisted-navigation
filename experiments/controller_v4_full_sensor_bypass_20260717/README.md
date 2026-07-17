# controller_v4_full_sensor_bypass_20260717

**Purpose**: development, regression, and Phase 4 task-validation evidence
for `controller_v4` (culminating in `controller_v4_timebase_fix_20260717`,
now frozen), plus Objective 5's communication-performance analyzer/relay
implementation and baseline diagnostics.

**Scenarios and versions covered**:
- `static_box_{a,b,c,d}`, `static_box_fusion_{a,b2,b3}` — controller_v4 development pilots (02_controller_regression)
- `head_on_cpa_pure_c1` — pure dual-robot CPA exclusionary pilot (03_phase4_task_validation)
- `combined_trial1`, `combined_trial2_timebasefix` — combined-scenario exclusionary pilots (03_phase4_task_validation)
- `combined_formal_trial01..05` — **Phase 4 formal batch, SEALED, 5/5 PASS** (03_phase4_task_validation)
- `combined_formal_trial01_INCOMPLETE_no_controller_log` — excluded first attempt (09_legacy_and_excluded)
- `comm_baseline_trial{1,2,3}` — Objective 5 diagnostic, `/mnt/c` bag-loss issue (04_objective5_comm_baseline)
- `comm_baseline_native_trial0{1,2}` — Objective 5 diagnostic, native-WSL-path root-cause isolation (04_objective5_comm_baseline)

**Config**: `config/static_box_v4/`, `config/head_on_cpa_v4/`, `config/combined_v4/`, `config/comm_baseline_v1/`

**PASS/FAIL criteria**: see each pilot's own `analysis/*_verdict.json`. For
the sealed formal batch specifically, see
`PHASE4_FORMAL_EVIDENCE_MANIFEST_20260717.md`.

**Actual results**: Phase 4 formal batch 5/5 PASS (see
`PHASE4_FORMAL_BATCH_SUMMARY_20260717.md`). All 5/5 trials triggered via
`PROXIMITY_FALLBACK`, not `PREDICTED_CPA` — see that summary's binding
naming rule. Objective 5 comm-baseline diagnostics found and traced a
`/mnt/c` rosbag-write message-loss issue (see
`config/comm_baseline_v1/analyze_measurement_chain.py` and the two
`comm_baseline_native_trial0{1,2}` diagnostic trials, both PASS with
aligned-window PDR=1.0).

**Included in dissertation**: Phase 4 formal batch YES (with the
`PROXIMITY_FALLBACK` limitation noted). Everything else in this directory
is development/diagnostic evidence, NOT formal statistics.

**How to reproduce**: each pilot config directory's `run_*.sh` script is
self-contained (sources ROS2 Humble + the `epuck_ws` workspace, launches
Webots via `simulation_comm_experiment_v1/working/run_*.py`, records a
rosbag, runs its analyzer). See `PHASE4_FORMAL_EVIDENCE_MANIFEST_20260717.md`
for the exact frozen configuration fingerprint (file hashes, geometry,
parameters) used for the sealed batch.

**Git commits**: see `experiments/experiment_registry.csv` for the
per-trial commit hashes; the controller itself was frozen at `980e7d0`.

**Details**: `HANDOFF_20260717.md` in this directory has the full
controller v1→v4 technical narrative and open findings.

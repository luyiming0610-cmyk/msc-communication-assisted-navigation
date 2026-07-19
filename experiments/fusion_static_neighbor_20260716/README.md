# fusion_static_neighbor_20260716

**Purpose**: Phase 1 formal task-validation batch — static wooden obstacle
with a stationary communicated neighbour, under `controller_v1`.

**Scenario and version**: centred 0.06m square box, identical initial
poses, periodic 10Hz state policy, calibrated local avoidance enabled,
`epuck1` runtime 14s, `epuck2` stationary. `controller_v1`.

**Trials** (7 bag directories):
- `local_static_trial_01` — diagnostic oscillation/failure case (09_legacy_and_excluded)
- `local_static_long_trial_02` — retained as functional evidence but excluded from short-window path statistics (later arena-boundary avoidance was recorded)
- `local_static_long_trial_03..07` — **formal locked-condition batch** (03_phase4_task_validation)

**PASS/FAIL criteria and actual results**: see
`local_static_locked_batch_03_07_summary.md` and
`../master_experiment_plan_20260716.md`'s Phase 1 section. Gate: 5/5
locked-condition trials, no collision, obstacle passed, stationary-peer
displacement zero, final command zero, no invalid state messages.

**Included in dissertation**: YES — Phase 1 formal batch, controller_v1
era, still valid (Phase 1's short runtime never reached the extended-tail
window where the later-discovered `controller_v1` defect manifests).

**Note**: directory naming (`local_static_long_trial_02..07`, 6 dirs) vs.
the batch summary document's name (`..._batch_03_07_summary.md`, implying
5 trials 03-07) has a discrepancy not resolved during this indexing pass —
flagged in `project_status.json`'s `known_limitations`.

**Config**: no `config/` subdirectory in this experiment folder; the pilot
scripts that produced these bags live in
`2-1.仿真通信实验/working/` (outside the git repo) — see
`run_fusion_static_neighbor.py`, `run_fusion_static_long_course.py`.

**Git commit**: baseline `e76adf3` (`controller_v1`), predates this
session's detailed commit-by-commit history.

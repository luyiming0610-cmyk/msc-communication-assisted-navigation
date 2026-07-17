# Phase 4 formal batch — sealed evidence manifest

Date sealed: 2026-07-17. This manifest freezes and closes the Phase 4
formal batch (Trials 01-05, all PASS, git `cbb1897`). **No trial in this
batch may be modified, reprocessed with different parameters, or rerun.**
Any future Phase 4 work (e.g. a deliberately-engineered true-collision-
course variant) must use a new trial series and a new manifest, never
overwrite this one.

## Frozen configuration fingerprint (identical across all 5 trials)

| Item | Value |
|---|---|
| Controller version | `controller_v4_timebase_fix_20260717` |
| Controller-fix commit | `980e7d0` |
| Protocol | `EpuckState` `PROTOCOL_VERSION=1`, frozen at `b5a0351` (`src/epuck2_comm_interfaces/PROTOCOL_FREEZE_20260717.md`) |
| `EpuckState.msg` SHA-256 | `a7ec4184dec52b157a87beea20b44fb2dff5c6dee199d0c76b7c347c26abe15b` |
| World file | `simulation_comm_experiment_v1/working/combined_wood_moving_peer_world.wbt` |
| World file SHA-256 | `9ebf375c14e68786821bb3a9525d686fb2c6b584de7b59725ee0b77914d9f196` |
| Pilot run script | `experiments/controller_v4_full_sensor_bypass_20260717/config/combined_v4/run_combined_v4_pilot.sh` |
| Run script SHA-256 | `44907343af17f119003fc1eedcf96f82183e51aa873c7b224e6e5676351a7824` |
| Controllers launch script | `.../combined_v4/run_combined_v4_controllers.py` |
| Controllers launch SHA-256 | `5ec667d883c54ae9e5d0a362278485dfc27d19ee7955cfd22091083e2033b668` |
| Task coordinator | `.../combined_v4/combined_task_coordinator_v4.py` |
| Task coordinator SHA-256 | `931a5bab73a1b65614907afb1c85f9cfddb1b38e39b50d692ac5dfbf5bf310ae` |
| `cooperative_avoider.py` SHA-256 | `026130988670b3cb5f99e07d0ffe2bbf41f69e42dca13c81d4b204894a053aa2` |
| `local_obstacle_logic.py` SHA-256 | `859f25ceee5ded4efe7df33cc746b7d35d004631040214c9857d087c7724b981` |
| `epuck1` initial pose | `(-0.55, 0.0, 0.0)`, `startup_hold_s=5.0` |
| `epuck2` initial pose | `(0.45, 0.0, pi)`, `startup_hold_s=42.0` |
| `max_runtime_s` | `100.0` (both robots) |

Note: run script SHA-256 above was computed AFTER Trial 03's retry fix
(adding the `analyze_trigger_reason.py` pipeline step and `TIMEBASE_INIT`/
`TIMEBASE_RESET` verdict fields), which happened between Trial 01/02 and
Trial 03/04/05. This is an **analysis-only** change (confirmed: no
controller, world, pose, or CPA/local parameter was touched) and is
recorded transparently rather than glossed over -- Trials 01-02 have their
trigger-reason data in separate `formal_trial0{1,2}_verdict.json` files
(computed via the same `analyze_trigger_reason` module, just invoked
manually instead of from inside the pilot script), not because the
underlying measurement differs.

## Per-trial record

### Trial 01 -- manual, directly observed by the user

- Bag: `bags/controller_v4_full_sensor_bypass_20260717_combined_formal_trial01/`
- Git commit: `af73661`
- Observation: manual, step-by-step (5 separate terminals), user watched
  Webots throughout in real time
- Analysis: `analysis/summary.json`, `analysis/combined_task_summary.json`,
  `analysis/trigger_reason_summary.{json,md}`,
  `analysis/trigger_classification.csv`,
  `analysis/formal_trial01_verdict.json`
- Verdict: **PASS**
- Collision: No (min separation 0.2666 m)
- Box clearance: 0.1151 m
- `trigger_reason`: **PROXIMITY_FALLBACK** (`dcpa_at_trigger=0.1979m`)
- Realtime factor: not auto-measured (manual trial); user directly
  confirmed on-screen factor was within 0.8-1.2 (chat-recorded
  confirmation)
- First attempt (same name) failed to capture the controller log and was
  preserved, not deleted, as
  `bags/controller_v4_full_sensor_bypass_20260717_combined_formal_trial01_INCOMPLETE_no_controller_log/`
  -- excluded from this manifest's statistics

### Trial 02 -- automated

- Bag: `bags/controller_v4_full_sensor_bypass_20260717_combined_formal_trial02/`
- Git commit: `cee361d`
- Observation: automated (`run_combined_v4_pilot.sh`), unattended
- Analysis: `analysis/summary.json`, `analysis/combined_task_summary.json`,
  `analysis/trigger_reason_summary.{json,md}`,
  `analysis/trigger_classification.csv`,
  `analysis/static_v4_combined_verdict.json`,
  `analysis/formal_trial02_verdict.json` (supplementary, adds
  `trigger_reason`/`timebase` fields not yet in the script at this trial)
- Verdict: **PASS**
- Collision: No (min separation 0.2712 m)
- Box clearance: 0.1225 m
- `trigger_reason`: **PROXIMITY_FALLBACK** (`dcpa_at_trigger=0.1972m`)
- Realtime factor: preload 0.987, full-load 0.963

### Trial 03 -- automated

- Bag: `bags/controller_v4_full_sensor_bypass_20260717_combined_formal_trial03/`
- Git commit: `21cc69b`
- Observation: automated, unattended
- Analysis: `analysis/summary.json`, `analysis/combined_task_summary.json`,
  `analysis/trigger_reason_summary.{json,md}`,
  `analysis/trigger_classification.csv`,
  `analysis/static_v4_combined_verdict.json` (now includes
  `trigger_reason`/`timebase_init_count`/`timebase_reset_count` natively)
- Verdict: **PASS**
- Collision: No (min separation 0.2740 m)
- Box clearance: 0.1258 m
- `trigger_reason`: **PROXIMITY_FALLBACK** (`dcpa_at_trigger=0.2084m`)
- Realtime factor: preload 0.959, full-load 1.018
- `timebase_init_count=2`, `timebase_reset_count=0`

### Trial 04 -- automated

- Bag: `bags/controller_v4_full_sensor_bypass_20260717_combined_formal_trial04/`
- Git commit: `f10a367`
- Observation: automated, unattended
- Analysis: same set as Trial 03
- Verdict: **PASS**
- Collision: No (min separation 0.2777 m)
- Box clearance: 0.1237 m
- `trigger_reason`: **PROXIMITY_FALLBACK** (`dcpa_at_trigger=0.2131m`)
- Realtime factor: preload 1.002, full-load 0.965
- `timebase_init_count=2`, `timebase_reset_count=0`

### Trial 05 -- automated

- Bag: `bags/controller_v4_full_sensor_bypass_20260717_combined_formal_trial05/`
- Git commit: `20c4056`
- Observation: automated, unattended
- Analysis: same set as Trial 03
- Verdict: **PASS**
- Collision: No (min separation 0.2787 m)
- Box clearance: 0.1274 m
- `trigger_reason`: **PROXIMITY_FALLBACK** (`dcpa_at_trigger=0.2157m`)
- Realtime factor: preload 0.966, full-load 0.991
- `timebase_init_count=2`, `timebase_reset_count=0`

## Batch-wide statement (binding)

**All 5 of 5 trials triggered via `PROXIMITY_FALLBACK`
(`current_distance<0.34m`). None triggered via `PREDICTED_CPA`
(`tcpa<=4.0s and dcpa<0.14m`).** `dcpa_at_trigger` ranged 0.1972-0.2157m in
every trial, always comfortably above the 0.14m predicted-conflict
threshold. This batch therefore **must not be cited as PREDICTED_CPA
evidence or as proof that the robots' original trajectories were
necessarily on a collision course.** It is valid evidence that the frozen
controller correctly executes the two-stage "staged local-obstacle
avoidance followed by communication-assisted proximity/cooperative
avoidance" mechanism, safely and reproducibly, across 5 independent runs
with 0 collisions.

Batch statistics (n=5 for geometry, n=4 for auto-measured realtime factor):
mean minimum robot-robot separation 0.2736 ± 0.0044 m, mean box clearance
0.1229 m, mean `dcpa_at_trigger` 0.2065 m, mean preload factor 0.979, mean
full-load factor 0.984.

## Freeze declaration

Phase 4 is CLOSED as of this manifest. Trials 01-05's bags, logs, and
analysis outputs are final and must not be edited, regenerated with
different parameters, or deleted. Any bug found later in an analyzer script
must be documented as a new, dated finding with its own commit -- never by
silently editing this manifest's numbers or the underlying trial artifacts.

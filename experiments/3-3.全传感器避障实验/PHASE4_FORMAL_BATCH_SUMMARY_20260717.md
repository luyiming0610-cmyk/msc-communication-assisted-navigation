# Phase 4 formal batch — combined box + moving-peer scenario, 2026-07-17

Controller: `controller_v4_timebase_fix_20260717` (git `980e7d0`), frozen at
this batch's start. Protocol: `EpuckState` `PROTOCOL_VERSION=1`, frozen at
`b5a0351` (`src/epuck2_comm_interfaces/PROTOCOL_FREEZE_20260717.md`), no
field changed across the batch. Configuration: `combined_v4/` (world,
initial poses, `startup_hold_s=5/42`, all CPA/local parameters), identical
across all 5 trials and identical to the excluded `combined_trial2_timebasefix`
pilot that validated the timebase fix beforehand.

**Official naming for this scenario (do not use any other phrasing):**
"staged local-obstacle avoidance followed by communication-assisted
proximity/cooperative avoidance." Never "synchronized," never "triggered by
predicted CPA," never a claim that the two robots' original trajectories
were necessarily on a collision course.

## Trials

| Trial | Verdict | Collision | Min separation (m) | Box clearance (m) | trigger_reason | dcpa@trigger (m) | Preload / full-load factor |
|---|---|---|---:|---:|---|---:|---|
| 01 (manual, directly observed) | PASS | No | 0.2666 | 0.1151 | PROXIMITY_FALLBACK | 0.1979 | not auto-measured; user directly confirmed on-screen factor was in 0.8-1.2 |
| 02 (automated) | PASS | No | 0.2712 | 0.1225 | PROXIMITY_FALLBACK | 0.1972 | 0.987 / 0.963 |
| 03 (automated) | PASS | No | 0.2740 | 0.1258 | PROXIMITY_FALLBACK | 0.2084 | 0.959 / 1.018 |
| 04 (automated) | PASS | No | 0.2777 | 0.1237 | PROXIMITY_FALLBACK | 0.2131 | 1.002 / 0.965 |
| 05 (automated) | PASS | No | 0.2787 | 0.1274 | PROXIMITY_FALLBACK | 0.2157 | 0.966 / 0.991 |

**Result: 5/5 PASS, 0/5 collision, 0/5 box collision, 0/5 FAILSAFE, 0/5
SENSOR_INVALID, 0/5 TASK_TIMEOUT, 0/5 stopped_by_max_runtime_only, 0/5
TIMEBASE_RESET, 5/5 epuck2_local_count=0 (never mis-triggered local box
avoidance), 5/5 epuck1 genuinely passed the box before its first AVOID_TURN,
5/5 trigger_reason=PROXIMITY_FALLBACK.**

Batch statistics (n=5): mean minimum robot-robot separation
0.2736 ± 0.0044 m (range 0.2666-0.2787), mean box clearance 0.1229 m
(range 0.1151-0.1274), mean dcpa at trigger 0.2065 m (range 0.1972-0.2157,
every value comfortably above the 0.14 m predicted-conflict threshold).
Realtime factors (n=4, automated trials only): mean preload 0.979, mean
full-load 0.984, both individual trials and the batch mean within 0.8-1.2.

Evidence: `bags/controller_v4_full_sensor_bypass_20260717_combined_formal_trial0{1..5}/`,
each with `analysis/summary.json`, `analysis/combined_task_summary.json`,
`analysis/trigger_reason_summary.{json,md}`,
`analysis/trigger_classification.csv`, and either
`analysis/formal_trial0{1,2}_verdict.json` (Trials 01-02, built manually
before the trigger-reason step was folded into the automated script) or
`analysis/static_v4_combined_verdict.json` (Trials 02-05, produced directly
by `run_combined_v4_pilot.sh`, which now runs `analyze_trigger_reason.py`
as a standard step). Git commits: `af73661` (Trial 01), `cee361d` (Trial 02),
`21cc69b` (Trial 03), `f10a367` (Trial 04), `20c4056` (Trial 05).

## Shared batch-wide limitation (not scored against any individual trial)

All 5 trials, without exception, triggered via `PROXIMITY_FALLBACK`
(`current_distance<0.34m`), never `PREDICTED_CPA`
(`tcpa<=4.0s and dcpa<0.14m`). `dcpa_at_trigger` was 0.197-0.216m in every
trial -- well above the 0.14m predicted-conflict threshold, meaning that
under a constant-velocity extrapolation from the moment avoidance began, the
two robots were never actually headed for a collision. Root cause: `epuck1`'s
`LOCAL_RECOVER` after the box bypass restores heading only, not lateral
position (confirmed: `epuck1` finishes each trial at `y != 0`, e.g. Trial 01
`y=-0.224m`), so the two robots' paths after the box encounter are close but
not truly head-on, unlike the original symmetric `y=0` design intent
(inherited from the validated `head_on_centered` CPA baseline geometry).
This is a genuine, reproducible property of the frozen configuration, not a
per-trial anomaly, a controller defect, or something engineered to make the
batch pass more easily. It does not invalidate what this batch demonstrates
(genuine two-stage triggering: local box avoidance, then a real,
communication-driven cooperative maneuver, completed safely every time) but
it does mean this batch cannot be cited as evidence that the controller
prevents an otherwise-certain head-on collision. A future scenario with
deliberately engineered post-box collision geometry would be needed for that
specific claim, and is out of scope for this batch.

## Gate status

Per `master_experiment_plan_20260716.md`'s Phase 4 definition (n=5
canonical geometry), this gate is now met: 5/5 accepted formal repetitions,
0/5 collision. Phase 4 is complete. See that file's corresponding update for
the full gate record and what remains (Phase 5 communication policy, Phase 6
physical/HIL) -- though per the current project re-prioritization (this
session's route re-alignment against the official COMP5200M Spec/SP),
further avoidance-scenario work is intentionally paused in favor of the
communication-library core objectives.

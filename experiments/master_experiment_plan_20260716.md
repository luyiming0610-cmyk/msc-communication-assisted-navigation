# e-puck2 MSc Experimental Plan — Locked Sequence

Date: 2026-07-16

## Design principle

Validate one factor at a time before combining moving robots, local obstacles and
communication faults. Diagnostic trials remain available as failure cases but are
not pooled with post-fix trials that use different controllers or record windows.

## Phase 1 — Static wooden obstacle with stationary communicated neighbour

Locked condition: centred 0.06 m square box, identical initial poses, periodic
10 Hz state policy, calibrated local avoidance enabled, `epuck1` runtime 14 s,
and `epuck2` stationary.

- Comparable batch: Trials 03–07, target `n=5`.
- Current status: Trials 03–07 passed; locked-condition pilot complete (`5/5`).
- Trial 01: retained as a diagnostic oscillation/failure case.
- Trial 02: retained as functional evidence but excluded from short-window path
  statistics because the later arena-boundary avoidance was recorded.

Gate to Phase 2: all five locked-condition trials complete, no collision, obstacle
passed, stationary-peer displacement zero, final command zero and no invalid state
messages. Any failure is analyzed before parameters are changed or the phase is
advanced.

Batch statistics: success rate, collision rate, minimum clearance, minimum front
range, path length, path efficiency, maximum lateral deviation, task/motion time,
angular sign changes, and final-stop integrity. Report mean, standard deviation,
median and range; confidence intervals are added for the final larger batch.

## Phase 2 — Moving two-robot communication/CPA baseline without a wooden box

Use two simulated e-puck2 robots. The physical project still has only one real
e-puck2, so this phase must not be described as a two-physical-robot experiment.

Pilot scenarios, target `n=5` each:

1. Centred head-on encounter.
2. Head-on encounter with lateral offset.
3. Ninety-degree crossing paths.

Current status: the clean centred head-on post-fix batch is complete. Diagnostic
Trial 01 is excluded; Trials 02–06 passed (`5/5`, collision `0/5`). Trial 05
retained a one-off 0.114 s initial-turn duration asymmetry without safety or
completion impact; Trial 06 returned to a 0.0014 s difference. Advance to a
separately frozen head-on lateral-offset scenario without retuning the controller.

Lateral-offset status: a 0.040 m nominal path-offset pilot passed with 0.195333 m
minimum centre separation, no collision and automatic completion. The world,
origins and controller parameters are frozen. The pilot is excluded from the
formal statistics. Formal Trial 01 passed and is repetition `1/5`; retain the
headings at 0 and pi because the experimental factor is path-centre offset, not
crossing angle.

Timing-integrity correction: earlier Webots repetitions were found to use variable
accelerated simulation/wall-time factors. Treat the earlier centred batch and
non-controlled accelerated-factor offset runs as functional evidence rather than
strict timing comparisons.
A new controlled-realtime offset protocol now requires pre-load and full-load
factors of 0.8–1.2, a fresh WSL/Webots/ROS graph per repetition, and recovery-based
completion before a 60 s safety ceiling.

Controlled lateral-offset status: scripted-protocol validation passed, followed by five
accepted formal repetitions (`5/5`, collision `0/5`, invalid states `0`). Mean
minimum centre separation was 0.182786 ± 0.005664 m and mean full-load factor was
0.9624 ± 0.0162.

Controlled centred status: five accepted formal repetitions are complete (`5/5`,
collision `0/5`, invalid states `0`). Mean minimum centre separation was 0.147039
± 0.001377 m and the mean bag-derived state-time factor was 0.959959 ± 0.004386.
The centred and offset batches are now suitable for controlled geometry comparison.

Controlled ninety-degree crossing status: five accepted repetitions are complete
(`5/5`, collision `0/5`, invalid states `0`). Mean minimum centre separation was
0.141695 ± 0.001439 m and the mean bag-derived state-time factor was 0.962222 ±
0.003969. All three Phase 2 geometries are complete without controller retuning.
Advance to Phase 3 cross-method ablation.

First establish a communication/CPA-only baseline with local obstacle avoidance
disabled. Record minimum centre distance, safety margin, CPA/TCPA/DCPA, avoidance
onset difference, path length, task time, lateral deviation, command smoothness,
success and collision.

## Phase 3 — Local sensing versus communication ablation

Repeat one representative moving-peer scenario under:

1. Local sensing only — complete (`5/5`, collision `0/5`), mean minimum centre
   separation `0.096032 ± 0.002114 m`.
2. Communication/CPA only — matched controlled-realtime baseline complete (`5/5`,
   collision `0/5`), mean minimum centre separation `0.147039 ± 0.001377 m`.
3. Fused local sensing plus communication/CPA — complete (`5/5`, collision `0/5`),
   mean minimum centre separation `0.150203 ± 0.005517 m`.

Use identical initial poses, speed and runtime. Do not tune parameters separately
for each method after the batch begins.

The local-only group retained state publishers solely as measurement
instrumentation; its controllers did not subscribe to peer state or calculate
CPA risk. Compared with the matched communication baseline, the local-only mean
minimum separation was 0.051007 m (34.7%) lower, while both conditions retained
5/5 completion and 0/5 collision. The fused batch enabled both inputs without
retuning and increased mean minimum separation by 0.054171 m over local-only.
Its 0.003164 m difference from communication/CPA is small relative to the fused
trial spread. In all five clean head-on repetitions CPA acted before the armed
local fallback, so the local channel remained available but did not take over.
Phase 3 is complete; advance to Phase 4.

## Phase 4 — Wooden box plus moving communicated robot

Only after Phases 1–3 pass, introduce both the wooden box and a moving peer. Test
priority resolution between stale/invalid safety stop, local obstacle avoidance,
CPA avoidance and nominal cruise. Start with one canonical geometry at `n=5`, then
add an offset geometry if the canonical case is stable.

Current status: Phases 1–3 have passed. The next experiment is the canonical
wooden-box plus moving-peer Trial 01, followed by four frozen-protocol repetitions
if the directly observed run passes.

### 2026-07-17 update: controller_v1 defect chain and controller_v4 fix

The first attempt at the canonical combined scenario (`pilot_01`-`pilot_04` in
`experiments/cooperative_avoidance_20260716/config/combined_wood_moving_peer/`)
found a `controller_v1` safety defect, not a scenario-geometry problem: after
`epuck1`'s box bypass, a grazing IR flicker near the box's trailing corner could
repeatedly re-arm an unbounded turn (a single continuous turn of up to ~4.8 s /
~1.2 rad was observed), swinging the robot into the box. This paused Phase 4 and
triggered three successive controller revisions (see
`experiments/3-1.局部避障锁存实验/`,
`experiments/3-2.统一遭遇避障实验/`, and
`experiments/3-3.全传感器避障实验/`), each committed and
tested independently before the next was designed.

`controller_v4_ros_time_consistency` (the current tip) additionally found and
fixed a second, independent defect: the controller's motion/state-machine
timers used `time.monotonic()` instead of the ROS/simulation clock, so
Webots-simulation-speed variance (already flagged in
`simulation_rate_integrity_audit_20260716.md`'s "Engineering follow-up") could
desynchronize `max_runtime_s`/`startup_hold_s`/message-freshness from what the
controller's own state machine believed had elapsed. This is now fixed: every
such timer reads `self.get_clock().now()` exclusively.

Three excluded, non-statistical v4 pilots were run to re-validate all three
components before returning to Phase 4's formal Trial 01 (evidence: git commits
`06e2b5c`, `4d70beb`, `fd0f03d`; full pilot detail in
`experiments/3-3.全传感器避障实验/`):

- `pilot_v4_b3` (static box, `enable_peer_avoidance:=true`, no moving peer):
  PASS. Real `max_x=0.1895m` past the pass threshold, clearance 0.1237m, no
  collision, no FAILSAFE.
- `pilot_v4_c` (`head_on_cpa_v4`, pure dual-robot CPA, no box, local avoidance
  left enabled to positively confirm it never mis-triggers): PASS. Both robots
  completed `CRUISE->AVOID_TURN->AVOID_PASS->RECOVER->CRUISE->COMPLETE`
  symmetrically; zero `LOCAL_*` occurrences (zones stayed clear throughout, as
  expected in a box-free arena).
- Combined box+moving-peer pilot (`combined_v4`, geometry copied verbatim from
  the pre-defect `pilot_04` configuration): PASS. `epuck1` genuinely passed the
  box (clearance 0.120m), entered `LOCAL_*` before its first `AVOID_TURN`,
  `epuck2` never entered a local-obstacle mode, both robots completed CPA
  avoidance, zero collision, zero FAILSAFE.

**Formal Trial 01 for Phase 4 has NOT been run.** Per explicit user instruction,
it remains for manual, directly-observed execution (one interface command at a
time) and must not be auto-run. The three v4 pilots above are exclusionary
evidence only and are not pooled with any formal-trial statistics.

### 2026-07-17, later same day: Phase 4 formal batch complete (5/5 PASS)

Formal Trial 01 was run manually, step-by-step, with the user directly
observing Webots throughout (per explicit instruction); Trials 02-05 then
ran automatically via `run_combined_v4_pilot.sh` once Trials 01-02 both
confirmed the identical frozen `combined_v4/` configuration passes. Full
detail, per-trial table, and batch statistics:
`experiments/3-3.全传感器避障实验/PHASE4_FORMAL_BATCH_SUMMARY_20260717.md`.

**Result: 5/5 PASS, 0/5 collision.** Mean minimum robot-robot separation
0.2736 ± 0.0044 m, mean box clearance 0.1229 m, mean full-load realtime
factor 0.984 (n=4 automated trials; Trial 01 confirmed in range by direct
user observation). Phase 4's n=5 canonical-geometry gate is met; **Phase 4
is complete.**

Naming rule for this scenario, binding for the dissertation: **"staged
local-obstacle avoidance followed by communication-assisted
proximity/cooperative avoidance."** All 5 trials triggered via
`PROXIMITY_FALLBACK` (`dcpa_at_trigger` 0.197-0.216m, always above the
0.14m predicted-conflict threshold) -- confirmed via
`analyze_trigger_reason.py` (analysis-only, does not touch the frozen
controller) on every trial. This is a genuine, reproducible property of the
frozen geometry (epuck1's post-box `LOCAL_RECOVER` restores heading, not
lateral position), not a controller defect or an engineered result. This
batch is valid evidence of the two-stage local-then-communication
avoidance mechanism working correctly and safely, but must never be cited
as evidence of preventing an otherwise-certain head-on collision.

Per the current project re-prioritization (route re-alignment against the
official COMP5200M Spec/SP, `HANDOFF_20260717.md`), further avoidance-
scenario expansion is intentionally paused here. Remaining project effort
moves to the communication-library core objectives (protocol audit, formal
communication metrics, a controlled delay/loss impairment matrix, and
physical Pi-puck validation).

### 2026-07-17 second update: started_at timebase fix, trigger-reason classification, and official Spec/SP re-alignment

**Timebase fix.** A `self.started_at` clock-initialization race was found and
fixed (`controller_v4_timebase_fix_20260717`, commit `980e7d0`; 5 new tests,
96/96 passing at that point). Re-validated with a fresh combined-pilot
regression, `combined_trial2_timebasefix` (commit `06e0f0f`): PASS,
`stopped_by_max_runtime_only=false`, `TIMEBASE_INIT` confirmed logged at each
robot's real first-valid clock sample. The v4 controller is now FROZEN at
these commits (documentation state `f1830c5`); no further avoidance-algorithm
changes are planned unless a new blocking safety defect is found.

**Naming correction for the combined Phase 4 scenario.** The combined
scenario's actual mechanism, confirmed by offline trigger-reason
reconstruction (`analyze_trigger_reason`, commit `28f78d6`, analysis-only —
the frozen controller was not touched or rerun), is: `epuck1` completes its
local wooden-box bypass first (`startup_hold_s=5s`), `epuck2` then starts
after a fixed delay (`startup_hold_s=42s`), and the two robots' subsequent
mutual avoidance is triggered by `current_distance < trigger_distance_m`
(`PROXIMITY_FALLBACK`), not by the predicted-collision condition
(`tcpa<=4.0s and dcpa<safety_radius_m`, `PREDICTED_CPA`) at trigger time
(combined `trial2_timebasefix`: `dcpa_at_trigger=0.211m`, well above the
0.14m predicted-conflict threshold). This scenario must be described as
**"staged local-obstacle avoidance followed by communication-assisted
proximity/cooperative avoidance"** — never as "synchronized" or as a scenario
"triggered by predicted CPA." Both trigger conditions are genuine,
independently-coded parts of the same frozen controller
(`collision_math.collision_risk`'s `predicted_conflict OR proximity_conflict`);
which one fires first is a property of the approach geometry and speed, not a
controller defect, and is now recorded explicitly per pilot rather than left
implicit. Interestingly, even the box-free pure-CPA pilot (`pilot_v4_c`)
triggers via `PROXIMITY_FALLBACK` at `t=11.220s` — full-resolution replay
shows this is a genuine near-tie (`dcpa` crosses its own 0.14m threshold
essentially one control tick, ~0.001s, after `current_distance` crosses
0.34m), not a mislabeling. See
`bags/*/analysis/trigger_reason_summary.{json,md}` and
`trigger_classification.csv` for the full per-sample time series.

**Scope re-anchor to the official specification.** Per the COMP5200M Project
Specification (`LU26-Spec.pdf`) and Scoping and Planning Document
(`LU2026-SP.pdf`), this project's core deliverables are the `e-puck2-Comm`
library, its ROS2 Custom Msg protocol, the neighbor-state subscription
abstraction, the simulation package, and an evaluation comparing packet
delivery / coordination efficiency in simulation versus physical hardware
(reality gap). The avoidance controller (v1-v4) is a task-specific validation
vehicle for that library (matching the Spec's "Task-Specific Validation"
objective), not the project's primary contribution. With `controller_v4`
passing all exclusionary safety pilots and formal Phase 4 pending only manual
Trial 01, further avoidance-algorithm scope is intentionally frozen. Work now
shifts to: (a) an `EpuckState` protocol audit and formal communication
metrics (delivery ratio, latency/age, sequence-loss, jitter, stale-state
safety stops, bandwidth), (b) a minimal controlled communication-impairment
matrix (no-peer / baseline / two delay levels / two loss levels), and
(c) physical Pi-puck validation and reality-gap comparison. See
`HANDOFF_20260717.md` for the full route and the Webots-vs-Gazebo deviation
note.

## Phase 5 — Communication policy and impairment

On a fixed representative scenario compare:

- periodic versus event-triggered state transmission;
- baseline latency versus controlled latency;
- baseline delivery versus controlled loss;
- fresh state versus stale/timeout state;
- timeout safety stop enabled versus the defined ablation condition.

Screen conditions at `n=5`; use at least `n=10` for the final primary comparison.
Report message count/bytes, rate, latency, jitter, loss, state age, timeout count,
recovery time, minimum separation and completion rate.

## Phase 6 — Single physical robot and HIL validation

- Repeat physical local-obstacle avoidance.
- Measure real network rate and latency.
- Verify watchdog disconnection stop.
- Feed a virtual neighbour state to the real e-puck2.
- Demonstrate CPA response to the virtual neighbour.

Label these as single-robot physical and HIL results, never as two-physical-robot
cooperative trials.

## Ground-truth improvement before final statistical batches

Add Webots Supervisor ground-truth pose and contact-event recording. Retain wheel
odometry for controller realism, but use Supervisor data for final geometric
clearance, collision and trajectory metrics.

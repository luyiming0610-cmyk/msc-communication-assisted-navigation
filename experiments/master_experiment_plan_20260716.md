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
`experiments/controller_v2_local_latch_20260717/`,
`experiments/controller_v3_unified_encounter_20260717/`, and
`experiments/controller_v4_full_sensor_bypass_20260717/`), each committed and
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
`experiments/controller_v4_full_sensor_bypass_20260717/`):

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

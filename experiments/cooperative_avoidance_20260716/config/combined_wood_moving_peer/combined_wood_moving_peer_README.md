# Canonical local-obstacle plus moving-peer fusion scenario

Date frozen: 2026-07-17

## Purpose

Demonstrate both fused inputs in one run without allowing a prior CPA turn to
distort the wooden-box approach. `epuck1` first repeats the already accepted
central-box path; `epuck2` then starts after a fixed delay and both robots perform
communicated CPA avoidance.

## Locked geometry and timing

- `epuck1`: `(-0.55, 0.0, 0.0)`; normal 5 s startup hold.
- `epuck2`: `(0.45, -0.187, pi)`; fixed 18 s startup hold.
- Wooden box centre: `(-0.25, 0.0)`, size `0.06 m` square.
- Clean 1.5 m arena; no other boxes.
- Controller speeds, sensor thresholds, CPA thresholds and local bypass settings
  are unchanged.

The box geometry is exactly the condition that passed 5/5 locked static trials
with mean odometry-derived surface clearance `0.121223 m`. The `epuck2` lane is
aligned with the observed post-box `epuck1` lane (`y` approximately `-0.187 m`).
Its delayed start keeps the peer event after the local event.

## Expected sequence

1. `epuck1` cruises toward the wooden box while `epuck2` remains in startup hold.
2. `epuck1` enters `LOCAL_*`, passes to its own right and recovers its original
   heading after completely clearing the box.
3. `epuck2` begins moving; both robots enter `AVOID_TURN` from communicated CPA
   risk and pass to their own right.
4. `epuck2` stops after cooperative recovery. A task monitor confirms the box
   pass, close peer encounter, post-pass separation, recovered headings and the
   `epuck2` stop, then shuts the launch down cleanly.

## Acceptance gates

- Realtime factors in `0.8–1.2` and a fresh WSL/Webots/ROS session.
- `epuck1` local mode occurs before its first peer `AVOID_TURN`.
- Both robots enter CPA avoidance; `epuck2` never enters a local-obstacle mode.
- No robot-to-robot or robot-to-box collision and no repeated oscillation.
- `epuck1` crosses `x=-0.175 m`; odometry-derived box clearance is at least
  `0.005 m`.
- Both robots recover their desired headings, `epuck2` stops normally and the
  task monitor ends the run before the 55 s safety deadline.
- Complete rosbag with state, odometry, commands and both robots' local sensors.

## Excluded diagnostics

The original observed combined Trial 01 placed the box after the head-on peer
event. Robot-to-robot avoidance passed with minimum centre separation
`0.145509 m`, and `epuck1` detected the box, but it stopped at `x=0.189625 m`
before the full-body pass threshold `x=0.355 m`; it is a task-incomplete
diagnostic, not a formal trial. Extending that approach produced a later pilot
with `-0.007527 m` geometric box overlap. Those records remain preserved and are
excluded from formal statistics.

The corrected scenario must pass an excluded automated pilot before a new formal
Trial 01 is recorded.

## 2026-07-17: controller_v1 defect found, scenario paused

Four excluded pilots were run against this scenario while validating the
corrected local-then-peer ordering:

- `pilot_01`: `epuck2` startup hold 18 s, `epuck2` lane `y=-0.187`. Excluded:
  `epuck2` entered `LOCAL_*` during the CPA pass; `TASK_TIMEOUT`.
- `pilot_02`: `epuck2` startup hold extended to 42 s (same lane). Excluded:
  `epuck2` still entered `LOCAL_*` during CPA; `TASK_TIMEOUT`.
- `pilot_03`: `epuck2` lane flipped to `y=+0.187` (opposite side). Excluded:
  CPA `AVOID_TURN` never armed (peer passed at 0.38 m without triggering
  avoidance); **user visually observed `epuck1` collide with the wooden box**
  during this run.
- `pilot_04`: `epuck2` lane recentred to `y=0.0` (matching the validated
  `head_on_centered` baseline geometry). This **did** fix the CPA geometry:
  both robots entered `AVOID_TURN`/`RECOVER`/`CRUISE` in sync at the same
  timestamps and `epuck2` never entered `LOCAL_*`. However, the run still
  timed out because `combined_task_coordinator.py`'s `encounter_seen` latch,
  which is computed from the periodic `/epuck1,2/state` topics rather than
  odometry, never sampled a pair below `ENCOUNTER_DISTANCE_M=0.22 m` (true
  minimum separation via `analyze_cooperative_bag` was `0.202 m`; the
  state-topic-sampled minimum was only `0.272 m`). This is a task-monitor
  resolution gap, not a controller or geometry defect.

While diagnosing the timeout, re-running `analyze_combined_task.py` against
all four bags (which the trial script never reached because of the timeout)
surfaced an independent, more serious finding:

**`box_collision_detected["/epuck1/state"] == true` in pilot_01, pilot_02, and
pilot_03; pilot_04 cleared by only `0.0014 m`, below the `0.005 m` acceptance
gate.** All four near-misses/collisions occur at nearly the same position
(`x≈-0.28 to -0.31, y≈-0.05`) and simulated time (`t≈23-28 s`), independent of
every `epuck2` geometry/timing change tried — proving this is not caused by
the CPA work in this scenario.

Root cause (traced in `cooperative_avoider.py` +
`local_obstacle_logic.py::LocalAvoidanceLatch`): after `epuck1`'s first clean
box bypass, its `LOCAL_RECOVER` heading correction brings it back along a
shallow, grazing path past the box's trailing corner. The left-side IR
reading flickers back and forth across the `side_warn_m=0.052`/
`side_release_m=0.058 m` band as the robot's own avoidance turn changes the
sensor geometry. Every flicker back into `LOCAL_LEFT_SIDE` resets
`LocalAvoidanceLatch.last_active_s`, extending the `clear_hold_s=1.0 s`
grace-turn (`LOCAL_CLEARANCE`) that continues turning in the *same*
`turn_sign` direction. Because neither the raw decision function nor the
latch imposes a cumulative-turn or re-arm-distance bound, this repeated
retrigger lets a single `-0.30 rad/s` turn continue essentially unbroken for
~4.8 s (`pilot_04`: yaw went from `-0.004 rad` to `-1.193 rad`), long enough
to swing the body into the box corner. Static Phase 1 (`fusion_static_neighbor_20260716`,
5/5, mean clearance `0.121 m`) never observed this because its controller
`max_runtime_s` cap (14-22 s) ended the run before this window (~22-24 s in)
was ever reached — it only ever validated the first bypass, never this
extended-runtime tail behaviour.

**Status: this scenario (and any further combined-scenario pilots) is
paused.** This is registered as a `controller_v1` safety defect, not a
scenario-geometry problem. See
`controller_v2_local_latch_design_20260717.md` (same directory) for the
proposed fix design. `pilot_01`-`pilot_04` bags/logs are preserved as
diagnostic evidence and must never be counted as formal trials or merged
with any `controller_v2` statistics.

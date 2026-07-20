# cooperative_exit_n2_comm_off_EXCLUSIONARY_PILOT04 -- FAILED, real structural scene-design error (preserved, not deleted)

**EXCLUSIONARY_DIAGNOSTIC. Not counted toward any formal or pilot statistic.**

## What happened

The orchestrator-level checks all looked clean: `DATA_VALIDITY=VALID`,
`stop_reason=TASK_COMPLETE_GOAL`, `cmd_vel_zero_at_end=true`,
`controller_crashed=false`. The task_completion_monitor fired
`TASK_COMPLETE_GOAL` at sim-time 17.42s for both robots. But the
trajectory data shows this "success" was fake:

- `path_length_m`: **0.0 for both robots** -- neither robot ever moved.
- `final_positions`: identical to the frozen start poses,
  `epuck1(-0.35, 0.0)`, `epuck2(0.35, 0.0)`.
- `edge_based_mode_entry_counts`: only `SAFE_STOP_STALE` and
  `STARTUP_HOLD` for both robots -- the controllers were still in their
  normal startup-hold phase (not yet even begun CRUISE) when the
  orchestrator killed them.
- `minimum_pairwise_distance_m`: 0.7 -- exactly the static start
  separation.

## Root cause (confirmed by reading controller.log and the frozen goal
parameters directly, not guessed)

The frozen goal region was `center=(0,0), radius=0.5`. Both robots'
start poses, `(-0.35,0)` and `(0.35,0)`, are only 0.35 m from the
origin -- **inside** a 0.5 m-radius circle. `task_completion_monitor.py`
started accumulating each robot's continuous-in-goal hold timer from the
very first `/epuckN/state` sample it received, which already reported
the robot sitting at its start pose, already inside the goal region.
After `goal_hold_time_s=2.0` seconds of the robots doing nothing but
sitting still in `STARTUP_HOLD` (not a bug -- this is the controller's
normal startup phase), the hold requirement was trivially satisfied and
`TASK_COMPLETE_GOAL` fired -- before either robot had moved a single
centimeter, let alone performed any navigation or avoidance.

This is a genuine scene-design flaw discovered by the pilot process
doing exactly what it is for, not a code defect in the new monitor,
the orchestrator, or the frozen controller. The monitor's own logic
(streaming `GoalHoldTracker`, tested in `test_goal_hold_tracker.py`
against the batch analyzer) behaved exactly as specified; the goal
region itself was simply sized/placed so that "success" required zero
task completion.

## Fix

`goal_radius_m` reduced from 0.5 to 0.20 in the orchestrator's frozen
constants (`GOAL_RADIUS_M` in `run_cooperative_exit_n2_trial.sh`) and in
`n2_scene_freeze_20260720.md`. Start-pose distance from the goal center
(0.35 m) now clearly exceeds the goal radius (0.20 m), requiring a
genuine net displacement of >= 0.15 m before the hold timer can even
begin -- while remaining geometrically achievable for two robots to
simultaneously satisfy alongside `safety_radius_m=0.14` (a 0.40 m-wide
region comfortably fits two points >= 0.14 m apart), consistent with
prior (pre-monitor) pilots' trajectory data showing both robots
repeatedly within roughly 0.10-0.16 m of the origin during and after
the encounter.

This is a pre-run correction of a discovered structural flaw, decided
and frozen BEFORE the corrected retry (`PILOT05`) is run -- not a
post-hoc adjustment to manufacture a particular outcome.

## Disposition

- Native WSL bag + diag_logs preserved at
  `/home/eamon/epuck_comm_bags/cooperative_exit_n2_comm_off_EXCLUSIONARY_PILOT04`
  (+ `_diag_logs`) and a SHA-256-verified Windows copy under this
  directory's sibling `bags/` path (gitignored).
- Not rerun under this same name -- the corrected retry uses
  `EXCLUSIONARY_PILOT05`.
- `DATA_VALIDITY=VALID` at the orchestrator/process level is genuinely
  correct (nothing crashed, cmd_vel genuinely ended at zero, bag/relay
  clean) -- only the trajectory-level `TASK_OUTCOME=SUCCESS` was wrong,
  which is exactly why `task_outcome` is computed from the actual bag
  data and never trusted from the orchestrator's own process-level
  checks alone.

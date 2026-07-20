# N2 scene freeze (2026-07-20, post-correction)

This record exists because the earlier pilots (`N2_COMM_OFF_EXCLUSIONARY_PILOT01-03`,
`N2_COMM_ON_EXCLUSIONARY_PILOT01`, all preserved unmodified as excluded
evidence) reached the goal region but the trial only ever stopped via
`max_runtime_s`, so `TASK_OUTCOME` could never be `SUCCESS` under the
pre-registered rule that max-runtime is never read as success. Two
changes were required to make a genuine `TASK_COMPLETE_GOAL` stop
reachable at all: an independent, read-only task-completion monitor
(`task_completion_monitor.py`), and a visible common exit/goal-region
marker in the Webots world (the shared A-D world has none). Neither
change touches the frozen CPA/local IR/ToF avoidance code, the
`EpuckState` protocol, or Objective 5 A-D's own world file/config/data.

## Visible common exit/goal-region marker

The shared, frozen `two_epuck_head_on_clean_world.wbt` (used unmodified
by Objective 5 A-D) has no visible destination marker at all -- it was
never designed to show where a task "ends". Rather than modify that
shared file, this study uses an **additive** copy:

- New file: `2-1.仿真通信实验/working/two_epuck_cooperative_exit_n2_world.wbt`
  -- identical `RectangleArena`, robot models, and start poses to the
  original, plus one new `Solid` node (`name "goal_exit_marker"`): a
  flat, semi-transparent green `Cylinder` (radius 0.5 m, height 1 mm) at
  the origin, **with no `boundingObject`** -- Webots only computes
  physics/collision for a `Solid` that has a `boundingObject`, so this
  marker is purely visual and has zero effect on robot motion,
  avoidance, or collision detection.
- New file: `2-1.仿真通信实验/working/run_dual_head_on_clean_n2_exit.py`
  -- an exact copy of `run_dual_head_on_clean.py`, differing only in
  which `EPUCK_WORLD_FILE` it points at. `run_dual_head_on_clean.py`
  (used by A-D) is never read or modified.
- The original `two_epuck_head_on_clean_world.wbt` SHA-256
  (`b8d8a99d3f4f20182cd77e21accf913be13ba1629e85fcd1a9deb0b4c7264735`)
  was recorded before this change and is unchanged after it -- verified
  by re-hashing after the new files were added.

## Frozen parameters (written into `frozen_params.json` by the
orchestrator BEFORE every trial, never adjusted after seeing a result)

| Parameter | Value | Shared by OFF/ON |
|---|---|---|
| `goal_center_x_m`, `goal_center_y_m` | 0.0, 0.0 | yes |
| `goal_radius_m` | 0.20 (corrected from an initial 0.5 -- see PILOT04 finding below) | yes |
| `goal_hold_time_s` | 2.0 | yes |
| `safety_radius_m` | 0.14 (unchanged, per instruction) | yes |
| `collision_contact_distance_m` | 0.07 | yes |
| `max_runtime_s` | 28.0 (failure backstop only, never success) | yes |
| epuck1 start pose | (-0.35, 0, 0), yaw 0 | yes |
| epuck2 start pose | (0.35, 0, 0), yaw pi | yes |
| obstacles | none (open arena, `RectangleArena 1.5x1.5`) | yes |
| world file | `two_epuck_cooperative_exit_n2_world.wbt` | yes (both OFF and ON launch the identical world/scene; only `enable_peer_avoidance` and the relay/no-relay wiring differ) |

## PILOT04 finding: initial goal_radius_m=0.5 was structurally wrong

`N2_COMM_OFF_EXCLUSIONARY_PILOT04` (preserved, excluded, see its own
`NOTE.md`) exposed that a 0.5 m goal radius is larger than the robots'
own 0.35 m start-distance from the origin -- both robots' STATIC start
poses were already inside the goal region, so the monitor's hold timer
was satisfied before either robot had moved (`path_length_m=0.0` for
both, still in `STARTUP_HOLD`). `goal_radius_m` was corrected to 0.20
(and the visual marker's `Cylinder radius` updated to match) BEFORE the
corrected retry (`PILOT05`) was run -- a pre-run fix of a discovered
structural flaw, not a post-hoc adjustment to manufacture a particular
outcome.

## Task-completion monitor (`task_completion_monitor.py`, new)

Independent, read-only rclpy node. Subscribes only to `/epuck1/state`
and `/epuck2/state` (the same already-published topic each robot's own
controller reads its own state from). Tracks, per robot, continuous
dwell inside the frozen goal region using the exact same anti-single-
frame-trigger state machine as
`task_completion_analyzer.robot_goal_completion_time()` (proven
identical by `test_goal_hold_tracker.py`'s batch-vs-stream cross-check
tests). On all robots satisfying the hold requirement, writes a single
`TASK_COMPLETE_GOAL` log line and a `monitor_verdict.json` file, then
exits. It never publishes cmd_vel or a navigation target, never
subscribes to Supervisor ground truth, and never forwards state data
back into either robot's own control loop -- functionally the same
category of read-only external measurement as Supervisor-based
post-hoc analysis, just running live so the trial can actually stop on
genuine task completion instead of only via internal controller-
recovery timing or `max_runtime_s`.

The orchestrator's wait loop watches the monitor's log for
`TASK_COMPLETE_GOAL` and, on seeing it, immediately SIGINTs the
controller process group via the existing `stop_pid_group()` helper --
the SAME safe-stop path used at every other end-of-trial point, which
triggers `cooperative_avoider.py`'s own, unmodified `stop()` method
(publishes a zero `Twist` three times from its SIGINT/KeyboardInterrupt
handler). No new stop mechanism was invented; the existing one is
reused. `trial_verdict.json` now records `stop_reason` (`TASK_COMPLETE_GOAL`
/ `CONTROLLER_SELF_COMPLETE` / `CONTROLLER_EXITED_EARLY` / `MAX_RUNTIME`)
as a field separate from `data_validity`, plus a post-hoc, bag-derived
`cmd_vel_zero_at_end` check (`verify_cmd_vel_zero.py`) confirming the
last recorded `/epuckN/cmd_vel` sample for each robot is genuinely zero.

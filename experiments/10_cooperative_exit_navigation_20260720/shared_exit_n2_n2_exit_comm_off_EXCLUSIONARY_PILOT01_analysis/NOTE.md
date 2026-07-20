# shared_exit_n2_n2_exit_comm_off_EXCLUSIONARY_PILOT01 -- FAILED, Robot B's goal_navigator never started (preserved, not deleted)

**EXCLUSIONARY_DIAGNOSTIC. Not counted toward any formal or pilot statistic.**

## CLASSIFICATION: EXCLUDED_OBSTACLE_CONFOUNDED_DIAGNOSTIC (added retroactively, revision 2)

Ran on the original obstacle-containing world (`shared_exit_n2_world.wbt`).
This particular pilot's own failure (argparse rejecting a negative
waypoint token) is unrelated to the obstacle, but it shares the same
scene geometry as PILOT02/03 -- relabeled alongside them, per the
user's explicit instruction after the root-cause audit, so all three
pre-revision-2 pilots are identifiable as having run on the
obstacle-confounded world, not the current core obstacle-free world
(`shared_exit_n2_obstacle_free_world.wbt`) used from PILOT04 onward.

## What happened

Orchestrator-level checks looked clean: `DATA_VALIDITY=VALID`, no crash,
`cmd_vel_zero_at_end=true`, `velocity_settled_at_end=true`, bag clean.
But `stop_reason=CONTROLLER_SELF_COMPLETE` (not `TASK_COMPLETE_GOAL`),
and `monitor_verdict_present=false` -- `task_completion_monitor.py`
never detected the exit condition. Bag inspection shows `/epuck1/
nav_intent` has 121 messages but `/epuck2/nav_intent` has **zero** --
Robot B's `goal_navigator` process never ran at all.

## Root cause (confirmed by reading `goal_navigator_epuck2.log`, not guessed)

```
usage: goal_navigator.py [-h] --robot-id ROBOT_ID --state-topic STATE_TOPIC
                         ...
goal_navigator.py: error: argument --waypoints: expected one argument
```

The orchestrator passes Robot B's waypoint list as
`--waypoints "$ROBOT_B_WAYPOINTS"`, where the value is
`-0.2:-0.2,0.05:-0.35,0.25:0.05,0.5:0.5` -- it starts with `-` (a
negative x-coordinate). Python's `argparse`, given a value that starts
with `-` as a SEPARATE token after `--waypoints`, treats it as
potentially another option flag rather than the argument's value, and
rejects it. This is a genuine argument-passing bug in the orchestrator
script, not a defect in `goal_navigator.py`'s own logic (which is fully
unit-tested via `navigation_target_state.py`, 10/10 passing).

Because Robot B's navigator never started, it never published
`NavigationIntent`, so Robot B's `enable_dynamic_heading` input stayed
permanently stale and `cooperative_avoider.py` correctly fell back to
its launch-time default heading (per the tested, documented fallback
behavior) -- Robot B never performed the waypoint search at all, just
held one fixed heading the whole trial. The trial only ended because
one or both controllers' internal `stop_after_recovery` /
`max_runtime_s` paths eventually fired (`ended_by_max_runtime_hits=1`).

## Fix

Use `--waypoints=$ROBOT_B_WAYPOINTS` (equals-sign syntax) instead of a
separate argument token -- `argparse` always accepts an `=`-joined value
regardless of its leading character, since it is unambiguously not a
new flag.

## Disposition

- Native WSL bag + diag_logs preserved at
  `/home/eamon/epuck_comm_bags/shared_exit_n2_n2_exit_comm_off_EXCLUSIONARY_PILOT01`
  (+ `_diag_logs`) and a SHA-256-verified Windows copy under this
  directory's sibling `bags/` path (gitignored).
- Not rerun under this same name -- the corrected retry uses
  `EXCLUSIONARY_PILOT02`.
- Process cleanup confirmed clean after this pilot.

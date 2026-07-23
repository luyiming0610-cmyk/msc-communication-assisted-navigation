# HIL safety checklist (2026-07-23)

This checklist is the authoritative gate for any nonzero `/cmd_vel` to
the real robot. It is enforced partly in code (`hil_cmd_vel_guard.py`,
`hil_preflight.py`, `run_hil_shared_exit_trial.sh`) and partly as a
human procedure (this document) -- neither substitutes for the other.

## Never modify, for any HIL purpose

- `EpuckState.msg`, `GoalAnnouncement.msg`, `NavigationIntent.msg`.
- `cooperative_avoider.py`'s CPA formula, `safety_radius_m` (0.14),
  `trigger_distance_m` (0.34), IR/ToF thresholds
  (`local_front_danger_m`=0.100, `local_front_warn_m`=0.180,
  `local_front_release_m`=0.220, `local_side_danger_m`=0.042,
  `local_side_warn_m`=0.052, `local_side_release_m`=0.058),
  `local_v4_max_encounter_duration_s` (25.0).
- The frozen N2/N3 world files and any completed formal experiment
  results.
- `goal_navigator.py` and `state_publisher.py` themselves (HIL reuses
  them via `hil_topic_adapter.py`, an import wrapper, not an edit).

## `hil_cmd_vel_guard.py` invariants (verified by `test_hil_cmd_vel_guard.py`)

- Default `armed=False`; never auto-arms.
- Publishes zero `Twist` by default, before any upstream command exists.
- Independently enforces a hard linear cap of 0.02 m/s -- does not trust
  `cooperative_avoider`'s own `nominal_speed_mps` clamp.
- Refuses ANY nonzero angular command while
  `max_angular_speed_rps` is `None`/unconfirmed.
- Fails closed to zero velocity if: heartbeat (upstream cmd_vel stream)
  is stale/missing; physical `EpuckState` is stale/missing; physical
  `EpuckState.version` mismatches the frozen protocol version; physical
  `validity_flags` does not have all of `FLAG_ODOM_VALID | FLAG_IR_VALID
  | FLAG_TOF_VALID` set (value 7 -- ODOM alone is insufficient, since
  `cooperative_avoider.py`'s local IR/ToF avoidance path requires both;
  hardened in commit `9e2b586`); the virtual peer's state is
  stale/missing (only when `require_virtual_peer=true`, i.e.
  HIL_COMM_ON); the upstream `/cmd_vel_unguarded` topic has zero or more
  than one publisher; the final, driver-facing `/cmd_vel` topic has zero
  or more than one publisher, or its sole publisher is not the guard
  itself.
- Guarantees zero velocity on process exit / Ctrl+C (`stop()`, mirrors
  `cooperative_avoider.py`'s own shutdown pattern of publishing zero
  three times before teardown).

## Launcher invariants (verified by `bash -n` and manual inspection)

- No arguments, or `--check-only`: runs `hil_preflight.py`'s offline
  checks only. Never starts Webots, the driver, the bridge, SSH, any
  controller, `rosbag`, and never publishes `cmd_vel` or writes raw
  trial data.
- `--dry-run`: prints the planned step sequence only; executes nothing.
- Any other mode: returns `PHYSICAL_MOTION_LOCKED_UNTIL_LAB_VALIDATION`
  and exits nonzero. There is no bypass flag, environment variable, or
  hidden option -- the only way past this is for
  `hil_frozen_params.json`'s `required_before_ground_motion` fields to
  actually contain measured values (verified by
  `hil_preflight.check_required_params_confirmed`).

## Before any nonzero `/cmd_vel` to the physical robot

All four, verbatim, from the user, in the same session as the test:

1. `ROBOT_ON_STAND=YES`
2. `WHEELS_CLEAR_OF_GROUND=YES`
3. `USER_AT_EMERGENCY_STOP=YES`
4. `TEST_AREA_CLEAR=YES`

Only then does the suspended-wheel test sequence run: zero velocity
first, verify left wheel direction, verify right wheel direction,
verify low forward speed, verify stop, verify the 0.02 m/s guard cap,
verify heartbeat/stale/invalid state forces zero -- each nonzero action
kept as short as possible, user observing throughout.

After the suspended-wheel test: stop all motion nodes, guarantee zero
velocity, verify the guarded `cmd_vel` publisher count, check
wheel/state, stop the actual PIDs directly (never `pkill`), confirm
`PROCESSES_CLEAN`.

## Before the first ground `EXCLUSIONARY_HIL_PILOT`

All of the following must hold simultaneously:

- Offline checks (`--check-only`) pass.
- Read-only physical connectivity check passes.
- All four suspension confirmations given and the suspended-wheel test
  observed passing.
- Field geometry measured and frozen into `hil_frozen_params.json`
  (not adjusted post-hoc to force a pass).
- Exactly one guarded `cmd_vel` publisher confirmed.
- Guard verified armed and fail-closed behaviour re-verified live.
- All physical/virtual states valid and fresh.
- User present at the emergency stop.

Then: exactly one `EXCLUSIONARY_HIL_PILOT`, `HIL_COMM_OFF` first, max
linear speed <=0.02 m/s, user watching throughout with Ctrl+C on any
anomaly, auto-recording per `hil_recorder_plan.py`. Never
auto-start `HIL_COMM_ON` or any formal batch. Any pilot failure is
preserved, never deleted/overwritten/auto-rerun. `HIL_COMM_ON` is only
handed to the user as a command after they report `HIL_COMM_OFF`
observations and complete a data audit.

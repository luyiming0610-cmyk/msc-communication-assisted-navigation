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

## Required for any future powered physical session (2026-07-23 incident)

Added after `safety_incident_unexpected_motion_20260723/SUMMARY.md`
(`UNEXPECTED_PHYSICAL_MOTION`, root cause **NOT_MEASURABLE**, not
solved). These are binding preconditions for every future session in
which the physical robot's driver/bridge/server stack is powered,
independent of and in addition to everything else in this checklist:

1. **Robot wheels suspended (on its stand) during all driver, server,
   and bridge startup, and during any reconnect** -- never bring up or
   reconnect any part of the physical stack while the robot is on the
   ground.
2. **Continuous recording of `/cmd_vel_unguarded`, `/cmd_vel`,
   `/hil_guard/arm`, `/epuck1/state` validity_flags, and
   `/epuck_bridge/status`**, with local wall-clock and monotonic
   timestamps on every message -- implemented and tested offline as
   `tools/hil_command_evidence_recorder.py` (started/stopped via
   `tools/run_hil_command_evidence_recorder.sh start|stop`, which does
   exact-PID cleanup, bounded shutdown, a new timestamped output
   directory per session, and SHA-256-verifies the produced CSV). A
   Publisher-count=0 snapshot is not a substitute; only continuous
   recording can settle a future command-origin question. **Not yet
   run against the physical stack -- design/implementation/tests are
   complete offline only; see `COMMAND_EVIDENCE_ACTIVATION.md` for the
   exact steps to actually use it in a session.**
3. **Pi-side, timestamped logging of every received motor command** --
   designed, implemented, and unit-tested (24/24 tests) as
   `pi_command_audit/pi_epuck_tcp_server_sensors_audited.py`, disabled
   by default (`command_audit_enabled` parameter). **Not yet deployed
   to the Pi.** The Pi continues running the original, unaudited file
   until this variant is explicitly reviewed and deployed -- see
   `pi_command_audit/PROVENANCE.md`.
4. **A single fail-closed guard process (`hil_cmd_vel_guard.py`) as the
   sole permitted publisher onto the real `/cmd_vel`** -- already an
   existing invariant of this checklist, restated here as a hard
   precondition specifically for the ground-placement step.
5. **Explicit zero-output verification** (guarded `cmd_vel` publisher
   count exactly 1, its output confirmed zero, upstream armed=False)
   completed and observed **immediately before** the robot is placed on
   the ground -- not merely at some earlier point in the session.
6. **Immediate incident stop on any unexplained movement** -- physical
   e-stop first, then software disarm, then halt all further physical
   work and audit before any further live action, exactly as this
   incident was handled.

## Required in addition, after the second 2026-07-23 incident (shared-domain test hazard)

Added after `safety_incident_unexpected_motion_2_20260723/SUMMARY.md`
found that `epuck2_comm`'s own `colcon test`/`pytest` suite constructs
real, unremapped rclpy nodes (`CooperativeAvoider`, `StatePublisher`,
`NetworkImpairmentRelay`, `SequenceCounterNode`) with no
`ROS_DOMAIN_ID` isolation from any live physical process:

7. **Never run `pytest`/`colcon test`/`python3 -m unittest` for
   `epuck2_comm` directly while any part of the physical stack could be
   live.** Use `run_isolated_test_suite.sh` instead -- see
   `HIL_LAB_RUNBOOK.md` step 1. This is required in addition to (not
   instead of) the test files' own `-r __ns:=/pytest_isolated` remaps
   and `conftest.py`'s forced test-only `ROS_DOMAIN_ID`.
8. **Never bring up any part of the physical stack while a test run is
   in progress**, for the same reason in reverse.

## Robot must remain suspended (wheels off the ground) until ALL of the following are true

Added as the command-evidence chain (Part 2/3 above) reached a
testable, offline-complete state. The robot must stay on its stand --
not merely "about to be lowered" -- until every one of these is
confirmed, in this session, not assumed from an earlier one:

1. The Pi-side command audit (`command_audit_enabled=true`) is active
   and its audit file is confirmed growing.
2. The WSL-side command-evidence recorder
   (`run_hil_command_evidence_recorder.sh start`) is active and its
   manifest confirms the process is running.
3. The guard (`hil_cmd_vel_guard.py`) is confirmed the sole publisher
   on the real `/cmd_vel` (`Publisher count` == 1, and that publisher
   is the guard itself).
4. The guard's current output is confirmed continuously zero (sampled
   more than once, not a single snapshot) immediately before ground
   placement.

Only once all four hold simultaneously may the robot be placed on the
ground. See `pi_command_audit/PROVENANCE.md` and
`tools/run_hil_command_evidence_recorder.sh` for exactly how items 1
and 2 are activated -- both are currently implemented and tested
offline only, not yet run against the physical stack.

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

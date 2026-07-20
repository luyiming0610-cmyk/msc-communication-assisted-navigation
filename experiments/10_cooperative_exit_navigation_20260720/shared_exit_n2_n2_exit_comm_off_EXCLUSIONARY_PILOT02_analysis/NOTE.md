# shared_exit_n2_n2_exit_comm_off_EXCLUSIONARY_PILOT02 -- FAILED, genuine task timeout + a real clock-basis bug found (preserved, not deleted)

**EXCLUSIONARY_DIAGNOSTIC. Not counted toward any formal or pilot statistic.**

## REVISION NOTE (adds context, does not withdraw anything below)

A full read-only root-cause audit (see `experiments/10_
cooperative_exit_navigation_20260720/root_cause_audit_dynamic_
heading_20260720.md`) additionally examined this pilot's sensor data.
Robot B's first `LOCAL_FRONT_WARN` encounter here (`t≈33.1s`) IS
supported by genuine sensor evidence of the pre-registered obstacle
(a monotonically closing `front_distance_m` as Robot B turns toward
and past it, both `left`/`right` staying `inf` throughout -- a real
front-only detection, not a phantom trigger). The obstacle's role in
Robot B's difficulty is not being withdrawn here. However, the
DOMINANT, confirmed root cause found across both PILOT02 and PILOT03 is
a separate mechanism unrelated to this obstacle: `goal_navigator`'s
informed-mode target never stops updating after a robot individually
arrives, causing a go-to-goal heading singularity and continuous
in-place spin (see PILOT03's NOTE.md revision and the audit doc for the
full mechanism). Both robots in both pilots show the same repeating
encounter pattern; the obstacle alone does not explain Robot A's
identical behavior far from it.

## Two separate findings

### 1. Genuine task-level finding: Robot B did not complete its search within `max_runtime_s`

The argparse fix from PILOT01 worked -- both `goal_navigator` processes
ran, both published `NavigationIntent` (125/127 messages). Real
navigation happened:

- **Robot A** (informed): reached the exit region and held it,
  `individual_completion_time_s=20.38s` -- a genuine individual success.
- **Robot B** (uninformed, search): `path_length_m=0.556m` of the
  planned 1.2535m waypoint route -- never reached the exit,
  `individual_completion_time_s=null`. `stop_duration_s=20.92s` (~38%
  of the trial stationary), consistent with the pre-registered obstacle
  on Robot B's path causing more local-avoidance dwell time than the
  original max_runtime_s computation's 20% margin assumed.
- Trial ended via `max_runtime_s` (`ended_by_max_runtime=true`) ->
  `TASK_OUTCOME=TASK_FAILURE` per the pre-registered rule, correctly
  computed. `collision_count=0`, `safety_margin_m=0.410` -- safe
  throughout, just did not finish in time.

**This finding is NOT being used to justify adjusting `max_runtime_s`
or `nominal_speed_mps` post-hoc** -- those stay frozen at their
pre-registered values (`55.0s`, `0.04 m/s`) pending the corrected
retry. If the retry shows the same pattern, that becomes a genuine,
reportable result for the user to decide on (a design review, not a
unilateral adjustment).

### 2. Real bug: `goal_navigator.py` never enabled `use_sim_time`

`exit_discovery_time_s` in this pilot's analysis report is
`1784576082.97` -- a raw wall-clock Unix epoch value, not a
trial-relative duration. Root cause (confirmed by reading
`goal_navigator.py`'s `main()`, not guessed): `rclpy.init(args=[])` was
called with no `use_sim_time` override, so `self.get_clock().now()`
followed the WALL clock while every other process in this study
(`state_publisher`, `cooperative_avoider`, `task_completion_monitor`)
follows Webots simulation time via `/clock`. This did NOT affect actual
navigation behavior (waypoint advancement and heading computation are
purely position-based, never clock-based, confirmed by reading
`navigation_target_state.py`) -- only the logged event timestamps
(`EXIT_KNOWN_AT_START`, `WAYPOINT_REACHED`, and would-be
`ANNOUNCEMENT_TX_FIRST`/`SEARCH_TO_GOAL_SWITCH` under COMM_ON) were
corrupted. `exit_announcement_*`/`switch_time_s` fields are unaffected
here since this trial is COMM_OFF (`NOT_APPLICABLE`), but this bug
would have silently corrupted the ON pilot's most important
communication-timing metrics.

## Fix

`rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true"])`.

## Disposition

- Native WSL bag + diag_logs preserved at
  `/home/eamon/epuck_comm_bags/shared_exit_n2_n2_exit_comm_off_EXCLUSIONARY_PILOT02`
  (+ `_diag_logs`) and a SHA-256-verified Windows copy under this
  directory's sibling `bags/` path (gitignored).
- Not rerun under this same name -- the corrected retry uses
  `EXCLUSIONARY_PILOT03`.
- `off_leak_detected=false`, `off_leak_check_message_count=0` --
  confirmed no `GoalAnnouncement` leak under COMM_OFF in this pilot,
  unaffected by either finding above.

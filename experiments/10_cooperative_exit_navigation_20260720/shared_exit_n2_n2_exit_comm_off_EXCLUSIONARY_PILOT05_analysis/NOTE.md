# shared_exit_n2_n2_exit_comm_off_EXCLUSIONARY_PILOT05 -- FAILED, genuine "visual marker detected as real obstacle" defect found (preserved, not deleted)

**EXCLUSIONARY_DIAGNOSTIC. Not counted toward any formal or pilot statistic.**

Retry of PILOT04 after widening the two parking zones to 0.300m
center-to-center spacing (commit `33bca94`). Same symptom persists:
`stop_reason=CONTROLLER_SELF_COMPLETE`, `monitor_verdict_present=false`,
`ended_by_max_runtime=true`, `latched_failsafe=true`, neither robot's
`individual_completion_time_s` set.

## Root cause (confirmed by raw sensor dump, not guessed)

`dump_state_sensor_trace.py` on `/epuck1/state` around `t=33-40s` shows
Robot A's position freeze at `(0.564, 0.4516)` from `t=34.5s` onward
while its yaw sweeps through many radians and `front_distance_m`
oscillates between `~0.17m` and `>1.0m` in direct correlation with that
sweep -- the classic signature of the robot spinning in place while
its front sensor genuinely re-detects a small, real, nearby piece of
scene geometry at a fixed bearing each rotation (`left`/`right` stay
`inf` throughout -- a real front-only detection). `left`/`right` `inf`
the whole time rules out a distant/ambient cause. `(0.564, 0.4516)` is
only `0.073m` from Robot A's own parking-zone marker at
`(0.6273, 0.4151)` -- well within front-sensor range.

The parking-zone markers (and the pre-existing gate-post and
completion-zone markers) were authored as `Solid` nodes with no
`boundingObject`/`Physics` -- correctly non-colliding for PHYSICS
purposes, but Webots' e-puck infra-red `DistanceSensor`s ray-trace
against ANY `Solid`'s rendering geometry regardless of
`boundingObject`/`Physics` presence. A `Solid` intended as a purely
visual marker is therefore still a real obstacle to the robots' local
IR/ToF avoidance -- the frozen `local_obstacle_logic.py` correctly,
repeatedly detected it and could not resolve a "stationary point object
directly ahead" encounter within its turn-ledger budget, matching
Robot B's `TURN_LEDGER_CEILING` failsafe in both this pilot and
PILOT04. This same mechanism plausibly explains the ORIGINAL root-cause
audit's much earlier, tentative "consistent with detecting the nearby
visual gate-post markers" note for the pre-fix spin bug -- never
previously confirmed to the specific marker, now confirmed here.

## Fix

All four visual-only markers (both gate posts, the exit completion
zone marker, both parking zone markers) converted from `Solid` to
`Transform` nodes in `shared_exit_n2_obstacle_free_world.wbt` --
`Transform` is a pure rendering/grouping node with no solid/physics
semantics, so it is invisible to `DistanceSensor` ray casts while
remaining visually present for verification. Nothing about
`local_obstacle_logic.py`, the CPA formula, IR/ToF thresholds, or
`safety_radius_m` was touched -- this is a scene-authoring correction
to markers newly added by this study, not a change to any frozen
safety mechanism.

## Disposition

- Native WSL bag + diag_logs preserved at
  `/home/eamon/epuck_comm_bags/shared_exit_n2_n2_exit_comm_off_EXCLUSIONARY_PILOT05`
  (+ `_diag_logs`) and a SHA-256-verified Windows copy under this
  directory's sibling `bags/` path (gitignored).
- Not rerun under this same name -- the corrected retry uses
  `EXCLUSIONARY_PILOT06`.
- Process cleanup confirmed clean after this pilot.
- `off_leak_detected=false`, `off_leak_check_message_count=0` --
  confirmed no `GoalAnnouncement` leak under COMM_OFF, unaffected by
  this finding.

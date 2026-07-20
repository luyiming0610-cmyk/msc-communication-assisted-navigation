# shared_exit_n2_n2_exit_comm_off_EXCLUSIONARY_PILOT04 -- FAILED, genuine parking-zone-spacing design defect found (preserved, not deleted)

**EXCLUSIONARY_DIAGNOSTIC. Not counted toward any formal or pilot statistic.**

First pilot on the obstacle-free core world (revision 2 of
`shared_exit_frozen_params.json`, `shared_exit_n2_obstacle_free_world.wbt`),
run after implementing the ARRIVED_HOLD arrival-lock fix, `enable_dynamic_speed`,
and the two post-exit parking zones.

## What happened

`stop_reason=CONTROLLER_SELF_COMPLETE`, `data_validity=VALID`,
`monitor_verdict_present=false`. Neither robot reached its own parking
zone (`individual_completion_time_s` both `null`), `verdict.task_outcome
=TASK_FAILURE`, `ended_by_max_runtime=true`, `latched_failsafe=true`.
Robot A's `path_length_m=0.922m` (vs. a 0.4031m straight-line distance
to the exit) and Robot B's `path_length_m=1.637m` (vs. a 1.2535m
planned search path) both show substantial extra travel/dwell beyond
their planned routes.

## Root cause (confirmed by reading `controller.log`'s TRANSITION lines, not guessed)

Robot B (`cooperative_avoider-2`) cruised normally from `t=21.4s` to
`t=61.7s` (40s with no encounter at all -- confirms the ARRIVED_HOLD
fix and obstacle removal are both working correctly up to this point).
At `t=61.7s` it entered `LOCAL_FRONT_WARN`, cycled through
`DETECT_TURN -> SIDE_TRACK -> FRONT_WARN` again, and at
`encounter_elapsed=7.46s` tripped the frozen local-avoidance state
machine's `TURN_LEDGER_CEILING` failsafe (`turn_ledger=1.4338rad` >
`max_turn_ledger_rad=1.40`) -- a real, unmodified safety mechanism
working exactly as designed, not a defect in `local_obstacle_logic.py`.

This is genuinely explained by parking-zone geometry, not a controller
bug: Robot A's parking zone `(0.64, 0.50)` and Robot B's parking zone
`(0.50, 0.64)` were placed exactly `0.198m` apart (center-to-center) --
satisfying the literal `> 0.14m` (`safety_radius_m`) spacing
requirement, but NOT accounting for the local IR/ToF sensor's own
`local_front_warn_m=0.180m` / `local_front_release_m=0.220m`
thresholds. Robot B's transit leg from the exit-region boundary to its
own parking zone at `(0.50, 0.64)` passes within roughly `0.14m` of
Robot A's parking position at `(0.64, 0.50)` (both are close to the
shared exit center along nearly-perpendicular directions) -- well
inside the front sensor's warn/danger range. Since both robots
genuinely constitute a real mutual-avoidance object for each other's
local sensors (by design, independent of `enable_peer_avoidance`,
confirmed in the original root-cause audit), Robot B's local avoidance
correctly, repeatedly detected Robot A sitting in its own parking zone
directly along Robot B's final approach leg, and could not cleanly
resolve the encounter (a near-stationary "obstacle" directly ahead)
within the turn-ledger budget.

## Disposition

This is NOT a request to weaken or retune the frozen local-avoidance
thresholds (`local_front_warn_m`, `TURN_LEDGER_CEILING`, etc.) -- those
stay untouched. The fix is entirely in the NEW parking-zone geometry
(Part V), which this pilot proves was placed too close together
relative to the local sensor's own detection range, not just the
`>0.14m` body-collision margin. A corrected geometry (both zones moved
further from the shared exit center along directions roughly opposite
each other, raising true center-to-center spacing to `~0.30m`) is
being verified with `verify_shared_exit_geometry.py` before any retry.

- Native WSL bag + diag_logs preserved at
  `/home/eamon/epuck_comm_bags/shared_exit_n2_n2_exit_comm_off_EXCLUSIONARY_PILOT04`
  (+ `_diag_logs`) and a SHA-256-verified Windows copy under this
  directory's sibling `bags/` path (gitignored).
- Not rerun under this same name -- the corrected retry uses
  `EXCLUSIONARY_PILOT05`.
- Process cleanup confirmed clean after this pilot.
- `off_leak_detected=false`, `off_leak_check_message_count=0` --
  confirmed no `GoalAnnouncement` leak under COMM_OFF, unaffected by
  this finding.

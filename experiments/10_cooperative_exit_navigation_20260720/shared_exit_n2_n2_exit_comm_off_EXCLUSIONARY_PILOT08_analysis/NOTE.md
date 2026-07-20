# shared_exit_n2_n2_exit_comm_off_EXCLUSIONARY_PILOT08 -- FAILED, genuine parked-vs-wall sensor clearance defect found (preserved, not deleted)

**EXCLUSIONARY_DIAGNOSTIC. Not counted toward any formal or pilot statistic. CLASSIFICATION: EXCLUDED_EXIT_GEOMETRY_DIAGNOSTIC.**

First pilot to actually run the full duration on revision-3 geometry
(exit at (0.50,0.50), elevated markers, watchdog fixed). Neither robot
completed: `stop_reason=CONTROLLER_SELF_COMPLETE` (both controllers hit
their own internal `max_runtime_s=102.0` fallback), `data_validity=
VALID`, `monitor_verdict_present=false`.

## Root cause (confirmed by controller.log TRANSITION lines + raw sensor dump, not guessed)

Robot A (`cooperative_avoider-1`) cycled through FOUR separate
`LOCAL_FRONT_WARN -> ... -> LOCAL_RECOVER` encounters (t=39.1s, 64.6s,
90.7s, each ~19-21s to reach `PASS_CONFIRM`/`RECOVER`), never once
reaching `ARRIVED_HOLD`, for the entire 102s runtime. This is a
REPEATING cycle against something that reappears every time -- a
signature distinct from PILOT04-06's single stuck encounters.

Robot A's parking zone (revision 3: `(0.6520, 0.3904)`) had only
`0.021m` of wall clearance by the physical/CPA-style measure
(`arena_half_extent - x - parking_radius - robot_radius`) -- far
inside `local_front_release_m=0.220m`. Robot A's own local IR/ToF
sensor genuinely, repeatedly detects the `+x` ARENA WALL ITSELF while
trying to settle into that corner-hugging spot. Unlike a discrete
object, a wall can never be "passed" by `local_obstacle_logic.py`'s
`SIDE_TRACK`/`PASS_CONFIRM` bypass maneuver -- the robot backs off,
tries to re-approach its target (which is still right next to the
wall), and re-triggers the same encounter, indefinitely.

## Why revision 3's geometry check missed this

`verify_shared_exit_geometry.py`'s parking-zone wall-clearance check
only verified `parking_radius + robot_radius` margin (~0.077m,
appropriate for physics/CPA-style non-overlap) -- it never checked
clearance against `local_front_release_m`, the actual relevant
threshold for a false LOCAL avoidance trigger. This is the same CLASS
of oversight as PILOT04 (checked `safety_radius_m` but not
`local_front_release_m` for the peer-vs-peer case) -- now understood
and fixed for the peer-vs-wall case too.

## Fix

The shared exit itself relocated from `(0.50, 0.50)` to `(0.25, 0.25)`
-- still clearly corner/edge-directional (not central) -- giving
enough depth beyond it for both parking zones to simultaneously
satisfy (a) parked-vs-parked spacing (`>0.324m`, still met: `0.340m`)
and (b) a NEW, corrected parked-vs-wall check using
`local_front_release_m + robot_radius_m + geometry_margin_m =
0.287m` (both parking zones now clear this on both axes, tightest
margin `0.072m`). `verify_shared_exit_geometry.py` gained this new
hard check so it can never again let this class of defect through.
Robot start poses are UNCHANGED; `robot_b.search_waypoints_m`'s final
waypoint updated to the new exit location (shortening the frozen
search path and its travel-time term in `max_runtime_s`, recomputed to
`91.0s`). `local_obstacle_logic.py`, the CPA formula, IR/ToF
thresholds, `safety_radius_m`, and `DURATION_CEILING` remain untouched.

## Disposition

- Native WSL bag + diag_logs preserved at
  `/home/eamon/epuck_comm_bags/shared_exit_n2_n2_exit_comm_off_EXCLUSIONARY_PILOT08`
  (+ `_diag_logs`) and a SHA-256-verified Windows copy under this
  directory's sibling `bags/` path (gitignored).
- Not rerun under this name -- the corrected retry uses `PILOT09`.
- Process cleanup confirmed clean after this pilot.

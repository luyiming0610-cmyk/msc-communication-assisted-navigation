# shared_exit_n2_n2_exit_comm_off_EXCLUSIONARY_PILOT09 -- FAILED, two distinct findings: a controller-launch bug and a safety-margin violation (preserved, not deleted)

**EXCLUSIONARY_DIAGNOSTIC. Not counted toward any formal or pilot statistic. CLASSIFICATION: EXCLUDED_EXIT_GEOMETRY_DIAGNOSTIC + infrastructure bug.**

First pilot on revision-4 geometry (exit relocated to (0.25,0.25)).
Real progress: Robot B (`epuck2`) genuinely reached and latched
`ARRIVED_HOLD` at `t=84.66s` (`individual_completion_time_s=84.66`,
confirmed via `goal_navigator_epuck2.log`) -- the FIRST time any robot
in this study has cleanly reached its own parking zone since the
arrival-lock fix was implemented. Robot A did not: `data_validity=
VALID`, `task_outcome=UNSAFE_FAILURE`, `ended_by_max_runtime=true`.

## Finding 1: `stop_after_recovery=True` is wrong for this study (confirmed root cause)

Robot A's `controller.log` shows exactly ONE local-avoidance encounter
(`t=54.9-77.5s`, no failsafe), reaching `LOCAL_RECOVER` and then
genuinely returning to `CRUISE` at `t=93.26s` -- but
`run_shared_exit_n2_controllers.py` still set
`stop_after_recovery=True` (inherited from the pre-ARRIVED_HOLD
design, where "a recovery maneuver just finished" was the only
completion proxy available). This permanently stopped Robot A's
controller the instant it returned to `CRUISE`
(`COMPLETE: local recovery completed`, `t=93.76s`), before it could
continue navigating toward its own parking zone at all. This is a
controller-LAUNCH configuration bug, not scene geometry -- genuine
completion is now judged by `goal_navigator`'s own `ARRIVED_HOLD` latch
and `task_completion_monitor.py`'s live `TASK_COMPLETE_GOAL` signal;
`stop_after_recovery` must stay `False` so the controller keeps
navigating after a recovery instead of quitting early.

**Fix applied**: `stop_after_recovery` changed from `True` to `False`
in `run_shared_exit_n2_controllers.py`. `max_runtime_s` remains the
ultimate backstop.

## Finding 2: a genuine safety-margin violation, reported by the pre-registered analyzer exactly as designed

`analyze_shared_exit_trial.py` correctly scored this trial
`UNSAFE_FAILURE`: `minimum_pairwise_distance_m=0.1033m` <
`safety_radius_m=0.14m` (margin `-0.0367m`) -- though `collision_count
=0` (no actual contact; `collision_contact_distance_m=0.07m` was never
breached). This is the pre-registered `build_task_verdict()` logic
working exactly as designed, not a defect in the analyzer.

The stored bag has now been decoded independently on Windows and the
minimum-distance sample has been reconstructed exactly. At 74.42s after
the first state, Robot B was already near its parking point at
`(0.182372, 0.372461)` while Robot A was still approaching the exit at
`(0.089470, 0.417701)`. Their centre distance was `0.103331776m`.
Both robots reported `OBSTACLE_CLEAR` and valid local sensors at this
instant. The event occurred well before Robot A's later recovery and
early-stop event, so it is independent of `stop_after_recovery=True`.

Root cause: revision 4 checked Robot B's route against parked Robot A,
but did not check the symmetric case of Robot A's route against parked
Robot B. Robot B's old parking point lay directly beside Robot A's
ingress corridor. Revision 5 therefore adds hard, symmetric checks for
both ingress routes and both post-exit legs, in addition to the existing
wall and parked-vs-parked checks. The revised holding points are
`(-0.20, 0.40)` and `(0.40, -0.20)`; the smallest of the five checked
transit clearances is `0.42426m`, above the local-sensor-aware
requirement of `0.324m`. No controller, CPA or sensor threshold is
changed.

## Disposition

- Native WSL bag + diag_logs preserved at
  `/home/eamon/epuck_comm_bags/shared_exit_n2_n2_exit_comm_off_EXCLUSIONARY_PILOT09`
  (+ `_diag_logs`) and a SHA-256-verified Windows copy under this
  directory's sibling `bags/` path (gitignored).
- Process cleanup confirmed clean after this pilot.
- Given this trial is now the sixth preserved diagnostic pilot in this
  round (PILOT04-09), well beyond the originally-authorized "PILOT07
  only, stop on geometry/parking failure" scope, and given Finding 2 is
  a genuinely new, more serious category of concern (a safety-margin
  violation, not merely a completion-timing issue), this is reported to
  the user for a decision rather than proceeding directly to another
  pilot.

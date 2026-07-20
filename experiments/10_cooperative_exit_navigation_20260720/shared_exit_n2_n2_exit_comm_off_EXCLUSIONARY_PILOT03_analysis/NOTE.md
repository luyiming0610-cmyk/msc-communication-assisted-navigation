# shared_exit_n2_n2_exit_comm_off_EXCLUSIONARY_PILOT03 -- FAILED (genuine), sim-time fix confirmed working, but Robot B does not reliably complete under COMM_OFF

**EXCLUSIONARY_DIAGNOSTIC. Not counted toward any formal or pilot statistic.**

## CLASSIFICATION: EXCLUDED_OBSTACLE_CONFOUNDED_DIAGNOSTIC (added retroactively, revision 2)

Ran on the original obstacle-containing world (`shared_exit_n2_world.wbt`).
This pilot's `DURATION_CEILING` failsafe was later shown (see the
REVISION NOTE above) to have latched on Robot A via the arrival-lock
gap, not the obstacle -- but the scene itself still contains the
obstacle confound, so per the user's explicit instruction this pilot
is excluded from the core OFF/ON comparison and not rerun. The core
study from PILOT04 onward uses the obstacle-free world
(`shared_exit_n2_obstacle_free_world.wbt`).

## REVISION NOTE (superseding the original attribution below, not deleting it)

The original text below attributed the `DURATION_CEILING` failsafe to
"Robot B's encounter with the pre-registered search-path obstacle."
That attribution is **withdrawn** after a full read-only root-cause
audit (see `experiments/10_cooperative_exit_navigation_20260720/
root_cause_audit_dynamic_heading_20260720.md`):

- The failsafe actually latched on **Robot A** (`cooperative_avoider-1`),
  not Robot B.
- It triggered **after** Robot A had already individually reached and
  held the goal (`t≈32.3s`) -- while `goal_navigator` continued to
  recompute and publish a fresh `atan2`-based heading toward the same
  fixed target every 0.5s, forever, with no arrival/hold logic.
- Bag evidence: Robot A's position is frozen at (0.542, 0.496) from
  `t≈33.8s` onward while its yaw sweeps continuously through many
  radians (a classic go-to-goal heading singularity at near-zero
  distance-to-target), and `front_distance_m` cycles between 0.17-0.31m
  in direct correlation with that spin -- consistent with detecting the
  nearby visual gate-post markers as the robot rotates in place, not
  the pre-registered obstacle (which Robot A never approaches).
- The pre-registered obstacle's causal role in *Robot B's* difficulty
  is genuinely supported by separate sensor evidence (PILOT02, a
  monotonically closing `front_distance_m` consistent with the
  obstacle's real position) -- that part of the original diagnosis was
  not wrong, only its application to Robot A's failsafe.

**Do not cite the original "Robot B / obstacle" attribution below for
Robot A's failsafe.** The original text is preserved unmodified beneath
this note for traceability, not because it remains correct.

## Sim-time fix confirmed working

`exit_discovery_time_s=4.08` (trial-relative, sensible) -- the PILOT02
clock-basis bug is fixed. `off_leak_detected=false`,
`off_leak_check_message_count=0` -- confirmed no `GoalAnnouncement`
leak under COMM_OFF.

## Genuine finding: Robot B did not complete, for the SECOND consecutive corrected attempt

- **Robot A** (informed): reached the exit at `t=20.42s` -- consistent
  with PILOT02's `20.38s`, a repeatable individual success.
- **Robot B** (uninformed, search): `path_length_m=0.559m` of the
  planned 1.2535m route (near-identical to PILOT02's 0.556m) --
  never reached the exit.
- **This time the trial ended via a latched FAILSAFE, not a plain
  timeout**: `failsafe_cause=DURATION_CEILING` (confirmed by reading
  `controller.log` directly) -- the frozen local-avoidance state
  machine's `local_v4_max_encounter_duration_s=25.0s` ceiling was
  exceeded during Robot B's encounter with the pre-registered
  search-path obstacle. This is the SAME frozen safety mechanism
  documented in the project's controller-v4 history (a deliberate,
  tested escalation, not a new defect) -- it means Robot B's local
  avoidance could not cleanly resolve the obstacle encounter within its
  designed time ceiling.
- `collision_count=0`, `safety_margin_m=0.594m` -- safe throughout in
  both senses (no collision, and the failsafe itself is a safety
  mechanism working as designed, not a violation).

## Why this is reported, not silently retried with different parameters

Per instruction, `max_runtime_s` and `nominal_speed_mps` are frozen,
pre-registered values that must never be adjusted after seeing pilot
results. Two independent corrected attempts (PILOT02: plain
`max_runtime_s` timeout with slow progress; PILOT03: a `DURATION_
CEILING` failsafe from a prolonged obstacle encounter) both show Robot
B failing to complete its search under COMM_OFF with the current
obstacle placement and timing budget. This is now a reproducible
pattern, not a one-off artifact, and is reported to the user as a
genuine finding requiring their decision (design review of the
obstacle/waypoint/timing geometry, or acceptance as a valid descriptive
COMM_OFF outcome) rather than adjusted unilaterally.

## Disposition

- Native WSL bag + diag_logs preserved at
  `/home/eamon/epuck_comm_bags/shared_exit_n2_n2_exit_comm_off_EXCLUSIONARY_PILOT03`
  (+ `_diag_logs`) and a SHA-256-verified Windows copy under this
  directory's sibling `bags/` path (gitignored).
- Process cleanup confirmed clean after this pilot.
- Phase 3 halted here pending the user's decision -- the ON pilot has
  NOT been run.

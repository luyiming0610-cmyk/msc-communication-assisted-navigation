# shared_exit_n2_n2_exit_comm_off_EXCLUSIONARY_PILOT06 -- FAILED, Solid-to-Transform hypothesis DISPROVEN, real cause is gate-post proximity + slow encounter resolution (preserved, not deleted)

**EXCLUSIONARY_DIAGNOSTIC. Not counted toward any formal or pilot statistic.**

Retry of PILOT05 after converting all four visual markers from `Solid`
to `Transform` nodes. Same overall symptom persists:
`stop_reason=CONTROLLER_SELF_COMPLETE`, `ended_by_max_runtime=true`,
`latched_failsafe=true`, neither robot's `individual_completion_time_s`
set, `data_validity=VALID`, `collision_count=0`,
`minimum_pairwise_distance_m=0.2056m` (safe throughout).

## PILOT05's Solid-to-Transform hypothesis is DISPROVEN

`dump_state_sensor_trace.py` on `/epuck1/state` at `t=59-62s` shows
Robot A freezing at `(0.6047, 0.2202)` -- NOT near its own
`Transform`-based parking marker this time -- with the same spinning
signature (`front_distance_m` oscillating `0.17m` to `~0.95m` as yaw
sweeps, `left`/`right` staying `inf`). `(0.6047, 0.2202)` is `0.1506m`
from `exit_gate_post_2` at `(0.456, 0.244)` -- inside
`local_front_warn_m=0.180m`. Converting markers to `Transform` did NOT
stop them being detected: Webots' `DistanceSensor` ray-casting operates
on the RENDERED scene graph (any visible `Shape`, whether wrapped in
`Solid` or `Transform`), not the PHYSICS scene graph -- the
`Solid`/`Transform` distinction only affects physics/collision
semantics, not optical ray-cast visibility. This was an incorrect
hypothesis in PILOT05's NOTE.md, recorded here as a correction, not
deleted.

The genuinely differentiating factor between the flat, thin
(`height=0.001m`) parking/completion markers and the TALL
(`height=0.06m`, comparable to the robot's own IR sensor height) gate
posts is height, not node type: this pilot's encounter was against a
gate post, not a parking marker, while PILOT05's was against a parking
marker -- both are real, height-overlapping geometry to the sensor.

## A second, independent contributing factor: mutual-avoidance resolution time

Robot A's TWO encounters here (`t=34.8-56s` and `t=60.1-85s`, ~20-22s
each to reach `PASS_CONFIRM`/`RECOVER`) and Robot B's `TURN_LEDGER_
CEILING` failsafe (`t=61.9-85s`, `encounter_elapsed=13.66s`) overlap in
time -- both robots are near the shared gate simultaneously. This is
consistent with the frozen `local_obstacle_logic.py`'s designed
resolution time for a genuine encounter (`required_lateral_offset_m=
0.070m` + `required_longitudinal_progress_m=0.10m` at
`avoidance_speed<=0.012m/s` inherently takes on the order of 20s to
satisfy `PASS_CONFIRM`) -- not a defect, a property of the existing,
unmodified, tested state machine. `max_runtime_s=68.0s` (startup_hold
5.0s + ~63s of run budget) does not currently allocate enough time for
a robot to potentially need 1-2 such encounters (gate-post proximity
and/or mutual robot encounter) near a single shared narrow gate.

## Why this is reported, not further unilaterally patched

Three consecutive pilots (PILOT04/05/06) on the obstacle-free core
world have each failed for a DIFFERENT proximate cause (parking-zone
spacing vs. sensor range; a disproven Solid/Transform hypothesis; gate-
post proximity + encounter-resolution time). The first two fixes were
confined to this study's own new scene geometry (parking zones,
marker node type) and were implemented directly, consistent with
established practice. This third finding implicates `max_runtime_s` --
one of the explicitly pre-registered, frozen parameters -- and/or the
gate-post geometry (which, unlike the parking zones, was frozen back in
Phase 1/revision 1 as part of the original scene design, not newly
authored this round). Per instruction, any revision to a pre-
registered parameter must be a deliberately reasoned, formula-derived
decision made BEFORE the next run, not tuned post-hoc to force a pass
-- this is reported to the user for that decision rather than adjusted
unilaterally.

## Disposition

- Native WSL bag + diag_logs preserved at
  `/home/eamon/epuck_comm_bags/shared_exit_n2_n2_exit_comm_off_EXCLUSIONARY_PILOT06`
  (+ `_diag_logs`) and a SHA-256-verified Windows copy under this
  directory's sibling `bags/` path (gitignored).
- Process cleanup confirmed clean after this pilot.
- `off_leak_detected=false`, `off_leak_check_message_count=0` --
  confirmed no `GoalAnnouncement` leak under COMM_OFF, unaffected by
  this finding.
- Phase 3.5 halted here pending the user's decision -- the ON pilot has
  NOT been run, per the pre-registered rule that COMM_OFF must first
  produce a valid result before COMM_ON is attempted.

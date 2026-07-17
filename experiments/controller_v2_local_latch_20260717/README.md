# controller_v2_local_latch_20260717

**Purpose**: `controller_v2` development regression evidence — first fix
attempt at the `controller_v1` local-obstacle-avoidance defect (bound
side-lane turn, fix retrigger defect). NOT formal comm-performance
statistics; NOT pooled with Phase 4 formal batches (category
02_controller_regression).

**Scenario**: static wooden box local avoidance (no moving peer).

**Trials** (2 bag directories):
- `static_box_pilot_a` — FAIL (per prior-session record; not re-verified this indexing pass)
- `static_box_pilot_a2` — PASS (per prior-session record; not re-verified this indexing pass)

**Config**: `config/static_box_v2/` — `run_static_box_v2_pilot.sh`,
`analyze_static_v2_controller_log.py`, and
`controller_v3_unified_encounter_design_20260717.md` (the design doc for
the NEXT controller revision, written after this one's remaining defects
were found).

**Included in dissertation**: NO — superseded by `controller_v3` and
`controller_v4`; retained purely as development-history evidence of the
defect-fix chain.

**Git commit**: `922a580` ("controller_v2_local_latch_20260717: bound
side-lane turn, fix retrigger defect").

**Next step at the time**: `controller_v3_unified_encounter_20260717`
(replace the direction-latch design with a single unified encounter state
machine) — see that directory.

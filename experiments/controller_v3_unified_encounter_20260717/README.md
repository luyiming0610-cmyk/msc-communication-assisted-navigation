# controller_v3_unified_encounter_20260717

**Purpose**: `controller_v3` development regression evidence — replaced
`controller_v2`'s direction-latch design with a single unified encounter
state machine. NOT formal comm-performance statistics; NOT pooled with
Phase 4 formal batches (category 02_controller_regression).

**Scenario**: static wooden box local avoidance (no moving peer).

**Trial** (1 bag directory):
- `static_box_a3` — pilot_a3, forensic diagnostic evidence. This is the
  trial whose forensic trajectory/mode-transition analysis directly
  motivated `controller_v4`'s full-sensor-bypass and command-gated
  turn-ledger redesign (per this session's earlier deep forensic
  investigation, which also retracted an initial incorrect hypothesis
  blaming CPA/peer avoidance for this trial's outcome — see prior-session
  notes; not re-derived during this indexing pass).

**Config**: `config/static_box_v3/` — `run_static_box_v3_pilot.sh`,
`analyze_static_v3_controller_log.py`.

**Included in dissertation**: NO — superseded by `controller_v4`; retained
as development-history / forensic evidence.

**Git commit**: `d2ef811` ("controller_v3_unified_encounter_20260717:
implement unified encounter state machine"), evidence at `40c23e3`.

**Next step at the time**: `controller_v4_full_sensor_bypass_20260717`
(full front→mid→rear sensor-sequence + encounter-local lateral-displacement
bypass, command-gated turn ledger) — see that directory.

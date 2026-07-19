# Condition D Trial 04 — batch STOP condition triggered (RECLASSIFIED)

**CORRECTION (2026-07-19, user-directed, offline-only)**: this trial's
verdict axes are corrected below. The original framing in this document
("a genuine, isolated single-message anomaly between the relay's ROS
publish and the rosbag recorder's capture") incorrectly implied the
message may not have reached any downstream consumer. It has since been
proven, via the independent online `sequence_counter` subscriber (a
SEPARATE ROS2 node subscribing to the same `/epuck2/state` topic as the
bag recorder), that sequence 17 **was received live**:
`epuck2_counter.json`'s `state` section shows `received_count=450`,
`unique_sequence_count=450` (the relay's full forwarded range), with zero
internal duplicates. The message left the relay AND was received by a live
subscriber. Only the **rosbag recording chain specifically** failed to
capture it. The correct, precise wording is: **"rosbag-only single-message
capture gap"** -- not "relay-to-downstream loss". This is NOT a
communication-impairment operational failure and NOT a task failure.

**Corrected verdict** (see `three_axis_verdict.json`, five-axis schema):

| axis | value |
|---|---|
| `DATA_ARTIFACT_INTEGRITY` | VALID |
| `MANIPULATION_VALIDITY` | **VALID** (relay: 0 drops, reordering genuinely induced, forwarded stream self-consistent, independently confirmed complete by the online counter) |
| `TASK_OUTCOME` | SUCCESS (no collision, margin +3.099mm) |
| `FORMAL_MEASUREMENT_VALIDITY` | **INVALID** (bag-capture-chain-only gap, sequence 17 epuck2→epuck1) |
| `FORMAL_BATCH_INCLUSION` | **EXCLUDED** |

**Exclusion reason** (verbatim): "ROSbag-only single-message capture gap:
sequence 17 was forwarded and received by the online counter but absent
from the bag. This violates the preregistered aligned-window bag capture
ratio=1.0 requirement."

This trial is now classified `EXCLUDED_MEASUREMENT_CHAIN_ATTEMPT` -- kept
in full (not deleted, not rerun, attempt01 not overwritten), excluded from
the formal n=5 count, and explicitly listed (not hidden) in any future
Condition D batch summary. D01, D02, D03 remain the currently valid formal
trials; D05 (and, contingent on D05 passing all axes, D06 as the sole
replacement for D04) continue the batch under the pre-declared replacement
rule.

---

## Original report text (preserved below, superseded by the correction above)

`objective5_impairment_matrix_v1_condition_D_trial04_attempt01` triggered an
explicit, pre-declared stop condition. Per instruction, the batch is halted
immediately: **D04 is not rerun, D05 is not started**, no parameter/
controller/relay/threshold was touched, and all evidence is preserved as-is.

## What triggered the stop

`MANIPULATION_VALIDITY = INVALID` for the `epuck2_to_epuck1` direction:

- `actual_missing_count = 1` (sequence **17**) — an explicit stop condition
  (`actual_missing_count`不为0).
- `aligned_window_forwarded_to_bag_capture_ratio = 0.9976958525345622`
  (433/434) — not equal to the expected 1.0, another explicit stop condition.

All other checks for this trial were otherwise clean: `DATA_VALIDITY=VALID`,
`TASK_OUTCOME=SUCCESS` (`min_interrobot_distance_m=0.14309905640064896`,
margin +3.099mm), `epuck1_to_epuck2` direction fully clean (missing=0,
ratio=1.0), relay `total_drop_count=0` both directions, queue drained,
realtime factor in band, process cleanup CLEAN, all 4 behavioral SHA-256
matched the frozen values, directory was newly created (no overwrite).

## Root-cause check (read-only, no code/data changed)

Verified directly against the raw `epuck2_relay.csv` (not through any
analyzer): row `17,forwarded,0.056478,15.260000,15.316478,15.260000,15.320000,`
— the relay genuinely **received and forwarded** sequence 17 at
`actual_release_time_s=15.32`, with no `drop_reason`. The message left the
relay. It never appears in the bag's `/epuck2/state` topic (confirmed via
direct bag sequence extraction: bag set for that topic has no 17, min=16,
max=449). `bag_sequences_not_in_forwarded_set=[]` rules out a bag/relay
sequence-space mismatch — this is a message that was forwarded but not
captured, not a set-alignment bug in the audit tool.

**This is NOT the reordering-accounting bug fixed for D01–D03** (that bug
was in `sequence_counter.py`'s adjacent-delta gap accounting; here the
set-based, reorder-safe analyzer itself reports a genuine single-sequence
gap within the aligned window). It is a distinct, single-message anomaly
between the relay's ROS publish and the rosbag recorder's capture of
`/epuck2/state` — a genuine, isolated transport/capture loss (subscriber-side
DDS drop or a rosbag recorder gap), not reproduced in any other direction or
trial in this session (A, B, C, D01–D03 all had 0 such gaps).

`bag_record.log` reported "no drop/warn/error" — that check only covers
`ros2 bag record`'s own internal logging, not DDS-level subscription drops
upstream of the recorder, so it does not contradict this finding.

## What is NOT concluded here

- Not concluded to be a controller, relay, or protocol defect — no code was
  changed to investigate further, per the explicit "stop, do not fix by
  tuning" instruction.
- Not concluded to be reproducible or a property of Condition D generally —
  n=1 occurrence out of 4 valid Condition D runs (D01–D04) so far.
- Not silently retried, discarded, or reinterpreted as acceptable.

## Preserved evidence

- Native WSL bag + diag_logs: `/home/eamon/epuck_comm_bags/objective5_impairment_matrix_v1_condition_D_trial04_attempt01[_diag_logs]`
- Windows SHA-256-verified copy: `experiments/05_objective5_impairment_matrix/bags/objective5_impairment_matrix_v1_condition_D_trial04_attempt01/` (gitignored)
- Full analysis directory (this directory): `matrix_analysis.json` (with
  METHOD_INVALID annotation), `reordering_delivery_audit.json` (the
  authoritative reorder-safe result showing the gap), `trigger_mechanism_audit.json`,
  `startup_sync_audit.json`, `three_axis_verdict.json`,
  `objective5_condition_D_trial04_trajectory_audit.json`, `final_verdict.json`,
  `runtime_manifest.json`, `summary.md`, `README.md`.

## Boundary

D02 and D03 both independently passed all three axes
(`DATA_VALIDITY=VALID`, `MANIPULATION_VALIDITY=VALID`, `TASK_OUTCOME=SUCCESS`)
and remain valid, standalone formal trials regardless of D04's outcome. No
Condition D n=5 batch summary is generated, since D02–D05 did not all pass.
D05 is **not started**. Awaiting user instruction on how to proceed (e.g.
whether to investigate the single-message gap further, whether D04 counts as
a genuine data point to keep as-is, or how the batch should continue).

# Condition D formal batch summary (FORMAL_SIM, n=5)

Fixed `delay_s=0.15`, jitter `jitter_s=0.30` (full peak-to-peak spread,
i.e. per-message release delay uniformly spread across `[0, 0.30]s`),
`drop_probability=0.0`, outage disabled. Same frozen controller, world,
thresholds, and behavioral-code SHA-256 as Conditions A/B/C.
`manual_observation.status=NOT_OBSERVED` for every trial in this batch
(user did not directly observe any Condition D trial; D01's final
acceptance was confirmed by the user from automated evidence after an
initial sequence-accounting correction).

## Verdict schema (five axes, corrected 2026-07-19)

Verdicts use five independent axes, replacing an earlier single strict-gate
label that was invalid under reordering:

- **DATA_ARTIFACT_INTEGRITY** -- schema/bag-mechanics self-consistency
  (`measurement_validity`, `legacy_replay=false`, p99 finite, queue drained,
  realtime factor in band, no bag drop/warn/error).
- **MANIPULATION_VALIDITY** -- did the relay genuinely apply the configured
  jitter with zero relay-level loss and genuine reordering? Judged from the
  relay's own CSV (received/forwarded/dropped) and cross-checked against
  the independent online `sequence_counter` subscriber -- deliberately NOT
  gated on whether the bag recorder specifically captured every message.
- **TASK_OUTCOME** -- collision-free completion, safety margin.
- **FORMAL_MEASUREMENT_VALIDITY** -- did the ROSBAG RECORDING CHAIN
  specifically capture every relay-forwarded message within the aligned
  window (`aligned_window_forwarded_to_bag_capture_ratio=1.0`)? This is a
  distinct, downstream, single-sink question from MANIPULATION_VALIDITY.
- **FORMAL_BATCH_INCLUSION** -- INCLUDED only if all four axes above pass.

All missing/capture figures use the reorder-safe, set-based analyzer
(`tools/reorder_safe_delivery_analyzer.py`), NOT the live
`sequence_counter.py`'s adjacent-delta accounting, which is METHOD_INVALID
under reordering (see each trial's `matrix_analysis.json` annotation and
`final_verdict.json`'s `LEGACY_METHOD_INVALID_UNDER_REORDERING` block --
original values preserved, never deleted, just correctly labeled).

## Included trials (5/5)

| trial | seeds (e1→e2/e2→e1) | DATA_ARTIFACT_INTEGRITY | MANIPULATION_VALIDITY | TASK_OUTCOME | FORMAL_MEASUREMENT_VALIDITY | min_interrobot_distance_m | safety_margin |
|---|---|---|---|---|---|---:|---:|
| 01 | 4001/14001 | VALID | VALID | SUCCESS | VALID | 0.1444723957613864 | +4.472mm |
| 02 | 4002/14002 | VALID | VALID | SUCCESS | VALID | 0.1516223239385512 | +11.62mm |
| 03 | 4003/14003 | VALID | VALID | SUCCESS | VALID | 0.14016992055075728 | **+0.170mm** |
| 05 | 4005/14005 | VALID | VALID | SUCCESS | VALID | 0.14919158786608294 | +9.19mm |
| 06 | 4006/14006 | VALID | VALID | SUCCESS | VALID | 0.14431858718680218 | +4.32mm |

**All 5 → `FORMAL_BATCH_INCLUSION=INCLUDED`.**

### D03 -- RAZOR-THIN MARGIN, explicitly flagged

D03's safety margin is only **+0.170mm** (`min_interrobot_distance_m=
0.14016992055075728` vs `safety_radius_m=0.14`). This is a genuine,
unadjusted **threshold pass** -- not rerun, not modified, no
parameter/threshold change made to improve it. It must **never** be
characterized as a comfortable or robust safety margin in any downstream
writeup; it is the tightest recorded margin across the entire Condition D
batch and among the tightest across the whole A-D matrix so far.

### D06 -- sole preregistered replacement for excluded D04

D06 (seeds `4006`/`14006`, `trial_index=6`) is the **sole authorized
replacement trial** for D04, which was retained as valid data but excluded
from the formal n=5 (see below). The seed pair was added to
`objective5_impairment_matrix_conditions.csv`'s Condition D row for exactly
this purpose (commit `fa47182`); `n_trials` for Condition D remains `5` --
these five trials (D01, D02, D03, D05, D06) are the complete, final formal
set. D06's `runtime_manifest.json` records `replacement_for=D04` and the
conditions-CSV SHA-256 before/after the seed-extension edit.

## Excluded trial (not counted toward n=5)

| trial | seeds | DATA_ARTIFACT_INTEGRITY | MANIPULATION_VALIDITY | TASK_OUTCOME | FORMAL_MEASUREMENT_VALIDITY | classification |
|---|---|---|---|---|---|---|
| **04** | 4004/14004 | VALID | VALID | **SUCCESS** | **INVALID** | **`EXCLUDED_MEASUREMENT_CHAIN_ATTEMPT`** |

D04's task itself **SUCCEEDED** (no collision, `min_interrobot_distance_m=
0.14309905640064896`, margin +3.099mm) and the communication manipulation
itself was genuinely and correctly applied (relay: 0 drops, reordering
induced both directions, independently confirmed complete by the online
`sequence_counter` subscriber -- `epuck2_counter.json`: `received_count=
unique_sequence_count=450`, exactly matching the relay's full forwarded
count). D04 was excluded for exactly one reason: a **rosbag-only
single-message capture gap** -- sequence 17 (`epuck2_to_epuck1`) was
genuinely forwarded by the relay and received live by the independent
online counter, but the rosbag recording chain specifically never captured
it (`aligned_window_forwarded_to_bag_capture_ratio=0.9976958525345622`,
not 1.0). This is **not** a communication-manipulation failure and **not**
a task failure -- it is a downstream, single-sink recording-chain artifact.
D04 is preserved in full (raw bag, all analysis files, a dedicated
`STOP_CONDITION_REPORT.md` with a correction section), not rerun, not
deleted, and explicitly listed here rather than hidden.

## Communication metrics (reorder-safe, both directions, all 5 included trials)

| trial | dir | relay forwarded (unique) | relay dropped | actual_missing | out_of_order (relay fwd, adjacent) | aligned_window capture_ratio | median age (s) |
|---|---|---:|---:|---:|---:|---:|---:|
| 01 | e1→e2 | 434 | 0 | 0 | 89 | 1.0 | 0.16 |
| 01 | e2→e1 | 430 | 0 | 0 | 92 | 1.0 | 0.16 |
| 02 | e1→e2 | 430 | 0 | 0 | 91 | 1.0 | 0.16 |
| 02 | e2→e1 | 419 | 0 | 0 | 83 | 1.0 | 0.16 |
| 03 | e1→e2 | 433 | 0 | 0 | 80 | 1.0 | 0.18 |
| 03 | e2→e1 | 440 | 0 | 0 | 78 | 1.0 | 0.16 |
| 05 | e1→e2 | 438 | 0 | 0 | 103 | 1.0 | 0.16 |
| 05 | e2→e1 | 445 | 0 | 0 | 94 | 1.0 | 0.16 |
| 06 | e1→e2 | 429 | 0 | 0 | 96 | 1.0 | 0.16 |
| 06 | e2→e1 | 436 | 0 | 0 | 94 | 1.0 | 0.16 |

**All 10 direction-measurements**: `actual_missing_count=0`,
`relay dropped=0`, `out_of_order>0` (reordering genuinely applied, per
trial per direction), `aligned_window_forwarded_to_bag_capture_ratio=1.0`.
Message-age median is `0.16s` in 9/10 direction-measurements (D03's
e1→e2 is `0.18s`), consistent with the configured `[0, 0.30]s` jitter
range and clearly wider than Condition B's fixed `0.20s` or Condition A's
near-zero delay. All `p99_message_age_s` values are finite under the strict
schema gate.

## Trigger / trajectory (edge-based, both directions, all 5 included trials)

`first_trigger_reason=PREDICTED_CPA` and `LOCAL_*` all zero in every one of
the 5 included trials (and in D04) -- **PURE_COMMUNICATION_CPA_AVOIDANCE,
6/6 attempted trials.** Edge/transition-based mode counting (not raw
log-line grep, which over-counts due to periodic diagnostic lines) confirms
each robot in each trial has exactly **one**
`CRUISE→AVOID_TURN→AVOID_PASS→RECOVER→COMPLETE` cycle.

| trial | e1 reversals | e2 reversals | e1 path efficiency | e2 path efficiency |
|---|---:|---:|---:|---:|
| 01 | 1 | 1 | 0.9998 | 0.9997 |
| 02 | 2 | 1 | 0.9995 | 0.9998 |
| 03 | 1 | 1 | 0.9999 | 0.9997 |
| 05 | 1 | 1 | 0.9998 | 0.9996 |
| 06 | 1 | 2 | 0.9998 | 0.9996 |

D01-D06's angular-reversal counts (1-2 per robot) and near-ideal path
efficiency (0.9995-0.9999) are consistent across the whole Condition D set
-- Condition D does **not** reproduce Condition C's 9-reversal repeated
S-curve at any of the 6 attempted trials, despite Condition D's genuine
reordering (out_of_order 78-103 per direction across the batch). This is
descriptive only; the oscillatory degradation in Condition C appears to
track delay MAGNITUDE (1.00s fixed) more than jitter/reordering per se
(D's mean configured delay is only 0.15s), but this observation is not
statistically established at n=5 per condition.

## Descriptive comparison against Conditions A/B/C (n=5 per condition, descriptive only; no significance claimed)

| condition | delay/jitter | typical out_of_order | typical message-age | typical safety margin | notable finding |
|---|---|---:|---:|---:|---|
| A | 0 / 0 | 0 | ~0s | mean 6.73mm (min 1.79mm) | control; no reordering by construction |
| B | 0.20 / 0 | 0 | 0.20s fixed | mean 4.66mm (min 1.89mm) | no reordering by construction |
| C | 1.00 / 0 | 0 (but repeated internal S-curve) | 1.00s fixed | mean 3.734mm (**1/5 UNSAFE_FAILURE**, C05 −1.091mm) | 9 angular reversals/trial, oscillatory degradation |
| **D** | 0.15 / 0.30 | **89-103 per direction (genuine reordering)** | 0.16-0.18s (spread [0,0.30]) | +0.170mm to +11.62mm, **all 5 positive** | reordering does NOT reproduce C's oscillation; margins vary widely trial-to-trial, including one razor-thin pass (D03) |

Condition D's margins span a wider range than B's (min 1.89mm to max
7.77mm) despite D's mean configured delay (0.15s) being smaller than B's
fixed 0.20s -- consistent with jitter/reordering introducing additional,
less predictable timing variance into the CPA decision, even though the
average avoidance behavior (reversal counts, path efficiency) stays closer
to B's than to C's. Not statistically established at n=5; a candidate
finding for the dissertation's discussion section, to be corroborated (or
not) by Conditions E/F/G.

## Realtime factor / duration

Preload 0.955-1.004, full-load 0.948-0.975 across the 5 included trials
(all in the `[0.8, 1.2]` band). Bag duration approximately 52-56s per
trial.

## Code identity (frozen, identical across all 6 attempted trials)

- orchestrator: `20d2ef63a152a7d65632e4fd3414c9cd1cdaa2a449f58daf7eac1bd28110913b`
- network_impairment_relay.py: `f5d408bc3379f79fa70628370b4dfb6d537c4d03a1968fe8dc75a691c3e6d5ff`
- network_impairment.py: `253e0d960e9b587a3c5e60587ce7ac56c167fd6aba1c98f8b7b940e821210561`
- sequence_counter.py: `57bb0699a444df644d75c4e834b5fd13b5f15a6283d7b1d276ec0b65674f1fd3`
- Identical to Conditions A/B/C's frozen values across D01-D06. Only the
  `objective5_impairment_matrix_conditions.csv` configuration file changed
  during this batch (commit `fa47182`, adding the sole D06 replacement
  seed pair `4006`/`14006` to Condition D's row; no other row touched,
  `n_trials` unchanged at 5) -- never a behavioral-code change.

## Batch status

**Condition D formal n=5 batch: COMPLETE, 5/5 INCLUDED** (D01, D02, D03,
D05, D06). D04 is preserved and explicitly listed as
`EXCLUDED_MEASUREMENT_CHAIN_ATTEMPT` (rosbag-only capture gap, not a
communication or task failure) -- not counted toward the n=5, not deleted,
not rerun. Conditions E-G have not started and are not auto-started by this
batch's completion.

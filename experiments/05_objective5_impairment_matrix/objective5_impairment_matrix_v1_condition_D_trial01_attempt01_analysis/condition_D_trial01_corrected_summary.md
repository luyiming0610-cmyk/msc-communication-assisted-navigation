# Condition D Trial 01 — corrected verdict summary

Corrects the sequence accounting and the reused strict-gate verdict for
`objective5_impairment_matrix_v1_condition_D_trial01_attempt01`. Offline and
read-only: no Webots run, no parameter change, no rerun, no modification of the
raw bag/logs or of the frozen controller/relay/world/thresholds/seeds. The
original `matrix_analysis.json` and `final_verdict.json` are preserved; the
disputed sequence fields are annotated METHOD_INVALID, not deleted.

## Naming correction (2026-07-19, statistics-naming-only, no value change)

- `forwarded_to_bag_capture_ratio` → **`aligned_window_forwarded_to_bag_capture_ratio`**:
  epuck1→epuck2 aligned window 20–433 = 414/414 = 1.0; epuck2→epuck1 aligned
  window 18–440 = 423/423 = 1.0. The relay's full-lifetime forwarded counts
  (434/430) differ from the bag counts (414/423) because the bag recorder
  started a few messages *after* the relay began forwarding — a recording
  start boundary, not loss. This ratio proves no capture loss *within the
  aligned window*; it does not characterize the bag's capture rate over the
  relay's entire lifetime.
- `source_to_forwarded_delivery_ratio` → **`relay_received_to_forwarded_ratio`**:
  the denominator is the relay's own CSV-received count, not the full
  `/epuckN/state_raw` source-side publication lifecycle. It is not a
  complete source-to-relay PDR unless the source and relay-input sequence
  sets are separately aligned (not attempted here).

## manual_observation

`status = NOT_OBSERVED`, `reason = user did not directly observe this trial`.
No manual-observation content is inferred, back-filled, or inherited. The
automated task result and the manual-observation status are kept separate.

## Three-axis verdict (replaces the reused single strict-gate label)

The earlier `final_verdict.json` returned `FAIL` only because it reused the
A/B/C strict gate requiring `out_of_order_count == 0`. That criterion is
**inverted** for a reordering condition and must not be used to fail D.

| axis | value |
|---|---|
| **DATA_VALIDITY** | VALID |
| **MANIPULATION_VALIDITY** | VALID (reordering genuinely applied, no unplanned loss) |
| **TASK_OUTCOME** | SUCCESS (no collision; min distance 0.14447 m, margin +4.472 mm) |

Condition-D-specific pass criteria — all met: `out_of_order > 0`,
`actual_missing = 0` (as configured), relay drop = 0,
`forwarded_to_bag_capture_ratio = 1.0`, `queue_drained = true`,
`DATA_VALIDITY = VALID`, `TASK_OUTCOME = SUCCESS`.

## Corrected sequence/delivery numbers

| metric | epuck1→epuck2 | epuck2→epuck1 |
|---|---:|---:|
| relay forwarded (unique) | 434 | 430 |
| relay dropped | 0 | 0 |
| bag received (unique) | 414 | 423 |
| **actual_missing_count** (was 189 / 192) | **0** | **0** |
| duplicate_count | 0 | 0 |
| out_of_order (adjacent / displaced, forwarded) | 89 / 92 | 92 / 97 |
| **forwarded_to_bag_capture_ratio** (was 1.0 / 1.00233) | **1.0** | **1.0** |
| source_to_forwarded_delivery_ratio | 1.0 | 1.0 |

### Root cause of the original 189/192 and ratio > 1

- **missing 189/192** — `sequence_counter.py` accumulates adjacent
  forward-jumps into `sequence_gap_count` and never reconciles them when the
  skipped sequence arrives out of order. Under reordering this is not loss.
  True missing = 0/0 (relay dropped nothing; every forwarded sequence in the
  bag window was captured).
- **capture_ratio 1.00233 (epuck2→epuck1)** — `expected_count` used the first
  **arrived** sequence (12) as the minimum, but the relay forwarded sequence
  **11**, which arrived later out of order. Denominator one too small →
  430/429 > 1. With the true minimum (11): expected = 430 = unique → 1.0.

## Trigger / trajectory (edge-based, corrected)

- `first_trigger_reason = PREDICTED_CPA`; all `LOCAL_*` counts 0 → no
  ToF/IR local-safety-layer takeover.
- Edge/transition-based mode counting (not raw log-line grep): each robot had
  exactly **one** `CRUISE→AVOID_TURN→AVOID_PASS→RECOVER→COMPLETE` cycle. The
  earlier grep counts (AVOID_TURN=8, AVOID_PASS=86, RECOVER=10) were per-tick
  diagnostic lines within a single sustained mode, not independent triggers.
- Within the single AVOID_PASS: 1 angular-direction reversal per robot, path
  efficiency ≈ 0.9998/0.9997 — resembles Condition B, **not** Condition C's
  9-reversal S-curve. Descriptive only, n=1; confirm with D02–05.

## Latency / throughput (unaffected by the bug, still valid)

Message age median 0.16 s, max 0.30 s both directions
(`VALID_AT_SIM_CLOCK_RESOLUTION`, p99 finite) — measurably wider than A (~0),
B (0.20 s fixed), C (1.00 s fixed), consistent with the configured [0, 0.30] s
jitter range. Throughput ~708–709 bytes/s both directions.

## Can D01 be a formal n=5 trial despite NOT_OBSERVED?

**Yes, as a formal automated-measurement trial.** DATA_VALIDITY=VALID,
MANIPULATION_VALIDITY=VALID, TASK_OUTCOME=SUCCESS, and all reorder-safe metrics
are self-consistent and reproducible from preserved raw evidence. Condition A
Trials 02–05 and Condition B Trials 02–05 were likewise not individually
observed yet count as formal automated trials; the gating requirement for
inclusion has always been DATA_VALIDITY=VALID plus a clean, reproducible
measurement chain — not manual observation. The `NOT_OBSERVED` status is
recorded honestly and does not, by itself, exclude the trial. (Manual
observation remains available to the user as an independent cross-check for
any trial they choose to watch.)

## Boundary

Correction complete. D02–D05 are **not** started. D01 is not rerun. Only after
the corrected D01 is accepted will D02–D05 be authorized.

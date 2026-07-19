# Condition D Trial 01 — reorder-safe delivery/sequence audit

Offline, read-only correction of the sequence accounting for
`objective5_impairment_matrix_v1_condition_D_trial01_attempt01`. No Webots
run, no parameter change, no modification of the raw bag/logs or of the
frozen controller/relay/world. Tool:
`experiments/05_objective5_impairment_matrix/tools/reorder_safe_delivery_analyzer.py`
(v1, set-based), unit-tested by `test_reorder_safe_delivery_analyzer.py`
(12/12 pass). Machine-readable output: `reordering_delivery_audit.json`.

## Why the original figures were wrong

The live `sequence_counter.py` computes missing/expected from **adjacent
arrival deltas**, which is only correct for an in-order stream. Under
Condition D (jitter → genuine reordering) it fails two ways:

1. **Forward-jump accumulation never reconciled.** When a later sequence
   arrives before an earlier one, `sequence_gap_count += seq - prev - 1`.
   When the skipped sequence later arrives out of order, it only increments
   `out_of_order_count` — the earlier "gap" is never removed. A
   **lossless-but-reordered** stream therefore reports a large bogus missing
   count. Here: reported `missing_count = 189 / 192`; **true value = 0 / 0**.

2. **First arrival used as the minimum sequence.** `expected_count =
   last_arrived − first_arrived + 1`. If the first message to *arrive* is not
   the smallest sequence (a lower one arrives later, out of order), expected
   is too small and `capture_ratio = unique / expected` exceeds 1. Here the
   epuck2→epuck1 relay forwarded seq **11** but seq **12 arrived first**, so
   the counter used min=12 → expected=429, with unique=430 →
   `capture_ratio = 430/429 = 1.0023310023310024` (mathematically impossible).

## Ground-truth reconstruction

Reconstructed per-message sequence **sets** from the authoritative sources —
the two relay CSVs (`received_seq` + `action`) and the delivered bag topics
(`/epuck1/state`, `/epuck2/state`); `sequence_counter` JSON used only as a
cross-reference. Source (`state_raw`) sets confirm the relays' inputs.

| direction | relay fwd set | dropped | fwd min–max | fwd true-min = first-arrival? | bag set | bag min–max |
|---|---:|---:|---:|:--:|---:|---:|
| epuck1→epuck2 | {0..433} = 434 | 0 | 0–433 | yes (0) | 414 | 20–433 |
| epuck2→epuck1 | {11..440} = 430 | 0 | 11–440 | **no** (arr 12, min 11) | 423 | 18–440 |

The bag windows (20–433 and 18–440) start after the relay's first forwarded
sequence because rosbag began recording a few messages into the run — a
**recording-window** offset, explicitly reported, not loss. Metrics are
computed over each stream's own true [min, max] and over the common
forwarded∩bag window.

## Corrected metrics (both directions)

| metric | epuck1→epuck2 | epuck2→epuck1 |
|---|---:|---:|
| relay forwarded (unique) | 434 | 430 |
| relay dropped | 0 | 0 |
| `source_to_forwarded_delivery_ratio` | 1.0 | 1.0 |
| bag delivered (unique) | 414 | 423 |
| **actual_missing_count** | **0** | **0** |
| duplicate_count | 0 | 0 |
| out_of_order (adjacent inversions) | 89 (fwd) / 87 (bag) | 92 (fwd) / 90 (bag) |
| out_of_order (displaced) | 92 (fwd) / 90 (bag) | 97 (fwd) / 95 (bag) |
| **`forwarded_to_bag_capture_ratio`** | **1.0** | **1.0** |
| ratio in [0,1]? | yes | yes |
| data_validity (accounting) | VALID | VALID |

`out_of_order` uses two independent flavours (adjacent inversion; displaced
below running-max) — both are strictly reordering measures and are never
counted as loss. The forwarded-stream reversal counts (89 / 92) match the
original `sequence_counter.out_of_order_count`, confirming the reordering
itself was measured correctly all along; only the missing/capture derivation
was broken.

## Root cause, in one line each

- **missing = 189 / 192**: unreconciled adjacent forward-jump accumulation in
  `sequence_counter.py` (`sequence_gap_count`), which is not real loss under
  reordering. True missing = **0 / 0** (every forwarded sequence in the bag
  window was captured; relay dropped nothing).
- **capture_ratio = 1.00233 (epuck2→epuck1)**: `expected_count` used the first
  **arrived** sequence (12) as the minimum instead of the true minimum (11),
  making the denominator one too small. With the true min, expected = 430 =
  unique → **1.0**.

## Scope

This audit corrects only the sequence-integrity/delivery accounting. Latency
(`VALID_AT_SIM_CLOCK_RESOLUTION`, mean ≈ 0.16 s, max ≈ 0.30 s), throughput,
relay counts, queue drain, realtime factor, and task behaviour in
`matrix_analysis.json` were never affected by this bug and remain valid. The
disputed fields there are annotated METHOD_INVALID (see
`matrix_analysis.json`'s `_method_validity_annotation`), preserved but no
longer used as Condition D loss metrics.

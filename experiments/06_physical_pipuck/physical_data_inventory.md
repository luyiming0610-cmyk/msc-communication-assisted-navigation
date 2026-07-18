# Physical experiment data inventory (2026-07-18, read-only)

Source directory:
`C:\Users\路一鸣\Desktop\硬件实验毕设\real_robot_avoidance_v1\experiment_data_20260715\`

Classification legend, superseded 2026-07-18 per explicit instruction: old
JSON `/epuck1/state`-family data is labeled **only** `LEGACY_PHYSICAL_EVIDENCE`
(replacing the earlier dual `HISTORICAL_PHYSICAL_VALIDATION`+
`LEGACY_PROTOCOL_ONLY` labeling in a prior pass of this document). This is
NOT a result under the current `EpuckState` protocol and must never be
pooled with, compared against, or presented alongside any future formal
batch's statistics. Other possible labels (unused below): `EXCLUDED`,
`FORMAL_PAPER_ELIGIBLE` (none of this data qualifies),
`CURRENT_PROTOCOL_COMPATIBLE` (none of this data qualifies).

None of these batches were re-run this pass. None are re-classified as
`FORMAL_PAPER_ELIGIBLE` — see rationale below.

| bag directory | classification | reason |
|---|---|---|
| `ground_right_ir_01` | `LEGACY_PHYSICAL_EVIDENCE` | pre-placed right-front obstacle, single trial. Real robot, real wireless link, no collision, RTT mean ~65.43ms / max ~68.93ms, connection rate 100%, CRC delta 0. Startup-phase trial (obstacle pre-existed before motion), not usable for reaction-time statistics per the batch's own summary. Uses old JSON `/epuck1/state`-family logging, not current `EpuckState` protocol. |
| `ground_left_dynamic_01` | `LEGACY_PHYSICAL_EVIDENCE` | dynamic left-front obstacle, single trial. First-TOF-warning-to-first-turn ~91.89ms, min TOF 0.038m, RTT mean ~65.70ms / max ~76.83ms, connection rate 100%, CRC delta 0. Single trial only, not a repeated-condition batch. |
| `ground_center_dynamic_pdr_01` | `LEGACY_PHYSICAL_EVIDENCE` | dynamic center obstacle, first trial with precise sequence-delivery stats: 99 unique state packets, 0 missing, 0 out-of-order, 100% delivery ratio. RTT min ~7.03ms / mean ~18.26ms / p95 ~62.31ms / max ~120.76ms. First-warning-to-first-turn ~12.63ms. Included in the 4-trial repeat summary below. |
| `ground_center_dynamic_pdr_02_retry1` | `EXCLUDED` | a retry attempt for the center-dynamic repeat batch; not included in `center_dynamic_repeat_summary_20260715.md`'s "4 valid trials" (`pdr_01`, `03_clean`, `04_clean`, `05_clean`). Exact retry reason not re-verified this pass (log exists at `logs/ground_center_dynamic_pdr_02_retry1.log` but was not read line-by-line this pass); excluded per the existing summary document's own selection, not re-decided here. |
| `ground_center_dynamic_pdr_02_retry2` | `EXCLUDED` | same as above — second retry of trial 02, also excluded from the 4-valid-trial summary; not re-verified line-by-line this pass. |
| `ground_center_dynamic_pdr_03_clean` | `LEGACY_PHYSICAL_EVIDENCE` | part of the 4-trial valid repeat batch (see `center_dynamic_repeat_summary_20260715.md`): 4/4 successful avoidance, 4/4 no collision, 4/4 full auto-stop, all turned right, 0 negative/reverse commands. |
| `ground_center_dynamic_pdr_04_clean` | `LEGACY_PHYSICAL_EVIDENCE` | part of the same 4-trial valid repeat batch. |
| `ground_center_dynamic_pdr_05_clean` | `LEGACY_PHYSICAL_EVIDENCE` | part of the same 4-trial valid repeat batch. |

## Aggregate figures for the 4-trial center-dynamic repeat batch

(from `center_dynamic_repeat_summary_20260715.md`, not recomputed this pass)

- Success/no-collision/full-auto-stop: 4/4 (100%) each.
- First-warning-to-first-turn: 64.25 ± 38.96 ms (range 12.63–98.02 ms).
- Min TOF distance: 0.0513 ± 0.0068 m (range 0.043–0.059 m).
- Path length: 0.03151 ± 0.00339 m.
- Sensor rate: 8.92 ± 0.16 Hz.
- Mean-of-trial-mean RTT: 35.31 ms.
- Connection rate 100%, 0 missing/out-of-order state packets, 0 CRC errors,
  across all 4 trials.

## Why nothing here is `FORMAL_PAPER_ELIGIBLE` or `CURRENT_PROTOCOL_COMPATIBLE`

Per `physical_protocol_gap_report.md` (same pass): all of this data was
produced by the old JSON `/epuck1/state`-family publishers/loggers, not the
current frozen `EpuckState.msg` (`protocol_v1.1_stamp_semantics`). The two
are not field-compatible (wrong hardcoded `source`, finite- vs.
`+Inf`-for-no-detection convention, no `validity_flags`, no protocol version,
no v4 zones — see the gap report for the full table). **None of it may be
pooled with, compared against, or presented alongside current or future
Objective 5 formal statistics.** It is retained and labeled
`LEGACY_PHYSICAL_EVIDENCE`: real robot, real wireless bridge, real sensors
and motors, safe stops, no collisions — useful as qualitative prior
evidence that the physical link and safety mechanisms work, nothing more.

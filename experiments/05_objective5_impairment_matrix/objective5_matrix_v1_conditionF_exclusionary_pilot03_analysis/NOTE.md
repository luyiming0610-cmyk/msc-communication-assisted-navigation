# Superseded: `ANALYZER_OK` was a hardcoded placeholder at the time of this run

This pilot originally ran before `matrix_analyzer.py` existed --
`ANALYZER_OK="true"` was a hardcoded placeholder, never actually
checked. **Preserved as-is (not deleted), but superseded by the real
analysis-only replay in `matrix_analysis_v2.json`**, produced by the
same `analyze_trial()` function the orchestrator itself now calls for
every trial (commit `f4e8d6c`), reading this run's already-recorded
relay CSVs and `sequence_counter.py` JSON output -- Webots was NOT
re-run.

`matrix_analysis_v2.json` result: `measurement_validity="VALID"`
(genuine `ANALYZER_OK=true`). Both directions:
`latency_measurement_status="RESOLUTION_LIMITED"` (Condition F's own
delay_s/jitter_s are both 0.0 outside the outage mechanism, so a
near-zero observed age is the frozen-correct classification, not a
clock-domain-mismatch label); `mean_message_age_s≈0.0175s`/`≈0.0169s`
(epuck1_to_epuck2/epuck2_to_epuck1); `capture_ratio≈0.9536`/`≈0.9541`
-- both match the outage drop counts exactly (30 missing = 30
`outage_drop_count`, confirming the sequence/relay cross-check is
self-consistent); `relay_forwarded_matches_consumer_received=true`
both directions. **Honest limitation**: `p99_message_age_s` is `null`
in this replay for the same reason as the Condition A pilot -- the
original `sequence_counter.py` binary predates the p99 field.

# Condition F bidirectional synchronization: reconstructed per-window evidence

Two independent lines of evidence, kept separate on purpose:

## 1. Code-level guarantee (proof, not measurement)

`ImpairmentDecider.outage_status(elapsed_s)` is a pure, stateless
function of `elapsed_s`; `network_impairment_relay.py` sets
`elapsed_s = self._now_s()` — the raw absolute value of the shared
`/clock` topic — for both relay instances, not a value relative to each
instance's own construction time. `test_both_relay_instances_reading_same_sim_clock_see_synchronized_outage_windows`
constructs two relay instances 3.0 simulated-wall-clock seconds apart
and confirms both classify the same absolute sim-time instant
identically. This is a proof about the mechanism, true regardless of
whether any given trial happens to have a message in flight during a
given window.

## 2. Reconstructed per-window timestamps from this run's raw data (measurement)

The native (not git-tracked) per-message relay CSV logs
(`epuck1_relay.csv`, `epuck2_relay.csv` — columns include
`receive_time_s` in sim time and `drop_reason`) let the actual outage
windows be reconstructed: group each direction's `drop_reason=="outage"`
rows into contiguous runs by `receive_time_s`; window start = first
outage-tagged receive time, window end = last outage-tagged receive
time before the next non-outage message.

Result for this run (5 windows detected in each direction):

| window | epuck1→epuck2 start/end | epuck2→epuck1 start/end | start dev | end dev |
|---|---|---|---|---|
| 0 | 25.0400 / 25.6000 | 25.0400 / 25.6000 | 0.0000s | 0.0000s |
| 1 | 40.0600 / 40.6000 | 40.0600 / 40.6000 | 0.0000s | 0.0000s |
| 2 | 55.0600 / 55.6000 | 55.0600 / 55.6000 | 0.0000s | 0.0000s |
| 3 | 70.1000 / 70.6600 | 70.1000 / 70.6600 | 0.0000s | 0.0000s |
| 4 | 85.1000 / 85.6600 | 85.1000 / 85.6600 | 0.0000s | 0.0000s |

Max observed start-time deviation: **0.0000s**. Max observed end-time
deviation: **0.0000s**.

## Honest caveats on the measurement (not the mechanism)

- **Window count**: the design doc predicted the first outage at
  `outage_phase_s=10.0`, but only 5 windows were detected here, starting
  at 25.04s, not 6 starting at ~10s. The most likely explanation: no
  message happened to be received during the `[10.0, 10.7)` window
  (state_raw/relay startup takes a few seconds after the sim clock
  begins advancing), so that window left no `drop_reason="outage"` row
  to reconstruct from. **This is a limitation of the reconstruction
  method (it can only see windows during which at least one message was
  attempted), not evidence that the outage mechanism itself skipped or
  misfired that window** — `outage_status()` is a pure function of
  elapsed_s and does not depend on whether a message happens to arrive.
- **Window end precision**: the reconstructed "end" is the timestamp of
  the *last dropped message observed*, a lower bound on the true
  `outage_duration_s` boundary (messages arrive at discrete ~0.115s
  intervals, not continuously) — not the exact instant `_in_outage()`
  flips back to `False`. The **agreement between the two directions**
  (both use the same discretization, so both windows' end estimates are
  biased the same way) is still a valid, honest measurement of relative
  synchronization; it is not a claim of sub-message-period absolute
  precision.
- **What this evidence supports**: "the code-level synchronization
  mechanism has been verified (shared absolute sim clock, proven by a
  dedicated unit test), and this pilot's actual reconstructed outage
  windows are consistent with that mechanism — zero measured deviation
  between directions across all 5 comparably-reconstructed windows."
  **What it does not support**: a claim that every outage window's
  boundary is aligned to sub-message-period ("逐时刻") precision — that
  precision is not resolvable from discrete message arrivals, and is
  not claimed.

Reconstruction script: ad hoc, not committed (a one-off analysis of
already-collected data, not a reusable pipeline component); the raw
CSV logs it reads live outside git at
`/home/eamon/epuck_comm_bags/objective5_matrix_v1_conditionF_exclusionary_pilot03_diag_logs/`.

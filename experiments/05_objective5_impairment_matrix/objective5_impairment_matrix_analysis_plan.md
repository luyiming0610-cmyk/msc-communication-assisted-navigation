# Objective 5 impairment matrix — analysis plan v1 (revision 2)

Status: **design only, not executed**. Describes how trials WILL be
analyzed once run; contains no experiment data. Revision 2: Condition A
is now a full fresh n=5 (not partially reused, see design doc section
1); the queue-drain check below is now a REAL, implemented, tested
mechanism (`relay_drain.py`, commit `4e79b5a`), not a proposed rule.

## 1. Per-trial verdict: two independent dimensions

See `objective5_impairment_matrix_design_v1.md` section 6 for the full
rationale. Every trial gets both labels, computed independently:

**DATA_VALIDITY** = `VALID` | `INVALID` — infrastructure/measurement
question, checked via:
- relay parameters and BOTH directions' seeds actually deployed match
  the condition's frozen config (cross-check `frozen_params.json`,
  written by the orchestrator before anything starts, against
  `objective5_impairment_matrix_conditions.csv`; also cross-check the
  relay's own startup log line, `network_impairment_relay.py`, against
  the intended config)
- `realtime_factor_ok` (existing field in `objective5_formal_baseline_verdict.json`)
- bag `metadata.yaml` present and non-empty
- sequence/PDR statistics computed successfully (analyzer did not error)
- analyzer completed without exception; no orchestration-script abort
- **both robots' relay queues confirmed drained** (`pending_queue_depth
  == 0` on `relay_status`, polled via `relay_drain.poll_until_drained`)
  before the relay/bag were stopped -- an undrained queue at shutdown
  must never be counted as network-impairment loss (design doc section
  5); a `DrainTimeoutError` or the orchestrator's own recorded
  `data_validity: "INVALID"` in `preliminary_runtime_manifest.json`
  means this trial's delay/jitter/loss numbers cannot be trusted and it
  must be treated as `INVALID`, not silently included
- **Only `INVALID` halts the batch for diagnosis.**

**TASK_OUTCOME** = `SUCCESS` | `SAFE_DEGRADATION` | `COLLISION` |
`TASK_TIMEOUT` | `STALE_STATE_STOP` | `PEER_TIMEOUT_STOP` | other named
category — scientific-result question, never grounds for excluding a
trial, retrying it, or stopping the condition's remaining trials when
`DATA_VALIDITY=VALID`.

## 2. Per-trial communication metrics

Reusing the existing tier structure established in
`analyze_comm_performance.py` and `objective5_formal_baseline_verdict.json`
(the same fields the two Condition-A trials already produce), extended
with impairment-specific fields:

- `APPLICATION_STATE_SEQUENCE_DELIVERY_RATIO` (per robot; existing
  `aligned_window_pdr_bag_vs_relay` + `counter_relayed` fields)
- `missing` / `duplicate` / `out_of_order` counts (existing
  `sequence_gap_count` / `duplicate_count` / `out_of_order_count`
  fields, per robot, both `bag_raw`/`bag_relayed`/`counter_relayed` tiers
  kept separate exactly as they already are — never conflated)
- `latency`/`state_age` distribution: sample count, mean, median, p95,
  p99, max (existing `live_counter_*` fields in
  `counter_relayed`/`communication_metrics`) — **valid for every
  condition** in simulation, unlike the physical baseline, because sim
  runs on one shared `/clock` domain (see design doc section 7 boundary
  note)
- throughput: existing `actual_rate_hz` / `mean_bandwidth_bytes_per_s`
- `stale_events`: count of `_control()` ticks where `self.mode ==
  "SAFE_STOP_STALE"`, extracted from the controller's own transition
  log (`_log_transition`, `cooperative_avoider.py` lines 504-561, fires
  on every mode change) — a NEW extraction, not present in the existing
  analyzer, must be added
- `peer_timeout_events`: subset of `stale_events` attributable
  specifically to `not peer_fresh` (vs. `not own_fresh`) — requires
  reading the controller's log line context, since `_fresh()` itself
  (lines 351-352) doesn't distinguish which side failed in its return
  value; the calling code at lines 658-663 does (`own_fresh` /
  `peer_fresh` are separate booleans) — extraction must capture both
  independently, not just merge them into one "stale" bucket

## 3. Per-trial task metrics

- minimum inter-robot distance over the trial (from bag `/epuck1/state`,
  `/epuck2/state` positions — same `x_m`/`y_m` fields
  `closest_point_of_approach` already consumes)
- collision flag: minimum distance `< safety_radius_m (0.14m)` at close
  range with near-zero or negative relative velocity (not simply
  "distance below threshold," since CPA geometry can produce a
  momentarily small `current_distance_m` during a clean pass — use the
  same `collision_risk` predicate's `closing_speed_mps` sign as a
  secondary check, consistent with how the live controller itself
  reasons about risk)
- CPA trigger time and `trigger_reason` (`PREDICTED_CPA` /
  `PROXIMITY_FALLBACK` / `NONE`) via `analyze_trigger_reason.py`,
  reused unmodified (module docstring, lines 1-20, already implements
  exactly this reconstruction from bag data)
- avoidance completion rate: fraction of trials reaching `AVOID_PASS` →
  `RECOVER`/`LOCAL_RECOVER` → `CRUISE` without a `SAFE_STOP_*` or
  `COMPLETE: maximum runtime reached` outcome
- recovery completion rate: fraction reaching `COMPLETE: {source}
  recovery completed` (`_control_body`, lines 644-653) rather than
  `COMPLETE: maximum runtime reached` (line 655)
- safe-stop rate: fraction of trials that entered any `SAFE_STOP_*` mode
  at all during the run (distinct from "ended via a safe stop" — a
  trial can pass through a safe stop and still recover)
- task completion time: `recovery_completed_at - started_at` for trials
  that complete via recovery; `max_runtime_s` (60.0) recorded as a
  right-censored value (not a completion time) for trials that time out
  — statistics in section 5 must treat these as censored, not drop them
  or treat 60.0 as a real completion time

## 4. Per-trial resource/infrastructure metrics

- Webots `preload_realtime_factor` / `full_load_realtime_factor`
  (existing fields)
- `bag_record_drop_warn_error_lines` (existing field, must be empty for
  `DATA_VALIDITY=VALID`)
- relay's own CSV log (`received_seq,action,scheduled_delay_s,
  receive_time_s,release_time_s,source_stamp_s,actual_release_time_s`,
  `network_impairment_relay.py` lines 75-78) cross-checked against the
  bag-observed sequence, per robot — this is the relay-side ground
  truth the existing docstring (lines 20-24) already describes using
  for exactly this purpose

## 5. Per-condition statistics (n=5)

- Report every trial's raw value for every metric above — n=5 is small;
  summary statistics never substitute for showing the 5 individual
  points.
- mean, median, stdev, IQR for continuous metrics (state_age, min
  distance, task completion time among non-censored trials).
- `SUCCESS` proportion (successes / 5) reported as a simple fraction,
  explicitly captioned as **not statistically significant at n=5** —
  no confidence interval, no p-value, no claim of significance is made
  from a 5-trial proportion. Effect sizes (e.g. difference in mean
  state_age between conditions, difference in min-distance distribution)
  and the shape of the 5-point distribution are reported in preference
  to any significance claim.
- Task-completion-time censoring (section 3) handled explicitly:
  non-censored trials' mean/median computed separately from the
  count of censored (timed-out) trials; a condition where 3/5 trials
  time out is reported as "3/5 censored, remaining 2 completed at
  [values]," never as a mean over all 5 that silently includes 60.0 as
  if it were a real completion time.
- Matched-seed comparisons (E vs G, design doc section 5): report the
  per-seed-pair delta (trial `NN` of E vs trial `NN` of G) alongside the
  condition-level aggregate, with the RNG-consumption-order caveat from
  design doc section 2.5 restated inline wherever a matched comparison
  is presented, so a reader never assumes byte-identical event
  sequences between E and G.

## 6. Cross-condition analysis

- B vs C vs A: delay-magnitude sweep (state_age, task metrics) at fixed
  jitter=drop=0.
- D vs A: jitter/reordering-isolated effect.
- E vs A: loss-isolated effect.
- G vs (A, B, D, E): combined-vs-single-factor comparison; whether G's
  effect exceeds the naive sum of B+D+E's individual deviations from A
  (superadditive/interaction) or is closer to additive.
- F vs A (once implemented): burst-specific stale/peer-timeout behavior,
  compared against whichever of B/C/D/E/G also produced any
  `STALE_STATE_STOP`/`PEER_TIMEOUT_STOP` events, to check whether F's
  mechanism (deterministic full outage) produces qualitatively different
  safe-stop patterns than incidental staleness under the other
  conditions (e.g. outage-triggered stops clustering at fixed intervals
  vs. incidental stops scattered randomly).

## 7. Physical-baseline comparison boundary (repeated from design doc, binding here too)

`physical_single_device_zero_impairment_baseline_v1` may be compared
against simulation Condition A **only** on: application-level delivery
ratio, RTT/status-snapshot distribution (physical-only metric, no sim
equivalent), message frequency, CPU/RAM/Wi-Fi resource figures, and
qualitative protocol/transport stability. It is never compared against,
or cited as evidence about, dual-robot task success rate, collision
rate, or avoidance/recovery completion — no controller, no second
robot, and no cooperative-avoidance task were ever part of that batch.

## 8. Deliverables per condition (once run)

Matching the structure already established for
`physical_single_device_zero_impairment_baseline_v1`: per-trial
`final_verdict.json` / `final_summary.md` / `runtime_manifest.json`
(REUSED/FRESH markers for controller/relay/analyzer versions, exact
relay parameters and seed actually deployed, DATA_VALIDITY and
TASK_OUTCOME both recorded explicitly), plus a per-condition
`condition_X_summary.md`/`.csv` (5 raw trials + the section-5 statistics),
and a final `objective5_impairment_matrix_batch_summary.md`/`.csv`
across all conditions once the full matrix (or whichever subset is
authorized) completes. Raw bag/CSV/log data stays outside git per
existing project policy; only derived summaries, manifests, and hash
manifests are committed.

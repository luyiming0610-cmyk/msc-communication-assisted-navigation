# EXCLUSIONARY_DIAGNOSTIC -- final-orchestrator Condition A pilot

Run at commit `5f1acb6` (the current fully-fixed orchestrator, after all
five bugs found via `objective5_matrix_v1_conditionA_exclusionary_pilot01`
and `objective5_matrix_v1_conditionF_exclusionary_pilot01/02` were
fixed: process-group leaks in controller/sim/state_publisher launches,
the DATA_VALIDITY field mislabeling bug, the `grep -c` false positive,
and the relay-status echo timeout race). This pilot exists because the
original Condition A pilot (`_pilot01`) predates all of those fixes and
is therefore not full end-to-end evidence for the orchestrator as it
exists now.

**Seeds**: Condition A is deterministic; both directions report
`seed=0` (unused).

**Result**: `DATA_VALIDITY=VALID`, `TASK_OUTCOME=SUCCESS`
(`trial_verdict.json`).

- Relay counts: epuck1 received=418/forwarded=418, epuck2
  received=437/forwarded=437, `independent_drop_count=0` and
  `outage_drop_count=0` both directions.
- Queue drain: `drain_duration_s=0.2302`; both directions'
  `pending_queue_depth=0` after drain (`queue_drained=true`).
- Rosbag: `metadata.yaml` present/non-empty; `bag_record.log` has zero
  drop/warn/error lines.
- Sequence integrity (`comm_performance_summary.json`, ran separately
  via `analyze_comm_performance.py`): `/epuck1/state` and
  `/epuck2/state` both `missing_sequence_count=0`,
  `duplicate_count=0`, `out_of_order_count=0`,
  `packet_delivery_ratio=1.0`.
- Latency (original, superseded): `NOT_MEASURED` --
  `latency_domain_mismatch_detected=true`, all samples
  `anomalous_age_sample_count`. **This label was imprecise, not wrong
  about the underlying data**: the bag-based age computation
  (`analyze_comm_performance.py`, subtracting `message.stamp` (sim
  time) from the rosbag recorder's own wall-clock receive timestamp)
  genuinely cannot measure latency for this pipeline -- that part is
  correct. But mislabeling it "clock-domain mismatch" invited the wrong
  conclusion (that the *simulation's* clocks disagree, which is false:
  `state_publisher`/`network_impairment_relay`/`sequence_counter` all
  use `use_sim_time=true` and read the one shared `/clock` topic). See
  **`matrix_analysis_v2.json`** below for the corrected,
  same-clock-domain measurement.

## Superseded: `ANALYZER_OK` was a hardcoded placeholder at the time of this run

This pilot originally ran before `matrix_analyzer.py` existed --
`ANALYZER_OK="true"` in the orchestrator was a hardcoded placeholder,
never actually checked. **This is preserved as-is below (not deleted),
but is superseded by the real analysis-only replay in
`matrix_analysis_v2.json`**, produced by the same `analyze_trial()`
function the orchestrator itself now calls for every trial (commit
`f4e8d6c`), reading this run's already-recorded relay CSVs and
`sequence_counter.py` JSON output -- Webots was NOT re-run.

`matrix_analysis_v2.json` result: `measurement_validity="VALID"`
(genuine `ANALYZER_OK=true` this time). Both directions:
`latency_measurement_status="RESOLUTION_LIMITED"` (not a clock-domain
mismatch -- the frozen, correct label for a genuinely zero-delay,
zero-jitter configured condition observing near-zero message age),
`mean_message_age_s=0.0` exactly (`epuck1_to_epuck2`: 418/418 samples;
`epuck2_to_epuck1`: 437/437 samples), `capture_ratio=1.0` both
directions, `relay_forwarded_matches_consumer_received=true` both
directions. **Honest limitation**: `p99_message_age_s` is `null` in
this replay -- the original run's `sequence_counter.py` binary predates
the p99 field being added (commit `f4e8d6c`); only the raw aggregates
it actually computed at the time (mean/median/p95/max) are available
from the archived JSON. A future trial run under the current code would
report p99 too.
- Realtime factor: preload 0.972, full load 0.955 (within the 0.8-1.2
  tolerance band).
- `min_interrobot_distance_m=0.14811` (> `safety_radius_m=0.14`).
- `complete_count=2` (both robots reached goal; stop reason: normal
  task completion, not a timeout or safety stop).

**Post-run process check**: `pgrep` for
`webots|state_publisher|cooperative_avoider|run_matrix_relay_and_counter|run_comm_baseline_formal_controllers|run_dual_head_on_clean|ros2 bag record`
returned no matches -- fully clean, confirming the process-group fixes
hold under this run.

**Status**: `EXCLUSIONARY_DIAGNOSTIC`, not part of the formal n=5. Raw
bag/logs remain outside git at
`/home/eamon/epuck_comm_bags/objective5_matrix_v1_conditionA_exclusionary_pilot02_final_orchestrator`.

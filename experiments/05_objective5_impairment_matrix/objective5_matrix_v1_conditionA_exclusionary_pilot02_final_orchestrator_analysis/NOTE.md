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
  via `analyze_comm_performance.py` -- the orchestrator's own
  `ANALYZER_OK` is still a hardcoded placeholder, not yet wired to a
  real analyzer call, a known documented gap): `/epuck1/state` and
  `/epuck2/state` both `missing_sequence_count=0`,
  `duplicate_count=0`, `out_of_order_count=0`,
  `packet_delivery_ratio=1.0`.
- Latency: `NOT_MEASURED` -- `latency_domain_mismatch_detected=true`,
  all samples `anomalous_age_sample_count`. Expected and consistent
  with every prior zero-impairment trial this session (e.g.
  `objective5_comm_baseline_zero_impairment_formal_trial01`): a
  zero-delay relay produces `message_age_s=0.0` by construction, which
  the analyzer's age-domain check correctly flags rather than silently
  reporting a meaningless near-zero latency distribution.
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

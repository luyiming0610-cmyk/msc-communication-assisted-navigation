# controller_v4_full_sensor_bypass_20260717

**Purpose**: development, regression, and Phase 4 task-validation evidence
for `controller_v4` (culminating in `controller_v4_timebase_fix_20260717`,
now frozen), plus Objective 5's communication-performance analyzer/relay
implementation and baseline diagnostics.

**Scenarios and versions covered**:
- `static_box_{a,b,c,d}`, `static_box_fusion_{a,b2,b3}` — controller_v4 development pilots (02_controller_regression)
- `head_on_cpa_pure_c1` — pure dual-robot CPA exclusionary pilot (03_phase4_task_validation)
- `combined_trial1`, `combined_trial2_timebasefix` — combined-scenario exclusionary pilots (03_phase4_task_validation)
- `combined_formal_trial01..05` — **Phase 4 formal batch, SEALED, 5/5 PASS** (03_phase4_task_validation)
- `combined_formal_trial01_INCOMPLETE_no_controller_log` — excluded first attempt (09_legacy_and_excluded)
- `comm_baseline_trial{1,2,3}` — Objective 5 diagnostic, `/mnt/c` bag-loss issue (04_objective5_comm_baseline)
- `comm_baseline_native_trial0{1,2}` — Objective 5 diagnostic, native-WSL-path root-cause isolation (04_objective5_comm_baseline)
- `objective5_comm_baseline_zero_impairment_formal_trial01` — **Objective 5 formal, PASS**: genuine `cooperative_avoider` task completion under zero impairment, native WSL bag path; metric_coverage PDR/sequence/throughput/task=VALID, latency=NOT_MEASURED (04_objective5_comm_baseline)
- `objective5_timestamp_latency_validation_pilot01` — Objective 5 diagnostic (not formal): validates the stamp/latency measurement chain (2 conditions, delay=0 / delay=0.25s) before trial02_stamp (04_objective5_comm_baseline)
- `objective5_comm_baseline_zero_impairment_formal_trial02_stamp` — **Objective 5 formal, PASS**: latency-complete companion to trial01 (protocol_v1.1_stamp_semantics); metric_coverage all VALID including latency; does NOT replace trial01 (04_objective5_comm_baseline)

**Config**: `config/static_box_v4/`, `config/head_on_cpa_v4/`, `config/combined_v4/`, `config/comm_baseline_v1/`

**PASS/FAIL criteria**: see each pilot's own `analysis/*_verdict.json`. For
the sealed formal batch specifically, see
`PHASE4_FORMAL_EVIDENCE_MANIFEST_20260717.md`.

**Actual results**: Phase 4 formal batch 5/5 PASS (see
`PHASE4_FORMAL_BATCH_SUMMARY_20260717.md`). All 5/5 trials triggered via
`PROXIMITY_FALLBACK`, not `PREDICTED_CPA` — see that summary's binding
naming rule. Objective 5 comm-baseline diagnostics found and traced a
`/mnt/c` rosbag-write message-loss issue (see
`config/comm_baseline_v1/analyze_measurement_chain.py` and the two
`comm_baseline_native_trial0{1,2}` diagnostic trials, both PASS with
aligned-window PDR=1.0). That fix was then validated at the task level:
`objective5_comm_baseline_zero_impairment_formal_trial01` PASSED —
aligned-window PDR=1.0 both robots, zero sequence gaps/duplicates/
out-of-order, `sequence_counter` `complete=true` both robots, realtime
factor 0.963/0.951, genuine `cooperative_avoider` task completion (not
`max_runtime`/FAILSAFE/TASK_TIMEOUT).

**metric_coverage for trial01**: `PDR=VALID sequence_integrity=VALID
throughput=VALID task_behavior=VALID latency=NOT_MEASURED`. Message
age/latency from this trial is **N/A**, not a real number. Correction:
`EpuckState.stamp` was NOT unset (an earlier pass of this document said
so incorrectly) — `state_publisher.py` does set it correctly via
`self.get_clock().now().to_msg()`. The actual root cause was
`analyze_comm_performance.py` computing age as
`bag_record_time(rosbag2's own wall-clock recording timestamp) -
message.stamp(sim time)` — two different clock domains, producing an
epoch-scale (~1.78e9s) number that is not real latency. This trial's
PASS verdict is unaffected; it is registered permanently as
`latency=NOT_MEASURED` and is **not** re-analyzed or backfilled.

**protocol_v1.1_stamp_semantics** (patch label, wire schema unchanged —
still `PROTOCOL_VERSION=1`, same SHA-256): `state_publisher.py` now
holds publication (`WAITING_FOR_CLOCK`) until the ROS clock is valid, so
`stamp` is never a fake zero; `sequence_counter.py` now computes its own
LIVE per-message latency (`message.stamp` vs its own
`get_clock().now()` at receipt, same clock domain, no mismatch);
`network_impairment_relay.py`'s CSV now also records `source_stamp_s`
and `actual_release_time_s`. This was validated by
`objective5_timestamp_latency_validation_pilot01` (diagnostic-only, 2
conditions, PASS: zero-delay condition measured ~microsecond-scale age;
0.25s-delay condition measured ~0.26s observed increment, error ~0.01s
against the tolerance of 0.05s) before running
`objective5_comm_baseline_zero_impairment_formal_trial02_stamp`, which
PASSED with all five metrics VALID (aligned-window PDR=1.0 both robots,
0 gaps/dup/oo, `complete=true` both robots, realtime factor
0.996/1.003, live-counter mean/median/p95/max `message_age_s`=0.0 both
robots at zero configured delay). **trial02_stamp does not replace
trial01** — both remain separately registered with different
metric_coverage; see
`objective5_comm_baseline_zero_impairment_formal_trial01/analysis/objective5_formal_baseline_verdict.json`
and
`objective5_comm_baseline_zero_impairment_formal_trial02_stamp/analysis/objective5_formal_baseline_verdict.json`
for the full machine-readable verdicts.

**peer_timeout_s audit finding** (read-only; the frozen
`cooperative_avoider` controller was NOT modified): peer freshness is
judged by callback receipt time, not `msg.stamp`. A constant relay
delay does not by itself trigger `peer_timeout` — only jitter (variance)
or real message loss can plausibly widen a receipt gap past
`peer_timeout_s`. An earlier impairment-matrix draft's claim that a
0.6s configured delay would trigger peer-timeout degradation is
**retracted**.

## Execution attempts (kept, not deleted, even for failed attempts)

Per instruction, failed attempts are never deleted, directories are
never overwritten/reused, and each attempt gets a unique name going
forward (`TRIALNAME_attemptNN` or a new unique trial name).

| attempt_id | result | failure_stage | failure_reason | produced_valid_bag | included_in_statistics | retained_artifacts |
|---|---|---|---|---|---|---|
| `objective5_comm_baseline_zero_impairment_formal_trial01` attempt 1 | FAIL | shutdown | WSL binfmt_misc interop dropped mid-orchestration (`cmd.exe: cannot execute binary file`) before the run could complete | No | No | artifact not retained — the working directory was cleaned (`rm -rf`) before this documentation requirement existed; only the execution log line showing the failure is preserved in this session's transcript, not as a file on disk |
| `objective5_comm_baseline_zero_impairment_formal_trial01` attempt 2 | FAIL | shutdown | `stop_pid` sent SIGINT only to the relay+counter launch-service's own PID, not its child rclpy processes; the children were orphaned before writing `complete=true` (observed via file mtimes: checkpoints kept updating ~20s after the parent PID was already confirmed dead), so `sequence_counter`'s output stayed `complete=false` — comm data itself (PDR, gaps, dup, oo) was otherwise clean | Yes (bag recorded, PDR=1.0, 0 gaps/dup/oo) | No (verdict=FAIL on `complete=false`, per the trial's own acceptance criteria) | artifact not retained — cleaned before this documentation requirement existed |
| `objective5_comm_baseline_zero_impairment_formal_trial01` attempt 3 | FAIL | shutdown | Fix for attempt 2 (`stop_pid_group`, process-group signaling) exposed a second bug: its SIGKILL fallback branch unconditionally returned 1 even on success, and this script runs non-interactively (job control off) so plain `&` backgrounding does not isolate each job into its own process group — combined with `set -e` and no `\|\| true` guard at the call site, the whole script aborted immediately after the "stopping relay + sequence_counter" log line | No (script aborted before the controller/rosbag stages completed their normal shutdown sequence) | No | artifact not retained — cleaned before this documentation requirement existed |
| `objective5_comm_baseline_zero_impairment_formal_trial01` attempt 4 (final, registered) | **PASS** | — | — | Yes | **Yes** — registered as `objective5_comm_baseline_zero_impairment_formal_trial01` in `experiment_registry.csv` | `objective5_comm_baseline_zero_impairment_formal_trial01/` (this directory), including `metadata.yaml` and `analysis/objective5_formal_baseline_verdict.json` |
| `objective5_timestamp_latency_validation_pilot01` condition_a_delay0 | **PASS** | — | — | Yes | No (diagnostic-only by design, never pooled with formal statistics) | `objective5_timestamp_latency_validation_pilot01_condition_a_delay0/` |
| `objective5_timestamp_latency_validation_pilot01` condition_b_delay025 | **PASS** | — | — | Yes | No (diagnostic-only by design) | `objective5_timestamp_latency_validation_pilot01_condition_b_delay025/` |
| `objective5_comm_baseline_zero_impairment_formal_trial02_stamp` attempt 1 (final, registered) | **PASS** | — | — | Yes | **Yes** — registered as `objective5_comm_baseline_zero_impairment_formal_trial02_stamp` in `experiment_registry.csv` | `objective5_comm_baseline_zero_impairment_formal_trial02_stamp/`, including `metadata.yaml` and `analysis/objective5_formal_baseline_verdict.json` |

Attempts 1-3 above were WSL-interop/orchestration-script failures, not
communication-result failures; they are excluded from the formal
statistics per `included_in_statistics=No`, but are retained in this
audit table (rather than pretending they never happened) so the
attempt history is traceable, per instruction.

**Included in dissertation**: Phase 4 formal batch YES (with the
`PROXIMITY_FALLBACK` limitation noted).
`objective5_comm_baseline_zero_impairment_formal_trial01` YES for
PDR/sequence/throughput/task_behavior (latency=NOT_MEASURED, N/A).
`objective5_comm_baseline_zero_impairment_formal_trial02_stamp` YES for
all five metrics including latency. Everything else in this directory
is development/diagnostic evidence, NOT formal statistics.

**How to reproduce**: each pilot config directory's `run_*.sh` script is
self-contained (sources ROS2 Humble + the `epuck_ws` workspace, launches
Webots via `simulation_comm_experiment_v1/working/run_*.py`, records a
rosbag, runs its analyzer). See `PHASE4_FORMAL_EVIDENCE_MANIFEST_20260717.md`
for the exact frozen configuration fingerprint (file hashes, geometry,
parameters) used for the sealed batch.

**Git commits**: see `experiments/experiment_registry.csv` for the
per-trial commit hashes; the controller itself was frozen at `980e7d0`.

**Details**: `HANDOFF_20260717.md` in this directory has the full
controller v1→v4 technical narrative and open findings.

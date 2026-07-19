# PROJECT_HANDOFF.md — start here

Single entry point for any AI (Claude, Codex, or otherwise) or human
picking up this project. Last updated: 2026-07-18. This document was
current as of index-files commit `026223b` (source state at scan time:
`2558216`) — **do not trust a hardcoded commit hash in any document,
including this one, as "the current commit."** Always run
`git rev-parse HEAD` / `git log -1` for the true current state; a
document's own commit field necessarily lags the commit that contains it.

## What this project is

**Official title** (COMP5200M Project Specification, `LU26-Spec.pdf`):
"Developing a Lightweight Communication Library for Collaborative
Navigation in e-puck2 Robot Swarms." Supervisor: Dr. Arshad Jhumka.

**Official five Objectives** (from the Spec and Scoping/Planning Document):
1. **Environment Setup** — ROS2/Gazebo simulation + physical e-puck2/Pi-puck hardware interface.
2. **Protocol Design** — a streamlined ROS2 Custom Msg / Protobuf message format.
3. **Library Implementation** — a P2P abstraction, neighbor "subscription" model (`e-puck2-Comm`).
4. **Task-Specific Validation** — a navigation/avoidance task measuring how communication latency affects collision avoidance.
5. **Performance Analysis** — packet delivery rate + coordination efficiency, virtual vs. real-world (reality gap).

**Deliverables**: the `e-puck2-Comm` library, the Simulation Package, an Evaluation Report.

**The avoidance controller (v1-v4) is Objective 4's task-specific
validation vehicle, not the project's primary contribution.** It is now
frozen (see below). Current project effort is on Objectives 2/3/5
(protocol/library formal metrics) and, next, Objectives 1/6/7 (physical
Pi-puck, reality gap).

## Current overall status

- **Objective 1** (Environment Setup): simulation side done (ROS2 Humble + Webots, not Gazebo — see the deviation note below). Physical Pi-puck side: bringup verified 2026-07-18 (Pi ROS 2 Foxy driver + Pi TCP server + WSL TCP client all confirmed running, `/scan`/`/odom`/`/tof`/`/ps0` all confirmed publishing, robot stationary throughout, no `/cmd_vel` sent) — see `experiments/06_physical_pipuck/physical_device_registry.md`. No pilot or formal physical experiment has been run under the current protocol; `physical_protocol_gap_report.md` documents that the pre-existing physical bridge data (2026-07-15, `real_robot_avoidance_v1/`) uses an old JSON protocol incompatible with the current frozen `EpuckState.msg` and cannot be pooled with current formal statistics.
- **Objective 2** (Protocol Design): `EpuckState.msg` implemented and **frozen as PROTOCOL_VERSION=1** (commit `b5a0351`).
- **Objective 3** (Library Implementation): `epuck2_comm` library implemented — `state_publisher`, `cooperative_avoider`, `network_impairment_relay`, analyzers. 126/126 tests passing.
- **Objective 4** (Task-Specific Validation): controller v1→v4 defect chain resolved; **Phase 4 formal batch SEALED, 5/5 PASS** (commit `e32560e`). Avoidance-scenario scope is now intentionally frozen.
- **Objective 5** (Performance Analysis): analyzer + impairment relay implemented and unit-tested; **zero-impairment baseline is still diagnostic only** — root cause of an earlier ~40-55% message-loss anomaly has been traced to writing rosbag directly to a `/mnt/c` (Windows-mounted) path from WSL2, confirmed via two independent native-WSL-path diagnostic trials (PDR=1.0 both times), but a full formal baseline (with the actual task controller running) has not yet been completed under that fixed workflow. **No formal PDR/latency figures exist yet.**
- **Objective 6/7** (physical validation, reality gap): bridge/driver bringup verified (see Objective 1 above). **First formal physical result now exists**: `physical_single_device_zero_impairment_baseline_v1` (stationary, no ground motion, no controller, single e-puck2 #5809, expanded Pi-TCP-WSL bridge + `EpuckState.msg`) is **FINAL_BATCH_PASS (5/5 FINAL_PASS)** — see `experiments/06_physical_pipuck/single_device_bringup/physical_single_device_zero_impairment_baseline_v1_batch/batch_summary.md` and the per-trial `physical_single_device_zero_impairment_baseline_v1_trial0N_attemptNN_analysis/` directories. All 5 trials are one continuous driver/Pi-expanded-server/WSL-bridge session (not 5 independent cold starts). Tier A delivery ratio 1.0/5 trials (application-level, not IP/TCP loss; `duplicate_count` NOT_MEASURABLE, never 0); Tier B bag-capture ratio 1.0 at ~8.88-8.94 Hz actual; Tier C raw sensors ~9.2 Hz, no PDR claimed; RTT tail ~20-25% >50/100ms, 0% >200ms recurring across all 5 windows with **no root-cause attribution**; one-way Pi-to-WSL latency **NOT reported/measured** (no clock-sync verified); `trial01/02_attempt01_short_window` are excluded diagnostic evidence (window-timing defect, since fixed), not part of this n=5. This is a stationary, comm-layer-only result — no ground-motion or controller-driven physical trial has run yet, and reality-gap comparison (Objective 7) has not started.

## Key paths

| What | Path |
|---|---|
| Windows repo root | `C:\Users\路一鸣\Desktop\硬件实验毕设\e-puck2-Comm` |
| WSL repo root (same repo, mounted) | `/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/e-puck2-Comm` |
| ROS2 workspace (built package lives here, synced from repo `src/`) | `~/epuck_ws` (i.e. `/home/eamon/epuck_ws`) |
| Webots world/launch working directory (**outside** the git repo) | `/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/simulation_comm_experiment_v1/working` |
| Native WSL scratch path for rosbag recording (see Objective 5 finding below) | `/home/eamon/epuck_comm_bags/` |

**Workflow note**: the repo's `src/epuck2_comm` package is edited in the
Windows-visible path, then **synced into `~/epuck_ws/src/epuck2_comm`**
(`rsync`) and rebuilt (`colcon build --packages-select epuck2_comm
--symlink-install`) before any `ros2 run`/pilot script will see the
change. Every pilot `.sh` script sources `/opt/ros/humble/setup.bash` then
`~/epuck_ws/install/setup.bash`.

## Build / test commands

```bash
source /opt/ros/humble/setup.bash
source ~/epuck_ws/install/setup.bash
cd ~/epuck_ws
colcon build --packages-select epuck2_comm --symlink-install
colcon test --packages-select epuck2_comm
colcon test-result --verbose
```

Fast unit-test-only loop (no ROS workspace rebuild needed for pure-Python logic changes, but the workspace copy must still be synced before any `ros2 run`):
```bash
cd "/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/e-puck2-Comm/src/epuck2_comm"
python3 -m pytest test/ -q
```

## Protocol and message definition

- File: `src/epuck2_comm_interfaces/msg/EpuckState.msg`
- **Frozen as `PROTOCOL_VERSION=1`** at commit `b5a0351`. SHA-256:
  `a7ec4184dec52b157a87beea20b44fb2dff5c6dee199d0c76b7c347c26abe15b`.
- Full rules and rationale: `src/epuck2_comm_interfaces/PROTOCOL_FREEZE_20260717.md`.
- **Do not add, remove, or reorder fields.** A genuine structural need requires a new, distinct `PROTOCOL_VERSION=2` message, never mutating v1.

## Frozen controller

- Version: `controller_v4_timebase_fix_20260717`
- Key commits: `980e7d0` (timebase fix), `06e0f0f` (post-fix regression evidence), `f1830c5` (docs)
- Files: `src/epuck2_comm/epuck2_comm/cooperative_avoider.py`, `local_obstacle_logic.py`
- **Frozen. Do not modify the avoidance algorithm, safety thresholds, CPA parameters, or the world/geometry it was validated against unless a new blocking safety defect is found and proven with log evidence.** Analysis-only tooling (new analyzer scripts, offline classifiers) may be added freely — see `analyze_trigger_reason.py`, `analyze_comm_performance.py` as examples of the accepted pattern (new read-only script, never touches the controller).

## Experiment categories (full detail: `experiments/EXPERIMENT_INDEX.md`)

01 protocol/unit tests · 02 controller regression (v1-v4 dev evidence, NOT formal comm stats) · 03 Phase 4 task validation (Phases 1-4, formal vs. pilot/diagnostic clearly separated) · 04 Objective 5 comm baseline (currently diagnostic only) · 05 Objective 5 impairment matrix (not started) · 06 physical Pi-puck (not started) · 07 reality gap (not started) · 08 paper-ready outputs (currently empty) · 09 legacy/excluded (old protocol-format bags, failed pilots — never deleted, always indexed with an exclusion reason).

## Formal vs. diagnostic — the distinction that matters most

**Formal** (`evidence_level=FORMAL_SIM` or `FORMAL_PHYSICAL` in the
registry): counted toward dissertation statistics, run under a frozen,
documented configuration, typically part of an n≥5 batch.

**Diagnostic/pilot** (`evidence_level=PILOT`): exploratory, debugging,
measurement-chain-isolation, or pre-validation runs. **Never pooled with
formal statistics, regardless of PASS/FAIL.** Most of the Objective 5
comm-baseline work is still diagnostic-only; the exceptions are two
formal, separately-registered zero-impairment trials (`evidence_level=
FORMAL_SIM`, both PASS, both genuine task-level runs — real
`cooperative_avoider` task completion, native WSL bag path, PDR=1.0,
zero gaps/duplicates/out-of-order):
- `objective5_comm_baseline_zero_impairment_formal_trial01` — metric_coverage
  `PDR=VALID sequence_integrity=VALID throughput=VALID task_behavior=VALID
  latency=NOT_MEASURED`. Message age/latency from this trial is **N/A**,
  permanently — not re-analyzed or backfilled. See known limitation 3
  below for the actual root cause (corrected from an earlier, wrong
  statement in this document).
- `objective5_comm_baseline_zero_impairment_formal_trial02_stamp` — latency-
  complete companion (protocol_v1.1_stamp_semantics), metric_coverage all
  five metrics VALID. **Does not replace trial01** — both remain
  separately registered.

A third, distinct formal batch now exists: the **Objective 5 impairment
matrix's Condition A** (zero impairment, the A-G matrix's first
condition), a genuine formal n=5 batch, **COMPLETE, 5/5 PASS**:
`objective5_impairment_matrix_v1_condition_A_trial01..05_attempt01`
(`evidence_level=FORMAL_SIM`). Real analyzer (`matrix_analyzer.py`,
strict schema — `legacy_replay=false`, p99 always a finite value, never
null), `DATA_VALIDITY=VALID`/`TASK_OUTCOME=SUCCESS`/`analyzer_ok=true`
in all 5 trials, `capture_ratio=1.0` and 0 drops both directions in all
5, `min_interrobot_distance_m` in `[0.14178534915907265,
0.15064050840214138]`, all above `safety_radius_m=0.14` but with a
notably tight margin on Trial 05 (~1.79mm) — reported explicitly, not
grounds to alter the frozen geometry/controller. Behavioral-code
SHA-256 (orchestrator, relay, impairment core, `sequence_counter.py`)
verified identical across all 5 trials. Full cross-trial statistics:
`experiments/05_objective5_impairment_matrix/objective5_impairment_matrix_v1_condition_A_formal_batch_summary.md`.
**This does not replace or supersede the two comm-baseline trials
above** — it is a separate, later, more rigorously-schema-gated formal
result within the same Objective 5 zero-impairment question, run via
the impairment-matrix orchestrator rather than the earlier
comm-baseline script. **Conditions B and C are now complete. Condition B: 5/5 PASS. Condition C: COMPLETE_FAILED_SAFETY_GATE (4/5 SUCCESS + 1/5 UNSAFE_FAILURE).** C05 reached 0.1389086m, about 1.091mm below the frozen 0.14m safety radius; the valid failure was retained and not rerun. All five C trials reproduced exactly 9 steering-direction reversals per robot inside one continuous AVOID_PASS, with PREDICTED_CPA and no LOCAL_* takeover.

The registry's first `evidence_level=FORMAL_PHYSICAL` row (distinct from
`FORMAL_SIM`) is `physical_single_device_zero_impairment_baseline_v1`
(5/5 FINAL_PASS, `06_physical_pipuck`) — see the Objective 6/7 status line
above and `experiments/EXPERIMENT_INDEX.md`'s `06_physical_pipuck` section
for the full scope note (Tier A/B/C semantics, RTT no-root-cause,
one-way-latency not measured, 2 excluded short-window attempts).

Full machine-readable record: `experiments/experiment_registry.csv` (one
row per experiment/batch — `status`, `evidence_level`,
`formal_or_diagnostic`, `included_in_paper` columns are authoritative).

## Known limitations (see `experiments/project_status.json` for the full list)

1. Phase 4's combined scenario (5/5 formal) triggers via `PROXIMITY_FALLBACK` in every trial, never `PREDICTED_CPA` — must be named "staged local-obstacle avoidance followed by communication-assisted proximity/cooperative avoidance," never described as preventing a certain collision.
2. 5 pre-protocol-freeze bags cannot be reprocessed by current analyzers (old `EpuckState` wire shape) — original historical analysis remains valid, re-analysis does not work.
3. `/mnt/c` rosbag-write message loss (confirmed via two native-path diagnostic trials) is fully resolved for formal work by using the native WSL ext4 bag path — both formal zero-impairment trials validated this at the task level (PASS). **Corrected** (an earlier pass of this document said `EpuckState.stamp` was "never populated by `state_publisher.py`" — that was wrong; it does set it, via `self.get_clock().now().to_msg()`): the real root cause of trial01's epoch-scale "age" figures was `analyze_comm_performance.py` computing age as `bag_record_time (rosbag2's own wall-clock recording timestamp) - message.stamp (sim time)` — two different clock domains. Fixed this session under the `protocol_v1.1_stamp_semantics` patch label (wire schema unchanged, still `PROTOCOL_VERSION=1`): `state_publisher.py` now holds publication (`WAITING_FOR_CLOCK`) until the clock is valid; `analyze_comm_performance.py` now detects and excludes negative/implausible ages rather than silently averaging them in; `sequence_counter.py` now computes its own live, same-clock-domain latency (validated by `objective5_timestamp_latency_validation_pilot01`, then used for trial02_stamp's latency=VALID coverage). PDR/rate/bandwidth were never affected by this bug. trial01 itself is **not** re-analyzed or backfilled — it stays registered as `latency=NOT_MEASURED` permanently.
4. `sequence_counter` was rewritten this session (periodic atomic checkpoints + a `finally`-block final write, no custom SIGINT handler) and verified working in the formal baseline PASS (`complete=true` both robots). A related orchestration-script bug — the shell script signaling only the relay+counter launch-service's own PID, not its child processes — caused the first 3 attempts at trial01 to fail with `complete=false` (these 3 failed attempts are documented, not hidden, in `experiments/controller_v4_full_sensor_bypass_20260717/README.md`'s execution_attempts table); fixed by running that process tree under `setsid` and signaling the whole process group on shutdown (`run_objective5_comm_baseline_formal_trial.sh`'s `stop_pid_group`).
5. `peer_timeout_s` audit finding (read-only; frozen `cooperative_avoider` NOT modified): peer freshness is judged by callback receipt time, not `msg.stamp`. A constant relay delay does not by itself trigger `peer_timeout` — only jitter or real loss can plausibly do that. An earlier impairment-matrix draft's "0.6s delay triggers timeout" claim is retracted.
6. Webots R2025a is used, not Gazebo as the Spec names — this is a disclosed, deliberate deviation (protocol/library are simulator-agnostic; see `HANDOFF_20260717.md` for the full risk note and the recommendation to confirm with the supervisor). Do not silently redo the platform in Gazebo.
7. No CPU/memory overhead measurement exists yet (would need a live psutil-style sampling companion).
8. No physical-hardware clock-sync procedure exists yet (`verify_clock_sync()` intentionally raises `NotImplementedError` until Objective 6 begins).

## Current single next step

Conditions A and B are complete at 5/5 PASS. Condition C is also complete,
but its correct result is `4/5 SUCCESS + 1/5 UNSAFE_FAILURE`; do not rerun
C05 or relabel the batch as a pass. **Condition D is IN PROGRESS and HALTED
at Trial 04.** D01 (offline-corrected for a sequence-accounting bug, see
below) and D02-D03 all independently passed the three-axis verdict
(`DATA_VALIDITY / MANIPULATION_VALIDITY / TASK_OUTCOME` all
`VALID/VALID/SUCCESS`); D03's margin is extremely tight (+0.17mm, preserved
as genuine data). **D04 triggered an explicit stop condition**:
`MANIPULATION_VALIDITY=INVALID` because the `epuck2_to_epuck1` direction has
`actual_missing_count=1` (sequence 17) — verified directly against the raw
relay CSV to be a genuine single-message gap between the relay's forward and
the bag's capture, NOT the sequence-accounting bug below. D04 was **not**
rerun and D05 was **not** started; see
`objective5_impairment_matrix_v1_condition_D_trial04_attempt01_analysis/STOP_CONDITION_REPORT.md`.
No Condition D n=5 batch summary exists yet.

Running D01 surfaced a **sequence-accounting bug**: `sequence_counter.py`'s
adjacent-delta missing/expected accounting is wrong under reordering (it
reported bogus `missing=189/192` and an impossible `capture_ratio=1.00233`
for D01). It is corrected by a new offline, versioned, set-based analyzer
`experiments/05_objective5_impairment_matrix/tools/reorder_safe_delivery_analyzer.py`
(13 unit tests). Its output fields are named
`aligned_window_forwarded_to_bag_capture_ratio` and
`relay_received_to_forwarded_ratio` (renamed 2026-07-19 for accuracy, no
value change). For Conditions D and the jitter component of G, use
`reordering_delivery_audit.json`, NOT `matrix_analysis.json`'s
`sequence.missing_count/capture_ratio` (annotated METHOD_INVALID). The frozen
`sequence_counter.py` itself was NOT modified (it stays correct for the
in-order streams of A/B/C/E). Conditions E-G have not started; Condition F
additionally remains blocked by the mandatory outage-window/startup-offset
audit in `experiments/project_status.json`. Use the same frozen orchestrator;
the CSV remains the only parameter source.
## What to read first, in order

1. **This file.**
2. `experiments/EXPERIMENT_INDEX.md` — the full experiment taxonomy and where everything is.
3. `experiments/project_status.json` — machine-readable current state, blocked items, known limitations.

Then, depending on what you're doing: `experiments/experiment_registry.csv`
(per-experiment detail), `experiments/path_manifest.csv` (exact paths,
Windows+WSL, existence/size), or
`experiments/controller_v4_full_sensor_bypass_20260717/HANDOFF_20260717.md`
(the detailed controller v1→v4 technical narrative).

## Do not

- Do not overwrite, delete, move, or rename any existing bag/log/analysis/CSV/JSON/report. If you must copy, keep the original and record the copy's path + SHA-256 in `path_manifest.csv`.
- Do not mix formal and diagnostic/pilot data into the same statistical batch.
- Do not modify the frozen controller (`cooperative_avoider.py`, `local_obstacle_logic.py`) or the frozen `PROTOCOL_VERSION=1` message without a proven, logged, blocking defect.
- Do not re-open or rerun the SEALED Phase 4 formal batch (Trials 01-05, commit `e32560e`).
- Do not claim a cross-device (physical hardware) latency measurement without first running and passing a real clock-sync verification.
- Do not silently redo the simulation platform in Gazebo — the Webots deviation is disclosed and accepted pending supervisor confirmation, not a defect to quietly fix.

## Maintenance rule

After every experiment run, update (in this order): that experiment's own
evidence (already handled by its own pilot script), then
`experiments/experiment_registry.csv`, `experiments/path_manifest.csv`,
`experiments/project_status.json`, `experiments/EXPERIMENT_INDEX.md`, and
— only once a new formal gate is reached — this file and the paper-output
directory (`08_paper_ready_outputs` in the index).

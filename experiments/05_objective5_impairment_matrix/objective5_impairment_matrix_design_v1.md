# Objective 5 communication impairment matrix — design v1

Status: **design only, not executed**. No simulation trial has been run
from this document. Condition A's Trial 01 is a REUSE of an already-
existing, already-sealed formal trial (no new run); Condition A Trials
02-05 and all of Conditions B-G are **not started**.

This design supersedes the retracted claim (see
`experiments/controller_v4_full_sensor_bypass_20260717/README.md` lines
77-84 and `experiments/project_status.json`'s `known_limitations[4]`)
that "a 0.6s configured delay would trigger peer-timeout degradation."
That claim is wrong for the reason worked out in section 2.5 below: a
**stable** fixed delay does not change the interval between successive
message arrivals, so it cannot by itself cause a staleness timeout no
matter how large it is, as long as it is applied uniformly per message
(no per-message accumulation, no queue backlog). This document does not
repeat that error.

## 1. Condition A disposition

### 1.1 Reused as Condition A Trial 01 (no rerun)

`objective5_comm_baseline_zero_impairment_formal_trial02_stamp`, commit
`59588e9`, verdict PASS, `fail_reasons: []`
(`experiments/controller_v4_full_sensor_bypass_20260717/bags/controller_v4_full_sensor_bypass_20260717_objective5_comm_baseline_zero_impairment_formal_trial02_stamp/analysis/objective5_formal_baseline_verdict.json`).

Compatibility, checked field by field against the current live code
(confirmed via `git log --oneline -- <file>` that none of the files
below have a commit after `d21f950`, which is itself the commit that
added this trial's registry row and closed out `protocol_v1.1_stamp_semantics`
— i.e. the code this trial ran is byte-identical to the code on disk
today):

| check | trial02_stamp | current code | compatible |
|---|---|---|---|
| `EpuckState.msg` SHA-256 | `a7ec4184dec52b157a87beea20b44fb2dff5c6dee199d0c76b7c347c26abe15b`, frozen commit `06dae306` (`src/epuck2_comm_interfaces/PROTOCOL_FREEZE_20260717.md`) | same file, no commit since | YES |
| PROTOCOL_VERSION | 1 (`protocol_v1.1_stamp_semantics` = stamp-gating behavior only, wire format untouched) | 1 | YES |
| timestamp semantics | `state_publisher.py`'s `WAITING_FOR_CLOCK` gate + real `stamp` population (line ~275) | identical, unchanged since | YES |
| controller version | `controller_v4_timebase_fix_20260717`, commits `980e7d0`/`06e0f0f`/`f1830c5` | `cooperative_avoider.py`/`local_obstacle_logic.py` unchanged since `d21f950` | YES |
| relay version/params | `network_impairment_relay.py`, `delay_s=0.0 jitter_s=0.0 drop_probability=0.0 immediate_passthrough=true` | unchanged since `03ce36c` (predates both trials) | YES |
| scenario/initial pose | `run_dual_head_on_clean.py`, epuck1 origin `(-0.35, 0.0, 0.0)`, epuck2 origin `(0.35, 0.0, π)` (`run_comm_baseline_formal_controllers.py`) | same script, same config dir, unchanged | YES |
| trial duration | task-completion-driven, not fixed; bag `duration.nanoseconds: 94215112581` (~94.2s); `max_runtime_s=60.0` ceiling | same orchestrator/config | YES (duration is an outcome, not a frozen input — see 1.2) |
| topic list | `/epuck1/state_raw /epuck2/state_raw /epuck1/state /epuck2/state /epuck1/cmd_vel /epuck2/cmd_vel` (`run_objective5_comm_baseline_formal_trial.sh` line 258-261) | same script | YES |
| WSL native bag recording | native WSL ext4 first (`/home/eamon/epuck_comm_bags`), copied into git tree only after clean stop + non-empty `metadata.yaml` | same script | YES |
| analyzer version | `analyze_comm_performance.py` (`--warmup-s 2.0 --cooldown-s 2.0 --peer-timeout-s 0.5`) + `analyze_objective5_formal_baseline.py` | same scripts, unchanged since `d21f950` | YES |
| trial count n | **1** | matrix requires n=5 | **NO — see 1.2** |
| realtime factor | preload 0.996 / full_load 1.003 | N/A (per-run measurement) | n/a |
| PDR/latency field completeness | `metric_coverage: PDR=VALID sequence_integrity=VALID throughput=VALID task_behavior=VALID latency=VALID` (registry row, `experiment_registry.csv` line 38) | n/a | YES (complete, unlike trial01) |
| formal (not diagnostic/pilot) | `formal_or_diagnostic=formal`, `evidence_level=FORMAL_SIM` | n/a | YES |

**Every dimension is compatible except sample size.** Condition A is
therefore **sealed as Trial 01 by reuse**, not rerun, and needs 4 more
trials (Trials 02-05, identical configuration) before it has n=5.

### 1.2 Old `objective5_comm_baseline_zero_impairment_formal_trial01` — LEGACY/EXCLUDED

Commit `9f4d7b2`. `metric_coverage: ... latency=NOT_MEASURED` (registry
row, `experiment_registry.csv` line 36) — permanent, not backfilled
(root cause: `analyze_comm_performance.py` at that commit mixed
rosbag2's wall-clock recording timestamp with `message.stamp` sim time;
fixed by the live-sequence-counter approach `trial02_stamp` uses, not by
correcting the old bag-based path). Per instruction: **kept as
LEGACY/EXCLUDED, not deleted, not recomputed under the current protocol,
not counted toward Condition A's n=5.**

### 1.3 Condition A execution status

- Trial 01: **sealed by reuse**, see 1.1.
- Trials 02-05: **not started**, pending this document's confirmation.
  Once confirmed, same orchestrator (`run_objective5_comm_baseline_formal_trial.sh`),
  same config, run 4 times with unique trial names
  (`objective5_impairment_matrix_v1_condition_A_trial{02,03,04,05}`).

## 2. Real data extracted (cited, not invented)

### 2.1 Publish/control timing

- `state_publisher.py` line 47: `self.declare_parameter("publish_rate_hz", 10.0)` (nominal target).
- `state_publisher.py` line 119: `self.timer = self.create_timer(min(0.02, minimum_interval_s), self._timer_callback)`.
- **Actual measured rate** (trial02_stamp `objective5_formal_baseline_verdict.json`, `communication_metrics`): `/epuck1/state actual_rate_hz: 8.691791270309283`, `/epuck2/state actual_rate_hz: 8.690203513139396` → **measured publish period ≈ 0.1151s** (`comm_performance_summary.json`'s `mean_interval_s: 0.11502631462660412`, epuck1). The matrix uses this MEASURED period, not the nominal 10Hz target, wherever "publish period" is a design input.
- Controller decision loop: `cooperative_avoider.py` line 262: `self.timer = self.create_timer(0.05, self._control)` → **20 Hz (0.05s)**.

### 2.2 Freshness / staleness / peer-timeout semantics

- `cooperative_avoider.py` line 56: `self.declare_parameter("peer_timeout_s", 0.5)`.
- `cooperative_avoider.py` lines 317-319 (`_peer_callback`): `self.peer_received = self._now_s()` — set on EVERY callback invocation, i.e. every time a message (of any age/delay) actually arrives.
- `cooperative_avoider.py` lines 351-352 (`_fresh`): `return received_at is not None and now - received_at <= self.peer_timeout`.
- **Peer timeout uses the callback-RECEIPT interval (`now - self.peer_received`), NOT the message's own `stamp`/production time.** A message that was delayed by the relay still refreshes `peer_received` the instant it arrives — the delay value itself is invisible to the freshness check.
- **Consequence (corrects the retracted claim)**: a *stable* fixed `delay_s`, applied uniformly per message with no queue backlog, shifts every message's arrival time by the same constant but does **not** change the *interval* between consecutive arrivals (that interval is still ≈ the source publish period, 0.1151s, regardless of `delay_s`'s magnitude). So fixed delay alone — however large — will not trip `peer_timeout_s=0.5` in steady state. What CAN trip it: (a) a genuine gap in message production (unrelated to delay), (b) loss (dropped messages skip a `peer_received` refresh, and enough consecutive drops close the 0.5s window), (c) the relay's shutdown-boundary behavior (section 2.6) losing the last few queued messages, or (d) transient startup-of-relay effects before its internal state has any history. Condition C (high fixed delay) is therefore designed to test CPA-prediction staleness under a *stable* large delay, not to trigger `SAFE_STOP_STALE` — if it produces a stale-state event anyway, that is a real, reportable finding (see section 6), not an expected/designed outcome.

### 2.3 CPA / collision-risk thresholds

- `cooperative_avoider.py` lines 55, 52, 106-107: `cpa_horizon_s=4.0`, `trigger_distance_m=0.34`, `safety_radius_m=0.14`.
- `cooperative_avoider.py` line 53: `release_distance_m=0.24`. Line 91: `rearm_distance_m=0.45`.
- `collision_math.py` lines 49-65 (`collision_risk`), quoted in full:
  ```
  predicted_conflict = (result.time_to_cpa_s <= horizon_s and result.distance_at_cpa_m < safety_radius_m)
  proximity_conflict = result.current_distance_m < trigger_distance_m
  return predicted_conflict or proximity_conflict
  ```
  (gated by `closing_speed_mps > minimum_closing_speed_mps=0.001`, preventing post-pass retriggers).
- This is also the exact formula `analyze_trigger_reason.py` (module docstring, lines 1-20) reconstructs offline to classify each trigger as `PREDICTED_CPA` / `PROXIMITY_FALLBACK` / `NONE` — reused unmodified for this matrix's trigger-reason field.
- Speed parameters (`cooperative_avoider.py` lines 48-50): `nominal_speed_mps=0.025`, `avoidance_speed_mps=0.012`, `turn_rate_rps=0.65`. These are slow relative to the 4.0s CPA horizon (max travel in one horizon ≈ 0.10m at nominal speed) — a design constraint noted in section 3's rationale for delay-tier sizing.

### 2.4 Runtime/hold timers

- `startup_hold_s=5.0` (line 57), `max_runtime_s` declared default `22.0` (line 58) but **the formal orchestrator overrides this to `60.0`** (`run_comm_baseline_formal_controllers.py` line ~confirmed in the controller-launcher config quoted by the prior research pass: `max_runtime_s: 60.0, stop_after_recovery: True, post_recovery_hold_s: 0.5`).
- Command smoothing (lines 92-95): `max_linear_accel_mps2=0.05`, `max_linear_decel_mps2=0.10`, `max_angular_accel_rps2=3.0`, `max_angular_decel_rps2=4.0`. Any safety stop (`_publish(..., force_zero=True)`) bypasses the smoother entirely — zero command is immediate, not ramped (`_publish`, lines 321-349, and the explicit comment at lines 775-781 generalizing "same-tick safety stop ... must never wait for max_linear_decel_mps2").

### 2.5 Relay implementation — exact mechanics (`network_impairment_relay.py` + `network_impairment.py`)

- **Delay**: `ImpairmentDecider.decide()` (`network_impairment.py` lines 40-47): `release_delay = max(0.0, self.config.delay_s + jitter)`. Applied per-message via a min-heap keyed on `release_time_s = now + release_delay_s` (`network_impairment_relay.py` lines 110-115), flushed by a `0.01s` (10ms) timer (`_flush_queue`, line 87). The heap flushes in ascending `release_time_s` order, not receipt order — see reordering note below.
- **Jitter**: symmetric uniform, `self._rng.uniform(-jitter_s/2, +jitter_s/2)` (`network_impairment.py` line 45) — **not** Gaussian, **not** one-sided.
- **Reordering**: possible and NOT prevented by the relay — because the queue flushes in `release_time_s` order, if message N's `jitter_N` is large-negative and message N+1's `jitter_{N+1}` is large-positive, `release_time_N` can exceed `release_time_{N+1}`, and the relay will deliver N+1 to the controller before N (a genuine out-of-order delivery, correctly countable via `EpuckState.sequence`). This requires `jitter_s` to be a non-trivial fraction of the publish period (≈0.1151s, section 2.1) to have meaningful probability.
- **Drop**: `network_impairment.py` line 41: `if self.config.drop_probability > 0.0 and self._rng.random() < self.config.drop_probability`. **Independent per-message Bernoulli only** — there is no consecutive/burst/periodic dropout logic anywhere in this file or `network_impairment.py`. Confirmed by reading both files in full; no other drop-shaping code exists in the repo (`Grep "burst|consecutive.*drop|outage"` across `src/epuck2_comm/` returns nothing beyond this design doc itself).
- **RNG/seed**: one `random.Random(config.seed)` instance per relay NODE instance (one node per robot). The existing precedent script `run_relay_counter_configurable.py` (lines 60-63) assigns `epuck1` seed `S`, `epuck2` seed `S+1` — this matrix reuses that convention (section 5).
- **RNG call order matters for seed-matching**: `decide()` draws `self._rng.random()` FIRST only if `drop_probability > 0.0` (short-circuit `and`), THEN `self._rng.uniform(...)` only if `jitter_s > 0.0` and the message wasn't dropped. This means the exact sequence of draws consumed from a given seed's stream differs between conditions with different `(drop_probability>0, jitter_s>0)` combinations, even at the identical seed value. Section 5 documents this honestly: "matched seed" here means *same seed value*, not *identical RNG event sequence* across conditions with different impairment types.
- **Relay clock**: `_now_s()` = `self.get_clock().now().nanoseconds / 1e9` — the ROS node clock. Every relay launch site found (`run_comm_baseline_formal_controllers.py`, `run_relay_counter_configurable.py`, `run_diagnostic_relay_and_counter.py`, `run_comm_baseline_pilot.sh`, `run_comm_baseline_native_diagnostic.sh`) passes `"use_sim_time": True` / `-p use_sim_time:=true` to the relay. **The relay runs on Webots simulation time, not wall-clock.**
- **Queue length**: `self._queue` (line 58) is a plain Python list used as a heap via `heapq` — **no explicit maximum size, no bound, no backpressure.** In practice bounded only by `(max release_delay) / (publish_period)` messages in flight.
- **Flush on shutdown**: `destroy_node()` (lines 143-147) only closes the log file (`self._log_file.close()`). **It does not drain/flush the pending delayed-message queue.** Any message still sitting in `self._queue` when the node is destroyed is silently never published. This is a real, evidence-based mechanism (not hypothetical) by which a trial's tail end could show a message gap even under otherwise-stable delay — relevant to Condition C's boundary behavior and noted as an explicit thing to check in the analysis plan, not something to "fix" before running (the matrix's job is to characterize behavior of the real, current tooling).
- **Immediate-passthrough short-circuit**: `is_zero_impairment()` (`network_impairment.py` lines 49-54) is true only when `delay_s<=0 and jitter_s<=0 and drop_probability<=0` — exactly Condition A's configuration; ANY nonzero value on ANY of the three parameters disables the fast path and switches to the 10ms-timer-driven queue, which is the mechanism whose shutdown-flush gap (previous bullet) applies.

## 3. A–G condition table (frozen candidates, pending confirmation)

See `objective5_impairment_matrix_conditions.csv` for the machine-readable version (identical values). All conditions share the fixed items in section 4. **Effects are described as "expected to be observable," never as a required pass/fail outcome — see section 6 for why COLLISION/TASK_TIMEOUT/STALE_STATE_STOP under a genuinely-applied condition is a valid, not excluded, result.**

### A — Zero impairment (control)
- `delay_s=0.0, jitter_s=0.0, drop_probability=0.0`, deterministic (`is_zero_impairment()`=true, no RNG draws occur at all).
- Purpose: control condition; every other condition is a controlled deviation from this one.
- Evidence: this IS the measured baseline (section 1, section 2.1-2.4) — not derived from it.
- Expected effect: none by construction (immediate passthrough, byte-identical to no relay).
- Contrast: reference for all of B-G.

### B — Moderate fixed delay
- `delay_s=0.20, jitter_s=0.0, drop_probability=0.0`, deterministic.
- Rationale: 0.20s ≈ 1.7× the measured publish period (0.1151s, section 2.1) — large enough to be a clearly nonzero, measurable added latency relative to the sampling grain — but well under `peer_timeout_s=0.5` (section 2.2), so it should NOT by itself risk staleness in steady state (a testable prediction, not an assumption suppressed from the report). Also small relative to `cpa_horizon_s=4.0` (5% of the horizon) and to robot travel distance at 0.20s and `nominal_speed_mps=0.025` (~0.005m position lag) — so CPA-prediction error from this delay alone should be small.
- Expected effect: measurable increase in `state_age`, near-zero effect on task success, given the analysis above — but this is what Trial 01 must actually confirm, not assume.
- Contrast: vs A (delay-only, single factor) and vs C (delay magnitude sweep).

### C — High fixed delay
- `delay_s=1.00, jitter_s=0.0, drop_probability=0.0`, deterministic.
- Rationale: chosen to be 2× `peer_timeout_s` (0.5s) specifically to probe the boundary case worked out in section 2.2 — a STABLE delay of this magnitude should still not trip `SAFE_STOP_STALE` in steady state (peer_timeout checks arrival interval, not delay magnitude), but the relay's shutdown-boundary non-flush behavior (section 2.5) becomes far more likely to manifest as an observable tail-end gap at this delay, since up to ~9 messages (1.0s / 0.1151s) can be in flight at any instant. 1.0s / 4.0s horizon = 25% of the CPA prediction window — large enough that CPA staleness effects on `dcpa`/trigger timing should be observable even though the freshness check itself is expected to remain satisfied.
- Expected effect: `state_age` distribution shifted by ~1.0s; possible tail-end message-count discrepancy from the shutdown-flush gap; CPA trigger timing/dcpa accuracy degraded relative to A/B; task completion NOT expected to fail from this factor alone, but genuinely reported either way per section 6.
- Contrast: vs B (delay magnitude sweep, same jitter=drop=0) and vs A.

### D — Jitter / reordering
- `delay_s=0.15, jitter_s=0.30` (uniform range exactly `[0.0, 0.30]`, no negative clamping since `delay_s` is centered in the jitter spread), `drop_probability=0.0`. Randomized (seed, section 5).
- Rationale: jitter spread (0.30s) ≈ 2.6× the measured publish period (0.1151s) — chosen specifically because reordering requires spread comparable to or exceeding inter-message spacing (section 2.5); centering `delay_s` at exactly half the jitter spread keeps the realized release-delay range at `[0, 0.30]` with no artificial clamping bias at either end (a delay_s smaller than jitter_s/2 would clamp negative jitter draws to exactly 0, skewing the distribution — avoided here by construction, not by accident).
- Expected effect: measurable `out_of_order_count` > 0 in the bridge/bag sequence stats (a genuinely new metric value relative to A-C, which are all 0 by construction); `state_age` distribution wider than B/C; whether task success is affected is an open question this condition tests, not a foregone conclusion.
- Contrast: vs A (jitter-only, isolates reordering from pure delay) and vs G (jitter component within combined impairment).

### E — Moderate independent loss
- `delay_s=0.0, jitter_s=0.0, drop_probability=0.15`. Randomized (seed, section 5).
- Rationale: independent-Bernoulli consecutive-drop probability `p^k` at `p=0.15`: `P(5 consecutive drops) = 0.15^5 ≈ 7.6e-5` — chosen so that closing the full `peer_timeout_s=0.5` window (≈4.3 publish periods, section 2.1-2.2) via loss alone stays rare, keeping this condition "moderate" (measurable delivery-ratio degradation, low probability of triggering staleness) rather than "high." Expected steady-state `APPLICATION_STATE_SEQUENCE_DELIVERY_RATIO ≈ 0.85`.
- Expected effect: PDR ≈ 0.85, `missing`/gap counts > 0, low-probability but nonzero chance of an isolated `SAFE_STOP_STALE` tick; task success expected likely but not guaranteed.
- Contrast: vs A (loss-only) and vs G (loss component within combined impairment); matched seeds with G (section 5) isolate the effect of adding delay+jitter on top of the identical drop-event sequence.

### F — Burst loss / dropout (stale/peer-timeout safety test) — **NOT EXECUTABLE with the current relay**
- **The current relay (`network_impairment.py`, section 2.5) implements ONLY independent per-message Bernoulli drop. There is no burst, consecutive-outage, or periodic-dropout mechanism anywhere in the codebase.** Per instruction, this condition is reported as a minimal versioned extension design, not implemented, and not approximated with an extreme independent `drop_probability` (which would not reliably produce a genuine ≥`peer_timeout_s`-length outage — an extreme independent `p` mostly still produces short runs, per the geometric-distribution tail, and would conflate "very lossy" with "briefly fully down," which are scientifically different failure modes this condition specifically wants to separate).
- **Minimal versioned extension proposal (design only, `network_impairment.py` v1.1, NOT implemented this pass)**:
  - New `ImpairmentConfig` fields: `outage_period_s: float = 0.0` (time between the START of successive scheduled outages), `outage_duration_s: float = 0.0` (how long each outage drops every message).
  - `ImpairmentDecider.decide()` gains a deterministic, seed-independent-position check: given elapsed sim time `t` since the relay's own start, if `outage_period_s > 0` and `t mod outage_period_s < outage_duration_s`, forward=False unconditionally (no RNG draw needed for the outage decision itself — the outage schedule is deterministic once seeded/started, only its literal start offset could optionally be seed-jittered in a v1.2 if desired later).
  - Suggested first Condition F parameters (to revisit once implemented, NOT frozen now): `outage_duration_s=0.7` (> `peer_timeout_s=0.5`, guaranteeing at least one `SAFE_STOP_STALE`/`PEER_TIMEOUT_STOP` per outage by construction) at `outage_period_s=15.0` (≈4-5 outages within `max_runtime_s=60.0`), `delay_s=jitter_s=0` outside the outage windows, `drop_probability=0` (all loss comes from the deterministic outage, not independent Bernoulli, keeping the mechanism single-factor and interpretable).
  - Required before Condition F can run: (1) implement the two new fields + the modulo check, (2) unit tests for `ImpairmentDecider` covering the outage boundary (message at `t = outage_start`, `outage_start + outage_duration - epsilon`, `outage_start + outage_duration`), (3) a CSV log field distinguishing `dropped_outage` from `dropped_bernoulli` (currently the log's `action` column only has `dropped`/`forwarded`, section 2.5's log-header quote) so post-hoc analysis can tell the two loss mechanisms apart even if both are ever enabled together, (4) a syntax/unit-test pass and a **separate commit** for the extension itself before any Condition F trial is authorized to run.

### G — Combined impairment
- `delay_s=0.20, jitter_s=0.20, drop_probability=0.10`. Randomized (seed, section 5).
- Rationale: each component set at a level individually comparable to a single-factor condition (delay ≈ B's 0.20s; jitter spread 0.20s, smaller than D's 0.30s to keep G "moderate-combined" rather than "worst-of-both"; drop_probability 0.10, below E's 0.15) so that a superposition/interaction analysis against B ∪ D ∪ E is meaningful (section 8) — G is not simply "the worst of everything," it is a realistic-degraded-link composite at moderate-per-factor levels.
- Expected effect: combined degradation in PDR, state_age, and possibly reordering; whether the combination is worse than the sum of B/D/E's individual effects (interaction) or merely additive is exactly what this condition is designed to reveal — not assumed in advance.
- Contrast: vs A (full combined effect) and vs each of B, D, E individually (isolate which factor dominates); matched seeds with E (section 5).

## 4. Common fixed items (locked across every condition)

| item | value | source |
|---|---|---|
| Webots world / scenario script | `run_dual_head_on_clean.py` (outside git tree, `/mnt/c/.../simulation_comm_experiment_v1/working`, per `run_objective5_comm_baseline_formal_trial.sh` line 210) | unchanged since trial01/trial02_stamp |
| Initial pose | epuck1 `(x=-0.35, y=0.0, yaw=0.0)`, epuck2 `(x=0.35, y=0.0, yaw=π)` | `run_comm_baseline_formal_controllers.py` |
| Controller commit | `controller_v4_timebase_fix_20260717` (`980e7d0`/`06e0f0f`/`f1830c5`), confirmed unchanged via `git log` (section 1.1) | `cooperative_avoider.py`, `local_obstacle_logic.py` |
| Protocol/message commit | `EpuckState.msg` SHA-256 `a7ec4184...`, frozen at `06dae306` | `PROTOCOL_FREEZE_20260717.md` |
| Speed/CPA/local-avoidance thresholds | `nominal_speed_mps=0.025, avoidance_speed_mps=0.012, turn_rate_rps=0.65, cpa_horizon_s=4.0, safety_radius_m=0.14, trigger_distance_m=0.34, release_distance_m=0.24, rearm_distance_m=0.45` — **not changed by any condition** | `cooperative_avoider.py` declared defaults |
| Total run time | `max_runtime_s=60.0` ceiling, `stop_after_recovery=True`, `post_recovery_hold_s=0.5`, `startup_hold_s=5.0` | `run_comm_baseline_formal_controllers.py` / `cooperative_avoider.py` |
| n per condition | 5 | this document |
| Recorded topics | `/epuck1/state_raw /epuck2/state_raw /epuck1/state /epuck2/state /epuck1/cmd_vel /epuck2/cmd_vel` | `run_objective5_comm_baseline_formal_trial.sh` |
| Analyzer | `analyze_comm_performance.py` (`--warmup-s 2.0 --cooldown-s 2.0 --peer-timeout-s 0.5`) + `analyze_objective5_formal_baseline.py` + `analyze_trigger_reason.py`, extended per `objective5_impairment_matrix_analysis_plan.md` | unchanged scripts, new orchestration only |
| Realtime factor threshold | `realtime_factor_ok` boolean already computed by the existing verdict script (preload/full_load factors both expected within the same tolerance band the formal trials already use, ~0.95-1.05) | `objective5_formal_baseline_verdict.json` field |
| Recording convention | native WSL ext4 (`/home/eamon/epuck_comm_bags`) first, copied into git tree only after clean stop + non-empty `metadata.yaml` | `run_objective5_comm_baseline_formal_trial.sh` |
| Unique directory / attempt rule | one directory per `condition_X_trialNN`; a failed/interrupted attempt is renamed with an `_attemptNN` suffix and preserved, never overwritten, matching the discipline already used for `physical_single_device_zero_impairment_baseline_v1`'s `trial01_attempt01_short_window` | this session's established convention |
| Success/collision/safe-stop criteria | see `objective5_impairment_matrix_analysis_plan.md` section on TASK_OUTCOME classification | new for this matrix |
| Simulator | Webots (current, validated) — **no migration to Gazebo**; "Webots-vs-Gazebo equivalence requires supervisor confirmation" limitation remains in force, unchanged | per instruction |

## 5. Repetition and randomness

- Condition A: deterministic, no seed applies (n=5: Trial01=reused trial02_stamp, Trials02-05=new, same zero-impairment config).
- Conditions B, C: deterministic (no RNG draws occur — `jitter_s=0` and `drop_probability=0` in both, so `ImpairmentDecider.decide()` never calls `self._rng` at all). n=5 trials each are still required (task-completion timing, controller-internal state-machine timing, and Webots physics have their own trial-to-trial variance even with a deterministic relay) but do not need distinct seeds.
- Conditions D, E, G: randomized. **5 base seeds, reusing the project's existing precedent value** (`objective5_timestamp_latency_validation_pilot01` used `seed=4001`, registry row confirmed): `{4001, 4002, 4003, 4004, 4005}` — trial `NN` (`02`..`05`, `01` for the first new trial in each condition) uses base seed `4000+NN`. Per-robot assignment follows the existing convention in `run_relay_counter_configurable.py` (lines 60-63): epuck1 gets the base seed, epuck2 gets `base+1`.
- **Matched-seed pairing**: Conditions E and G use the **identical 5 base seeds** — same trial index, same seed value — so the underlying PRNG stream starts identically for both. Section 2.5 documents the caveat honestly: because `decide()`'s RNG consumption pattern differs between E (drop-only, always draws exactly one `random()` call per message) and G (drop_probability>0 AND jitter_s>0, draws `random()` then conditionally `uniform()`), the two conditions do **not** receive the exact same sequence of drop/no-drop outcomes message-for-message despite the shared seed — "matched" here means matched starting seed value (a considered, documented choice), not a guaranteed identical event trace. Condition D's own 5 seeds also reuse `{4001..4005}` for consistency, with the same caveat noted.
- Conditions F: seed scheme deferred to whenever the outage-based extension (section 3, Condition F) is actually implemented — no seed is frozen for an unbuilt mechanism.
- Every trial gets a unique directory name (`condition_X_trialNN`); no bag directory is ever reused or overwritten; a failed/interrupted attempt is preserved under an `_attemptNN` suffix, matching this session's established convention for the physical baseline batch.

## 6. Two-dimensional verdict (DATA_VALIDITY × TASK_OUTCOME)

Every trial produces BOTH labels independently — one is never inferred from the other.

**DATA_VALIDITY** (infrastructure/measurement question — "did we get a
trustworthy record of what happened?"):
- `VALID`: relay parameters and seed match the condition's frozen
  config; bag `metadata.yaml` present/non-empty; `realtime_factor_ok`
  true; sequence/PDR statistics computed successfully; analyzer ran to
  completion without error; no infrastructure abort (WSL interop, bag
  recorder crash, launch failure, etc.).
- `INVALID`: any of the above fails — e.g. wrong relay parameter
  deployed, `realtime_factor` far outside tolerance, bag truncated,
  analyzer crashed, orchestration script aborted before completion.
  **Only `INVALID` stops the batch for diagnosis, per instruction.**

**TASK_OUTCOME** (scientific-result question — "what did the robots
actually do under this condition?"): `SUCCESS`, `SAFE_DEGRADATION`
(task did not complete cleanly but no unsafe event occurred — e.g. hit
`max_runtime_s` while still safely separated), `COLLISION`,
`TASK_TIMEOUT`, `STALE_STATE_STOP` (`SAFE_STOP_STALE` from `_fresh()`
returning false), `PEER_TIMEOUT_STOP` (same underlying mechanism as
`STALE_STATE_STOP`, labeled separately when directly attributable to
peer freshness specifically vs. own-state staleness), or an explicit
other named category if a trial exhibits something not covered above
(never silently forced into the nearest existing bucket).

**A `COLLISION`/`TASK_TIMEOUT`/`STALE_STATE_STOP` outcome produced by a
frozen, correctly-applied impairment condition is a valid experimental
result with `DATA_VALIDITY=VALID`.** It is never used to exclude the
trial, retry it, or stop the condition's remaining trials. Only
`DATA_VALIDITY=INVALID` triggers a stop-and-diagnose. No controller
parameter, CPA threshold, `peer_timeout_s`, speed, or relay parameter is
ever adjusted in response to a TASK_OUTCOME, for any condition, at any
point — conditions are frozen per section 3 before Trial 01 of each
condition runs and never revisited based on results.

## 7. Physical-baseline comparison boundary

`physical_single_device_zero_impairment_baseline_v1` (FINAL_BATCH_PASS,
5/5, single stationary e-puck2, no task, no second robot) may be
compared against simulation Condition A **only** on: application-level
state-sequence delivery ratio, RTT/status-snapshot distribution
(physical only — no simulation equivalent exists, since sim relay
delay=0 produces `message_age_s=0.0` by construction, not a snapshot
distribution), message frequency, CPU/RAM/Wi-Fi resource figures, and
qualitative protocol/transport stability. **It must never be presented
as evidence about, or compared against, dual-robot task success rate,
collision rate, or avoidance/recovery completion** — the physical batch
never launched a controller, never had a second robot, and never
attempted the cooperative-avoidance task at all.

## 8. Execution plan (not run; awaiting confirmation)

1. Condition A Trials 02-05 (4 trials, existing formal orchestrator, no
   new tooling needed) — can start immediately once confirmed.
2. Conditions B, C, D, E, G (25 trials) — require a NEW orchestrator
   variant accepting `--delay-s/--jitter-s/--drop-probability/--seed`
   CLI args, built by combining `run_objective5_comm_baseline_formal_trial.sh`'s
   full pipeline (controllers + bag + analyzer + native-WSL recording)
   with `run_relay_counter_configurable.py`'s parameterized relay launch
   pattern — this new script does not exist yet and must be written,
   syntax-checked, and (for its parameter-passing logic) unit-tested
   before Trial 01 of Condition B can run.
3. Condition F (5 trials) — blocked on the relay extension (section 3)
   being implemented, tested, and committed separately; not part of the
   immediately-executable 29.
4. **Total immediately-executable new trials: 4 (A) + 5×4 (B,C,D,E,G) = 24... 
   recount: A=4, B=5, C=5, D=5, E=5, G=5 → 29 new trials executable now.
   F=5 more once its extension lands (34 total once F is included, matching
   the "34 new trials" figure from the instruction — but only 29 of those
   34 can start under the current, unmodified relay).**

See `objective5_impairment_matrix_analysis_plan.md` for the full
per-trial/per-condition metrics and statistics plan.

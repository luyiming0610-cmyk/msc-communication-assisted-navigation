# Objective 5 communication impairment matrix — design v1 (revision 2)

Status: **design only, not executed**. No A-G simulation trial has been
run from this document. This is revision 2 of this design (revision 1
sealed as commit `a2ee878`) after user review rejected reusing
`objective5_comm_baseline_zero_impairment_formal_trial02_stamp` as
Condition A Trial 01 -- see section 1 below for why, and what changed.

**Unified tooling freeze**: every condition A-G in this matrix's
eventual n=35 run must use the SAME frozen relay/controller/analyzer/
orchestrator commit, because Condition F requires the burst/outage relay
extension (v1.1), which necessarily post-dates every commit this design
doc previously cited for Condition A's compatibility check. That
extension is now implemented and tested (commit `f0857f9`,
`network_impairment.py`/`network_impairment_relay.py`), and the unified
parameterized orchestrator all seven conditions will use is built and
tested (commit `4e79b5a`,
`experiments/05_objective5_impairment_matrix/tools/`). **Neither commit
has run a single A-G trial yet.** Once the user confirms this revised
design, Condition A's n=5 will be recorded FRESH under this same frozen
commit pair -- not reused from any earlier trial, including
trial02_stamp.

This design supersedes the retracted claim (see
`experiments/3-3.全传感器避障实验/README.md` lines
77-84 and `experiments/project_status.json`'s `known_limitations[4]`)
that "a 0.6s configured delay would trigger peer-timeout degradation."
That claim is wrong for the reason worked out in section 2.5 below: a
**stable** fixed delay does not change the interval between successive
message arrivals, so it cannot by itself cause a staleness timeout no
matter how large it is, as long as it is applied uniformly per message
(no per-message accumulation, no queue backlog). This document does not
repeat that error.

## 1. Condition A disposition (revised)

### 1.1 `objective5_comm_baseline_zero_impairment_formal_trial02_stamp` — kept as PRE_MATRIX_FORMAL_VALIDATION, NOT reused

Commit `59588e9`, verdict PASS, `fail_reasons: []`. Design v1 proposed
reusing this trial as Condition A Trial 01 -- rejected on review because
Condition F's relay extension (necessary to complete the matrix, section
3) changes the relay's own commit, and every condition in this matrix
must share ONE frozen relay/controller/analyzer/orchestrator commit
(section "Unified tooling freeze" above). trial02_stamp ran under a
relay commit (`03ce36c`) that predates the v1.1 burst/outage extension
(`f0857f9`) by definition -- it cannot be the same frozen commit every
other condition uses, so it cannot be part of this matrix's own n=5,
regardless of how compatible its OTHER dimensions are (protocol,
timestamp semantics, controller, scenario, topics, bag convention,
analyzer -- all still genuinely compatible, and still useful as
independent pre-matrix validation evidence that the underlying
zero-impairment path works).

**Disposition**: relabeled `PRE_MATRIX_FORMAL_VALIDATION` in this
document and in the registry (a follow-up registry update, not part of
this design-doc commit) -- kept exactly as-is, not deleted, not
demoted, cited as independent corroborating evidence, but explicitly
**not** one of Condition A's 5 trials.

### 1.2 Old `objective5_comm_baseline_zero_impairment_formal_trial01` — LEGACY/EXCLUDED (unchanged from v1)

Commit `9f4d7b2`. `metric_coverage: ... latency=NOT_MEASURED` (registry
row, `experiment_registry.csv` line 36) — permanent, not backfilled.
Kept as LEGACY/EXCLUDED, not deleted, not recomputed, not counted
toward Condition A's n=5. (Same disposition as v1; restated for
completeness now that trial02_stamp has moved to the same "not counted"
bucket for a different reason.)

### 1.3 Condition A execution status (revised)

**All 5 Condition A trials are new**, run under the unified frozen
commit pair (`f0857f9` relay extension, `4e79b5a` orchestrator), using
the same zero-impairment parameters trial02_stamp and trial01 both
already validated (`delay_s=0.0 jitter_s=0.0 drop_probability=0.0`,
`outage_period_s=0.0` -- disabled, confirmed byte-equivalent to the
pre-extension relay by `test_default_outage_relay_forwards_identically_to_pre_extension_relay`).
Not started; pending this revised document's confirmation. Command,
once authorized:
`run_objective5_impairment_matrix_trial.sh A 1` through
`run_objective5_impairment_matrix_trial.sh A 5`.

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
- **Jitter, exact formula (verified by test, not just read)**: `jitter_s` is the FULL peak-to-peak spread, not a half-amplitude. Each decision draws `jitter ~ Uniform(-jitter_s/2, +jitter_s/2)` (`network_impairment.py`), then `release_delay = max(0.0, delay_s + jitter)` — the floor applies to the SUM, never to `delay_s` or `jitter_s` individually. — **not** Gaussian, **not** one-sided. If `delay_s >= jitter_s/2`, the floor never actually clips anything (empirically confirmed, 20000 samples each): Condition D (`delay_s=0.15, jitter_s=0.30`, exactly at the boundary) has realized range `[0.0, 0.30]`, observed minimum `<0.002` (matches the true 0.0 floor closely, not clamped away from it), mean `0.15±0.01`, variance matching the theoretical `Uniform(0,0.30)` value `0.0075`; Condition G (`delay_s=0.20, jitter_s=0.20`) has realized range `[0.10, 0.30]`, observed minimum `>=0.10-1e-9` and `<0.105` (confirms no clamping, the range genuinely starts at 0.10 not artificially bounded away from it), mean `0.20±0.01`, variance matching `Uniform(0.10,0.30)`'s theoretical `0.003333`. Both cases: **no probability-mass spike at 0.0** (that only happens when `delay_s < jitter_s/2`, which neither D nor G's frozen parameters trigger). Reordering (D, G) confirmed possible via a dedicated empirical test: at a jitter spread of `0.30s` against the measured `0.1151s` publish period, two consecutively-generated release-delay draws produce a release-order crossover in a nonzero fraction of 2000 repeated trials.
- **Reordering**: possible and NOT prevented by the relay — because the queue flushes in `release_time_s` order, if message N's `jitter_N` is large-negative and message N+1's `jitter_{N+1}` is large-positive, `release_time_N` can exceed `release_time_{N+1}`, and the relay will deliver N+1 to the controller before N (a genuine out-of-order delivery, correctly countable via `EpuckState.sequence`). This requires `jitter_s` to be a non-trivial fraction of the publish period (≈0.1151s, section 2.1) to have meaningful probability.
- **Drop**: `network_impairment.py` line 41: `if self.config.drop_probability > 0.0 and self._rng.random() < self.config.drop_probability`. **Independent per-message Bernoulli only** — there is no consecutive/burst/periodic dropout logic anywhere in this file or `network_impairment.py`. Confirmed by reading both files in full; no other drop-shaping code exists in the repo (`Grep "burst|consecutive.*drop|outage"` across `src/epuck2_comm/` returns nothing beyond this design doc itself).
- **RNG/seed**: one `random.Random(config.seed)` instance per relay NODE instance (one node per robot). The existing precedent script `run_relay_counter_configurable.py` (lines 60-63) assigns `epuck1` seed `S`, `epuck2` seed `S+1` — this matrix reuses that convention (section 5).
- **RNG call order matters for seed-matching**: `decide()` draws `self._rng.random()` FIRST only if `drop_probability > 0.0` (short-circuit `and`), THEN `self._rng.uniform(...)` only if `jitter_s > 0.0` and the message wasn't dropped. This means the exact sequence of draws consumed from a given seed's stream differs between conditions with different `(drop_probability>0, jitter_s>0)` combinations, even at the identical seed value. Section 5 documents this honestly: "matched seed" here means *same seed value*, not *identical RNG event sequence* across conditions with different impairment types.
- **Relay clock**: `_now_s()` = `self.get_clock().now().nanoseconds / 1e9` — the ROS node clock. Every relay launch site found (`run_comm_baseline_formal_controllers.py`, `run_relay_counter_configurable.py`, `run_diagnostic_relay_and_counter.py`, `run_comm_baseline_pilot.sh`, `run_comm_baseline_native_diagnostic.sh`) passes `"use_sim_time": True` / `-p use_sim_time:=true` to the relay. **The relay runs on Webots simulation time, not wall-clock.**
- **Queue length**: `self._queue` (line 58) is a plain Python list used as a heap via `heapq` — **no explicit maximum size, no bound, no backpressure.** In practice bounded only by `(max release_delay) / (publish_period)` messages in flight.
- **Flush on shutdown**: `destroy_node()` still (v1.1, unchanged from v1.0) only closes the log file. **It does not drain/flush the pending delayed-message queue.** This is not "fixed" in the relay itself -- per instruction, the correct fix lives in the ORCHESTRATOR, not the relay: `run_objective5_impairment_matrix_trial.sh` (commit `4e79b5a`) now holds the relay/clock/counter/bag running (with the controller already stopped and both robots' commanded velocity at zero) for `max_configured_delivery_delay + 2 publish periods` (`relay_drain.py`'s `compute_drain_duration_s`) after task completion, then polls each robot's new `relay_status` topic (1Hz, `{received_count, forwarded_count, dropped_bernoulli_count, dropped_outage_count, pending_queue_depth}`) until `pending_queue_depth == 0` before stopping the relay and bag. If the queue is still nonzero after the drain window, the trial's `DATA_VALIDITY` is set to `INVALID` (never silently treated as a real network-impairment result, per section 6's two-dimensional verdict). `max_configured_delivery_delay` itself is `delay_s + jitter_s/2` (`ImpairmentDecider.max_release_delay_s()`, mirrored independently in `relay_drain.max_configured_delivery_delay_s()` so the orchestrator-side calculation doesn't need to import the ROS package) -- outage-dropped messages never enter the queue at all (they're rejected at `release_delay_s=0.0`), so outage parameters do not extend the drain wait.
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

### F — Burst loss / dropout (stale/peer-timeout safety test) — **extension implemented (commit `f0857f9`), frozen parameters below, not yet run**
- **v1.1 extension, as actually built** (`network_impairment.py`, `ImpairmentConfig.outage_period_s`/`outage_duration_s`/`outage_phase_s`, all default `0.0` = disabled): `ImpairmentDecider.decide(elapsed_s)` checks
  `(elapsed_s - outage_phase_s) % outage_period_s < outage_duration_s` BEFORE the Bernoulli drop check -- a pure, stateless function of `elapsed_s` (no RNG draw, correct even under a backward sim-time jump, per `test_outage_is_a_pure_function_correct_under_backward_time_jump`). `drop_reason` distinguishes `"outage"` from `"bernoulli"` in both the relay's return value and its CSV log (new `drop_reason` column, appended not inserted).
- **Frozen parameters** (derived, not guessed, shown below): `outage_duration_s=0.7`, `outage_period_s=15.0`, `outage_phase_s=10.0`, `delay_s=jitter_s=drop_probability=0.0` outside/independent of outage windows (all loss is the deterministic outage mechanism, never independent Bernoulli, keeping F single-factor and interpretable).
- **Derivation, from the same measured quantities as every other condition**:
  - `outage_duration_s`: must exceed `peer_timeout_s=0.5` with margin for at least one full publish period (`~0.1151s`, section 2.1) and one control period (`0.05s`, section 2.1), so the controller's own freshness check is guaranteed to actually observe the gap, not just barely miss it: `0.5 + 0.1151 + 0.05 = 0.6651s`, rounded up to **0.7s** (margin over `peer_timeout_s`: `0.2s`, exceeding the required `0.1651s`).
  - `outage_period_s`: chosen so multiple outages occur within `max_runtime_s=60.0` with ample normal-operation recovery time between them, without the trial being "mostly down": **15.0s** gives `floor((60-10)/15)+1 = 4` outages (at elapsed `10, 25, 40, 55`), each separated by `15.0/0.5 = 30` `peer_timeout_s` windows of normal operation -- duty cycle `4*0.7/60 ≈ 4.7%`.
  - `outage_phase_s`: the first outage must not land before the encounter dynamic has had time to develop -- `startup_hold_s=5.0` (section 2.4) plus a buffer, so **10.0s**.
  - Expected outcome (not required, not assumed): 4 outage windows per trial, each individually designed to force at least one `STALE_STATE_STOP`/`PEER_TIMEOUT_STOP` by construction; whether the controller cleanly recovers after each, or degrades cumulatively across outages, is exactly what this condition tests.
- **Tests** (13 new in `test_network_impairment.py`, 4 new in `test_network_impairment_relay.py`, commit `f0857f9`): outage window boundary closed/open semantics, recurrence across multiple periods, correctness before the first phase offset, correctness under backward time jump, combined-with-Bernoulli precedence (outage checked first, short-circuits the RNG draw), zero-period/zero-duration disables outage, `is_zero_impairment()` correctly false when outage is configured, plus node-level drop_reason CSV logging and default-outage node-level equivalence to the pre-extension relay.
- **Empirical confirmation (`objective5_matrix_v1_conditionF_exclusionary_pilot03`, exclusionary, not formal n=5)**: code-level bidirectional synchronization is a proof, not a measurement (`test_both_relay_instances_reading_same_sim_clock_see_synchronized_outage_windows`, both relay instances reading the same absolute `/clock` value classify identically regardless of construction-time offset). Reconstructing this pilot's actual outage windows from its raw per-message relay CSV logs (5 windows, both directions) shows **0.0000s** measured start- and end-time deviation between directions across all 5 comparably-reconstructed windows — consistent with, and supporting, the code-level guarantee. See that pilot's `NOTE.md` for the full per-window table and the honest precision caveats (reconstruction resolution is bounded by ~0.115s message-arrival discretization, and one early window left no message to reconstruct from at all — a limitation of the reconstruction method, not evidence the mechanism itself missed it). This is stated as "windows reconstructed from this run are consistent with synchronized behavior," not as a claim of sub-message-period ("逐时刻") boundary alignment, which discrete message arrivals cannot resolve.

### G — Combined impairment
- `delay_s=0.20, jitter_s=0.20, drop_probability=0.10`. Randomized (seed, section 5).
- Rationale: each component set at a level individually comparable to a single-factor condition (delay ≈ B's 0.20s; jitter spread 0.20s, smaller than D's 0.30s to keep G "moderate-combined" rather than "worst-of-both"; drop_probability 0.10, below E's 0.15) so that a superposition/interaction analysis against B ∪ D ∪ E is meaningful (section 8) — G is not simply "the worst of everything," it is a realistic-degraded-link composite at moderate-per-factor levels.
- Expected effect: combined degradation in PDR, state_age, and possibly reordering; whether the combination is worse than the sum of B/D/E's individual effects (interaction) or merely additive is exactly what this condition is designed to reveal — not assumed in advance.
- Contrast: vs A (full combined effect) and vs each of B, D, E individually (isolate which factor dominates); matched seeds with E (section 5).

## 4. Common fixed items (locked across every condition)

| item | value | source |
|---|---|---|
| Webots world / scenario script | `run_dual_head_on_clean.py` (outside git tree, `/mnt/c/.../2-1.仿真通信实验/working`, per `run_objective5_comm_baseline_formal_trial.sh` line 210) | unchanged since trial01/trial02_stamp |
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

## 5. Repetition and randomness (finalized, concrete seeds)

- Condition A: deterministic, no seed applies. n=5, all new (section 1.3).
- Conditions B, C: deterministic (`jitter_s=0`, `drop_probability=0`, `outage_period_s=0` — `ImpairmentDecider.decide()` never calls `self._rng` at all). n=5 trials each still required (task-completion timing, controller-internal state-machine timing, and Webots physics have their own trial-to-trial variance even with a deterministic relay) but no seed needed.
- **Conditions D, E, F, G: randomized. Final, non-overlapping, per-direction seed mapping** (supersedes an earlier `base`/`base+1` scheme that allowed cross-trial seed reuse, e.g. trial 1's reverse direction and trial 2's forward direction would have collided on `4002` — caught and fixed before any formal trial ran):

  | Trial | epuck1→epuck2 | epuck2→epuck1 |
  |---|---|---|
  | 01 | 4001 | 14001 |
  | 02 | 4002 | 14002 |
  | 03 | 4003 | 14003 |
  | 04 | 4004 | 14004 |
  | 05 | 4005 | 14005 |

  Ten distinct values total; no seed value is ever reused across trials, directions, or conditions (`test_no_seed_value_is_ever_reused_across_the_whole_randomized_batch`). D/E/F/G all share this identical table, indexed by trial number.
- **Two-direction seed mapping, explicit** (each condition has TWO relay instances, one per robot's outgoing state stream -- see section 2.5's "one relay instance per robot" note): the **epuck1-to-epuck2 direction** (the relay in the `epuck1` namespace, relaying epuck1's own state to epuck2's controller) and the **epuck2-to-epuck1 direction** (the relay in the `epuck2` namespace) each get their own seed from the table above -- never the same value, and never derived from one another by a fixed offset within the same trial (the `+10000` gap between the two columns is purely a human-readable range separator, not a claim about the RNG relationship between the two streams). `test_two_direction_seeds_base_and_base_plus_one_produce_different_sequences` confirms the two directions' seeds produce genuinely different decision sequences.
- **Matched-seed pairing (E vs G)**: same 5-trial seed table above, for both conditions. Caveat, stated honestly: `decide()`'s RNG consumption differs between E (drop-only, always exactly one `random()` call per message) and G (`drop_probability>0` AND `jitter_s>0`, draws `random()` then conditionally `uniform()`), so E and G do **not** receive byte-identical drop/no-drop event sequences despite the shared seed -- "matched" means matched starting seed value (a considered, documented choice, verified reproducible via `test_same_seed_reproduces_the_identical_decision_sequence`), not a guaranteed identical event trace. Condition D's own seeds reuse the same table for consistency across the whole randomized set, with the same caveat.
- Condition F: same seed scheme (`seed` parameter still set per relay instance, for consistency and CSV-log provenance), but the outage schedule itself is deterministic and does NOT depend on the seed at all (section 3's `_in_outage` check draws no random number) -- only the (currently zero, per F's frozen params) Bernoulli component would ever consume it.
- Every trial gets a unique directory name (`objective5_impairment_matrix_v1_condition_<ID>_trial<NN>_attempt<NN>`, `unique_trial_dir.py`); `require_unique_trial_dir()` refuses to overwrite an existing directory; a failed/interrupted attempt is preserved under an incremented `_attemptNN`, matching the convention established for the physical baseline batch.
- **Condition D Trial 06 -- sole preregistered replacement for excluded Trial 04** (recorded 2026-07-19, user-authorized): D04 was retained as valid `DATA_ARTIFACT_INTEGRITY`/`MANIPULATION_VALIDITY`/`TASK_OUTCOME` data but excluded from the formal n=5 (`FORMAL_MEASUREMENT_VALIDITY=INVALID`, `FORMAL_BATCH_INCLUSION=EXCLUDED`) due to a rosbag-only single-message capture gap (sequence 17, epuck2→epuck1, forwarded by the relay and received live by the independent online `sequence_counter` counter, but never captured by the bag). A sixth deterministic seed pair, `4006`/`14006` (continuing the existing base+index pattern, not hand-picked), was appended to Condition D's `seed_epuck1_to_epuck2`/`seed_epuck2_to_epuck1` columns in `objective5_impairment_matrix_conditions.csv` for this sole purpose: **D06 replaces excluded D04's measurement-chain attempt.** `n_trials` for Condition D remains `5` -- the formal statistics still comprise exactly 5 valid trials (D01, D02, D03, D05, D06), never 6. D06 is not a general extension of the seed table and is not available to any other condition (E/F/G's own seed lists are untouched). CSV SHA-256 before this change: `7d0f31106c2cbcbe2355f158ad554f5769c1033ac8b505f8e3a65920dd1ad01f`; after: `f98a47b667aa4f23c09f562084eff4e555770d10da089173468ffa0a72c4d2d2`.

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
actually do under this condition?") is computed ONLY when
`DATA_VALIDITY=VALID` (otherwise `NOT_EVALUABLE` — a task outcome
cannot be trusted from data that already failed its own validity
checks). **As actually implemented in code**
(`matrix_verdict.classify_task_outcome`, superseding an earlier
six-category design sketch that was never wired into code — see below),
the four values are:
- `SUCCESS`: `complete_count >= expected_complete_count` (both robots
  reached their goal) and no collision.
- `SAFE_DEGRADATION`: task did not complete cleanly (e.g. hit
  `max_runtime_s`, or a safe stop that never recovered) but no unsafe
  event occurred.
- `UNSAFE_FAILURE`: the controller process crashed/exited abnormally,
  OR `min_interrobot_distance_m` fell below `safety_radius_m` at any
  point in the trial (a deliberately conservative collision heuristic —
  documented in `classify_task_outcome`'s own docstring — that does not
  attempt to distinguish a genuine collision from a very close clean
  pass via closing-speed sign).
- `NOT_EVALUABLE`: `DATA_VALIDITY=INVALID`.

**Note on an earlier drafting error**: an earlier revision of this
document, and one conversational status report during this session,
described a six-category scheme (`COLLISION`, `TASK_TIMEOUT`,
`STALE_STATE_STOP`, `PEER_TIMEOUT_STOP` as separate values) that was
never implemented — the simplified four-category scheme above is what
`matrix_verdict.py` actually outputs, confirmed by
`test_task_outcome_success`/`_safe_degradation_on_incomplete_but_no_collision`/
`_unsafe_failure_on_close_distance_even_if_complete`/`_unsafe_failure_on_controller_crash_regardless_of_distance`/
`_not_evaluable_when_data_invalid`, none of which return the value
`"VALID"` or `"INVALID"` for TASK_OUTCOME — those two strings are
exclusively DATA_VALIDITY values, and `TASK_OUTCOME` never takes them.
`test_task_outcome_signals_never_return_a_data_validity_string` makes
this non-overlap explicit.

**A `SAFE_DEGRADATION` or `UNSAFE_FAILURE` outcome produced by a frozen,
correctly-applied impairment condition is a valid experimental result
with `DATA_VALIDITY=VALID`.** It is never used to exclude the trial,
retry it, or stop the condition's remaining trials. Only
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

## 8. Execution plan (revised: all 7 conditions now tooled, 35 new trials total, none run)

Per the unified-tooling-freeze requirement (top of this document),
Condition A is no longer partially-reused -- **every condition is now a
fresh n=5 under the same frozen commit pair** (`f0857f9` relay
extension, `4e79b5a` orchestrator):

| condition | new trials | tooling status |
|---|---|---|
| A | 5 | ready (unified orchestrator, zero-impairment params) |
| B | 5 | ready |
| C | 5 | ready |
| D | 5 | ready (randomized, seeds frozen section 5) |
| E | 5 | ready (randomized) |
| F | 5 | ready (relay extension implemented+tested; frozen outage params section 3) |
| G | 5 | ready (randomized, matched seeds with E) |
| **total** | **35** | all seven conditions executable under the same frozen commit once confirmed |

All 35 trials use `run_objective5_impairment_matrix_trial.sh CONDITION_ID TRIAL_INDEX`
(`experiments/05_objective5_impairment_matrix/tools/`, commit `4e79b5a`)
-- the single unified orchestrator for every condition, parameters
resolved exclusively from the frozen
`objective5_impairment_matrix_conditions.csv` via
`load_condition_config.py` (no hand-typed override path exists). This
script has NOT been run end-to-end (building/syntax-checking/unit-
testing its parameter-resolution and drain-duration logic is not the
same as an actual Webots-integrated run, and no such run has happened,
per instruction). The first real run of ANY condition is therefore
still an open risk surface for issues this design/build pass could not
catch (e.g. an actual Webots launch timing edge case) -- Condition A
Trial 01 is deliberately the first one run, manually observed, exactly
as the physical baseline's own first-trial discipline required.

See `objective5_impairment_matrix_analysis_plan.md` for the full
per-trial/per-condition metrics and statistics plan.

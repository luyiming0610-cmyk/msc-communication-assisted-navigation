# Objective 5 Condition G formal batch summary

## Verdict

`FINAL_BATCH_PASS` for **data validity only**: 5/5 formal Webots trials produced trustworthy, complete evidence (`DATA_VALIDITY=VALID` for every trial). **Task-outcome results are mixed, not uniformly successful**: 4/5 trials were `SUCCESS`, and **1/5 (Trial 02) was a genuine `UNSAFE_FAILURE`** (a real, valid safety-radius violation, retained as-is, not excluded, retried, or reclassified).

## Frozen configuration and scope

- `delay_s=0.20`, `jitter_s=0.20` (full peak-to-peak spread, implemented as `Uniform(-0.10, +0.10)` added to `delay_s`; `release_delay_s = max(0.0, delay_s + jitter)`), `drop_probability=0.10` (independent Bernoulli), `outage_period_s=outage_duration_s=outage_phase_s=0.0` (outage mechanism disabled for this condition).
- `peer_timeout_s=0.5`, `safety_radius_m=0.14`, `max_runtime_s=60.0`, `startup_hold_s=5.0`, `stop_after_recovery=true`, `post_recovery_hold_s=0.5` -- unchanged from every other condition in this matrix.
- Seeds: `epuck1→epuck2 = [4001..4005]`, `epuck2→epuck1 = [14001..14005]`, matched with Condition E's own seed table per the design doc's matched-seed pairing (same starting seed value, not a guaranteed identical event trace -- see design doc section 5's caveat).
- Execution commit for all five trials: `e80ae7565a1f4583879382ad388dc3a4a8167e75`.

## Counted trials

| Trial | Trial ID | Seeds (e1→e2 / e2→e1) | DATA_VALIDITY | TASK_OUTCOME | Min. distance | Safety margin |
|---|---|---|---|---|---:|---:|
| 1 | `..._trial01_attempt01` | 4001 / 14001 | VALID | SUCCESS | 0.143197 m | +3.197 mm |
| 2 | `..._trial02_attempt01` | 4002 / 14002 | VALID | **UNSAFE_FAILURE** | 0.139669 m | **−0.331 mm** |
| 3 | `..._trial03_attempt01` | 4003 / 14003 | VALID | SUCCESS | 0.149891 m | +9.891 mm |
| 4 | `..._trial04_attempt01` | 4004 / 14004 | VALID | SUCCESS | 0.143048 m | +3.048 mm |
| 5 | `..._trial05_attempt01` | 4005 / 14005 | VALID | SUCCESS | 0.142903 m | +2.903 mm |

No attempt was excluded or invalid; every trial used `attempt01`.

## Task outcome counts

`SUCCESS=4`, `SAFE_DEGRADATION=0`, `UNSAFE_FAILURE=1`, `NOT_EVALUABLE=0`.

## Completion and recovery

All five trials: `controller_complete_count=2/2` (both robots completed), `controller_crashed=false` in every trial -- including Trial 02, whose safety-radius violation occurred despite (not because of) task incompletion. Completion is never treated as a proxy for safety in this batch.

## Configured vs. realised packet loss

| Direction | Configured | Realised (mean across 5 trials) | Range |
|---|---:|---:|---:|
| epuck1→epuck2 | 0.10 | 0.0994 | 0.0884 – 0.1165 |
| epuck2→epuck1 | 0.10 | 0.1044 | 0.0729 – 0.1186 |

## Measured message-age / delay distribution (mean across 5 trials)

| Direction | Mean age | Notes |
|---|---:|---|
| epuck1→epuck2 | 0.2100 s | p95/p99/max consistently at or near 0.30 s in every trial |
| epuck2→epuck1 | 0.2092 s | same |

**Realised jitter is `NOT_AVAILABLE` for every trial.** The mean/median/p95/p99/max figures above are message-age (measured-delay) distribution statistics produced by the existing analyzer; they are not a separately calculated jitter statistic, and no defined, reproducible jitter-specific metric exists in the current tooling. This is reported as a limitation, not inferred as zero.

## Reordering and duplication

`out_of_order_count` (mean across 5 trials): `37.2` (epuck1→epuck2), `34.8` (epuck2→epuck1) -- present in every trial, as expected for this jitter spread. `duplicate_count = 0` in both directions, every trial.

## Stale-state events

**10 total `SAFE_STOP_STALE`→recovery episodes across the batch (2 per trial, one per robot)**, all occurring within the first ~30 seconds of each 60-second trial and each recovering in well under 1 second. Per the accepted reporting limitation, these are reported as **startup diagnostic events**, not as in-task (during-navigation-encounter) events -- this has not been independently proven either way with the current tooling. `in_task_stale_episode_count` is `NOT_AVAILABLE` for every trial. No literal `PEER_TIMEOUT`-labeled event was found in any trial's controller log; the own-fresh-vs-peer-fresh breakdown remains `NOT_AVAILABLE` (a known, pre-existing gap in the analyzer, not computed for any A–G condition to date).

## Evidence-chain health (all 5 trials, uniformly)

`queue_drained=true`, `pending_queue_depth=0` (both directions, every trial), bag `metadata.yaml` present and non-empty, `bag_record.log` clean (no drop/warn/error lines), `analyzer_ok=true` (exit 0, schema-valid output), `realtime_factor_ok=true` (preload and full-load factors within 0.8-1.2 in every trial), no controller or simulation failure in any trial.

## SHA-256 evidence verification

15 tracked analysis-directory files (`frozen_params.json`, `trial_verdict.json`, `matrix_analysis.json` × 5 trials) hashed and recorded in `objective5_condition_G_formal_batch_summary.json`'s `evidence.analysis_directory_sha256` block (15/15 verified, 0 mismatches).

**Native-to-Windows raw evidence: 70/70 files SHA-256 matched.** All five trials' native WSL bag directories (`metadata.yaml` + `.db3`) and diagnostic-log directories (12 files/trial) were copied to `experiments/05_objective5_impairment_matrix/bags/` (gitignored, matching the E/F convention) and independently verified: source count 70, destination count 70, 0 missing, 0 extra, 0 size mismatches, 70/70 SHA-256 matches. Recorded in [objective5_condition_G_raw_evidence_sha256.csv](objective5_condition_G_raw_evidence_sha256.csv), using the same five-column schema as the Condition F CSV (`trial_index,relative_path,source_sha256,windows_sha256,match`).

## Limitations

- This is an **n=5 descriptive simulation result** under one fixed combined-impairment configuration. It carries no statistical-significance claim and no broad-generalisation claim.
- **Trial 02's `UNSAFE_FAILURE` is a real, valid safety finding**, not excluded, retried, or reclassified -- a 0.33 mm safety-radius violation under this combined delay+jitter+loss configuration.
- Realised jitter is not separately available; only message-age distribution statistics are reported.
- Startup-vs-in-task stale-event timing is not independently distinguished by current tooling.
- Simulation evidence does not replace dual-physical-robot validation.

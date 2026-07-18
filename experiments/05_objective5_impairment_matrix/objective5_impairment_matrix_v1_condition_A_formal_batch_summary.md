# Condition A formal batch summary (FORMAL_SIM, n=5)

**5/5 PASS.** Zero-impairment simulation baseline for the Objective 5
impairment matrix (Conditions A-G). Trial 01 was launched manually by
the user and individually observed; Trials 02-05 ran automatically
under explicit user authorization, each gated by the same strict
per-trial criteria, with no per-trial manual observation.

## Pass rate

| trial | verdict | DATA_VALIDITY | TASK_OUTCOME | min_interrobot_distance_m | safety_margin_m |
|---|---|---|---|---|---|
| 01 | PASS | VALID | SUCCESS | 0.1430942842844398 | 0.0030942842844398 |
| 02 | PASS | VALID | SUCCESS | 0.15064050840214138 | 0.010640508402141369 |
| 03 | PASS | VALID | SUCCESS | 0.14781142476139542 | 0.007811424761395402 |
| 04 | PASS | VALID | SUCCESS | 0.15030077874985828 | 0.01030077874985827 |
| 05 | PASS | VALID | SUCCESS | 0.14178534915907265 | 0.0017853491590726356 |

**5/5 = 100% pass rate.**

## Code identity (frozen across all 5 trials)

- orchestrator SHA-256: `20d2ef63a152a7d65632e4fd3414c9cd1cdaa2a449f58daf7eac1bd28110913b`
- network_impairment_relay.py SHA-256: `f5d408bc3379f79fa70628370b4dfb6d537c4d03a1968fe8dc75a691c3e6d5ff`
- network_impairment.py SHA-256: `253e0d960e9b587a3c5e60587ce7ac56c167fd6aba1c98f8b7b940e821210561`
- sequence_counter.py SHA-256: `57bb0699a444df644d75c4e834b5fd13b5f15a6283d7b1d276ec0b65674f1fd3`
- **All four identical across all 5 trials** (verified before each trial
  started). `git_commit` differs between Trial 01
  (`2837d08609292dbfdab10b93fd68c610da25357f`) and Trials 02-05
  (`48824897885d854e280da40b610a0d5ce67e9162`) -- this reflects
  docs/registry-only commits made between the two runs (Trial 01's own
  closeout + the manual-observation correction), never a behavioral
  code change; the four SHA-256 values above are what actually gate
  comparability and are unchanged.
- Frozen config: `delay_s=jitter_s=drop_probability=0.0`,
  `outage_period_s=outage_duration_s=outage_phase_s=0.0` (all 5
  trials).

## Per-direction communication metrics (mean / stdev / min / max across n=5)

### epuck1&rarr;epuck2

| metric | mean | stdev | min | max |
|---|---|---|---|---|
| message count | 428.8 | -- | 415 | 443 |
| capture_ratio | 1.0 | 0.0 | 1.0 | 1.0 |
| mean message age (s) | 1.867e-05 | 2.288e-05 | 0.0 | 4.819e-05 |
| median message age (s) | 0.0 | 0.0 | 0.0 | 0.0 |
| p95 message age (s) | 0.0 | 0.0 | 0.0 | 0.0 |
| p99 message age (s) | 0.0 | 0.0 | 0.0 | 0.0 |
| max message age (s) | 0.008 | 0.0098 | 0.0 | 0.02 |
| throughput (bytes/s) | 708.26 | 1.62 | 706.15 | 710.14 |

### epuck2&rarr;epuck1

| metric | mean | stdev | min | max |
|---|---|---|---|---|
| message count | 444.0 | -- | 434 | 461 |
| capture_ratio | 1.0 | 0.0 | 1.0 | 1.0 |
| mean message age (s) | 4.592e-05 | 7.139e-05 | 0.0 | 1.843e-04 |
| median message age (s) | 0.0 | 0.0 | 0.0 | 0.0 |
| p95 message age (s) | 0.0 | 0.0 | 0.0 | 0.0 |
| p99 message age (s) | 0.0 | 0.0 | 0.0 | 0.0 |
| max message age (s) | 0.02 | 0.031 | 0.0 | 0.08 |
| throughput (bytes/s) | 708.38 | 1.55 | 706.23 | 710.25 |

`p99_message_age_s` is a genuine finite value on both directions in
every trial (never null) -- the strict formal-trial schema gate held
for all 5 runs.

## Realtime factor (mean / stdev / min / max across n=5)

| | mean | stdev | min | max |
|---|---|---|---|---|
| preload_realtime_factor | 0.986 | 0.0346 | 0.952 | 1.041 |
| full_load_realtime_factor | 0.975 | 0.0189 | 0.945 | 0.999 |

All 20 individual readings (2 per trial x 5 trials) within the
0.8-1.2 tolerance band.

## Safety

`safety_radius_m=0.14` (fixed threshold, unchanged across the whole
project).

| | mean | stdev | min | max |
|---|---|---|---|---|
| min_interrobot_distance_m | 0.146726 | 0.003657 | 0.141785 | 0.150641 |
| safety_margin_m | 0.006726 | 0.003657 | **0.001785** | 0.010641 |

**5/5 PASS, but the margin varies meaningfully across trials and the
tightest (Trial 05, ~1.79mm) is markedly smaller than the batch mean
(~6.73mm).** This is retained explicitly, per instruction, and is not
grounds to alter the frozen geometry or controller -- it is reported as
an observed fact of this specific n=5 sample. Any future reader
comparing Condition A against B-G on safety margin should be aware the
batch-level minimum is close to the threshold, not comfortably above it.

## Task completion

`complete_count=2` in all 5 trials (both robots reached goal, no
timeout, no safety stop). Bag duration: mean 53.72s, stdev 1.44s, min
51.72s, max 55.51s.

## Latency methodology (frozen, applies identically to every condition A-G)

Condition A is the **zero-impairment simulation baseline**
(`delay_s=jitter_s=drop_probability=0`, outage disabled). All five
trials' `message_age_s` samples are 0 or near-0 seconds
(`RESOLUTION_LIMITED` classification; unified formula:
`message_age_s = consumer_callback_ros_time - EpuckState.production_stamp`,
both endpoints under `use_sim_time=true` reading the one shared
`/clock` topic). **This 0/near-0-second figure reflects simulation-clock
resolution** -- messages are forwarded and consumed within the same or
an adjacent simulation tick when no impairment is configured -- **and
does NOT represent, and must never be cited as, real physical network
transport delay.** It is the correct zero-impairment reference point
against which Conditions B-G's actual configured delay/jitter will be
compared using the identical formula and measurement path.

## Evidence

Each trial has its own
`objective5_impairment_matrix_v1_condition_A_trialNN_attempt01_analysis/`
directory: `frozen_params.json`, `matrix_analysis.json`,
`trial_verdict.json` (written by the orchestrator), `final_verdict.json`,
`runtime_manifest.json`, `summary.md`, `README.md`, `SHA256SUMS`
(written after each run). Raw bags/diag logs are NOT committed --
preserved at native WSL path `/home/eamon/epuck_comm_bags/` and a
SHA-256-verified Windows copy under
`experiments/05_objective5_impairment_matrix/bags/` (gitignored).

## Batch status

**Condition A formal n=5 batch: COMPLETE, 5/5 PASS.** Conditions B-G
have not started and are not auto-started by this batch's completion.

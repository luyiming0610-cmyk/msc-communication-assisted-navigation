# Condition B formal batch summary (FORMAL_SIM, n=5)

**5/5 PASS.** Fixed 0.20s relay delay (`jitter_s=0.0`,
`drop_probability=0.0`, outage disabled), world/controller/thresholds/
initial poses identical to Condition A, local ToF/IR safety layer
enabled throughout. Trial 01 launched manually and individually
observed by the user (final PASS confirmed after the startup-sync
audit); Trials 02-05 ran automatically under explicit user
authorization, behavioral-code SHA-256 verified identical to B01
before each run.

## Verdict distribution

DATA_VALIDITY: 5/5 VALID. TASK_OUTCOME: 5/5 SUCCESS. `complete_count=2`
in every trial. All 5 automated strict-criteria PASS (`analyzer_ok=true`,
`legacy_replay=false`, p99 finite both directions in every trial).

| trial | verdict | min_interrobot_distance_m | safety_margin_m | TIMEBASE_INIT delta (s) | startup leader |
|---|---|---|---|---|---|
| 01 | PASS | 0.14777153762172363 | 0.007772 | &minus;2.64 | epuck1 |
| 02 | PASS | 0.14483920109118267 | 0.004839 | &minus;2.38 | epuck1 |
| 03 | PASS | 0.14640502178021450 | 0.006405 | &minus;0.02 | epuck1 |
| 04 | PASS | 0.14236896629648180 | 0.002369 | &minus;0.34 | epuck1 |
| 05 | PASS | 0.14189365289902410 | **0.001894** | 0.00 | tie |

## Code identity (frozen, identical across all 5 trials)

- orchestrator: `20d2ef63a152a7d65632e4fd3414c9cd1cdaa2a449f58daf7eac1bd28110913b`
- network_impairment_relay.py: `f5d408bc3379f79fa70628370b4dfb6d537c4d03a1968fe8dc75a691c3e6d5ff`
- network_impairment.py: `253e0d960e9b587a3c5e60587ce7ac56c167fd6aba1c98f8b7b940e821210561`
- sequence_counter.py: `57bb0699a444df644d75c4e834b5fd13b5f15a6283d7b1d276ec0b65674f1fd3`
- Identical to the Condition A frozen values. `git_commit` varies only
  by docs-only commits (B01 at `3f30f70`, B02-05 at `14a0650`) --
  never a behavioral change.

## Latency vs configured 0.20s delay (mean/stdev/min/max across n=5)

| direction | mean age (s) | median (s) | p95 (s) | p99 (s) | max (s) |
|---|---|---|---|---|---|
| epuck1&rarr;epuck2 | 0.20901 (sd 0.00042) | 0.200 exactly, all 5 | 0.22 | 0.22 | 0.22 |
| epuck2&rarr;epuck1 | 0.20930 (sd 0.00035) | 0.200 exactly, all 5 | 0.22 | 0.22 | 0.22 |

Median = configured delay exactly in every trial; mean sits ~9ms above
(publish-period quantization); p95/p99/max land one ~0.02s sim-tick
above. All 10 direction-measurements:
`VALID_AT_SIM_CLOCK_RESOLUTION`. Capture ratio **1.0** and **0 drops**
in every trial, both directions. Throughput ~707-710 bytes/s both
directions (consistent with Condition A -- delay shifts arrival time,
not rate).

## Trigger classification

`first_trigger_reason=PREDICTED_CPA` in all 5 trials; every `LOCAL_*`
counter 0 in all 5 trials -- **PURE_COMMUNICATION_CPA_AVOIDANCE under
0.20s fixed delay, 5/5.** The local ToF/IR layer stayed enabled and
never engaged.

## Safety

Margin mean 4.66mm, stdev 2.27mm, min **1.89mm (Trial 05)**, max
7.77mm -- all positive, all PASS, tightest recorded explicitly.
Compared with Condition A (mean 6.73mm, min 1.79mm): B's mean margin
is descriptively smaller, but the two ranges overlap heavily and n=5
per condition supports no significance claim.

## Startup-delta covariate (recorded, not filtered)

TIMEBASE_INIT deltas (epuck1&minus;epuck2): [&minus;2.64, &minus;2.38,
&minus;0.02, &minus;0.34, 0.00] -- |delta| mean 1.08s, range
0.00-2.64s. In this batch epuck1 led (or tied) in all 5 trials; in
Condition A the direction flipped (A04: epuck2 led by 2.66s), so the
direction remains not fixed overall. **AVOID_TURN and RECOVER deltas
are 0.000s in all 5 trials** -- the avoidance phase stayed synchronized
regardless of startup offset, same as Condition A.

### Relationship check (descriptive only -- n=5, no significance claimed)

- |TIMEBASE_INIT delta| vs min_interrobot_distance: Pearson r = 0.59,
  but the pairing is **non-monotonic** (the two smallest deltas bracket
  both the smallest and second-largest distances) -- no consistent
  pattern discernible at n=5.
- |TIMEBASE_INIT delta| vs bag duration: Pearson r = 0.84 --
  descriptively, larger startup offsets co-occur with longer total
  recordings, which is mechanically expected (the encounter starts when
  the later robot is ready, so a bigger offset stretches the recording)
  rather than evidence of any communication effect.

No trial was excluded or adjusted on the basis of these covariates.

## Realtime factor / duration

Preload 0.955-0.984, full-load 0.945-0.999 across the 5 trials (all in
band). Bag duration mean 54.16s (52.38-55.70s).

## Condition F precondition (carried forward)

The startup offset (up to ~2.66s, ~3.8x Condition F's 0.7s outage
duration) is a recorded confound that MUST be checked against actual
outage-window timing before formal Condition F begins -- see
`project_status.json`'s dedicated blocked-item entry.

## Batch status

**Condition B formal n=5 batch: COMPLETE, 5/5 PASS.** Conditions C-G
have not started and are not auto-started by this batch's completion.

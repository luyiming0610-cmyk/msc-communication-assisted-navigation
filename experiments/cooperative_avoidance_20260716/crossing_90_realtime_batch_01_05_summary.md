# Ninety-degree crossing controlled-realtime CPA-only batch, Trials 01–05

Date: 2026-07-16

## Purpose and geometry

This batch tests whether the unchanged communication/CPA controller generalizes
from opposing paths to a perpendicular crossing encounter.

- `epuck1`: `(-0.35, 0.0)`, heading `0` rad, moving east.
- `epuck2`: `(0.0, -0.35)`, heading `pi/2` rad, moving north.
- Equal nominal speeds make the unavoided paths intersect at the arena centre at
  approximately the same time.
- Both robots apply the unchanged deterministic pass-right rule.
- Periodic state communication; local avoidance disabled.
- `max_runtime_s=60.0`, `stop_after_recovery=true`,
  `post_recovery_hold_s=0.5`.
- Pre-load and full-load realtime gates: 0.8–1.2.
- Trial 01 was directly observed; Trials 02–05 used the frozen scripted protocol
  in fresh WSL/Webots/ROS sessions.

## Per-trial results

| Trial | Pre-load factor | Full-load factor | Bag state-time factor | Minimum centre separation (m) | Safety margin (m) | Avoidance-onset skew (ms) | Collision |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 01 | display 0.92–1.08 | display 0.92–1.08 | 0.955955 | 0.143109 | 0.073109 | 3.0340 | No |
| 02 | 1.005 | 1.045 | 0.961087 | 0.140975 | 0.070975 | 0.8718 | No |
| 03 | 1.054 | 0.979 | 0.964070 | 0.140010 | 0.070010 | 0.6680 | No |
| 04 | 0.953 | 0.990 | 0.966343 | 0.141087 | 0.071087 | 0.0667 | No |
| 05 | 0.958 | 0.990 | 0.963654 | 0.143296 | 0.073296 | 0.0117 | No |

## Batch statistics

| Metric | Mean ± sample SD | Median | Range |
|---|---:|---:|---:|
| Minimum centre separation (m) | 0.141695 ± 0.001439 | 0.141087 | 0.140010–0.143296 |
| Safety margin above 0.070 m (m) | 0.071695 ± 0.001439 | 0.071087 | 0.070010–0.073296 |
| Bag-derived state time factor | 0.962222 ± 0.003969 | 0.963654 | 0.955955–0.966343 |
| Avoidance-onset skew (ms) | 0.9304 ± 1.2336 | 0.6680 | 0.0117–3.0340 |
| Last-motion-command skew (ms) | 25.186 ± 55.793 | 0.0208 | 0.0039–124.990 |

## Outcome

- Accepted repetitions: `5/5`.
- Collision: `0/5`.
- Invalid communicated state messages: `0` in all runs.
- Both robots completed recovery and commanded zero in every run.
- Each robot had one significant angular-command sign change per run; no
  repeated-turn oscillation was detected.
- Direct observation in Trial 01 confirmed that both robots passed toward their
  own right, recovered, did not spin and did not contact a wall.

Trial 03 had a 0.125 s final-command timing difference, but avoidance onset differed
by only 0.668 ms and both robots completed normally. The difference does not affect
collision status, minimum separation or completion classification.

## Controlled geometry comparison

| Scenario | Mean minimum centre separation (m) | Mean safety margin (m) | Success | Collision |
|---|---:|---:|---:|---:|
| Centred head-on | 0.147039 | 0.077039 | 5/5 | 0/5 |
| Head-on, 0.040 m offset | 0.182786 | 0.112786 | 5/5 | 0/5 |
| Ninety-degree crossing | 0.141695 | 0.071695 | 5/5 | 0/5 |

The crossing mean separation was 0.005344 m lower than the centred head-on mean,
but it retained a positive mean safety margin of 0.071695 m. All three controlled
geometries used closely matched realtime factors and the same controller settings.

## Evidence and limitation

- Bags: `bags/crossing_90_realtime_formal_trial_01/` through `05/`.
- Per-run metrics: each bag's `analysis/summary.json`.
- Controller and execution logs: corresponding files under `logs/`.
- Frozen world and protocol: `config/crossing_90/`.
- Machine-readable table: `crossing_90_realtime_batch_01_05.csv`.

Collision is inferred from centre separation and a 0.070 m two-robot diameter.
Supervisor contact-event ground truth remains planned for the final primary
cross-method statistical batches.

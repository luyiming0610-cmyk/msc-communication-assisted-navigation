# Locked Central-Obstacle Batch Summary — Trials 03–07

Date: 2026-07-16

## Scope and inclusion rule

This pilot batch contains five trials performed with identical initial poses,
obstacle geometry, periodic state publication, controller parameters and a 14 s
runtime. Trial 01 is retained as a pre-fix diagnostic failure. Trial 02 is retained
as functional evidence but excluded from this batch because its longer window
contains a later arena-boundary avoidance event.

## Functional outcomes

- Successful obstacle passage: 5/5 (100%).
- Visual collisions: 0/5.
- Analyzer geometric collisions: 0/5.
- Obvious oscillation: 0/5.
- Correct right-side avoidance: 5/5.
- Stationary-peer integrity: 5/5; displacement 0 m in every run.
- Final zero command: 5/5.
- Runs with invalid robot-state messages: 0/5.
- Angular sign changes: 1 in every run.

The Wilson 95% confidence interval for a 5/5 success proportion is approximately
56.6%–100%. The wide interval demonstrates that this is a successful pilot batch,
not a precise final estimate of reliability.

## Per-trial results

| Trial | Path (m) | Progress (m) | Efficiency | Max lateral (m) | Min front (m) | Min clearance (m) | Motion time (s) |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 03 | 0.759633 | 0.540660 | 0.711739 | 0.185268 | 0.172626 | 0.120235 | 11.975564 |
| 04 | 0.814779 | 0.598130 | 0.734100 | 0.185119 | 0.175209 | 0.120091 | 11.983046 |
| 05 | 0.743559 | 0.513771 | 0.690962 | 0.187553 | 0.169308 | 0.122510 | 11.956887 |
| 06 | 0.774210 | 0.550873 | 0.711530 | 0.187775 | 0.176315 | 0.122733 | 11.611504 |
| 07 | 0.801345 | 0.592686 | 0.739614 | 0.185573 | 0.169214 | 0.120547 | 11.981920 |

## Descriptive statistics

Values are `mean ± sample SD`; the final columns show median and range.

| Metric | Mean ± SD | Median | Min–max |
|---|---:|---:|---:|
| Path length (m) | 0.778705 ± 0.029296 | 0.774210 | 0.743559–0.814779 |
| Forward progress (m) | 0.559224 ± 0.035755 | 0.550873 | 0.513771–0.598130 |
| Path efficiency | 0.717589 ± 0.019607 | 0.711739 | 0.690962–0.739614 |
| Maximum lateral deviation (m) | 0.186258 ± 0.001297 | 0.185573 | 0.185119–0.187775 |
| Minimum front range (m) | 0.172535 ± 0.003275 | 0.172626 | 0.169214–0.176315 |
| Minimum surface clearance (m) | 0.121223 ± 0.001289 | 0.120547 | 0.120091–0.122733 |
| Motion duration (s) | 11.901784 ± 0.162609 | 11.975564 | 11.611504–11.983046 |

## Interpretation

The locked controller completed all five runs without collision, oscillation,
invalid state or stationary-peer motion. Lateral deviation and odometry-derived
clearance each have a sample SD near 1.3 mm, indicating repeatable side-passing
geometry under this simulation condition. Forward stopping position varies more:
the progress range is 0.084359 m. This longitudinal variation is retained as a
result rather than removed as an outlier.

The phase gate is passed. The next pilot phase is moving two-robot communication
and CPA avoidance without a wooden box. The wooden box and moving peer should be
combined only after the communication-only and local-only baselines are measured.

## Limitations

- `n=5` is a pilot repeatability batch, not a final reliability estimate.
- Geometric metrics use transformed wheel odometry rather than independent Webots
  Supervisor ground truth.
- Visual collision labels were recorded by the user; explicit contact-event
  logging is not yet present.
- Bag idle durations differ, although motion-window trajectory metrics are not
  changed by stationary pre-roll or post-roll.

## Gate status

**PASS — Phase 1 locked-condition pilot complete; proceed to Phase 2.**

# Centred head-on controlled-realtime CPA-only batch, Trials 01–05

Date: 2026-07-16

## Purpose

Establish a controlled-realtime centred head-on baseline that is directly
comparable with the accepted 0.040 m lateral-offset batch. The earlier centred
Trials 02–06 remain functional evidence but are not used for strict timing
comparison because their simulation/recorded-time factors varied from 2.179 to
4.097.

## Locked condition

- `epuck1` origin: `(-0.35, 0.0, 0.0)`.
- `epuck2` origin: `(0.35, 0.0, pi)`.
- Periodic communicated state; local avoidance disabled.
- Controller parameters unchanged from the accepted CPA-only condition.
- `max_runtime_s=60.0`, `stop_after_recovery=true`,
  `post_recovery_hold_s=0.5`.
- Pre-load and full-load realtime gates: 0.8–1.2.
- Trial 01 was directly observed; Trials 02–05 used the same frozen scripted
  protocol, with a fresh WSL/Webots/ROS session for every repetition.

## Acceptance gates

Each accepted run required two `cooperative recovery completed` messages, a
complete rosbag, no invalid communicated state messages, no detected collision,
positive geometric safety margin and a measured time factor inside 0.8–1.2.

## Per-trial results

| Trial | Pre-load factor | Full-load factor | Bag state-time factor | Minimum centre separation (m) | Safety margin (m) | Avoidance-onset skew (ms) | Collision |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 01 | display 0.95–1.03 | display 0.95–1.03 | 0.957343 | 0.148258 | 0.078258 | 0.0029 | No |
| 02 | 0.955 | 0.947 | 0.958546 | 0.147215 | 0.077215 | 0.5595 | No |
| 03 | 0.961 | 0.958 | 0.955134 | 0.144697 | 0.074697 | 1.2943 | No |
| 04 | 0.959 | 0.991 | 0.962711 | 0.147256 | 0.077256 | 0.0026 | No |
| 05 | 1.032 | 1.003 | 0.966062 | 0.147769 | 0.077769 | 0.0126 | No |

## Batch statistics

| Metric | Mean ± sample SD | Median | Range |
|---|---:|---:|---:|
| Minimum centre separation (m) | 0.147039 ± 0.001377 | 0.147256 | 0.144697–0.148258 |
| Safety margin above 0.070 m (m) | 0.077039 ± 0.001377 | 0.077256 | 0.074697–0.078258 |
| Bag-derived state time factor | 0.959959 ± 0.004386 | 0.958546 | 0.955134–0.966062 |
| Avoidance-onset skew (ms) | 0.3744 ± 0.5674 | 0.0126 | 0.0026–1.2943 |
| Last-motion-command skew (s) | 0.07213 ± 0.06846 | 0.03816 | 0.000003–0.16873 |

## Outcome

- Accepted repetitions: `5/5`.
- Collision: `0/5`.
- Invalid state messages: `0` in all runs.
- Both robots reached recovery completion and commanded zero in all runs.
- Each robot had one significant angular-command sign change in every run; no
  repeated-turn oscillation was detected.

Trial 04 had a 0.340 s difference between the first non-zero motion commands, but
the actual avoidance-turn onset difference was only 0.0026 ms and the final
motion-command difference was 0.0030 ms. It is therefore a startup scheduling
effect, not evidence of asymmetric avoidance.

## Comparison with the lateral-offset controlled batch

The centred mean minimum separation was 0.147039 m. The 0.040 m offset batch mean
was 0.182786 m, an increase of 0.035747 m (approximately 24.3%). Both batches had
`5/5` success, `0/5` collision and closely matched realtime factors, so they are
suitable for a controlled geometry comparison. The larger offset clearance is
consistent with the initial lateral geometry; it does not imply a controller
parameter change.

## Evidence

- Bags: `bags/head_on_centered_realtime_formal_trial_01/` through `05/`.
- Bag-derived per-run metrics: each bag's `analysis/summary.json`.
- Controller and execution logs: corresponding files under `logs/`.
- Frozen protocol:
  `config/head_on_centered_realtime/run_centered_realtime_trial.sh`.
- Machine-readable batch table:
  `head_on_centered_realtime_batch_01_05.csv`.

Collision is inferred from centre separation and a 0.070 m two-robot diameter.
Supervisor contact-event ground truth remains a planned improvement for final
cross-scenario statistical batches.

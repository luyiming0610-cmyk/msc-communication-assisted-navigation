# Controlled-realtime 0.040 m Lateral-offset CPA-only Batch — Trials 01–05

Date: 2026-07-16

## Batch decision

**PASS — 5/5 successful, 0/5 robot-to-robot collisions.**

This is the primary internally controlled lateral-offset batch. It is separate
from the earlier accelerated-factor pilot and Trial 01 because those runs used
uncontrolled accelerated Webots time factors.

## Locked condition

- `epuck1 (-0.35, -0.02, 0)` and `epuck2 (0.35, +0.02, pi)`.
- Nominal no-avoidance path offset: 0.040 m.
- Clean arena with no wooden box.
- Periodic communicated state.
- Communication/CPA-only reciprocal pass-right control.
- Local obstacle avoidance disabled.
- Pre-load and full-load Webots time-factor gates: 0.8–1.2.
- A fresh WSL/Webots/ROS graph was used for every repetition.
- `max_runtime_s:=60.0` was a safety ceiling; every valid run ended earlier via
  `cooperative recovery completed`.
- Automatic stop after recovery with a 0.5 s hold.

## Aggregate results

| Metric | Mean ± sample SD | Median | Range |
|---|---:|---:|---:|
| Pre-load realtime factor | 0.9694 ± 0.0214 | 0.9630 | 0.9480–0.9980 |
| Full-load realtime factor | 0.9624 ± 0.0162 | 0.9610 | 0.9470–0.9890 |
| Minimum centre separation (m) | 0.182786 ± 0.005664 | 0.181677 | 0.178408–0.192299 |
| Safety margin above 0.070 m (m) | 0.112786 ± 0.005664 | 0.111677 | 0.108408–0.122299 |
| Final centre separation (m) | 0.272657 ± 0.001395 | 0.272338 | 0.270684–0.274159 |
| Avoidance-onset skew (ms) | 0.447110 ± 0.649501 | 0.171377 | 0.014801–1.559915 |
| Initial-turn duration difference (ms) | 31.519182 ± 27.818814 | 38.089013 | 1.571545–59.251859 |
| Last-motion-command skew (ms) | 24.869421 ± 33.389794 | 0.906986 | 0.003295–63.504508 |

Integrity checks:

- Success rate: 100% (5/5).
- Geometric collision rate: 0% (0/5).
- Invalid communicated state messages: 0 in every trial.
- Peak absolute linear command: 0.025 m/s for both robots in every trial.
- Peak clockwise angular command: 0.650 rad/s for both robots in every trial.
- Significant angular-command sign changes: 1 per robot in every trial.
- Every run reached recovery completion and commanded zero automatically.

## Interpretation

The 0.040 m offset intentionally breaks exact centre-line symmetry while retaining
a geometric collision path without avoidance. The controller consistently
increased clearance to at least 0.178408 m and synchronized avoidance onset to
within 1.560 ms. The narrow final-separation range supports repeatability under
the controlled realtime condition.

## Exclusions

- The accelerated-factor offset pilot and formal Trial 01 are retained as
  visual/functional evidence but not pooled because their measured time factors
  were 4.226 and 2.562 respectively.
- Four protocol-development diagnostic runs are excluded: outer-process
  interruption, 30 s safety-ceiling expiry, incomplete realtime control, and
  same-session ROS-graph contamination. None is silently deleted.

## Evidence

- Machine-readable table: `head_on_offset_040_realtime_batch_01_05.csv`.
- Bags: `bags/head_on_offset_040_realtime_formal_trial_01/` through
  `bags/head_on_offset_040_realtime_formal_trial_05/`.
- Per-run bag-derived metrics: each bag's `analysis/summary.json`.
- Controller and execution logs: corresponding files under `logs/`.
- Frozen scripted protocol/configuration:
  `config/head_on_lateral_offset_040/run_offset_040_formal_batch.sh`.
- Timing-integrity audit: `simulation_rate_integrity_audit_20260716.md`.

## Limitation

Separation and collision remain odometry-derived rather than Webots Supervisor
ground truth/contact events. Add Supervisor trajectory and contact recording
before final thesis-level geometric claims.

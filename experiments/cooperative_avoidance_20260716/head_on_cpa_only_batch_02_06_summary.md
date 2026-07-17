# Clean Centred Head-on CPA-only Batch — Trials 02–06

Date: 2026-07-16

## Batch decision

**PASS — 5/5 successful, 0/5 robot-to-robot collisions.**

The post-fix clean centred head-on communication/CPA-only condition is complete.
All five formal repetitions used identical controller parameters and initial
poses. Diagnostic Trial 01 is excluded because it preceded the heading-crossing
and automatic-completion fixes.

## Locked condition

- `epuck1 (-0.35, 0, 0)`, `epuck2 (0.35, 0, pi)`.
- Clean arena, no wooden box.
- Periodic state communication.
- Local obstacle avoidance disabled.
- Reciprocal pass-right CPA control.
- Automatic stop after recovery with 0.5 s hold.

## Aggregate results

| Metric | Mean ± sample SD | Median | Range |
|---|---:|---:|---:|
| Minimum centre separation (m) | 0.151546 ± 0.011598 | 0.155619 | 0.131077–0.159625 |
| Safety margin above 0.070 m (m) | 0.081546 ± 0.011598 | 0.085619 | 0.061077–0.089625 |
| Final centre separation (m) | 0.360235 ± 0.014838 | 0.355917 | 0.345332–0.383661 |
| Avoidance-onset skew (ms) | 0.894539 ± 0.956241 | 0.530842 | 0.010193–2.448961 |
| Initial-turn duration difference (ms) | 28.014233 ± 48.823924 | 4.501654 | 0.537614–114.296612 |
| Last-motion-command skew (ms) | 24.636844 ± 36.352629 | 16.875219 | 0.015343–87.781898 |

Additional integrity checks:

- Success rate: 100% (5/5).
- Geometric collision rate: 0% (0/5).
- Invalid communicated state messages: 0 in every trial.
- Peak absolute linear command: 0.025 m/s in every trial.
- Significant angular-command sign changes: 3 per robot in every trial.
- All trials completed recovery and commanded zero automatically.
- No wall impacts or visible oscillation were reported.

## Timing interpretation

CPA avoidance itself was highly synchronized: the worst avoidance-onset skew was
only 2.449 ms. Trial 05 had a one-off 114.297 ms difference in the duration of the
initial clockwise turn. It remained collision-free and completed symmetrically,
and Trial 06 returned to a 1.428 ms duration difference. The event is therefore
retained as trial-to-trial closed-loop timing variability, not removed as an
outlier and not used to justify mid-batch parameter tuning.

## Gate decision

The centred head-on scenario passes the Phase 2 gate. The next controlled scenario
is a head-on encounter with a fixed lateral offset, using a separately frozen
world/configuration and the same controller parameters.

## Evidence

- Per-trial summaries: `head_on_cpa_only_trial_02_postfix_summary.md` through
  `head_on_cpa_only_trial_06_postfix_summary.md`.
- Machine-readable batch table: `head_on_cpa_only_batch_02_06.csv`.
- High-rate turn timing: `head_on_cpa_only_turn_timing_02_06.csv`.
- Bags and automated analysis: `bags/head_on_cpa_only_trial_02_postfix/` through
  `bags/head_on_cpa_only_trial_06_postfix/`.

## Limitations before final thesis claims

- Separation uses paired communicated odometry rather than Webots Supervisor
  ground truth.
- Collision is inferred from the 0.070 m geometric threshold; explicit Webots
  contact events are not yet recorded.
- Current automated summaries do not yet include full trajectory path length and
  lateral deviation; these should be added with Supervisor ground truth before
  the final statistical analysis.

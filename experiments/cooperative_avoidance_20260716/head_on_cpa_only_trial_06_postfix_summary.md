# CPA-only Head-on Trial 06 — Post-fix Formal Summary

Date: 2026-07-16

## Classification

**PASS — include as post-fix CPA-only formal repetition 5/5.**

## Experimental condition

- Clean Webots arena with no wooden box.
- Initial shared poses: `epuck1 (-0.35, 0, 0)` and
  `epuck2 (0.35, 0, pi)`.
- Periodic communicated `EpuckState` messages.
- Communication/CPA-only reciprocal pass-right control.
- Local obstacle avoidance disabled and local sensors not required.
- `stop_after_recovery:=true` with `post_recovery_hold_s:=0.5`.
- Maximum runtime safety limit: 30 s.
- Controller parameters unchanged throughout formal Trials 02–06.

## User observation

- All avoidance, collision, oscillation, recovery, automatic-stop and wall checks
  were normal.
- No visible difference in avoidance speed between the two robots occurred.

## High-rate command timing check

- Avoidance-command onset skew: 0.001116 s.
- `epuck1` initial clockwise-turn duration: 0.461737 s.
- `epuck2` initial clockwise-turn duration: 0.463165 s.
- Turn-duration difference: 0.001428 s.
- Clockwise peak angular command: 0.650000 rad/s for both robots.
- Controller completion timestamps differed by approximately 0.021596 s.

The high-rate command data agrees with the user's observation. The larger
turn-duration difference seen once in Trial 05 did not repeat.

## Automated rosbag results

- Bag duration: 48.762475 s, including pre-controller recording time.
- Valid paired state samples: 886.
- State samples: 443 for `epuck1` and 448 for `epuck2`.
- Invalid state messages: 0 for both robots.
- Minimum centre separation: 0.156850 m.
- Minimum geometric safety margin above the 0.070 m threshold: 0.086850 m.
- Robot-to-robot geometric collision detected: false.
- Final centre separation: 0.345332 m.
- Motion-start skew: 0.000011 s.
- Last-motion-command skew: 0.016875 s.
- Peak absolute linear command: 0.025 m/s for both robots.
- Significant angular-command sign changes: 3 per robot.

## Decision

All locked behavioural and automated acceptance checks passed. Trial 06 is formal
repetition 5/5 and completes the clean centred head-on CPA-only batch.

## Evidence

- Log: `logs/head_on_cpa_only_trial_06_postfix.log`
- Bag: `bags/head_on_cpa_only_trial_06_postfix/`
- Automated metrics:
  `bags/head_on_cpa_only_trial_06_postfix/analysis/summary.json`

## Limitation

Clearance and collision values use paired odometry rather than Webots Supervisor
ground truth/contact events. Supervisor evidence remains required before final
geometric claims.

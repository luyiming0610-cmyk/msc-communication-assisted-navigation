# CPA-only Head-on Trial 05 — Post-fix Formal Summary

Date: 2026-07-16

## Classification

**PASS with a recorded minor turn-timing asymmetry — include as formal repetition 4/5.**

## Experimental condition

- Clean Webots arena with no wooden box.
- Initial shared poses: `epuck1 (-0.35, 0, 0)` and
  `epuck2 (0.35, 0, pi)`.
- Periodic communicated `EpuckState` messages.
- Communication/CPA-only reciprocal pass-right control.
- Local obstacle avoidance disabled and local sensors not required.
- `stop_after_recovery:=true` with `post_recovery_hold_s:=0.5`.
- Maximum runtime safety limit: 30 s.
- Controller parameters unchanged from formal Trials 02–04.

## User observation

- Both robots avoided to their own right.
- No repeated rotation, collision, visible oscillation or wall impact occurred.
- Both robots stopped automatically and their final positions appeared symmetric.
- The user observed that one robot might have completed the initial avoidance turn
  slightly faster than the other.

## High-rate command timing check

The observation is supported by the bag, but the difference is in turn completion,
not CPA triggering:

- Avoidance-command onset skew: 0.000531 s.
- `epuck1` initial clockwise-turn duration: 0.469096 s.
- `epuck2` initial clockwise-turn duration: 0.354799 s.
- Turn-duration difference: 0.114297 s; `epuck2` entered the pass correction first.
- `epuck1` clockwise peak: 0.650000 rad/s.
- `epuck2` clockwise peak: 0.617625 rad/s.
- Final completion timestamps differed by approximately 0.016275 s.

For comparison, the initial-turn duration differences in formal Trials 02–04
were approximately 0.000538 s, 0.004502 s and 0.019308 s. Trial 05 therefore has
the largest turn-timing asymmetry so far. It did not affect collision clearance,
recovery, automatic stop or visible final-position symmetry.

## Automated rosbag results

- Bag duration: 37.089859 s, including pre-controller recording time.
- Valid paired state samples: 663.
- State samples: 337 for `epuck1` and 332 for `epuck2`.
- Invalid state messages: 0 for both robots.
- Minimum centre separation: 0.155619 m.
- Minimum geometric safety margin above the 0.070 m threshold: 0.085619 m.
- Robot-to-robot geometric collision detected: false.
- Final centre separation: 0.355917 m.
- Motion-start skew: 0.000010 s.
- Last-motion-command skew: 0.018176 s.
- Peak absolute linear command: 0.025 m/s for both robots.
- Significant angular-command sign changes: 3 per robot.

## Decision

All locked behavioural and safety acceptance checks passed. The timing difference
is retained as measured trial-to-trial variability and is not grounds for changing
controller parameters mid-batch. Trial 05 is formal repetition 4/5. Reassess the
distribution after Trial 06 completes the batch.

## Evidence

- Log: `logs/head_on_cpa_only_trial_05_postfix.log`
- Bag: `bags/head_on_cpa_only_trial_05_postfix/`
- Automated metrics:
  `bags/head_on_cpa_only_trial_05_postfix/analysis/summary.json`
- Cross-trial timing table: `head_on_cpa_only_turn_timing_02_05.csv`

## Limitation

Clearance and collision values use paired odometry rather than Webots Supervisor
ground truth/contact events. Supervisor evidence remains required before final
geometric claims.

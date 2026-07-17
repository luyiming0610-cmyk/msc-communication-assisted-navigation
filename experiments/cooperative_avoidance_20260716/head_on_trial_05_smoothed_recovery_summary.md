# Head-on Cooperative Avoidance — Trial 05 (Smoothed Recovery)

## Classification

- **Status:** Valid completed successful trial
- **Dataset role:** Include as the first formal nominal repetition of the
  smoothed communication-aware controller
- **Collision:** None
- **Recovery:** Complete

## Behavioural result

- Both robots executed the reciprocal pass-right manoeuvre.
- The controller followed the complete state sequence:
  `STARTUP_HOLD -> CRUISE -> AVOID_TURN -> AVOID_PASS -> RECOVER -> CRUISE`.
- Both robots returned smoothly to their original travel headings.
- No visible command stutter or repeated left-right oscillation occurred.
- Both robots issued zero commands and stopped at the runtime limit.
- No `LOCAL_EMERGENCY`, stale-state stop, or invalid-odometry stop occurred.

## Automated rosbag metrics

- Valid paired state samples: `1436`
- Invalid state messages: `0` for both robots
- Minimum centre separation: `0.148910 m`
- Minimum geometric safety margin: `0.078910 m`
- Collision detected: `false`
- Final centre separation: `0.594790 m`
- Motion-start skew: `0.095128 s`
- Last-motion-command skew: `0.090602 s`
- Peak absolute linear command: `0.025 m/s`
- Peak absolute angular command: `0.650 rad/s`
- Maximum measured angular slew:
  - e-puck1: `4.166 rad/s^2`
  - e-puck2: `4.036 rad/s^2`
- Significant angular sign changes: `1` per robot

## Information used by this trial

The robot-to-robot collision decision was produced by communicated typed state:
shared-frame position, yaw and linear velocity from `/epuck1/state` and
`/epuck2/state`. These values fed the closest-point-of-approach calculation.

Each robot's local front-distance field remained available as a higher-priority
emergency layer, but it did not trigger in this run. Therefore this trial is a
clean demonstration of communication-aware dynamic mutual avoidance rather than
a static-obstacle avoidance trial.

## Evidence

- Log: `logs/head_on_trial_05_smoothed_recovery.log`
- Bag: `bags/head_on_trial_05_smoothed_recovery/`
- Metrics: `bags/head_on_trial_05_smoothed_recovery/analysis/summary.json`

# Head-on Cooperative Avoidance — Trial 04 (Smoothed)

## Classification

- **Safety outcome:** Successful, no collision
- **Smoothness outcome:** Successful
- **Recovery outcome:** Incomplete because the configured runtime ended before
  the recovery transition
- **Dataset role:** Retain as a valid smoothing diagnostic; do not count it as a
  completed nominal-task repetition

## Observed behaviour

- Both robots executed the reciprocal pass-right avoidance manoeuvre.
- No visible command stutter or repeated left-right oscillation occurred.
- Both robots stopped safely at the runtime limit.
- The robots did not return to their original headings before stopping.

## Cause of incomplete recovery

The log shows that both controllers remained in `AVOID_PASS`. At the 35 s
runtime limit, the sampled centre separation was approximately 0.233–0.236 m,
just below the configured 0.240 m release distance. Consequently, the
`AVOID_PASS -> RECOVER` transition had not yet occurred. This is a trial-duration
issue, not a failure of the command smoother.

The next controlled trial should retain the same controller parameters and
increase `max_runtime_s` to 45 s. This isolates trial duration as the only
changed independent variable.

## Automated rosbag metrics

- Valid paired state samples: `1676`
- Invalid state messages: `0` for both robots
- Minimum centre separation: `0.143078 m`
- Minimum geometric safety margin: `0.073078 m`
- Collision detected: `false`
- Motion-start skew: `0.001806 s`
- Last-motion-command skew: `0.000868 s`
- Peak absolute angular command: `0.650 rad/s`
- Maximum measured angular slew:
  - e-puck1: `4.224 rad/s^2`
  - e-puck2: `4.010 rad/s^2`
- Significant angular sign changes:
  - e-puck1: `0`
  - e-puck2: `1`

## Comparison with Trial 03

- Trial 03 maximum angular slew was approximately 16.1–16.5 rad/s².
- Trial 04 reduced maximum angular slew by approximately 74%.
- The significant angular-command sign changes reduced from `1/2` to `0/1`.
- Minimum separation reduced from 0.1493 m to 0.1431 m because the turn command
  now ramps in rather than changing instantaneously.
- The remaining 73.1 mm geometric safety margin indicates that the smoother is
  still safe in this nominal head-on condition, but later network-impairment
  experiments must re-evaluate this margin.

## Evidence

- Log: `logs/head_on_trial_04_smoothed.log`
- Bag: `bags/head_on_trial_04_smoothed/`
- Metrics: `bags/head_on_trial_04_smoothed/analysis/summary.json`

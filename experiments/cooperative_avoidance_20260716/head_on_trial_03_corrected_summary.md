# Head-on Cooperative Avoidance — Trial 03 (Corrected)

## Classification

- **Status:** Valid successful trial
- **Dataset role:** Include in the formal cooperative-avoidance dataset
- **Scenario:** Symmetric head-on encounter in Webots
- **Controller:** Communication-aware reciprocal CPA avoidance with deterministic pass-right rule

## Experimental configuration

- Initial shared-frame poses:
  - e-puck1: `(-0.35 m, 0.0 m, 0 rad)`
  - e-puck2: `(0.35 m, 0.0 m, pi rad)`
- Initial centre-to-centre separation: `0.700 m`
- Cruise speed: `0.025 m/s`
- Avoidance turn rate: approximately `0.65 rad/s`
- Avoidance pass speed: approximately `0.012 m/s`
- Startup synchronization hold: `5 s`
- Trial duration: `48 s`

## Outcome

- Both robots left the startup hold at the same logged instant and entered `CRUISE` together.
- A single reciprocal `AVOID_TURN` event was triggered at an inter-robot distance of approximately `0.345 m`.
- Both robots followed the agreed pass-right behaviour from their own forward viewpoints.
- Automated rosbag analysis measured a minimum centre distance of
  `0.149315 m`, giving a `0.079315 m` safety margin above the `0.070 m`
  geometric collision threshold.
- No collision occurred.
- No repeated `AVOID_TURN` re-trigger and no obvious left-right oscillation were observed.
- Both robots entered `COMPLETE` at the same logged instant and issued zero velocity commands.

## Smoothness observation

After the robots cleared the encounter, a small hesitation/stutter was visible
while returning to the nominal heading. The controller log shows a discrete
transition from the avoidance-pass command to the faster cruise command,
followed by a decaying heading correction. Bag-derived command metrics found
one significant angular-command sign change for e-puck1 and two for e-puck2.
This is a short recovery correction rather than sustained oscillation: it did
not re-trigger the encounter state or create a renewed collision risk.

For the final experimental controller, add acceleration/angular-rate limiting or a short blended recovery phase, then quantify settling time and peak angular command. Preserve this trial as the pre-smoothing reference sample.

## Evidence files

- Log: `logs/head_on_trial_03_corrected.log`
- ROS 2 bag: `bags/head_on_trial_03_corrected/`
- Bag database: `bags/head_on_trial_03_corrected/head_on_trial_03_corrected_0.db3`
- Bag metadata: `bags/head_on_trial_03_corrected/metadata.yaml`
- Automated metrics: `bags/head_on_trial_03_corrected/analysis/summary.json`
- Separation series: `bags/head_on_trial_03_corrected/analysis/separation.csv`
- Command series: `bags/head_on_trial_03_corrected/analysis/commands.csv`

## Automated rosbag metrics

- Valid paired state samples: `1150`
- Minimum centre separation: `0.149315 m`
- Minimum geometric safety margin: `0.079315 m`
- Collision detected: `false`
- Minimum-separation time in the recording: `38.768 s`
- Motion-start skew: `0.0816 s`
- Last-motion-command skew: `0.000383 s`
- Peak absolute linear command: `0.025 m/s` for both robots
- Peak absolute angular command: `0.650 rad/s` for both robots
- Significant angular sign changes: e-puck1 `1`, e-puck2 `2`
- Invalid state messages: `2` per robot; these were transient and did not
  bypass the startup/freshness safety hold

## Validity decision

This run is suitable as a successful feasibility demonstration of synchronized, communication-aware two-robot collision avoidance. It is not by itself sufficient for the final quantitative evaluation; repeated trials and automated bag-derived metrics are still required.

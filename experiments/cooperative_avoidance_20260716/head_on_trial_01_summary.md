# Cooperative avoidance experiment: head_on_trial_01

Date: 2026-07-16  
Platform: Webots R2025a, ROS 2 Humble, two independently namespaced e-puck robots  
Communication mode: typed `EpuckState`, periodic target 10 Hz  
Controller: decentralized CPA prediction with deterministic pass-right rule

## Configuration

- Initial shared-frame poses: epuck1 `(-0.35, 0, 0)`, epuck2 `(0.35, 0, pi)`.
- Initial centre-to-centre separation: approximately `0.700 m`.
- Nominal speed: `0.025 m/s`.
- Avoidance speed: `0.012 m/s`.
- Avoidance turn rate: `0.650 rad/s`.
- Trigger distance: `0.340 m`.
- CPA safety radius: `0.140 m`.
- Armed runtime: `30 s`.

## Observed result

- Both robots moved towards each other and independently computed the same risk.
- Both transitioned from `CRUISE` to `AVOID_TURN` at approximately `0.349 m` separation.
- At trigger, the predicted closest separation was approximately `0.136 m`.
- Both applied the pass-right rule in their own body frame.
- Both transitioned to `AVOID_PASS` without a stale-state or local-emergency event.
- The minimum centre separation visible in the 0.5 s controller log was approximately `0.148 m`.
- Separation subsequently increased, demonstrating that the encounter had passed its closest point.
- No collision was observed.
- Both robots stopped completely at the runtime limit.

## Validity and limitation

This run is a valid successful collision-avoidance sample. It demonstrates
symmetrical state exchange, consistent decentralized risk classification, and
collision-free reciprocal action. The runtime ended while both controllers were
still in `AVOID_PASS`; therefore it is not yet a complete trajectory-recovery
sample. A follow-up run should use a longer runtime so that `RECOVER` and return
to the original headings can be observed. The `0.148 m` minimum is based on the
periodic text log; the rosbag should be used for the final exact minimum-distance
statistic.

## Evidence files

- `bags/head_on_trial_01/head_on_trial_01_0.db3`
- `bags/head_on_trial_01/metadata.yaml`
- `logs/head_on_trial_01.log`

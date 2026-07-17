# Local Static Obstacle Long-Course Trial 02 — Formal Summary

Date: 2026-07-16

## Experimental purpose

This trial evaluated the fused controller in a long-course Webots scenario. The
controller priority was: communication/sensor validity safety stop, local
IR/ToF obstacle avoidance, communicated neighbour-state CPA avoidance, and
nominal cruise. `epuck1` was the moving robot, while `epuck2` remained stationary
and continued publishing a valid neighbour state.

## Observed outcome

The run passed the functional acceptance criteria. `epuck1` detected the wooden
box, passed it on the robot's right-hand side, did not visibly oscillate, did not
collide, and stopped completely. `epuck2` remained stationary throughout.

The first wooden-box encounter followed the intended state sequence:

`CRUISE -> LOCAL_CLEARANCE -> LOCAL_BYPASS -> LOCAL_RECOVER -> CRUISE`

A later second avoidance sequence was caused by the robot continuing toward the
arena boundary after it had already passed the wooden box. It was not an
oscillation around the original obstacle.

## Static-obstacle metrics

- Valid `epuck1` state samples: 750.
- Invalid robot-state messages: 0.
- Path length over the full recorded motion: 1.756345 m.
- Final forward progress: 1.024600 m.
- Maximum forward progress: 1.115026 m.
- Whole-run path efficiency: 0.583371.
- Whole-run maximum absolute lateral deviation: 0.455299 m.
- Minimum measured front range: 0.166059 m.
- Minimum geometric robot-to-box surface clearance: 0.119530 m.
- Geometric collision detected: false.
- Wooden obstacle passed: true.
- Stationary peer displacement: 0.000000 m.
- Final velocity command zero: true.

The positive geometric surface clearance is the primary collision-safety result
for this trial. The minimum ToF/front range is not expected to equal the
geometric clearance because the ToF sensor measures only along its instantaneous
narrow field of view while the robot is turning.

## Communication and command metrics

- Valid neighbour-state samples: 752.
- `epuck1` command messages: 3377.
- Peak absolute linear command: 0.015 m/s.
- Peak absolute angular command: 0.450 rad/s.
- Maximum linear command step: 0.010 m/s.
- Maximum angular command step: 0.180 rad/s.
- Angular sign changes: 5.
- `epuck2` motion-command messages: 0.

The angular sign changes cover two distinct obstacle encounters and their
heading-recovery phases. They therefore must not be interpreted as five
oscillation events at the wooden box. Raw maximum slew estimates are also not
treated as physical acceleration measurements because they are sensitive to
irregular rosbag message intervals and immediate safety-stop transitions.

## Interpretation and limitation

Trial 02 is accepted as a successful functional validation of local-obstacle
avoidance while a valid communicated neighbour is present. It demonstrates full
obstacle passage, positive clearance, stationary-peer integrity, and fail-safe
termination.

The reported path efficiency and maximum lateral deviation describe the entire
recorded motion, including travel after the wooden box and the later arena-boundary
avoidance. They are not clean single-obstacle performance measures. A shorter or
event-windowed repeat is required before these two metrics are used in the final
thesis comparison. In addition, one successful run is not a statistical result;
repeated trials and ablation conditions are still required.

## Acceptance status

**PASS — functional validation only.**


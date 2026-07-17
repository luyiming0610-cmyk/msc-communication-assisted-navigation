# Local Static Obstacle Long-Course Trial 03 — Formal Summary

Date: 2026-07-16

## Experimental purpose

Trial 03 repeated the long-course static-obstacle fusion scenario from the same
initial conditions as Trial 02. The recording window was deliberately terminated
after the wooden-box approach, avoidance, passage and initial heading recovery,
before the later arena-boundary encounter that contaminated the whole-run metrics
in Trial 02. `epuck1` was the moving robot. `epuck2` remained stationary while
publishing a valid neighbour state.

## User observation

- `epuck1` passed the wooden box.
- The robot avoided toward its own right-hand side.
- No obvious oscillation was observed.
- No collision was observed.
- `epuck2` remained stationary.
- The final stop was complete.

## Controller evidence

The intended single-obstacle state sequence was recorded:

`STARTUP_HOLD -> CRUISE -> LOCAL_CLEARANCE -> LOCAL_BYPASS -> LOCAL_RECOVER -> CRUISE -> COMPLETE`

The controller reached its 14 s runtime limit and commanded zero velocity. No
second local-avoidance sequence occurred within the motion window.

## Static-obstacle metrics

- Valid `epuck1` state samples: 2505.
- Invalid robot-state messages: 0.
- Bag duration: 269.341319 s.
- Motion start in bag: 100.709047 s.
- Last non-zero motion command: 112.684611 s.
- Path length: 0.759633 m.
- Final and maximum forward progress: 0.540660 m.
- Path efficiency: 0.711739.
- Maximum absolute lateral deviation: 0.185268 m.
- Minimum measured front range: 0.172626 m.
- Minimum odometry-derived robot-to-box surface clearance: 0.120235 m.
- Geometric collision detected by the current analyzer: false.
- Wooden obstacle passed: true.
- `epuck2` displacement: 0.000000 m.
- Final velocity command zero: true.

## Command metrics

- `epuck1` command messages: 17440.
- Peak absolute linear command: 0.015 m/s.
- Peak absolute angular command: 0.450 rad/s.
- Maximum linear command step: 0.015 m/s.
- Maximum angular command step: 0.059858 rad/s.
- Angular sign changes: 1.

The large command-message count and total bag duration include long stationary
periods before controller start and after automatic completion. They must not be
used as communication-performance measurements for this trial.

## Interpretation and limitation

Trial 03 passed the functional acceptance criteria and provides an independent
repeat of the successful wooden-box avoidance observed in Trial 02. The shorter
motion window excludes the later arena-boundary avoidance and therefore gives
cleaner single-obstacle path-efficiency and lateral-deviation metrics.

The geometric clearance is derived from wheel odometry transformed into the
shared frame. No independent Webots Supervisor ground-truth pose was recorded.
Consequently, the positive clearance agrees with the visual no-collision
observation but is not an independent ground-truth measurement. Future repeated
and thesis-grade trials should record Supervisor ground-truth poses and contact
events, and should limit pre-run and post-run idle recording.

Two successful trials remain functional evidence, not a statistical result.
Additional repeated conditions and ablations are still required.

## Acceptance status

**PASS — repeated functional validation; not yet a statistical result.**

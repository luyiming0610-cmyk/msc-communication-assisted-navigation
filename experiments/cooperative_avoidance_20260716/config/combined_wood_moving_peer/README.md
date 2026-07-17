# Canonical wooden-box plus moving-peer fusion scenario

Date frozen: 2026-07-16

## Purpose

Test the controller priority chain in one run: peer-state CPA avoidance begins
first, then `epuck1` encounters a local wooden obstacle while recovering from the
completed pass. The obstacle is outside `epuck2`'s mirrored pass path.

## Locked geometry

- `epuck1`: `(-0.35, 0.0, 0.0)`.
- `epuck2`: `(0.35, 0.0, pi)`.
- Wooden box centre: `(0.50, -0.115)`, size `0.06 m` square.
- Clean 1.5 m arena; no other boxes.

The box location was selected from the accepted fused-baseline trajectory and
excluded protocol-development pilots. It lies on `epuck1`'s post-encounter
straight path, after the moving-peer event is complete, and behind the initial
heading of `epuck2`. This produces two unambiguous events in one run without
placing the box in `epuck2`'s path. No controller threshold or speed is changed.

## Expected sequence

1. Both robots cruise toward the predicted encounter.
2. Both enter `AVOID_TURN` from peer-state CPA and pass right.
3. After the peer passes and cooperative recovery finishes, `epuck1` continues
   forward, detects the box head-on and enters a `LOCAL_*` mode.
4. `epuck2` completes cooperative recovery and stops. `epuck1` keeps the same
   avoidance controller active until a separate task monitor confirms that the
   entire robot has crossed the box's east face and recovered its heading; the
   launch then stops both controllers and commands zero.

## Acceptance gates

- Realtime factors in `0.8–1.2` and a fresh WSL/Webots/ROS session.
- Both robots confirm fused inputs and enter CPA avoidance.
- `epuck1` confirms local-obstacle activation after CPA onset.
- No robot-to-robot or robot-to-box collision and no repeated oscillation.
- Odometry-derived `epuck1` box clearance is at least `0.005 m`.
- `epuck2` recovers and stops normally; `epuck1` crosses `x=0.575 m`, recovers
  heading within 0.10 rad and is stopped by the task monitor before its 90 s
  safety ceiling.
- Complete rosbag with state, odometry, commands and both robots' local sensors.

The first run is directly observed. Only after it passes will the geometry be
used for the four remaining formal repetitions.

The original observed Trial 01 used `stop_after_recovery=true` for both robots.
It stopped `epuck1` safely at approximately `(0.190, -0.179)` with positive box
clearance, but before the robot crossed the box. That run is retained as an
excluded task-completion diagnostic. The task monitor changes only the stopping
criterion; avoidance speeds, thresholds and priority logic remain unchanged.

An excluded task-monitor pilot at box centre `(0.28, -0.14)` completed the pass
but left only `0.000052 m` odometry-derived box clearance. A second excluded
position `(0.28, -0.11)` failed the geometric collision gate. The final locked
position `(0.50, -0.115)` separates the CPA and local events in time and retains
the formal `0.005 m` clearance gate.

# Suspended-wheel angular-rate diagnostic test -- epuck5809 (2026-07-23)

**Classification: `DIAGNOSTIC_PHYSICAL`, stationary/suspended-wheel only.
Not a ground-motion trial, not an `EXCLUSIONARY_HIL_PILOT`, not part of
any formal batch.** Robot on stand, wheels clear of the ground
throughout. User present at the emergency stop for the entire test.
Frozen git commit at test time: `004c3650141a3c3782ef1f080968d7c76e265209`.

## Precondition confirmations (verbatim, from the user, same session)

```
ROBOT_ON_STAND=YES
WHEELS_CLEAR_OF_GROUND=YES
USER_AT_EMERGENCY_STOP=YES
TEST_AREA_CLEAR=YES
```

## Preconditions reconfirmed before this test

- `validity_flags=7` (re-read live from `/epuck1/state`).
- `/cmd_vel Publisher count: 0` (before the guard started).

## Test-scoped safety parameter

`hil_frozen_params.json`'s `hil_guard_limits.max_angular_speed_rps` was
set to **0.1 rad/s** by explicit user decision, scoped to this test
only. This is **not** a measured operating limit for epuck5809 ground
motion and has been reset to `UNCONFIRMED_PHYSICAL_MEASUREMENT`
immediately after this test (see below). `max_linear_speed_mps`
remained at its frozen value, 0.02, and linear velocity was never
requested (no CLI flag for it exists in the test publisher used here).

## New tooling used

`hil_angular_suspension_test.py` -- a new, minimal, symmetric
counterpart to `hil_wheel_suspension_test.py`: bounded-duration,
self-terminating, publishes only to `cmd_vel_unguarded` through
`hil_cmd_vel_guard.py`, has no CLI flag for linear velocity at all (so
it structurally cannot request forward/backward motion).
`hil_wheel_suspension_test.py` itself was not modified.

## Sequence run (all via `hil_cmd_vel_guard.py` PID 3894, started DISARMED)

1. **Guard start** -- `max_angular_speed_rps=0.1`, `max_linear_speed_mps=0.02`,
   `required_validity_flags=7`. DISARMED, blocked as expected. `/cmd_vel`
   confirmed zero; guard confirmed sole publisher.
2. **Zero-only pipeline check** (`--pulse-angular-rps 0.0`) -- proved the
   upstream heartbeat pipeline works: `UPSTREAM_CMD_VEL_PUBLISHER_COUNT_INVALID`
   and `HEARTBEAT_STALE_OR_MISSING` both cleared while the publisher ran,
   leaving only `DISARMED`. No motion.
3. **Arm** (`/hil_guard/arm` = true). No live publisher at that instant;
   guard correctly still blocked on stale heartbeat. `/cmd_vel` zero.
4. **+0.1 rad/s pulse, run 1** (2s). Live capture: `linear.x` 166/166
   samples at `0.0`; `angular.z` 146 at `0.0`, 20 at exactly `0.1` (max
   abs 0.1, never exceeded). `/cmd_vel` zero afterward.
   - User's physical observation for run 1: left/right wheels rotated
     in opposite directions (YES); left wheel BACKWARD, right wheel
     FORWARD; smooth (YES); no abnormal vibration/noise (NO); both
     wheels stopped immediately (YES); no other anomaly. This matches
     the standard differential-drive convention for a positive
     (counter-clockwise) `angular.z`.
5. **+0.1 rad/s pulse, run 2** (repeat, same parameters, at the user's
   request for clearer observation, 2s). Live capture: `linear.x`
   165/165 at `0.0`; `angular.z` 125 at `0.0`, 40 at exactly `0.1`.
   `/cmd_vel` zero afterward.
6. **+0.1 rad/s pulse, run 3** (longer duration, 5s, at the user's
   request). Live capture: `linear.x` 242/242 at `0.0`; `angular.z` 142
   at `0.0`, 100 at exactly `0.1` (sustained through the full 5s pulse
   window, never exceeded). `/cmd_vel` zero afterward.
7. **-0.1 rad/s pulse** (2s). Live capture: `linear.x` 178/178 at `0.0`;
   `angular.z` 138 at `0.0`, 40 at exactly `-0.1` (abs max 0.1, never
   exceeded). `/cmd_vel` zero afterward.
   - User's physical observation: left wheel FORWARD, right wheel
     BACKWARD (YES); smooth (YES); no abnormal vibration/noise (NO);
     both wheels stopped immediately (YES); no other anomaly. Exact
     mirror-image of the +0.1 rad/s result, as expected.
8. **Disarm** (`/hil_guard/arm` = false, logged
   `HIL_GUARD_ARM_STATE_CHANGED armed=False`). `/cmd_vel` confirmed zero.
9. **Teardown**: guard stopped by its exact PID (3894) via `kill -INT`,
   never `pkill`. Confirmed no `hil_cmd_vel_guard` or
   `hil_angular_suspension_test` process remains, and `/cmd_vel`
   `Publisher count: 0` afterward.

## Verdict

**`SUSPENDED_WHEEL_ANGULAR_DIAGNOSTIC_TEST_PASS`.** Both `+0.1 rad/s`
and `-0.1 rad/s` produced the expected, opposite, symmetric
differential-wheel rotation, smooth, no vibration/noise, immediate
stop, and every guarded `/cmd_vel` sample numerically confirmed within
`abs(angular.z) <= 0.1` and `linear.x == 0.0`. No collision, no
unexpected motion, no process left running, no nonzero `/cmd_vel` after
teardown.

This test does **not** establish a final measured safe ground-motion
turning rate -- 0.1 rad/s was a conservative, arbitrarily chosen
test-only value, not derived from any torque/traction/stability
analysis for actual ground contact (which introduces friction and load
dynamics entirely absent when wheels are suspended). It also does not
authorize ground motion and is not an `EXCLUSIONARY_HIL_PILOT`.
Field-geometry measurement remains outstanding before any ground-motion
test can run.

## Evidence

Raw logs (from `/home/eamon/epuck_comm_bags/` on the native WSL
filesystem, SHA-256-verified against the Windows copies in
`raw_logs/` below, copies never overwritten):

| File | SHA-256 |
|---|---|
| `hil_guard_angular_test.log` | `00eca371053a9bdb0ec5eecdb2893867c63dd9676fba1a3a3d80d4c2015d8a66` |
| `cmd_vel_during_pos01_pulse.log` | `d334551a7b1d833d8be1fb3a7ca70b01e5eaffe08636253ef82c632d2766602d` |
| `cmd_vel_during_pos01_pulse_repeat.log` | `dd98776243587dc6737da46b23018c34bbacd477a28cb77cfed6402450efaa13` |
| `cmd_vel_during_pos01_pulse_longer.log` | `f9d79eec8657c9ad11cf8a1ace3ad47dc74ecf7b509ad368066992476f36a6e1` |
| `cmd_vel_during_neg01_pulse.log` | `cfc1bf64c34e05a9574ff1732628e36d4606d160d2af8b3f2ef41398027baa3b` |

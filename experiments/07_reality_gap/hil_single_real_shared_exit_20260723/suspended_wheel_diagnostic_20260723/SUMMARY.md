# Suspended-wheel diagnostic test -- epuck5809 (2026-07-23)

**Classification: `DIAGNOSTIC_PHYSICAL`, stationary/suspended-wheel only.
Not a ground-motion trial, not an `EXCLUSIONARY_HIL_PILOT`, not part of
any formal n=5 batch.** Robot on stand, wheels clear of the ground
throughout. User present at the emergency stop for the entire test.
Frozen git commit at test time: `6aa7fd7298639394b1b4ff9f80daaa5727bb0d13`.

## Precondition confirmations (verbatim, from the user, same session)

```
ROBOT_ON_STAND=YES
WHEELS_CLEAR_OF_GROUND=YES
USER_AT_EMERGENCY_STOP=YES
TEST_AREA_CLEAR=YES
```

## Physical ROS stack state at test time

Brought up this session via user-run, per-window commands (each
individually verified before proceeding to the next):

| Component | Command | PID(s) |
|---|---|---|
| Pi driver | `ros2 run epuck_ros2_driver driver` | 789 / 790 |
| Pi expanded TCP server | `cd /home/pi/real_robot_avoidance_v1/ && python3 pi_epuck_tcp_server_sensors.py` | 801 |
| WSL expanded bridge | `cd .../real_robot_avoidance_v1/ && python3 wsl_epuck_tcp_bridge_sensors.py` | (this session's PID, confirmed connected) |
| state_publisher | `ros2 run epuck2_comm state_publisher --ros-args -p robot_id:=1 -p source:=hardware -p use_sim_time:=false -p mode:=periodic -r state:=/epuck1/state` | wrapper 2540 / actual 2541 |

Read-only physical preflight (`run_hil_physical_preflight.sh`) confirmed
before this test: `PHYSICAL_DEVICE_REACHABLE=YES`, `DRIVER_STATUS=RUNNING`,
`BRIDGE_STATUS=CONNECTED`, `SENSOR_TOPICS_READY=YES`, `VALIDITY_FLAGS=7`,
`CMD_VEL_PUBLISHER_COUNT=0` (before the guard started).

## Test-scoped safety parameter

`hil_frozen_params.json`'s `hil_guard_limits.max_angular_speed_rps` was
set to **0.0** by explicit user decision, scoped to this test only. This
is **not** a measured turning rate for epuck5809 and **must not** be
reused for any turning or ground-motion trial -- it must be reset to
`UNCONFIRMED_PHYSICAL_MEASUREMENT` or replaced by a genuinely measured
value before any test that requires rotation. `max_linear_speed_mps`
remained at its frozen value, 0.02.

## Sequence run (all via `hil_cmd_vel_guard.py` PID 3171, started DISARMED)

1. **Guard start** -- DISARMED, blocked as expected
   (`DISARMED,UPSTREAM_CMD_VEL_PUBLISHER_COUNT_INVALID(0),HEARTBEAT_STALE_OR_MISSING`).
   `/cmd_vel` confirmed zero; guard confirmed sole publisher
   (`Publisher count: 1`, `guarded_publisher_is_self=True`).
2. **Zero-only pass** (`hil_wheel_suspension_test.py --pulse-linear-mps 0.0`)
   -- proved the upstream heartbeat pipeline works: while the publisher
   ran, `UPSTREAM_CMD_VEL_PUBLISHER_COUNT_INVALID` and
   `HEARTBEAT_STALE_OR_MISSING` both cleared, leaving only `DISARMED` as
   the block reason. No motion (disarmed).
3. **Arm** (`/hil_guard/arm` = true). With no publisher active at that
   instant, the guard still correctly blocked on
   `HEARTBEAT_STALE_OR_MISSING`; `/cmd_vel` confirmed zero.
4. **Forward pulse** (`--pulse-linear-mps 0.015 --pulse-s 2`). Live
   `/cmd_vel` capture: 143 samples at `0.0`, 40 samples at exactly
   `0.015` (max observed `0.015`), zero afterward.
   - **User's physical observation**: both wheels rotated forward
     correctly (YES); neither wheel reversed or failed to rotate (NO);
     no abnormal vibration/noise (NO); wheels stopped immediately after
     the pulse (YES); no other anomaly.
5. **Clamp test** (`--pulse-linear-mps 0.05 --pulse-s 2`, requesting
   above the 0.02 m/s cap). Live capture: upstream
   (`/cmd_vel_unguarded`) max requested `0.05`; guarded (`/cmd_vel`)
   max observed **`0.02`** -- `CLAMP_CHECK_PASS`. Angular.z confirmed
   `0.0` in every sample of the guarded output (no exceptions).
   - **User's physical observation**: both wheels rotated forward
     correctly (YES); motion stable (YES); no unexpected
     turning/vibration/noise (NO); wheels stopped after the pulse
     (YES); no other anomaly.
6. **Disarm** (`/hil_guard/arm` = false, logged
   `HIL_GUARD_ARM_STATE_CHANGED armed=False`). `/cmd_vel` confirmed
   zero.
7. **Teardown**: guard stopped by its exact PID (3171) via `kill -INT`,
   never `pkill`. Confirmed no `hil_cmd_vel_guard` or
   `hil_wheel_suspension_test` process remains, and `/cmd_vel`
   `Publisher count: 0` afterward.

## Verdict

**`SUSPENDED_WHEEL_DIAGNOSTIC_TEST_PASS`.** All observed behavior
matched expectations: correct wheel direction, low-speed forward motion
at the requested rate, clean stop, and a numerically verified 0.02 m/s
hard clamp against an intentionally over-limit request. No collision,
no unexpected motion, no process left running, no nonzero `/cmd_vel`
after teardown.

This test does **not** establish a measured safe turning rate, does
**not** authorize ground motion, and is **not** an `EXCLUSIONARY_HIL_PILOT`.
Field-geometry measurement and a genuinely measured
`max_angular_speed_rps` remain outstanding before any ground-motion
test can run.

## Evidence

Raw logs (from `/home/eamon/epuck_comm_bags/` on the native WSL
filesystem, SHA-256-verified against the Windows copies in
`raw_logs/` below, copies never overwritten):

| File | SHA-256 |
|---|---|
| `hil_guard_wheel_test.log` | `71c42dc423bf90b09da73d3f15f1e516f42fc74b83e5f3c82d6a7dc190b9eacc` |
| `cmd_vel_during_pulse_015.log` | `2f8c57bd9a0c9cf6cbbeb7067a463013d3c5f0c5ff158038ba77b8e25925b0c4` |
| `cmd_vel_unguarded_during_clamp.log` | `540141de6057e3d84b8af471096769e1cffb9265d1e27db4d2d1a214dcc43b64` |
| `cmd_vel_during_clamp.log` | `b075c9a4a646b07614b437fdd3c5c117bc327766ba1ae09d14299813ce1e1560` |

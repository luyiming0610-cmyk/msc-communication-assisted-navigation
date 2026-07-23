# Single-real-robot hardware-in-the-loop (HIL) shared-exit design (2026-07-23)

## Why this experiment exists

A second physical e-puck2/Pi-puck unit is confirmed unavailable (only
`epuck5809` exists; another student uses three units, one is faulty,
and the project has no claim to a fifth). The completed N2/N3 formal
batches (`experiments/10_cooperative_exit_navigation_20260720/`) are
pure-simulation results. This experiment characterizes the **reality
gap**: whether communicated exit information changes the *real*
robot's navigation behaviour, completion time, and safety state, when
paired with one or more simulated/virtual cooperating peers.

This is **not**, and must never be described as, a completed
dual-physical-robot experiment. The virtual peer is a scripted
kinematic stand-in (`hil_virtual_peer.py`); it does not have a genuine
physical collision-avoidance interaction with the real robot.

## Architecture

```
 [real epuck5809]                          [hil_virtual_peer.py]
   driver + bridge                            (scripted unicycle
        |                                      model, own clock)
   state_publisher.py (UNMODIFIED)                   |
        | EpuckState                          EpuckState / GoalAnnouncement
        v                                            |
 hil_topic_adapter.py --------- GoalAnnouncement -----+
   (wraps GoalNavigator,                              |
    real-time clock)                                  |
        | NavigationIntent                            |
        v
 cooperative_avoider.py (UNMODIFIED)
        | cmd_vel_unguarded
        v
 hil_cmd_vel_guard.py (NEW -- only publisher onto the real /cmd_vel)
        | cmd_vel (capped <= 0.02 m/s, fail-closed)
        v
   real e-puck2 driver
```

Everything under "(UNMODIFIED)" is exactly the code already exercised
by the N2/N3 formal batches. Every HIL-specific behaviour lives in the
five new `tools/hil_*.py` files plus `run_hil_shared_exit_trial.sh` --
nothing else in the repository is touched to make HIL work.

## Frozen HIL experiment scope

- **HIL_COMM_OFF**: `hil_topic_adapter.py` (wrapping the real robot's
  `GoalNavigator`) does not subscribe to the virtual peer's
  `GoalAnnouncement` topic. `hil_cmd_vel_guard.py` runs with
  `require_virtual_peer=false`. The real robot navigates using only its
  own state/goal-navigation/local-avoidance stack; if the virtual peer
  enters CPA range, `cooperative_avoider`'s existing peer-avoidance path
  still applies (it does not require communication, only EpuckState
  visibility -- the same distinction the N2/N3 studies draw between
  collision avoidance and exit-information communication).
- **HIL_COMM_ON**: identical to HIL_COMM_OFF, plus
  `hil_topic_adapter.py` subscribes to the virtual peer's
  `GoalAnnouncement` topic (`--goal-announcement-topic`) and
  `hil_cmd_vel_guard.py` runs with `require_virtual_peer=true`.

Exactly one dimension changes between conditions: whether the real
robot's `GoalNavigator` instance can receive the virtual peer's
`GoalAnnouncement`. Everything else (guard limits, avoider parameters,
recorder topics) is identical across conditions.

## Coordinate systems and clock domains

See `hil_frozen_params.json`'s `coordinate_system` and `clock_domains`
sections. Summary:

- The real robot and the virtual peer are expressed in one shared map
  frame (`hil_shared_exit_map`) via `state_publisher.py`'s existing
  `origin_x_m`/`origin_y_m`/`origin_yaw_rad` parameters -- no new
  transform code is required, only correctly measured origin values for
  the physical robot (the virtual peer's frame is defined to coincide
  with the map frame by construction).
- The real robot runs on a real-time (`use_sim_time:=false`) ROS clock
  via `hil_topic_adapter.py`; the virtual peer may run sim-time or
  wall-clock depending on its own launch. Because the two clock sources
  are not synchronized, every freshness check in
  `hil_cmd_vel_guard.py`/`hil_preflight.py` uses the RECEIVING node's
  own wall-clock timestamp for each message, never a sender-embedded
  stamp -- this sidesteps any need for clock sync.

## Unmeasured physical facts

Every physical fact not yet measured on the real hardware/lab space is
recorded literally as the string `UNCONFIRMED_PHYSICAL_MEASUREMENT` in
`hil_frozen_params.json`, never guessed or backfilled from the Webots
N2/N3 world files (those describe a simulated arena, not this lab
space). `hil_preflight.py`'s `check_required_params_confirmed()` walks
`required_before_ground_motion` and blocks with
`BLOCKED_AWAITING_LAB_MEASUREMENT` if any of them remain unconfirmed;
`run_hil_shared_exit_trial.sh` has no mode that bypasses this.

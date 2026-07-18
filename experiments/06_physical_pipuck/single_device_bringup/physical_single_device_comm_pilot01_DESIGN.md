# physical_single_device_comm_pilot01 — design only, NOT executed

Status: design proposal, submitted for confirmation. Nothing in this
document has been run. The robot has not moved and no `/cmd_vel` command has
been sent at any point during this bringup session.

## Preconditions (all confirmed this session, read-only)

- Pi ROS 2 Foxy driver running (`EPuck Driver has been initialized`,
  `Driver mode: hw`).
- Pi TCP server running (`Pi TCP bridge listening on 0.0.0.0:5809`).
- WSL TCP client connected (`TCP bridge connected`).
- `/epuck_bridge/status`: `connected=true`, `crc_errors=0`.
- `/scan` ≈9.09–9.20 Hz, `/odom` ≈9.09–9.15 Hz (WSL side, via bridge).
- `/tof` and `/ps0` confirmed publishing directly on the Pi's own ROS graph
  (`/tof` via `echo` ≈15.6 Hz judging by timestamp deltas; `/ps0` via `hz`
  ≈15.62 Hz, matching the historical baseline).
- No `PASS` has been written anywhere for this pilot — this is a design
  document only.

## Scope

- Robot stays stationary for the entire pilot. No armed motion.
- No `/cmd_vel` publication at all (not even zero-velocity commands beyond
  what the bridge's own watchdogs already send).
- Uses the **already-running** base bridge (`/scan`, `/odom`,
  `/epuck_bridge/status`) plus a read-only sample of `/tof` and `/ps0`–`/ps7`
  taken directly on the Pi (as already done this session) — does not require
  switching to the sensor-extended bridge unless the extended `range_sensors`
  field is specifically wanted; if so, that is a separate confirmation, not
  assumed here.

## What is recorded

- `sequence` (state message sequence numbers, from `/epuck_bridge/status`'s
  `state_seq_first`/`state_seq_last`/`state_unique_received`/`state_missing`/
  `state_out_of_order`/`state_delivery_ratio` fields, already implemented
  per `重复实验阶段补充说明_20260715.md`).
- `ack` / RTT: **sequence-acknowledgement RTT** (`last_rtt_ms` from
  `/epuck_bridge/status`), sampled continuously, not a single snapshot.
  Report distribution statistics (mean, median, min, max, p95, sample count),
  not a single value.
- `estimated_one_way_ms` reported **only** as `RTT/2`, explicitly labeled a
  symmetric-link estimate — never described as measured one-way network
  latency.
- `wall_clock_delta_ms` recorded **only** as a clock-offset diagnostic,
  explicitly never used to compute or report a latency figure.
- CRC error count (`crc_errors`).
- Reconnect count (bridge disconnect/reconnect events over the run).
- `/scan`, `/odom`, `/tof`, IR (`/ps0`–`/ps7`) publish rates (Hz), sampled
  via `ros2 topic hz` or an equivalent counting subscriber over the full run
  duration, not a short snapshot.
- CPU and memory usage on both the Pi (`top`/`/proc` sampling) and the WSL
  side (`psutil`-style sampling), at a fixed interval.
- Wi-Fi link quality on the Pi (e.g. `iw dev wlan0 link` or
  `/proc/net/wireless` signal/noise/link-quality figures), sampled at the
  same interval.

## Directory and data-integrity rules

- New, unique directory name: `physical_single_device_comm_pilot01` (or
  `_attempt02` etc. if a retry is ever needed — never reuse or overwrite,
  per the same discipline already established for the simulation-side
  Objective 5 work this session).
- Does not touch, overwrite, or delete any of the 8 existing
  `real_robot_avoidance_v1/experiment_data_20260715/` bag directories.
- Does not touch the old JSON `/epuck1/state`-family scripts.

## Explicitly out of scope for this pilot

- No `cooperative_avoider` or any avoidance controller.
- No `EpuckState.msg` conversion (that converter does not exist yet per
  `physical_protocol_gap_report.md` — building it is a separate, later
  decision).
- No motion of any kind.
- No claim of PASS/FAIL until the run actually happens and is read back.

## Open questions requiring explicit confirmation before running

1. Run duration — how many minutes of stationary logging is wanted?
2. Sampling interval for CPU/memory/Wi-Fi (e.g. 1s, 5s)?
3. Base bridge only, or switch to the sensor-extended bridge
   (`pi_epuck_tcp_server_sensors.py` / `wsl_epuck_tcp_bridge_sensors.py`) to
   also capture the `range_sensors` (`ps0`–`ps7`+`tof`) field inside the
   bridge protocol itself, rather than sampling `/tof`/`/ps0`–`/ps7` directly
   on the Pi as done this session?

# Physical protocol gap report (2026-07-18)

Read-only audit. No wire schema was modified. Compares three things:

1. The old JSON `/epuck1/state` publisher: `epuck1_state_publisher.py`
   (Windows snapshot copy read at
   `simulation_comm_experiment_v1/source_snapshot/epuck1_state_publisher.py`;
   WSL original per handoff doc at
   `/home/eamon/epuck_ws/epuck_comm_project/epuck1_state_publisher.py`, not
   independently re-read this pass — the two are expected to be identical
   per the handoff doc, not verified byte-for-byte here).
2. The current frozen `EpuckState.msg`
   (`src/epuck2_comm_interfaces/msg/EpuckState.msg`, `PROTOCOL_VERSION=1`,
   `protocol_v1.1_stamp_semantics`).
3. What the real-robot TCP bridge (`real_robot_bridge/`,
   `real_robot_avoidance_v1/`) currently carries, to determine what a WSL-side
   converter would need to fill in.

## Field-by-field comparison

| Old JSON field | New EpuckState field | Compatible? | Note |
|---|---|---|---|
| `robot_id` (string, e.g. `"epuck1"`) | `robot_id` (uint16) | NO, needs mapping | old is a free-text string, new is a numeric ID; a converter must assign a fixed numeric ID (not present in old data at all as a number) |
| `source` (string, **hardcoded `"webots"`**) | `source` (uint8 enum) | NO, and old value is WRONG for hardware runs | the old script writes `"webots"` unconditionally, even in the validated runs that fed it real `/odom`/`/scan` from the physical Pi bridge (see `VALIDATION_20260715.md`, `epuck1_state_publisher.py` "worked unchanged with real /scan and /odom"). **Old data's `source` field must never be trusted; it does not indicate sim vs. real.** A converter must set `SOURCE_HARDWARE=2` explicitly, never copy the old field. |
| `seq` (Python int, unbounded) | `sequence` (uint32, wraps at 2**32) | Needs adaptation | old code never wraps; harmless in short runs, but a converter must apply `% (2**32)` like `state_publisher.py` already does |
| `timestamp` (`time.time()`, wall-clock epoch) | `stamp` (`builtin_interfaces/Time`, from `get_clock().now()`) | Compatible in domain **on real hardware only** | on real hardware `use_sim_time=false`, so `get_clock().now()` is also wall-clock (`RCL_SYSTEM_TIME`) — same domain as `time.time()`, unlike the sim-vs-sim-recording clock-domain bug found and fixed this session for Objective 5. This is a genuinely different situation from the simulation case; do not assume the same bug applies here, and do not assume the two are numerically identical either (different clock APIs) — a converter must set `stamp` from its own `get_clock().now()`, not copy the old field. |
| `x`, `y`, `yaw` | `x_m`, `y_m`, `yaw_rad` | YES, same physical quantity | both derived from the same `/odom`; new field names only add explicit units |
| `linear_velocity`, `angular_velocity` | `linear_velocity_mps`, `angular_velocity_rps` | YES | same as above |
| `front_distance`, `left_distance`, `right_distance` | `front_distance_m`, `left_distance_m`, `right_distance_m` | **NO — different no-detection convention** | old code returns `LaserScan.range_max` (a **finite** value, e.g. ~1.0 m) when no valid return exists in a sector; `EpuckState.msg`'s documented convention is **`+Inf`** for no detection. Any code that does `math.isinf()` on old-shaped data will behave differently than intended. A converter must not literally copy these fields; it must recompute using the current no-detection convention. |
| `obstacle_status` (string: `clear`/`front_obstacle`/`left_obstacle`/`right_obstacle`) | `obstacle_status` (uint8 enum: `OBSTACLE_UNKNOWN`/`CLEAR`/`FRONT`/`LEFT`/`RIGHT`/`MULTIPLE`) | NO, and old is lossy | old never reports `MULTIPLE` (single if/elif priority chain: front > left > right) and never distinguishes true-clear from unknown-because-no-scan-yet (both collapse to `"clear"`) |
| — (absent) | `validity_flags` (uint8, `FLAG_ODOM_VALID`/`FLAG_IR_VALID`/`FLAG_TOF_VALID`) | **MISSING, cannot be backfilled** | old script has no per-sensor freshness signal in the message itself (only an internal `self.odom is None` gate that isn't published); a converter must compute this fresh from the bridge's own per-topic timestamps, it cannot be reconstructed from old recorded data |
| — (absent) | `version`, `PROTOCOL_VERSION` marker | **MISSING** | old data carries no protocol version at all |
| — (absent) | `left_front_m`, `left_mid_m`, `left_rear_m`, `right_front_m`, `right_mid_m`, `right_rear_m` (v4 zones) | **MISSING, but fillable going forward** | old JSON has no per-IR-sensor breakdown at all (only 3 LaserScan-derived sectors, no rear coverage). The **sensor-extended** bridge (`real_robot_avoidance_v1/pi_epuck_tcp_server_sensors.py`) already forwards a `range_sensors` dict with raw `ps0`..`ps7` + `tof` values in its own bridge-level JSON (not `EpuckState.msg`) — a new WSL-side converter node could compute the v4 zones from that raw data reusing `state_publisher.py`'s existing zone-mapping logic, but this does not exist yet and nothing here should be read as claiming it does. |

## Message-type transport compatibility (Foxy vs. Humble)

- `sensor_msgs/msg/Range`, `sensor_msgs/msg/LaserScan`, `nav_msgs/msg/Odometry`
  are standard `common_interfaces` types present in both ROS 2 Foxy and
  Humble; the existing architecture already relies on this (the Pi driver
  publishes these natively, the WSL bridge subscribes to them after they're
  re-published via the TCP/JSON bridge — not via cross-distro DDS discovery,
  which the handoff doc already establishes does not work reliably here).
- `EpuckState.msg` is a **custom** message that only exists in this
  repository's `epuck2_comm_interfaces` package, built for ROS 2 Humble. It
  is not installed on the Pi's Foxy workspace at all, and cross-distro
  binary CDR compatibility for a custom, repo-local message is not something
  to assume without explicit testing. **This is not actually a problem for
  the current architecture**: the TCP/JSON bridge already fully isolates the
  two ROS graphs — the Pi/Foxy side never needs to know `EpuckState` exists
  at all. Any `EpuckState` conversion must happen entirely on the WSL/Humble
  side, in a new node consuming the bridge's already-republished topics
  (`/scan`, `/odom`, and — if the sensor-extended bridge is used —
  `/epuck_bridge/status`'s `range_sensors` field), not by trying to publish
  `EpuckState` from the Pi.

## What must go through the TCP bridge for conversion

Nothing new needs to be added to the wire protocol between Pi and WSL to
attempt an `EpuckState` conversion — the sensor-extended bridge
(`pi_epuck_tcp_server_sensors.py` / `wsl_epuck_tcp_bridge_sensors.py`)
already carries raw `/scan`, `/odom`, and per-IR-sensor + ToF ranges. What is
missing is a **new WSL-side converter node** (not yet written) that:

- assigns a fixed numeric `robot_id`,
- sets `source=SOURCE_HARDWARE` explicitly (never copies any old field),
- computes `sequence` with proper `% 2**32` wraparound,
- sets `stamp` from its own `get_clock().now()` (real hardware, so this is
  wall-clock via `RCL_SYSTEM_TIME`, not sim time — a different, and in this
  case unproblematic, situation from the Objective 5 sim clock-domain bug),
- recomputes front/left/right (and, if the sensor-extended bridge is used,
  the six v4 zones) using the **current** `+Inf`-for-no-detection convention,
  not the old finite-fallback convention,
- computes `validity_flags` from the bridge's own per-topic freshness (data
  age relative to `/epuck_bridge/status`'s `last_state_age_s` or equivalent
  per-topic tracking), not from anything present in old recorded data.

This node does not exist yet. Building it is out of scope for this read-only
audit and is not started here.

## Can old physical data be compared with current formal Objective 5 batches?

**No.** Old data (JSON over `std_msgs/String`, no protocol version, wrong
hardcoded `source`, finite-fallback distance convention, no
`validity_flags`, no zones) cannot be pooled with or directly compared
against current `EpuckState`-based Objective 5 formal statistics. It can
serve as **qualitative historical validation evidence** (bridge works, real
sensors, real motors, real wireless link, no crashes) but not as
protocol-compatible quantitative data. See the accompanying
`physical_data_inventory.md` for the per-batch classification.

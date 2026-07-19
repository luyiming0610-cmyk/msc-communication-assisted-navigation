# Physical protocol adaptation: capability audit + minimal design (2026-07-18)

Read-only audit. **No code was changed.** No converter was implemented or
run. This follows `physical_protocol_gap_report.md` (2026-07-18, earlier
this session) and is informed by the field data from
`physical_single_device_transport_diagnostic_pilot01_attempt01`.

## 1. Can `state_publisher.py` already consume the bridge and publish `EpuckState`?

**Likely yes, with no code changes — but not empirically verified this
session (no test was run against real hardware topics).**

`src/epuck2_comm/epuck2_comm/state_publisher.py` subscribes to (all
relative topic names, i.e. resolve to the global namespace unless remapped):

- `odom` (`nav_msgs/msg/Odometry`)
- `ps0`..`ps7` (`sensor_msgs/msg/Range`)
- `tof` (`sensor_msgs/msg/Range`)

The **extended** sensor bridge
(`1-1.实体机器人避障实验/wsl_epuck_tcp_bridge_sensors.py`, read directly
this session) publishes exactly these topics, in the same global namespace,
with the same message types:

```python
self._scan_pub = self.create_publisher(LaserScan, "/scan", 10)
self._odom_pub = self.create_publisher(Odometry, "/odom", 10)
self._range_pubs = {name: self.create_publisher(Range, "/" + name, 10)
                     for name in RANGE_SENSOR_NAMES}  # ps0..ps7, tof
```

No topic remapping is needed for `state_publisher.py` to receive this data
as-is.

**QoS**: `state_publisher.py` subscribes with `qos_profile_sensor_data`
(BEST_EFFORT). The bridge's publishers use `create_publisher(..., 10)` with
no explicit QoS profile, i.e. the rclpy default (RELIABLE, KEEP_LAST depth
10). Per the standard ROS 2 QoS compatibility rules, a BEST_EFFORT-requesting
subscriber is compatible with a RELIABLE publisher (BEST_EFFORT is the
weaker requirement) — so this **should** match without a mismatch, but this
was reasoned from the QoS compatibility matrix, not confirmed by actually
running the two together. **Recommended first action if this is pursued: a
short read-only smoke test (`ros2 topic echo /state` after launching
`state_publisher` against the extended bridge, still zero `/cmd_vel`) rather
than assuming success.**

**Parameters already exist to set this up correctly** (no hardcoded-wrong
values, unlike the old JSON script):

| parameter | value needed for hardware | already supported? |
|---|---|---|
| `source` | `hardware` (→ `EpuckState.SOURCE_HARDWARE`) | yes, `-p source:=hardware` |
| `robot_id` | e.g. `1` | yes, numeric parameter already |
| `mode` | `periodic` or `event` | yes |
| `use_sim_time` | `false` (default) | yes, real hardware wall clock via `WAITING_FOR_CLOCK` gate already added this session |

## 2. Does the expanded bridge supply full sensor input without changing the Pi TCP wire protocol?

**Yes.** `pi_epuck_tcp_server_sensors.py` / `wsl_epuck_tcp_bridge_sensors.py`
use the **same** `epuck_bridge_v1` envelope (`bridge_protocol.py`, unchanged,
same CRC32 framing, same TCP port 5809) — the extended variant only adds a
`range_sensors` dict field inside the existing JSON `state` payload. No wire
protocol version bump, no port change.

**Additional finding, materially relevant to the diagnostic pilot's
NOT_MEASURABLE PDR gap**: the extended bridge **already implements**
`state_seq_first` / `state_seq_last` / `state_unique_received` /
`state_missing` / `state_out_of_order` / `state_delivery_ratio` in its own
`/epuck_bridge/status` output (`update_sequence_stats()`, read directly this
session). The **base** bridge currently in use does not expose these at
all. Switching to the extended bridge would therefore resolve BOTH gaps at
once: full sensor input (`/tof`, `/ps0`-`/ps7`) AND real
sequence-based PDR — without any wire-protocol change.

## 3. Field mapping table

| physical data source | bridge topic/JSON field | EpuckState field | unit conversion | validity flag | missing/degraded strategy |
|---|---|---|---|---|---|
| Pi `/odom` (real wheel odometry) | extended/base bridge → WSL `/odom` (`nav_msgs/Odometry`) | `x_m`, `y_m` (from `pose.pose.position`), `yaw_rad` (from quaternion, same `atan2` formula `state_publisher.py` already uses), `linear_velocity_mps`, `angular_velocity_rps` (from `twist.twist`) | none needed, already SI units | `FLAG_ODOM_VALID` | `state_publisher.py` already gates on `odom_timeout_s` staleness |
| Pi `/ps0`-`/ps7` raw IR (extended bridge only) | WSL `/ps0`-`/ps7` (`sensor_msgs/Range`) | `front_distance_m`/`left_distance_m`/`right_distance_m` (existing v1-v3 3-sector reduction) + `left_front_m`/`left_mid_m`/`left_rear_m`/`right_front_m`/`right_mid_m`/`right_rear_m` (existing v4 6-zone mapping) | none, `state_publisher.py`'s existing `_snapshot()`/zone logic already expects raw IR ranges in meters | `FLAG_IR_VALID` | already gated on `sensor_timeout_s`; `+Inf` for no-detection (matches current convention, NOT the old JSON script's finite-fallback bug) |
| Pi `/tof` (extended bridge only) | WSL `/tof` (`sensor_msgs/Range`) | folds into the same front/left/right + zone computation `state_publisher.py` already does for simulated ToF | none | `FLAG_TOF_VALID` | same staleness gate |
| N/A (Pi does not currently publish this) | N/A | `sequence` (uint32) | `state_publisher.py` maintains its own `self.sequence` counter with existing `% 2**32` wraparound — unrelated to the bridge's own internal `seq`, no conversion needed | N/A | already implemented |
| N/A | N/A | `stamp` | `self.get_clock().now().to_msg()`, real wall-clock under `use_sim_time=false`, already gated by this session's `WAITING_FOR_CLOCK` fix | N/A | already implemented |
| launch parameter, NOT a topic | `-p source:=hardware` | `source` | `SOURCE_HARDWARE=2` — must be set explicitly via parameter; never inferred or copied from any bridge field (the old JSON script's exact mistake) | N/A | N/A |
| `/scan` (LaserScan, base+extended bridge) | WSL `/scan` | **not consumed by `state_publisher.py` at all** (only the old JSON script used it) | N/A | N/A | N/A -- not part of the current protocol's obstacle model |

## 4. Recommendation

**Reuse `state_publisher.py` as-is. Do not build a new converter.** The
existing node already has the right subscriptions, the right parameters,
the right no-detection convention, and the right staleness/validity-flag
logic — it was written generically enough (for Webots) that it does not
need to know its `/odom`/`/ps*`/`/tof` inputs came from a TCP bridge instead
of a simulator. The only two things this session did NOT do (explicitly
out of scope, no implementation attempted):

1. Empirically launch `state_publisher` against the extended bridge's
   topics and confirm QoS actually matches and `/state` messages look
   correct (recommended smoke test, not done).
2. Switch from the base bridge to the extended bridge (deferred per
   instruction — "不自动开始expanded bridge").

## 5. Constraints this design respects (per instruction, all unchanged)

- Current `EpuckState` protocol version is unchanged (`PROTOCOL_VERSION=1`,
  `protocol_v1.1_stamp_semantics`) — no new fields, no reordering.
- Simulation and physical hardware would use the exact same message
  definition — `state_publisher.py` is already written to be
  source-agnostic (a `source` parameter selects `SOURCE_WEBOTS` vs.
  `SOURCE_HARDWARE`, never inferred).
- No Webots-specific `source` value would ever be written into a
  hardware-origin message, unlike the old JSON script's `"webots"` hardcode.
- `stamp` uses production-time (real `get_clock().now()`) semantics,
  already implemented this session.
- `sequence`, `validity_flags`, and the invalid-data safe-stop semantics
  (`FLAG_*_VALID` bits, staleness timeouts) are already explicit and
  unchanged from the simulation path.
- Old physical data (`LEGACY_PHYSICAL_EVIDENCE`) stays completely separate
  from any future `EpuckState`-based physical data — nothing here
  backfills or reprocesses it.

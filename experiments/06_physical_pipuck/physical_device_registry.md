# Physical device registry (2026-07-18)

Read-only bringup record. No experiment PASS/FAIL is recorded here — only
verified operational state. See `physical_single_device_comm_pilot01_DESIGN.md`
for the first pilot design (not yet run).

## Robot

- School e-puck2, serial #5809. Only one physical unit exists — the paper
  must never claim a completed dual-physical-robot experiment; the second
  node is a virtual/laptop peer (`fake_epuck2_state_publisher.py`) per the
  handoff document.
- Pi-puck + University of York expansion board + Raspberry Pi Zero 2 W,
  mounted on top of the e-puck2. Selector position A (decimal 10).
- e-puck2 main processor firmware: currently the **temporary** official
  Pi-puck I2C extension firmware
  (`e-puck2_main-processor_extension_b346841_07.06.19.elf`,
  SHA-256 `1ED206241B61D4A0560039AB91D8FA0E52D43EDAC04C7D754EF28B706DB03623`).
  The original school firmware backup
  (`epuck2_5809_main_flash_backup_20260715.bin`,
  SHA-256 `C0EA234A57FEC0F6743D8BFC82C1B9369514CFAB168A53F239C423C0A57400BE`)
  must be restored only after all physical experiments are finished, with a
  post-restore SHA-256 re-verification — **not done yet, not attempted this
  session**.

## Network (verified this session, read-only)

- Hotspot SSID `EPUCK5809` open on the laptop; Pi joined.
- Pi IP: `192.168.137.71` (matches the handoff document).
- Windows ping: 3/3 success, 0% loss, RTT min 2ms / max 12ms / mean ~6ms
  (as reported by the user before this session's checks began).
- WSL ping (this session): 3/3 success, 0% loss, RTT 2.148–3.158ms.
- SSH: `systemctl is-active ssh` → `active`, listening on `0.0.0.0:22` and
  `[::]:22`. Note: the user's initial report this session said TCP 22 was
  not listening / SSH failed — that did not reproduce; SSH connected
  successfully both from a second SSH client session and was already
  `active` per systemd. No fix was applied (nothing was actually down by
  the time it was checked); this discrepancy is recorded, not explained.
- COM8 (Pi-puck CP210x UART console): connected successfully, 115200/8/N/1,
  reached the `raspberrypi` login prompt and a working shell.

## ROS 2 / bridge processes (verified this session)

- Pi ROS 2 Foxy driver (`ros2 run epuck_ros2_driver driver`): started
  successfully, log matches the handoff document exactly
  (`EPuck Driver has been initialized`, `Driver mode: hw`). Left running in
  the COM8 serial console.
- Pi TCP server (`~/real_robot_bridge/pi_epuck_tcp_server.py`, base
  version — not the sensor-extended variant): started successfully,
  `Pi TCP bridge listening on 0.0.0.0:5809; watchdog 0.50s; limits
  0.040m/s 2.000rad/s`. Left running in a second SSH session.
- WSL TCP client (`~/epuck_ws/epuck_comm_project/real_robot_bridge/
  wsl_epuck_tcp_bridge.py`, base version): started in WSL and reported
  `TCP bridge connected`. Left running in the background.

## Topics verified this session (read-only, no `/cmd_vel` published)

| topic | where checked | result |
|---|---|---|
| `/epuck_bridge/status` | WSL | `connected=true`, `crc_errors=0`, `rx_count=410` (at time of check), `last_state_age_s≈0.012s`, `last_rtt_ms≈56.5ms` (single-sample snapshot only — a preliminary observation, NOT a distribution; whether this is actually elevated vs. the ~5–18ms historical range cannot be concluded from one sample, and is not concluded here), `estimated_one_way_ms≈28.2ms` (RTT/2 estimate only), `wall_clock_delta_ms≈12.5ms` (clock-offset diagnostic only, not latency) |
| `/scan` | WSL (via bridge) | ≈9.09–9.20 Hz |
| `/odom` | WSL (via bridge) | ≈9.09–9.15 Hz |
| `/tof` | Pi (direct) | `ros2 topic hz` gave no output; root cause not proven — possibly related to CLI/QoS compatibility behavior, not confirmed. `ros2 topic echo` confirmed continuous data, timestamp deltas ≈64ms (≈15.6 Hz), `range≈0.47–0.49m` (stationary baseline) |
| `/ps0` | Pi (direct) | `ros2 topic hz` ≈15.62 Hz, matches the historical baseline (`~15.616 Hz`) in the handoff document |

`/ps1`–`/ps7` were not individually re-checked this session (not required
given `/ps0` and `/tof` both confirmed against the historical baseline);
flag if per-sensor verification of the remaining 6 is wanted before the
static pilot.

## Not yet done (explicitly, per instruction — no PASS written for any of these)

- `physical_single_device_comm_pilot01` has not been run (design only, see
  the companion design document).
- No `EpuckState.msg` converter exists yet for real-hardware data (see
  `physical_protocol_gap_report.md`).
- No physical avoidance/motion experiment has been run this session.
- Original school firmware has not been restored (correctly — experiments
  are not finished).

# physical_expanded_bridge_epuckstate_integration_pilot01 (attempt01) — summary

**Verdict: `PASS_WITH_LIMITATION`.** `DIAGNOSTIC_PHYSICAL` only — this checks
that the expanded Pi-TCP-WSL bridge, when paired with the current
`epuck2_comm_interfaces/msg/EpuckState` protocol via `state_publisher.py`,
integrates correctly while the robot is stationary. **This is NOT a formal
baseline.** No ground motion occurred; no controller was launched.

## Process roles (corrected mid-session — see note)

A PID mislabeling was caught and corrected before any action was taken on
the wrong PID: an earlier draft incorrectly attributed PID 1206 to the
expanded server. Direct `pgrep -af`/`ss -ltnp` evidence, re-confirmed by
the user, established:

| role | PID | evidence |
|---|---|---|
| Pi driver | 813 / 814 | unchanged all session |
| Pi expanded server (`pi_epuck_tcp_server_sensors.py`) | **1168** | `pgrep -af` + port 5809 listener |
| Pi system sampler (`pi_system_sampler.py`) | **1206** | `pgrep -af`, command line matches this pilot's filename |
| WSL expanded bridge (`wsl_epuck_tcp_bridge_sensors.py`) | 2535 | |
| `state_publisher` (wrapper / actual) | 2813 / 2834 | |

## Run

- Robot wheels suspended throughout; zero ground motion. `/cmd_vel`: 0
  nonzero messages in the bag, 0 publishers on the WSL graph at all three
  checkpoints (start/mid/end).
- Main statistical window: **a real, measured 240.0s window from the
  4-way overlap** of Pi CSV / WSL system CSV / WSL status CSV / rosbag
  timestamps (overlap span 294.06s: 1784380377.24–1784380671.30 Unix
  seconds; window centered within it, 1784380404.27–1784380644.27).

## Timestamp integrity (measured)

| source | valid rows | monotonic | duplicates | max gap |
|---|---|---|---|---|
| wsl_expanded_status.csv | 296 | yes | 0 | 1.003s |
| wsl_system_metrics.csv | 307 | yes | 0 | 1.003s |
| pi_system_metrics.csv | 765 | yes | 0 | 1.017s |

Pi sampler spans 768.9s because it was started before and stopped after
the 300s ROS-side run — not an anomaly, per instruction.

## Known issue, impact verified AFTER the fact (not assumed)

`wsl_expanded_pilot_recorder.py` hit the same benign trailing
`rcl_shutdown already called` exception (SIGINT racing rclpy's own
handler) as `sequence_counter.py` and `wsl_transport_recorder.py`
earlier this session — **not yet patched in this file**. Per instruction,
its impact was **verified, not presumed, before any conclusion was
drawn**: the resulting `wsl_expanded_status.csv` has 296 valid,
monotonic, gap-free rows; all 3 `/cmd_vel` checkpoints (start/mid/end)
are present with `publisher_count=0`; and the analyzer's own tier A/B/C
computations over this data produced 0 `fail_reasons`. Only after these
checks passed is the exception treated as having no data-integrity
impact.

## Tier A — Pi → WSL application-level state transport

From the expanded bridge's own `/epuck_bridge/status` cumulative
counters (`update_sequence_stats()` in `wsl_epuck_tcp_bridge_sensors.py`):

- `state_seq_first=0`, `state_seq_last=9596`, `state_unique_received=9597`
- `state_missing=0`
- `state_out_of_order=0` (**this counter includes any duplicate as well
  as any genuine reorder** — the bridge's own code does not separately
  track duplicates; no separate duplicate figure is fabricated here)
- **`APPLICATION_STATE_SEQUENCE_DELIVERY_RATIO = 1.0`** — Pi
  application-level state-sequence receipt completeness at the WSL
  bridge. **Not** IP packet loss, **not** TCP packet loss, **not**
  automatically equal to tier B or any rosbag PDR.

## Tier B — state_publisher output → rosbag capture

Using `EpuckState.sequence` within the bag's own main-window capture
(this measures ONLY whether the bag captured every message
`state_publisher` generated — it does **not** measure tier A's Pi→WSL
transport):

- 2150 messages, sequence 6048→8197, 0 gaps, 0 duplicates, 0
  out-of-order, **`bag_capture_ratio = 1.0`**.

## Tier C — raw sensor topics (no source-side sequence, no PDR claimed)

`/odom`, `/scan`, `/tof`, `/ps0`–`/ps7`: each 2194 messages in the main
window, ≈9.15Hz average rate, max inter-arrival gap ≈0.30s, **0 total
stall time**.

## RTT / state_age (main window, 240 status ticks)

- **`rtt_status_snapshot_ms`** (1Hz snapshot of the most recently
  completed individual RTT — not a full transaction census): mean
  38.4ms, median 8.4ms, p95 119.1ms, p99 144.4ms, max 169.7ms, min 5.9ms.
  Tail: 65 samples (27.1%) > 50ms, 65 (27.1%) > 100ms, 0 (0.0%) > 200ms.
  Longest consecutive high-RTT run: 5 ticks. No root cause claimed.
- **`state_age_s`** (formula: `time.time() - self._last_state_time`,
  both operands WSL-local `time.time()`, single clock domain — VALID):
  mean 0.059s, median 0.058s, p95 0.107s, max 0.262s.
- **One-way Pi→WSL latency: NOT COMPUTED.** No NTP/chrony clock-sync
  procedure has been run or verified.

## EpuckState field checks (main window, all messages)

- `version=1`, `source=2` (`SOURCE_HARDWARE`, never a Webots hardcode),
  `robot_id=1` — **0 field errors** across all 2150 main-window messages.
- `validity_flags`: constant `7` (`ODOM_VALID|IR_VALID|TOF_VALID`) for
  the entire 239.9s main window — never dropped.
- NaN/Inf accounting: **0 NaN anywhere**; 6377 protocol-allowed `+Inf`
  samples (no-detection convention in distance/zone fields); 0 unexpected
  `+/-Inf` in pose/velocity fields.

## System metrics (main window)

- WSL: CPU mean 3.99% (p95 6.59%), mem mean 661.2MB.
- Pi: CPU mean 14.40% (p95 15.90%, higher than the base-bridge pilot's
  ~8.3%, consistent with the extra sensor-forwarding work the expanded
  server does), mem mean 169.7MB, Wi-Fi link quality 70/70 (max)
  throughout, signal mean ≈-25.1dBm.

## Scope reminder

This pilot demonstrates the expanded bridge + `state_publisher` +
`EpuckState` integration is functioning correctly while stationary. It
is **not** a formal communication-performance baseline and does **not**
authorize ground motion, avoidance behaviour, or any controller — those
remain separate, unstarted, future decisions.

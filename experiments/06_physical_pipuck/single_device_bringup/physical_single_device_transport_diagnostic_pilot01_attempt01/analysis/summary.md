# physical_single_device_transport_diagnostic_pilot01 (attempt01) — summary

**Verdict: `PASS_WITH_LIMITATION` — BASIC PHYSICAL TRANSPORT STABILITY**
(see `verdict.json` for the machine-readable form). **This is NOT a formal
communication-performance PASS, NOT a formal EpuckState protocol baseline,
and NOT a PDR experiment** — it validates ONLY the base Pi-TCP-WSL
transport link (`epuck_bridge_v1`, base variant, sensor-extended bridge not
used this pilot). This verdict is final for this attempt; the attempt was
not, and will not be, re-run to chase a different label.

**Corrections applied 2026-07-18 (after the original run, before any
further pilot) — this section documents what changed and why, per
instruction; no data was re-collected, only wording/interpretation was
corrected:**
1. RTT sample provenance clarified (was ambiguous about per-transaction vs.
   snapshot).
2. RTT tail distribution quantified (>50/100/200ms counts, longest
   consecutive high-RTT run) with no root-cause claim.
3. `state_age_s`'s exact formula, fields, and clock source spelled out to
   justify VALID.
4. The `/cmd_vel` zero-motion claim reframed as four joint, independent
   confirmations rather than "bag absence is the strongest evidence" (bag
   absence alone cannot prove no command was ever produced elsewhere in
   the system).
5. The bridge's delivery-ratio concept (relevant to the *next* pilot, not
   this one, since the base bridge doesn't expose it) is name-reserved as
   `APPLICATION_STATE_SEQUENCE_DELIVERY_RATIO` to avoid future confusion
   with IP/TCP packet loss or rosbag PDR.

## Run

- 2026-07-18, robot stationary throughout (wheels suspended/on stand), zero
  `/cmd_vel` ever published — confirmed by **four joint, independent
  checks, none of which alone is sufficient proof on its own**: (1) the bag
  contains **0** `/cmd_vel` messages at all (topic does not even appear in
  `ros2 bag info`'s topic list) — this shows nothing was recorded on that
  topic on the WSL ROS graph during the recording window, but a bag's
  silence cannot by itself prove no command was ever produced anywhere else
  in the system (e.g. before recording started, or on a process this bag
  wasn't subscribed early enough to catch); (2) the WSL-side recorder's own
  live subscription, active for the same window, independently observed 0
  nonzero `/cmd_vel` messages; (3) the Pi driver's own foreground log
  (user-confirmed via manual inspection, not independently re-verified by
  this analyzer) showed continuous `New velocity, left 0 and right 0`,
  confirming the TCP-server-to-driver path also only ever carried
  zero-velocity commands; (4) manual visual confirmation that the robot's
  wheels remained physically stationary throughout. Together these four
  cover the ROS graph, the recording, the Pi-side command log, and direct
  physical observation — no single one is treated as sufficient alone.
- Recorded via: Pi ROS 2 Foxy driver + Pi TCP server (already running,
  untouched), WSL TCP client (already running, untouched), a new
  WSL-side transport/status recorder, WSL-side and Pi-side local system
  samplers (1s interval, no new SSH connection per sample), and
  `ros2 bag record` writing to a native WSL ext4 path.
- **Main statistical window: a real, measured 240.0s window derived from the
  actual overlap of Pi CSV / WSL CSV / WSL status CSV / rosbag timestamps**
  (overlap span 294.44s: 1784377146.68–1784377441.12 Unix seconds; the
  240s window is centered within it, 1784377173.90–1784377413.90), NOT a
  fixed offset assumption from run start.

## Timestamp integrity (measured, not assumed)

| source | valid rows | monotonic | duplicates | max gap |
|---|---|---|---|---|
| wsl_transport_status.csv | 296 | yes | 0 | 1.002s |
| wsl_system_metrics.csv | 306 | yes | 0 | 1.004s |
| pi_system_metrics.csv | 501 | yes | 0 | 1.014s |

No stalls, no duplicate timestamps, no non-monotonic timestamps anywhere.

## Key results — main window (240.0s, sample_count=240 status ticks)

- `connected_fraction` = 1.0, **0 reconnects**, `crc_errors` delta = **0**.
- **RTT provenance (must be named precisely, per instruction)**: this is
  **not** a per-TCP-transaction census. `wsl_epuck_tcp_bridge.py` sends a
  command roughly every 0.1s and updates its internal `_last_rtt_ms` each
  time an incoming state message's `command_ack_seq` completes a
  round-trip (`rtt_ms = (time.monotonic() - command_sent) * 1000.0`, both
  operands from WSL's own `time.monotonic()`). `/epuck_bridge/status` is
  published once per second and reports **whichever individual
  command-ack RTT was most recently completed at that instant** — i.e.
  this recorder captured **240 one-Hz snapshots of the most recently
  completed individual RTT**, each snapshot itself a real measured value
  (not an average), but **not a complete census of every ~0.1s command
  transaction** (~2400 would have occurred over 240s; only 240 were
  sampled, ~1-in-10). Renamed here accordingly:
  **`rtt_status_snapshot_ms`** (240 valid samples, WSL-local clock only,
  single clock domain — VALID as a measurement, but as a *snapshot*
  sampling of the RTT process, not a full transaction log): mean 52.54,
  median 8.50, p95 116.58, p99 129.63, max 355.39.
  - Tail breakdown (240 valid samples): **104 samples (43.3%) > 50ms**;
    **100 samples (41.7%) > 100ms**; **1 sample (0.4%) > 200ms**. The
    tail is not smoothly spread — it clusters just above 100ms (matching
    p95/p99), with a single 355ms outlier as the max.
  - **Longest consecutive run of 1Hz snapshots with RTT>50ms: 6
    consecutive ticks (~6 seconds)**, starting near Unix time
    1784377262.12 (≈88s into the main window).
  - **No root-cause is claimed for the tail** (not attributed to Wi-Fi,
    Pi CPU load, WSL scheduling, or anything else) — only the measured
    shape is reported.
- **`state_age_s` — exact formula, fields, clock source (per
  instruction, to justify VALID)**: computed entirely inside
  `wsl_epuck_tcp_bridge.py._publish_status()` as
  `time.time() - self._last_state_time`, where `self._last_state_time`
  is set to `time.time()` inside `_record_state()` at the moment a state
  message from the Pi was successfully decoded. **Both the minuend and
  the subtrahend are `time.time()` calls made by the same WSL Python
  process** — no Pi-side timestamp is read or subtracted anywhere in this
  computation. Confirmed **VALID** (single clock domain, same machine, same
  process). Main window (240 valid samples): mean 0.056, median 0.052,
  p95 0.104, p99 0.178, max 0.485.
- `/scan`, `/odom` (from the bag, main window): 2227 messages each,
  ≈9.28 Hz average rate.
- `/epuck_bridge/status`: 240 messages, ≈1.00 Hz (matches the bridge's own
  1Hz status timer).
- 0 nonzero `/cmd_vel` in the bag's main window; 0 observed on the WSL
  topic in the main window.
- CPU/mem: WSL mean CPU 2.66%, mean mem 599.8MB used. Pi mean CPU 8.34%,
  mean mem 165.1MB used, Wi-Fi link quality mean 70/70 (max), signal mean
  ≈-28.9dBm.

Full-run figures (296 status ticks over the whole ~295s recorded span) are
consistent with the main-window figures — see `verdict.json`/`metrics.csv`
for both side by side.

## Whether RTT is "elevated" vs. the ~5–18ms historical snapshots

**Not concluded here as a simple yes/no.** The historical values were single
snapshots. This run's own median (8.5ms) sits inside that historical range;
the mean (52.5ms) and tail (p95/p99/max) are well above it because of a
real, measured heavy right tail in the RTT distribution — not because the
"typical" RTT shifted. Whether that tail itself is new, or was always there
and simply never sampled by a single snapshot, cannot be determined from
this one attempt; would need a second attempt (or the historical `RTT`
figures re-examined as distributions, which they were not) to say more.

## NOT_MEASURABLE

- `sequence_gap_count` / `duplicate_count` / `out_of_order_count` /
  `aligned_window_pdr`: the currently-running **base**
  `wsl_epuck_tcp_bridge.py` does not expose the Pi's internal per-state
  `seq` field on any WSL ROS topic or in `/epuck_bridge/status` — it is
  used only for de-duplication in `_publish_latest_state`, then discarded.
  There is no paired source sequence number to compute true PDR without
  modifying the bridge, which was explicitly out of scope this pilot.
  **`/scan`/`/odom` message counts being equal to each other is NOT
  interpreted as PDR=1** — it only shows the two topics arrived in matched
  pairs via the bridge's own re-publish-on-new-state logic, not that every
  source-side state was received.
  **Naming note for the next pilot**: the sensor-extended bridge (not used
  this pilot) implements a comparable field, which must be called
  **`APPLICATION_STATE_SEQUENCE_DELIVERY_RATIO`** — it reflects Pi
  application-level state-sequence receipt completeness at the WSL bridge,
  and must never be called IP packet loss, TCP packet loss, or
  automatically equated with rosbag PDR (a separate, bag-vs-live-source
  comparison).

## NOT_VALID

- `one_way_latency_ms` / any Pi-vs-WSL cross-clock-domain latency: no
  NTP/chrony clock-sync procedure has been run or verified between the Pi
  and the laptop/WSL. `wall_clock_delta_ms` exists purely as a clock-offset
  diagnostic and was never used to compute a one-way latency figure here.

## Known issue in this run (transparently recorded, not hidden)

`wsl_transport_recorder.py`'s **pre-fix** version was used for this attempt.
On SIGINT it hit a benign trailing `rcl_shutdown already called on the
given context` exception — the same bug class already fixed in
`sequence_counter.py` earlier this session (rclpy's own SIGINT handler can
call `shutdown()` before a Python-level `finally` block does). **All
data-critical work (CSV writes, `wsl_transport_totals.json`) completed
successfully before that trailing exception** — independently verified via
row counts (296 valid rows, matching the ~295s span at 1Hz) and the
totals.json contents (`total_scan_count=2750`, `total_odom_count=2750`,
`total_nonzero_cmd_vel_count=0`). The script was patched immediately after
(wraps the final `rclpy.shutdown()` in try/except) for future runs. This
attempt's data was not discarded or re-collected.

## Scope reminder

This diagnostic pilot says nothing about the current `EpuckState` protocol
(no converter exists yet — see `physical_protocol_gap_report.md`) and
nothing about avoidance behaviour (no controller was launched). It only
shows that the base Pi-TCP-WSL transport link stayed connected, sent zero
motor commands, and produced complete, gap-free 1Hz status/system-metric
recordings for a full 240s statistically-windowed run.

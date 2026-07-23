# Stationary physical health audit -- epuck5809, attempt02 (2026-07-23)

**Classification: `STATIONARY_PHYSICAL_DIAGNOSTIC`, `PASS`.** Not a
formal HIL navigation trial, not an `EXCLUSIONARY_HIL_PILOT`. Read-only:
no `hil_cmd_vel_guard`, no controller, no virtual peer, no
`goal_navigator`, no Webots, no rosbag, no `/cmd_vel` publisher started
at any point. Robot remained stationary throughout. Frozen git commit
at test time: `c2686b8f3da4efb2ea5f87e6915bd069cc89a445`.

This is the corrected repeat of the first attempt
(`stationary_physical_diagnostic_20260723/`, marked
`INCOMPLETE_DIAGNOSTIC`/`EXCLUDED` due to a missing-termination bug in
the recorder process). This run used the fixed orchestrator,
`tools/run_stationary_physical_diagnostic.sh`.

## Physical stack state confirmed before this run (restart after power loss)

- `wsl_epuck_tcp_bridge_sensors` PID 5477.
- `state_publisher` wrapper PID 5552 / actual PID 5553.
- `/epuck_bridge/status`: `connected=true`, `crc_errors=0`,
  `last_rtt_ms=54.53`, `state_delivery_ratio=1.0`,
  `last_state_age_s=0.034`.
- `validity_flags=7`.
- `/epuck1/state` Publisher count: 1, Subscription count: 0.
- `/cmd_vel` Publisher count: 0, Subscription count: 1.

## Bounding correctness (the actual bug fix, verified this run)

Inspected `run_stationary_physical_diagnostic.sh` before running it:
the recorder is stopped by an explicit `kill -INT` on its **exact**
PID immediately after the script's own `sleep "$DURATION_S"`, inside a
`trap ... EXIT` cleanup (both as the normal-path call and as a
safety net for any early exit). `OUT_DIR` is timestamp-generated per
run, so this attempt could not overwrite the excluded first attempt.

**Actual elapsed wall time for the whole script: 5m14.351s** (predicted
~305-320s beforehand) -- well under the 12-minute alarm threshold.
`/cmd_vel Publisher count` confirmed `0` at preflight, 2s after start,
and at the end (`cmd_vel_checkpoints.json`: `publisher_count: 0` at all
three of start/mid/end checkpoints).

## Findings (full ~300s window, all collectors ran concurrently)

**Sensor topic rates** (`/odom`, `/scan`, `/tof`, `/ps0`-`/ps7`, each
measured over the complete window, not a short snapshot): all
converged to **8.915-8.917 Hz**, matching each other and matching the
first attempt's 8.929 Hz almost exactly. This independently
re-confirms (second run, same conclusion): the isolated `/ps0 ~5.055 Hz`
reading from the very first short-window physical preflight check was
a measurement artifact, not a real per-topic rate difference.

**Bridge status** (310 CSV rows, ~300s): `connected=true` for all 310
rows; `crc_errors=0` (max); `state_missing=0` (max);
`state_out_of_order=0` (max); `state_delivery_ratio=1.0` for all 310
rows. `rx_count` grew from 1388 to 4152 (+2764 over ~300s, ~9.2/s,
consistent with the ~9 Hz EpuckState rate). `last_rtt_ms` ranged
**5.60-164.89 ms**. `last_state_age_s` ranged 0.0022-1.2022s (the
1.20s maximum is a single elevated sample, consistent with the
periodic dropout noted below, not a sustained problem).

**Latency semantics (per instruction, kept separate)**: `last_rtt_ms`
(5.60-164.89 ms) is the reported figure. `estimated_one_way_ms`
(2.80-82.45 ms, an RTT/2-style bridge-side estimate) is recorded for
completeness but is **not** reported or treated as a measured true
one-way latency -- no NTP/chrony clock-sync procedure between the Pi
and WSL/laptop has been run or verified, so a genuine one-way figure
cannot be computed from this data.

**`validity_flags`** (full window, all real samples classified):
2676 of 2687 real messages read exactly `7`; **11 read `0`**. Their
line positions (61, 643, 1223, 1803, 2383, 2965, 3551, 4135, 4719,
5299, 5301) are spaced almost exactly ~580-586 lines apart (~32-33s at
the observed rate) -- **this is the same recurring pattern found in
the first (excluded) attempt** (there: 15 occurrences at nearly
identical spacing). Seeing the same ~32s periodic single-sample
dropout survive an independent, correctly-bounded re-run makes this a
genuinely reproducible finding, not a fluke of the earlier buggy run.
Root cause is not diagnosed here (would need correlation against the
bridge CSV's own per-second counters at the matching offsets, and
possibly Pi-side driver/timer logs) -- flagged for dedicated follow-up.

## Evidence

Raw logs preserved on the native WSL filesystem
(`/home/eamon/epuck_comm_bags/stationary_physical_diagnostic_20260723_135505/`),
SHA-256-verified identical to the local (gitignored) Windows copy in
`raw_logs/` (manifest: `raw_logs/SHA256SUMS.txt`), copies never
overwritten. Only this `SUMMARY.md` is tracked in git; the raw
`.log`/`.csv`/`.json` files and the SHA-256 manifest stay local per
project evidence-handling convention.

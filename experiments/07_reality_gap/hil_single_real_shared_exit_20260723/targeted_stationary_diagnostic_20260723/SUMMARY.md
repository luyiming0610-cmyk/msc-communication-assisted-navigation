# Targeted validity_flags periodicity diagnostic -- epuck5809 (2026-07-23)

**Classification: `TARGETED_STATIONARY_DIAGNOSTIC`.** Not a formal HIL
navigation trial. Read-only: no `hil_cmd_vel_guard`, controller,
virtual peer, `goal_navigator`, Webots, rosbag, or `/cmd_vel` publisher
started at any point. Robot remained stationary throughout. Frozen git
commit at test time: `de9aecb91ad5fb4ab53baccace18c164a2f3f251`.

## Correction to the prior audit's wording

`validity_flags=0` proves that the ODOM, IR, and TOF validity
conditions were all false in the *same* `state_publisher` snapshot. It
does **not** by itself prove that an upstream scheduling race/stall is
impossible as an alternative explanation, nor did it previously rule
one in with certainty. This run supplies the first millisecond-precision,
per-topic evidence that distinguishes between those possibilities (see
below) -- the underlying root cause is narrowed but still not
completely closed out.

## Recorder and method

`hil_targeted_validity_diagnostic_recorder.py` (new, committed
`de9aecb`): single lightweight process, 9 concurrent subscriptions
(`/epuck1/state`, `/epuck_bridge/status`, `/odom`, `/tof`, `/ps0`,
`/ps1`, `/ps2`, `/ps5`, `/ps6`, `/ps7`), zero publishers. Every
callback appends one small in-memory dict (`local_time_ns`,
`local_monotonic_ns`, per-message stamp, `validity_flags`/`sequence`
for `/epuck1/state`, bridge JSON fields for `/epuck_bridge/status`);
all CSV writing happens once, at shutdown. Ran for 120s
(self-bounded internally, 9839 rows collected, confirmed via
`recorder.log`), `/cmd_vel Publisher count: 0` before, 2s in, and after.

## Result: 4 flags=0 events captured, all four fully explained by a common gap across every physical-input topic

Events at elapsed **16.138s, 49.285s, 82.346s, 115.459s** (intervals:
33.15s, 33.06s, 33.11s -- essentially the same ~32.4-33.2s period found
in both prior broader audits, now pinned to millisecond precision).

**For every one of the 4 events, without exception**: `/odom`, `/tof`,
`/ps0`, `/ps1`, `/ps2`, `/ps5`, `/ps6`, and `/ps7` all show a
gap-before-arrival of **~0.917-1.033 seconds** (roughly 8-9x their
normal ~0.11-0.12s inter-arrival interval), landing within a few
milliseconds of each other and of the `/epuck1/state` zero-flags
sample itself. This directly answers the required question:

- **A common gap across all physical input topics**: **YES, confirmed
  exactly**, on every event.
- **Only a state_publisher output-timing issue**: **NO** -- the gap is
  visible at the raw topic-arrival level itself (before
  `state_publisher` even sees the data), not only in its published
  output.
- **Bridge `rx_count` stagnation / `state_age` increase**: **partially**
  -- 2 of the 4 events (elapsed 16.1s and 115.5s) show the bridge's own
  `last_state_age_s` also elevated (0.906s and 0.960s), meaning the raw
  TCP "state" message reception itself was delayed at those moments.
  The other 2 events (49.3s and 82.3s) show a *normal* `last_state_age_s`
  (0.014s and 0.011s) despite the identical ~1s gap on every ROS-side
  topic -- meaning the TCP data had already arrived on schedule, but
  the bridge's own `_publish_latest_state()` ROS timer (which
  republishes odom + all range sensors from one shared payload, once,
  every 0.02s -- confirmed by reading `wsl_epuck_tcp_bridge_sensors.py`
  in full) did not fire on time.
- **RTT increase without input-topic starvation**: **NOT the pattern
  observed here** -- RTT at these 4 exact events (69.8, 57.1, 60.9,
  61.4 ms) is unremarkable, and all 4 events DO show input-topic
  starvation. (The earlier, coarser audit's apparent RTT correlation
  was based on ±1s nearest-row alignment against a 1Hz bridge-status
  sample and is superseded by this precise result -- it was likely
  coincidental proximity, not a real distinguishing signal.)

## Refined (not yet fully closed) conclusion

Two of four events show the stall reaching all the way back into the
bridge's TCP-receive bookkeeping; the other two show it isolated to the
bridge's own ROS-side republish timer while TCP reception stayed on
schedule. Both sub-patterns point at the same process --
`wsl_epuck_tcp_bridge_sensors.py` -- since it is the single component
whose one function, on one 0.02s timer, republishes every one of the
affected topics from one shared payload (confirmed by source: `odom`,
`/scan`, and every range sensor are all published inside
`_publish_latest_state()` in one call). A periodic (~33s), roughly
0.9-1.0s pause **somewhere in that one WSL process** -- consistent with
a CPython garbage-collection cycle, WSL2/host-OS scheduling
contention, or another periodic background load on this machine -- is
the best-supported hypothesis. It is **not proven**: no internal
instrumentation of the bridge process's own GC/thread scheduling has
been captured, and no matching ~30-33s constant exists in
`state_publisher.py`, `wsl_epuck_tcp_bridge_sensors.py`, or
`pi_epuck_tcp_server_sensors.py` (all three read in full this session).
No code defect has been found in any of them -- the validity-flag
logic in `state_publisher.py` continues to behave exactly as designed,
correctly reporting the genuine upstream gap it observes.

## Actions explicitly not taken (per instruction)

No freshness threshold changed. No sample hidden or dropped from this
report. No guard weakened. No ground motion attempted, at any point.

## Evidence

Raw output preserved on the native WSL filesystem
(`/home/eamon/epuck_comm_bags/targeted_stationary_diagnostic_20260723_141802/`),
SHA-256-verified identical to the local (gitignored) Windows copy in
`raw_logs/` (manifest: `raw_logs/SHA256SUMS.txt`), copies never
overwritten. Only this `SUMMARY.md` is tracked in git.

# Webots/Wall-clock Timing Integrity Audit

Date: 2026-07-16

## Finding

The existing controller uses ROS simulation time for node timers but also uses
Python `time.monotonic()` for maximum runtime, message freshness and command
smoothing. Consequently, changes in Webots simulation speed relative to wall time
can change task termination and time-dependent command behaviour.

This was discovered during scripted repeated execution of the lateral-offset
condition: a controlled-realtime run reached the 30 s wall-clock ceiling before
recovery, whereas accelerated-factor runs completed because Webots simulation time
advanced multiple times faster than wall time.

## Measured non-controlled accelerated factors

Operator identity is not an experimental factor and is not recorded in the
grouping or statistics. Runs are classified only by scenario, protocol validity
and the measured simulation-time/recorded-time factor.

Centred head-on post-fix Trials 02–06:

| Trial | Simulation-time / recorded-time factor |
|---|---:|
| 02 | 2.179326 |
| 03 | 3.636728 |
| 04 | 4.097455 |
| 05 | 2.830962 |
| 06 | 3.483485 |

Lateral-offset setup runs:

| Run | Factor |
|---|---:|
| Pilot 01 | 4.226172 |
| Manual formal Trial 01 | 2.562016 |

These runs remain valid functional evidence for collision avoidance, recovery and
automatic zero command, but they must not be pooled with controlled-realtime runs
for wall-time, smoothing or cross-condition timing comparisons.

## Corrective experimental protocol

1. Measure `/clock` advancement relative to `time.monotonic()` before starting
   state publishers.
2. Repeat the measurement after rosbag and both controllers are running.
3. Accept only runs with factors in the locked 0.8–1.2 interval.
4. Start every repetition in a fresh WSL/Webots/ROS graph; do not reuse a session.
5. Use `max_runtime_s:=60.0` only as a safety ceiling and require both robots to
   end via `cooperative recovery completed`.
6. Preserve failed protocol-development runs as excluded diagnostics.

## Controlled lateral-offset result

The five accepted formal runs had pre-load factors 0.948–0.998 and full-load
factors 0.947–0.989. All five completed recovery before the safety ceiling, with
no collision and no invalid communicated state.

## Controlled centred result

The five accepted centred head-on runs had a bag-derived state-time factor of
0.959959 ± 0.004386 (range 0.955134–0.966062). Trial 01 was also directly observed
at 0.95–1.03 on the Webots display. Trials 02–05 passed explicit pre-load factors
of 0.955–1.032 and full-load factors of 0.947–1.003. This batch is therefore
timing-compatible with the controlled lateral-offset batch.

## Controlled ninety-degree crossing result

The five accepted crossing-path runs had a bag-derived state-time factor of
0.962222 ± 0.003969 (range 0.955955–0.966343). Trial 01 was directly observed at
0.92–1.08 on the Webots display. Trials 02–05 passed explicit pre-load factors of
0.953–1.054 and full-load factors of 0.979–1.045. This batch is timing-compatible
with the controlled centred and lateral-offset batches.

## Controlled local-sensing-only ablation result

The five accepted local-only head-on repetitions had a bag-derived state-time
factor of 0.955668 ± 0.004339 (range 0.949473–0.961033). Trial 01 was directly
observed at approximately 0.92–1.03 on the Webots display. Trials 02–05 passed
explicit pre-load factors of 0.958–1.064 and full-load factors of 0.932–1.049.
The local-only batch is therefore timing-compatible with the matched
communication/CPA baseline.

## Controlled fused-ablation result

The five accepted fused head-on repetitions had a bag-derived state-time factor
of 0.962805 ± 0.000626 (range 0.961980–0.963551). Trial 01 was observed with a
normal Webots display rate; its two bag-derived robot factors were 0.961883 and
0.962847. Trials 02–05 passed explicit pre-load factors of 0.958–0.999 and
full-load factors of 1.004–1.079. The fused batch is therefore timing-compatible
with both matched ablation conditions.

## Required treatment in the dissertation

- Label the earlier centred head-on batch as functional/reproducibility evidence,
  not a strictly rate-controlled timing comparison.
- Use the controlled-realtime centred and lateral-offset batches for quantitative
  timing, repeatability and controlled geometry-comparison statements.
- Apply the same dual rate gate and fresh-session protocol to all later 90-degree,
  ablation, impairment and fusion batches.
- The controlled centred rerun is complete; do not pool the earlier accelerated
  centred runs into this timing comparison.

## Engineering follow-up

For a future controller revision, replace wall-clock-dependent control timing with
one consistent clock abstraction that follows ROS time in simulation and system
time on hardware. Such a code change would define a new controller version and
would require new post-change batches; it was not introduced mid-batch here.

## 2026-07-17: follow-up implemented (controller_v4_ros_time_consistency)

The above follow-up is now implemented. `cooperative_avoider.py`'s `_now_s()`
reads exclusively from `self.get_clock().now()` (the ROS node clock, which
follows Webots simulation time under `use_sim_time:=true`), and every timer
that previously used `time.monotonic()` -- `max_runtime_s`, message
freshness, command smoothing, `startup_hold_s`, and the local-obstacle
encounter/turn-ledger state machine's own timers -- now derives from it.
`time.monotonic()`/`time.time()` are reserved for the external shell-script
watchdog and confirmed diagnostic-only log fields.

This directly explains a previously observed discrepancy: a rigorous offline
replay of recorded bag data through the real, unmodified state machine showed
several `controller_v1`-era encounters should have completed successfully,
yet the live runs hit `FAILSAFE`. Root cause was the wall-clock/sim-time
mismatch this audit originally flagged. Post-fix pilot evidence (`pilot_v4_b2`
then `pilot_v4_b3`, `experiments/3-3.全传感器避障实验/`)
confirmed zero FAILSAFE on the same scenario that previously failed reliably
at the same PASS_CONFIRM-then-FAILSAFE pattern.

Realtime factors measured for the three v4 exclusionary pilots (dual pre-load/
full-load gate, same protocol as this audit's controlled-realtime runs):

| Pilot | Preload factor | Full-load factor |
|---|---:|---:|
| `pilot_v4_b3` | 0.956 | 0.970 |
| `pilot_v4_c` | 0.956 | 0.950 |
| combined box+peer | 1.004 | 0.912 |

All six values are within the locked 0.8-1.2 interval.

### `self.started_at` initialization race — FIXED (controller_v4_timebase_fix_20260717)

The finding below is now fixed (git commit `980e7d0`, re-validated by a
fresh combined-pilot regression at `06e0f0f`). Left in place for context.

While reconciling `pilot_v4_b3`'s stop reason, the controller's own internal
`elapsed >= max_runtime_s` check was observed to fire at `ros_time=70.000`
exactly (matching the configured ceiling exactly), while the earliest real
`TRANSITION` log line in the same run was at `ros_time=16.260` -- roughly 16s
of sim time had already elapsed before the controller's first observed
activity. This suggests `self.started_at = self._now_s()` may be captured
before the node's ROS-time clock subscription has received its first
`/clock` message (a known rclpy race, where `Clock.now()` can return `0`
until the first `/clock` sample arrives), making `max_runtime_s` and
`startup_hold_s` fire earlier, in absolute sim-time terms, than the
node-construction instant they are meant to be measured from. This did not
affect `pilot_v4_b3`'s PASS verdict (which was earned via external,
ground-truth x-position and mode monitoring, not this internal timer), and
has not been fixed or further diagnosed -- flagged here for a future revision.

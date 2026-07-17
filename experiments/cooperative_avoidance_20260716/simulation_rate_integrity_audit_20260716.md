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

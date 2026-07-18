# Condition B Trial 01 startup-timing synchronization audit (offline, no Webots run)

Answers the user's real-time observation: avoidance itself was
synchronized, but epuck1 visibly began moving before epuck2. This
audit reads only `controller.log` + the preserved bag for Condition B
Trial 01 and all 5 Condition A trials -- no Webots run, no
controller/relay/matrix-parameter change.

## Method

`controller.log`'s `TRANSITION` lines carry both a wall-clock epoch
(matching the bracketed rclpy logger prefix) and the ROS/sim clock
(`ros_time=`) for the same instant. A per-robot linear fit
(`ros_time = a*wall_epoch + b`, mean-centered least squares to avoid
the catastrophic-cancellation numerical error a naive unshifted fit
produced on the first attempt -- confirmed and fixed before any result
was trusted) maps bag-record wall-clock timestamps (`cmd_vel` has no
own stamp field) onto the same sim-time axis. `EpuckState` messages
carry their own `stamp` field (already sim time) -- used directly for
"first sustained real motion," no mapping needed. "Sustained nonzero" =
3 consecutive samples with `|value| > 0.005` (`linear.x` for
`cmd_vel`, `linear_velocity_mps` for real motion), per instruction.

## Condition B Trial 01: epuck1 &minus; epuck2 deltas (seconds)

| event | delta (s) |
|---|---|
| TIMEBASE_INIT | **&minus;2.64** |
| first valid own state received | &minus;0.0074 |
| first valid peer state received | &minus;0.0046 |
| STARTUP_HOLD start | &minus;2.50 |
| STARTUP_HOLD end | &minus;2.64 |
| first CRUISE | &minus;2.64 |
| first sustained nonzero cmd_vel | &minus;2.634 |
| first sustained real motion | &minus;2.66 |
| **first AVOID_TURN** | **0.000** |
| **first RECOVER** | **0.000** |

Negative = epuck1 earlier. epuck1 leads epuck2 by ~2.5-2.66s through
*every* startup-phase event, but the actual encounter response
(`AVOID_TURN`, `RECOVER`) is synchronized to the millisecond
(`delta=0.000s` exactly). "First valid own/peer state received"
deltas are near-zero (~5-7ms) -- both robots' `state_publisher`
messages arrive at essentially the same sim time, well before either
controller's own `TIMEBASE_INIT`. **The asynchrony is not a
communication/state-delivery effect; it originates entirely in when
each robot's own controller process reaches its first valid clock
tick.**

## Condition A comparison table (same method, all 5 trials)

| trial | TIMEBASE_INIT delta (s) | motion start delta (s) | CRUISE entry delta (s) | AVOID_TURN delta | RECOVER delta |
|---|---|---|---|---|---|
| 01 | &minus;0.92 | &minus;0.92 | &minus;0.90 | 0.000 | 0.000 |
| 02 | &minus;1.02 | &minus;1.06 | &minus;1.00 | 0.000 | 0.000 |
| 03 | &minus;0.46 | &minus;0.34 | &minus;0.46 | 0.000 | 0.000 |
| 04 | **+2.66** | **+2.66** | **+2.64** | 0.000 | 0.000 |
| 05 | &minus;0.08 | &minus;0.12 | &minus;0.06 | 0.000 | 0.000 |

## Cross-trial analysis

- **Startup offset already present in Condition A**: yes, in every
  trial, magnitude 0.06s-2.7s.
- **Always epuck1 first?** No. epuck1 leads in Trials 01/02/03/05
  (negative delta); **epuck2 leads in Trial 04** (`+2.66s` --
  numerically almost identical magnitude to Condition B Trial 01's
  `-2.64s`, just opposite sign). The sign is not fixed.
- **Close to one publish period (~0.1151s)?** Only Trial 05
  (0.06-0.12s). Trials 01-04 range 0.34s-2.7s -- 3x to 23x the publish
  period, not explained by publish-period-scale jitter alone.
- **Does Condition B Trial 01 exceed Condition A's distribution?**
  **No.** Its magnitude (~2.5-2.66s) falls squarely within Condition
  A's own observed range, and matches Trial 04 almost exactly.
- **Avoidance-phase synchronization**: `AVOID_TURN` and `RECOVER`
  deltas are exactly `0.000s` in **every** audited trial (all 5
  Condition A trials plus Condition B Trial 01), independent of the
  startup-phase offset's magnitude or direction.

## Root-cause read-only check

- **Launch structure**: both `cooperative_avoider` instances are
  declared in the **same** `launch.LaunchDescription`/`LaunchService`
  call (`run_comm_baseline_formal_controllers.py` lines 42-48) -- not
  two separate sequential launches.
- **`startup_hold_s` identical**: yes, both robots use
  `cooperative_avoider.py`'s identical declared default (5.0s); not
  overridden per-robot in the launch config.
- **`TIMEBASE_INIT` independence**: fires independently per node on
  that node's own first control-timer tick with a valid clock
  (`_ensure_timebase()`) -- not a shared/synchronized event, not
  gated on peer-state receipt.
- **Relay delay's effect on `STARTUP_HOLD` exit**: none by design --
  exit is gated purely by a fixed-duration timer since that node's own
  `started_at`, not by peer-state freshness at the exit instant. The
  0.20s relay delay cannot mechanically cause this offset through that
  path.
- **Orchestrator sequencing**: relay+counter and `state_publisher` are
  launched well before the controller in every trial (both
  conditions) -- this explains the ~7-11s lag between first
  state-message arrival and `TIMEBASE_INIT` (a known, designed
  launch-ordering artifact), but not the epuck1-vs-epuck2 *relative*
  asymmetry, since both controllers launch together.

**Conclusion**: the offset is consistent with ordinary OS-level
process-spawn/scheduling variance for two `ExecuteProcess` actions in
the same `LaunchDescription` (Python interpreter startup, `rclpy`
init, parameter-file loading) under the WSL2/Webots-shared-CPU
environment -- not a deterministic code-level ordering bug, and not a
relay-delay-driven effect.

## Classification against the decision rules

| rule | applies? |
|---|---|
| ~1 publish period, consistent A&B | No -- most trials exceed one publish period by 3-23x |
| large but stably identical across A/B | No -- sign flips within Condition A itself (Trial 04) |
| B exceeds A's distribution, or B-only | No -- B01 falls within A's own range, matches Trial 04 |
| **orchestrator instability / launch race** | **Yes** |

**Final classification: `ORCHESTRATOR_LAUNCH_TIMING_VARIANCE`** --
pre-existing in Condition A, reproduced (not amplified) in Condition B
Trial 01. Not a communication-impairment confound. Not evidence
against this trial's PASS. Avoidance-phase synchronization is
unaffected in every audited trial.

## Fix-impact assessment (no code change made or recommended here)

- **Current impact on measured communication metrics**: none
  identified. `message_age_s`, `capture_ratio`, drop counts,
  `min_interrobot_distance_m`, `complete_count`/`TASK_OUTCOME` are all
  measured relative to each trial's own encounter phase, which is
  synchronized to `0.000s` regardless of the startup offset. The
  offset shifts *when* the encounter begins in absolute `ros_time`,
  not the encounter's relative dynamics or the relay's delay/jitter/
  drop behavior (which acts on message content, not on either robot's
  `STARTUP_HOLD` timer progress).
- **Potential future risk -- KNOWN CONFOUND, must be checked before
  formal Condition F begins**: Condition F's periodic outage is
  anchored to absolute elapsed `ros_time` (shared `/clock`), not to
  either robot's own `started_at`. *(Corrected: an earlier revision of
  this bullet reasoned "the 0.06-2.7s range is small relative to
  `outage_period_s=15.0s`, so not expected to be a practical problem"
  -- that inference was rejected by the user as over-reach and is
  retracted.)* The relevant comparison is against the **0.7s outage
  duration**, not the 15.0s period: the observed startup offset (up to
  ~2.66s) is roughly 3.8x the outage duration, so it can materially
  shift where each trial's `TIMEBASE_INIT`/`CRUISE`/`AVOID_TURN` phase
  falls relative to the fixed global outage windows (elapsed
  10/25/40/55s) -- including whether an outage window overlaps the
  critical CPA decision period at all. **Before formal Condition F
  Trial 01, each trial's actually-experienced outage windows must be
  reconstructed against that trial's own
  `TIMEBASE_INIT`/`CRUISE`/`AVOID_TURN` times** -- proving which
  windows each trial actually hit and whether any covered the CPA
  decision phase -- not assumed from the configured schedule alone.
  Recorded as a Condition-F precondition in `project_status.json`; no
  change is made in the current Condition B phase.
- **Recommended action**: not a code-change recommendation (out of
  scope for this audit by instruction). The user should decide whether
  to (a) accept this as known, pre-existing, non-impairment-related
  launch variance and proceed with Trials 02-05, or (b) request a
  dedicated, separate, explicitly-authorized investigation into
  reducing launch-time variance (e.g. explicit startup barriers)
  before continuing.

## Stop point

Per instruction, this audit does not itself authorize Trial 02. It
classifies the observed startup asynchrony as
`ORCHESTRATOR_LAUNCH_TIMING_VARIANCE`, present identically (in
magnitude range, though not in sign) in Condition A, and not a
communication-impairment confound. Whether this is sufficient grounds
to proceed with Trial 02 is the user's decision.

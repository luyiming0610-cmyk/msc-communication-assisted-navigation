# cooperative_exit_n2_comm_off_EXCLUSIONARY_PILOT02 -- FAILED, real structural scenario error (preserved, not deleted)

**EXCLUSIONARY_DIAGNOSTIC. Not counted toward any formal or pilot statistic.**

## What happened

`DATA_VALIDITY=VALID` (bag clean, no crash, `complete_count=2`), but this
was a misleadingly "clean" run: both robots' `controller.log` shows
exactly one transition, `SAFE_STOP_STALE -> COMPLETE` (via
`ended_by_max_runtime_hits=2`, i.e. "maximum runtime reached" -- NOT a
genuine recovery completion). Trajectory data confirms **neither robot
ever moved**: `final_positions` are identical to the initial poses
(`epuck1 (-0.35, 0.0)`, `epuck2 (0.35, 0.0)`), `path_length_m=0.0` for
both, `stop_duration_s` ≈ the entire trial duration (~34s of ~38s).

## Root cause (confirmed by reading `network_impairment_relay.py`'s own
module docstring, not guessed)

The relay bridges `/epuckN/state_raw -> /epuckN/state` **within the same
robot's own namespace** -- it is not cross-robot forwarding. Each robot's
`cooperative_avoider` subscribes to the bare relative topic `"state"`
(`== /epuckN/state`) as its OWN state; the OTHER robot's controller
separately subscribes to that SAME absolute topic as its PEER state via
`peer_state_topic`. In other words, `/epuckN/state` is the one shared
output both the owning robot's own-state subscription and the other
robot's peer-state subscription read from -- the relay is required
plumbing even for a robot's own odometry to reach its own controller, not
optional "peer-only" wiring.

`run_cooperative_exit_n2_trial.sh`'s original COMM_OFF branch launched
`state_publisher` with `-r state:=state_raw` (remapping its output away
from the bare `"state"` topic) and deliberately launched NO relay ("no
cross-robot state topic will exist"). This left the bare `/epuckN/state`
topic with **no publisher at all** -- `cooperative_avoider`'s own-state
subscription never received a single message, `own_received` stayed
`None` forever, `_fresh(self.own_received, now)` was always `False`, and
the controller latched in `SAFE_STOP_STALE` for the entire trial before
completing via `max_runtime_s` (an outcome my own
`build_task_verdict(..., ended_by_max_runtime=True)` correctly refused to
call a success: `TASK_OUTCOME=TASK_FAILURE`).

## This was caught by the required verification step, not skipped

Per instruction ("若数据有效且没有结构性场景错误，再运行N2_COMM_ON
pilot01"), COMM_ON was NOT attempted until this structural error was
found and understood. This is exactly the kind of structural scenario
error that check is designed to catch.

## Fix

`run_cooperative_exit_n2_trial.sh`'s COMM_OFF branch no longer remaps
`state:=state_raw` and launches NO relay at all -- `state_publisher`
publishes directly to the un-remapped `/epuckN/state` topic, which is a
strictly STRONGER form of "no communication" than A-D's zero-impairment
relay pass-through (no relay process exists in COMM_OFF at all, not
merely a no-op one). Bag topic list updated accordingly (`/epuckN/state`
instead of `/epuckN/state_raw` for COMM_OFF, since there is no
raw-vs-relayed distinction when no relay runs). See the orchestrator's
git history for the exact diff.

## Disposition

- Native WSL bag + diag_logs preserved at
  `/home/eamon/epuck_comm_bags/cooperative_exit_n2_comm_off_EXCLUSIONARY_PILOT02`
  (+ `_diag_logs`) and a SHA-256-verified Windows copy under this
  directory's sibling `bags/` path (gitignored).
- Not rerun under this same name -- the corrected retry uses
  `EXCLUSIONARY_PILOT03`.
- `DATA_VALIDITY=VALID`, `TASK_OUTCOME=TASK_FAILURE` (ended via
  `max_runtime_s`, zero motion) is the genuine, correctly-computed result
  for this specific (broken) launch configuration -- preserved as-is.

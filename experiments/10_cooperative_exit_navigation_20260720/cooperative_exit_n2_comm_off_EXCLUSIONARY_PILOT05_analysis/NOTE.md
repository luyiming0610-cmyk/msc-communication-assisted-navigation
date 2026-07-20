# cooperative_exit_n2_comm_off_EXCLUSIONARY_PILOT05 -- FAILED, cmd_vel-verification check correctly caught an unverified stop (preserved, not deleted)

**EXCLUSIONARY_DIAGNOSTIC. Not counted toward any formal or pilot statistic.**

## What happened

The goal-region fix from PILOT04 worked as intended: this trial shows
genuine motion. `controller.log` shows both robots steadily in `CRUISE`
at `cmd=(0.025, ~0.000)`, `local` distance to the goal shrinking
monotonically (epuck1: 0.535m -> 0.204m; epuck2 mirrored), over about 17
seconds of sim time. `task_completion_monitor` reported
`TASK_COMPLETE_GOAL` at a plausible time, not instantly. Both controller
processes exited cleanly (`process has finished cleanly`, no crash, no
Traceback).

But the new `verify_cmd_vel_zero.py` check (added this round precisely
to enforce "verify /cmd_vel reaches zero after task completion") found
the LAST recorded `/epuckN/cmd_vel` sample for BOTH robots was still
`linear.x=0.025` -- the pre-SIGINT steady-state CRUISE command, not
zero. The orchestrator correctly set `DATA_VALIDITY=INVALID` and halted
before any further pilot ran.

## Root cause (confirmed by reading controller.log timing and
cooperative_avoider.py's own stop()/main() code, not guessed)

`cooperative_avoider.py`'s frozen `stop()` method publishes a zero
`Twist` 3 times (with a 30ms gap) when SIGINT/KeyboardInterrupt is
caught, then `main()`'s `finally` block immediately calls
`destroy_node()` and exits. The controller's OWN code comment already
acknowledges this exact scenario:

> "During ros2 launch SIGINT handling the shared context may be
> invalidated between rclpy.ok() and publish(). Motion is also bounded
> by the driver watchdog, so cleanup must remain quiet."

i.e. the frozen design does not guarantee the zero-Twist publishes are
actually delivered before the publishing node's DDS participant is torn
down moments later -- it deliberately relies on a lower-level driver/
watchdog safety net instead of a guaranteed final zero /cmd_vel message.
This was never previously observable because every PRIOR pilot (before
this round's task_completion_monitor) only ever stopped via the SAME
SIGINT path, either at `max_runtime_s` or via a controller self-
completion -- the raw `/epuckN/cmd_vel` topic's literal last sample was
simply never checked before `verify_cmd_vel_zero.py` was added this
round. This is not a new bug introduced by the monitor/orchestrator
changes; it is a pre-existing characteristic of the frozen controller's
shutdown design, newly exposed by a newly-added check.

## Fix

Checking the raw `/epuckN/cmd_vel` topic's last sample is not a
reliable safety signal for this shutdown path (nothing publishes to it
once the controller process has exited, so no amount of extra recording
time can recover a lost sample). Replaced with a more robust,
physically meaningful check:
`verify_state_velocity_settled.py` checks each robot's LAST recorded
`/epuckN/state.linear_velocity_mps` (Webots' own physical odometry,
published by the separate, still-running `state_publisher` process) is
near zero. The orchestrator now inserts a 1.0s settle window between
stopping the controller and stopping `state_publisher`, so the bag
captures the robot's actual post-stop deceleration. `verify_cmd_vel_
zero.py`'s check is kept and logged but demoted to informational-only,
no longer a `DATA_VALIDITY` gate.

## Disposition

- Native WSL bag + diag_logs preserved at
  `/home/eamon/epuck_comm_bags/cooperative_exit_n2_comm_off_EXCLUSIONARY_PILOT05`
  (+ `_diag_logs`) and a SHA-256-verified Windows copy under this
  directory's sibling `bags/` path (gitignored).
- Not rerun under this same name -- the corrected retry uses
  `EXCLUSIONARY_PILOT06`.
- The trial's own `DATA_VALIDITY=INVALID` verdict is genuinely correct
  for the check as it existed at the time -- exactly what the check was
  built to catch. Preserved as-is.

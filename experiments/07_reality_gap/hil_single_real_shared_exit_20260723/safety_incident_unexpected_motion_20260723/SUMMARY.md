# Safety incident: unexpected physical motion (2026-07-23)

**Classification: `UNEXPECTED_PHYSICAL_MOTION` / `SAFETY_INCIDENT_DIAGNOSTIC`.**
Reported by the user during ongoing offline HIL software/configuration
work. This document is the permanent, read-only record of the incident
and its audit. **The root cause is not solved and is not claimed
solved.** No code was changed until this audit was complete; the one
fix made as a direct result (topic-isolation of the offline integration
test, below) is recorded separately and is not claimed to explain the
incident.

## Reported facts (from the user, verbatim substance)

- Date: 2026-07-23. Estimated window: **15:15-15:35 BST** (exact minute
  not identifiable).
- Robot state: **on the ground, wheels bearing weight** (not suspended).
- Direction: mixed/unclear.
- Duration: brief, approximately less than one second; exact duration
  unknown.
- The safety report itself was raised at 15:38:25 BST, after the motion
  had already been observed.
- The user stated the same kind of brief movement had also been
  observed on earlier occasions.

## Exact reconstructed timeline, 15:15-15:38 BST

Built from this session's own command-execution history (verified
against the literal shell commands issued, not from memory) cross-checked
against `.ros/log` node output on the WSL host.

| Time (BST) | Action | Real-topic contact |
|---|---|---|
| 15:15-15:20 | Syntax checks; unit tests on shutdown script (fake/nonexistent PIDs); frozen-params-to-env converter; orchestrator edits | None -- no ROS processes started |
| 15:24:21 | Validate `hil_frozen_params.json` after edits | None |
| 15:24:40 | `run_hil_shared_exit_trial.sh --check-only` | Read-only report only |
| 15:24:51 | `run_hil_shared_exit_trial.sh --dry-run` | Print-only, no processes |
| 15:25:03 | `run_hil_shared_exit_trial.sh --comm-off` | **Refused**: `PHYSICAL_MOTION_LOCKED_UNTIL_LAB_VALIDATION` (captured stdout), 0 processes started |
| 15:25:58 | Re-run `--comm-off` (after an unrelated preflight-script fix) | Same refusal, same captured evidence |
| 15:26:28 | `run_hil_shared_exit_trial.sh --comm-on` | Same refusal; `pgrep -af 'hil_cmd_vel_guard\|cooperative_avoider\|hil_virtual_peer\|hil_topic_adapter'` captured in the same call confirmed none of those processes existed afterward |
| 15:28:12 | Syntax-check `test_hil_integration_offline.sh` | None |
| 15:28:26 | 1st run of `test_hil_integration_offline.sh` | Failed at a `set -u`/ROS-sourcing ordering bug before any child process started -- no `.ros/log` entry exists for this attempt |
| 15:28:48 | 2nd run (after the `set -u` fix) | Produced `hil_cmd_vel_guard` (PID 10684) and `hil_wheel_suspension_test` (PID 10714) at 15:29:03-10 BST. Both confirmed, via read `.ros/log` content **and** the literal invoking shell command (no CLI override), bound exclusively to `/hil_offline_test/...` topics |
| 15:29:38 | 3rd run (after a test-assertion-timing fix) | Same pattern: `hil_cmd_vel_guard` (PID 10894) and `hil_wheel_suspension_test` (PID 10925) at 15:29:51-58 BST, same test-namespaced topics only |
| 15:30:57-15:33:32 | Full HIL unit suite; colcon suite; `py_compile`; `bash -n`; `git status`/stage/commit; doc update | None -- offline tests and git operations only |
| **15:33:32 -> 15:37:26** | **No command issued in this session; no new `.ros/log` entry created in this gap** | -- |
| 15:37:26 | Read-only process/topic snapshot (start of this audit, after the user's report) | Read-only |
| 15:38:25 | User's safety-question screenshot | -- |

Throughout this entire window, `state_publisher` (PID 5552/5553) and the
WSL bridge (PID 7961) ran continuously with **no reconnect event**
logged since the bridge's last restart at 14:44:38 BST (confirmed from
two independent copies of its log) -- this rules out a
reconnect-triggered stale-command replay in this specific window.

## Command origin: NOT_MEASURABLE

No rosbag or other continuous recording of the real `/cmd_vel` or
`/cmd_vel_unguarded` existed during this window. Per the audit
requirement that a Publisher-count=0 snapshot cannot rule out a
transient publisher between samples, and that command origin must not
be asserted as "no command occurred" absent continuous evidence:
**command origin for this incident is recorded as NOT_MEASURABLE.**

What the evidence above does establish, with direct corroboration
(captured stdout/log content, not recollection):

- Every command issued in this session that could reach a real topic
  (`--comm-off`, `--comm-on`) is proven, by its own captured output, to
  have refused via the safety gate and started zero processes.
- The only processes that did start in this window (the offline
  integration test's guard/wheel-test pairs, twice) are proven, by
  `.ros/log` content plus the exact invoking shell command, to be bound
  only to `/hil_offline_test/...` topics, which have no path to the
  real driver or the physical robot.
- The bridge's TCP link was continuously connected with no reconnect in
  this window, ruling out a reconnect/stale-command-replay explanation
  for this specific interval.

This evidence rules out every action *taken in this session* as a known
cause. It does **not** rule out some other origin: the plain
(non-instrumented) WSL bridge and the Pi-side TCP server log only
connect/disconnect events, not individual command payloads, so there is
no positive record of what was actually forwarded to the Pi motor
controller during this window. A hardware fault, EMI, a stray process
from an earlier, unrelated session, or some other uninstrumented cause
remain open, unresolved possibilities.

## Related, confirmed design flaw (found during this audit)

`test_hil_integration_offline.sh` started its guard without an
`--arm-topic` override, so the guard defaulted to
`hil_cmd_vel_guard.py`'s global `--arm-topic` default (`/hil_guard/arm`)
-- the same topic a real or another test's guard instance could be
subscribed to -- and the script's own `ros2 topic pub` armed that same
global topic. **This is a real, independently confirmed design flaw,
fixed in commit `feca560` ("fix: isolate offline hardware-in-loop test
topics"). It is not claimed to be the cause of this incident** -- the
reconstructed timeline shows this script's guard instances were bound
to test-namespaced topics only, so arming them (even on a shared global
arm topic) could not itself have moved the real robot. The flaw is
recorded and fixed as a matter of defense-in-depth, independent of
whether it explains this incident.

## Addendum (added during the second incident's audit): the colcon-test row above needs correction

Row `15:30:57-15:33:32` above states the colcon suite run in that
interval had "None -- offline tests... only" real-topic contact. **This
was an incomplete conclusion, corrected here rather than silently
edited away, per this session's own rule against rewriting a prior
finding without saying so.** The audit for the second incident
(`safety_incident_unexpected_motion_2_20260723/SUMMARY.md`) found that
this exact `colcon test` run for `epuck2_comm` constructs real,
unremapped `CooperativeAvoider`/`StatePublisher`/
`NetworkImpairmentRelay`/`SequenceCounterNode` rclpy nodes with no
`ROS_DOMAIN_ID` isolation, and that `CooperativeAvoider`'s test
instances are driven to publish genuine nonzero `Twist` commands on the
bare `cmd_vel` topic -- the same topic name the WSL bridge (PID 7961,
confirmed alive and connected throughout this incident's window) is
subscribed to. This run landed at 2026-07-23T14:31:08Z (15:31:08 BST),
**inside this incident's own reported window (15:15-15:35 BST).**

This does **not** change the formal conclusion: command origin remains
**NOT_MEASURABLE** for this incident (no continuous `/cmd_vel`
recording exists either way). It does mean the same-domain unremapped
test publisher must now be treated as a **high-confidence candidate
cause for this incident too**, not proven fact -- exactly the same
status assigned to it in the second incident's own record. See that
document for the full mechanism-level finding and the fix implemented
(topic namespacing + `ROS_DOMAIN_ID` isolation + `run_isolated_test_suite.sh`).

## Explicitly not done / not concluded

- Root cause is **not solved** and is **not claimed solved**.
- No Pi-side per-command logging exists yet; this is recorded as a
  blocking requirement for any future powered session (see the updated
  `HIL_SAFETY_CHECKLIST.md`), not implemented as part of this audit.
- No code was changed until this read-only audit was complete, other
  than the arm-topic isolation fix explicitly authorized afterward.
- No physical-stack work or motion has been resumed since the incident
  was reported.

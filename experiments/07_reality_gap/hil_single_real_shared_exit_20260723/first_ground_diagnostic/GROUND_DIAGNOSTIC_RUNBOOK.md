# First ground diagnostic -- supervised runbook

One command at a time, operator-executed, exactly as every other
physical HIL session in this project. Does not authorize skipping any
step in `../HIL_SAFETY_CHECKLIST.md` -- it only sequences them for this
specific, bounded diagnostic. Stop immediately and report if any
step's actual result differs from its expected result.

**Read `FIRST_GROUND_DIAGNOSTIC_SPEC.md` and complete
`FIELD_MEASUREMENT_FORM.md` before starting.** All commands assume the
working directory
`experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools`
and that ROS2 Humble + this workspace are sourced, unless stated
otherwise.

## Fixed terminal/window map

Stable window labels so commands, logs, PIDs, and evidence can be
found without confusion across a whole session. Before every command
below, its heading states which window runs it, in the bracketed form
used throughout the numbered steps further down this document. Never
reuse a long-running-process window for another command. This map is
documentation/operation guidance only -- it does not itself start any
process.

| Window | Long-running? | Purpose | Do not do here |
|---|---|---|---|
| Pi Window 1 -- physical driver | Yes, whole session | Runs `ros2 run epuck_ros2_driver driver`; shows hardware init, sensor-driver status, final zero-wheel/SIGINT messages | Any other command |
| Pi Window 2 -- audited command server | Yes, whole session | Runs `pi_epuck_tcp_server_sensors_audited.py`; writes the approved Pi JSONL; shows port/watchdog/limits/connection/shutdown status | Verification commands |
| Pi Window 3 -- Pi read-only verification | No | Checks `/scan`, Pi process PIDs, JSONL existence/growth; runs the Pi-side audit verifier; shows the Pi verdict | Starting any long-running driver/server process |
| WSL Window 1 -- TCP bridge | Yes, whole session | Runs `wsl_epuck_tcp_bridge_sensors.py`; shows connection/reconnection status | Any other command |
| WSL Window 2 -- state publisher | Yes, whole session | Runs `state_publisher`; publishes `/epuck1/state` | Any other command |
| WSL Window 3 -- command-evidence recorder control | Yes, whole session | Starts the WSL recorder (approved evidence root, `--duration-s 3600`); shows/stores PID + manifest path; checks recorder log/CSV growth; stops the recorder LAST via manifest/exact PID | Anything unrelated to the recorder |
| WSL Window 4 -- command guard | Yes, whole session | Runs `hil_cmd_vel_guard.py`; shows armed/disarmed state and block reasons; must start DISARMED | Pulse publisher or verification commands |
| WSL Window 5 -- read-only HIL verification | No | Checks topic publishers, `validity_flags`; runs WSL live-zero-state and the combined WSL+Pi gate; identifies exact PIDs during shutdown | Starting long-running control processes |
| PowerShell Window 1 -- operator transfer and host checks | No | Opens SSH sessions when instructed; performs explicit SCP copies requiring the operator's password; verifies Windows-side copied-file SHA-256 | ROS publishers or tests |

**Open item, not yet resolved:** none of the nine windows above is
described as the place to run the one-shot arm/disarm topic
publishes and the pulse command itself (step 13/14) -- `WSL Window 4`
explicitly excludes pulse publishing, and `WSL Window 5` is read-only
verification only. Do not silently pick one -- confirm which window
(or a tenth, explicitly added) is authorized for that step before it
is ever reached.

## Emergency procedure (read this before step 1, keep it visible throughout)

- **Any unexpected sound or motion: hardware power off first.** The
  physical e-stop / power switch is the primary stop, not software,
  not Ctrl+C.
- **Do not run any further diagnostic command while motion continues.**
  Do not attempt to "catch up" with a stop command while the robot is
  still moving unexpectedly -- cut power, then assess.
- **Ctrl+C on a running tool is a secondary measure only**, useful
  once power is already cut or motion has already stopped -- never the
  first or only response.
- **Preserve the WSL CSV and the Pi JSONL exactly as they are** the
  moment anything unexpected happens -- do not delete, truncate, or
  "clean up" either file.
- **Record the time and a plain description of what was observed**
  immediately, before doing anything else procedural.
- **Mark the run `EXCLUDED` pending analysis** -- never `PASS`, never
  silently re-attempt. See `../safety_incident_unexpected_motion_20260723/SUMMARY.md`
  and `../safety_incident_unexpected_motion_2_20260723/SUMMARY.md` for
  how the two prior incidents were handled; the same discipline applies
  here. **If the WSL command-evidence recorder exits (for any reason,
  including its own duration timeout) before the pulse step
  completes, the run is EXCLUDED** -- never silently restart the
  recorder mid-run and continue as if nothing happened.

## Step -1: initialize the per-session confirmation file (start of every session)

`[WSL Window 5 -- read-only HIL verification]` -- not long-running, no
persistent output beyond the session-state file itself.

```bash
python3 hil_ground_diagnostic_session.py init --path ground_diagnostic_session_state.json
```
Always resets to a fresh timestamp with all four confirmations `false`
-- never reuse a file from an earlier session (`GROUND_DIAGNOSTIC_PRE_STACK_CHECK`
below rejects it once older than 4 hours). After the physical checks in
step 2 below, set each confirmed field:
```bash
python3 hil_ground_diagnostic_session.py set --path ground_diagnostic_session_state.json --field floor_condition_confirmed
python3 hil_ground_diagnostic_session.py set --path ground_diagnostic_session_state.json --field travel_path_clear_confirmed
python3 hil_ground_diagnostic_session.py set --path ground_diagnostic_session_state.json --field operator_present_confirmed
python3 hil_ground_diagnostic_session.py set --path ground_diagnostic_session_state.json --field wifi_checked_in_test_area
```

## Step 0: pre-stack preflight (repeat before proceeding, and again after step 9)

`[WSL Window 5 -- read-only HIL verification]` -- not long-running.

```bash
bash run_ground_diagnostic_preflight.sh pre-stack
```

Read-only; starts nothing. Must report
`GROUND_DIAGNOSTIC_PRE_STACK_CHECK_PASS` before continuing past any
stop point below -- this requires both the tracked geometry/venue
fields in `ground_diagnostic_params.json` AND all four session-file
confirmations above to be true; a tracked value can never substitute
for a session-file confirmation. If it reports
`GROUND_DIAGNOSTIC_PRE_STACK_CHECK_BLOCKED`, the printed reason says
exactly what is missing -- do not proceed on a guess.

## 1. Robot powered off, placed at the measured start pose

No terminal command -- physical action. Per
`FIELD_MEASUREMENT_FORM.md`'s `start_x_m`/`start_y_m`/`start_yaw_rad`.
Robot remains powered off through this step.

## 2. Physical safety confirmations (STOP POINT)

No terminal command -- verbal confirmation. Do not proceed without the
user providing, verbatim, in the same session:
```
ROBOT_ON_STAND=YES
WHEELS_CLEAR_OF_GROUND=YES
USER_AT_EMERGENCY_STOP=YES
TEST_AREA_CLEAR=YES
FLOOR_CONDITION_CLEAR=YES
TRAVEL_PATH_CLEAR=YES
```
Note: for this diagnostic the robot is placed on the GROUND per step 1
above, but these six confirmations are still required verbatim before
any process starts -- "wheels clear of ground" here means confirmed
clear of unintended obstructions at the ground-contact points, not
suspended; the wheels-suspended requirement applies only during
bring-up (steps 3-6 below), per step 11. The first four are inherited
unchanged from `../HIL_SAFETY_CHECKLIST.md`. `FLOOR_CONDITION_CLEAR`
and `TRAVEL_PATH_CLEAR` are explicit and separate from `TEST_AREA_CLEAR`
-- neither may be inferred or set automatically from it; each must be
its own, distinct confirmation, then recorded via `hil_ground_diagnostic_session.py
set --field floor_condition_confirmed` / `--field travel_path_clear_confirmed`
per Step -1 above.

## 3. Pi driver

`[Pi Window 1 -- physical driver]` -- remains open for the whole
session; do not run any other command in this window. Record its exact
PID once started.

```bash
ros2 run epuck_ros2_driver driver
```

`[Pi Window 3 -- Pi read-only verification]` -- verify:
```bash
ros2 topic info /scan
```
Expected: `Publisher count: 1`.

## 4. Audited Pi server, new JSONL path

`[Pi Window 3 -- Pi read-only verification]` -- confirm the new path
does not already exist first:
```bash
test ! -e /home/pi/real_robot_avoidance_v1/command_audit_<RUN_ID>.jsonl && echo PATH_AVAILABLE
```

`[Pi Window 2 -- audited command server]` -- remains open for the
whole session; do not run verification commands in this window. This
is the evidence path for this window: the Pi JSONL. Record its exact
PID once started.
```bash
cd /home/pi/real_robot_avoidance_v1/
python3 pi_epuck_tcp_server_sensors_audited.py --ros-args \
    -p command_audit_enabled:=true \
    -p command_audit_path:=/home/pi/real_robot_avoidance_v1/command_audit_<RUN_ID>.jsonl
```
Confirm that `command_audit_enabled=True` appears in the startup log
with watchdog/limits unchanged.

## 5. WSL bridge

`[WSL Window 1 -- TCP bridge]` -- remains open for the whole session;
do not run other commands here. Record its exact PID once started.

```bash
source /opt/ros/humble/setup.bash && source ~/epuck_ws/install/setup.bash && \
  cd /home/eamon/epuck_ws/epuck_comm_project/real_robot_avoidance_v1/ && \
  python3 wsl_epuck_tcp_bridge_sensors.py
```
Verify: `TCP bridge connected`.

## 6. state_publisher

`[WSL Window 2 -- state publisher]` -- remains open for the whole
session; do not run other commands here. Record its exact PID (and its
`ros2 run` wrapper PID, if separate) once started.

```bash
ros2 run epuck2_comm state_publisher --ros-args \
  -p robot_id:=1 -p source:=hardware -p use_sim_time:=false -p mode:=periodic \
  -r state:=/epuck1/state
```

`[WSL Window 5 -- read-only HIL verification]` -- verify:
```bash
ros2 topic echo /epuck1/state --field validity_flags --once
```
Expected: `7`.

## 7. WSL command-evidence recorder

`[WSL Window 3 -- command-evidence recorder control]` -- remains open
for the whole session; do not run anything unrelated to the recorder
here. Uses the approved evidence root and `--duration-s 3600` (raised
2026-07-24 from 600s, which expired mid-session during the first live
attempt before the diagnostic reached the pulse step). The recorder
stays bounded -- it is still stopped LAST, by exact PID via its
manifest, during normal shutdown; **if it exits on its own (duration
timeout or otherwise) before the pulse step completes, the run is
EXCLUDED** -- never silently start a replacement recorder mid-run.
This is the evidence path for this window: the WSL CSV, `recorder.log`,
and `manifest.json`. Record its exact PID and manifest path once
started.

```bash
bash run_hil_command_evidence_recorder.sh start \
    --upstream-cmd-vel-topic cmd_vel_unguarded \
    --guarded-cmd-vel-topic cmd_vel \
    --arm-topic /hil_guard/arm \
    --state-topic /epuck1/state \
    --bridge-status-topic /epuck_bridge/status \
    --flush-interval-s 1 \
    --duration-s 3600
```
Confirm `HIL_COMMAND_EVIDENCE_RECORDER_TOPIC_VERIFY ok=True` in its log,
manifest path recorded. Live CSV growth must be re-checked (step 8)
before guard startup (step 9) -- never assume it is still growing from
an earlier check.

## 8. Confirm both evidence files are growing

`run_ground_diagnostic_preflight.sh live-zero-state`, run from WSL,
cannot read the Pi's local command-audit JSONL by a plain path -- the
Pi and WSL share no filesystem, only a network connection (found
2026-07-24 during the first live attempt; see
`hil_ground_diagnostic_phases.py`'s module docstring). The Pi side of
the evidence must be verified separately, on the Pi itself.

`[Pi Window 3 -- Pi read-only verification]` -- this is also the
evidence path for this window's own output: the Pi verdict JSON.

**Deployment unit, not yet copied to the Pi as of 2026-07-24:**
`pi_ground_diagnostic_audit_verifier.py` is not standalone -- it
imports `compute_pi_command_maxima` and `load_pi_jsonl_records` from
`analyze_ground_diagnostic.py` (reused, not duplicated). Both files
must be copied to the Pi together, into the same directory, before
this step can run there. Verify both files' SHA-256 against the
reviewed values below immediately after copying, before running
either:
```
pi_ground_diagnostic_audit_verifier.py  ff9390597cdf27a6e485bb0470bf9fa9a66d53ff8fb3da0bf79c4e88c0972565
analyze_ground_diagnostic.py            faa14871314ab8bcbfa401204679f45173e661573379893b53a086864eb4b73f
```
Neither file has any ROS/rclpy or third-party dependency -- Python 3
standard library only (`json`, `sys`, `time`, `dataclasses`,
`datetime`, `typing`, `argparse`, `csv`, `hashlib`). Neither publishes,
subscribes, or otherwise touches ROS/hardware state -- both only read
a file and write a separate output file. The verifier can safely run
while the audited server is still writing the same JSONL (a plain,
unlocked file read against a concurrent appender).

```bash
python3 pi_ground_diagnostic_audit_verifier.py \
    --path /home/pi/real_robot_avoidance_v1/command_audit_<RUN_ID>.jsonl \
    --run-id <RUN_ID> \
    --output-json /home/pi/real_robot_avoidance_v1/pi_audit_verdict_<RUN_ID>.json
```
`<RUN_ID>` must be the same identifier used for both the Pi JSONL and
WSL CSV paths this session.

`[PowerShell Window 1 -- operator transfer and host checks]` -- copy
the resulting verdict JSON to the WSL evidence root with one explicit
SCP command (never non-interactive/automated), then verify its
SHA-256 on both sides:
```bash
scp pi@<PI_IP>:/home/pi/real_robot_avoidance_v1/pi_audit_verdict_<RUN_ID>.json /home/eamon/epuck_comm_bags/<WSL_EVIDENCE_DIR>/pi_audit_verdict_<RUN_ID>.json
```

`[WSL Window 5 -- read-only HIL verification]` -- this is also the
evidence path for this window's own output: the live/combined gate
result.
```bash
GROUND_DIAGNOSTIC_WSL_CSV=<output_dir>/command_evidence.csv \
GROUND_DIAGNOSTIC_RUN_ID=<RUN_ID> \
GROUND_DIAGNOSTIC_PI_JSONL=/home/pi/real_robot_avoidance_v1/command_audit_<RUN_ID>.jsonl \
GROUND_DIAGNOSTIC_PI_AUDIT_VERDICT=<path to the copied verdict JSON> \
  bash run_ground_diagnostic_preflight.sh live-zero-state
```
`GROUND_DIAGNOSTIC_PI_JSONL` here is the *declared* Pi JSONL path used
only to check the copied verdict's own recorded path matches --
nothing reads it as a local file from WSL. This phase is expected to
still report `GROUND_DIAGNOSTIC_LIVE_ZERO_STATE_CHECK_BLOCKED` at this
point -- the guard has not started yet -- this step is only to confirm
both evidence streams are growing and zero-only, not a stop point by
itself. Without `GROUND_DIAGNOSTIC_PI_AUDIT_VERDICT`, the check reports
`PI_LIVE_AUDIT_NOT_AVAILABLE` and blocks; a malformed, stale, or
run-ID/path-mismatched verdict blocks with its own distinct reason --
none is ever silently treated as passing or as proven nonzero.

## 9. Guard started DISARMED, diagnostic-only limits

`[WSL Window 4 -- command guard]` -- remains open for the whole
session until guard shutdown; do not run a pulse publisher or
verification command in this window. Record its exact PID once
started.

```bash
python3 hil_cmd_vel_guard.py \
    --physical-state-topic /epuck1/state \
    --upstream-cmd-vel-topic cmd_vel_unguarded \
    --guarded-cmd-vel-topic cmd_vel \
    --max-linear-speed-mps 0.02 \
    --max-angular-speed-rps 0.0 \
    --required-validity-flags 7
```
Values match `ground_diagnostic_params.json`'s `diagnostic_command_limits`
exactly (`max_linear_speed_mps` is the existing, already-confirmed
frozen guard cap; `max_angular_speed_rps` is fixed at `0.0` --
prohibited for this test, never the suspended-wheel angular value).
Verify startup log shows `armed=False`.

## 10. Confirm sole publisher and zero output (STOP POINT -- re-run live-zero-state)

`[WSL Window 5 -- read-only HIL verification]`:
```bash
ros2 topic info /cmd_vel -v
ros2 topic echo /cmd_vel --once
```
Re-run the Pi-side verifier (`[Pi Window 3 -- Pi read-only
verification]`, step 8) once more to get a fresh verdict (a stale one
is rejected -- default max age 300s), copy it back
(`[PowerShell Window 1 -- operator transfer and host checks]`), then
re-run the combined gate (`[WSL Window 5 -- read-only HIL
verification]`):
```bash
GROUND_DIAGNOSTIC_WSL_CSV=<output_dir>/command_evidence.csv \
GROUND_DIAGNOSTIC_RUN_ID=<RUN_ID> \
GROUND_DIAGNOSTIC_PI_JSONL=/home/pi/real_robot_avoidance_v1/command_audit_<RUN_ID>.jsonl \
GROUND_DIAGNOSTIC_PI_AUDIT_VERDICT=<path to the freshly copied verdict JSON> \
  bash run_ground_diagnostic_preflight.sh live-zero-state
```
Sole publisher must be `hil_cmd_vel_guard`; output must be exactly
zero on every field; the preflight re-run must report
`GROUND_DIAGNOSTIC_LIVE_ZERO_STATE_CHECK_PASS`. Do not proceed
otherwise.

## 11. Wheels placed on ground only after zero checks pass

No terminal command -- physical action. Only after step 10 passes --
the robot was placed at the start pose in step 1 with wheels off the
ground for bring-up; this is the point it actually bears weight for
the first time this session.

## 12. Explicit human approval immediately before motion (STOP POINT)

No terminal command -- verbal confirmation. Require an explicit,
separate, verbatim confirmation from the user at this exact point --
distinct from step 2's six confirmations -- e.g.
`APPROVED_FOR_SINGLE_PULSE=YES`. Do not issue the pulse command without
this, and do not accept step 2's confirmations as satisfying it.

## 13. Arm and issue one bounded straight pulse

**Window not yet assigned -- see "Open item" in the window map above.**
Confirm which window is authorized before this step is ever reached.

```bash
ros2 topic pub --once /hil_guard/arm std_msgs/msg/Bool "{data: true}"
python3 hil_wheel_suspension_test.py \
    --upstream-cmd-vel-topic cmd_vel_unguarded \
    --pulse-linear-mps 0.015 --zero-hold-s 1 --pulse-s 2 --post-hold-s 1
```
Reuses `hil_wheel_suspension_test.py` exactly as already tested for the
suspended-wheel diagnostic -- no new pulse mechanism. This single
invocation already includes the pre/post zero holds and exits on its
own; it is never looped or re-run automatically.

## 14. Command immediate zero

**Window not yet assigned -- same open item as step 13.**

```bash
ros2 topic pub --once /hil_guard/arm std_msgs/msg/Bool "{data: false}"
```
Disarms immediately regardless of any other input -- the same
emergency-stop software command used throughout this project.

## 15. Human observation

No terminal command -- verbal/written record. Record: did the robot
move as expected (short, straight, low-speed) and stop completely? Any
sound, drift, or unexpected direction? Measured stopping clearance
(fill into `FIELD_MEASUREMENT_FORM.md`).

## 16. Exact-PID reverse shutdown, evidence recorder last

Shutdown order, by window label and exact PID (never `pkill`, always
`kill -INT` on the exact recorded PID):

1. `[WSL Window 4 -- command guard]` -- guard PID.
2. `[WSL Window 2 -- state publisher]` -- state_publisher PID (and its
   `ros2 run` wrapper PID, if separate).
3. `[WSL Window 1 -- TCP bridge]` -- bridge PID.
4. `[Pi Window 2 -- audited command server]` -- audited server PID
   (confirm the Pi JSONL closes with safe zero/shutdown records).
5. `[Pi Window 1 -- physical driver]` -- driver PID (and its `ros2 run`
   wrapper PID, if separate).
6. `[WSL Window 3 -- command-evidence recorder control]` -- recorder,
   LAST, via its manifest (`run_hil_command_evidence_recorder.sh stop
   <manifest.json>`) or exact PID.

Verify each stop (`[Pi Window 3 -- Pi read-only verification]` for Pi
PIDs, `[WSL Window 5 -- read-only HIL verification]` for WSL PIDs) --
**closing a terminal window is not a substitute for an exact-PID
`kill -INT`.**

## Window/evidence table for the run summary

Copy this table (filled in) into the run's `SUMMARY.md` so the
operator can later locate all data from one place:

| Window | Exact PID | Evidence path |
|---|---|---|
| Pi Window 1 -- physical driver | | (none -- console log only) |
| Pi Window 2 -- audited command server | | Pi JSONL: |
| Pi Window 3 -- Pi read-only verification | (not long-running) | Pi verifier verdict JSON: |
| WSL Window 1 -- TCP bridge | | (none -- console log only) |
| WSL Window 2 -- state publisher | | (none -- console log only) |
| WSL Window 3 -- command-evidence recorder control | | CSV / recorder.log / manifest.json: |
| WSL Window 4 -- command guard | | (none -- console log only) |
| WSL Window 5 -- read-only HIL verification | (not long-running) | live/combined gate output: |

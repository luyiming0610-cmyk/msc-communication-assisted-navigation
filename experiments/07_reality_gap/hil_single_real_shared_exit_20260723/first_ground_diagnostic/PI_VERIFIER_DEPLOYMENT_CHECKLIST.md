# Pi verifier deployment checklist (prepared, not executed)

One command at a time, operator-executed. This checklist is only
prepared here -- it has not been run. Deployment here means **file
transfer and Python compile verification only** -- it must not start
ROS, the driver, the server, the bridge, the guard, or any controller.
The Pi itself must be powered on and reachable for the SSH/SCP/hash/
`py_compile` steps below to work at all (an earlier version of this
checklist incorrectly required `PI_POWERED_OFF=YES`, which is
inconsistent with those same steps -- corrected here). It deploys
exactly the reviewed, two-file, hash-pinned unit documented in
`GROUND_DIAGNOSTIC_RUNBOOK.md` step 8:

| File | Reviewed SHA-256 |
|---|---|
| `pi_ground_diagnostic_audit_verifier.py` | `ff9390597cdf27a6e485bb0470bf9fa9a66d53ff8fb3da0bf79c4e88c0972565` |
| `analyze_ground_diagnostic.py` | `faa14871314ab8bcbfa401204679f45173e661573379893b53a086864eb4b73f` |

**Precondition, verbatim from the operator, before step 1:**
```
ROBOT_ON_STAND=YES
WHEELS_CLEAR_OF_GROUND=YES
USER_AT_EMERGENCY_STOP=YES
PHYSICAL_MOTION_STACK_STOPPED=YES
PI_POWERED_ON_AND_REACHABLE=YES
```
The robot stays physically secured on its stand, wheels clear of the
ground, throughout deployment -- the Pi being powered on for file
transfer is not authorization to place the robot on the ground or to
bring up any part of the motion stack. `PHYSICAL_MOTION_STACK_STOPPED=YES`
means none of the following processes is running, on either machine,
for the whole duration of this checklist:
```
epuck_ros2_driver
pi_epuck_tcp_server_sensors
pi_epuck_tcp_server_sensors_audited
wsl_epuck_tcp_bridge_sensors
state_publisher
hil_cmd_vel_guard
cooperative_avoider
hil_virtual_peer
goal_navigator
hil_topic_adapter
rosbag
```
Verify this before step 1 and re-verify before step 3 (the copy) and
step 5 (`py_compile`) -- if any of these processes is found running at
any point, stop immediately and do not continue this checklist.

## 0. Confirm none of the listed processes is running

`[Pi Window 3 -- Pi read-only verification]`:
```bash
ssh pi@<PI_IP> "pgrep -af '[e]puck_ros2_driver|[p]i_epuck_tcp_server_sensors|[p]i_epuck_tcp_server_sensors_audited' || echo PI_SIDE_CLEAN"
```
`[WSL Window 5 -- read-only HIL verification]`:
```bash
pgrep -af '[w]sl_epuck_tcp_bridge_sensors|[s]tate_publisher|[h]il_cmd_vel_guard|[c]ooperative_avoider|[h]il_virtual_peer|[g]oal_navigator|[h]il_topic_adapter|[r]osbag'
```
Expected: `PI_SIDE_CLEAN` and no output from the second command. Stop
and report if either check finds a match -- do not proceed while any
listed process is running.

## 1. Compute local (repository-side) hashes to confirm against the table above

`[PowerShell Window 1 -- operator transfer and host checks]`
```bash
sha256sum experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools/pi_ground_diagnostic_audit_verifier.py experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools/analyze_ground_diagnostic.py
```
Expected: both hashes match the table above exactly. Stop and report
if either does not match -- do not proceed with a mismatched file.

## 2. Check the intended Pi paths do not already exist

`[PowerShell Window 1 -- operator transfer and host checks]` -- opens
one SSH session, password entered manually by the operator:
```bash
ssh pi@<PI_IP> "test ! -e /home/pi/real_robot_avoidance_v1/pi_ground_diagnostic_audit_verifier.py && test ! -e /home/pi/real_robot_avoidance_v1/analyze_ground_diagnostic.py && echo BOTH_PATHS_AVAILABLE"
```
Expected: `BOTH_PATHS_AVAILABLE`. If either path already exists, stop
and report -- **never overwrite an existing file on the Pi.** Choose a
different destination name or resolve the conflict before continuing.

## 3. Copy both files together, in one command

Re-verify step 0 (none of the listed processes running) immediately
before this step.

`[PowerShell Window 1 -- operator transfer and host checks]` -- one
explicit, interactive SCP invocation, password entered manually:
```bash
scp experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools/pi_ground_diagnostic_audit_verifier.py experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools/analyze_ground_diagnostic.py pi@<PI_IP>:/home/pi/real_robot_avoidance_v1/
```
Both files are copied together in the same invocation so the
deployment unit can never be left partially copied.

## 4. Verify both deployed hashes on the Pi

`[PowerShell Window 1 -- operator transfer and host checks]`:
```bash
ssh pi@<PI_IP> "sha256sum /home/pi/real_robot_avoidance_v1/pi_ground_diagnostic_audit_verifier.py /home/pi/real_robot_avoidance_v1/analyze_ground_diagnostic.py"
```
Expected: both hashes match the table above exactly. Stop and report
if either does not match -- treat this as a failed, not a retried,
deployment; do not silently re-copy and re-check.

## 5. Confirm both files are syntactically valid Python -- no ROS process

Re-verify step 0 (none of the listed processes running) immediately
before this step.

`[PowerShell Window 1 -- operator transfer and host checks]`:
```bash
ssh pi@<PI_IP> "python3 -m py_compile /home/pi/real_robot_avoidance_v1/pi_ground_diagnostic_audit_verifier.py /home/pi/real_robot_avoidance_v1/analyze_ground_diagnostic.py && echo PY_COMPILE_OK"
```
Expected: `PY_COMPILE_OK`. This only byte-compiles the two files -- it
does not import ROS, does not start the driver, server, bridge, or
guard, and does not run the verifier itself.

## 6. Record the deployed paths and hashes

Fill this in (into the run's own evidence record, e.g. a future
`SUMMARY.md`) once steps 1-5 all pass:

| File | Deployed path | Verified SHA-256 |
|---|---|---|
| `pi_ground_diagnostic_audit_verifier.py` | `/home/pi/real_robot_avoidance_v1/pi_ground_diagnostic_audit_verifier.py` | |
| `analyze_ground_diagnostic.py` | `/home/pi/real_robot_avoidance_v1/analyze_ground_diagnostic.py` | |

## Stop here

Deployment ends at step 6. This checklist does not authorize starting
the driver, server, bridge, guard, or any physical attempt -- that
remains a separate, explicit decision, gated by its own runbook steps.

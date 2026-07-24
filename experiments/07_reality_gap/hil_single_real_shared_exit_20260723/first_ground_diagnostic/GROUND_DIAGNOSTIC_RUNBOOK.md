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
  here.

## Step 0: preflight (repeat before proceeding, and again at step 10)

```bash
bash run_ground_diagnostic_preflight.sh
```

Read-only; starts nothing. Must report `GROUND_DIAGNOSTIC_PREFLIGHT_PASS`
before continuing past any stop point below. If it reports
`GROUND_DIAGNOSTIC_PREFLIGHT_BLOCKED`, the printed reason says exactly
what is missing -- do not proceed on a guess.

## 1. Robot powered off, placed at the measured start pose

Per `FIELD_MEASUREMENT_FORM.md`'s `start_x_m`/`start_y_m`/`start_yaw_rad`.
Robot remains powered off through this step.

## 2. Physical safety confirmations (STOP POINT)

Do not proceed without the user providing, verbatim, in the same
session:
```
ROBOT_ON_STAND=YES
WHEELS_CLEAR_OF_GROUND=YES
USER_AT_EMERGENCY_STOP=YES
TEST_AREA_CLEAR=YES
```
Note: for this diagnostic the robot is placed on the GROUND per step 1
above, but these four confirmations (inherited unchanged from
`../HIL_SAFETY_CHECKLIST.md`) are still required verbatim before any
process starts -- "wheels clear of ground" here means confirmed clear
of unintended obstructions at the ground-contact points, not
suspended; the wheels-suspended requirement applies only during
bring-up (steps 3-6 below), per step 11.

## 3. Pi driver

```bash
ros2 run epuck_ros2_driver driver
```
Verify: `ros2 topic info /scan` shows `Publisher count: 1`.

## 4. Audited Pi server, new JSONL path

```bash
cd /home/pi/real_robot_avoidance_v1/
python3 pi_epuck_tcp_server_sensors_audited.py --ros-args \
    -p command_audit_enabled:=true \
    -p command_audit_path:=/home/pi/real_robot_avoidance_v1/command_audit_<NEW_UTC_TIMESTAMP>.jsonl
```
Confirm the new path did not already exist first (`test ! -e ...`), and
that `command_audit_enabled=True` appears in the startup log with
watchdog/limits unchanged.

## 5. WSL bridge

```bash
source /opt/ros/humble/setup.bash && source ~/epuck_ws/install/setup.bash && \
  cd /home/eamon/epuck_ws/epuck_comm_project/real_robot_avoidance_v1/ && \
  python3 wsl_epuck_tcp_bridge_sensors.py
```
Verify: `TCP bridge connected`.

## 6. state_publisher

```bash
ros2 run epuck2_comm state_publisher --ros-args \
  -p robot_id:=1 -p source:=hardware -p use_sim_time:=false -p mode:=periodic \
  -r state:=/epuck1/state
```
Verify: `ros2 topic echo /epuck1/state --field validity_flags --once` reads `7`.

## 7. WSL command-evidence recorder

```bash
bash run_hil_command_evidence_recorder.sh start \
    --upstream-cmd-vel-topic cmd_vel_unguarded \
    --guarded-cmd-vel-topic cmd_vel \
    --arm-topic /hil_guard/arm \
    --state-topic /epuck1/state \
    --bridge-status-topic /epuck_bridge/status \
    --flush-interval-s 1 \
    --duration-s 600
```
Confirm `HIL_COMMAND_EVIDENCE_RECORDER_TOPIC_VERIFY ok=True` in its log,
manifest path recorded.

## 8. Confirm both evidence files are growing

```bash
GROUND_DIAGNOSTIC_PI_JSONL=/home/pi/real_robot_avoidance_v1/command_audit_<NEW_UTC_TIMESTAMP>.jsonl \
GROUND_DIAGNOSTIC_WSL_CSV=<output_dir>/command_evidence.csv \
  bash run_ground_diagnostic_preflight.sh
```
(The Pi-side JSONL row count must be checked over SSH separately if
this script is run from WSL only; see step 0's script for the exact
non-growth failure condition either side would report.)

## 9. Guard started DISARMED, diagnostic-only limits

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

## 10. Confirm sole publisher and zero output (STOP POINT -- re-run step 0)

```bash
ros2 topic info /cmd_vel -v
ros2 topic echo /cmd_vel --once
bash run_ground_diagnostic_preflight.sh
```
Sole publisher must be `hil_cmd_vel_guard`; output must be exactly
zero on every field; the preflight re-run must report
`GROUND_DIAGNOSTIC_PREFLIGHT_PASS`. Do not proceed otherwise.

## 11. Wheels placed on ground only after zero checks pass

Only after step 10 passes -- the robot was placed at the start pose in
step 1 with wheels off the ground for bring-up; this is the point it
actually bears weight for the first time this session.

## 12. Explicit human approval immediately before motion (STOP POINT)

Require an explicit, separate, verbatim confirmation from the user at
this exact point -- distinct from step 2's four confirmations -- e.g.
`APPROVED_FOR_SINGLE_PULSE=YES`. Do not issue the pulse command without
this, and do not accept step 2's confirmations as satisfying it.

## 13. Arm and issue one bounded straight pulse

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

```bash
ros2 topic pub --once /hil_guard/arm std_msgs/msg/Bool "{data: false}"
```
Disarms immediately regardless of any other input -- the same
emergency-stop software command used throughout this project.

## 15. Human observation

Record: did the robot move as expected (short, straight, low-speed) and
stop completely? Any sound, drift, or unexpected direction? Measured
stopping clearance (fill into `FIELD_MEASUREMENT_FORM.md`).

## 16. Exact-PID reverse shutdown, evidence recorder last

Guard -> state_publisher -> WSL bridge -> audited Pi server (confirm
JSONL closes with safe zero/shutdown records) -> Pi driver -> WSL
command-evidence recorder last, via its manifest
(`run_hil_command_evidence_recorder.sh stop <manifest.json>`). Every
step: identify the exact PID first, `kill -INT` only, never `pkill`.

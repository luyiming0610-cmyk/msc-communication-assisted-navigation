# HIL lab runbook (2026-07-23)

This is the step-by-step operator procedure. It does not authorize
skipping any step in `HIL_SAFETY_CHECKLIST.md` -- it only sequences
them. Stop immediately and report if any step's actual result differs
from its expected result. All commands below assume the working
directory `experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools`
unless stated otherwise, and that ROS2 Humble + this workspace are
sourced.

## 1. Offline checks (no hardware, no network)

**Since the second 2026-07-23 `UNEXPECTED_PHYSICAL_MOTION` incident
(`safety_incident_unexpected_motion_2_20260723/SUMMARY.md`): direct
`pytest`/`colcon test`/`python3 -m unittest` invocation of the
`epuck2_comm` package is PROHIBITED whenever any part of the physical
stack (bridge, driver, state_publisher, guard, or controller) could be
live. Its own test suite was found to construct real, unremapped
rclpy nodes with no ROS_DOMAIN_ID isolation -- a routine test run could
place genuine nonzero commands onto the real `cmd_vel` topic if a live
bridge happened to share the same DDS domain. The isolated topic remaps
now present in the affected test files (`-r __ns:=/pytest_isolated`)
and `conftest.py`'s forced `ROS_DOMAIN_ID` are defense-in-depth, not a
reason to relax this rule. The ONLY sanctioned way to run either suite
from now on is:**

```bash
bash run_isolated_test_suite.sh
```

This refuses to run at all if any physical/HIL process is detected,
checks the real `/cmd_vel` publisher count in the default ROS domain
before touching anything, switches to a dedicated non-physical
`ROS_DOMAIN_ID` only for the duration of the test run, runs the HIL
suite and the colcon suite inside that isolation, then switches back
and re-checks `/cmd_vel` afterward. It prints
`SAFE_ISOLATED_TEST_RUN_PASS` or `SAFE_ISOLATED_TEST_RUN_FAIL` with the
specific reason.

The offline integration test and the launcher's own `--check-only`/
`--dry-run` modes remain safe to run directly (they start no rclpy
node with a real topic, or refuse to start anything):

```bash
bash test_hil_integration_offline.sh
bash run_hil_shared_exit_trial.sh --check-only
bash run_hil_shared_exit_trial.sh --dry-run
```

Expected: `run_isolated_test_suite.sh` reports
`SAFE_ISOLATED_TEST_RUN_PASS`; the offline integration test passes;
`--check-only` prints `hil_preflight.py`'s JSON report; while
`hil_frozen_params.json` still has any `UNCONFIRMED_PHYSICAL_MEASUREMENT`
field, its `status` is `BLOCKED_AWAITING_LAB_MEASUREMENT`, which is the
correct/expected result at this stage, not a failure.

## 2. Bring up the physical stack (one window each; leave all running)

**Per `HIL_SAFETY_CHECKLIST.md`'s "Required for any future powered
physical session" section (added after the 2026-07-23
`UNEXPECTED_PHYSICAL_MOTION` incident,
`safety_incident_unexpected_motion_20260723/SUMMARY.md`): the robot
must be on its stand, wheels suspended, for all of step 2 and for any
later reconnect of any of these processes -- never bring up or
reconnect any part of this stack with the robot on the ground.
Continuous recording of `/cmd_vel` and `/cmd_vel_unguarded` must also
be started and confirmed active before the robot is ever placed on the
ground (step 4 below) -- do not rely on a point-in-time publisher-count
snapshot.**

**Pi window 1 -- driver:**
```bash
ros2 run epuck_ros2_driver driver
```
Verify (Pi window 2 or a read-only check): `pgrep -af epuck_ros2_driver`
shows two PIDs.

**Pi window 2 -- expanded TCP server:**
```bash
cd /home/pi/real_robot_avoidance_v1/ && python3 pi_epuck_tcp_server_sensors.py
```
Verify: log line `Pi TCP bridge listening on 0.0.0.0:5809; ...`;
`ss -ltnp | grep ':5809'` shows one listener.

**WSL window 1 -- expanded bridge:**
```bash
source /opt/ros/humble/setup.bash && source ~/epuck_ws/install/setup.bash && \
  cd /home/eamon/epuck_ws/epuck_comm_project/real_robot_avoidance_v1/ && \
  python3 wsl_epuck_tcp_bridge_sensors.py
```
Verify: log line `TCP bridge connected`;
`ros2 topic echo /epuck_bridge/status --once --field data` shows
`"connected": true`.

**WSL window 2 -- state_publisher:**
```bash
source /opt/ros/humble/setup.bash && source ~/epuck_ws/install/setup.bash && \
  ros2 run epuck2_comm state_publisher --ros-args \
  -p robot_id:=1 -p source:=hardware -p use_sim_time:=false -p mode:=periodic \
  -r state:=/epuck1/state
```
Verify: `ros2 topic echo /epuck1/state --field validity_flags --once`
reads `7`.

## 3. HIL preflight (repeat before every session/trial)

```bash
bash run_hil_preflight.sh
```
Runs both the offline (`hil_preflight.py`) and read-only physical
(`run_hil_physical_preflight.sh`) checks in sequence. Prints
`HIL_PREFLIGHT_PASS` (exit 0) or `HIL_PREFLIGHT_BLOCKED` (nonzero) with
the specific reason. Read-only: starts nothing, publishes nothing.

## 4. Wheel-suspension confirmation and test (STOP POINT)

Do not proceed past step 3 without the user providing, verbatim, in
the same session:

```
ROBOT_ON_STAND=YES
WHEELS_CLEAR_OF_GROUND=YES
USER_AT_EMERGENCY_STOP=YES
TEST_AREA_CLEAR=YES
```

Before the robot is ever moved from the stand to the ground (any
session, any step after this one): confirm ALL FOUR conditions in
`HIL_SAFETY_CHECKLIST.md`'s "Robot must remain suspended until ALL of
the following are true" section hold simultaneously -- Pi-side command
audit active, WSL command-evidence recorder active, guard confirmed
sole `/cmd_vel` publisher, output confirmed continuously zero. See
`COMMAND_EVIDENCE_ACTIVATION.md` for the exact activation steps for the
first two (both implemented and tested offline only as of 2026-07-23,
never yet run against the physical stack). Stop immediately, at the
physical e-stop first, on any unexplained movement; see
`safety_incident_unexpected_motion_20260723/SUMMARY.md` and
`safety_incident_unexpected_motion_2_20260723/SUMMARY.md` for how both
2026-07-23 incidents were handled.

Then run the short suspended-wheel sequence described in
`HIL_SAFETY_CHECKLIST.md` (`hil_wheel_suspension_test.py` /
`hil_angular_suspension_test.py`, guard armed only for that test, a
temporary test-scoped `max_angular_speed_rps`, never adopted as the
real ground cap). After it, stop all motion nodes, verify zero
velocity, verify exactly one guarded `cmd_vel` publisher, stop the
actual PIDs directly (never `pkill`), confirm `PROCESSES_CLEAN`, and
wait for the user's own report on wheel directions / stopping /
anomalies / guard effectiveness before continuing. Completed and
recorded: `suspended_wheel_diagnostic_20260723/`,
`angular_suspension_diagnostic_20260723/`.

## 5. Field geometry and ground angular cap

Only after step 4 passes. Measure (do not estimate): arena dimensions,
start pose, exit location/size, parking zone, search waypoints for the
real (uninformed) robot, walls/boundary, obstacles, emergency-stop
position, WiFi coverage, minimum safety clearance, and a genuinely
measured (not suspended-wheel) ground turning-rate cap. Write these
into `hil_frozen_params.json`'s `field_geometry` and
`hil_guard_limits.max_angular_speed_rps`, replacing each
`UNCONFIRMED_PHYSICAL_MEASUREMENT`, then commit as
"docs: freeze measured hardware-in-loop geometry". Never adjust
geometry after the fact to force a later step to pass. Re-run step 3
afterward -- `run_hil_preflight.sh` should now report
`HIL_PREFLIGHT_PASS` for the offline layer.

## 6. HIL_COMM_OFF trial

```bash
bash run_hil_shared_exit_trial.sh --comm-off
```
Runs the full safety gate first (refuses with
`PHYSICAL_MOTION_LOCKED_UNTIL_LAB_VALIDATION` if step 5 is incomplete
or step 3's health check fails). If it passes the gate, starts the
recorder, monitor, virtual peer, adapter, and the real robot's frozen
`cooperative_avoider` (peer-avoidance visibility of the virtual peer
disabled in this mode, mirroring the frozen N2/N3 controllers' own
`enable_peer_avoidance=(comm_mode==COMM_ON)` semantics), then
`hil_cmd_vel_guard.py` **DISARMED**, then stops and prints
`AWAITING_EXPLICIT_ARM_CONFIRMATION` plus the exact arm/shutdown
commands and this trial's `pid_manifest.json` path. Arming is a
separate, explicit, human step -- never automated.

## 7. HIL_COMM_ON trial

Only after `HIL_COMM_OFF` has been run, observed, and its data audited
by the user.

```bash
bash run_hil_shared_exit_trial.sh --comm-on
```
Identical sequencing to step 6, except the virtual peer's
`GoalAnnouncement` reaches the real robot's adapter, and peer-avoidance
visibility is enabled.

## 8. Emergency stop (at any point during an armed trial)

Physical: press the e-stop first, always -- software is a second layer,
never the primary one.

Software:
```bash
ros2 topic pub --once /hil_guard/arm std_msgs/msg/Bool "{data: false}"
```
Disarms the guard immediately (forces zero velocity regardless of any
other input). Follow with step 9's shutdown once safe.

## 9. Normal shutdown (end of any trial)

```bash
bash run_hil_shutdown.sh <trial_output_dir>/pid_manifest.json
```
Stops every process from that trial by its **exact recorded PID**
(`kill -INT`, never `pkill`), in guard-first order, and reports
`PROCESSES_CLEAN` or exactly which PID did not stop.

## 10. Evidence locations

- Native WSL raw evidence (never committed): `/home/eamon/epuck_comm_bags/hil_<mode>_<timestamp>/` --
  `pid_manifest.json`, `recorder.log`, each process's own log, the
  rosbag under `bag/`, and `monitor_verdict.json`.
- Repo-tracked summaries (SHA-256-verified against the native copy,
  committed): `experiments/07_reality_gap/hil_single_real_shared_exit_20260723/<diagnostic_name>/SUMMARY.md`,
  following the same pattern as every diagnostic recorded so far this
  session.
- `HIL_KNOWN_LIMITATIONS_AND_READINESS_20260723.md`: the current
  overall readiness status and the known ~32-33s validity-flags
  disturbance -- read this before interpreting any trial's result.
- `safety_incident_unexpected_motion_20260723/SUMMARY.md`: the
  2026-07-23 `UNEXPECTED_PHYSICAL_MOTION` incident record (root cause
  NOT_MEASURABLE, not solved) and the preconditions it added to
  `HIL_SAFETY_CHECKLIST.md` -- read this before any future powered
  session.

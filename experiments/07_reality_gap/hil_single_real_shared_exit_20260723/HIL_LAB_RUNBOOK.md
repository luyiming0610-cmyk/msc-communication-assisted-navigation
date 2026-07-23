# HIL lab runbook (2026-07-23)

This is the step-by-step operator procedure. It does not authorize
skipping any step in `HIL_SAFETY_CHECKLIST.md` -- it only sequences
them. Stop immediately and report if any step's actual result differs
from its expected result. All commands below assume the working
directory `experiments/07_reality_gap/hil_single_real_shared_exit_20260723/tools`
unless stated otherwise, and that ROS2 Humble + this workspace are
sourced.

## 1. Offline checks (no hardware, no network)

```bash
python3 -m unittest discover -s . -p "test_hil_*.py" -v
bash test_hil_integration_offline.sh
bash run_hil_shared_exit_trial.sh --check-only
bash run_hil_shared_exit_trial.sh --dry-run
```

Expected: all unit tests and the offline integration test pass;
`--check-only` prints `hil_preflight.py`'s JSON report; while
`hil_frozen_params.json` still has any `UNCONFIRMED_PHYSICAL_MEASUREMENT`
field, its `status` is `BLOCKED_AWAITING_LAB_MEASUREMENT`, which is the
correct/expected result at this stage, not a failure.

## 2. Bring up the physical stack (one window each; leave all running)

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

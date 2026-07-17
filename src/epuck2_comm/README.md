# epuck2_comm

The package implements the reusable communication layer for the project.

## Cooperative bag analysis

After recording a two-robot trial, extract reproducible safety, synchronization,
and command-smoothness metrics with:

```bash
ros2 run epuck2_comm analyze_cooperative_bag /path/to/rosbag_directory
```

The command writes `summary.json`, `separation.csv`, and `commands.csv` into an
`analysis` directory inside the bag. The default e-puck collision threshold is
based on a 35 mm robot radius and can be changed with `--robot-radius-m`.

For a long-course static-obstacle run, use:

```bash
ros2 run epuck2_comm analyze_static_bag /path/to/rosbag_directory
```

The static analyzer writes geometric box-surface clearance, measured minimum
front range, path length, forward progress, lateral deviation, path efficiency,
peer displacement and final-zero-command evidence. Box geometry and the initial
course heading are command-line parameters, so the same analysis is reusable
for offset and angled-obstacle experiments.

The first milestone provides a namespaced state publisher with two transmission
policies:

- `periodic`: fixed-rate publication for the conventional baseline;
- `event`: threshold-triggered publication with a mandatory heartbeat.

The node subscribes to relative `odom`, `ps0`-`ps7` and `tof` topics, so the same
executable can run under `/epuck1` and `/epuck2`. Freshness is measured with a
monotonic clock; the message timestamp uses the ROS clock for simulation and
hardware compatibility.

The validated e-puck IR interface reports a finite clear-space baseline near
0.07 m. Values at or above the configurable `ir_no_detection_m` threshold are
therefore encoded as `+Inf`, preserving the distinction between a nearby return
and no obstacle detected in the useful IR range.

Later milestones add the neighbor cache, loss/latency metrics, short-horizon
prediction and communication-aware collision avoidance.

The `state_monitor` executable now uses `NeighborCache` to record wrap-safe
sequence gaps, duplicates, out-of-order messages, ROS-clock latency and CDR
serialized message size. It writes JSONL raw data and a CSV summary suitable for
the later statistical analysis pipeline.

The cooperative stage adds shared-frame odometry and a decentralized
`cooperative_avoider`. Each robot receives the peer's typed state, predicts the
constant-velocity closest point of approach (CPA), and executes a deterministic
pass-right manoeuvre when the predicted separation violates the safety radius.
The controller starts disarmed and stops on stale state, local emergency, or
maximum runtime. `dual_cooperative_avoidance.launch.py` starts the two symmetric
controller instances under `/epuck1` and `/epuck2`.

Normal velocity commands pass through configurable linear and angular slew-rate
limits. Safety stops bypass the smoother and command zero immediately; a local
obstacle emergency also forces linear velocity to zero immediately. This keeps
the recovery trajectory measurable and smooth without weakening stale-state or
runtime-stop behaviour.

The fused controller uses an explicit safety hierarchy: stale or invalid state,
then calibrated local IR/ToF obstacle avoidance, then communicated peer-state
CPA avoidance, and finally cruise/heading recovery.  The local thresholds are
based on the physical calibration of school e-puck2 5809 (ToF warn/danger at
0.18/0.10 m and IR side warn/danger at 0.052/0.042 m). Release thresholds add
hysteresis.  `enable_local_avoidance:=false` retains a communication-only
ablation condition, while `require_local_sensors:=true` fails safe when neither
fresh IR nor ToF data is available.

Short ToF/IR dropouts during a turn are handled by a direction latch rather
than immediately commanding the opposite heading correction. After the obstacle
leaves the sensor beam, the controller holds its clearance turn, travels a
configurable bypass distance on the diverted heading, and only then enters a
rate-limited `LOCAL_RECOVER` state. This separates obstacle clearance from path
recovery and prevents the left-right oscillation observed in the first static
obstacle diagnostic trial.

## controller_v4_ros_time_consistency (2026-07-17)

`local_obstacle_logic.py`'s `EncounterAvoidanceV4` replaces the earlier
direction-latch bypass with a single unified encounter state machine
(`CLOSED -> DETECT_TURN -> SIDE_TRACK -> PASS_CONFIRM -> RECOVERY_ALLOWED ->
CLOSED`, with a terminal `FAILSAFE`). The state machine itself is
clock-agnostic (it takes `now_s` purely as a parameter); `cooperative_avoider.py`
now supplies that `now_s`, and every other motion/state-transition timer
(`max_runtime_s`, message freshness, command smoothing, `startup_hold_s`),
exclusively from the ROS node clock (`self.get_clock().now()`), which follows
Webots simulation time under `use_sim_time:=true`. `time.monotonic()`/
`time.time()` are reserved for two things only: the external shell-script
watchdog in each pilot's run script, and confirmed diagnostic-only log fields
(e.g. `wall_time=` in the per-transition `TRANSITION` log line, purely for
human log correlation, never read by any decision).

A command-gated turn ledger (`_update_ledger`) only counts a yaw delta as
intentional turning when the previous tick's actual applied angular velocity
exceeds `ledger_command_gate_rps`; smaller deltas are checked against a noise
band and, if exceeded repeatedly, escalate to a `PERSISTENT_DRIFT` safety
stop. This closed the box-corner runaway-turn defect (a single dropped/
grazing sensor reading could previously keep re-arming an unbounded turn) that
the combined wooden-box-plus-moving-peer scenario originally surfaced.
`failsafe_cause` is now an explicit enum (`TURN_LEDGER_CEILING`,
`BYPASS_EXTENSION_CEILING`, `DURATION_CEILING`, `PERSISTENT_DRIFT`) and every
`self.mode` change emits an un-throttled `TRANSITION` log line with the full
ledger/zone/command snapshot.

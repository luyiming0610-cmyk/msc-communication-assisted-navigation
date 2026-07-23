# Safety incident #2: unexpected physical motion (2026-07-23, ~16:09 BST)

**Classification: `UNEXPECTED_PHYSICAL_MOTION` / `SAFETY_INCIDENT_DIAGNOSTIC`.**
A second, separate incident from `safety_incident_unexpected_motion_20260723/SUMMARY.md`.
This document records the reported facts, shutdown evidence, and a
**high-confidence, mechanism-level candidate cause found during this
audit** -- distinct from and stronger than anything found for incident
#1. As with incident #1: **root cause is not being claimed solved.**
The finding below is a plausible, code-confirmed hazard, not proof that
it is what happened, absent continuous `/cmd_vel` recording.

## Reported facts

- Date: 2026-07-23. Approximate time: **~16:09 BST**.
- Robot state: **on the stand, wheels suspended** (not bearing weight)
  -- notably different from incident #1, which was on the ground.
- Direction: mixed/unclear.
- Duration: very brief, well under 1 second.

## Shutdown evidence (as reported by the user, read-only, post-incident)

- `pgrep -af` for physical/HIL process names: only the WSL bridge, PID
  **7961**, remained running at the time of this check (state_publisher,
  guard, controller, virtual peer, wheel test, goal navigator, rosbag:
  none running).
- `/cmd_vel` immediately before bridge shutdown:
  `Publisher count: 0`, `Subscription count: 1` (the bridge itself, the
  sole subscriber).
- Bridge stopped by its exact PID: `kill -INT 7961`.
- Post-shutdown verification: `BRIDGE_STOPPED`;
  `ros2 topic info /cmd_vel` -> `Unknown topic '/cmd_vel'` (topic no
  longer exists at all -- confirms no publisher or subscriber remained).
- The WSL physical command chain is fully stopped. The physical e-puck
  is powered off. The Pi driver/expanded-server windows have been
  stopped.

This is materially stronger shutdown evidence than incident #1 had at
the equivalent point: `/cmd_vel`'s Publisher count was 0 immediately
before shutdown, and the topic ceased to exist entirely afterward,
confirming a clean, total teardown of the physical command path.

## Timing correlation found during this audit

This session's own command history places a **full `colcon test`
run for the `epuck2_comm`/`epuck2_comm_interfaces` packages at
2026-07-23T15:09:10Z (16:09:10 BST)** -- within seconds of the reported
incident time. This run had, until this audit, been treated as
unconditionally safe/offline, on the same assumption applied to every
other unit-test run this session: "it's a unit test, therefore no real
ROS graph, therefore no real-topic contact." **That assumption is
false for part of this suite**, found on inspection below.

## Mechanism-level finding: no ROS_DOMAIN_ID isolation, real `cmd_vel` publisher in test code

- `ROS_DOMAIN_ID` is **not set anywhere** in this environment -- not in
  `~/.bashrc`, not in the current shell environment, not referenced by
  the bridge scripts. Every WSL shell (the physical bridge, the
  state_publisher, any HIL script, and every `colcon test`/`pytest`
  invocation) therefore joins the **same default ROS2 DDS domain
  (0)**, with no isolation between "test" and "real" processes.
- `src/epuck2_comm/epuck2_comm/cooperative_avoider.py` (the frozen,
  production controller, line 289):
  `self.command_publisher = self.create_publisher(Twist, "cmd_vel", 10)`
  -- a hardcoded, unremapped topic name. In every real/HIL launch this
  is remapped via `-r cmd_vel:=cmd_vel_unguarded` (or, in plain N2/N3
  launches, connects straight to the real driver). **No remap is
  applied in the unit tests below.**
- **Complete audit (Phase 1, full sweep of `epuck2_comm`,
  `epuck2_comm_interfaces`, and the HIL study's own `test_hil_*.py`
  files) -- full hazard inventory:**

  | Test file | Node constructed | Topic(s) touched (pre-fix) | Can carry nonzero motion commands? | Isolated (pre-fix)? |
  |---|---|---|---|---|
  | `test_cooperative_avoider_v4_integration.py` | `CooperativeAvoider` | publishes `cmd_vel`; subscribes `state`, conditionally `nav_intent`/peer | **Yes -- drives real nonzero cruise/stop commands** | No |
  | `test_dynamic_speed.py` | `CooperativeAvoider` | publishes `cmd_vel`; subscribes `state`, `nav_intent` | **Yes** | No |
  | `test_dynamic_heading.py` | `CooperativeAvoider` | publishes `cmd_vel`; subscribes `state`, `nav_intent` | **Yes** | No |
  | `test_network_impairment_relay.py` | `NetworkImpairmentRelay` | publishes `state`, `relay_status`; subscribes `state_raw` | No (data relay only, no motion topic) | No |
  | `test_sequence_counter.py` | `SequenceCounterNode` | subscribes `state` only | No (subscriber only) | No |
  | `test_state_publisher_stamp_semantics.py` | `StatePublisher` | publishes `state`; subscribes `odom`, `tof`, `ps0`-`ps7` | No (sensor/state path, not motion) | No |
  | `test_state_publisher_v4_zones.py` | `StatePublisher` (x2 in one test) | same as above | No | No |
  | All `test_hil_*.py` (HIL study, 10 files) | none -- zero `rclpy.init`/`create_publisher`/`create_subscription` calls anywhere in this directory | n/a | No | n/a (no ROS graph at all) |
  | `epuck2_comm_interfaces` | n/a -- no test files in this package | n/a | No | n/a |

  Every other `epuck2_comm` test file (`test_collision_math.py`,
  `test_command_smoothing.py`, `test_encounter_avoidance_v4.py`,
  `test_local_obstacle_logic.py`, `test_neighbor_cache.py`,
  `test_transmission_policy.py`, the `analyze_*`/`bag_analysis`/`pilot_*`
  files) contains no `rclpy.init`, no `Node` construction, and no
  `create_publisher`/`create_subscription` call -- pure-function tests
  only, confirmed by exhaustive grep across the whole directory, not
  spot-checked.

- Only the three `CooperativeAvoider`-constructing files above can
  actually carry a nonzero motion command, because
  `cooperative_avoider.py`'s `command_publisher` is the only
  test-reachable publisher onto anything resembling the real motion
  contract. **This is treated as a HIGH-CONFIDENCE CANDIDATE CAUSE for
  both incidents, not a proven fact** -- command origin remains
  formally NOT_MEASURABLE for both, per the standing rule, because no
  continuous `/cmd_vel` recording exists for either window.
- Because there was no domain isolation, **any of these three tests, if
  run while a real subscriber exists on `cmd_vel` (e.g. the WSL
  bridge), would place genuine, DDS-delivered nonzero velocity commands
  onto the exact topic the bridge relays to the Pi's motors** --
  indistinguishable, from the bridge's point of view, from a legitimate
  command.
- The same `colcon test` run for this package has been executed
  **many times throughout this entire session**, including at least
  once within incident #1's reported window (2026-07-23T14:31:08Z /
  15:31:08 BST, inside the 15:15-15:35 BST window), every time
  previously assumed safe without this check.

**This is not proof of causation for either incident** -- no continuous
`/cmd_vel` recording exists for either window, so, per the standing
rule, command origin remains formally **NOT_MEASURABLE** for both. It
is, however, a materially stronger, code-level-confirmed candidate
mechanism than anything found in incident #1's audit, and it is a real,
present hazard independent of whether it explains either incident: **running
this package's `colcon test` suite while any part of the physical
command path is live is not actually safe**, contrary to how it was
treated at every prior verification step this session.

## Defense-in-depth fix implemented (Phase 2, after the complete Phase 1 audit above)

- Every test file constructing `CooperativeAvoider`, `StatePublisher`,
  `NetworkImpairmentRelay`, or `SequenceCounterNode` now pushes a
  private ROS namespace (`-r __ns:=/pytest_isolated`) in every
  `rclpy.init()` call, isolating every topic that node touches
  regardless of which one it is.
- `src/epuck2_comm/test/conftest.py` (new) forces a dedicated,
  non-physical `ROS_DOMAIN_ID` (`89`) for the entire pytest session via
  `pytest_configure`, independent of the per-file remaps -- a second,
  structural layer.
- `test_pytest_topic_isolation.py` (new, static) fails if any test file
  constructing one of the four hazard classes lacks the namespace
  remap, or if any remap target equals a real hardware topic name.
- `test_cooperative_avoider_topic_isolation_runtime.py` (new, runtime)
  proves, by actually spinning a namespaced `CooperativeAvoider` next
  to an unremapped observer node in the same process, that a genuine
  nonzero command arrives on `/pytest_isolated/cmd_vel` and never on
  bare `cmd_vel`.
- `run_isolated_test_suite.sh` (new) is now the only sanctioned way to
  run either test suite: refuses to run if any physical/HIL process is
  detected, checks the real `/cmd_vel` publisher count in the default
  domain before and after, and runs both suites only inside the
  isolated `ROS_DOMAIN_ID` in between.
- `HIL_LAB_RUNBOOK.md` and `HIL_SAFETY_CHECKLIST.md` updated to
  prohibit direct `pytest`/`colcon test`/`python3 -m unittest`
  invocation whenever the physical stack could be live.
- No controller or guard logic was weakened or otherwise changed.

## Explicitly not done / not concluded

- Root cause for neither incident is solved or claimed solved --
  command origin remains formally NOT_MEASURABLE for both. The
  same-domain unremapped test publisher is a high-confidence candidate
  cause, not a proven one.
- No pytest, colcon, or HIL script was run until this static audit and
  the isolation fix above were both complete; the only test execution
  after that point used `run_isolated_test_suite.sh`.
- No physical process was restarted at any point during this audit or
  fix.

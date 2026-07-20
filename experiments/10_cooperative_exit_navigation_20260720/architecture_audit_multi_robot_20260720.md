# Multi-robot architecture audit (read-only, 2026-07-20)

Read-only investigation performed before any design or code change, per
instruction, to determine whether the current codebase genuinely supports
3 or 4 simultaneously-communicating robots, or only 2. No file referenced
below was modified during this audit.

## 1. `cooperative_avoider.py` — exactly ONE peer subscription, scalar storage

- `peer_state_topic` is a single string parameter (default `/epuck2/state`),
  not a list (`cooperative_avoider.py:45,98`).
- Exactly one peer subscription is created:
  `self.create_subscription(EpuckState, self.peer_topic, self._peer_callback, 20)`
  (`cooperative_avoider.py:258-261`), gated only by `enable_peer_avoidance`.
- Peer state is stored as two scalars, not a collection:
  `self.peer_state = None` / `self.peer_received = None`
  (`cooperative_avoider.py:207,209`), overwritten in place by
  `_peer_callback` (`cooperative_avoider.py:317-319`).
- `_metrics()` (`cooperative_avoider.py:423-440`) calls
  `closest_point_of_approach(...)` exactly once, against `self.peer_state`
  — never a loop over peers.
- `_risk()` (`cooperative_avoider.py:442-448`) takes one `CpaResult` and
  returns one boolean — no aggregation across multiple peers exists.
- Freshness (`_fresh()`, `cooperative_avoider.py:351-352`) is checked
  against the single scalar `self.peer_received`.
- **Conclusion: for 3 or 4 robots, each robot would need simultaneous CPA
  risk against 2 or 3 peers respectively. This does not exist today and
  requires a real code change (see section 8, minimal-change design).**

## 2. `local_obstacle_logic.py` — no peer-count assumption

Robot-count-agnostic by construction: consumes only this robot's own
IR/ToF distances and zone snapshot (`decide_local_obstacle`,
`EncounterAvoidanceV4`). No numeric literal ties it to 2 robots. No change
needed for any N.

## 3. `collision_math.py` — pairwise math, N-agnostic

`closest_point_of_approach(own..., peer..., horizon_s)` and
`collision_risk(result, ...)` (`collision_math.py:49-107`) both operate on
exactly one `(own, peer)` pair. Nothing here assumes a global robot count
— the function is naturally callable once per peer pair for any N. It is
simply never called in a loop today (see section 1). **No change needed
to this module; it is the reusable building block for a multi-peer
extension.**

`local_to_global(x_m, y_m, yaw_rad, origin_x_m, origin_y_m, origin_yaw_rad)`
(`collision_math.py:25-38`) is the shared-frame transform already used by
`state_publisher.py` (see section 4) — confirms `EpuckState.x_m/y_m` are
ALREADY published in a common experiment-wide frame per robot, via each
robot's own `origin_x_m`/`origin_y_m`/`origin_yaw_rad` launch parameters.
This is directly reusable for an N-robot goal-region check and a full
pairwise-distance matrix, with no new coordinate-transform code needed.

## 4. `state_publisher.py` / relay / `sequence_counter.py` — already namespace-parameterized

None of these three files hardcode `epuck1`/`epuck2`. `robot_id` and the
shared-frame origin are declared ROS parameters
(`state_publisher.py:58-59,71-72`); namespacing is supplied externally at
launch time via `-r __ns:=/epuckN`. `network_impairment_relay.py`'s own
module docstring states the design intent explicitly: "one relay instance
per robot." `sequence_counter.py` has no robot-identity literal at all.
**These three are already N-robot-ready as building blocks — launching a
3rd or 4th instance is a matter of adding more launch entries, not
modifying these files.**

## 5. Webots world/launch — exactly 2 hardcoded, but same-pattern extension

- World files used by prior formal trials (`two_epuck_head_on_clean_world.wbt`
  and siblings under `experiments/cooperative_avoidance_20260716/config/`)
  declare exactly 2 `E-puck` nodes each. All follow the `two_epuck_...`
  naming/geometry pattern.
- `dual_namespaced_launch.py` (the Webots-side ROS2 launch description)
  hardcodes exactly 2 driver/spawner/`WaitForControllerConnection` entries
  via individually-named calls, not a loop over a robot list.
- `run_comm_baseline_formal_controllers.py` has a reusable
  `make_controller(namespace, robot_id, peer_topic, desired_heading)`
  factory but calls it exactly twice, hardcoded.
- `run_objective5_impairment_matrix_trial.sh` hardcodes exactly 2
  `state_publisher` invocations and a `for ns in epuck1 epuck2` loop.
- **Adding a 3rd/4th robot to the world file and to the per-robot
  launch/orchestration entries (state_publisher, relay, sequence_counter,
  controller) is mechanical repetition of an already-parameterized
  pattern — no architectural blocker there.** The one real blocker is
  `cooperative_avoider.py`'s single-peer design (section 1).

## 6. Existing tests — none target a specific N-robot count

19 test files under `src/epuck2_comm/test/`; none named with
"peer"/"two_robot"/"dual". `test_cooperative_avoider_v4_integration.py` is
the closest (exercises the single-peer control loop end-to-end) but no
test asserts a specific robot count.

## 7. `EpuckState.msg` fields (context only, unchanged)

`version`, `source`, `robot_id`, `sequence`, `stamp`; `x_m`, `y_m`,
`yaw_rad`, `linear_velocity_mps`, `angular_velocity_rps`;
`front_distance_m`, `left_distance_m`, `right_distance_m`,
`obstacle_status`, `validity_flags`; `left_front_m`, `left_mid_m`,
`left_rear_m`, `right_front_m`, `right_mid_m`, `right_rear_m`; constants
`PROTOCOL_VERSION`, `SOURCE_*`, `OBSTACLE_*`, `FLAG_*`. No change proposed.

## 8. Conclusion: what N2 needs vs what N3/N4 need

**N2 (2 robots) needs NO controller code change.** The existing
single-peer `cooperative_avoider.py` already handles exactly 2 robots
(1 peer each) — this is precisely what Conditions A-D already exercised.
The only NEW work for N2 is: (a) a goal/exit-region task-completion
criterion (does not exist anywhere today — Conditions A-D only measure
CPA-avoidance behavior, never "reached a common destination"), and (b) a
COMM_OFF mode, which the controller ALREADY supports via the existing
`enable_peer_avoidance=false` parameter (`cooperative_avoider.py:61,118-120,258`)
— confirmed already used and load-bearing in the frozen ablation design
(Phase 4's local-only condition). No new controller parameter is needed
for COMM_OFF.

**N3/N4 (3-4 robots) require a genuine multi-peer extension to
`cooperative_avoider.py`** — a real code change, not orchestration
repetition. See `multi_peer_extension_design_20260720.md` for the minimal
design (not implemented this round; N3/N4 pilots are not authorized yet).

## 9. Separate, critical finding: no closed-loop goal-seeking exists at all

`cooperative_avoider.py`'s `CRUISE` mode steers toward a FIXED
`desired_heading_rad` parameter (`cooperative_avoider.py:679,771-773`), not
toward a goal position. It is direction-holding, not position-seeking:
after a lateral deviation (from CPA avoidance or local obstacle avoidance),
the robot recovers to the SAME fixed heading, not a recomputed
bearing-to-goal — so it lands on a path parallel to, but offset from, its
original line, not back on a line toward a point goal.

This is a genuine architectural gap for ANY "reach a common exit region"
task, independent of robot count. Two ways to close it:

- **(a) Fixed-heading + wide goal region (zero controller change)**: place
  each robot's start pose and `desired_heading_rad` so its straight-line
  path already points at a sufficiently WIDE shared exit zone (wide enough
  to absorb the bounded lateral drift the existing avoidance layers can
  introduce — bounded today by `local_bypass_distance_m`≈0.08m and
  `local_v4_required_lateral_offset_m`≈0.07-0.10m, i.e. drift is on the
  order of 0.1-0.4m in the worst case seen in prior formal trials). No
  code change; purely a scenario-geometry design constraint.
- **(b) True closed-loop goal-seeking (recompute bearing-to-goal every
  tick)**: a real, non-trivial controller change (replacing the fixed
  `desired_heading` reference with `atan2(goal_y - own_y, goal_x - own_x)`
  in the relevant branches) — deeper than the multi-peer question and not
  proposed for this round.

**Decision for this round (see design doc): approach (a).** Reuses the
frozen `cooperative_avoider.py` completely unmodified for N2 — satisfies
the explicit "最小改动" instruction in the strongest possible sense (zero
lines changed in the frozen controller) at the cost of constraining
Phase 3/N2 scenario geometry to near-straight-line reachability with a
wide goal zone. This constraint is stated explicitly in the design doc's
scenario-fairness section so it is never silently forgotten when N3/N4
(which may need approach (b) or the multi-peer extension to remain
geometrically fair with more start positions) are designed later.

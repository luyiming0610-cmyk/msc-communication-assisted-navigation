# Stage 1 design: Two-Robot Shared Exit Navigation With/Without Communication

Phase 1 deliverable (read-only audit + design only — **no implementation
in this document**, per instruction). Supersedes the central-rendezvous
scenario as the active research vehicle; that work is preserved as
Stage 0 (see `STAGE_CLASSIFICATION.md`).

## 1. Research question (restated, precisely)

Does inter-robot communication help two robots complete a **shared
exit-finding task** faster and more safely than no communication —
specifically, does giving one robot (which discovers the exit) the
ability to tell the other robot (which does not) where the exit is,
change the *outcome*, not just the collision-avoidance behavior along
the way. This requires **asymmetric exit-discovery information**: if
both robots know the exit from t=0, communication can only ever show up
in the avoidance metrics, never in "did communication help find the
goal" — exactly the ambiguity the supervisor flagged.

## 2. Read-only audit findings

- **Arena**: the reused `RectangleArena` is `floorSize 1.5 1.5`, i.e.
  usable coordinates roughly `x,y ∈ [-0.75, 0.75]` before hitting the
  wall. Confirmed by reading `two_epuck_head_on_clean_world.wbt` /
  `two_epuck_cooperative_exit_n2_world.wbt` (Stage 0's world, unmodified,
  not reused for Stage 1's scene geometry — a new world file is needed
  since the exit must be at an edge/corner, not the center).
- **`cooperative_avoider.py`'s heading is fixed, not closed-loop**
  (already found during the original architecture audit,
  `architecture_audit_multi_robot_20260720.md`): `desired_heading_rad`
  is a declared ROS parameter, read ONCE in `__init__`
  (`self.desired_heading = float(self.get_parameter("desired_heading_rad").value)`,
  line 100) and never re-read afterward. CRUISE/RECOVER always steer
  toward this one fixed value. **This is why a genuine goal-directed
  navigation layer requires a real (minimal) code change** — see
  Section 5.
- **`nominal_speed_mps` (default 0.025) IS already a mutable, declared
  ROS launch parameter** (line 48), independent of the CPA formula,
  `safety_radius_m`, and local IR/ToF thresholds. At the Stage-0 default
  it is far too slow for a meaningful point-to-point exit-finding task
  over arena-scale distances (0.025 m/s × 28s max_runtime ≈ 0.7m
  one-way, no margin for search + avoidance + hold). Since it is not one
  of the three explicitly frozen items (CPA formula / safety radius /
  local sensor thresholds), it may be pre-registered at a different,
  frozen, OFF/ON-identical value for this study. Proposed:
  `nominal_speed_mps=0.06` (still well below the e-puck's physical
  ~0.13 m/s max), applied identically to both conditions before any
  pilot runs — never adjusted afterward.
- **Existing `EpuckState.msg` is PROTOCOL_VERSION=1, frozen** (commit
  `b5a0351`) and is a per-robot *kinematic state* broadcast (position,
  yaw, velocity, sequence) — not a task-level, one-shot announcement.
  It is reused unmodified for CPA avoidance exactly as in Stage 0; the
  exit-discovery message is a **separate, new, minimal message**
  (Section 4), never piggybacked onto `EpuckState`'s fields.
- **`task_completion_monitor.py`, `verify_state_velocity_settled.py`,
  the orchestrator's `TASK_COMPLETE_GOAL` stop path, and the
  `task_completion_analyzer.py` goal-hold/pairwise-safety math are all
  reused unmodified** — Stage 0 already proved these work on both the
  local-avoidance and CPA-avoidance code paths. Only the goal region's
  center/radius and the addition of exit-discovery timing fields change.

## 3. Scene design (frozen, pre-registered, identical OFF/ON except communication)

**New, additive world file** `two_epuck_shared_exit_n2_world.wbt` (does
NOT modify `two_epuck_head_on_clean_world.wbt` or Stage 0's
`two_epuck_cooperative_exit_n2_world.wbt` — both remain untouched
evidence).

- **Exit location**: a corner region, center **(0.55, 0.55)**, radius
  from wall ≈0.20m (arena half-extent 0.75). A visible "door frame":
  two small, non-colliding (no `boundingObject`) post markers at
  `(0.45, 0.65)` and `(0.65, 0.45)`, straddling the diagonal approach
  into the corner, plus a green, non-colliding disc/rectangle marker
  (radius 0.15m, matching the judged goal region) at the exit center —
  visually distinct from Stage 0's central marker (different color/
  shape convention: a rectangle behind two posts, not a bare disc, to
  make it unambiguous in any screenshot that this is a different scene).
- **Goal/exit-hold region** (what `task_completion_monitor.py` judges):
  `center=(0.55, 0.55), radius=0.15m`. Does not overlap either robot's
  start pose (see below); a robot must travel a genuine navigable path
  to enter it.
- **Obstacle**: one small, real (WITH `boundingObject`, genuine physics/
  collision) box at **(0.15, 0.15)**, ~0.08m × 0.08m × 0.10m — placed to
  intersect Robot B's direct diagonal path to the exit (forcing a
  detour, genuine local-avoidance engagement) while leaving Robot A's
  more direct, roughly-horizontal path clear. This is asymmetric by
  design (Robot B's task is harder along more than one axis, matching
  "genuine coordination need... not artificially biasing COMM_OFF to
  fail" — the obstacle affects search/navigation difficulty, not
  communication access) and is frozen before any pilot runs, never
  adjusted post-hoc.
- **Start poses** (asymmetric distance to exit, per instruction):
  - **Robot A ("informed", discovers the exit early)**: `(0.10, 0.55, yaw=0.0)`.
    Straight-line distance to exit center: **0.45m**.
  - **Robot B ("uninformed", must search)**: `(-0.20, -0.20, yaw=0.785)`
    (yaw ≈ 45°, facing generally toward the arena interior/search
    start). Straight-line distance to exit center: **1.06m** (≈2.4×
    Robot A's distance).
  - Neither start pose is inside the goal region (nearest is Robot A at
    0.45m, well outside the 0.15m-radius region — this is the exact
    failure mode Stage 0's PILOT04 exposed and fixed; Stage 1 verifies
    it does not recur before any pilot runs, see Section 8 checklist).
- **`safety_radius_m=0.14`** (unchanged, frozen, per instruction — no
  separate design review has been conducted to justify a different
  value, so it stays at the Objective-5/Stage-0 value).
- **`collision_contact_distance_m=0.07`** (unchanged from Stage 0).
- **`goal_hold_time_s=2.0`** (unchanged from Stage 0 — already proven
  achievable and non-trivial).
- **`max_runtime_s=60.0`** (increased from Stage 0's 28.0 — a genuine
  point-to-point navigation task over up to 1.06m at 0.06 m/s, plus
  Robot B's pre-discovery search sweep and an avoidance detour, needs
  materially more time than the Stage-0 encounter-and-recover pattern;
  pre-registered before any pilot runs, applied identically to both
  conditions, purely a failure backstop — never read as success per the
  existing rule in `task_completion_analyzer.build_task_verdict`).
- **`nominal_speed_mps=0.06`** (Section 2 justification).

OFF and ON share, byte-identical: world file, obstacle, start poses,
exit location/marker, `safety_radius_m`, `collision_contact_distance_m`,
`goal_hold_time_s`, `max_runtime_s`, `nominal_speed_mps`, local IR/ToF
avoidance logic and thresholds (frozen, untouched), Robot B's
pre-discovery search waypoint sequence (Section 6). The **only**
difference is whether Robot A's exit-discovery is ever communicated to
Robot B.

## 4. Exit-announcement message design

**New message, NOT a modification of `EpuckState.msg` (PROTOCOL_VERSION=1
stays frozen and untouched)**: `epuck2_comm_interfaces/msg/GoalAnnouncement.msg`.

```
uint32 protocol_version      # =1 for this message type, independent counter from EpuckState's
uint32 source_robot_id       # which robot discovered/is announcing the exit
uint32 sequence              # monotonic per-source counter, for missing/duplicate/out-of-order detection
builtin_interfaces/Time production_stamp   # when the announcing robot computed this message
string goal_id                # e.g. "shared_exit" -- future-proofs against multiple named goals
float64 goal_x_m
float64 goal_y_m
bool valid                    # false is never expected to be published in this study, but keeps the
                               # field explicit rather than relying on message presence/absence alone
```

**Why a new message instead of reusing `geometry_msgs/PointStamped`**:
`PointStamped` (`header{stamp, frame_id} + point{x,y,z}`) has no
`source_robot_id`, no `sequence`, and no explicit `valid` flag. This
project's own communication-metrics analyzers (`analyze_comm_performance.py`,
`sequence_counter.py`) already depend on a per-source sequence number to
compute missing/duplicate/out-of-order counts — the exact metrics
Section 6 (communication contribution) requires for the exit
announcement channel. Retrofitting that onto `PointStamped` (e.g.
encoding robot id in `frame_id` as a string) would be a fragile,
undocumented convention; a small, explicit message matching the
project's existing protocol-versioning discipline is more auditable and
consistent with how `EpuckState.msg` itself is designed. `goal_id` is a
single extra field for future-proofing (not required for N2, but avoids
an incompatible message revision if a later study needs it) and is
frozen at `"shared_exit"` for this study.

**COMM_ON wiring**: Robot A's controller stage publishes exactly one
`GoalAnnouncement` (on the topic `/epuck1/goal_announcement`, or the
appropriate namespace) the instant it determines it has "discovered"
the exit (Section 5) — republished at a low, bounded rate (e.g. once
per second) for the remainder of the trial for robustness, not a single
fire-and-forget message, so a single dropped message cannot silently
strand Robot B; each republish increments `sequence`. Robot B subscribes
and, on the FIRST valid, sequence-fresh message, records
`robot_b_search_to_goal_switch_time_s` and switches from search mode
to goal-directed navigation. **COMM_OFF launches no publisher/subscriber
for this topic at all** (same "strictly stronger than zero-impairment"
principle already used for Stage 0's `state`/`state_raw` wiring) — Robot
B has no code path that could ever receive exit information under
COMM_OFF, not merely an unused subscription.

## 5. Minimal goal-directed navigation layer

**Required code change (minimal, explicitly flagged for confirmation
before Phase 2 implementation)**: `desired_heading_rad` is currently
read once at `__init__` and never updated (Section 2). To support
genuine closed-loop navigation without touching the CPA formula, safety
radius, or local sensor thresholds, `cooperative_avoider.py` needs an
`add_on_set_parameters_callback` that updates `self.desired_heading`
live when an external process calls `set_parameters` on the already-
declared `desired_heading_rad` parameter. This is the ONLY proposed
change to the frozen controller file — no new modes, no new thresholds,
no change to `_risk()`/`_metrics()`/CPA math/local-avoidance latch logic.
Priority order (unchanged, already true today): CPA avoidance and
local IR/ToF safety-stop branches are checked and can override the
CRUISE/heading branch every tick, exactly as they do now — the goal
navigator only ever supplies a *desired direction*, never a command,
and never bypasses `SAFE_STOP_STALE` / `SAFE_STOP_LOCAL_SENSORS` /
`SAFE_STOP_INVALID_ODOM`.

**New, additive node** `goal_directed_navigator.py` (mirrors
`task_completion_monitor.py`'s pattern — external, read-only observer of
`/epuckN/state`, plus one small write path): for a robot with a known
target `(goal_x_m, goal_y_m)` (either the frozen exit location for
Robot A / COMM_OFF's Robot B's active search waypoint, or the
COMM_ON-received exit location for Robot B post-switch), computes
`desired_heading = atan2(goal_y - y, goal_x - x)` from the robot's own
`/epuckN/state` position, and periodically (e.g. 2 Hz — slow enough not
to fight the command-smoother's accel/decel limits, frozen and
unchanged) calls `set_parameters` on the running `cooperative_avoider`
node to update `desired_heading_rad`. It never publishes `cmd_vel` and
never reads Supervisor ground truth.

Arrival: once inside the exit region, `task_completion_monitor.py`
already handles hold-time judgment and orchestrator shutdown exactly as
in Stage 0 — no new "arrived" logic needed in the navigator or
controller.

## 6. Robot B's deterministic pre-discovery search strategy

**Frozen, identical OFF/ON, pre-registered before any pilot runs**: a
fixed sequence of waypoints, visited in order, each held as the current
`desired_heading` target until Robot B is within a small arrival radius
(e.g. 0.10m) of it, then advancing to the next:

1. `(-0.20, -0.20)` (start, immediate) →
2. `(0.10, -0.35)` →
3. `(0.30, 0.10)` →
4. `(0.55, 0.55)` (the exit itself, as the FINAL waypoint — Robot B
   eventually finds the exit on its own even under COMM_OFF, since
   `task_success` must remain reachable without communication; this is
   what makes COMM_OFF a genuine, completable baseline rather than a
   condition rigged to fail).

Under **COMM_ON**, this sequence is identical, EXCEPT that if a valid
`GoalAnnouncement` arrives while Robot B is still mid-sequence, it
immediately abandons the current waypoint and switches to goal-directed
navigation toward the announced exit location — recorded as
`robot_b_search_to_goal_switch_time_s`. Under **COMM_OFF**, Robot B
always completes the full waypoint sequence regardless of Robot A's own
progress.

## 7. Metrics (pre-registered, computed by extending `task_completion_analyzer.py` — additive, not replacing Stage 0's existing tested functions)

Task: `all_robots_exited` (= Stage 0's `all_robots_reached_goal`,
renamed at the reporting layer for this study's vocabulary),
`completed_robot_count`, `individual_completion_time_s`, `makespan_s`,
`timeout_count`, `task_success`.

Communication contribution (**COMM_ON only**; COMM_OFF reports the
literal string `NOT_APPLICABLE` for every field below, never a fake
`0`, per instruction): `exit_discovery_time_s` (when Robot A's own
controller/detection logic determines it has found the exit),
`exit_announcement_tx_time_s` (first publish), `exit_announcement_rx_time_s`
(Robot B's first valid receipt), `robot_b_search_to_goal_switch_time_s`,
message age (tx-to-rx latency, republish-aware), missing/duplicate/
out-of-order counts (via `GoalAnnouncement.sequence`, mirroring
`sequence_counter.py`'s existing, proven approach).

Safety (unchanged from Stage 0, reused): `collision_count`,
`minimum_pairwise_distance_m`, `safety_margin_m`,
`local_safety_intervention_count`, `stale_state_stop_count`/
`failsafe_count`.

Efficiency (unchanged from Stage 0, reused): `individual_path_length_m`,
`total_path_length_m`, `cumulative_heading_change_rad`,
`turn_reversal_count`, `stop_duration_s`.

## 8. Halting checklist (any one condition -> stop immediately, no auto-continue)

Start pose already inside the goal region; goal is a central rendezvous
point (not a real edge/corner exit); COMM_OFF's Robot B receives exit
information through any channel; Supervisor participates in real-time
navigation; trial ends via `max_runtime_s`; `DATA_VALIDITY=INVALID`;
any collision; analyzer parameters hardcoded/mismatched against the
frozen scene; OFF/ON scenario found to differ in anything other than
communication access.

## Revision 1 (2026-07-20, second review round)

Five corrections applied before any implementation:

1. **`nominal_speed_mps=0.04`** (not 0.06). Reason: local IR/ToF safety
   behavior was primarily validated at the lower speed; raising it
   introduces an untested safety variable, and simulation-vs-physical
   Pi-puck consistency needs to stay in view. Frozen, identical OFF/ON,
   never adjusted after seeing pilot results.
2. **`max_runtime_s` computed from geometry, not guessed.** Computed by
   `verify_shared_exit_geometry.py` (Section "Verified geometry" below):
   Robot B's full worst-case waypoint search path is 1.2535m; at
   0.04 m/s that is 31.34s. Adding `startup_hold_s=5.0s` (existing
   controller default), a 20% avoidance-detour margin (~6.3s),
   `goal_hold_time_s=2.0s`, and an orchestrator settle/shutdown buffer
   (~3s) gives ≈47.6s; rounded up to **`max_runtime_s=55.0`** for a
   ~15% extra buffer.
3. **Goal-directed heading interface is a ROS2 topic, not a parameter
   callback.** A new `goal_navigator.py` node computes
   `desired_heading_rad` from `/epuckN/state` and the current target
   (frozen local knowledge or a received announcement), and publishes it
   on a topic as a strongly-typed `NavigationIntent` message (Section
   "Navigation-intent message" below) — not raw floats, not shoved into
   an unrelated field. `cooperative_avoider.py` gains one new,
   **default-disabled** subscription (`enable_dynamic_heading=false`
   preserves A-D behavior byte-for-byte); when enabled, the subscription
   has its own freshness check (mirroring the existing `_fresh()`
   pattern already used for own/peer state) and a well-defined fallback
   on stale/invalid input — see Section "Controller subscription
   design". CPA avoidance, local IR/ToF safety-stop, and stale-state
   handling remain strictly higher priority; the navigation-intent
   subscription can never itself stop or override them.
4. **Geometry independently verified before any code was written** —
   see below; goal region, gate posts, obstacle, and both start poses
   checked against the real arena half-extent (0.75m, from
   `RectangleArena floorSize 1.5 1.5`) and the e-puck2's documented
   0.037m body radius (GCtronic spec, 70mm diameter). All fail-fast
   checks pass (`verify_shared_exit_geometry.py`, script committed,
   re-runnable, output below is exact, not hand-computed).
5. **Metrics are trial-relative, not absolute sim-clock**, and **all
   scene parameters are read from `shared_exit_frozen_params.json` by
   the analyzer, never hardcoded** — both were real defects caught in
   Stage 0 (`makespan_s` was reported as raw absolute sim time in the
   first Stage 0 report; the scratchpad analyzer script hardcoded a
   copy of the goal radius that drifted out of sync with the
   orchestrator's own frozen value after PILOT04's fix). The Stage 1
   analyzer normalizes every timestamp to each trial's own first sample
   and loads all scene geometry from the frozen-params file at runtime.

### Verified geometry (from `verify_shared_exit_geometry.py`, all checks PASS)

| Quantity | Value |
|---|---|
| Arena half-extent | 0.75m (`RectangleArena floorSize 1.5 1.5`) |
| Robot body radius | 0.037m (e-puck2 70mm diameter, GCtronic spec) |
| Goal/exit region | center (0.50, 0.50), radius 0.10m |
| Wall clearance (goal region + robot radius) | 0.113m on both +x/+y walls, 0.217m to the corner point |
| Gate posts (visual only, non-colliding) | (0.244, 0.456) and (0.456, 0.244); gate width 0.300m ≈ 4.05× robot diameter |
| Obstacle (real, colliding) | center (0.15, -0.15), 0.08×0.08×0.10m box |
| Robot A start | (0.10, 0.55); 0.403m from goal center; obstacle clearance 0.688m from A's direct path |
| Robot B start | (-0.20, -0.20); 0.990m from goal center; obstacle intersects B's waypoint path (min distance 0.000m ≤ 0.077m threshold — confirmed, not assumed) |
| Robot B search waypoints | (-0.20,-0.20) → (0.05,-0.35) → (0.25,0.05) → (0.50,0.50); total path 1.2535m |

Neither start pose is inside the goal region (0.403m and 0.990m vs.
0.10m radius) — the exact failure mode Stage 0's PILOT04 exposed does
not recur here, verified computationally before any pilot runs.

A real Webots top-down screenshot of the built world (not this computed
diagram) will be captured and included before Phase 3 pilots run, per
instruction.

### Navigation-intent message

`epuck2_comm_interfaces/msg/NavigationIntent.msg` (new, independent of
`EpuckState.msg` PROTOCOL_VERSION=1, which is not modified):

```
uint32 protocol_version
uint32 source_robot_id
uint32 sequence
builtin_interfaces/Time production_stamp
float64 desired_heading_rad
bool valid
```

Published by `goal_navigator.py` at a modest, bounded rate (2 Hz) on
each robot's own namespaced topic (e.g. `/epuck1/nav_intent`) — a
robot's navigator publishes only for that same robot's controller to
consume (never cross-robot; this is not a communication channel between
the two robots, `GoalAnnouncement` is that channel).

### Controller subscription design (`cooperative_avoider.py`, the one proposed code change)

- New parameters: `enable_dynamic_heading` (bool, default `false`),
  `nav_intent_timeout_s` (float, default matching the existing
  `peer_timeout_s` pattern).
- When `enable_dynamic_heading=false` (the default, and Objective 5 A-D
  and Stage 0's exact configuration): no subscription is created, no
  behavior changes — byte-for-byte identical to today.
- When `true`: subscribes to `nav_intent` (relative topic, own
  namespace). On each message, if `valid` and the message's own
  `production_stamp` is fresh (same `_fresh()` freshness pattern already
  used for own/peer `EpuckState`), updates `self.desired_heading`. If no
  message has ever arrived, or the latest one is stale or `valid=false`,
  the controller **falls back to the launch-time default
  `desired_heading_rad` parameter value** (already declared today) —
  never holds a stale heading as if it were current, and never stops
  the robot solely because of heading staleness. CPA avoidance, local
  IR/ToF safety-stop, and `SAFE_STOP_STALE`/`SAFE_STOP_INVALID_ODOM`
  remain the only branches that can halt the robot; heading input is
  advisory only, exactly as `desired_heading_rad` already is today.

### Communication-condition definitions (restated precisely)

**`N2_EXIT_COMM_OFF`**: Robot A navigates directly toward its known exit
location (own local knowledge, `enable_dynamic_heading=true` fed by its
own `goal_navigator` targeting the frozen exit coordinates — this is
not "communication", it is Robot A's own a-priori information). Robot B
runs the frozen deterministic waypoint search, `enable_dynamic_heading=true`
fed by its own `goal_navigator` targeting its current waypoint, ending at
the exit coordinates as its final waypoint. **No `GoalAnnouncement`
publisher or subscriber is launched for either robot** — Robot B has no
code path that could ever receive exit information, not merely an
unused topic. **Peer CPA avoidance is disabled** (`enable_peer_avoidance=false`,
matching Stage 0's COMM_OFF definition) — each robot uses only its own
state and local IR/ToF avoidance.

**`N2_EXIT_COMM_ON`**: Robot A additionally publishes `GoalAnnouncement`
(own exit knowledge) at a bounded rate from the start of the trial.
Robot B subscribes; on the first valid, fresh announcement, its
`goal_navigator` switches its target from the current search waypoint
to the announced exit coordinates (recorded as
`robot_b_search_to_goal_switch_time_s`), and continues navigating with
`enable_dynamic_heading=true`. **Peer CPA avoidance is enabled**
(`enable_peer_avoidance=true`) for both robots. Local IR/ToF avoidance
stays enabled in both conditions, unchanged.

**Stated limitation (for the paper, not re-litigated here)**: COMM_ON
combines two mechanisms — exit-information sharing AND peer CPA
coordination. The reported COMM_ON vs. COMM_OFF effect is their combined
effect; no ablation isolating one from the other is run in this phase.

## 9. Execution plan

- **Phase 1 (this document + frozen params) — COMPLETE.**
- **Phase 2**: implement `GoalAnnouncement.msg`, the minimal
  `cooperative_avoider.py` parameter-callback change (Section 5, flagged
  for explicit confirmation before writing code), `goal_directed_navigator.py`,
  the new world file + visual markers, the frozen-waypoint search
  controller for Robot B, the exit-discovery/announcement publisher for
  Robot A, and the extended analyzer fields (Section 7). Full unit +
  integration test coverage; all 165 existing colcon tests and all 46
  existing N2 pure-Python tests must remain passing.
- **Phase 3**: exactly 2 exclusionary pilots
  (`N2_EXIT_COMM_OFF_EXCLUSIONARY_PILOT01`,
  `N2_EXIT_COMM_ON_EXCLUSIONARY_PILOT01`), analyzed and process-cleaned
  in strict sequence, then STOP.
- **Stage 2/3**: formal trials / N3+N4 — not started, not scheduled.

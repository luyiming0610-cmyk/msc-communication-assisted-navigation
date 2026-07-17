# controller_v3_unified_encounter: design for the front+side compound overturn defect

Date: 2026-07-17 (revision 6.2 — second audit pass: the `pilot_a2`-replay
test is reframed as a non-safety-certifying counterfactual trend check,
`FAILSAFE` is made a hard latch with no automatic exit, and HOLD/CREEP
acceptance semantics are made explicit and binding)
Status: **design proposal, not yet implemented**. No code changed, no
scenario changed, no new pilot run. Direct response to the `pilot_a2`
collision (`controller_v2_local_latch_20260717`, bag
`controller_v2_local_latch_20260717_static_box_pilot_a2`, box clearance
`-0.0073 m`, geometric collision confirmed). Per reviewer instruction,
once this revision is confirmed, implementation of the `v3` safety
prototype may proceed directly (tests first, reported before any pilot is
run — §10, §13).

## 1. What `pilot_a2` proves and does not disprove

The forensic timeline (rebuilt from raw `/epuck1/state` and
`/epuck1/cmd_vel`, not the throttled controller log) confirms:

- The `controller_v2_local_latch_20260717` side-lane budget worked exactly
  as designed: `LOCAL_LEFT_SIDE`/its `LOCAL_CLEARANCE` tail accumulated
  ~`0.465 rad` (under the `0.50 rad` candidate cap) before `CAPPED_BYPASS`
  correctly froze `angular_rps` at exactly `0.0` — confirmed by yaw staying
  pinned at `-0.836 rad` for the remaining ~6+ seconds of the run.
- `LOCAL_FRONT_DANGER` fired correctly (raw `front_distance_m` genuinely
  `≤0.100 m` for ~0.68 s) and its own `LOCAL_CLEARANCE` continuation was a
  genuine latch continuation, not a false "clear" reading — front really
  did read `1.099 m` the instant it cleared.
- The `LOCAL_LEFT_SIDE` trigger that followed was pointed at the **same**
  box corner (monotonic `left_distance_m` trace), not a new obstacle.
- The front episode alone produced **~0.54 rad** of yaw deflection before
  the side lane's own, independently-budgeted `~0.465 rad` was added on
  top — two independently-granted turn allowances compounding within one
  physical encounter.
- Geometric clearance was already **negative** (`-0.0046 m`) by the time
  `CAPPED_BYPASS` engaged; it worsened to `-0.0073 m` while `CAPPED_BYPASS`
  drove forward at `0.012 m/s`.
- **New in this audit pass**: re-examining the raw `left_distance_m` trace
  through the entire observed `CAPPED_BYPASS` window (`t=24.67s` to at
  least `t=26.5s`) shows it **never once climbed above the
  `side_release_m=0.058 m` hysteresis band** — the raw sensor was reporting
  "still close" continuously the whole time `CAPPED_BYPASS` was blindly
  driving forward. The transition into the capped state was forced by the
  *budget*, not by the sensor going quiet — the robot never actually
  received a "this is safe to move through" signal from its own sensors
  during the segment that made the penetration worse.

None of this disproves that `controller_v2`'s side-lane mechanism is
internally correct — every one of its 22+1 tests still describes true
properties of the side lane in isolation. What it proves is that scoping
the budget to the side lane alone, while leaving the front lane's
structurally-identical retrigger mechanism unbounded and separate, was
insufficient, **and** that treating "budget exhausted" as equivalent to
"safe to creep forward" was itself an unjustified assumption never
supported by the robot's own sensor state.

## 2. Design goal

Replace the two independent per-lane budgets with **one encounter-scoped
turn ledger shared by the front lane, the side lane, and `LOCAL_NARROW`**,
measured from actual yaw motion, while satisfying:

- `LOCAL_FRONT_DANGER`/`LOCAL_SENSOR_INVALID` are never throttled, delayed,
  or clipped while genuinely raw-active.
- Once a raw front-danger/warn episode ends, everything that follows
  within the same encounter draws against what is left of the shared
  ledger, never a fresh allowance.
- Once the ledger is exhausted, the robot's next action is gated by its
  **own current sensor state**, not by an assumption that a fixed slow
  speed is safe by default.

## 3. Unified encounter concept

```
                any raw active local decision
                (FRONT_DANGER / FRONT_WARN / LEFT_SIDE / RIGHT_SIDE / NARROW)
     ┌───────────────────────────────────────────────────┐
     │                                                      ▼
  CLOSED ──────────────────────────────────────────────▶ ACTIVE
     ▲                                                       │
     │                                          raw clears, ledger
     │                                          still has room
     │                                                       │
     │                                                       ▼
     │                                          (continue ACTIVE via
     │                                           CLEARANCE tail, still
     │                                           drawing on the ledger)
     │                                                       │
     │                                          ledger exhausted
     │                                          (by ACTIVE turning or
     │                                           by the CLEARANCE tail)
     │                                                       │
     │                                                       ▼
     │                                                 CONSTRAINED ◀────┐
     │                                              (HOLD or CREEP,      │
     │                                               see §6 — never       │
     │                                               unconditional)       │
     │                                                       │           │
     │                                     distance ≥ local_bypass_      │ any raw
     │                                     distance_m AND continuous-    │ active
     │                                     quiet ≥ side_clear_confirm_s  │ local
     │                                                       │           │ decision
     │                                                       ▼           │ (front,
     │                                              RECOVERY_ALLOWED ────┘ side, or
     │                                                       │             narrow)
     │                          continuous-quiet ≥ rearm_quiet_s          
     └───────────────────────────────────────────────────────┘

  CONSTRAINED ──(distance ≥ max_bypass_extension_m OR
                  time_in_constrained_s ≥ max_constrained_duration_s,
                  while continuous-quiet confirm never reached)──▶ FAILSAFE
                                                                        │
                                                          (TERMINAL — no
                                                           automatic exit;
                                                           see §6.4)
```

The `FAILSAFE` trigger has **two** independent conditions (§6.4 explains
why the time-based one is new in this audit pass). Unlike revision 6,
`FAILSAFE` has **no outgoing edge** in this diagram — it is a terminal
state for the lifetime of the running controller node (§6.4).

## 4. Field lifecycle (unchanged from revision 6, restated)

- `phase`: `CLOSED | ACTIVE | CONSTRAINED | RECOVERY_ALLOWED | FAILSAFE`.
- `encounter_start_yaw`: captured once, on `CLOSED → ACTIVE` (§5).
- `turn_ledger_used_rad`: cumulative ledger (§5).
- `origin`: `(x, y)` captured once, on `→ CONSTRAINED`.
- `constrained_entered_s`: **new field**, captured once, on `→
  CONSTRAINED`, used by the time-based `FAILSAFE` condition (§6).
- `quiet_since_s`: reset to `None` by any raw-active tick in any phase
  from `ACTIVE` onward; starts counting the instant a tick is genuinely
  `LOCAL_CLEAR`.
- `last_raw_mode`: kept for `hysteresis_hint()`.

All reset to their unset values only on `phase → CLOSED` — never on any
front↔side mode switch or any preemption within an open encounter.

## 5. Ledger measurement: precise definition (audit point 1)

### 5.1 Encounter start

The encounter opens on the tick `decide_local_obstacle()` first returns an
`active=True` decision (any of `LOCAL_FRONT_DANGER`, `LOCAL_FRONT_WARN`,
`LOCAL_LEFT_SIDE`, `LOCAL_RIGHT_SIDE`, `LOCAL_NARROW`) while `phase ==
CLOSED`. On this tick: `phase → ACTIVE`, `encounter_start_yaw ←
own_yaw_rad`, `turn_ledger_used_rad ← 0.0`, `previous_yaw ←
own_yaw_rad` (so the very first tick contributes `0` to the ledger, not a
spurious jump from an undefined previous value).

### 5.2 Per-tick update, with correct ±π wraparound handling

Every tick from `ACTIVE` onward, **before** any phase-specific branch
decides what to output:

```
raw_delta        = own_yaw_rad − previous_yaw
delta            = normalize_angle(raw_delta)      # wraps to (−π, π]
turn_ledger_used_rad += abs(delta)
previous_yaw     = own_yaw_rad
```

`normalize_angle` is the existing helper (`atan2(sin(x), cos(x))`) already
used throughout `cooperative_avoider.py`/`collision_math.py`. Applying it
to the **delta** (not the absolute yaw values) is what makes this correct
across the `±π` seam: at 20 Hz, the true per-tick rotation is always small
compared to `2π` (in `pilot_a2`, the fastest observed rate was
`danger_turn_rps=0.65 rad/s × 0.05 s ≈ 0.033 rad/tick`), so
`normalize_angle(raw_delta)` unambiguously recovers the shortest-path
signed rotation between two consecutive samples even if `own_yaw_rad`
itself happens to be reported near `±π` and wraps sign between ticks. This
update runs **unconditionally** — including on ticks where the returned
decision is `LOCAL_FRONT_DANGER`/`LOCAL_FRONT_WARN` and is being passed
through completely unmodified (audit point 1's "FRONT_DANGER 原始阶段虽然不被限制，
但其实际发生的 yaw 变化必须进入账本" — the ledger update and the
output-clipping decision are two separate concerns; the former is
unconditional, the latter is gated per §5.3).

### 5.3 Why cumulative `Σ|Δyaw|`, not deflection-from-`encounter_start_yaw`

Two alternatives were considered and rejected:

- **Current deflection**, `abs(normalize_angle(own_yaw_rad −
  encounter_start_yaw))`: under-counts any trajectory that turns and then
  partially turns back — exactly the shape of the original `controller_v1`
  defect (repeated `LOCAL_LEFT_SIDE` retriggers, each briefly interrupted
  by a `LOCAL_CLEARANCE`/`LOCAL_RECOVER`-style partial correction before
  triggering again). A robot that oscillates `-0.3 → -0.1 → -0.3 → -0.1`
  would show a **current** deflection of only `0.1–0.3 rad` at any given
  instant despite having swept through `0.3 rad` twice — this measure
  would never detect the compounding retrigger pattern this whole design
  exists to bound.
- **Peak deflection from start**, `max` over time of the same quantity:
  better than current-deflection, but still blind to a "there and back and
  there again" oscillation that never sets a new peak — the second `-0.3
  rad` excursion in the example above would add nothing to a peak-tracking
  ledger.
- **Cumulative `Σ|Δyaw|` (chosen)**: sums every increment regardless of
  direction, so the oscillating example above correctly accumulates
  `0.2+0.2+0.2+0.2=0.8 rad`, not `0.3 rad`. This is the only one of the
  three that cannot be defeated by an oscillating trajectory, which is
  precisely the failure mode both `controller_v1`'s original defect and
  `pilot_a2`'s compound front+side sequence exhibited.

### 5.4 No re-opening on a front↔side mode switch (audit point 1's last requirement)

`turn_ledger_used_rad`, `encounter_start_yaw`, `origin`, and
`constrained_entered_s` are fields of the **encounter**, not of any
particular raw mode. A transition from `LOCAL_FRONT_DANGER` to
`LOCAL_CLEARANCE` to `LOCAL_LEFT_SIDE` to `LOCAL_CLEARANCE` (`pilot_a2`'s
exact sequence) never touches any of them except via §5.2's unconditional
per-tick update — there is no code path in this design that resets the
ledger except `phase → CLOSED` (only reachable from `RECOVERY_ALLOWED`,
per §6.4 — `FAILSAFE` never resets anything, it is terminal). This is the
direct fix for the mechanism `pilot_a2` exposed:
under `controller_v2`, `LOCAL_LEFT_SIDE` arriving after a front episode
started a **new** `TURNING` phase with `side_turn_budget_used_rad` reset
to `0`; under this design, the same arrival is just another `ACTIVE` tick
of the **same** encounter, reading and writing the one shared ledger.

## 6. `CONSTRAINED`: sensor-gated, not a default creep (audit points 2 and 3)

### 6.1 `0.70 rad` is a headroom candidate, not a proven safe bound (audit point 2)

Revision 6 proposed `max_turn_ledger_rad = 0.70 rad`, reasoning that
`pilot_a2`'s single front-danger episode alone used `~0.54 rad`, so a cap
near `0.50 rad` would leave no room for any side response afterward. That
reasoning is **only about headroom, not about safety** — restated
explicitly here because it was not stated carefully enough in revision 6:

`pilot_a2`'s own trajectory shows that **at `~-0.54 rad` of accumulated
deflection, geometric clearance was already only `~0.007 m`** (measured at
the instant `LOCAL_LEFT_SIDE` first triggered, `t=23.057 s`). A ledger cap
of `0.70 rad` does not mean the robot is safe up to `0.70 rad` — it means
the robot is *allowed to keep turning* up to `0.70 rad` before the ledger
itself intervenes, which `pilot_a2`'s data suggests may already be well
past the point of geometric safety in this specific box geometry. **No
value of `max_turn_ledger_rad` proposed in this document should be read as
validated** — it is a placeholder pending the empirical work in §6.4.

### 6.2 Real-time policy can only use the robot's own sensors, never ground truth (audit point 3, second half)

The controller has no access to true box position or geometric clearance
at runtime — `front_distance_m`/`left_distance_m`/`right_distance_m` (IR/
ToF proximity) are the only real-time signals available, exactly as in
`controller_v1`/`controller_v2`. Geometric clearance (as computed by
`analyze_combined_task.py` and used throughout this document's forensic
analysis) is a **post-hoc, ground-truth-only verification metric** —
computed from recorded `/epuck1/state` positions against known box
geometry, used exclusively to grade a completed pilot run pass/fail. It
must never be wired into the running controller as a decision input, by
Supervisor telemetry or any other channel — doing so would let the
controller "see" the box directly, which no real e-puck2 hardware sensor
suite can do, and would invalidate any hardware-transfer claim for this
work. This is a hard architectural boundary, not a per-pilot choice:

- **Real-time (in the controller)**: `front_distance_m`/`left_distance_m`/
  `right_distance_m` thresholds only — nothing else has ever been used for
  local-obstacle decisions in `controller_v1`/`v2`, and this design does
  not add anything new to that list.
- **Post-hoc (in analysis/verification only)**: geometric clearance from
  Supervisor ground truth or recorded odometry-derived state, used to
  compute the pass/fail verdict for a pilot and to build the forensic
  timelines in this document (§1, and the earlier chat-reported table).
  §6.4's proposed `v3` pilot instrumentation (Supervisor ground truth) is
  exclusively for this second use.

### 6.3 Revised `CONSTRAINED` policy: sensor-gated hold vs. creep

Given 6.2, `CONSTRAINED` cannot directly ask "is geometric clearance below
`0.005 m`" — but it **can** ask "does `decide_local_obstacle()` still
report an active local hazard right now", which `pilot_a2`'s data shows
was true (`left_distance_m` inside the release-hysteresis band) for the
entire window where blind creep made the penetration worse. Revised
policy, checked every tick while `phase == CONSTRAINED`:

- **If the raw decision this tick is active** (`LOCAL_FRONT_DANGER`,
  `LOCAL_FRONT_WARN`, `LOCAL_LEFT_SIDE`, `LOCAL_RIGHT_SIDE`, or
  `LOCAL_NARROW`): command a full **HOLD** — `linear_mps=0.0,
  angular_rps=0.0`. (`LOCAL_FRONT_DANGER`/`LOCAL_FRONT_WARN` specifically
  still preempt entirely per §7 and are returned unmodified rather than
  reaching this branch at all when they are the active mode — this bullet
  covers `LOCAL_LEFT_SIDE`/`RIGHT_SIDE`/`NARROW` reappearing while
  `CONSTRAINED`, which reuses §7's front-vs-side precedence: side/narrow
  reappearing does **not** turn, per the ledger being exhausted, but it
  also must not creep forward into a sensor that is actively reporting
  proximity.) `quiet_since_s` is reset (this tick was not quiet).
- **If the raw decision this tick is genuinely `LOCAL_CLEAR`**: only then
  may `CONSTRAINED` command a slow **CREEP** —
  `linear_mps=constrained_speed_mps` (§6.7 candidate), `angular_rps=0.0`.
  `quiet_since_s` accrues normally toward `side_clear_confirm_s`.
- `distance` (from `origin`) is updated every tick regardless of hold/
  creep, exactly as `controller_v2`'s `CAPPED_BYPASS` did — a `HOLD` tick
  simply contributes `0` to `distance` that tick, it does not stop the
  tracking.

This directly prevents `pilot_a2`'s specific failure: at `t=24.673 s`,
`left_distance_m=0.051 m` was still inside the release-hysteresis band, so
under this policy the robot would `HOLD` (not creep) at that instant,
rather than advancing `x` by `~0.012 m/s` while a proximity sensor was
still reporting the obstacle nearby.

### 6.4 `FAILSAFE` is a hard latch — no automatic exit (revision 6.2, reviewer point 2)

`HOLD` prevents making an already-marginal clearance worse, but it also
does not *recover* clearance — a robot already at negative clearance stays
there while holding. This creates a failure mode a pure "always hold while
active" policy would have: `distance` never advances while holding, so
the existing `distance ≥ max_bypass_extension_m` condition alone could
never fire, and the robot could sit in `CONSTRAINED` indefinitely.
`FAILSAFE` gains a second, independent trigger to bound this —
`time_in_constrained_s = now − constrained_entered_s ≥
max_constrained_duration_s` (candidate value, §6.7) — checked every tick
regardless of whether `distance` has advanced.

**Revision 6 (the first pass) had `FAILSAFE` exit back to `CLOSED` once
`continuous-quiet ≥ rearm_quiet_s`, mirroring `RECOVERY_ALLOWED`'s own
exit rule. This is withdrawn in this revision.** The reviewer's objection
is correct and decisive: if the robot is holding perfectly still because a
raw local sensor is genuinely, persistently active against a **static**
obstacle at a fixed relative position, nothing in the scene changes on its
own — the quiet timer's premise ("wait long enough and it will resolve
itself") does not generally hold for a static-obstacle encounter, and
designing an automatic recovery path on top of that premise is designing
a false sense of safety into the one state whose entire purpose is being
the last resort. `FAILSAFE` is therefore redefined as a **hard latch**:

- On entry: `linear_mps=0.0, angular_rps=0.0`, `safety_stop=True`,
  `mode="LOCAL_ENCOUNTER_FAILSAFE"` (renamed from revision 6's
  `LOCAL_SIDE_ENCOUNTER_FAILSAFE` — the encounter is no longer
  side-specific, so the name should not imply it is).
- Every subsequent tick, for the remaining lifetime of the running
  `CooperativeAvoider` node: the same zero-velocity, `safety_stop=True`
  decision is returned, **unconditionally** — `quiet_since_s`,
  `turn_ledger_used_rad`, `origin`, and `constrained_entered_s` are frozen
  at whatever they held on entry and are never read again for a
  phase-transition decision (only ever preserved for diagnostics/logging).
- The **only** ways `FAILSAFE` ends are: (a) the controller node is
  restarted (a fresh `LocalAvoidanceLatch` instance, matching "重新启动节点"),
  (b) an explicit external reset call is added in a future revision
  (not designed here — no such API exists yet, deliberately, so that
  "reset" cannot be triggered by anything inside the control loop itself),
  or (c) a new trial/pilot is started, which is a fresh process. Nothing
  in `local_obstacle_logic.py` or `cooperative_avoider.py` can transition
  `FAILSAFE → CLOSED` on its own.
- `LOCAL_SENSOR_INVALID` is the **only** exception, because `apply()`
  already checks it unconditionally before any phase logic runs at all
  (§7's table, first row) — this is unchanged structurally and needs no
  new carve-out. `LOCAL_FRONT_DANGER`/`LOCAL_FRONT_WARN` do **not** get a
  special peek-through once latched: since `FAILSAFE`'s own output is
  already `linear_mps=0.0, angular_rps=0.0` — identical in effect to what
  a fresh front-danger response would command in this already-stopped
  state — there is no behavioural reason to special-case it, and doing so
  would only reintroduce a field-update exception this revision is
  specifically trying to eliminate (revision 6's first draft of this
  section did carve out such an exception; it is withdrawn here in favour
  of the simpler, fully-unconditional latch in §7's table).

**Trial-script implication (already satisfiable without new plumbing)**:
`cooperative_avoider.py`'s `max_runtime_s` ceiling is checked at the very
top of `_control()`, before any local-decision processing, so a
node that is latched in `FAILSAFE` still reaches
`elapsed >= self.max_runtime` on schedule and still emits
`"COMPLETE: maximum runtime reached; commanding zero"` — the existing
trial-script watchdog pattern (waiting for any `COMPLETE:` line) does not
need to change to avoid hanging on a latched run. What **does** need to
change is the analysis layer: any pilot whose controller log shows
`LOCAL_ENCOUNTER_FAILSAFE` was ever entered must be reported as a distinct
outcome category, never as an unqualified pass — see §6.6.

### 6.5 The hold-vs-creep question is not fully closed by this document

What this document does **not** resolve, and explicitly defers to
instrumented `v3` pilot data: whether `HOLD` (safe but non-recovering),
the original `CREEP` (recovers distance but risks worsening marginal
clearance, as `pilot_a2` showed), or some other strategy (e.g. a slow
*reverse*, not designed here) produces the best real clearance-recovery
outcome from a near-contact state. §10 requires this to be settled by
instrumented `v3` pilots (ground-truth clearance logged throughout
`CONSTRAINED`, never fed back to the controller per §6.2) before either
policy is treated as final.

### 6.6 Three distinct pilot outcome categories (revision 6.2, reviewer point 3)

A pilot run under this design can end in exactly one of three categories,
which must be reported and counted separately — none of them are
interchangeable, and only one qualifies as full success:

1. **Collision** (`box_collision_detected=true` in post-hoc analysis, or
   `minimum_box_clearance_m < 0.005 m`): hard fail, regardless of what the
   controller's own mode sequence shows.
2. **`LOCAL_ENCOUNTER_FAILSAFE` latched, no collision**: the safety
   mechanism did what it is for — it prevented a collision by stopping —
   but the robot did **not** complete the box-avoidance task (it is
   sitting stopped, not past the box, not cruising). This must be reported
   as "safe stop effective, task incomplete", never phrased as or counted
   as a task success, and never merged into the same statistic as
   category 3.
3. **No collision, `FAILSAFE` never entered, `RECOVERY_ALLOWED → CLOSED →
   CRUISE` reached**: full success — collision avoided **and** the box
   was actually passed and normal cruising resumed.

`pilot_a3` (§13) is judged against category 3 specifically, per the
reviewer's own stated acceptance bar — categories 1 and 2 are both
"not passed", but are recorded and reasoned about differently, since a
category-2 run is informative evidence that the latch/safety-stop
machinery itself works, even on a run that does not count as a formal
success.

### 6.7 Candidate parameters for §6 (all explicitly unvalidated)

- `constrained_speed_mps`: `0.006 m/s` (matches `LOCAL_NARROW`'s existing
  cautious speed) — used only during genuine-`LOCAL_CLEAR` creep ticks.
- `max_constrained_duration_s`: proposed candidate `10.0 s` — long enough
  that a normal, successful `CONSTRAINED → RECOVERY_ALLOWED` transition
  (expected to take on the order of `local_bypass_distance_m /
  constrained_speed_mps ≈ 0.08/0.006 ≈ 13 s` of *creep* time if creeping
  continuously, longer if interleaved with `HOLD` ticks) is not
  prematurely cut off, while still bounding worst-case `CONSTRAINED`
  duration. This interacts with `constrained_speed_mps` and
  `local_bypass_distance_m` in a way `v3` pilots must check empirically —
  flagged, not resolved, here.

## 7. Explicit priority table (audit point 4)

```
SENSOR_INVALID  >  raw FRONT_DANGER  >  raw FRONT_WARN  >  CONSTRAINED/FAILSAFE  >  other local recovery
```

**Important exception, added in revision 6.2**: this precedence and its
field effects apply while `phase` is `CLOSED`, `ACTIVE`, `CONSTRAINED`, or
`RECOVERY_ALLOWED`. **Once `phase == FAILSAFE`, §6.4's latch rule
overrides this table entirely** — no field is updated by any subsequent
tick, of any kind, ever again for that node's lifetime. The row below
marked `(pre-latch only)` does not apply once latched; a separate row
covers the latched state explicitly.

| Preemption | ledger | quiet_since_s | origin | phase | distance/extension tracking |
|---|---|---|---|---|---|
| `SENSOR_INVALID` fires | untouched | untouched | untouched | untouched | untouched (whole `apply()` returns before any encounter field is read or written — identical to `controller_v2`'s existing first check; this row is the one exception that still applies even while latched in `FAILSAFE`, since it is checked before phase logic of any kind) |
| raw `FRONT_DANGER` fires *(pre-latch only)* | `+= |Δyaw|` (§5.2, unconditional) | reset to `None` (still busy) | untouched | untouched (whatever phase was active resumes next tick once front clears) | untouched this tick (decision passed through unmodified; `LOCAL_FRONT_DANGER` always forces `linear_mps=0.0` via `force_linear_zero`, so `distance` does not advance this tick either) |
| raw `FRONT_WARN` fires *(pre-latch only)* | `+= |Δyaw|` | reset to `None` | untouched | untouched | updates normally (its `linear_mps` is not force-zeroed, so `distance` may advance if the robot has residual forward motion) |
| `CONSTRAINED` own logic (§6.3) | untouched by this tier itself — only read to decide state, never written by the phase handler directly | as in §6.3 | set once on `→ CONSTRAINED`, otherwise untouched | transitions per §3/§6.4 | updated every tick per §6.3 |
| `FAILSAFE` **latched** (§6.4) — replaces the `FRONT_DANGER`/`FRONT_WARN` row's effects once reached | frozen at whatever it held on entry; never incremented again, even by a fresh raw `FRONT_DANGER` tick | frozen | frozen | **never changes** — no code path exits `FAILSAFE` | frozen; `max_bypass_extension_m`/`max_constrained_duration_s` are never re-checked because `phase` can no longer be `CONSTRAINED` again for this node |
| other local recovery (`RECOVERY_ALLOWED`'s masked hand-off to `LOCAL_RECOVER`, or `CLOSED`'s idle state) | untouched (`RECOVERY_ALLOWED`) / reset to `0.0` (`CLOSED`, per §5.1's re-open) | as in `controller_v2` §2.4 | untouched (`RECOVERY_ALLOWED`) / reset to `None` (`CLOSED`) | as in §3 | untouched (`RECOVERY_ALLOWED`) / reset (`CLOSED`) |

This table is the direct answer to audit point 4 — every preemption tier's
effect on every shared field is enumerated, not left implicit, including
the special-cased latched state added in this revision.

## 8. What is explicitly unchanged

- `decide_local_obstacle()` itself.
- `local_front_danger_m`/`_warn_m`/`_release_m`, `local_side_danger_m`/
  `_warn_m`/`_release_m`, `danger_turn_rps`, `warning_turn_rps`,
  `side_turn_rps` — locked.
- `local_bypass_distance_m` (0.08 m), `local_recovery_turn_rps` (0.18
  rad/s) — locked, reused by `RECOVERY_ALLOWED`'s hand-off exactly as in
  `controller_v2`.
- The `LOCAL_RECOVERY_READY` one-shot hand-off and shared
  `_local_recover_command()` helper — structurally unchanged from
  `controller_v2` §5.
- The peer-CPA control path — untouched; still needs its own regression
  pilot before any `v3` combined-scenario work.
- Box position, `epuck1` initial pose — **not touched by this document**,
  per the explicit instruction not to mask `pilot_a2`'s collision by
  adjusting scenario geometry.

## 9. Deterministic `pilot_a2`-replay regression / counterfactual short-term trend check (revision 6.2, reviewer point 1)

**Naming and scope, corrected in this revision**: this is a
**deterministic regression / counterfactual short-term check**, not a
safety-acceptance test and not proof of a `≥0.005 m` clearance guarantee.
`pilot_a2`'s recorded trajectory reflects the **old** (`controller_v2`)
commands; once `controller_v3` issues different commands, every
subsequent position, sensor reading, and mode transition would differ from
what was recorded — a replay cannot know what the *real* closed-loop
sensor stream would have been under the new policy, only what a simple
kinematic model predicts from the *last real position before behaviour
diverges*. This test is therefore scoped to exactly three things a replay
*can* legitimately check, and explicitly excluded from a fourth that it
cannot:

- **Can check** (pure latch-logic, no kinematics involved): whether
  `FRONT_DANGER → FRONT_CLEARANCE → LEFT_SIDE → SIDE_CLEARANCE` is
  recognised as one encounter (item 1 below).
- **Can check**: yaw-wraparound handling, ledger accumulation, and phase
  transitions in response to the exact recorded raw-decision sequence
  (item 2 below).
- **Can check**: what command `CONSTRAINED` issues at the exact recorded
  tick the ledger is exhausted, since that only depends on the raw
  decision and ledger state at that one instant, not on anything
  downstream (item 3 below).
- **Cannot check, and this design no longer claims to**: whether the
  *actual* geometric clearance stays `≥0.005 m` once the candidate
  policy's different commands change the robot's real trajectory beyond
  that one instant — that requires a real closed-loop simulation
  (`pilot_a3`, §13), because only Webots' physics and sensor model can
  produce what the *new* raw decisions would actually be at each
  subsequent tick. Item 4 below is reframed accordingly: it is a
  **short-horizon, low-fidelity counterfactual trend comparison** against
  `pilot_a2`'s own recorded outcome — informative for catching an
  obviously-wrong implementation early and cheaply, but its result is
  **never** cited as evidence of `≥0.005 m` clearance, in any report, pilot
  summary, or thesis text. Only `pilot_a3`'s real Webots run, graded by
  Supervisor/analyzer ground truth (§6.2, §6.6), may be cited for that.

`pilot_a2`'s exact raw sequence (§1's timeline) is encoded as a scripted
input to the latch in isolation — no Webots, no ROS node — so it is fast
and fully deterministic:

1. **Encounter identity**: feed `LOCAL_FRONT_DANGER` (matching the
   recorded `~0.68 s` raw-active duration and `danger_turn_rps=0.65`
   magnitude) → its `LOCAL_CLEARANCE` tail → `LOCAL_LEFT_SIDE` (matching
   the recorded raw-active duration) → its `LOCAL_CLEARANCE` tail. Assert
   `phase` never returns to `CLOSED` between these four segments — it is
   one encounter throughout.
2. **No second allowance**: assert `turn_ledger_used_rad` after the
   `LOCAL_FRONT_DANGER` segment already reflects its full contribution
   (matching the injected yaw trajectory), and assert the subsequent
   `LOCAL_LEFT_SIDE` segment's *available* turning is `max(0,
   max_turn_ledger_rad − <front's contribution>)`, not a fresh
   `max_turn_ledger_rad` — directly falsifiable against `controller_v2`'s
   old behaviour, which this test would have failed.
3. **No unconditional creep near an active sensor**: construct the
   `CONSTRAINED` entry with the raw decision still reporting
   `LOCAL_LEFT_SIDE`-range proximity (mirroring `pilot_a2`'s
   `left_distance_m=0.051 m` at the exact tick the ledger is exhausted),
   and assert the returned decision is the `HOLD` shape
   (`linear_mps=0.0, angular_rps=0.0`), not `constrained_speed_mps`.
4. **Counterfactual short-term trend check** (renamed from revision 6's
   "clearance-preserving replay simulation" — reframed per this revision's
   §9 preamble): a companion test replays `pilot_a2`'s recorded `own_x`/
   `own_y`/`own_yaw_rad` trajectory *up to the tick `CONSTRAINED` is
   entered* (using the real recorded values), then, from that exact state,
   forward-simulates **two** command sequences in parallel over the same
   short wall-clock window (a few seconds, not the whole remaining run —
   long-horizon extrapolation compounds the model's inaccuracy and is not
   attempted): (a) `controller_v2`'s actual recorded `CAPPED_BYPASS`
   commands (the real, already-known-bad outcome, used as the baseline),
   and (b) `controller_v3`'s candidate `CONSTRAINED` commands (`HOLD`
   while the last-recorded raw reading was active, `CREEP` only from a
   point where it was recorded genuinely clear). Both are integrated
   through the same simple unicycle model (`x += v cos(yaw) dt`, `y += v
   sin(yaw) dt`, `yaw += ω dt`) and scored with the same clearance formula
   `analyze_combined_task.py` uses. **Assertion**: the candidate policy's
   short-horizon simulated clearance trend must be **no worse than** the
   baseline's actual recorded trend over the same window (e.g., does not
   *increase* the rate of clearance loss relative to what really
   happened) — a directional, comparative check, not an absolute
   `≥0.005 m` gate. Any test report, docstring, or log line for this test
   must state its result as "counterfactual trend: {improved | neutral |
   regressed} relative to the pilot_a2 baseline over N simulated ticks",
   never as "clearance verified" or "safety confirmed".

## 10. Test checklist — code-level only, all run and reported **before** any pilot (revision 6.2, reviewer point 5)

Every item below is a `pytest`-level test (latch-logic unit test or the
`cooperative_avoider`-level integration test, mirroring `controller_v2`'s
split between `test_local_avoidance_latch_v2.py`-style and
`test_cooperative_avoider_v2_integration.py`-style tests). **None of them
require Webots.** `pilot_a3` (§13) is a separate, later step, run only
after every item here is green and reported.

1. `±π` wraparound (audit point 1 / §5.2): inject a yaw trajectory that
   crosses the `±π` seam between two consecutive ticks with a small true
   rotation, assert `turn_ledger_used_rad`'s increment matches the true
   small rotation, not a spurious near-`2π` jump.
2. `FRONT_DANGER → FRONT_CLEARANCE → LEFT_SIDE → SIDE_CLEARANCE` shares one
   ledger (§9 items 1-2, the deterministic `pilot_a2`-sequence replay):
   one encounter throughout, no second independent allowance granted to
   `LEFT_SIDE`.
3. Oscillation stress test (§5.3): inject a `LOCAL_LEFT_SIDE ↔
   LOCAL_CLEARANCE` sequence that repeatedly turns and partially recovers
   without ever setting a new peak deflection from `encounter_start_yaw`;
   assert the ledger still reaches `max_turn_ledger_rad` and caps, proving
   cumulative `Σ|Δyaw|` is not defeated by oscillation the way a
   peak-based measure would be.
4. Raw-active-gates-HOLD (§9 item 3 / §6.3): construct `CONSTRAINED` entry
   with the raw decision still reporting proximity (mirroring `pilot_a2`'s
   `left_distance_m=0.051 m`), assert `HOLD` (`linear_mps=0.0,
   angular_rps=0.0`), not `constrained_speed_mps`.
5. Clear-then-CREEP, re-trigger-same-tick-HOLD (§6.3): from `CONSTRAINED`
   with raw genuinely clear, assert a `CREEP` tick is issued; then inject
   a raw side/narrow re-trigger and assert the **same tick** reverts to
   `HOLD` (not the tick after) — this is a same-tick requirement, not an
   eventual one.
6. `FAILSAFE` is a hard latch with no automatic exit (§6.4, reviewer
   point 2 — the test this whole revision exists to add): drive
   `CONSTRAINED` into `FAILSAFE` via either trigger (`distance ≥
   max_bypass_extension_m` or `time_in_constrained_s ≥
   max_constrained_duration_s`), then feed a long stream of genuinely
   `LOCAL_CLEAR` ticks — including a stream well past `rearm_quiet_s` in
   duration — and assert `phase` remains `FAILSAFE` and the output remains
   the frozen zero-velocity `safety_stop=True` decision throughout. This
   test must fail against revision 6's first-draft design (which did
   auto-exit via the quiet timer) to prove it is actually exercising the
   fix.
7. `SENSOR_INVALID`/raw `FRONT_DANGER`/raw `FRONT_WARN` preemption during
   every **pre-latch** phase (`ACTIVE`, `CONSTRAINED`, `RECOVERY_ALLOWED`):
   assert the exact field-by-field behaviour in §7's table. A companion
   assertion for the **latched** `FAILSAFE` case (§7's dedicated row):
   feed a fresh raw `FRONT_DANGER` tick while latched and assert the
   output is still the frozen `FAILSAFE` decision, not the front-danger
   response, and no field changes.
8. `RECOVERY_ALLOWED` interrupted by front, side, or narrow (extends
   `controller_v2`'s side/narrow-only coverage to include front): falls
   back to `CONSTRAINED`, ledger/origin/`constrained_entered_s` retained.
9. `LOCAL_RECOVERY_READY` hand-off and shared `_local_recover_command`
   helper: unchanged assertions from `controller_v2`.
10. `controller_v2`'s existing 23 tests: rewritten (not just re-run) for
    the unified-encounter structure — `controller_v2`'s original test
    files retained/renamed with a clear note (e.g.
    `test_local_avoidance_latch_v2.py` kept as `controller_v2`-labelled
    historical evidence; new `v3` test files added alongside, not
    overwriting them), per §11's data-separation rule extended to test
    code.
11. Full `colcon test --packages-select epuck2_comm`: `41`
    (`controller_v1`, unmodified) `+` `v3`'s new/superseding tests `+` the
    replay tests from §9, `0` failures.

## 11. v1 / v2 / v3 data separation (unchanged from revision 6)

- `controller_v1`: git baseline `e76adf3`. Untouched.
- `controller_v2_local_latch_20260717`: git commit `922a580`. Its 23 tests
  and its exclusionary pilots (`pilot_a`, excluded — premature-stop test
  design flaw; `pilot_a2`, excluded — confirms the front+side compounding
  defect this document addresses) remain as `controller_v2`-labelled
  evidence. No `controller_v2` bag, log, or statistic is reused as a `v3`
  formal result.
- `controller_v3_unified_encounter` (working name, pending confirmation):
  new git commits, new/renamed test files, new bag/log naming convention,
  to be finalised only once implementation is authorised.
- Box position, `epuck1` initial pose, and all locked speed/threshold
  parameters remain exactly as they were for `pilot_a`/`pilot_a2` — this
  document proposes no scenario-geometry, speed, or locked-threshold
  change of any kind.

## 12. Implementation and version management (revision 6.2, reviewer point 4)

- Implementation begins from a clean checkout of the current `v2` tip
  (`922a580`) — a new commit sequence on top, not a rewrite of `v2`'s
  history.
- `v1` (`e76adf3`), `v2` (`922a580` and any fixup commits on top),
  and `v3` (new commits, this design) remain three independently
  checkable points in git history; no `v3` commit force-modifies or
  squashes `v2`'s.
- New `v3` bag/log/analysis directories use a `controller_v3_unified_
  encounter_20260717` (or the actual implementation date, if later)
  condition-name prefix, mirroring the `controller_v2_local_latch_
  20260717` convention already established — never reusing or overwriting
  a `controller_v2`-prefixed directory.
- Not touched by this design or its implementation: box position, `epuck1`
  initial pose, `nominal_speed_mps`/`avoidance_speed_mps`, any
  `local_front_*`/`local_side_*` threshold, `local_bypass_distance_m`,
  `local_recovery_turn_rps`.
- Explicitly still-unvalidated candidates, to ship with an unambiguous
  code comment on each (matching `controller_v2`'s existing discipline):
  `max_turn_ledger_rad=0.70`, `constrained_speed_mps=0.006`,
  `max_constrained_duration_s=10.0`, and (carried over unchanged from
  `controller_v2`) `max_bypass_extension_m=0.30`,
  `side_clear_confirm_s=1.0`, `rearm_quiet_s=1.5`.

## 13. `pilot_a3`: the acceptance gate this design defers to (not run until §10 is green and reported)

Once §10's full test suite is green and reported, the next and only
subsequent step is one new exclusionary pilot, **not** run automatically
and **not** run until this report has been reviewed:

- Same world, same `epuck1`/`epuck2` start poses, same
  `max_runtime_s=55 s`, `stop_after_recovery=false` as `pilot_a2` —
  identical scenario, only the controller code differs, so any behaviour
  change is attributable to `controller_v3` and nothing else.
- **Instrumented with Webots Supervisor ground-truth pose** (§6.2 — feeding
  the analyzer only, never the running controller) so clearance can be
  computed continuously and precisely, not just reconstructed from
  odometry-derived `/epuck1/state` after the fact as `pilot_a`/`pilot_a2`
  were.
- Graded strictly against §6.6's category 3: **no collision, minimum
  clearance `≥0.005 m` throughout, `LOCAL_ENCOUNTER_FAILSAFE` never
  entered, box actually passed and `CRUISE` resumed.** A no-collision run
  that ends latched in `FAILSAFE` (category 2) is not a pass and must be
  reported as such, separately from a category-1 collision and from a
  category-3 success — per §6.6, none of the three are interchangeable in
  any report.

## 14. Recommendation

The mechanism identified in `pilot_a2` is real, precisely diagnosed, and
not addressed by any parameter tweak to `controller_v2` — implementing
`controller_v3_unified_encounter` is recommended, proceeding directly from
this confirmed design to implementation and the test suite in §10, with
`pilot_a3` (§13) as the only subsequent step and only after §10 is
reported green. §6.5 explicitly flags that the `HOLD` vs. `CREEP` question
inside `CONSTRAINED` is not settled by this document alone and needs
`pilot_a3`'s real, Supervisor-instrumented evidence before being treated
as final — this document's own reasoning is not a substitute for that
evidence, only a justification for why `pilot_a3` is structured the way
§13 describes.

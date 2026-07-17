# controller_v2_local_latch: fix design for the LOCAL_LEFT_SIDE retrigger defect

Date: 2026-07-17 (revision 5)
Status: **design proposal, not yet implemented**. No code has been changed.
Defect registered against: `controller_v1`, see
`combined_wood_moving_peer_README.md` for discovery/evidence across
`pilot_01`-`pilot_04`.

**Why revision 5 exists**: revision 4 masked the side lane unconditionally
once `RECOVERY_ALLOWED` was entered — a genuinely new side/`LOCAL_NARROW`
detection during that phase would still be silently ignored, letting
`LOCAL_RECOVER` keep turning toward an obstacle that had, in fact,
reappeared. Revision 4's hand-off to recovery also relied on returning a
plain inactive decision, which `cooperative_avoider.py`'s existing generic
fallback branch would treat as a brand-new case, creating a *second*,
unvalidated `local_bypass_origin` and running another `0.08 m` straight
leg before `LOCAL_RECOVER` actually started — inconsistent with "one
origin per encounter" and adding an untested extra path. Revision 5 fixes
both: `RECOVERY_ALLOWED` now continuously re-checks for genuine new side
activity and falls back to `CAPPED_BYPASS` (not `TURNING`) if it sees any,
and the hand-off to `LOCAL_RECOVER` is now an explicit one-shot signal
that `cooperative_avoider.py` special-cases directly, bypassing the
generic fallback entirely.

## 1. Root cause (unchanged, restated briefly)

`decide_local_obstacle()`'s side thresholds (`side_warn_m=0.052`,
`side_release_m=0.058`) are 6 mm apart. `epuck1`'s post-box
`LOCAL_RECOVER` grazes the box's trailing corner at a shallow angle, and
its own turning motion causes the left IR reading to flicker across that
band. `controller_v1`'s latch has no cap on how many times a flicker can
restart the turn, so a single-direction turn ran unbroken for ~4.8 s in
`pilot_04` (yaw `-0.004 rad` → `-1.193 rad`), swinging the body into the
box (confirmed geometric collision in `pilot_01`-`pilot_03`;
`0.0014 m` clearance, below the `0.005 m` gate, in `pilot_04`).

## 2. Side-lane phase state machine

Scoped to the side lane only (§3: the front lane and `LOCAL_NARROW`'s
*own* independent semantics remain structurally isolated from the budget
and `origin`/`distance` machinery; `LOCAL_NARROW` does, as of this
revision, participate in one specific way described in §2.4/§2.5 below —
its narrow-passage detection is treated as a genuine hazard signal for the
purpose of interrupting `RECOVERY_ALLOWED`, exactly like a side
detection, per reviewer point 1).

```
CLOSED ──(new side trigger)──▶ TURNING ──(budget spent,
   ▲                              │        clipped exactly
   │                              │        at the limit)──▶ CAPPED_BYPASS ◀───────────┐
   │                              │                              │      │              │
   │                              │(raw goes inactive              │      │(distance ≥    │(any raw side/
   │                              │ before budget spent:            │      │ max_bypass_   │ NARROW active
   │                              │ ordinary v1-style clean          │      │ extension_m   │ while in
   │                              │ clearance tail, phase             │      │ while still   │ RECOVERY_ALLOWED:
   │                              │ returns to CLOSED after           │      │ CAPPED_BYPASS)│ quiet_since_s
   │                              │ clear_hold_s, exactly like v1)     │      ▼              │ reset, origin/
   │                              ▼                                    │  FAILSAFE           │ budget/distance
   │                         (back to CLOSED)                          │      │              │ retained,
   │                                                                    │      │(rearm_       │ output forced
   │             (distance ≥ local_bypass_distance                     │      │ quiet_s)     │ back to straight,
   │              AND continuous-quiet ≥ side_clear_confirm_s)          │      ▼              │ zero angular)
   │                              │                                    │   CLOSED             │
   │                              ▼                                    │                       │
   │                    RECOVERY_ALLOWED ───(any raw side/NARROW active)───────────────────────┘
   │                    (one-shot LOCAL_RECOVERY_READY
   │                     pulse on entry; every subsequent
   │                     tick while genuinely clear:
   │                     plain inactive, hands off to
   │                     cooperative_avoider.py's own
   │                     LOCAL_RECOVER branch)
   │                              │
   └──────(continuous-quiet ≥ rearm_quiet_s, never interrupted)────────┘
```

### 2.1 Unified continuous-quiet tracker (unchanged from revision 4)

`quiet_since_s: float | None`. Every tick, in every side phase:
- Raw decision inactive this tick: `if quiet_since_s is None: quiet_since_s
  = now_s` (start a streak; leave a running one alone).
- Raw decision active this tick (`LOCAL_LEFT_SIDE`/`RIGHT_SIDE`) **or**
  `LOCAL_NARROW`: `quiet_since_s = None` (streak broken, must restart).
- `continuous_quiet_s = (now_s - quiet_since_s) if quiet_since_s is not
  None else 0.0`.

Two thresholds, both still explicitly **unvalidated `v2`-pilot candidates,
not locked constants** (reviewer's standing requirement, reaffirmed for
this revision):

- `side_clear_confirm_s` (candidate `1.0 s`, equal to the existing locked
  `clear_hold_s=1.0 s` by deliberate choice, kept as a separately-named,
  independently-tunable field) — gates `CAPPED_BYPASS → RECOVERY_ALLOWED`.
- `rearm_quiet_s` (candidate `1.5 s`, strictly greater than
  `side_clear_confirm_s`) — gates `RECOVERY_ALLOWED → CLOSED` and
  `FAILSAFE → CLOSED`.

### 2.2 `TURNING` (unchanged from revision 4)

Entry from `CLOSED` on the first raw `LOCAL_LEFT_SIDE`/`LOCAL_RIGHT_SIDE`.
Accumulates `budget_used_rad += abs(angular_rps) * dt` every active/
clearance-grace tick; the tick that would exceed
`max_side_encounter_turn_rad` (candidate `0.50 rad`, still unvalidated —
§6) is clipped to `remaining/dt` exactly, no overshoot, and the phase
becomes `CAPPED_BYPASS` starting next tick. A clean encounter that clears
before the budget is spent behaves exactly as `controller_v1` and returns
to `CLOSED` via the existing `clear_hold_s` tail, untouched by any of this
design.

### 2.3 `CAPPED_BYPASS`

Entry (from `TURNING`, budget exhausted — **or, new in this revision, an
interrupted `RECOVERY_ALLOWED`**, §2.4): if this is the *first* entry for
the encounter, `origin = (own_x, own_y)` is captured once; if this is a
**re-entry** from an interrupted `RECOVERY_ALLOWED`, `origin` and
`budget_used_rad` are **not** touched — they carry over unchanged from
whenever they were originally set, per reviewer point 1's explicit
requirement to retain "同一次遭遇原有的累计转角预算、origin、bypass
distance." `quiet_since_s` is reset to `None` on every entry/re-entry
(a fresh streak must be established from here).

Every tick:
1. Update `quiet_since_s` per §2.1.
2. `distance = hypot(own_x - origin.x, own_y - origin.y)` — updated every
   tick regardless of active/inactive status, unconditionally (this is
   what makes the `FAILSAFE` check correct under flicker, unchanged
   reasoning from revision 4 §2.2 point 2 fix).
3. Checked in order:
   - `distance >= local_bypass_distance` **and** `continuous_quiet_s >=
     side_clear_confirm_s` → transition to `RECOVERY_ALLOWED` (§2.4),
     emitting the one-shot `LOCAL_RECOVERY_READY` signal this same tick.
   - Else `distance >= max_bypass_extension_m` (candidate `0.30 m`, still
     unvalidated) → transition to `FAILSAFE` (§2.5).
   - Else → remain in `CAPPED_BYPASS`.
4. Output while remaining in `CAPPED_BYPASS` (the "remain" branch only —
   see §2.4 for what is returned on the transition tick):
   `LocalObstacleDecision(active=True, safety_stop=False,
   mode="LOCAL_SIDE_BYPASS", linear_mps=side_speed_mps, angular_rps=0.0)`
   — angular always exactly zero.

### 2.4 `RECOVERY_ALLOWED` (revised: no longer a one-way mask)

**Entry** (from `CAPPED_BYPASS`, §2.3's first branch): on this exact tick,
the latch returns a **one-shot** decision
`LocalObstacleDecision(active=True, safety_stop=False,
mode="LOCAL_RECOVERY_READY", linear_mps=0.0, angular_rps=0.0)`.
`cooperative_avoider.py` special-cases this mode name (§5) to hand control
directly to its own heading-error-based recovery-turn computation and set
`self.mode = "LOCAL_RECOVER"`, **without** touching
`local_bypass_origin` and **without** running the generic fallback's own
bypass-distance leg — this is the entire fix for reviewer point 2. This
pulse fires **exactly once** per entry into `RECOVERY_ALLOWED` (including
re-entries after an interruption-then-reconfirmation cycle) — it is not
repeated on subsequent ticks.

**Every subsequent tick while remaining in `RECOVERY_ALLOWED`:**
1. Inspect the raw decision this tick (`decide_local_obstacle()`'s output,
   *before* any masking — the latch always evaluates it, it just chooses
   what to expose).
2. If the raw decision is `LOCAL_LEFT_SIDE`, `LOCAL_RIGHT_SIDE`, **or**
   `LOCAL_NARROW` (all three treated identically — reviewer point 1
   explicitly includes `LOCAL_NARROW`): this is genuine renewed proximity,
   not a stale artifact. **Immediately**: `quiet_since_s = None`; phase
   reverts to `CAPPED_BYPASS` (never `TURNING` — no budget
   re-accumulation, ever, for this encounter); `origin`, `budget_used_rad`
   retained unchanged; the latch returns the same
   `LOCAL_SIDE_BYPASS`/`angular=0.0` output `CAPPED_BYPASS` would (i.e.
   the transition and the tick's output happen together, so there is no
   tick where a stale `LOCAL_RECOVER` command and a fresh hazard detection
   coexist).
3. Otherwise (raw decision genuinely inactive): update `quiet_since_s` per
   §2.1 (extends the streak); return a **plain inactive** decision
   (`active=False`, ordinary `LOCAL_CLEAR` shape, *not* another
   `LOCAL_RECOVERY_READY` pulse) — `cooperative_avoider.py` sees nothing
   active from the local-decision path this tick, so its own persisted
   `self.mode == "LOCAL_RECOVER"` (set on the entry tick) is picked up by
   the **existing, unmodified** `elif self.mode == "LOCAL_RECOVER":`
   branch and runs the correct heading-based turn, exactly as it would for
   any ordinary recovery, including its own existing exit to `CRUISE` once
   `abs(heading_error) < 0.08`.
4. If `continuous_quiet_s >= rearm_quiet_s` (only reachable via
   uninterrupted step-3 ticks): transition to `CLOSED` —
   `budget_used_rad = 0`, `origin = None`, `quiet_since_s = None`, ready
   for a genuinely fresh encounter.

### 2.5 `FAILSAFE` (unchanged from revision 4)

Entry (from `CAPPED_BYPASS` only, §2.3's second branch): `quiet_since_s`
carries over unchanged. Returns
`LocalObstacleDecision(active=True, safety_stop=True,
mode="LOCAL_SIDE_ENCOUNTER_FAILSAFE", linear_mps=0.0, angular_rps=0.0)`
every tick — full stop via the existing `safety_stop` channel, no new
priority wiring needed. Exit: `continuous_quiet_s >= rearm_quiet_s` →
`CLOSED`, identical rule to `RECOVERY_ALLOWED`. `LOCAL_FRONT_DANGER`/
`LOCAL_FRONT_WARN`/`LOCAL_SENSOR_INVALID` still preempt at any point
during `FAILSAFE`, exactly as during any other side phase (§3).

## 3. Front lane and priority: unchanged from revision 4, still structurally isolated

`apply()`'s first two checks run before any side-phase logic and are
unaffected by anything in §2, including this revision's `RECOVERY_ALLOWED`
interruption path:

1. `decision.safety_stop` (`LOCAL_SENSOR_INVALID`) → return unchanged, no
   side-phase field touched.
2. `decision.mode in ("LOCAL_FRONT_DANGER", "LOCAL_FRONT_WARN")` →
   handled entirely by the separate, unbounded front-lane bookkeeping,
   returned unchanged. **No side-phase field (`phase`, `budget_used_rad`,
   `origin`, `quiet_since_s`) is read or written on this path, in any
   phase, including `RECOVERY_ALLOWED` and `FAILSAFE`** — a front
   preemption can interrupt `LOCAL_RECOVER` exactly as it can interrupt
   anything else (`cooperative_avoider.py`'s existing priority order is
   untouched: local avoidance, including the front lane, is still checked
   every tick ahead of whatever `self.mode` currently is), but it never
   perturbs the side encounter's own bookkeeping, so resuming afterward
   picks up exactly where it left off.

Only after these two checks does side-phase logic (§2, including the new
`LOCAL_NARROW`-triggers-a-`RECOVERY_ALLOWED`-interruption rule) run.

## 4. `apply()` signature and `dt`/position ownership (unchanged from revision 3/4)

```
apply(decision: LocalObstacleDecision, now_s: float, own_x: float, own_y: float) -> LocalObstacleDecision
```

`dt` computed internally from consecutive `now_s` (clamped). `own_x`/
`own_y` feed only the side lane's `origin`/`distance` tracking; ignored by
the front lane.

## 5. Expected changes to `cooperative_avoider.py` (four small, targeted edits — one more than revision 4)

1. `_local_decision()` passes `self.local_latch.hysteresis_hint()` instead
   of `self.mode` as `previous_mode` (unchanged from revision 2-4).
2. Call site: `self.local_latch.apply(decision, now, self.own_state.x_m, self.own_state.y_m)`
   (unchanged from revision 3-4).
3. **New in this revision**: inside the existing
   `if local_decision is not None and local_decision.active and not
   local_decision.safety_stop:` branch, add a special case checked first:
   ```
   if local_decision.mode == "LOCAL_RECOVERY_READY":
       self.mode, linear, angular = self._local_recover_command(heading_error, now)
   else:
       self.mode = local_decision.mode
       linear, angular = local_decision.linear_mps, local_decision.angular_rps
       if self.mode != "LOCAL_CLEARANCE":
           self.local_bypass_origin = None
   ```
   where `_local_recover_command()` is a new small private helper holding
   **exactly** the formula the existing `elif self.mode == "LOCAL_RECOVER":`
   branch already uses (source line ~439-445) — **not** the peer-CPA
   `elif self.mode == "RECOVER":` branch's formula, which is a different,
   faster gain/clamp pair (`1.2 * heading_error`, `±0.30 rad/s`) tuned for
   the communicated-CPA recovery, not the local/box-avoidance recovery:

   ```
   def _local_recover_command(self, heading_error, now):
       linear = self.avoidance_speed
       angular = clamp(
           0.8 * heading_error,
           -self.local_recovery_turn,
           self.local_recovery_turn,
       )
       mode = "LOCAL_RECOVER"
       if abs(heading_error) < 0.08:
           mode = "CRUISE"
           self.local_bypass_origin = None
           self.recovery_source = "local"
           self.recovery_completed_at = now
       return mode, linear, angular
   ```

   `self.local_recovery_turn` is the existing locked `local_recovery_turn_rps`
   parameter (`0.18 rad/s`) — untouched, reused, not widened. Both the new
   `LOCAL_RECOVERY_READY` special case and the existing
   `elif self.mode == "LOCAL_RECOVER":` branch call this same helper, so
   there is exactly one copy of the formula, not two that could drift
   apart. (Revision 5 originally wrote this special case using the
   peer-CPA `RECOVER` branch's `1.2`/`±0.30` formula by transcription
   error; this has been corrected here — the local-avoidance recovery must
   use the slower, locked `0.8`/`±0.18` formula, exactly matching what
   `LOCAL_RECOVER` already does today.)
4. When `local_decision.safety_stop` is `True`, `_log()` is passed the raw
   `local_decision.mode` string alongside the hardcoded
   `self.mode = "SAFE_STOP_LOCAL_SENSORS"`, so
   `LOCAL_SIDE_ENCOUNTER_FAILSAFE` is distinguishable in logs/tests
   (unchanged from revision 3-4).

No `local_bypass_origin`/`_bypass_progress()` involvement anywhere in the
capped/recovery path — the latch owns `origin`/`distance` end-to-end, and
the `LOCAL_RECOVERY_READY` hand-off bypasses the generic fallback branch
entirely (item 3 above never falls through to it). That existing
mechanism continues to serve only its original role: a clean, non-capped
encounter's own straight-then-recover tail, which this design does not
touch.

## 6. `max_side_encounter_turn_rad` and the other three constants: still candidates, not locked (reaffirmed, unchanged conclusion)

All four numeric thresholds introduced or reused by this design —
`max_side_encounter_turn_rad` (`0.50 rad`), `max_bypass_extension_m`
(`0.30 m`), `side_clear_confirm_s` (`1.0 s`), `rearm_quiet_s` (`1.5 s`) —
remain **conservative starting candidates for exclusionary `v2` pilots,
not validated constants**. `pilot_04`'s only clean, uncontested encounter
was front-lane (`LOCAL_FRONT_WARN`, peak `|yaw|=0.4323 rad`); no pilot in
the current dataset contains a clean side-lane-only encounter, so none of
these four values have direct empirical support yet. §7's item 14
requires the design's implementation to ship with this caveat intact in
code comments/docstrings, not just in this document, so it cannot be
silently forgotten once implemented.

## 7. Test checklist (revision 5: adds the `RECOVERY_ALLOWED`-interruption and hand-off-purity cases, keeps all revision 4 items)

1. `TURNING` budget clipping: exact, no overshoot (unchanged from
   revision 4 item 1).
2. `LOCAL_CLEARANCE` ticks during `TURNING` count toward the budget
   (unchanged, revision 4 item 2).
3. Single inactive tick in `CAPPED_BYPASS` followed by active: does not
   transition to `RECOVERY_ALLOWED` (unchanged, revision 4 item 3).
4. Flicker that never reaches an unbroken `side_clear_confirm_s` run: must
   reach `FAILSAFE` at `max_bypass_extension_m`, not stall (unchanged,
   revision 4 item 4).
5. Genuinely unbroken quiet run of exactly `side_clear_confirm_s` with
   distance already satisfied: transitions to `RECOVERY_ALLOWED` on the
   threshold tick (unchanged, revision 4 item 5).
6. **New**: on the `CAPPED_BYPASS → RECOVERY_ALLOWED` transition tick,
   assert the returned decision is exactly
   `active=True, mode="LOCAL_RECOVERY_READY", angular_rps=0.0`.
7. **New**: while in `RECOVERY_ALLOWED`, feed a raw `LOCAL_LEFT_SIDE`/
   `LOCAL_RIGHT_SIDE` re-trigger: assert the phase reverts to
   `CAPPED_BYPASS` on that exact tick, `origin` and `budget_used_rad` are
   byte-identical to their values before the interruption, the returned
   decision is `mode="LOCAL_SIDE_BYPASS", angular_rps=0.0` (**not** the
   side-avoidance turn rate — direct test of "角速度不重新变为侧向避让角速度"),
   and `quiet_since_s` is `None` immediately after.
8. **New**: identical to item 7 but with a raw `LOCAL_NARROW` decision
   instead of `LOCAL_LEFT_SIDE`/`RIGHT_SIDE` — same assertions, confirming
   `LOCAL_NARROW` is treated as a genuine interrupt during
   `RECOVERY_ALLOWED` (reviewer point 1's explicit inclusion).
9. **New**: after an interruption (item 7/8) sends the phase back to
   `CAPPED_BYPASS`, feed a repeating active/inactive pattern that never
   reaches confirmation again, with position advancing past
   `max_bypass_extension_m`: assert `FAILSAFE` is reached (not a silent
   drift into `RECOVERY_ALLOWED` and not an infinite loop) — direct test
   of "反复抖动时必须最终触发extension FAILSAFE".
10. **New**: after a genuine, uninterrupted `RECOVERY_ALLOWED` (no
    re-trigger at all), assert exactly **one** `LOCAL_RECOVERY_READY`
    pulse was returned (on entry only) and every subsequent tick until
    `CLOSED` returns a plain inactive decision — direct test that the
    hand-off does not repeat or re-trigger the generic fallback.
11. **New** (integration-level, using the real `CooperativeAvoider` node
    or a lightweight harness around its `_control()` method): drive a full
    `TURNING → CAPPED_BYPASS → RECOVERY_ALLOWED (uninterrupted) → CLOSED`
    sequence and assert `self.local_bypass_origin` is **never** set
    (remains `None` throughout) — direct test of "recovery_ready 后不得创建
    第二个 bypass origin", and that `self.mode` transitions directly
    `"LOCAL_SIDE_BYPASS" → "LOCAL_RECOVER" → "CRUISE"` with no intervening
    generic-fallback `"LOCAL_BYPASS"` state ever appearing in the mode
    sequence.
12. Front-preemption mid-`CAPPED_BYPASS` **and** mid-`RECOVERY_ALLOWED`:
    `origin`, `budget_used_rad`, `quiet_since_s`, and phase are
    byte-identical before and after a `LOCAL_FRONT_DANGER`/
    `LOCAL_FRONT_WARN` tick, in both phases.
13. `LOCAL_SENSOR_INVALID` immediately pre-empts every side phase
    (`TURNING`/`CAPPED_BYPASS`/`RECOVERY_ALLOWED`/`FAILSAFE`), unchanged
    from `controller_v1` — existing safety-stop unit tests pass against
    the new latch unmodified.
14. Cumulative side turn across an entire encounter — including any
    number of `CAPPED_BYPASS ↔ RECOVERY_ALLOWED` interruption cycles —
    never exceeds `max_side_encounter_turn_rad`; this should be provable
    directly from the fact that `budget_used_rad` is only ever
    incremented during `TURNING` (§2.2) and is read-only (carried over,
    never re-accumulated) in every later phase, but is still worth an
    explicit assertion in a multi-cycle scenario test.
15. Rearm timing: `rearm_quiet_s` must be reached with **zero**
    interruptions from `RECOVERY_ALLOWED` to reach `CLOSED`; any
    interruption anywhere in the window restarts the requirement from
    zero on the next `RECOVERY_ALLOWED` entry (unchanged principle from
    revision 4, now also exercised via the interruption path rather than
    only a fresh encounter).
16. Existing `test_local_obstacle_logic.py`/`test_collision_math.py`
    suites unmodified and green — reported as a separate count from the
    new tests above (reviewer's explicit accounting requirement: "旧
    controller_v1 的41项测试保持全绿，新测试单独计数").
17. Full `colcon test --packages-select epuck2_comm`: 41 (unchanged
    `v1` tests) + N (new, itemized above) tests, 0 failures.

## 8. Risk analysis (carried forward from revision 4, with one addition)

`LOCAL_FRONT_DANGER` preemption remains an additional, independent safety
layer, not the primary safety argument. The primary arguments are: the
hard, flicker-immune `max_bypass_extension_m` ceiling (§2.3); the
recorded `LOCAL_SIDE_ENCOUNTER_FAILSAFE` stop; and, new in this revision,
**`RECOVERY_ALLOWED`'s continuous re-monitoring** (§2.4) — recovery is no
longer a one-way, un-interruptible commitment once entered, which removes
the specific hazard the reviewer identified (turning toward a
newly-reappeared obstacle while `LOCAL_RECOVER` runs unmonitored).

The shared `quiet_since_s` tracker (revision 4's risk-concentration
observation) still applies and is unchanged: one well-tested mechanism
serving two thresholds is preferred over independently-written checks
that could diverge, which is exactly what caused revision 3's gap.

**New risk surface in this revision**: the `LOCAL_RECOVERY_READY` special
case in `cooperative_avoider.py` (§5 item 3) duplicates the recovery-turn
formula rather than calling into the existing `elif` branch (which cannot
be called directly without restructuring the `_control()` method's
branching). This is flagged as an implementation-quality item (shared
helper method, §5) specifically so the duplication is a one-line risk
(two call sites of one helper) rather than two independently-maintained
formulas that could silently drift apart in a future edit.

## 9. Sequencing and out-of-scope items (unchanged from revision 2-4)

Implement → new tests green → full suite green → excluded `v2` static-box
pilot (extended runtime) → excluded `v2` CPA-only pilot
(`head_on_centered`, confirms no regression) → excluded `v2`
combined-scenario pilot → formal `v2` batches, with all `v2` baselines
rebuilt from scratch and `v1` data never merged into `v2` statistics.
`combined_task_coordinator.py`'s `encounter_seen` sampling gap remains
out of scope for this commit.

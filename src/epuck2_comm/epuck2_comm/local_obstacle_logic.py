"""Pure local-obstacle decision logic for simulation and physical e-puck2.

The distance thresholds are derived from the 2026-07-15 calibration of school
robot 5809.  The state publisher normalises clear IR returns to ``+Inf`` and
encodes sensor freshness in ``validity_flags``.  This module intentionally has
no ROS dependency so the safety and priority rules can be unit-tested.

controller_v3_unified_encounter_20260717: ``LocalAvoidanceLatch`` was
rewritten around a single, unified "local obstacle encounter" state machine
(``CLOSED -> ACTIVE -> CONSTRAINED -> RECOVERY_ALLOWED -> CLOSED``, with a
terminal ``FAILSAFE`` branch) that replaces controller_v2_local_latch's two
independent front/side turn budgets with one shared turn ledger. This fixes a
controller_v2 defect (see ``pilot_a2``, bag
``controller_v2_local_latch_20260717_static_box_pilot_a2``) where a raw
``LOCAL_FRONT_DANGER`` episode (never budgeted, by design, so its own
emergency response is never weakened) and a subsequent ``LOCAL_LEFT_SIDE``
episode against the *same* obstacle each received an independent turn
allowance, and the two compounded into a real geometric collision even
though each budget was individually respected. See
``experiments/controller_v2_local_latch_20260717/config/static_box_v2/
controller_v3_unified_encounter_design_20260717.md`` for the full design and
the raw-sensor forensic timeline that motivated it.

controller_v1's front-danger/front-warn responsiveness is unchanged: while a
raw ``LOCAL_FRONT_DANGER``/``LOCAL_FRONT_WARN`` decision is genuinely active,
it is always returned completely unmodified, regardless of ledger state. The
ledger only ever affects what happens *after* raw activity ends: its own
``LOCAL_CLEARANCE`` continuation, and any further ``LOCAL_LEFT_SIDE``/
``LOCAL_RIGHT_SIDE``/``LOCAL_NARROW`` trigger within the same encounter now
draw against what is left of the one shared ledger, never a fresh
allowance.
"""

import math
from dataclasses import dataclass, field


IR_VALID_FLAG = 2
TOF_VALID_FLAG = 4

# controller_v3_unified_encounter_20260717 candidate thresholds.
#
# *** v3 pilot candidates, NOT validated constants. ***
# pilot_a2 (controller_v2) showed a single raw LOCAL_FRONT_DANGER episode
# alone can consume ~0.54 rad of yaw while geometric clearance was already
# only ~0.007 m -- these defaults leave some headroom above that single
# observed episode so a subsequent side correction is not automatically
# impossible, but NONE of them should be read as a proven safe bound. See
# controller_v3_unified_encounter_design_20260717.md sections 6.1 and 6.7
# for the full reasoning and the explicit statement that only a real,
# Supervisor-instrumented v3 pilot (not this module, not any unit test) may
# be cited as evidence of a safe geometric clearance margin.
DEFAULT_MAX_TURN_LEDGER_RAD = 0.70
DEFAULT_MAX_BYPASS_EXTENSION_M = 0.30
DEFAULT_SIDE_CLEAR_CONFIRM_S = 1.0
DEFAULT_REARM_QUIET_S = 1.5
DEFAULT_CONSTRAINED_SPEED_MPS = 0.006
DEFAULT_MAX_CONSTRAINED_DURATION_S = 10.0

SIDE_ACTIVE_MODES = ("LOCAL_LEFT_SIDE", "LOCAL_RIGHT_SIDE")
FRONT_ACTIVE_MODES = ("LOCAL_FRONT_DANGER", "LOCAL_FRONT_WARN")


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class LocalObstacleDecision:
    active: bool
    safety_stop: bool
    mode: str
    linear_mps: float
    angular_rps: float


@dataclass
class LocalAvoidanceLatch:
    """One shared "local obstacle encounter" state machine.

    Front (``LOCAL_FRONT_DANGER``/``LOCAL_FRONT_WARN``), side
    (``LOCAL_LEFT_SIDE``/``LOCAL_RIGHT_SIDE``), and ``LOCAL_NARROW`` all
    read and write the *same* encounter record -- there are no longer two
    independent lanes. ``LOCAL_SENSOR_INVALID`` bypasses everything below
    unconditionally, exactly as in every prior version.

    Phases: ``CLOSED`` (idle) -> ``ACTIVE`` (turning; raw front/side/narrow
    active, or continuing via a ``LOCAL_CLEARANCE`` tail, both drawing on
    ``turn_ledger_used_rad``) -> ``CONSTRAINED`` (ledger exhausted; HOLD
    while any raw local decision is active, CREEP only once genuinely
    clear -- see ``apply()``) -> ``RECOVERY_ALLOWED`` (masks the side/front
    output so ``cooperative_avoider.py``'s own ``LOCAL_RECOVER`` branch can
    run undisturbed, but reopens back to ``CONSTRAINED`` -- never
    ``ACTIVE`` -- the instant a genuine raw local decision reappears) ->
    ``CLOSED`` once genuinely quiet for ``rearm_quiet_s``. ``CONSTRAINED``
    can also reach ``FAILSAFE`` (distance or time ceiling exceeded without
    a confirmed-clear resolution) -- ``FAILSAFE`` is a **terminal, hard
    latch**: nothing in this class transitions it back to ``CLOSED``: only
    restarting the controller node (a fresh ``LocalAvoidanceLatch``
    instance) ends it.
    """

    clear_hold_s: float = 1.0
    clearance_speed_mps: float = 0.006
    clearance_turn_rps: float = 0.30
    local_bypass_distance_m: float = 0.08

    # *** v3 pilot candidates, NOT validated constants. *** See the
    # module-level DEFAULT_* constants' docstring above.
    max_turn_ledger_rad: float = DEFAULT_MAX_TURN_LEDGER_RAD
    max_bypass_extension_m: float = DEFAULT_MAX_BYPASS_EXTENSION_M
    side_clear_confirm_s: float = DEFAULT_SIDE_CLEAR_CONFIRM_S
    rearm_quiet_s: float = DEFAULT_REARM_QUIET_S
    constrained_speed_mps: float = DEFAULT_CONSTRAINED_SPEED_MPS
    max_constrained_duration_s: float = DEFAULT_MAX_CONSTRAINED_DURATION_S

    # Encounter state.
    phase: str = "CLOSED"
    encounter_start_yaw: float = field(default=None)
    turn_ledger_used_rad: float = 0.0
    previous_yaw: float = field(default=None)
    turn_sign: float = -1.0
    last_raw_mode: str = ""
    origin: tuple = field(default=None)
    constrained_entered_s: float = field(default=None)
    # Timestamp of the most recent tick where ANY raw local decision (front,
    # side, or narrow) was active, updated unconditionally in apply() before
    # any phase dispatch -- this is the single source of truth "how long has
    # it genuinely been quiet" is measured against everywhere (the ACTIVE
    # phase's clear_hold_s tail, CONSTRAINED's side_clear_confirm_s, and
    # RECOVERY_ALLOWED's rearm_quiet_s all just compare now_s - last_active_s
    # to their own threshold; none of them need their own separate "first
    # noticed quiet" bookkeeping).
    last_active_s: float = field(default=None)

    def hysteresis_hint(self) -> str:
        """previous_mode hint for decide_local_obstacle()'s own hysteresis.

        Returns the most recent genuine raw local mode for the whole
        duration of an open encounter (including through ``CONSTRAINED``/
        ``RECOVERY_ALLOWED``/``FAILSAFE``), independent of whatever
        synthetic mode name this latch is emitting this tick -- this is
        what keeps decide_local_obstacle()'s front/side release-hysteresis
        bands in effect for the whole encounter instead of silently
        reverting to the tighter warn-only thresholds once the latch starts
        emitting a synthetic mode name.
        """
        if self.phase != "CLOSED":
            return self.last_raw_mode
        return ""

    def apply(
        self,
        decision: LocalObstacleDecision,
        now_s: float,
        own_x: float,
        own_y: float,
        own_yaw_rad: float,
    ) -> LocalObstacleDecision:
        if decision.safety_stop:
            return decision

        if self.phase == "FAILSAFE":
            # Hard latch: nothing updates, nothing is re-evaluated, ever
            # again for this node's lifetime.
            return LocalObstacleDecision(
                True, True, "LOCAL_ENCOUNTER_FAILSAFE", 0.0, 0.0
            )

        front_active = decision.active and decision.mode in FRONT_ACTIVE_MODES
        side_active = decision.active and decision.mode in SIDE_ACTIVE_MODES
        is_narrow = decision.mode == "LOCAL_NARROW"
        any_raw_active = front_active or side_active or is_narrow

        if self.phase == "CLOSED":
            if not any_raw_active:
                return decision
            self._open_encounter(now_s, own_yaw_rad)

        # An encounter is open now (possibly just opened this tick).
        self._update_ledger(own_yaw_rad)

        if any_raw_active:
            self.last_raw_mode = decision.mode
            self.last_active_s = now_s
            if abs(decision.angular_rps) > 1e-9:
                self.turn_sign = 1.0 if decision.angular_rps > 0.0 else -1.0

        if front_active:
            # Unconditional priority: front danger/warn is never clipped,
            # delayed, or gated by ledger state, in any phase -- the
            # decision below is always returned completely unmodified.
            if self.phase == "RECOVERY_ALLOWED":
                # A front hazard reappearing while "recovery allowed" is
                # exactly the kind of renewed local risk that must revert
                # to CONSTRAINED (never re-arming a fresh ledger), the same
                # way a side/narrow re-trigger already does -- it must not
                # be silently absorbed into the masked recovery phase.
                # ACTIVE and CONSTRAINED are left untouched: a front
                # preemption there simply resumes whatever phase it was
                # once front clears.
                self.phase = "CONSTRAINED"
            return decision

        if self.phase == "ACTIVE":
            return self._handle_active(side_active, is_narrow, decision, now_s, own_x, own_y)
        if self.phase == "CONSTRAINED":
            return self._handle_constrained(side_active, is_narrow, now_s, own_x, own_y)
        if self.phase == "RECOVERY_ALLOWED":
            return self._handle_recovery_allowed(side_active, is_narrow, now_s, own_x, own_y)

        raise AssertionError(f"unreachable phase {self.phase!r}")

    # -- encounter lifecycle -----------------------------------------------

    def _open_encounter(self, now_s: float, own_yaw_rad: float) -> None:
        self.phase = "ACTIVE"
        self.encounter_start_yaw = float(own_yaw_rad)
        self.previous_yaw = float(own_yaw_rad)
        self.turn_ledger_used_rad = 0.0
        self.turn_sign = -1.0
        self.last_raw_mode = ""
        self.origin = None
        self.constrained_entered_s = None
        self.last_active_s = now_s

    def _update_ledger(self, own_yaw_rad: float) -> None:
        delta = normalize_angle(float(own_yaw_rad) - self.previous_yaw)
        self.turn_ledger_used_rad += abs(delta)
        self.previous_yaw = float(own_yaw_rad)

    def _close(self) -> None:
        self.phase = "CLOSED"
        self.encounter_start_yaw = None
        self.turn_ledger_used_rad = 0.0
        self.previous_yaw = None
        self.turn_sign = -1.0
        self.last_raw_mode = ""
        self.origin = None
        self.constrained_entered_s = None
        self.last_active_s = None

    # -- ACTIVE --------------------------------------------------------

    def _handle_active(
        self,
        side_active: bool,
        is_narrow: bool,
        decision: LocalObstacleDecision,
        now_s: float,
        own_x: float,
        own_y: float,
    ) -> LocalObstacleDecision:
        if side_active:
            if self.turn_ledger_used_rad < self.max_turn_ledger_rad:
                return decision
            self._enter_constrained(now_s, own_x, own_y)
            return self._constrained_tick(True, False, now_s, own_x, own_y)

        if is_narrow:
            return decision

        # Raw decision inactive: controller_v1/v2-style clearance tail,
        # now drawing on the one shared ledger regardless of whether the
        # turn it continues came from a front or a side episode. Measured
        # from last_active_s (the last genuinely active tick), not from
        # when this inactive tick happened to be observed -- robust to
        # irregular polling, matching controller_v1/v2's exact semantics.
        if now_s - self.last_active_s <= self.clear_hold_s:
            if self.turn_ledger_used_rad < self.max_turn_ledger_rad:
                return LocalObstacleDecision(
                    True, False, "LOCAL_CLEARANCE",
                    self.clearance_speed_mps, self.clearance_turn_rps * self.turn_sign,
                )
            self._enter_constrained(now_s, own_x, own_y)
            return self._constrained_tick(False, False, now_s, own_x, own_y)

        # clear_hold_s elapsed with no further trigger: ordinary close --
        # the ledger was never exhausted, otherwise CONSTRAINED would
        # already have been entered above.
        self._close()
        return decision

    # -- CONSTRAINED -----------------------------------------------------

    def _enter_constrained(self, now_s: float, own_x: float, own_y: float) -> None:
        self.phase = "CONSTRAINED"
        if self.origin is None:
            self.origin = (float(own_x), float(own_y))
        if self.constrained_entered_s is None:
            self.constrained_entered_s = now_s

    def _handle_constrained(
        self, side_active: bool, is_narrow: bool, now_s: float, own_x: float, own_y: float
    ) -> LocalObstacleDecision:
        return self._constrained_tick(side_active, is_narrow, now_s, own_x, own_y)

    def _constrained_tick(
        self, side_active: bool, is_narrow: bool, now_s: float, own_x: float, own_y: float
    ) -> LocalObstacleDecision:
        distance = math.hypot(own_x - self.origin[0], own_y - self.origin[1])
        quiet_confirmed = now_s - self.last_active_s >= self.side_clear_confirm_s

        if distance >= self.local_bypass_distance_m and quiet_confirmed:
            self.phase = "RECOVERY_ALLOWED"
            return LocalObstacleDecision(True, False, "LOCAL_RECOVERY_READY", 0.0, 0.0)

        time_in_constrained = now_s - self.constrained_entered_s
        if (
            distance >= self.max_bypass_extension_m
            or time_in_constrained >= self.max_constrained_duration_s
        ):
            self.phase = "FAILSAFE"
            return LocalObstacleDecision(
                True, True, "LOCAL_ENCOUNTER_FAILSAFE", 0.0, 0.0
            )

        if side_active or is_narrow:
            # A raw local sensor is still reporting proximity: hold, never
            # creep blindly toward something the robot's own sensors say
            # is still there.
            return LocalObstacleDecision(True, False, "LOCAL_ENCOUNTER_HOLD", 0.0, 0.0)
        # Genuinely clear this tick: a slow, cautious creep is permitted.
        return LocalObstacleDecision(
            True, False, "LOCAL_ENCOUNTER_CREEP", self.constrained_speed_mps, 0.0
        )

    # -- RECOVERY_ALLOWED --------------------------------------------------

    def _handle_recovery_allowed(
        self, side_active: bool, is_narrow: bool, now_s: float, own_x: float, own_y: float
    ) -> LocalObstacleDecision:
        if side_active or is_narrow:
            # Genuine re-trigger: fall back to CONSTRAINED (never ACTIVE --
            # no fresh ledger, no resumed turning), retaining origin,
            # ledger, and constrained_entered_s from this same encounter.
            # last_active_s was already refreshed to now_s above (apply()'s
            # unconditional update), so the rearm clock is correctly reset.
            self.phase = "CONSTRAINED"
            return self._constrained_tick(side_active, is_narrow, now_s, own_x, own_y)
        if now_s - self.last_active_s >= self.rearm_quiet_s:
            self._close()
        return LocalObstacleDecision(False, False, "LOCAL_CLEAR", 0.0, 0.0)


def _distance(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return math.inf
    return value if math.isfinite(value) and value >= 0.0 else math.inf


def _turn_away(left_m: float, right_m: float, tie_margin_m: float = 0.002):
    """Return ROS angular sign: positive left, negative right.

    A centred or ambiguous obstacle uses the same deterministic pass-right
    convention as the communication-aware collision avoider.
    """
    if left_m + tie_margin_m < right_m:
        return -1.0
    if right_m + tie_margin_m < left_m:
        return 1.0
    return -1.0


def decide_local_obstacle(
    front_distance_m,
    left_distance_m,
    right_distance_m,
    validity_flags: int,
    previous_mode: str = "",
    *,
    front_danger_m: float = 0.100,
    front_warn_m: float = 0.180,
    front_release_m: float = 0.220,
    side_danger_m: float = 0.042,
    side_warn_m: float = 0.052,
    side_release_m: float = 0.058,
    warning_speed_mps: float = 0.010,
    side_speed_mps: float = 0.012,
    danger_turn_rps: float = 0.65,
    warning_turn_rps: float = 0.45,
    side_turn_rps: float = 0.30,
) -> LocalObstacleDecision:
    """Select a local safety action from the compact state summary.

    Static-obstacle actions are deliberately returned independently of the
    cooperative policy.  The caller gives them priority over peer-state CPA
    avoidance.  ``previous_mode`` provides release hysteresis.
    """
    flags = int(validity_flags)
    ir_valid = (flags & IR_VALID_FLAG) != 0
    tof_valid = (flags & TOF_VALID_FLAG) != 0
    if not ir_valid and not tof_valid:
        return LocalObstacleDecision(True, True, "LOCAL_SENSOR_INVALID", 0.0, 0.0)

    front = _distance(front_distance_m)
    left = _distance(left_distance_m) if ir_valid else math.inf
    right = _distance(right_distance_m) if ir_valid else math.inf
    turn = _turn_away(left, right)

    front_threshold = front_warn_m
    if previous_mode.startswith("LOCAL_FRONT"):
        front_threshold = front_release_m

    if front <= front_danger_m:
        return LocalObstacleDecision(
            True, False, "LOCAL_FRONT_DANGER", 0.0, danger_turn_rps * turn
        )
    if front <= front_threshold:
        return LocalObstacleDecision(
            True,
            False,
            "LOCAL_FRONT_WARN",
            warning_speed_mps,
            warning_turn_rps * turn,
        )

    if not ir_valid:
        return LocalObstacleDecision(False, False, "LOCAL_CLEAR", 0.0, 0.0)

    side_threshold = side_warn_m
    if previous_mode in ("LOCAL_LEFT_SIDE", "LOCAL_RIGHT_SIDE", "LOCAL_NARROW"):
        side_threshold = side_release_m

    if left <= side_danger_m and right <= side_danger_m:
        return LocalObstacleDecision(True, False, "LOCAL_NARROW", 0.0, 0.0)
    if left <= side_threshold and right <= side_threshold:
        return LocalObstacleDecision(
            True, False, "LOCAL_NARROW", min(side_speed_mps, 0.006), 0.0
        )
    if left <= side_threshold:
        return LocalObstacleDecision(
            True, False, "LOCAL_LEFT_SIDE", side_speed_mps, -side_turn_rps
        )
    if right <= side_threshold:
        return LocalObstacleDecision(
            True, False, "LOCAL_RIGHT_SIDE", side_speed_mps, side_turn_rps
        )
    return LocalObstacleDecision(False, False, "LOCAL_CLEAR", 0.0, 0.0)

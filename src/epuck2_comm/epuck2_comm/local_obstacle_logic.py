"""Pure local-obstacle decision logic for simulation and physical e-puck2.

The distance thresholds are derived from the 2026-07-15 calibration of school
robot 5809.  The state publisher normalises clear IR returns to ``+Inf`` and
encodes sensor freshness in ``validity_flags``.  This module intentionally has
no ROS dependency so the safety and priority rules can be unit-tested.

controller_v2_local_latch_20260717: ``LocalAvoidanceLatch`` was extended with
a side-lane phase state machine (``TURNING`` -> ``CAPPED_BYPASS`` ->
``RECOVERY_ALLOWED`` -> ``CLOSED``, with a ``FAILSAFE`` branch) to fix a
controller_v1 defect where a flickering IR reading near a side threshold
boundary could re-arm an unbounded single-direction turn (see
``controller_v2_local_latch_design_20260717.md`` for the full derivation and
evidence). The front lane (``LOCAL_FRONT_DANGER`` / ``LOCAL_FRONT_WARN``) and
``LOCAL_SENSOR_INVALID`` keep controller_v1's original, unbounded behaviour
untouched; see the module-level constants below for the new side-lane
thresholds this introduces, all of which are v2 pilot candidates, not
validated values.
"""

import math
from dataclasses import dataclass, field


IR_VALID_FLAG = 2
TOF_VALID_FLAG = 4

# controller_v2_local_latch_20260717 side-lane thresholds.
#
# *** v2 pilot candidates, NOT validated constants. ***
# pilot_04's only clean, uncontested box encounter was a front-lane
# (LOCAL_FRONT_WARN) response, not a side-lane one; no pilot in the
# controller_v1 dataset contains a clean side-lane-only encounter. These four
# values are conservative starting points for exclusionary v2 pilots and must
# be re-tuned against a dedicated side-lane-triggering pilot before being
# treated as settled. See controller_v2_local_latch_design_20260717.md
# sections 6 and 9.
DEFAULT_MAX_SIDE_ENCOUNTER_TURN_RAD = 0.50
DEFAULT_MAX_BYPASS_EXTENSION_M = 0.30
DEFAULT_SIDE_CLEAR_CONFIRM_S = 1.0
DEFAULT_REARM_QUIET_S = 1.5

SIDE_ACTIVE_MODES = ("LOCAL_LEFT_SIDE", "LOCAL_RIGHT_SIDE")
FRONT_ACTIVE_MODES = ("LOCAL_FRONT_DANGER", "LOCAL_FRONT_WARN")


@dataclass(frozen=True)
class LocalObstacleDecision:
    active: bool
    safety_stop: bool
    mode: str
    linear_mps: float
    angular_rps: float


@dataclass
class LocalAvoidanceLatch:
    """Hold the last avoidance direction through intermittent range dropouts.

    Two lanes are tracked independently:

    - The front lane (``LOCAL_FRONT_DANGER`` / ``LOCAL_FRONT_WARN`` and their
      ``LOCAL_CLEARANCE`` continuation) is unbounded, byte-for-byte
      controller_v1 behaviour — never touched by the side-lane state machine
      below.
    - The side lane (``LOCAL_LEFT_SIDE`` / ``LOCAL_RIGHT_SIDE`` and their
      continuation) is a bounded phase state machine:
      ``CLOSED -> TURNING -> CAPPED_BYPASS -> RECOVERY_ALLOWED -> CLOSED``,
      with a ``FAILSAFE`` branch out of ``CAPPED_BYPASS``. ``LOCAL_NARROW``
      is never itself capped or replaced, but it counts as a genuine hazard
      signal for the purpose of the side lane's quiet-time tracking and for
      interrupting ``RECOVERY_ALLOWED`` (see ``apply()``).
    """

    clear_hold_s: float = 1.0
    clearance_speed_mps: float = 0.006
    clearance_turn_rps: float = 0.30
    side_speed_mps: float = 0.012
    local_bypass_distance_m: float = 0.08

    # *** v2 pilot candidates, NOT validated constants. *** See the
    # module-level DEFAULT_* constants' docstring above.
    max_side_encounter_turn_rad: float = DEFAULT_MAX_SIDE_ENCOUNTER_TURN_RAD
    max_bypass_extension_m: float = DEFAULT_MAX_BYPASS_EXTENSION_M
    side_clear_confirm_s: float = DEFAULT_SIDE_CLEAR_CONFIRM_S
    rearm_quiet_s: float = DEFAULT_REARM_QUIET_S

    # Front lane state (unbounded, controller_v1-identical).
    front_last_active_s: float = -math.inf
    front_turn_sign: float = -1.0

    # Side lane phase state machine.
    side_phase: str = "CLOSED"
    side_turn_sign: float = -1.0
    side_hysteresis_mode: str = ""
    side_budget_used_rad: float = 0.0
    side_origin: tuple = field(default=None)
    side_quiet_since_s: float = field(default=None)

    _last_call_s: float = field(default=None, repr=False)

    def hysteresis_hint(self) -> str:
        """previous_mode hint for decide_local_obstacle()'s own hysteresis.

        Returns the locked side mode for the whole duration of a side
        encounter (including through CAPPED_BYPASS/RECOVERY_ALLOWED/
        FAILSAFE), independent of whatever synthetic mode name this latch is
        emitting this tick — fixes the controller_v1 bug where
        decide_local_obstacle()'s side_release_m hysteresis silently stopped
        applying once the latch started emitting "LOCAL_CLEARANCE".
        """
        if self.side_phase != "CLOSED":
            return self.side_hysteresis_mode
        return ""

    def _dt(self, now_s: float) -> float:
        if self._last_call_s is None:
            dt = 0.0
        else:
            dt = min(0.20, max(0.0, now_s - self._last_call_s))
        self._last_call_s = float(now_s)
        return dt

    def apply(
        self,
        decision: LocalObstacleDecision,
        now_s: float,
        own_x: float,
        own_y: float,
    ) -> LocalObstacleDecision:
        dt = self._dt(now_s)

        if decision.safety_stop:
            return decision

        if decision.mode in FRONT_ACTIVE_MODES:
            self.front_last_active_s = float(now_s)
            if abs(decision.angular_rps) > 1e-9:
                self.front_turn_sign = 1.0 if decision.angular_rps > 0.0 else -1.0
            return decision

        is_narrow = decision.mode == "LOCAL_NARROW"
        is_side_active = decision.active and decision.mode in SIDE_ACTIVE_MODES

        if self.side_phase == "CLOSED":
            if is_side_active:
                self._start_turning(decision, now_s)
                return self._handle_turning(decision, True, False, now_s, dt, own_x, own_y)
            if is_narrow:
                return decision
            return self._front_clearance_or_clear(decision, now_s)

        if self.side_phase == "TURNING":
            return self._handle_turning(decision, is_side_active, is_narrow, now_s, dt, own_x, own_y)

        if self.side_phase == "CAPPED_BYPASS":
            return self._handle_capped_bypass(decision, is_side_active, is_narrow, now_s, own_x, own_y)

        if self.side_phase == "RECOVERY_ALLOWED":
            return self._handle_recovery_allowed(decision, is_side_active, is_narrow, now_s)

        if self.side_phase == "FAILSAFE":
            return self._handle_failsafe(is_side_active, is_narrow, now_s)

        raise AssertionError(f"unreachable side_phase {self.side_phase!r}")

    # -- side lane phase handlers -----------------------------------------

    def _start_turning(self, decision: LocalObstacleDecision, now_s: float) -> None:
        self.side_phase = "TURNING"
        self.side_turn_sign = 1.0 if decision.angular_rps > 0.0 else -1.0
        self.side_hysteresis_mode = decision.mode
        self.side_budget_used_rad = 0.0
        self.side_origin = None
        self.side_quiet_since_s = None

    def _clip_to_budget(self, angular_rps: float, dt: float) -> float:
        remaining = max(0.0, self.max_side_encounter_turn_rad - self.side_budget_used_rad)
        if dt <= 0.0:
            return 0.0
        return math.copysign(min(abs(angular_rps), remaining / dt), angular_rps)

    def _handle_turning(
        self,
        decision: LocalObstacleDecision,
        is_side_active: bool,
        is_narrow: bool,
        now_s: float,
        dt: float,
        own_x: float,
        own_y: float,
    ) -> LocalObstacleDecision:
        if is_side_active:
            increment = abs(decision.angular_rps) * dt
            self.side_turn_sign = 1.0 if decision.angular_rps > 0.0 else -1.0
            self.side_hysteresis_mode = decision.mode
            self.side_quiet_since_s = None
            if self.side_budget_used_rad + increment <= self.max_side_encounter_turn_rad:
                self.side_budget_used_rad += increment
                return decision
            clipped_angular = self._clip_to_budget(decision.angular_rps, dt)
            self.side_budget_used_rad = self.max_side_encounter_turn_rad
            clipped = LocalObstacleDecision(
                decision.active, decision.safety_stop, decision.mode,
                decision.linear_mps, clipped_angular,
            )
            self._enter_capped_bypass(now_s, own_x, own_y)
            return clipped

        if is_narrow:
            self.side_quiet_since_s = None
            return decision

        # Raw decision inactive: controller_v1-style clearance tail,
        # budget-tracked in case a renewed flicker keeps it going.
        if self.side_quiet_since_s is None:
            self.side_quiet_since_s = now_s
        if now_s - self.side_quiet_since_s <= self.clear_hold_s:
            clearance_angular = self.clearance_turn_rps * self.side_turn_sign
            increment = abs(clearance_angular) * dt
            if self.side_budget_used_rad + increment <= self.max_side_encounter_turn_rad:
                self.side_budget_used_rad += increment
                return LocalObstacleDecision(
                    True, False, "LOCAL_CLEARANCE",
                    self.clearance_speed_mps, clearance_angular,
                )
            clipped_angular = self._clip_to_budget(clearance_angular, dt)
            self.side_budget_used_rad = self.max_side_encounter_turn_rad
            clipped = LocalObstacleDecision(
                True, False, "LOCAL_CLEARANCE",
                self.clearance_speed_mps, clipped_angular,
            )
            self._enter_capped_bypass(now_s, own_x, own_y)
            return clipped

        # clear_hold_s elapsed with no further trigger: ordinary v1-style
        # close — the budget was never exhausted, so no capping was needed.
        self._close()
        return decision

    def _enter_capped_bypass(self, now_s: float, own_x: float, own_y: float) -> None:
        self.side_phase = "CAPPED_BYPASS"
        if self.side_origin is None:
            self.side_origin = (float(own_x), float(own_y))
        self.side_quiet_since_s = None

    def _handle_capped_bypass(
        self,
        decision: LocalObstacleDecision,
        is_side_active: bool,
        is_narrow: bool,
        now_s: float,
        own_x: float,
        own_y: float,
    ) -> LocalObstacleDecision:
        if is_side_active or is_narrow:
            self.side_quiet_since_s = None
        elif self.side_quiet_since_s is None:
            self.side_quiet_since_s = now_s

        distance = math.hypot(own_x - self.side_origin[0], own_y - self.side_origin[1])
        quiet_confirmed = (
            self.side_quiet_since_s is not None
            and now_s - self.side_quiet_since_s >= self.side_clear_confirm_s
        )

        if distance >= self.local_bypass_distance_m and quiet_confirmed:
            self.side_phase = "RECOVERY_ALLOWED"
            # side_quiet_since_s carries over unchanged into RECOVERY_ALLOWED.
            return LocalObstacleDecision(True, False, "LOCAL_RECOVERY_READY", 0.0, 0.0)

        if distance >= self.max_bypass_extension_m:
            self.side_phase = "FAILSAFE"
            return LocalObstacleDecision(
                True, True, "LOCAL_SIDE_ENCOUNTER_FAILSAFE", 0.0, 0.0,
            )

        if is_narrow:
            # LOCAL_NARROW keeps its own independent, more cautious speed
            # (both sides close) rather than being sped up to the ordinary
            # bypass speed just because the turn budget happened to be
            # spent — it is never itself replaced by LOCAL_SIDE_BYPASS.
            # The distance/failsafe checks above still run unconditionally,
            # so a persistent narrow passage still cannot stall forever.
            return decision

        return LocalObstacleDecision(
            True, False, "LOCAL_SIDE_BYPASS", self.side_speed_mps, 0.0,
        )

    def _side_interrupt(
        self, decision: LocalObstacleDecision, now_s: float
    ) -> LocalObstacleDecision:
        """Genuine side/NARROW re-trigger during RECOVERY_ALLOWED.

        Falls back to CAPPED_BYPASS (never TURNING) with origin, budget and
        the bypass-extension distance all retained unchanged from this same
        encounter. A LOCAL_NARROW re-trigger keeps its own cautious speed
        (angular is already zero in its raw form); a side re-trigger's
        angular component is always suppressed, since resuming a turn is
        exactly what this design exists to prevent.
        """
        self.side_phase = "CAPPED_BYPASS"
        self.side_quiet_since_s = None
        if decision.mode == "LOCAL_NARROW":
            return decision
        return LocalObstacleDecision(
            True, False, "LOCAL_SIDE_BYPASS", self.side_speed_mps, 0.0,
        )

    def _handle_recovery_allowed(
        self,
        decision: LocalObstacleDecision,
        is_side_active: bool,
        is_narrow: bool,
        now_s: float,
    ) -> LocalObstacleDecision:
        if is_side_active or is_narrow:
            return self._side_interrupt(decision, now_s)
        if self.side_quiet_since_s is None:
            self.side_quiet_since_s = now_s
        if now_s - self.side_quiet_since_s >= self.rearm_quiet_s:
            self._close()
        return LocalObstacleDecision(False, False, "LOCAL_CLEAR", 0.0, 0.0)

    def _handle_failsafe(
        self, is_side_active: bool, is_narrow: bool, now_s: float
    ) -> LocalObstacleDecision:
        if is_side_active or is_narrow:
            self.side_quiet_since_s = None
        else:
            if self.side_quiet_since_s is None:
                self.side_quiet_since_s = now_s
            if now_s - self.side_quiet_since_s >= self.rearm_quiet_s:
                self._close()
                return LocalObstacleDecision(False, False, "LOCAL_CLEAR", 0.0, 0.0)
        return LocalObstacleDecision(
            True, True, "LOCAL_SIDE_ENCOUNTER_FAILSAFE", 0.0, 0.0,
        )

    def _close(self) -> None:
        self.side_phase = "CLOSED"
        self.side_budget_used_rad = 0.0
        self.side_origin = None
        self.side_hysteresis_mode = ""
        self.side_quiet_since_s = None

    def _front_clearance_or_clear(
        self, decision: LocalObstacleDecision, now_s: float
    ) -> LocalObstacleDecision:
        if now_s - self.front_last_active_s <= self.clear_hold_s:
            return LocalObstacleDecision(
                True, False, "LOCAL_CLEARANCE",
                self.clearance_speed_mps, self.clearance_turn_rps * self.front_turn_sign,
            )
        return decision


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

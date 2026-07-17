"""Pure local-obstacle decision logic for simulation and physical e-puck2.

The distance thresholds are derived from the 2026-07-15 calibration of school
robot 5809.  The state publisher normalises clear IR returns to ``+Inf`` and
encodes sensor freshness in ``validity_flags``.  This module intentionally has
no ROS dependency so the safety and priority rules can be unit-tested.

controller_v4_full_sensor_bypass_20260717: replaces controller_v3's single
turn-ledger ``LocalAvoidanceLatch`` with ``EncounterAvoidanceV4``, a five-phase
state machine (``CLOSED -> DETECT_TURN -> SIDE_TRACK -> PASS_CONFIRM ->
RECOVERY_ALLOWED -> CLOSED``, with a terminal ``FAILSAFE`` branch) driven by
this design motivation: pilot_a3's real-Webots collision (see
``experiments/controller_v3_unified_encounter_20260717/`` and its forensic
diagnostic in the design doc) showed the true failure was NOT the turn-ledger
mechanism (which worked correctly -- the second encounter's own ledger stayed
under its cap) but that v1/v2/v3's exit condition -- a fixed forward
``local_bypass_distance_m`` plus a pure-heading ``LOCAL_RECOVER`` correction --
never verified genuine LATERAL clearance from the obstacle before resuming
straight-line cruise, and the legacy generic ``LOCAL_BYPASS`` fallback branch
in ``cooperative_avoider.py`` could silently re-run after an encounter closed
cleanly through its own exit. v4 fixes both: PASS_CONFIRM requires a full
front->mid->rear sensor sequence PLUS an independent encounter-local lateral
displacement check (so a sensor blind spot at a grazing corner -- confirmed in
pilot_a3's forensic ps0-ps7 trace, where left/right briefly reported +Inf at
the exact instant of worst geometric penetration -- cannot alone fake
"passed") before any recovery is allowed, and every encounter exit is now an
explicit hand-off (never a bare inactive decision that could fall through to
the legacy fallback -- which controller_v4 no longer has at all; see
cooperative_avoider.py).

controller_v1's front-danger/front-warn responsiveness (raw thresholds) is
unchanged: ``decide_local_obstacle()`` below is untouched from v1 through v4.
What changed across versions is only how ``LocalAvoidanceLatch``/
``EncounterAvoidanceV4`` acts on top of that raw decision.
"""

import math
from dataclasses import dataclass, field


IR_VALID_FLAG = 2
TOF_VALID_FLAG = 4

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


@dataclass(frozen=True)
class ZoneSnapshot:
    """Zone-aggregated raw ps0-ps7 coverage (EpuckState's v4-added fields).

    ``left_front_m``/``left_mid_m``/``left_rear_m`` come from ps7 / min(ps5,
    ps6) / ps4; ``right_front_m``/``right_mid_m``/``right_rear_m`` from ps0 /
    min(ps1,ps2) / ps3 (state_publisher.py's zone split -- see its own
    docstring for the forensic ps0-ps7 mapping this mirrors). ``+Inf`` means
    no detection, matching front/left/right's convention.
    """

    left_front_m: float
    left_mid_m: float
    left_rear_m: float
    right_front_m: float
    right_mid_m: float
    right_rear_m: float


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

    Unchanged since controller_v1. Every later version (v2/v3/v4) only
    changes what happens to this raw decision afterwards.
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


# controller_v4_full_sensor_bypass_20260717 candidate thresholds.
#
# *** v4 pilot candidates, NOT validated constants. *** Only a real,
# Supervisor/analyzer-instrumented v4 pilot may certify a safe geometric
# margin -- these are starting points for the first static-box iteration,
# centrally configured here (not scattered across call sites) so any pilot
# revision changes exactly one place.
DEFAULT_MAX_INPLACE_TURN_RAD = 0.90
DEFAULT_MAX_TURN_LEDGER_RAD = 1.40
DEFAULT_MAX_BYPASS_EXTENSION_M = 0.40
DEFAULT_MAX_ENCOUNTER_DURATION_S = 18.0
DEFAULT_PASS_CONFIRM_HOLD_S = 1.0
DEFAULT_REARM_QUIET_S = 1.5
DEFAULT_SIDE_TRACK_CREEP_MPS = 0.010
DEFAULT_SIDE_TRACK_WARN_MPS = 0.005
# Danger-band half-width for this scenario: box_half_m(0.03) +
# robot_radius_m(0.035) = 0.065m. required_lateral_offset_m is the first
# UNCALIBRATED pilot candidate (0.065 + ~0.005 margin); it is not a general
# constant and must be re-derived for any different box/robot geometry.
DEFAULT_REQUIRED_LATERAL_OFFSET_M = 0.070
DEFAULT_REQUIRED_LONGITUDINAL_PROGRESS_M = 0.10

_TRACKING_ZONES = {
    "LEFT": ("left_front_m", "left_mid_m", "left_rear_m"),
    "RIGHT": ("right_front_m", "right_mid_m", "right_rear_m"),
}


@dataclass
class EncounterAvoidanceV4:
    """Five-phase local-obstacle encounter state machine.

    ``CLOSED`` (idle) -> ``DETECT_TURN`` (raw front danger/warn active, OR an
    encounter has just opened via a raw side/narrow trigger: linear is always
    forced to 0 here regardless of what the raw front decision itself would
    have commanded, turning in place toward the avoidance direction) ->
    ``SIDE_TRACK`` (front clear, tracking the obstacle along the flank it was
    pushed to via that flank's front/mid/rear zone triple; forward speed is
    gated by the tracking zone's own danger/warn/clear band, dropping to zero
    on the SAME tick -- never waiting on command smoothing -- the instant that
    band re-enters danger) -> ``PASS_CONFIRM`` (all exit conditions
    momentarily met; held for ``pass_confirm_hold_s`` before being trusted --
    any raw side/narrow re-trigger or lost condition reverts straight back to
    ``SIDE_TRACK``, not a fresh ledger) -> ``RECOVERY_ALLOWED`` (masks the
    output so the caller's own ``LOCAL_RECOVER`` branch can run via the
    ``LOCAL_RECOVERY_READY`` one-shot hand-off, reopening to ``SIDE_TRACK`` --
    never ``DETECT_TURN`` -- on any genuine re-trigger) -> ``CLOSED`` once
    genuinely quiet for ``rearm_quiet_s``. Any phase can reach ``FAILSAFE``
    (turn-ledger, bypass-distance, or duration ceiling exceeded without a
    confirmed pass) -- a **terminal, hard latch**: nothing in this class ever
    transitions it back to ``CLOSED``; only a fresh instance (node restart)
    ends it.
    """

    clearance_speed_mps: float = 0.006
    max_inplace_turn_rad: float = DEFAULT_MAX_INPLACE_TURN_RAD
    max_turn_ledger_rad: float = DEFAULT_MAX_TURN_LEDGER_RAD
    max_bypass_extension_m: float = DEFAULT_MAX_BYPASS_EXTENSION_M
    max_encounter_duration_s: float = DEFAULT_MAX_ENCOUNTER_DURATION_S
    pass_confirm_hold_s: float = DEFAULT_PASS_CONFIRM_HOLD_S
    rearm_quiet_s: float = DEFAULT_REARM_QUIET_S
    side_track_creep_mps: float = DEFAULT_SIDE_TRACK_CREEP_MPS
    side_track_warn_mps: float = DEFAULT_SIDE_TRACK_WARN_MPS
    required_lateral_offset_m: float = DEFAULT_REQUIRED_LATERAL_OFFSET_M
    required_longitudinal_progress_m: float = DEFAULT_REQUIRED_LONGITUDINAL_PROGRESS_M
    zone_danger_m: float = 0.042
    zone_warn_m: float = 0.052
    zone_release_m: float = 0.058

    phase: str = "CLOSED"
    encounter_opened_s: float = field(default=None)
    encounter_start_yaw: float = field(default=None)
    origin: tuple = field(default=None)
    previous_yaw: float = field(default=None)
    turn_ledger_used_rad: float = 0.0
    turn_sign: float = -1.0
    tracking_side: str = "LEFT"
    last_raw_mode: str = ""
    last_active_s: float = field(default=None)
    front_seen: bool = False
    mid_seen: bool = False
    rear_seen: bool = False
    pass_confirm_since_s: float = field(default=None)

    def hysteresis_hint(self) -> str:
        if self.phase != "CLOSED":
            return self.last_raw_mode
        return ""

    # -- public entry point --------------------------------------------

    def apply(
        self,
        decision: LocalObstacleDecision,
        zones: ZoneSnapshot,
        now_s: float,
        own_x: float,
        own_y: float,
        own_yaw_rad: float,
    ) -> LocalObstacleDecision:
        if decision.safety_stop:
            return decision

        if self.phase == "FAILSAFE":
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
            self._open_encounter(now_s, own_x, own_y, own_yaw_rad, decision)

        self._update_ledger(own_yaw_rad)
        if self._check_failsafe_ceilings(now_s, own_x, own_y):
            return LocalObstacleDecision(
                True, True, "LOCAL_ENCOUNTER_FAILSAFE", 0.0, 0.0
            )

        if any_raw_active:
            self.last_raw_mode = decision.mode
            self.last_active_s = now_s
            if abs(decision.angular_rps) > 1e-9:
                self.turn_sign = 1.0 if decision.angular_rps > 0.0 else -1.0
                self.tracking_side = "RIGHT" if self.turn_sign > 0.0 else "LEFT"
        self._update_seen_flags(zones)

        if front_active:
            if self.phase in ("PASS_CONFIRM", "RECOVERY_ALLOWED"):
                self.phase = "SIDE_TRACK"
            else:
                self.phase = "DETECT_TURN"
            # DETECT_TURN's defining rule: linear is always forced to 0 here,
            # even for LOCAL_FRONT_WARN (which v1-v3 let creep forward at
            # warning_speed_mps) -- never approach at cruise/warn speed.
            return LocalObstacleDecision(True, False, decision.mode, 0.0, decision.angular_rps)

        if self.phase == "DETECT_TURN":
            return self._handle_detect_turn(side_active, is_narrow, decision, zones, now_s, own_x, own_y, own_yaw_rad)
        if self.phase == "SIDE_TRACK":
            return self._handle_side_track(side_active, is_narrow, decision, zones, now_s, own_x, own_y, own_yaw_rad)
        if self.phase == "PASS_CONFIRM":
            return self._handle_pass_confirm(side_active, is_narrow, decision, zones, now_s, own_x, own_y, own_yaw_rad)
        if self.phase == "RECOVERY_ALLOWED":
            return self._handle_recovery_allowed(side_active, is_narrow, now_s)

        raise AssertionError(f"unreachable phase {self.phase!r}")

    # -- lifecycle --------------------------------------------------------

    def _open_encounter(self, now_s, own_x, own_y, own_yaw_rad, decision) -> None:
        self.phase = "DETECT_TURN"
        self.encounter_opened_s = now_s
        self.encounter_start_yaw = float(own_yaw_rad)
        self.origin = (float(own_x), float(own_y))
        self.previous_yaw = float(own_yaw_rad)
        self.turn_ledger_used_rad = 0.0
        self.turn_sign = 1.0 if decision.angular_rps > 0.0 else -1.0
        self.tracking_side = "RIGHT" if self.turn_sign > 0.0 else "LEFT"
        self.last_raw_mode = ""
        self.last_active_s = now_s
        self.front_seen = False
        self.mid_seen = False
        self.rear_seen = False
        self.pass_confirm_since_s = None

    def _close(self) -> None:
        self.phase = "CLOSED"
        self.encounter_opened_s = None
        self.encounter_start_yaw = None
        self.origin = None
        self.previous_yaw = None
        self.turn_ledger_used_rad = 0.0
        self.turn_sign = -1.0
        self.tracking_side = "LEFT"
        self.last_raw_mode = ""
        self.last_active_s = None
        self.front_seen = False
        self.mid_seen = False
        self.rear_seen = False
        self.pass_confirm_since_s = None

    def _update_ledger(self, own_yaw_rad: float) -> None:
        delta = normalize_angle(float(own_yaw_rad) - self.previous_yaw)
        self.turn_ledger_used_rad += abs(delta)
        self.previous_yaw = float(own_yaw_rad)

    def _check_failsafe_ceilings(self, now_s, own_x, own_y) -> bool:
        distance = math.hypot(own_x - self.origin[0], own_y - self.origin[1])
        time_in_encounter = now_s - self.encounter_opened_s
        if (
            self.turn_ledger_used_rad >= self.max_turn_ledger_rad
            or distance >= self.max_bypass_extension_m
            or time_in_encounter >= self.max_encounter_duration_s
        ):
            self.phase = "FAILSAFE"
            return True
        return False

    def _tracking_zones(self, zones: ZoneSnapshot):
        front_name, mid_name, rear_name = _TRACKING_ZONES[self.tracking_side]
        return (
            _distance(getattr(zones, front_name)),
            _distance(getattr(zones, mid_name)),
            _distance(getattr(zones, rear_name)),
        )

    def _update_seen_flags(self, zones: ZoneSnapshot) -> None:
        front_m, mid_m, rear_m = self._tracking_zones(zones)
        if front_m <= self.zone_release_m:
            self.front_seen = True
        if mid_m <= self.zone_release_m:
            self.mid_seen = True
        if rear_m <= self.zone_release_m:
            self.rear_seen = True

    def _tracking_band(self, zones: ZoneSnapshot) -> str:
        front_m, mid_m, rear_m = self._tracking_zones(zones)
        closest = min(front_m, mid_m, rear_m)
        if closest <= self.zone_danger_m:
            return "DANGER"
        if closest <= self.zone_warn_m:
            return "WARN"
        return "CLEAR"

    def _tracking_zones_all_clear(self, zones: ZoneSnapshot) -> bool:
        front_m, mid_m, rear_m = self._tracking_zones(zones)
        return front_m > self.zone_release_m and mid_m > self.zone_release_m and rear_m > self.zone_release_m

    def _encounter_local_offset(self, own_x, own_y):
        """Project (own_x, own_y) - origin into the encounter's own start-yaw
        frame: (longitudinal, lateral). Handles any initial heading via a
        plain rotation -- no +/-pi special-casing needed since this is a
        linear coordinate transform, not an angle comparison."""
        dx = own_x - self.origin[0]
        dy = own_y - self.origin[1]
        cos_a = math.cos(-self.encounter_start_yaw)
        sin_a = math.sin(-self.encounter_start_yaw)
        longitudinal = dx * cos_a - dy * sin_a
        lateral = dx * sin_a + dy * cos_a
        return longitudinal, lateral

    def _pass_confirm_conditions_met(self, zones, now_s, own_x, own_y) -> bool:
        if not self.rear_seen:
            return False
        if not self._tracking_zones_all_clear(zones):
            return False
        longitudinal, lateral = self._encounter_local_offset(own_x, own_y)
        if abs(lateral) < self.required_lateral_offset_m:
            return False
        if longitudinal < self.required_longitudinal_progress_m:
            return False
        return True

    # -- DETECT_TURN --------------------------------------------------

    def _handle_detect_turn(self, side_active, is_narrow, decision, zones, now_s, own_x, own_y, own_yaw_rad):
        if side_active or is_narrow:
            self.phase = "SIDE_TRACK"
            return self._handle_side_track(side_active, is_narrow, decision, zones, now_s, own_x, own_y, own_yaw_rad)
        if self.turn_ledger_used_rad >= self.max_inplace_turn_rad:
            # Safety valve: raw front cleared without ever reporting a side
            # trigger (sparse polling / a narrow obstacle grazed past) --
            # move to SIDE_TRACK's HOLD rather than spin in place forever.
            self.phase = "SIDE_TRACK"
            return LocalObstacleDecision(True, False, "LOCAL_SIDE_TRACK_HOLD", 0.0, 0.0)
        # Neither front nor side/narrow raw active this tick, ledger not yet
        # capped: keep turning in place toward the committed avoidance
        # direction using the locked candidate rate below (front's own
        # danger/warning turn rate is no longer available once front itself
        # has cleared).
        return LocalObstacleDecision(
            True, False, "LOCAL_DETECT_TURN", 0.0, 0.45 * self.turn_sign
        )

    # -- SIDE_TRACK --------------------------------------------------

    def _handle_side_track(self, side_active, is_narrow, decision, zones, now_s, own_x, own_y, own_yaw_rad):
        if side_active or is_narrow:
            band = self._tracking_band(zones)
            if band == "DANGER":
                linear = 0.0
            elif band == "WARN":
                linear = self.side_track_warn_mps
            else:
                linear = self.side_track_creep_mps
            return LocalObstacleDecision(True, False, "LOCAL_SIDE_TRACK", linear, decision.angular_rps)
        if self._pass_confirm_conditions_met(zones, now_s, own_x, own_y):
            self.phase = "PASS_CONFIRM"
            self.pass_confirm_since_s = now_s
            return LocalObstacleDecision(True, False, "LOCAL_PASS_CONFIRM", self.side_track_creep_mps, 0.0)
        # Raw (aggregate) quiet but the joint PASS_CONFIRM conditions aren't
        # met yet: this must still make forward progress -- freezing here
        # forever (pilot_v4_a's actual failure mode: raw quiet, conditions
        # never met, robot stalls at (0,0) until the duration ceiling forces
        # FAILSAFE) is exactly the "prolonged edge-hugging stall" the
        # acceptance checklist forbids. Still gated on real sensor evidence,
        # though: the TRACKING ZONE band (not the raw aggregate, which can
        # go quiet before the zone triple genuinely clears) decides whether
        # a straight-ahead creep is safe -- DANGER still holds.
        band = self._tracking_band(zones)
        if band == "DANGER":
            return LocalObstacleDecision(True, False, "LOCAL_SIDE_TRACK_HOLD", 0.0, 0.0)
        linear = self.side_track_warn_mps if band == "WARN" else self.side_track_creep_mps
        return LocalObstacleDecision(True, False, "LOCAL_SIDE_TRACK_CREEP", linear, 0.0)

    # -- PASS_CONFIRM --------------------------------------------------

    def _handle_pass_confirm(self, side_active, is_narrow, decision, zones, now_s, own_x, own_y, own_yaw_rad):
        if side_active or is_narrow:
            self.phase = "SIDE_TRACK"
            return self._handle_side_track(side_active, is_narrow, decision, zones, now_s, own_x, own_y, own_yaw_rad)
        if not self._pass_confirm_conditions_met(zones, now_s, own_x, own_y):
            self.phase = "SIDE_TRACK"
            return self._handle_side_track(side_active, is_narrow, decision, zones, now_s, own_x, own_y, own_yaw_rad)
        if now_s - self.pass_confirm_since_s >= self.pass_confirm_hold_s:
            self.phase = "RECOVERY_ALLOWED"
            return LocalObstacleDecision(True, False, "LOCAL_RECOVERY_READY", 0.0, 0.0)
        return LocalObstacleDecision(True, False, "LOCAL_PASS_CONFIRM", self.side_track_creep_mps, 0.0)

    # -- RECOVERY_ALLOWED --------------------------------------------------

    def _handle_recovery_allowed(self, side_active, is_narrow, now_s):
        if side_active or is_narrow:
            # Genuine re-trigger during recovery: fall back to SIDE_TRACK
            # (never DETECT_TURN, never a fresh ledger) and re-require a full
            # PASS_CONFIRM before trying again.
            self.phase = "SIDE_TRACK"
            self.front_seen = self.mid_seen = self.rear_seen = False
            return LocalObstacleDecision(True, False, "LOCAL_SIDE_TRACK_HOLD", 0.0, 0.0)
        if now_s - self.last_active_s >= self.rearm_quiet_s:
            self._close()
        return LocalObstacleDecision(False, False, "LOCAL_CLEAR", 0.0, 0.0)

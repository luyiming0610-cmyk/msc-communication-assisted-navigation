import math
import time

import rclpy
from epuck2_comm_interfaces.msg import EpuckState
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .command_smoothing import CommandSmoother
from .collision_math import (
    closest_point_of_approach,
    collision_risk,
    normalize_angle,
    right_turn_target_reached,
    velocity_vector,
)
from .local_obstacle_logic import EncounterAvoidanceV4, ZoneSnapshot, decide_local_obstacle


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class CooperativeAvoider(Node):
    """Decentralized communication-assisted reciprocal collision avoider."""

    def __init__(self):
        super().__init__("cooperative_avoider")
        self.declare_parameter("robot_id", 1)
        self.declare_parameter("peer_state_topic", "/epuck2/state")
        self.declare_parameter("armed", False)
        self.declare_parameter("desired_heading_rad", 0.0)
        self.declare_parameter("nominal_speed_mps", 0.025)
        self.declare_parameter("avoidance_speed_mps", 0.012)
        self.declare_parameter("turn_rate_rps", 0.65)
        self.declare_parameter("pass_angle_rad", 0.45)
        self.declare_parameter("trigger_distance_m", 0.34)
        self.declare_parameter("release_distance_m", 0.24)
        self.declare_parameter("safety_radius_m", 0.14)
        self.declare_parameter("cpa_horizon_s", 4.0)
        self.declare_parameter("peer_timeout_s", 0.5)
        self.declare_parameter("startup_hold_s", 5.0)
        self.declare_parameter("max_runtime_s", 22.0)
        self.declare_parameter("stop_after_recovery", False)
        self.declare_parameter("post_recovery_hold_s", 0.5)
        self.declare_parameter("enable_peer_avoidance", True)
        self.declare_parameter("enable_local_avoidance", True)
        self.declare_parameter("require_local_sensors", True)
        self.declare_parameter("local_front_danger_m", 0.100)
        self.declare_parameter("local_front_warn_m", 0.180)
        self.declare_parameter("local_front_release_m", 0.220)
        self.declare_parameter("local_side_danger_m", 0.042)
        self.declare_parameter("local_side_warn_m", 0.052)
        self.declare_parameter("local_side_release_m", 0.058)
        self.declare_parameter("local_clear_hold_s", 1.0)
        self.declare_parameter("local_bypass_distance_m", 0.08)
        self.declare_parameter("local_recovery_turn_rps", 0.18)
        # controller_v4_full_sensor_bypass_20260717: full front->mid->rear
        # sensor-sequence + encounter-local lateral-displacement bypass.
        # *** v4 pilot candidates, NOT validated constants. *** Only a real
        # Webots pilot may certify a safe geometric margin -- see
        # local_obstacle_logic.py's module docstring for the pilot_a3
        # forensic finding that motivated this redesign.
        self.declare_parameter("local_v4_max_inplace_turn_rad", 0.90)
        self.declare_parameter("local_v4_max_turn_ledger_rad", 1.40)
        self.declare_parameter("local_v4_max_bypass_extension_m", 0.40)
        self.declare_parameter("local_v4_max_encounter_duration_s", 25.0)
        self.declare_parameter("local_v4_pass_confirm_hold_s", 1.0)
        self.declare_parameter("local_v4_rearm_quiet_s", 1.5)
        self.declare_parameter("local_v4_side_track_creep_mps", 0.010)
        self.declare_parameter("local_v4_side_track_warn_mps", 0.005)
        self.declare_parameter("local_v4_required_lateral_offset_m", 0.070)
        self.declare_parameter("local_v4_required_longitudinal_progress_m", 0.10)
        self.declare_parameter("local_v4_required_lateral_offset_no_evidence_m", 0.10)
        self.declare_parameter("local_v4_pass_confirm_hold_no_evidence_s", 2.0)
        self.declare_parameter("rearm_distance_m", 0.45)
        self.declare_parameter("max_linear_accel_mps2", 0.05)
        self.declare_parameter("max_linear_decel_mps2", 0.10)
        self.declare_parameter("max_angular_accel_rps2", 3.0)
        self.declare_parameter("max_angular_decel_rps2", 4.0)

        self.robot_id = int(self.get_parameter("robot_id").value)
        self.peer_topic = str(self.get_parameter("peer_state_topic").value)
        self.armed = bool(self.get_parameter("armed").value)
        self.desired_heading = float(self.get_parameter("desired_heading_rad").value)
        self.nominal_speed = float(self.get_parameter("nominal_speed_mps").value)
        self.avoidance_speed = float(self.get_parameter("avoidance_speed_mps").value)
        self.turn_rate = float(self.get_parameter("turn_rate_rps").value)
        self.pass_angle = float(self.get_parameter("pass_angle_rad").value)
        self.trigger_distance = float(self.get_parameter("trigger_distance_m").value)
        self.release_distance = float(self.get_parameter("release_distance_m").value)
        self.safety_radius = float(self.get_parameter("safety_radius_m").value)
        self.cpa_horizon = float(self.get_parameter("cpa_horizon_s").value)
        self.peer_timeout = float(self.get_parameter("peer_timeout_s").value)
        self.startup_hold = float(self.get_parameter("startup_hold_s").value)
        self.max_runtime = float(self.get_parameter("max_runtime_s").value)
        self.stop_after_recovery = bool(
            self.get_parameter("stop_after_recovery").value
        )
        self.post_recovery_hold = max(
            0.0, float(self.get_parameter("post_recovery_hold_s").value)
        )
        self.enable_peer_avoidance = bool(
            self.get_parameter("enable_peer_avoidance").value
        )
        self.enable_local_avoidance = bool(
            self.get_parameter("enable_local_avoidance").value
        )
        self.require_local_sensors = bool(
            self.get_parameter("require_local_sensors").value
        )
        self.local_front_danger = float(
            self.get_parameter("local_front_danger_m").value
        )
        self.local_front_warn = float(
            self.get_parameter("local_front_warn_m").value
        )
        self.local_front_release = float(
            self.get_parameter("local_front_release_m").value
        )
        self.local_side_danger = float(
            self.get_parameter("local_side_danger_m").value
        )
        self.local_side_warn = float(
            self.get_parameter("local_side_warn_m").value
        )
        self.local_side_release = float(
            self.get_parameter("local_side_release_m").value
        )
        self.local_clear_hold = float(
            self.get_parameter("local_clear_hold_s").value
        )
        self.local_bypass_distance = float(
            self.get_parameter("local_bypass_distance_m").value
        )
        self.local_recovery_turn = float(
            self.get_parameter("local_recovery_turn_rps").value
        )
        # controller_v4_full_sensor_bypass_20260717: v4 pilot candidates,
        # NOT validated constants.
        self.local_v4_max_inplace_turn = float(
            self.get_parameter("local_v4_max_inplace_turn_rad").value
        )
        self.local_v4_max_turn_ledger = float(
            self.get_parameter("local_v4_max_turn_ledger_rad").value
        )
        self.local_v4_max_bypass_extension = float(
            self.get_parameter("local_v4_max_bypass_extension_m").value
        )
        self.local_v4_max_encounter_duration = float(
            self.get_parameter("local_v4_max_encounter_duration_s").value
        )
        self.local_v4_pass_confirm_hold = float(
            self.get_parameter("local_v4_pass_confirm_hold_s").value
        )
        self.local_v4_rearm_quiet = float(
            self.get_parameter("local_v4_rearm_quiet_s").value
        )
        self.local_v4_side_track_creep = float(
            self.get_parameter("local_v4_side_track_creep_mps").value
        )
        self.local_v4_side_track_warn = float(
            self.get_parameter("local_v4_side_track_warn_mps").value
        )
        self.local_v4_required_lateral_offset = float(
            self.get_parameter("local_v4_required_lateral_offset_m").value
        )
        self.local_v4_required_longitudinal_progress = float(
            self.get_parameter("local_v4_required_longitudinal_progress_m").value
        )
        self.local_v4_required_lateral_offset_no_evidence = float(
            self.get_parameter("local_v4_required_lateral_offset_no_evidence_m").value
        )
        self.local_v4_pass_confirm_hold_no_evidence = float(
            self.get_parameter("local_v4_pass_confirm_hold_no_evidence_s").value
        )
        self.rearm_distance = float(self.get_parameter("rearm_distance_m").value)
        self.max_linear_accel = float(
            self.get_parameter("max_linear_accel_mps2").value
        )
        self.max_linear_decel = float(
            self.get_parameter("max_linear_decel_mps2").value
        )
        self.max_angular_accel = float(
            self.get_parameter("max_angular_accel_rps2").value
        )
        self.max_angular_decel = float(
            self.get_parameter("max_angular_decel_rps2").value
        )

        self.own_state = None
        self.peer_state = None
        self.own_received = None
        self.peer_received = None
        self.started_at = time.monotonic()
        self.mode = "WAITING"
        self.finished = False
        self.encounter_complete = False
        self.complete_logged = False
        self.previous_pass_error = None
        self.recovery_completed_at = None
        self.recovery_source = None
        self.last_log = 0.0
        self._last_logged_drift_events = 0
        self.last_publish_time = time.monotonic()
        self.local_latch = EncounterAvoidanceV4(
            clearance_speed_mps=min(self.avoidance_speed, 0.006),
            max_inplace_turn_rad=self.local_v4_max_inplace_turn,
            max_turn_ledger_rad=self.local_v4_max_turn_ledger,
            max_bypass_extension_m=self.local_v4_max_bypass_extension,
            max_encounter_duration_s=self.local_v4_max_encounter_duration,
            pass_confirm_hold_s=self.local_v4_pass_confirm_hold,
            rearm_quiet_s=self.local_v4_rearm_quiet,
            side_track_creep_mps=self.local_v4_side_track_creep,
            side_track_warn_mps=self.local_v4_side_track_warn,
            required_lateral_offset_m=self.local_v4_required_lateral_offset,
            required_longitudinal_progress_m=self.local_v4_required_longitudinal_progress,
            required_lateral_offset_no_evidence_m=self.local_v4_required_lateral_offset_no_evidence,
            pass_confirm_hold_no_evidence_s=self.local_v4_pass_confirm_hold_no_evidence,
            zone_danger_m=self.local_side_danger,
            zone_warn_m=self.local_side_warn,
            zone_release_m=self.local_side_release,
        )
        self.smoother = CommandSmoother(
            max_linear_accel_mps2=self.max_linear_accel,
            max_linear_decel_mps2=self.max_linear_decel,
            max_angular_accel_rps2=self.max_angular_accel,
            max_angular_decel_rps2=self.max_angular_decel,
        )

        self.command_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(EpuckState, "state", self._own_callback, 20)
        if self.enable_peer_avoidance:
            self.create_subscription(
                EpuckState, self.peer_topic, self._peer_callback, 20
            )
        self.timer = self.create_timer(0.05, self._control)

        self.get_logger().info(
            f"robot={self.robot_id} peer={self.peer_topic} armed={self.armed} "
            f"heading={self.desired_heading:.3f}rad"
        )
        if self.enable_peer_avoidance:
            self.get_logger().info(
                "priority: stale/invalid safe stop > calibrated IR/ToF "
                "avoidance > peer-state CPA avoidance > cruise."
            )
        else:
            self.get_logger().info(
                "ablation: peer-state CPA avoidance disabled; control input is "
                "own state plus calibrated IR/ToF only."
            )
        self.get_logger().info(
            f"local avoidance enabled={self.enable_local_avoidance}, "
            f"require sensors={self.require_local_sensors}, "
            f"front danger/warn/release={self.local_front_danger:.3f}/"
            f"{self.local_front_warn:.3f}/{self.local_front_release:.3f}m"
        )
        self.get_logger().info(
            f"local dropout hold={self.local_clear_hold:.2f}s, "
            f"bypass distance={self.local_bypass_distance:.3f}m, "
            f"recovery turn limit={self.local_recovery_turn:.3f}rad/s"
        )
        self.get_logger().info(
            "controller_v4_full_sensor_bypass_20260717: full-sensor bypass "
            "bounds (v4 pilot candidates, NOT validated constants) "
            f"max_inplace_turn={self.local_v4_max_inplace_turn:.3f}rad, "
            f"max_turn_ledger={self.local_v4_max_turn_ledger:.3f}rad, "
            f"max_bypass_extension={self.local_v4_max_bypass_extension:.3f}m, "
            f"max_encounter_duration={self.local_v4_max_encounter_duration:.1f}s, "
            f"pass_confirm_hold={self.local_v4_pass_confirm_hold:.2f}s, "
            f"rearm_quiet={self.local_v4_rearm_quiet:.2f}s, "
            f"side_track_creep={self.local_v4_side_track_creep:.3f}m/s, "
            f"required_lateral_offset={self.local_v4_required_lateral_offset:.3f}m, "
            f"required_longitudinal_progress={self.local_v4_required_longitudinal_progress:.3f}m"
        )
        self.get_logger().info(
            "command smoothing: linear accel/decel="
            f"{self.max_linear_accel:.3f}/{self.max_linear_decel:.3f}m/s2, "
            "angular accel/decel="
            f"{self.max_angular_accel:.3f}/{self.max_angular_decel:.3f}rad/s2"
        )
        self.get_logger().info(
            f"stop after recovery={self.stop_after_recovery}, "
            f"post-recovery hold={self.post_recovery_hold:.2f}s"
        )

    def _own_callback(self, message: EpuckState) -> None:
        self.own_state = message
        self.own_received = time.monotonic()

    def _peer_callback(self, message: EpuckState) -> None:
        self.peer_state = message
        self.peer_received = time.monotonic()

    def _publish(
        self,
        linear: float,
        angular: float,
        force_zero: bool = False,
        force_linear_zero: bool = False,
    ):
        now = time.monotonic()
        dt = min(0.20, max(0.0, now - self.last_publish_time))
        self.last_publish_time = now
        target_linear = float(clamp(linear, 0.0, self.nominal_speed))
        target_angular = float(clamp(angular, -self.turn_rate, self.turn_rate))
        command = Twist()
        if self.armed and not self.finished:
            applied_linear, applied_angular = self.smoother.step(
                target_linear,
                target_angular,
                dt,
                force_zero=force_zero,
                force_linear_zero=force_linear_zero,
            )
            command.linear.x = float(applied_linear)
            command.angular.z = float(applied_angular)
        else:
            self.smoother.reset()
        self.command_publisher.publish(command)
        if self.armed:
            return float(command.linear.x), float(command.angular.z)
        return target_linear, target_angular

    def _fresh(self, received_at, now: float) -> bool:
        return received_at is not None and now - received_at <= self.peer_timeout

    def _local_decision(self, now: float):
        if not self.enable_local_avoidance or self.own_state is None:
            return None
        decision = decide_local_obstacle(
            self.own_state.front_distance_m,
            self.own_state.left_distance_m,
            self.own_state.right_distance_m,
            self.own_state.validity_flags,
            previous_mode=self.local_latch.hysteresis_hint(),
            front_danger_m=self.local_front_danger,
            front_warn_m=self.local_front_warn,
            front_release_m=self.local_front_release,
            side_danger_m=self.local_side_danger,
            side_warn_m=self.local_side_warn,
            side_release_m=self.local_side_release,
            warning_speed_mps=min(self.avoidance_speed, 0.010),
            side_speed_mps=self.avoidance_speed,
            danger_turn_rps=self.turn_rate,
        )
        zones = ZoneSnapshot(
            left_front_m=float(self.own_state.left_front_m),
            left_mid_m=float(self.own_state.left_mid_m),
            left_rear_m=float(self.own_state.left_rear_m),
            right_front_m=float(self.own_state.right_front_m),
            right_mid_m=float(self.own_state.right_mid_m),
            right_rear_m=float(self.own_state.right_rear_m),
        )
        return self.local_latch.apply(
            decision, zones, now, self.own_state.x_m, self.own_state.y_m, self.own_state.yaw_rad
        )

    def _local_recover_command(self, heading_error: float, now: float):
        """LOCAL_RECOVER's heading-based turn, shared by two call sites.

        controller_v2_local_latch_20260717: this is the single formula both
        the plain `elif self.mode == "LOCAL_RECOVER":` branch and the new
        `LOCAL_RECOVERY_READY` hand-off use, so there is exactly one copy of
        the local-avoidance recovery math, not two that could drift apart.
        Uses the locked `local_recovery_turn_rps` (0.18 rad/s) — not the
        faster peer-CPA `RECOVER` branch's own, separate formula.
        """
        linear = self.avoidance_speed
        angular = clamp(
            0.8 * heading_error,
            -self.local_recovery_turn,
            self.local_recovery_turn,
        )
        mode = "LOCAL_RECOVER"
        if abs(heading_error) < 0.08:
            mode = "CRUISE"
            self.recovery_source = "local"
            self.recovery_completed_at = now
        return mode, linear, angular

    @staticmethod
    def _state_usable(message) -> bool:
        return (
            message is not None
            and int(message.version) == int(EpuckState.PROTOCOL_VERSION)
            and (int(message.validity_flags) & int(EpuckState.FLAG_ODOM_VALID)) != 0
        )

    def _metrics(self):
        own_vx, own_vy = velocity_vector(
            self.own_state.linear_velocity_mps, self.own_state.yaw_rad
        )
        peer_vx, peer_vy = velocity_vector(
            self.peer_state.linear_velocity_mps, self.peer_state.yaw_rad
        )
        return closest_point_of_approach(
            self.own_state.x_m,
            self.own_state.y_m,
            own_vx,
            own_vy,
            self.peer_state.x_m,
            self.peer_state.y_m,
            peer_vx,
            peer_vy,
            self.cpa_horizon,
        )

    def _risk(self, metrics) -> bool:
        return collision_risk(
            metrics,
            horizon_s=self.cpa_horizon,
            safety_radius_m=self.safety_radius,
            trigger_distance_m=self.trigger_distance,
        )

    def _peer_is_behind(self) -> bool:
        dx = self.peer_state.x_m - self.own_state.x_m
        dy = self.peer_state.y_m - self.own_state.y_m
        bearing = normalize_angle(math.atan2(dy, dx) - self.own_state.yaw_rad)
        return abs(bearing) > math.pi / 2.0

    def _log(
        self,
        now: float,
        metrics,
        linear: float,
        angular: float,
        raw_local_mode: str = "",
    ) -> None:
        if now - self.last_log < 0.5:
            return
        self.last_log = now
        prefix = "ARMED" if self.armed else "DRY_RUN"
        local = (
            float(self.own_state.front_distance_m),
            float(self.own_state.left_distance_m),
            float(self.own_state.right_distance_m),
        )
        if metrics is None:
            peer_metrics = "peer=disabled"
        else:
            peer_metrics = (
                f"distance={metrics.current_distance_m:.3f}m "
                f"tcpa={metrics.time_to_cpa_s:.2f}s "
                f"dcpa={metrics.distance_at_cpa_m:.3f}m "
                f"closing={metrics.closing_speed_mps:.3f}m/s"
            )
        # controller_v2_local_latch_20260717: when a safety_stop collapses
        # self.mode to the shared "SAFE_STOP_LOCAL_SENSORS" label, the raw
        # decision mode (e.g. LOCAL_SENSOR_INVALID vs.
        # LOCAL_SIDE_ENCOUNTER_FAILSAFE) is preserved here so the two remain
        # distinguishable in logs without changing command behaviour.
        raw_mode_suffix = f" raw_local_mode={raw_local_mode}" if raw_local_mode else ""
        # controller_v4_full_sensor_bypass_20260717 pilot_v4_a attempt #3
        # fix: surface the command-gated ledger's drift/instability counter
        # whenever nonzero, so a yaw reading that moved despite a ~0
        # commanded angular (and was therefore NOT trusted into the ledger)
        # is visible in the log, never silently dropped.
        drift_suffix = (
            f" drift_events={self.local_latch.drift_events}"
            if self.local_latch.drift_events
            else ""
        )
        self.get_logger().info(
            f"{prefix} mode={self.mode} {peer_metrics} "
            f"local=({local[0]:.3f},{local[1]:.3f},{local[2]:.3f})m "
            f"cmd=({linear:.3f},{angular:.3f}){raw_mode_suffix}{drift_suffix}"
        )

    def _complete(self, message: str) -> None:
        self.mode = "COMPLETE"
        if not self.complete_logged:
            self.get_logger().info(message)
            self.complete_logged = True
        self.finished = True
        self._publish(0.0, 0.0, force_zero=True)

    def _control(self) -> None:
        now = time.monotonic()
        elapsed = now - self.started_at
        if (
            self.stop_after_recovery
            and self.recovery_completed_at is not None
            and now - self.recovery_completed_at >= self.post_recovery_hold
        ):
            source = self.recovery_source or "cooperative"
            self._complete(
                f"COMPLETE: {source} recovery completed; commanding zero"
            )
            return
        if elapsed >= self.max_runtime:
            self._complete("COMPLETE: maximum runtime reached; commanding zero")
            return

        own_fresh = self._fresh(self.own_received, now)
        peer_fresh = self._fresh(self.peer_received, now)
        if not own_fresh or (self.enable_peer_avoidance and not peer_fresh):
            self.mode = "SAFE_STOP_STALE"
            self._publish(0.0, 0.0, force_zero=True)
            return

        own_usable = self._state_usable(self.own_state)
        peer_usable = self._state_usable(self.peer_state)
        if not own_usable or (self.enable_peer_avoidance and not peer_usable):
            self.mode = "SAFE_STOP_INVALID_ODOM"
            self._publish(0.0, 0.0, force_zero=True)
            return

        metrics = self._metrics() if self.enable_peer_avoidance else None
        if elapsed < self.startup_hold:
            self.mode = "STARTUP_HOLD"
            self._publish(0.0, 0.0, force_zero=True)
            self._log(now, metrics, 0.0, 0.0)
            return

        heading_error = normalize_angle(self.desired_heading - self.own_state.yaw_rad)
        if (
            self.enable_peer_avoidance
            and self.encounter_complete
            and metrics.current_distance_m > self.rearm_distance
        ):
            self.encounter_complete = False
        local_decision = self._local_decision(now)
        if self.local_latch.drift_events > self._last_logged_drift_events:
            # controller_v4_full_sensor_bypass_20260717 pilot_v4_a attempt #3
            # fix: a drift/instability event (yaw moved despite a ~0
            # commanded angular, outside the measured noise band) must
            # never be silently ignored -- log it immediately, bypassing
            # the periodic _log() throttle.
            self.get_logger().warn(
                f"local encounter drift/instability detected: yaw moved "
                f"despite ~0 commanded angular, outside the measured noise "
                f"band (total drift_events={self.local_latch.drift_events})"
            )
            self._last_logged_drift_events = self.local_latch.drift_events
        if (
            local_decision is not None
            and local_decision.safety_stop
            and self.require_local_sensors
        ):
            self.mode = "SAFE_STOP_LOCAL_SENSORS"
            self._publish(0.0, 0.0, force_zero=True)
            self._log(now, metrics, 0.0, 0.0, raw_local_mode=local_decision.mode)
            return
        if (
            local_decision is not None
            and local_decision.active
            and not local_decision.safety_stop
        ):
            if local_decision.mode == "LOCAL_RECOVERY_READY":
                # controller_v2_local_latch_20260717: hand off directly to
                # the local-avoidance recovery turn, reusing the exact same
                # formula LOCAL_RECOVER already uses below.
                self.mode, linear, angular = self._local_recover_command(
                    heading_error, now
                )
            else:
                self.mode = local_decision.mode
                linear, angular = local_decision.linear_mps, local_decision.angular_rps
        elif self.enable_peer_avoidance and self.mode in (
            "AVOID_TURN",
            "AVOID_PASS",
        ):
            pass_heading = normalize_angle(self.desired_heading - self.pass_angle)
            pass_error = normalize_angle(pass_heading - self.own_state.yaw_rad)
            target_reached = right_turn_target_reached(
                self.previous_pass_error,
                pass_error,
                tolerance_rad=0.08,
            )
            if self.mode == "AVOID_TURN" and not target_reached:
                linear, angular = self.avoidance_speed * 0.5, -self.turn_rate
                self.previous_pass_error = pass_error
            else:
                self.mode = "AVOID_PASS"
                self.previous_pass_error = None
                linear = self.avoidance_speed
                angular = clamp(1.8 * pass_error, -0.35, 0.35)
                if (
                    self._peer_is_behind()
                    and metrics.current_distance_m > self.release_distance
                ):
                    self.mode = "RECOVER"
                    self.encounter_complete = True
        elif self.mode == "RECOVER":
            linear = self.avoidance_speed
            angular = clamp(1.2 * heading_error, -0.30, 0.30)
            if abs(heading_error) < 0.08:
                self.mode = "CRUISE"
                self.recovery_source = "cooperative"
                self.recovery_completed_at = now
        elif (
            self.enable_peer_avoidance
            and not self.encounter_complete
            and self._risk(metrics)
        ):
            self.mode = "AVOID_TURN"
            pass_heading = normalize_angle(self.desired_heading - self.pass_angle)
            self.previous_pass_error = normalize_angle(
                pass_heading - self.own_state.yaw_rad
            )
            linear, angular = self.avoidance_speed * 0.5, -self.turn_rate
        elif self.mode == "LOCAL_RECOVER":
            self.mode, linear, angular = self._local_recover_command(
                heading_error, now
            )
        else:
            self.mode = "CRUISE"
            linear = self.nominal_speed
            angular = clamp(1.0 * heading_error, -0.25, 0.25)

        # controller_v4_full_sensor_bypass_20260717: whenever the target
        # linear command is exactly zero, bypass the command smoother's
        # deceleration ramp entirely -- a same-tick safety stop (DETECT_TURN,
        # LOCAL_SIDE_TRACK_HOLD, LOCAL_SIDE_TRACK's own danger-band tick,
        # LOCAL_FRONT_DANGER) must never wait for max_linear_decel_mps2 to
        # bring the wheels down. Generalises the old LOCAL_FRONT_DANGER-only
        # special case, which never covered v4's new zero-linear phases.
        applied_linear, applied_angular = self._publish(
            linear,
            angular,
            force_linear_zero=linear <= 0.0,
        )
        self._log(now, metrics, applied_linear, applied_angular)

    def stop(self) -> None:
        self.finished = True
        if not rclpy.ok():
            return
        command = Twist()
        for _ in range(3):
            try:
                self.command_publisher.publish(command)
            except Exception:
                # During ros2 launch SIGINT handling the shared context may be
                # invalidated between rclpy.ok() and publish(). Motion is also
                # bounded by the driver watchdog, so cleanup must remain quiet.
                break
            time.sleep(0.03)


def main(args=None):
    rclpy.init(args=args)
    node = CooperativeAvoider()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop()
        try:
            node.destroy_node()
        except (KeyboardInterrupt, RuntimeError, ValueError):
            # A launch-level SIGINT may invalidate or interrupt individual ROS
            # entities while rclpy is destroying them. The zero command and
            # driver watchdog already provide the motion-safety guarantee.
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except (KeyboardInterrupt, RuntimeError):
                pass


if __name__ == "__main__":
    main()

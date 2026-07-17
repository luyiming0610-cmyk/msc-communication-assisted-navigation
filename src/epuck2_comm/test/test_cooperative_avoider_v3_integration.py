"""controller_v3_unified_encounter_20260717 integration test.

Drives the real CooperativeAvoider node's _control() through a full
ACTIVE -> CONSTRAINED -> RECOVERY_ALLOWED (uninterrupted) -> CLOSED
sequence and asserts self.local_bypass_origin is never set (the
LOCAL_RECOVERY_READY hand-off must bypass the generic fallback's own
origin-creation path, unchanged from controller_v2), and that the
generic fallback's own "LOCAL_BYPASS" mode name never appears -- only the
v3 "LOCAL_ENCOUNTER_HOLD"/"LOCAL_ENCOUNTER_CREEP" names should.

time.monotonic() is monkeypatched so the test does not depend on
wall-clock timing and runs in well under a second.
"""

import math

import rclpy

from epuck2_comm.cooperative_avoider import CooperativeAvoider
from epuck2_comm_interfaces.msg import EpuckState


VALID_ALL = (
    EpuckState.FLAG_ODOM_VALID | EpuckState.FLAG_IR_VALID | EpuckState.FLAG_TOF_VALID
)


def _state(x, y, yaw, front, left, right):
    msg = EpuckState()
    msg.version = EpuckState.PROTOCOL_VERSION
    msg.validity_flags = VALID_ALL
    msg.x_m = float(x)
    msg.y_m = float(y)
    msg.yaw_rad = float(yaw)
    msg.linear_velocity_mps = 0.0
    msg.front_distance_m = float(front)
    msg.left_distance_m = float(left)
    msg.right_distance_m = float(right)
    return msg


def test_constrained_recovery_never_creates_generic_origin(monkeypatch):
    fake_clock = {"t": 2000.0}

    def fake_monotonic():
        return fake_clock["t"]

    monkeypatch.setattr("epuck2_comm.cooperative_avoider.time.monotonic", fake_monotonic)

    rclpy.init(
        args=[
            "--ros-args",
            "-p", "armed:=true",
            "-p", "enable_peer_avoidance:=false",
            "-p", "startup_hold_s:=0.0",
            "-p", "max_runtime_s:=1000.0",
            "-p", "local_bypass_distance_m:=0.02",
            "-p", "local_encounter_max_turn_rad:=0.02",
            "-p", "local_encounter_clear_confirm_s:=0.05",
            "-p", "local_encounter_rearm_quiet_s:=0.08",
            "-p", "local_clear_hold_s:=0.05",
        ]
    )
    try:
        node = CooperativeAvoider()
        modes = []
        origins_seen = []
        state = {"x": 0.0, "y": 0.0, "yaw": 0.0}

        def tick(front, left, right, dt=0.05):
            fake_clock["t"] += dt
            node._own_callback(
                _state(state["x"], state["y"], state["yaw"], front, left, right)
            )
            node._control()
            modes.append(node.mode)
            origins_seen.append(node.local_bypass_origin)
            # Closed-loop unicycle integration using the actual smoothed
            # command _control() just applied (reads the real applied
            # linear/angular velocity, including acceleration/deceleration
            # ramping -- not an approximation of it).
            applied_linear = node.smoother.linear_mps
            applied_angular = node.smoother.angular_rps
            state["x"] += applied_linear * math.cos(state["yaw"]) * dt
            state["y"] += applied_linear * math.sin(state["yaw"]) * dt
            state["yaw"] += applied_angular * dt

        # Left side stays close (0.05m) until the ledger is exhausted and
        # CONSTRAINED takes over; front and right stay clear throughout.
        for _ in range(60):
            tick(math.inf, 0.05, math.inf)
            if any(m in modes for m in ("LOCAL_ENCOUNTER_HOLD", "LOCAL_ENCOUNTER_CREEP")):
                break

        assert "LOCAL_LEFT_SIDE" in modes
        assert any(m in modes for m in ("LOCAL_ENCOUNTER_HOLD", "LOCAL_ENCOUNTER_CREEP"))

        # Now genuinely clear on all sides: CONSTRAINED may creep, confirm
        # quiet+distance, and hand off to LOCAL_RECOVER.
        for _ in range(120):
            tick(math.inf, math.inf, math.inf)
            if node.mode in ("LOCAL_RECOVER", "CRUISE"):
                break

        assert "LOCAL_RECOVER" in modes or "CRUISE" in modes

        for _ in range(400):
            tick(math.inf, math.inf, math.inf)
            if node.mode == "CRUISE":
                break

        assert node.mode == "CRUISE"
        assert "LOCAL_BYPASS" not in modes, (
            "the generic v1/v2 fallback bypass leg must never run for a "
            "capped encounter's LOCAL_RECOVERY_READY hand-off"
        )
        assert "LOCAL_ENCOUNTER_FAILSAFE" not in modes, (
            "this scenario is tuned to resolve cleanly; a FAILSAFE here "
            "would indicate a bug in the test's own tuning, not a real "
            "safety event"
        )
        assert all(origin is None for origin in origins_seen), (
            "local_bypass_origin must never be set by the "
            "LOCAL_RECOVERY_READY hand-off path"
        )
    finally:
        rclpy.shutdown()

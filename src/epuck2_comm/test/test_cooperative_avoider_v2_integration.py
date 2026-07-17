"""controller_v2_local_latch_20260717 integration test.

This is test item 11 of controller_v2_local_latch_design_20260717.md
section 7: drives the real CooperativeAvoider node's _control() through a
full TURNING -> CAPPED_BYPASS -> RECOVERY_ALLOWED (uninterrupted) -> CLOSED
sequence and asserts self.local_bypass_origin is never set (the whole point
of the LOCAL_RECOVERY_READY hand-off is to bypass the generic fallback's own
origin-creation path), and that the generic fallback's own "LOCAL_BYPASS"
mode name never appears in the mode sequence — only the new
"LOCAL_SIDE_BYPASS" name should.

time.monotonic() is monkeypatched so the test does not depend on wall-clock
timing and runs in well under a second.
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


def test_capped_bypass_recovery_never_creates_generic_origin(monkeypatch):
    fake_clock = {"t": 1000.0}

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
            "-p", "local_side_max_turn_rad:=0.02",
            "-p", "local_side_clear_confirm_s:=0.05",
            "-p", "local_side_rearm_quiet_s:=0.08",
            "-p", "local_clear_hold_s:=0.05",
        ]
    )
    try:
        node = CooperativeAvoider()
        modes = []
        origins_seen = []

        def tick(x, y, yaw, front, left, right, dt=0.05):
            fake_clock["t"] += dt
            node._own_callback(_state(x, y, yaw, front, left, right))
            node._control()
            modes.append(node.mode)
            origins_seen.append(node.local_bypass_origin)

        # Start with a nonzero yaw so the eventual recovery hand-off has a
        # real heading error to correct, not an instant zero-error exit.
        yaw = -0.30
        x = 0.0

        # Trigger and sustain LOCAL_LEFT_SIDE until the (tiny) turn budget is
        # spent and the phase moves into CAPPED_BYPASS.
        for _ in range(40):
            tick(x, 0.0, yaw, math.inf, 0.05, math.inf)
            if "LOCAL_SIDE_BYPASS" in modes:
                break

        assert "LOCAL_LEFT_SIDE" in modes
        assert "LOCAL_SIDE_BYPASS" in modes

        # Drive straight (matching LOCAL_SIDE_BYPASS's own commanded motion)
        # with the side sensor genuinely clear, advancing x past
        # local_bypass_distance_m, until recovery is handed off.
        for _ in range(60):
            x += 0.01
            tick(x, 0.0, yaw, math.inf, math.inf, math.inf)
            if node.mode in ("LOCAL_RECOVER", "CRUISE"):
                break

        assert "LOCAL_RECOVER" in modes or "CRUISE" in modes

        # Let the recovery turn (or immediate CRUISE) settle.
        for _ in range(200):
            tick(x, 0.0, yaw, math.inf, math.inf, math.inf)
            yaw += 0.02 * (1 if yaw < 0 else 0)  # crude convergence toward 0
            if node.mode == "CRUISE":
                break

        assert node.mode == "CRUISE"
        assert "LOCAL_BYPASS" not in modes, (
            "the generic v1 fallback's own bypass leg must never run for a "
            "capped encounter's hand-off"
        )
        assert all(origin is None for origin in origins_seen), (
            "local_bypass_origin must never be set by the LOCAL_RECOVERY_READY "
            "hand-off path"
        )
    finally:
        rclpy.shutdown()

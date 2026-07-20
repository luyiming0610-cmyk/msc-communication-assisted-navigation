"""shared_exit_navigation: tests for the new, default-disabled dynamic
speed input (NavigationIntent.desired_linear_speed_mps). Covers: (1)
enable_dynamic_speed=false (the default, and every prior study's
configuration) behaves byte-for-byte like before -- CRUISE linear speed
always equals nominal_speed_mps regardless of any received
NavigationIntent; (2) when enabled, a fresh valid message's speed is
adopted in CRUISE; (3) a stale/never-received NavigationIntent zeroes
linear velocity -- the critical asymmetry vs. dynamic heading, which
falls back to a default heading rather than stopping; (4) an
ARRIVED_HOLD-shaped message (speed=0.0) drives CRUISE linear to exactly
zero, not nominal_speed_mps; (5) dynamic heading and dynamic speed share
the same NavigationIntent message and the same freshness timestamp.
"""

import math

import rclpy

from epuck2_comm.cooperative_avoider import CooperativeAvoider
from epuck2_comm_interfaces.msg import EpuckState, NavigationIntent


VALID_ALL = (
    EpuckState.FLAG_ODOM_VALID | EpuckState.FLAG_IR_VALID | EpuckState.FLAG_TOF_VALID
)


def _state(x, y, yaw, front=math.inf, left=math.inf, right=math.inf):
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


def _nav_intent(heading_rad, speed_mps, valid=True, sequence=1, phase="GO_TO_EXIT"):
    msg = NavigationIntent()
    msg.protocol_version = NavigationIntent.PROTOCOL_VERSION
    msg.source_robot_id = 1
    msg.sequence = sequence
    msg.desired_heading_rad = float(heading_rad)
    msg.desired_linear_speed_mps = float(speed_mps)
    msg.navigation_phase = phase
    msg.valid = valid
    return msg


def _make_node(fake_clock, extra_args=None):
    args = [
        "--ros-args",
        "-p", "armed:=true",
        "-p", "enable_peer_avoidance:=false",
        "-p", "startup_hold_s:=0.0",
        "-p", "max_runtime_s:=1000.0",
        "-p", "desired_heading_rad:=0.0",
        "-p", "nominal_speed_mps:=0.04",
    ]
    if extra_args:
        args += extra_args
    rclpy.init(args=args)
    return CooperativeAvoider()


def test_disabled_by_default_speed_always_nominal(monkeypatch):
    fake_clock = {"t": 1000.0}
    monkeypatch.setattr(
        "epuck2_comm.cooperative_avoider.CooperativeAvoider._now_s",
        lambda self: fake_clock["t"],
    )
    node = _make_node(fake_clock, extra_args=["-p", "enable_dynamic_heading:=true"])
    try:
        assert node.enable_dynamic_speed is False
        node._nav_intent_callback(_nav_intent(0.0, 0.0))  # would zero speed if enabled
        node._own_callback(_state(0.0, 0.0, 0.0))
        fake_clock["t"] += 0.05
        node._control()
        assert node.mode == "CRUISE"
        assert node.smoother.linear_mps > 0.0  # accelerating toward nominal_speed_mps
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_enabled_fresh_valid_message_adopts_speed(monkeypatch):
    fake_clock = {"t": 2000.0}
    monkeypatch.setattr(
        "epuck2_comm.cooperative_avoider.CooperativeAvoider._now_s",
        lambda self: fake_clock["t"],
    )
    node = _make_node(
        fake_clock,
        extra_args=[
            "-p", "enable_dynamic_heading:=true",
            "-p", "enable_dynamic_speed:=true",
            "-p", "nav_intent_timeout_s:=1.0",
        ],
    )
    try:
        node._nav_intent_callback(_nav_intent(0.0, 0.02))
        node._own_callback(_state(0.0, 0.0, 0.0))
        fake_clock["t"] += 0.05
        node._control()
        assert node.mode == "CRUISE"
        assert node.desired_linear_speed == 0.02
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_stale_intent_zeros_speed_not_nominal_fallback(monkeypatch):
    fake_clock = {"t": 3000.0}
    monkeypatch.setattr(
        "epuck2_comm.cooperative_avoider.CooperativeAvoider._now_s",
        lambda self: fake_clock["t"],
    )
    node = _make_node(
        fake_clock,
        extra_args=[
            "-p", "enable_dynamic_heading:=true",
            "-p", "enable_dynamic_speed:=true",
            "-p", "nav_intent_timeout_s:=0.5",
        ],
    )
    try:
        node._nav_intent_callback(_nav_intent(0.0, 0.03))
        node._own_callback(_state(0.0, 0.0, 0.0))
        fake_clock["t"] += 0.05
        node._control()
        assert node.mode == "CRUISE"
        assert node.smoother.linear_mps > 0.0  # accelerating toward 0.03

        # Let the nav_intent go stale -- speed must zero out (NOT fall back
        # to nominal_speed_mps, unlike the heading fallback).
        fake_clock["t"] += 1.0
        for _ in range(60):
            node._own_callback(_state(0.0, 0.0, 0.0))
            fake_clock["t"] += 0.05
            node._control()
        assert node.mode == "CRUISE"
        assert node.smoother.linear_mps == 0.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_never_received_any_message_speed_is_zero(monkeypatch):
    fake_clock = {"t": 4000.0}
    monkeypatch.setattr(
        "epuck2_comm.cooperative_avoider.CooperativeAvoider._now_s",
        lambda self: fake_clock["t"],
    )
    node = _make_node(
        fake_clock,
        extra_args=["-p", "enable_dynamic_heading:=true", "-p", "enable_dynamic_speed:=true"],
    )
    try:
        node._own_callback(_state(0.0, 0.0, 0.0))
        fake_clock["t"] += 0.05
        node._control()
        assert node.mode == "CRUISE"
        assert node.smoother.linear_mps == 0.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_arrived_hold_shaped_intent_drives_zero_not_nominal(monkeypatch):
    fake_clock = {"t": 5000.0}
    monkeypatch.setattr(
        "epuck2_comm.cooperative_avoider.CooperativeAvoider._now_s",
        lambda self: fake_clock["t"],
    )
    node = _make_node(
        fake_clock,
        extra_args=[
            "-p", "enable_dynamic_heading:=true",
            "-p", "enable_dynamic_speed:=true",
            "-p", "nav_intent_timeout_s:=1.0",
        ],
    )
    try:
        node._nav_intent_callback(_nav_intent(0.3, 0.0, phase="ARRIVED_HOLD"))
        for _ in range(10):
            node._own_callback(_state(0.0, 0.0, 0.0))
            fake_clock["t"] += 0.05
            node._control()
        assert node.mode == "CRUISE"
        assert node.smoother.linear_mps == 0.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_heading_and_speed_share_same_freshness_source(monkeypatch):
    fake_clock = {"t": 6000.0}
    monkeypatch.setattr(
        "epuck2_comm.cooperative_avoider.CooperativeAvoider._now_s",
        lambda self: fake_clock["t"],
    )
    node = _make_node(
        fake_clock,
        extra_args=[
            "-p", "enable_dynamic_heading:=true",
            "-p", "enable_dynamic_speed:=true",
            "-p", "nav_intent_timeout_s:=0.5",
            "-p", "desired_heading_rad:=0.0",
        ],
    )
    try:
        node._nav_intent_callback(_nav_intent(1.5, 0.02))
        received_at = node.nav_intent_received_at
        node._own_callback(_state(0.0, 0.0, 0.0))
        fake_clock["t"] += 0.05
        node._control()
        assert node.desired_heading == 1.5
        assert node.desired_linear_speed == 0.02

        fake_clock["t"] += 1.0  # beyond nav_intent_timeout_s=0.5
        node._own_callback(_state(0.0, 0.0, 0.0))
        node._control()
        # Both fall back together -- heading to its launch default, speed to
        # zero -- because both are gated on the identical received_at.
        assert node.desired_heading == 0.0
        assert node.smoother.linear_mps == 0.0
        assert node.nav_intent_received_at == received_at
    finally:
        node.destroy_node()
        rclpy.shutdown()

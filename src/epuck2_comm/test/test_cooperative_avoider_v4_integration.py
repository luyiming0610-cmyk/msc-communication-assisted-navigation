"""controller_v4_full_sensor_bypass_20260717 integration tests.

Replaces test_cooperative_avoider_v3_integration.py (retired -- recoverable
via `git show d2ef811:src/epuck2_comm/test/test_cooperative_avoider_v3_integration.py`)
since EncounterAvoidanceV4's apply() signature and phase set are materially
different from v3's LocalAvoidanceLatch.

Covers: (1) command smoothing never delays a same-tick safety stop -- a
mid-cruise robot commanded to zero linear must show applied_linear==0.0 on
the very next control tick, not a decel-ramped approach to zero; (2) the
legacy v1/v2 generic "LOCAL_BYPASS" fallback branch is gone from v4 --
running a full encounter through to CRUISE must never show that mode string,
including via the specific ACTIVE-clean-close path that let it leak in
pilot_a3 (encounter #1 there resolved through DETECT_TURN/front-warn only,
never a raw side trigger -- exactly the case this test drives).
"""

import math

import rclpy

from epuck2_comm.cooperative_avoider import CooperativeAvoider
from epuck2_comm_interfaces.msg import EpuckState


VALID_ALL = (
    EpuckState.FLAG_ODOM_VALID | EpuckState.FLAG_IR_VALID | EpuckState.FLAG_TOF_VALID
)


def _state(x, y, yaw, front, left, right, **zones):
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
    msg.left_front_m = float(zones.get("left_front_m", math.inf))
    msg.left_mid_m = float(zones.get("left_mid_m", math.inf))
    msg.left_rear_m = float(zones.get("left_rear_m", math.inf))
    msg.right_front_m = float(zones.get("right_front_m", math.inf))
    msg.right_mid_m = float(zones.get("right_mid_m", math.inf))
    msg.right_rear_m = float(zones.get("right_rear_m", math.inf))
    return msg


def test_command_smoothing_never_delays_a_same_tick_safety_stop(monkeypatch):
    fake_clock = {"t": 3000.0}

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
            "-p", "max_linear_accel_mps2:=0.05",
            "-p", "max_linear_decel_mps2:=0.02",  # deliberately slow decel
        ]
    )
    try:
        node = CooperativeAvoider()
        state = {"x": 0.0, "y": 0.0, "yaw": 0.0}

        def tick(front, left, right, dt=0.05):
            fake_clock["t"] += dt
            node._own_callback(_state(state["x"], state["y"], state["yaw"], front, left, right))
            node._control()
            return node.smoother.linear_mps

        # Drive to full cruise speed first.
        applied = 0.0
        for _ in range(60):
            applied = tick(math.inf, math.inf, math.inf)
        assert applied > 0.0, "sanity: robot should be cruising at nonzero speed"

        # Now trigger FRONT_DANGER on the very next tick: applied linear
        # must be exactly 0.0 immediately, not decel-ramped (which at
        # 0.02 m/s^2 over one 0.05s tick could only shed ~0.001 m/s).
        applied_after = tick(0.05, math.inf, math.inf)
        assert applied_after == 0.0
    finally:
        rclpy.shutdown()


def test_legacy_local_bypass_fallback_never_appears_for_a_clean_front_only_encounter(monkeypatch):
    fake_clock = {"t": 4000.0}

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
            "-p", "local_v4_max_inplace_turn_rad:=6.0",
        ]
    )
    try:
        node = CooperativeAvoider()
        modes = []
        state = {"x": 0.0, "y": 0.0, "yaw": 0.0}

        def tick(front, left, right, dt=0.05):
            fake_clock["t"] += dt
            node._own_callback(_state(state["x"], state["y"], state["yaw"], front, left, right))
            node._control()
            modes.append(node.mode)
            linear = node.smoother.linear_mps
            angular = node.smoother.angular_rps
            state["x"] += linear * math.cos(state["yaw"]) * dt
            state["y"] += linear * math.sin(state["yaw"]) * dt
            state["yaw"] += angular * dt

        # Front-only encounter (pilot_a3 encounter #1's actual shape: never
        # a raw side/narrow trigger) resolving through DETECT_TURN alone --
        # exactly the ACTIVE-clean-close path that leaked into the legacy
        # LOCAL_BYPASS branch pre-v4.
        for _ in range(40):
            tick(0.08, math.inf, math.inf)
        for _ in range(400):
            tick(math.inf, math.inf, math.inf)
            if node.mode == "CRUISE":
                break

        assert "LOCAL_BYPASS" not in modes, (
            "the legacy v1/v2 generic fallback must not exist in v4 at all"
        )
        assert "LOCAL_ENCOUNTER_FAILSAFE" not in modes
    finally:
        rclpy.shutdown()


def test_full_encounter_reaches_pass_confirm_and_recovers_to_cruise(monkeypatch):
    fake_clock = {"t": 5000.0}

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
            "-p", "local_v4_required_lateral_offset_m:=0.05",
            "-p", "local_v4_required_longitudinal_progress_m:=0.03",
            "-p", "local_v4_pass_confirm_hold_s:=0.2",
        ]
    )
    try:
        node = CooperativeAvoider()
        modes = []
        state = {"x": 0.0, "y": 0.0, "yaw": 0.0}

        def tick(dt=0.05, **zones):
            fake_clock["t"] += dt
            node._own_callback(
                _state(state["x"], state["y"], state["yaw"], math.inf, math.inf, math.inf, **zones)
            )
            node._control()
            modes.append(node.mode)
            linear = node.smoother.linear_mps
            angular = node.smoother.angular_rps
            state["x"] += linear * math.cos(state["yaw"]) * dt
            state["y"] += linear * math.sin(state["yaw"]) * dt
            state["yaw"] += angular * dt

        # Raw left-side trigger to open the encounter.
        for _ in range(3):
            node._own_callback(_state(state["x"], state["y"], state["yaw"], math.inf, 0.045, math.inf))
            node._control()
            modes.append(node.mode)
            fake_clock["t"] += 0.05
        assert "LOCAL_LEFT_SIDE" in modes or "LOCAL_SIDE_TRACK" in modes

        # Sweep the left front->mid->rear zones while side stays active.
        for name, val in (("left_front_m", 0.045), ("left_mid_m", 0.045), ("left_rear_m", 0.045)):
            node._own_callback(_state(state["x"], state["y"], state["yaw"], math.inf, 0.045, math.inf, **{name: val}))
            node._control()
            modes.append(node.mode)
            fake_clock["t"] += 0.05

        # Now genuinely clear with enough odometry displacement.
        for _ in range(200):
            state["x"] += 0.006
            state["y"] -= 0.006
            tick()
            if node.mode == "CRUISE" and node.recovery_source == "local":
                break

        assert node.mode == "CRUISE"
        assert node.recovery_source == "local"
        assert "LOCAL_BYPASS" not in modes
    finally:
        rclpy.shutdown()

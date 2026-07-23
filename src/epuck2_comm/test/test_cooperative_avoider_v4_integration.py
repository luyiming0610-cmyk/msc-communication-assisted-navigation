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
VALID_NO_ODOM = EpuckState.FLAG_IR_VALID | EpuckState.FLAG_TOF_VALID  # 0x06, missing FLAG_ODOM_VALID


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

    def fake_now_s(self):
        return fake_clock["t"]

    monkeypatch.setattr("epuck2_comm.cooperative_avoider.CooperativeAvoider._now_s", fake_now_s)

    rclpy.init(
        args=[
            "--ros-args",
            "-r", "__ns:=/pytest_isolated",
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

    def fake_now_s(self):
        return fake_clock["t"]

    monkeypatch.setattr("epuck2_comm.cooperative_avoider.CooperativeAvoider._now_s", fake_now_s)

    rclpy.init(
        args=[
            "--ros-args",
            "-r", "__ns:=/pytest_isolated",
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

    def fake_now_s(self):
        return fake_clock["t"]

    monkeypatch.setattr("epuck2_comm.cooperative_avoider.CooperativeAvoider._now_s", fake_now_s)

    rclpy.init(
        args=[
            "--ros-args",
            "-r", "__ns:=/pytest_isolated",
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


def test_invalid_odometry_sample_never_reaches_the_encounter_ledger(monkeypatch):
    """controller_v4_ros_time_consistency: a single sample with
    validity_flags missing FLAG_ODOM_VALID (0x06, the exact reset-artifact
    signature found mid-run in pilot_v4_a attempt #3) must be caught by
    _state_usable() and never reach _local_decision()/EncounterAvoidanceV4
    at all -- confirmed here by checking the latch's own turn_ledger_used_rad
    is completely unaffected by the glitch's bogus (0,0,0) pose, and that
    self.mode correctly shows SAFE_STOP_INVALID_ODOM for that one tick."""
    fake_clock = {"t": 6000.0}

    def fake_now_s(self):
        return fake_clock["t"]

    monkeypatch.setattr("epuck2_comm.cooperative_avoider.CooperativeAvoider._now_s", fake_now_s)

    rclpy.init(
        args=[
            "--ros-args",
            "-r", "__ns:=/pytest_isolated",
            "-p", "armed:=true",
            "-p", "enable_peer_avoidance:=false",
            "-p", "startup_hold_s:=0.0",
            "-p", "max_runtime_s:=1000.0",
        ]
    )
    try:
        node = CooperativeAvoider()

        def tick(x, y, yaw, front, left, right, validity=VALID_ALL, dt=0.05):
            fake_clock["t"] += dt
            msg = _state(x, y, yaw, front, left, right)
            msg.validity_flags = validity
            node._own_callback(msg)
            node._control()

        # Open a real encounter with genuine turning.
        tick(-0.50, 0.0, 0.0, 0.08, math.inf, math.inf)
        tick(-0.49, 0.0, -0.10, math.inf, 0.045, math.inf)
        assert node.local_latch.phase in ("DETECT_TURN", "SIDE_TRACK")
        ledger_before = node.local_latch.turn_ledger_used_rad

        # The glitch: x=0,y=0,yaw=0, validity=0x06 (odom invalid).
        tick(0.0, 0.0, 0.0, math.inf, math.inf, math.inf, validity=VALID_NO_ODOM)
        assert node.mode == "SAFE_STOP_INVALID_ODOM"
        assert node.local_latch.turn_ledger_used_rad == ledger_before, (
            "the invalid-odom glitch sample must never reach the ledger at all"
        )

        # A subsequent genuinely valid sample must resume normally, with the
        # ledger continuing from where it was (not corrupted by the glitch).
        tick(-0.48, 0.0, -0.15, math.inf, 0.045, math.inf)
        assert node.mode != "SAFE_STOP_INVALID_ODOM"
    finally:
        rclpy.shutdown()


def test_started_at_is_not_captured_while_ros_clock_reads_zero(monkeypatch):
    """controller_v4_timebase_fix_20260717: reproduces the exact pilot_v4_b3
    race -- the ROS clock reads 0.0 for several ticks (as it does before a
    node's clock subscription has received its first /clock sample under
    use_sim_time=true) and then jumps straight to 16.26s (the real
    first-activity ros_time observed in that pilot). started_at must be
    captured at 16.26, never at 0.0 or anything in between."""
    fake_clock = {"t": 0.0}

    def fake_now_s(self):
        return fake_clock["t"]

    monkeypatch.setattr("epuck2_comm.cooperative_avoider.CooperativeAvoider._now_s", fake_now_s)

    rclpy.init(
        args=[
            "--ros-args",
            "-r", "__ns:=/pytest_isolated",
            "-p", "armed:=true",
            "-p", "enable_peer_avoidance:=false",
            "-p", "startup_hold_s:=5.0",
            "-p", "max_runtime_s:=70.0",
        ]
    )
    try:
        node = CooperativeAvoider()
        assert node.started_at is None

        # Clock reads exactly 0.0 for a few ticks: must stay in
        # WAITING_FOR_CLOCK, started_at must stay None.
        for _ in range(3):
            node._control()
            assert node.mode == "WAITING_FOR_CLOCK"
            assert node.started_at is None

        # Clock jumps straight to the real pilot_v4_b3 value.
        fake_clock["t"] = 16.260
        node._control()
        assert node.started_at == 16.260, (
            "started_at must be captured at the first valid (nonzero) "
            "clock sample, not at 0.0 or silently left stale"
        )
        assert node.mode != "WAITING_FOR_CLOCK"
    finally:
        rclpy.shutdown()


def test_max_runtime_is_measured_from_first_valid_clock_sample_not_from_zero(monkeypatch):
    """controller_v4_timebase_fix_20260717: with the same 0.0 -> 16.26s
    jump as pilot_v4_b3, max_runtime_s=70.0 must require a FULL 70s from
    16.26 (i.e. must not complete until ros_time=86.26), not from an
    effective zero baseline (which would complete at ros_time=70.0, exactly
    the bug this fix closes)."""
    fake_clock = {"t": 0.0}

    def fake_now_s(self):
        return fake_clock["t"]

    monkeypatch.setattr("epuck2_comm.cooperative_avoider.CooperativeAvoider._now_s", fake_now_s)

    rclpy.init(
        args=[
            "--ros-args",
            "-r", "__ns:=/pytest_isolated",
            "-p", "armed:=true",
            "-p", "enable_peer_avoidance:=false",
            "-p", "startup_hold_s:=0.0",
            "-p", "max_runtime_s:=70.0",
        ]
    )
    try:
        node = CooperativeAvoider()
        node._control()  # clock still 0.0 -> WAITING_FOR_CLOCK, started_at stays None

        fake_clock["t"] = 16.260
        node._control()
        assert node.started_at == 16.260

        # At the OLD (buggy) ros_time=70.0 -- which is what the bug's
        # elapsed-from-zero accounting would have completed at -- the
        # controller must NOT have completed yet under the fix.
        fake_clock["t"] = 70.0
        node._control()
        assert node.mode != "COMPLETE", (
            "max_runtime_s must be measured from the first valid clock "
            "sample (16.26s), not from an effective zero baseline"
        )

        # Just under the true 70s budget from 16.26 (=86.26): still running.
        fake_clock["t"] = 16.260 + 70.0 - 0.05
        node._control()
        assert node.mode != "COMPLETE"

        # At or past the true budget: completes via max_runtime.
        fake_clock["t"] = 16.260 + 70.0 + 0.05
        node._control()
        assert node.mode == "COMPLETE"
    finally:
        rclpy.shutdown()


def test_startup_hold_measured_from_valid_clock_start(monkeypatch):
    """controller_v4_timebase_fix_20260717: startup_hold_s must also be
    measured relative to the first valid clock sample, not from zero."""
    fake_clock = {"t": 0.0}

    def fake_now_s(self):
        return fake_clock["t"]

    monkeypatch.setattr("epuck2_comm.cooperative_avoider.CooperativeAvoider._now_s", fake_now_s)

    rclpy.init(
        args=[
            "--ros-args",
            "-r", "__ns:=/pytest_isolated",
            "-p", "armed:=true",
            "-p", "enable_peer_avoidance:=false",
            "-p", "startup_hold_s:=5.0",
            "-p", "max_runtime_s:=1000.0",
        ]
    )
    try:
        node = CooperativeAvoider()
        node._control()  # WAITING_FOR_CLOCK

        fake_clock["t"] = 16.260
        node._own_callback(_state(0.0, 0.0, 0.0, math.inf, math.inf, math.inf))
        node._control()
        assert node.started_at == 16.260
        assert node.mode == "STARTUP_HOLD", (
            "startup_hold_s must still gate motion measured from 16.26, "
            "not have already elapsed against a zero baseline"
        )

        # Just under startup_hold_s later: still holding.
        fake_clock["t"] = 16.260 + 5.0 - 0.05
        node._own_callback(_state(0.0, 0.0, 0.0, math.inf, math.inf, math.inf))
        node._control()
        assert node.mode == "STARTUP_HOLD"

        # Past startup_hold_s: normal operation begins.
        fake_clock["t"] = 16.260 + 5.0 + 0.05
        node._own_callback(_state(0.0, 0.0, 0.0, math.inf, math.inf, math.inf))
        node._control()
        assert node.mode != "STARTUP_HOLD"
        assert node.mode != "WAITING_FOR_CLOCK"
    finally:
        rclpy.shutdown()


def test_timebase_resets_safely_on_backward_ros_time_jump(monkeypatch):
    """controller_v4_timebase_fix_20260717: a backward ROS-time jump
    (simulation reset / fresh world reload reusing the same process) must
    re-initialize started_at from the new, lower time rather than silently
    keeping a stale (and now nonsensical, possibly negative) elapsed-time
    baseline."""
    fake_clock = {"t": 0.0}

    def fake_now_s(self):
        return fake_clock["t"]

    monkeypatch.setattr("epuck2_comm.cooperative_avoider.CooperativeAvoider._now_s", fake_now_s)

    rclpy.init(
        args=[
            "--ros-args",
            "-r", "__ns:=/pytest_isolated",
            "-p", "armed:=true",
            "-p", "enable_peer_avoidance:=false",
            "-p", "startup_hold_s:=0.0",
            "-p", "max_runtime_s:=1000.0",
        ]
    )
    try:
        node = CooperativeAvoider()
        fake_clock["t"] = 50.0
        node._control()
        assert node.started_at == 50.0
        assert node.timebase_reset_count == 0

        # Simulation resets: /clock jumps back down to near zero.
        fake_clock["t"] = 1.0
        node._control()
        assert node.started_at == 1.0, "started_at must follow the reset, not stay at the stale 50.0"
        assert node.timebase_reset_count == 1

        # A second, larger backward jump increments the counter again.
        fake_clock["t"] = 0.5
        node._control()
        assert node.started_at == 0.5
        assert node.timebase_reset_count == 2

        # Tiny forward jitter (well within tolerance) must NOT be treated
        # as a backward jump.
        fake_clock["t"] = 0.505
        node._control()
        assert node.timebase_reset_count == 2
    finally:
        rclpy.shutdown()


def test_nonzero_first_clock_sample_behaves_exactly_as_before(monkeypatch):
    """controller_v4_timebase_fix_20260717: on real hardware (or any clock
    source that is never exactly 0.0, e.g. a Unix-epoch system clock),
    started_at must be captured on the very first control tick, exactly as
    it always was -- this fix must not add any delay or behaviour change
    for the non-simulation case."""
    fake_clock = {"t": 1_770_000_000.0}  # a large epoch-like value

    def fake_now_s(self):
        return fake_clock["t"]

    monkeypatch.setattr("epuck2_comm.cooperative_avoider.CooperativeAvoider._now_s", fake_now_s)

    rclpy.init(
        args=[
            "--ros-args",
            "-r", "__ns:=/pytest_isolated",
            "-p", "armed:=true",
            "-p", "enable_peer_avoidance:=false",
            "-p", "startup_hold_s:=0.0",
            "-p", "max_runtime_s:=1000.0",
        ]
    )
    try:
        node = CooperativeAvoider()
        node._control()
        assert node.started_at == 1_770_000_000.0
        assert node.mode != "WAITING_FOR_CLOCK"
    finally:
        rclpy.shutdown()

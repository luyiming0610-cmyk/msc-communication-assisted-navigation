"""ROS integration tests for NetworkImpairmentRelay.

Drives _on_message()/_flush_queue() directly (like the cooperative_avoider
v4 integration tests) with a monkeypatched fake clock, rather than relying
on real timer threads -- deterministic and fast.
"""

import rclpy

from epuck2_comm.network_impairment_relay import NetworkImpairmentRelay
from epuck2_comm_interfaces.msg import EpuckState


def _state(sequence):
    msg = EpuckState()
    msg.version = EpuckState.PROTOCOL_VERSION
    msg.sequence = sequence
    msg.robot_id = 1
    msg.x_m = 0.123
    msg.y_m = -0.456
    return msg


def test_zero_impairment_forwards_immediately_and_unchanged(monkeypatch):
    fake_clock = {"t": 100.0}
    monkeypatch.setattr(
        "epuck2_comm.network_impairment_relay.NetworkImpairmentRelay._now_s",
        lambda self: fake_clock["t"],
    )
    rclpy.init(args=["--ros-args", "-p", "delay_s:=0.0", "-p", "jitter_s:=0.0", "-p", "drop_probability:=0.0"])
    try:
        node = NetworkImpairmentRelay()
        received = []
        node.create_subscription(EpuckState, "state", lambda msg: received.append(msg), 20)

        msg = _state(7)
        node._on_message(msg)

        assert node.received_count == 1
        assert node.forwarded_count == 1
        assert node.dropped_count == 0
        # Zero-impairment must publish synchronously in the callback itself
        # (no queued timer-based delay), matching the "equivalent to a
        # direct connection" requirement.
        assert not hasattr(node, "timer")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_fixed_delay_holds_message_until_release_time(monkeypatch):
    fake_clock = {"t": 100.0}
    monkeypatch.setattr(
        "epuck2_comm.network_impairment_relay.NetworkImpairmentRelay._now_s",
        lambda self: fake_clock["t"],
    )
    rclpy.init(args=["--ros-args", "-p", "delay_s:=0.2", "-p", "jitter_s:=0.0", "-p", "drop_probability:=0.0"])
    try:
        node = NetworkImpairmentRelay()
        node._on_message(_state(1))
        assert node.forwarded_count == 0, "message must not be forwarded before its delay elapses"
        assert len(node._queue) == 1

        fake_clock["t"] = 100.1
        node._flush_queue()
        assert node.forwarded_count == 0, "message must still be held at 0.1s < 0.2s delay"

        fake_clock["t"] = 100.2
        node._flush_queue()
        assert node.forwarded_count == 1, "message must be released once its scheduled time is reached"
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_drop_probability_one_drops_every_message(monkeypatch):
    fake_clock = {"t": 0.0}
    monkeypatch.setattr(
        "epuck2_comm.network_impairment_relay.NetworkImpairmentRelay._now_s",
        lambda self: fake_clock["t"],
    )
    rclpy.init(args=["--ros-args", "-p", "drop_probability:=1.0"])
    try:
        node = NetworkImpairmentRelay()
        for i in range(10):
            node._on_message(_state(i))
        assert node.received_count == 10
        assert node.dropped_count == 10
        assert node.forwarded_count == 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_message_content_is_never_mutated_by_the_relay(monkeypatch):
    fake_clock = {"t": 0.0}
    monkeypatch.setattr(
        "epuck2_comm.network_impairment_relay.NetworkImpairmentRelay._now_s",
        lambda self: fake_clock["t"],
    )
    rclpy.init(args=["--ros-args", "-p", "delay_s:=0.0"])
    try:
        node = NetworkImpairmentRelay()
        published = []
        node.publisher.publish = lambda msg: published.append(msg)

        original = _state(42)
        node._on_message(original)

        assert len(published) == 1
        assert published[0] is original
        assert published[0].sequence == 42
        assert published[0].x_m == 0.123
        assert published[0].y_m == -0.456
    finally:
        node.destroy_node()
        rclpy.shutdown()

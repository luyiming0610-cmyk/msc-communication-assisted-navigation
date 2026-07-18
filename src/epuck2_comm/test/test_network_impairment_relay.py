"""ROS integration tests for NetworkImpairmentRelay.

Drives _on_message()/_flush_queue() directly (like the cooperative_avoider
v4 integration tests) with a monkeypatched fake clock, rather than relying
on real timer threads -- deterministic and fast.
"""

import csv
import json

import rclpy

from epuck2_comm.network_impairment_relay import NetworkImpairmentRelay
from epuck2_comm_interfaces.msg import EpuckState


def _state(sequence, stamp_s=None):
    msg = EpuckState()
    msg.version = EpuckState.PROTOCOL_VERSION
    msg.sequence = sequence
    msg.robot_id = 1
    msg.x_m = 0.123
    msg.y_m = -0.456
    if stamp_s is not None:
        msg.stamp.sec = int(stamp_s)
        msg.stamp.nanosec = int(round((stamp_s - int(stamp_s)) * 1.0e9))
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


def test_relay_csv_records_drop_reason_for_outage_and_bernoulli_separately(tmp_path, monkeypatch):
    fake_clock = {"t": 0.0}
    monkeypatch.setattr(
        "epuck2_comm.network_impairment_relay.NetworkImpairmentRelay._now_s",
        lambda self: fake_clock["t"],
    )
    log_path = str(tmp_path / "relay.csv")
    rclpy.init(args=[
        "--ros-args",
        "-p", "drop_probability:=1.0",
        "-p", "outage_period_s:=15.0",
        "-p", "outage_duration_s:=0.7",
        "-p", "outage_phase_s:=0.0",
        "-p", f"log_path:={log_path}",
    ])
    try:
        node = NetworkImpairmentRelay()
        fake_clock["t"] = 0.3  # inside the outage window (elapsed since node start = 0.3s)
        node._on_message(_state(1))
        fake_clock["t"] = 10.0  # outside the outage window; drop_probability=1.0 still drops
        node._on_message(_state(2))
        assert node.dropped_outage_count == 1
        assert node.dropped_bernoulli_count == 1
        assert node.dropped_count == 2
    finally:
        node.destroy_node()
        rclpy.shutdown()

    with open(log_path, encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    assert len(rows) == 2
    assert rows[0]["drop_reason"] == "outage"
    assert rows[1]["drop_reason"] == "bernoulli"


def test_pending_queue_depth_reflects_undelivered_messages(monkeypatch):
    fake_clock = {"t": 100.0}
    monkeypatch.setattr(
        "epuck2_comm.network_impairment_relay.NetworkImpairmentRelay._now_s",
        lambda self: fake_clock["t"],
    )
    rclpy.init(args=["--ros-args", "-p", "delay_s:=0.5"])
    try:
        node = NetworkImpairmentRelay()
        assert node.pending_queue_depth() == 0
        node._on_message(_state(1))
        node._on_message(_state(2))
        assert node.pending_queue_depth() == 2
        fake_clock["t"] = 100.5
        node._flush_queue()
        assert node.pending_queue_depth() == 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_status_topic_publishes_counts_and_queue_depth(monkeypatch):
    fake_clock = {"t": 100.0}
    monkeypatch.setattr(
        "epuck2_comm.network_impairment_relay.NetworkImpairmentRelay._now_s",
        lambda self: fake_clock["t"],
    )
    rclpy.init(args=["--ros-args", "-p", "delay_s:=0.5"])
    try:
        node = NetworkImpairmentRelay()
        node._on_message(_state(1))
        node._on_message(_state(2))
        published = []
        node.status_publisher.publish = lambda msg: published.append(msg)
        node._publish_status()
        assert len(published) == 1
        payload = json.loads(published[0].data)
        assert payload["received_count"] == 2
        assert payload["forwarded_count"] == 0
        assert payload["pending_queue_depth"] == 2
        assert payload["dropped_bernoulli_count"] == 0
        assert payload["dropped_outage_count"] == 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_default_outage_relay_forwards_identically_to_pre_extension_relay(monkeypatch):
    """End-to-end node-level equivalence check (complements the pure
    decider-level test in test_network_impairment.py): a relay
    constructed with no outage parameters set at all (the ROS parameter
    defaults) must forward/drop/delay exactly as the pre-v1.1 relay
    would for the same delay/jitter/drop config."""
    fake_clock = {"t": 100.0}
    monkeypatch.setattr(
        "epuck2_comm.network_impairment_relay.NetworkImpairmentRelay._now_s",
        lambda self: fake_clock["t"],
    )
    rclpy.init(args=["--ros-args", "-p", "delay_s:=0.2", "-p", "jitter_s:=0.0", "-p", "drop_probability:=0.0"])
    try:
        node = NetworkImpairmentRelay()
        node._on_message(_state(1))
        assert node.forwarded_count == 0
        assert len(node._queue) == 1
        fake_clock["t"] = 100.2
        node._flush_queue()
        assert node.forwarded_count == 1
        assert node.dropped_outage_count == 0
        assert node.dropped_bernoulli_count == 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_relay_csv_records_source_stamp_and_actual_release_time(tmp_path, monkeypatch):
    fake_clock = {"t": 100.0}
    monkeypatch.setattr(
        "epuck2_comm.network_impairment_relay.NetworkImpairmentRelay._now_s",
        lambda self: fake_clock["t"],
    )
    log_path = str(tmp_path / "relay.csv")
    rclpy.init(args=["--ros-args", "-p", "delay_s:=0.2", "-p", f"log_path:={log_path}"])
    try:
        node = NetworkImpairmentRelay()
        node._on_message(_state(1, stamp_s=99.5))
        fake_clock["t"] = 100.2
        node._flush_queue()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    with open(log_path, encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    assert len(rows) == 1
    row = rows[0]
    assert row["received_seq"] == "1"
    assert row["action"] == "forwarded"
    assert abs(float(row["source_stamp_s"]) - 99.5) < 1e-6
    assert abs(float(row["actual_release_time_s"]) - 100.2) < 1e-6

import json
import os

import rclpy
from rclpy.utilities import remove_ros_args

from epuck2_comm.sequence_counter import (
    SequenceCounterNode,
    TopicCounter,
    _arguments,
    atomic_write_json,
)
from epuck2_comm_interfaces.msg import EpuckState


def test_clean_sequential_stream_has_no_gaps_or_duplicates():
    counter = TopicCounter()
    for seq in range(20):
        counter.observe(seq, now_s=100.0 + seq * 0.1)
    summary = counter.summary()
    assert summary["received_count"] == 20
    assert summary["unique_sequence_count"] == 20
    assert summary["sequence_gap_count"] == 0
    assert summary["duplicate_count"] == 0
    assert summary["out_of_order_count"] == 0
    assert summary["first_sequence"] == 0
    assert summary["last_sequence"] == 19
    assert summary["expected_count"] == 20


def test_missing_sequence_is_counted_as_a_gap():
    counter = TopicCounter()
    for seq in (0, 1, 2, 5, 6):
        counter.observe(seq, now_s=0.0)
    summary = counter.summary()
    assert summary["sequence_gap_count"] == 2  # missing 3, 4
    assert summary["expected_count"] == 7  # 0..6 inclusive


def test_duplicate_sequence_is_counted_once_per_repeat():
    counter = TopicCounter()
    for seq in (0, 1, 1, 2):
        counter.observe(seq, now_s=0.0)
    summary = counter.summary()
    assert summary["duplicate_count"] == 1
    assert summary["unique_sequence_count"] == 3


def test_out_of_order_arrival_is_counted():
    counter = TopicCounter()
    for seq in (0, 1, 2, 4, 3, 5):
        counter.observe(seq, now_s=0.0)
    summary = counter.summary()
    assert summary["out_of_order_count"] == 1


def test_arguments_strips_ros_injected_tokens_without_crashing(tmp_path):
    # This is exactly the argv shape launch_ros appends: the node's own
    # arguments, followed by --ros-args -r __ns:=... --params-file ...
    # (the bug this fixes: argparse used to choke on the ROS-specific
    # tail and the process would exit immediately with code 2).
    # remove_ros_args validates --params-file as real YAML, so this uses
    # an actual (empty) params file, matching what launch_ros generates.
    params_file = tmp_path / "launch_params.yaml"
    params_file.write_text("{}\n", encoding="utf-8")
    argv = [
        "sequence_counter",
        "--topics", "state_raw", "state",
        "--output-path", "/tmp/out.json",
        "--ros-args", "-r", "__ns:=/epuck1", "--params-file", str(params_file),
    ]
    args = _arguments(remove_ros_args(argv)[1:])
    assert args.topics == ["state_raw", "state"]
    assert args.output_path == "/tmp/out.json"


def test_atomic_write_json_produces_valid_readable_file(tmp_path):
    output_path = str(tmp_path / "checkpoint.json")
    atomic_write_json(output_path, {"complete": False, "n": 1})
    with open(output_path, encoding="utf-8") as fh:
        data = json.load(fh)
    assert data == {"complete": False, "n": 1}


def test_atomic_write_json_leaves_no_temp_file_behind(tmp_path):
    output_path = str(tmp_path / "checkpoint.json")
    atomic_write_json(output_path, {"complete": True})
    leftovers = [p for p in os.listdir(tmp_path) if p != "checkpoint.json"]
    assert leftovers == []


def test_atomic_write_json_overwrites_cleanly_on_repeated_calls(tmp_path):
    output_path = str(tmp_path / "checkpoint.json")
    atomic_write_json(output_path, {"complete": False, "n": 1})
    atomic_write_json(output_path, {"complete": False, "n": 2})
    atomic_write_json(output_path, {"complete": True, "n": 3})
    with open(output_path, encoding="utf-8") as fh:
        data = json.load(fh)
    assert data == {"complete": True, "n": 3}
    leftovers = [p for p in os.listdir(tmp_path) if p != "checkpoint.json"]
    assert leftovers == []


def test_atomic_write_json_does_not_corrupt_existing_file_on_serialization_failure(tmp_path):
    output_path = str(tmp_path / "checkpoint.json")
    atomic_write_json(output_path, {"complete": False, "n": 1})

    class Unserializable:
        pass

    try:
        atomic_write_json(output_path, {"bad": Unserializable()})
    except TypeError:
        pass

    # The original, last-good checkpoint must still be intact -- a failed
    # write must never leave a half-written or missing destination file.
    with open(output_path, encoding="utf-8") as fh:
        data = json.load(fh)
    assert data == {"complete": False, "n": 1}
    leftovers = [p for p in os.listdir(tmp_path) if p != "checkpoint.json"]
    assert leftovers == []


def test_message_age_is_computed_from_stamp_and_reported_as_mean_median_p95_max():
    counter = TopicCounter()
    # now_s - stamp_s = age; ages here are 0.10, 0.20, 0.30, 0.40, 0.50s.
    for i, age in enumerate((0.10, 0.20, 0.30, 0.40, 0.50)):
        counter.observe(i, now_s=100.0 + i, stamp_s=100.0 + i - age)
    summary = counter.summary()
    assert summary["valid_age_sample_count"] == 5
    assert abs(summary["mean_message_age_s"] - 0.30) < 1e-9
    assert abs(summary["median_message_age_s"] - 0.30) < 1e-9
    assert abs(summary["max_message_age_s"] - 0.50) < 1e-9
    assert summary["negative_age_sample_count"] == 0
    assert summary["anomalous_age_sample_count"] == 0


def test_negative_age_sample_is_excluded_from_stats_and_counted_separately():
    counter = TopicCounter()
    counter.observe(0, now_s=100.0, stamp_s=100.5)  # stamp AFTER receipt -> negative age
    counter.observe(1, now_s=101.0, stamp_s=100.9)  # normal positive age
    summary = counter.summary()
    assert summary["negative_age_sample_count"] == 1
    assert summary["valid_age_sample_count"] == 1
    assert abs(summary["mean_message_age_s"] - 0.1) < 1e-9


def test_implausibly_large_age_is_flagged_anomalous_not_reported_as_real_latency():
    counter = TopicCounter()
    # Simulates the clock-domain-mismatch symptom this session found: an
    # epoch-scale "age" must never be silently averaged into the reported
    # latency statistics.
    counter.observe(0, now_s=1_784_360_987.0, stamp_s=12.5)
    summary = counter.summary()
    assert summary["anomalous_age_sample_count"] == 1
    assert summary["valid_age_sample_count"] == 0
    assert summary["mean_message_age_s"] is None
    assert summary["p95_message_age_s"] is None
    assert summary["max_message_age_s"] is None


def test_no_stamp_provided_leaves_age_fields_as_none_not_zero():
    counter = TopicCounter()
    counter.observe(0, now_s=100.0)  # stamp_s omitted, matches non-EpuckState callers
    summary = counter.summary()
    assert summary["valid_age_sample_count"] == 0
    assert summary["mean_message_age_s"] is None


def test_node_callback_extracts_stamp_and_computes_a_plausible_live_age(monkeypatch, tmp_path):
    output_path = str(tmp_path / "checkpoint.json")
    fake_clock = {"t": 100.2}
    monkeypatch.setattr(
        "epuck2_comm.sequence_counter.SequenceCounterNode._now_s",
        lambda self: fake_clock["t"],
    )
    rclpy.init()
    try:
        node = SequenceCounterNode(["state"], output_path, checkpoint_period_s=1000.0)
        callback = node._make_callback("state")

        msg = EpuckState()
        msg.sequence = 1
        msg.stamp.sec = 100
        msg.stamp.nanosec = 100_000_000  # stamp_s = 100.1, receipt at 100.2 -> age=0.1s
        callback(msg)

        summary = node.counters["state"].summary()
        assert summary["valid_age_sample_count"] == 1
        assert abs(summary["mean_message_age_s"] - 0.1) < 1e-6
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_periodic_checkpoint_writes_complete_false_and_final_write_sets_complete_true(monkeypatch, tmp_path):
    output_path = str(tmp_path / "checkpoint.json")
    fake_clock = {"t": 0.0}
    monkeypatch.setattr(
        "epuck2_comm.sequence_counter.SequenceCounterNode._now_s",
        lambda self: fake_clock["t"],
    )
    rclpy.init()
    try:
        node = SequenceCounterNode(["state"], output_path, checkpoint_period_s=1000.0)

        msg = EpuckState()
        msg.sequence = 5
        node.counters["state"].observe(int(msg.sequence), fake_clock["t"])

        # Simulate a mid-run periodic checkpoint (abnormal-exit safety net).
        node._checkpoint()
        with open(output_path, encoding="utf-8") as fh:
            data = json.load(fh)
        assert data["complete"] is False
        assert data["state"]["received_count"] == 1
        assert data["state"]["first_sequence"] == 5

        # Simulate a clean shutdown.
        node.write_final_summary()
        with open(output_path, encoding="utf-8") as fh:
            data = json.load(fh)
        assert data["complete"] is True
        assert data["state"]["received_count"] == 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

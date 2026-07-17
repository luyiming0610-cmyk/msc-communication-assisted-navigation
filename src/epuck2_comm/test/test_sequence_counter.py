from rclpy.utilities import remove_ros_args

from epuck2_comm.sequence_counter import TopicCounter, _arguments


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

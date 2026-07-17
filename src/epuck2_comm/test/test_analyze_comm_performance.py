"""Read-only communication-performance analyzer tests.

Bypasses the rosbag reader (_read_state_bag) with a synthetic generator so
these tests exercise the actual PDR/gap/duplicate/reset/windowing logic
without needing a real rosbag2 install or recorded data.
"""

from types import SimpleNamespace

from epuck2_comm import analyze_comm_performance as acp


def _stamp_from_seconds(seconds_float):
    """Split a float seconds value into integer sec + nanosec, matching
    builtin_interfaces/Time's actual representation -- a naive
    SimpleNamespace(sec=seconds_float) would silently truncate the
    fractional part when later read as int(message.stamp.sec)."""
    sec = int(seconds_float)
    nanosec = int(round((seconds_float - sec) * 1e9))
    return SimpleNamespace(sec=sec, nanosec=nanosec)


def _msg(sequence, stamp_sec):
    return SimpleNamespace(sequence=sequence, stamp=_stamp_from_seconds(stamp_sec))


def _fake_bag(rows):
    """rows: list of (topic, msg, bag_ns, size_bytes)."""
    def _gen(bag_path):
        yield from rows
    return _gen


def _rows_for_topic(topic, count, start_bag_s=10.0, dt=0.1, size_bytes=64, seq_start=0):
    rows = []
    for i in range(count):
        bag_s = start_bag_s + i * dt
        seq = seq_start + i
        rows.append((topic, _msg(seq, bag_s), int(bag_s * 1e9), size_bytes))
    return rows


def test_clean_stream_has_full_pdr_and_no_gaps(monkeypatch):
    rows = _rows_for_topic("/epuck1/state", 100) + _rows_for_topic("/epuck2/state", 100)
    monkeypatch.setattr(acp, "_read_state_bag", _fake_bag(rows))
    result = acp.analyze("fake", warmup_s=1.0, cooldown_s=1.0)
    topic = result["topics"]["/epuck1/state"]
    session = topic["sessions"][0]
    assert session["packet_delivery_ratio"] == 1.0
    assert session["missing_sequence_count"] == 0
    assert session["duplicate_count"] == 0
    assert session["out_of_order_count"] == 0
    assert session["mean_message_age_s"] == 0.0


def test_missing_sequence_numbers_reduce_pdr(monkeypatch):
    all_rows = _rows_for_topic("/epuck1/state", 100)
    # Drop 10 messages from the middle (simulate loss): keep seq 0-39 and 50-99.
    kept = [r for r in all_rows if r[1].sequence < 40 or r[1].sequence >= 50]
    monkeypatch.setattr(acp, "_read_state_bag", _fake_bag(kept))
    result = acp.analyze("fake", warmup_s=1.0, cooldown_s=1.0)
    session = result["topics"]["/epuck1/state"]["sessions"][0]
    assert session["missing_sequence_count"] == 10
    assert 0.0 < session["packet_delivery_ratio"] < 1.0


def test_duplicate_messages_are_counted_not_treated_as_new(monkeypatch):
    rows = _rows_for_topic("/epuck1/state", 20)
    # Duplicate message at sequence 5 (same seq, arrives again later).
    dup = ("/epuck1/state", _msg(5, 12.55), int(12.55e9), 64)
    rows_with_dup = rows + [dup]
    monkeypatch.setattr(acp, "_read_state_bag", _fake_bag(rows_with_dup))
    result = acp.analyze("fake", warmup_s=0.0, cooldown_s=0.0)
    session = result["topics"]["/epuck1/state"]["sessions"][0]
    assert session["duplicate_count"] == 1
    assert session["unique_sequence_count"] == 20


def test_out_of_order_arrival_is_detected(monkeypatch):
    # bag_ns (arrival order) stays strictly increasing 0..9, but the
    # sequence numbers carried by arrivals 4 and 5 are swapped -- sequence
    # 5 physically arrives (in bag-timestamp order) before sequence 4.
    seqs = [0, 1, 2, 3, 5, 4, 6, 7, 8, 9]
    rows = [
        ("/epuck1/state", _msg(seq, 10.0 + i * 0.1), int((10.0 + i * 0.1) * 1e9), 64)
        for i, seq in enumerate(seqs)
    ]
    monkeypatch.setattr(acp, "_read_state_bag", _fake_bag(rows))
    result = acp.analyze("fake", warmup_s=0.0, cooldown_s=0.0)
    session = result["topics"]["/epuck1/state"]["sessions"][0]
    assert session["out_of_order_count"] >= 1


def test_sequence_reset_starts_a_new_session(monkeypatch):
    first_session = _rows_for_topic("/epuck1/state", 50, seq_start=0)
    # Publisher restarts: sequence drops back to 0 partway through.
    second_session = _rows_for_topic(
        "/epuck1/state", 50, start_bag_s=20.0, seq_start=0
    )
    rows = first_session + second_session
    monkeypatch.setattr(acp, "_read_state_bag", _fake_bag(rows))
    result = acp.analyze("fake", warmup_s=0.0, cooldown_s=0.0)
    topic = result["topics"]["/epuck1/state"]
    assert topic["session_count"] == 2
    # Each session's own PDR must be computed independently, not polluted
    # by the other session's sequence range.
    assert topic["sessions"][0]["packet_delivery_ratio"] == 1.0
    assert topic["sessions"][1]["packet_delivery_ratio"] == 1.0


def test_warmup_and_cooldown_exclude_edge_messages(monkeypatch):
    rows = _rows_for_topic("/epuck1/state", 100, start_bag_s=0.0, dt=0.1)
    # Total span: 0.0s to 9.9s. With warmup=2s, cooldown=2s, only [2.0, 7.9] remains.
    monkeypatch.setattr(acp, "_read_state_bag", _fake_bag(rows))
    result = acp.analyze("fake", warmup_s=2.0, cooldown_s=2.0)
    topic = result["topics"]["/epuck1/state"]
    assert topic["excluded_by_window"] > 0
    assert topic["message_count_in_window"] < topic["message_count_total_in_bag"]


def test_message_age_reflects_stamp_to_bag_time_gap(monkeypatch):
    # stamp is 0.05s earlier than bag record time for every message.
    rows = []
    for i in range(50):
        bag_s = 10.0 + i * 0.1
        stamp_s = bag_s - 0.05
        rows.append(("/epuck1/state", _msg(i, stamp_s), int(bag_s * 1e9), 64))
    monkeypatch.setattr(acp, "_read_state_bag", _fake_bag(rows))
    result = acp.analyze("fake", warmup_s=0.0, cooldown_s=0.0)
    session = result["topics"]["/epuck1/state"]["sessions"][0]
    assert abs(session["mean_message_age_s"] - 0.05) < 1e-9
    assert abs(session["max_message_age_s"] - 0.05) < 1e-9


def test_verify_clock_sync_is_not_implemented_and_must_stay_that_way_for_sim_only_phase():
    import pytest

    with pytest.raises(NotImplementedError):
        acp.verify_clock_sync("host_a", "host_b")

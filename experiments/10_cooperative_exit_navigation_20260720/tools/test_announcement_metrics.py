from announcement_metrics import (
    AnnouncementRecord,
    AnnouncementSequenceStats,
    analyze_announcement_sequence,
    build_off_communication_summary,
    build_on_communication_summary,
    normalize_trial_relative,
    NOT_APPLICABLE,
)


def _rec(seq, prod=0.0, recv=None, valid=True):
    return AnnouncementRecord(
        sequence=seq, production_stamp_s=prod, recv_stamp_s=recv if recv is not None else prod, valid=valid
    )


def test_empty_records_returns_zeroed_stats():
    stats = analyze_announcement_sequence([])
    assert stats.message_count == 0
    assert stats.missing_count == 0
    assert stats.duplicate_count == 0
    assert stats.out_of_order_count == 0
    assert stats.mean_age_s is None
    assert stats.max_age_s is None


def test_clean_in_order_sequence_no_anomalies():
    records = [_rec(i, prod=float(i), recv=float(i) + 0.01) for i in range(1, 6)]
    stats = analyze_announcement_sequence(records)
    assert stats.message_count == 5
    assert stats.missing_count == 0
    assert stats.duplicate_count == 0
    assert stats.out_of_order_count == 0
    assert stats.mean_age_s is not None and abs(stats.mean_age_s - 0.01) < 1e-9


def test_missing_sequence_detected():
    records = [_rec(1), _rec(2), _rec(4), _rec(5)]  # 3 is missing
    stats = analyze_announcement_sequence(records)
    assert stats.missing_count == 1


def test_duplicate_sequence_detected():
    records = [_rec(1), _rec(2), _rec(2), _rec(3)]
    stats = analyze_announcement_sequence(records)
    assert stats.duplicate_count == 1
    assert stats.missing_count == 0


def test_out_of_order_detected():
    records = [_rec(1), _rec(3), _rec(2), _rec(4)]  # 2 arrives after 3
    stats = analyze_announcement_sequence(records)
    assert stats.out_of_order_count == 1


def test_max_age_tracks_worst_latency():
    records = [_rec(1, prod=0.0, recv=0.1), _rec(2, prod=0.0, recv=0.5)]
    stats = analyze_announcement_sequence(records)
    assert stats.max_age_s == 0.5


def test_normalize_trial_relative_subtracts_epoch():
    assert normalize_trial_relative(17.42, 12.54) == 17.42 - 12.54


def test_normalize_trial_relative_never_negative_for_post_epoch_events():
    assert normalize_trial_relative(20.0, 12.54) > 0


def test_off_summary_always_not_applicable_regardless_of_leak_count():
    clean = build_off_communication_summary(off_leak_message_count=0)
    for key in (
        "exit_announcement_tx_time_s", "exit_announcement_rx_time_s",
        "robot_b_search_to_goal_switch_time_s", "message_count",
        "missing_count", "duplicate_count", "out_of_order_count",
        "mean_age_s", "max_age_s",
    ):
        assert clean[key] == NOT_APPLICABLE
    assert clean["off_leak_detected"] is False
    assert clean["off_leak_check_message_count"] == 0


def test_off_summary_flags_leak_but_keeps_not_applicable_fields():
    leaked = build_off_communication_summary(off_leak_message_count=3)
    assert leaked["off_leak_detected"] is True
    assert leaked["off_leak_check_message_count"] == 3
    # Even with a detected leak, the contribution fields themselves are
    # never fabricated -- OFF simply never has valid communication
    # metrics to report, leak or not.
    assert leaked["exit_announcement_tx_time_s"] == NOT_APPLICABLE


def test_off_summary_never_reports_a_fake_zero():
    clean = build_off_communication_summary(off_leak_message_count=0)
    assert clean["message_count"] != 0
    assert clean["message_count"] == NOT_APPLICABLE


def test_on_summary_carries_through_real_values():
    stats = AnnouncementSequenceStats(
        message_count=10, missing_count=0, duplicate_count=1,
        out_of_order_count=0, mean_age_s=0.05, max_age_s=0.12,
    )
    summary = build_on_communication_summary(
        tx_time_s=1.2, rx_time_s=1.25, switch_time_s=1.3, seq_stats=stats
    )
    assert summary["exit_announcement_tx_time_s"] == 1.2
    assert summary["exit_announcement_rx_time_s"] == 1.25
    assert summary["robot_b_search_to_goal_switch_time_s"] == 1.3
    assert summary["message_count"] == 10
    assert summary["duplicate_count"] == 1
    assert summary["max_age_s"] == 0.12

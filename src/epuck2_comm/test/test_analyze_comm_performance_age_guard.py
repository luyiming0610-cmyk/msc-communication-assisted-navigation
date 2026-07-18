"""protocol_v1.1_stamp_semantics: analyze_comm_performance.py must never
report a negative or clock-domain-mismatched "age" as if it were real
latency -- these samples are excluded from the statistics and counted
separately, with latency_domain_mismatch_detected raised when every
sample in a session was excluded this way."""

from epuck2_comm.analyze_comm_performance import _session_metrics


def _row(bag_ns, seq, stamp_ns, size_bytes=64):
    return (bag_ns, seq, stamp_ns, size_bytes)


def test_normal_small_ages_are_reported_without_any_anomaly_flag():
    rows = [
        _row(bag_ns=(100 + i) * 1_000_000_000, seq=i, stamp_ns=(100 + i - 0.05) * 1_000_000_000)
        for i in range(5)
    ]
    metrics = _session_metrics(rows, peer_timeout_s=0.5)
    assert metrics["negative_age_sample_count"] == 0
    assert metrics["anomalous_age_sample_count"] == 0
    assert metrics["latency_domain_mismatch_detected"] is False
    assert metrics["valid_age_sample_count"] == 5
    assert abs(metrics["mean_message_age_s"] - 0.05) < 1e-6


def test_clock_domain_mismatch_all_epoch_scale_ages_flagged_not_averaged():
    # Reproduces exactly what happened in
    # objective5_comm_baseline_zero_impairment_formal_trial01: bag_ns is
    # wall-clock epoch scale, stamp_ns is small sim-time scale.
    rows = [
        _row(bag_ns=1_784_360_987 * 1_000_000_000 + i, seq=i, stamp_ns=int((12.5 + i) * 1_000_000_000))
        for i in range(3)
    ]
    metrics = _session_metrics(rows, peer_timeout_s=0.5)
    assert metrics["latency_domain_mismatch_detected"] is True
    assert metrics["valid_age_sample_count"] == 0
    assert metrics["anomalous_age_sample_count"] == 3
    assert metrics["mean_message_age_s"] is None
    assert metrics["p95_message_age_s"] is None
    assert metrics["max_message_age_s"] is None


def test_negative_age_samples_are_excluded_and_counted():
    rows = [
        _row(bag_ns=100 * 1_000_000_000, seq=0, stamp_ns=101 * 1_000_000_000),  # stamp after bag record -> negative
        _row(bag_ns=101 * 1_000_000_000, seq=1, stamp_ns=int(100.9 * 1_000_000_000)),
    ]
    metrics = _session_metrics(rows, peer_timeout_s=0.5)
    assert metrics["negative_age_sample_count"] == 1
    assert metrics["valid_age_sample_count"] == 1
    assert abs(metrics["mean_message_age_s"] - 0.1) < 1e-6

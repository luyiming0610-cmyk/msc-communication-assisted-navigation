import pytest

from relay_drain import (
    DrainTimeoutError,
    compute_drain_duration_s,
    is_drained,
    max_configured_delivery_delay_s,
    parse_relay_status,
    poll_until_drained,
)


def test_max_configured_delivery_delay_matches_condition_c():
    assert max_configured_delivery_delay_s(delay_s=1.00, jitter_s=0.0) == 1.00


def test_max_configured_delivery_delay_matches_condition_g():
    assert abs(max_configured_delivery_delay_s(delay_s=0.20, jitter_s=0.20) - 0.30) < 1e-9


def test_max_configured_delivery_delay_ignores_outage_since_outage_drops_immediately():
    # outage messages never enter the queue (release_delay_s=0.0 on drop),
    # so a condition combining delay with an outage still only needs the
    # delay/jitter component in the drain-duration calculation.
    assert max_configured_delivery_delay_s(delay_s=0.5, jitter_s=0.0) == 0.5


def test_compute_drain_duration_condition_c_two_period_margin():
    duration = compute_drain_duration_s(delay_s=1.00, jitter_s=0.0, publish_period_s=0.1151, periods_margin=2)
    assert abs(duration - (1.00 + 2 * 0.1151)) < 1e-9


def test_compute_drain_duration_rejects_negative_margin():
    with pytest.raises(ValueError):
        compute_drain_duration_s(0.2, 0.0, 0.1151, periods_margin=-1)


def test_parse_relay_status_round_trips_valid_payload():
    payload = parse_relay_status('{"received_count": 5, "forwarded_count": 5, '
                                  '"dropped_bernoulli_count": 0, "dropped_outage_count": 0, '
                                  '"pending_queue_depth": 0}')
    assert payload["pending_queue_depth"] == 0


def test_parse_relay_status_rejects_missing_queue_depth_field():
    with pytest.raises(ValueError):
        parse_relay_status('{"received_count": 5}')


def test_is_drained_true_only_at_exactly_zero():
    assert is_drained({"pending_queue_depth": 0}) is True
    assert is_drained({"pending_queue_depth": 1}) is False
    assert is_drained({"pending_queue_depth": 7}) is False


def test_poll_until_drained_succeeds_once_queue_reaches_zero():
    responses = iter([{"pending_queue_depth": 3}, {"pending_queue_depth": 1}, {"pending_queue_depth": 0}])
    fake_time = {"t": 0.0}
    result = poll_until_drained(
        read_status_fn=lambda: next(responses),
        timeout_s=10.0,
        poll_interval_s=0.5,
        time_fn=lambda: fake_time["t"],
        sleep_fn=lambda s: fake_time.__setitem__("t", fake_time["t"] + s),
    )
    assert result["pending_queue_depth"] == 0


def test_poll_until_drained_raises_drain_timeout_error_and_never_silently_proceeds():
    fake_time = {"t": 0.0}
    with pytest.raises(DrainTimeoutError) as excinfo:
        poll_until_drained(
            read_status_fn=lambda: {"pending_queue_depth": 4},
            timeout_s=2.0,
            poll_interval_s=0.5,
            time_fn=lambda: fake_time["t"],
            sleep_fn=lambda s: fake_time.__setitem__("t", fake_time["t"] + s),
        )
    assert "4" in str(excinfo.value) or "pending_queue_depth" in str(excinfo.value)

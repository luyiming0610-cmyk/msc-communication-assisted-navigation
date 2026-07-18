import json
import subprocess
import sys
from pathlib import Path

from matrix_analyzer import (
    build_direction_report,
    build_latency_block,
    build_sequence_block,
    classify_latency_measurement_status,
    overall_measurement_validity,
    summarize_relay_csv,
    validate_output_schema,
)

MATRIX_ANALYZER_PATH = Path(__file__).resolve().parent / "matrix_analyzer.py"


def _relay_rows(*, forwarded=0, independent=0, outage=0):
    rows = []
    for _ in range(forwarded):
        rows.append({"action": "forwarded", "drop_reason": ""})
    for _ in range(independent):
        rows.append({"action": "dropped", "drop_reason": "independent"})
    for _ in range(outage):
        rows.append({"action": "dropped", "drop_reason": "outage"})
    return rows


def _counter_topic(*, received=0, expected=None, missing=0, duplicate=0, out_of_order=0,
                    sample_count=0, mean_age=None):
    return {
        "received_count": received,
        "unique_sequence_count": received,
        "expected_count": expected if expected is not None else received,
        "sequence_gap_count": missing,
        "duplicate_count": duplicate,
        "out_of_order_count": out_of_order,
        "valid_age_sample_count": sample_count,
        "mean_message_age_s": mean_age,
        "median_message_age_s": mean_age,
        "p95_message_age_s": mean_age,
        "p99_message_age_s": mean_age,
        "max_message_age_s": mean_age,
        "negative_age_sample_count": 0,
        "anomalous_age_sample_count": 0,
    }


# --- summarize_relay_csv ---

def test_summarize_relay_csv_counts_forward_independent_outage_separately():
    rows = _relay_rows(forwarded=90, independent=7, outage=3)
    result = summarize_relay_csv(rows)
    assert result == {
        "received_count": 100,
        "forwarded_count": 90,
        "dropped_count": 10,
        "independent_drop_count": 7,
        "outage_drop_count": 3,
    }


# --- classify_latency_measurement_status ---

def test_condition_a_same_sim_clock_all_zero_age_is_resolution_limited_not_mismatch():
    """The core requirement: an all-zero observed age under a genuinely
    zero-delay/zero-jitter configured condition must NOT be reported as a
    clock-domain mismatch -- it is a legitimate resolution-limited result."""
    status = classify_latency_measurement_status(
        sample_count=376, mean_age_s=0.0, configured_delay_s=0.0, configured_jitter_s=0.0
    )
    assert status == "RESOLUTION_LIMITED"


def test_genuine_cross_clock_domain_or_negative_age_is_metric_invalid():
    """A large or negative mean age under a zero-configured condition IS
    a real problem and must be flagged, not silently accepted."""
    status = classify_latency_measurement_status(
        sample_count=100, mean_age_s=1.78e9, configured_delay_s=0.0, configured_jitter_s=0.0
    )
    assert status == "METRIC_INVALID"


def test_condition_b_fixed_020s_delay_within_tolerance_is_valid():
    status = classify_latency_measurement_status(
        sample_count=50, mean_age_s=0.205, configured_delay_s=0.20, configured_jitter_s=0.0
    )
    assert status == "VALID_AT_SIM_CLOCK_RESOLUTION"


def test_condition_b_fixed_020s_delay_grossly_mismatched_is_metric_invalid():
    status = classify_latency_measurement_status(
        sample_count=50, mean_age_s=0.90, configured_delay_s=0.20, configured_jitter_s=0.0
    )
    assert status == "METRIC_INVALID"


def test_condition_d_jitter_range_within_band_is_valid():
    # Condition D: delay_s=0.15, jitter_s=0.30 -> realized range [0, 0.30], mean 0.15
    status = classify_latency_measurement_status(
        sample_count=200, mean_age_s=0.15, configured_delay_s=0.15, configured_jitter_s=0.30
    )
    assert status == "VALID_AT_SIM_CLOCK_RESOLUTION"


def test_condition_g_combined_jitter_range_within_band_is_valid():
    # Condition G: delay_s=0.20, jitter_s=0.20 -> realized range [0.10, 0.30], mean 0.20
    status = classify_latency_measurement_status(
        sample_count=200, mean_age_s=0.20, configured_delay_s=0.20, configured_jitter_s=0.20
    )
    assert status == "VALID_AT_SIM_CLOCK_RESOLUTION"


def test_no_samples_is_not_applicable_not_metric_invalid():
    status = classify_latency_measurement_status(
        sample_count=0, mean_age_s=None, configured_delay_s=0.20, configured_jitter_s=0.0
    )
    assert status == "NOT_APPLICABLE_NO_SAMPLES"


# --- E/F: dropped messages vs received-message latency must be separate ---

def test_dropped_messages_never_contribute_latency_samples_and_are_reported_separately():
    """Condition E/F: dropped messages have no receipt event at all, so
    they can never produce an age sample -- sequence_counter.py's
    observe() is only ever called for messages that actually arrived.
    This test pins that the analyzer reports relay-level drop counts and
    consumer-level latency sample counts as two genuinely independent
    numbers, not derived from one another."""
    relay_rows = _relay_rows(forwarded=60, independent=20, outage=20)
    counter_topic = _counter_topic(received=60, sample_count=60, mean_age=0.02)
    report = build_direction_report("epuck1_to_epuck2", relay_rows, counter_topic, 0.0, 0.0)
    assert report["relay"]["dropped_count"] == 40
    assert report["relay"]["independent_drop_count"] == 20
    assert report["relay"]["outage_drop_count"] == 20
    assert report["latency"]["sample_count"] == 60
    assert report["relay"]["forwarded_count"] == report["latency"]["sample_count"]
    assert report["relay_forwarded_matches_consumer_received"] is True


def test_relay_forwarded_not_matching_consumer_received_is_flagged():
    relay_rows = _relay_rows(forwarded=90, independent=10)
    counter_topic = _counter_topic(received=85, sample_count=85, mean_age=0.0)
    report = build_direction_report("epuck1_to_epuck2", relay_rows, counter_topic, 0.0, 0.0)
    assert report["relay_forwarded_matches_consumer_received"] is False


# --- overall_measurement_validity ---

def test_overall_validity_is_valid_when_all_directions_clean():
    directions = [
        build_direction_report("a", _relay_rows(forwarded=10), _counter_topic(received=10, sample_count=10, mean_age=0.0), 0.0, 0.0),
        build_direction_report("b", _relay_rows(forwarded=10), _counter_topic(received=10, sample_count=10, mean_age=0.0), 0.0, 0.0),
    ]
    assert overall_measurement_validity(directions) == "VALID"


def test_overall_validity_is_invalid_if_any_direction_metric_invalid():
    directions = [
        build_direction_report("a", _relay_rows(forwarded=10), _counter_topic(received=10, sample_count=10, mean_age=0.0), 0.0, 0.0),
        build_direction_report("b", _relay_rows(forwarded=10), _counter_topic(received=10, sample_count=10, mean_age=999.0), 0.0, 0.0),
    ]
    assert overall_measurement_validity(directions) == "INVALID"


def test_overall_validity_is_invalid_if_relay_consumer_mismatch():
    directions = [
        build_direction_report("a", _relay_rows(forwarded=10), _counter_topic(received=8, sample_count=8, mean_age=0.0), 0.0, 0.0),
    ]
    assert overall_measurement_validity(directions) == "INVALID"


# --- validate_output_schema ---

def test_validate_output_schema_accepts_complete_payload():
    payload = {
        "condition_id": "A",
        "measurement_validity": "VALID",
        "directions": [
            {"direction": "a", "relay": {}, "sequence": {}, "latency": {}},
        ],
        "queue_drain": {},
    }
    assert validate_output_schema(payload) == []


def test_validate_output_schema_reports_missing_top_level_field():
    payload = {"measurement_validity": "VALID", "directions": [{"direction": "a", "relay": {}, "sequence": {}, "latency": {}}], "queue_drain": {}}
    problems = validate_output_schema(payload)
    assert any("condition_id" in p for p in problems)


def test_validate_output_schema_reports_empty_directions():
    payload = {"condition_id": "A", "measurement_validity": "VALID", "directions": [], "queue_drain": {}}
    problems = validate_output_schema(payload)
    assert any("directions" in p for p in problems)


def test_validate_output_schema_reports_missing_direction_field():
    payload = {
        "condition_id": "A", "measurement_validity": "VALID",
        "directions": [{"direction": "a", "relay": {}, "sequence": {}}],  # missing "latency"
        "queue_drain": {},
    }
    problems = validate_output_schema(payload)
    assert any("latency" in p for p in problems)


# --- CLI: exit codes, missing/corrupt inputs ---

def _run_cli(args, cwd=None):
    return subprocess.run(
        [sys.executable, str(MATRIX_ANALYZER_PATH)] + args,
        capture_output=True, text=True, cwd=cwd,
    )


def test_cli_exits_nonzero_when_diag_log_dir_missing(tmp_path):
    output_path = tmp_path / "out.json"
    result = _run_cli([
        "--native-diag-log-dir", str(tmp_path / "does_not_exist"),
        "--native-bag-dir", str(tmp_path / "bag_does_not_exist"),
        "--frozen-params-json", str(tmp_path / "missing_frozen.json"),
        "--queue-drain-json", "{}",
        "--realtime-factor-json", "{}",
        "--task-outcome-inputs-json", "{}",
        "--output-path", str(output_path),
    ])
    assert result.returncode != 0
    assert output_path.exists()
    written = json.loads(output_path.read_text(encoding="utf-8"))
    assert written["measurement_validity"] == "INVALID"
    assert written["errors"]


def test_cli_exits_nonzero_when_counter_summary_missing(tmp_path):
    diag_dir = tmp_path / "diag"
    diag_dir.mkdir()
    (diag_dir / "epuck1_relay.csv").write_text("received_seq,action,drop_reason\n0,forwarded,\n", encoding="utf-8")
    (diag_dir / "epuck2_relay.csv").write_text("received_seq,action,drop_reason\n0,forwarded,\n", encoding="utf-8")
    # epuck1_counter.json / epuck2_counter.json deliberately not written
    frozen = tmp_path / "frozen_params.json"
    frozen.write_text(json.dumps({"condition_id": "A", "trial_index": 1, "delay_s": 0.0, "jitter_s": 0.0}), encoding="utf-8")
    output_path = tmp_path / "out.json"
    result = _run_cli([
        "--native-diag-log-dir", str(diag_dir),
        "--native-bag-dir", str(tmp_path / "bag_does_not_exist"),
        "--frozen-params-json", str(frozen),
        "--queue-drain-json", "{}",
        "--realtime-factor-json", "{}",
        "--task-outcome-inputs-json", "{}",
        "--output-path", str(output_path),
    ])
    assert result.returncode != 0
    written = json.loads(output_path.read_text(encoding="utf-8"))
    assert written["measurement_validity"] == "INVALID"


def test_cli_exits_nonzero_when_counter_json_is_corrupt(tmp_path):
    diag_dir = tmp_path / "diag"
    diag_dir.mkdir()
    (diag_dir / "epuck1_relay.csv").write_text("received_seq,action,drop_reason\n0,forwarded,\n", encoding="utf-8")
    (diag_dir / "epuck2_relay.csv").write_text("received_seq,action,drop_reason\n0,forwarded,\n", encoding="utf-8")
    (diag_dir / "epuck1_counter.json").write_text("{not valid json", encoding="utf-8")
    (diag_dir / "epuck2_counter.json").write_text("{}", encoding="utf-8")
    frozen = tmp_path / "frozen_params.json"
    frozen.write_text(json.dumps({"condition_id": "A", "trial_index": 1, "delay_s": 0.0, "jitter_s": 0.0}), encoding="utf-8")
    output_path = tmp_path / "out.json"
    result = _run_cli([
        "--native-diag-log-dir", str(diag_dir),
        "--native-bag-dir", str(tmp_path / "bag_does_not_exist"),
        "--frozen-params-json", str(frozen),
        "--queue-drain-json", "{}",
        "--realtime-factor-json", "{}",
        "--task-outcome-inputs-json", "{}",
        "--output-path", str(output_path),
    ])
    assert result.returncode != 0
    written = json.loads(output_path.read_text(encoding="utf-8"))
    assert written["measurement_validity"] == "INVALID"
    assert written["errors"]


def test_cli_exits_nonzero_and_flags_invalid_when_required_fields_missing_from_schema():
    """validate_output_schema itself, exercised directly (the CLI path
    to this is covered by the missing-counter-summary test above, which
    also happens to produce a schema gap) -- this test pins the schema
    check in isolation."""
    incomplete = {"measurement_validity": "VALID", "directions": [{"direction": "a"}], "queue_drain": {}}
    problems = validate_output_schema(incomplete)
    assert len(problems) >= 2  # missing condition_id, and direction missing relay/sequence/latency

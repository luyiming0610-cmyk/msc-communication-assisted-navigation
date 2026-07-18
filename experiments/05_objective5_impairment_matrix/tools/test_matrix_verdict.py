from matrix_verdict import (
    DataValidityCheck,
    TaskOutcomeSignals,
    classify_data_validity,
    classify_task_outcome,
)


def test_data_validity_all_checks_pass():
    verdict, reasons = classify_data_validity(DataValidityCheck(True, True, True, True, True))
    assert verdict == "VALID"
    assert reasons == []


def test_data_validity_reports_every_failing_check_not_just_the_first():
    verdict, reasons = classify_data_validity(DataValidityCheck(False, False, True, True, True))
    assert verdict == "INVALID"
    assert len(reasons) == 2
    assert any("seed" in r for r in reasons)
    assert any("realtime_factor" in r for r in reasons)


def test_data_validity_queue_not_drained_alone_is_invalid():
    verdict, reasons = classify_data_validity(DataValidityCheck(True, True, True, True, False))
    assert verdict == "INVALID"
    assert any("queue-drain" in r for r in reasons)


def test_data_validity_bag_log_not_clean_alone_is_invalid_and_not_mislabeled_as_queue():
    """Regression test for the real Pilot 2 (Condition F) bug: a
    bag_record.log warning must be attributed to its own bag_log_clean
    reason, never to the unrelated queue-drain reason, even though both
    ultimately gate the same overall DATA_VALIDITY verdict."""
    verdict, reasons = classify_data_validity(
        DataValidityCheck(True, True, True, True, True, bag_log_clean=False)
    )
    assert verdict == "INVALID"
    assert any("bag_record.log" in r for r in reasons)
    assert not any("queue-drain" in r for r in reasons)


def test_data_validity_bag_log_clean_defaults_true_for_backward_compatibility():
    verdict, reasons = classify_data_validity(DataValidityCheck(True, True, True, True, True))
    assert verdict == "VALID"
    assert reasons == []


def test_task_outcome_not_evaluable_when_data_invalid():
    signals = TaskOutcomeSignals(
        complete_count=2, expected_complete_count=2, min_interrobot_distance_m=0.5,
        safety_radius_m=0.14, controller_crashed=False,
    )
    outcome, reason = classify_task_outcome("INVALID", signals)
    assert outcome == "NOT_EVALUABLE"


def test_task_outcome_success():
    signals = TaskOutcomeSignals(
        complete_count=2, expected_complete_count=2, min_interrobot_distance_m=0.20,
        safety_radius_m=0.14, controller_crashed=False,
    )
    outcome, _ = classify_task_outcome("VALID", signals)
    assert outcome == "SUCCESS"


def test_task_outcome_safe_degradation_on_incomplete_but_no_collision():
    signals = TaskOutcomeSignals(
        complete_count=1, expected_complete_count=2, min_interrobot_distance_m=0.25,
        safety_radius_m=0.14, controller_crashed=False,
    )
    outcome, reason = classify_task_outcome("VALID", signals)
    assert outcome == "SAFE_DEGRADATION"
    assert "1" in reason


def test_task_outcome_unsafe_failure_on_close_distance_even_if_complete():
    """A trial that technically reached complete_count but had a
    dangerously close approach along the way is still UNSAFE_FAILURE --
    completion does not override a safety-radius violation."""
    signals = TaskOutcomeSignals(
        complete_count=2, expected_complete_count=2, min_interrobot_distance_m=0.05,
        safety_radius_m=0.14, controller_crashed=False,
    )
    outcome, reason = classify_task_outcome("VALID", signals)
    assert outcome == "UNSAFE_FAILURE"
    assert "0.05" in reason


def test_task_outcome_unsafe_failure_on_controller_crash_regardless_of_distance():
    signals = TaskOutcomeSignals(
        complete_count=0, expected_complete_count=2, min_interrobot_distance_m=1.0,
        safety_radius_m=0.14, controller_crashed=True,
    )
    outcome, reason = classify_task_outcome("VALID", signals)
    assert outcome == "UNSAFE_FAILURE"
    assert "crash" in reason


def test_task_outcome_high_impairment_safe_degradation_is_not_excluded_or_penalized():
    """This is the core requirement being tested here at the code level:
    a trial deliberately run under a high-impairment condition (e.g.
    Condition F) that results in an incomplete task via a genuine safety
    stop, with DATA_VALIDITY still VALID, must classify as
    SAFE_DEGRADATION -- not NOT_EVALUABLE, not silently dropped, not
    treated as if the trial itself were invalid."""
    signals = TaskOutcomeSignals(
        complete_count=0, expected_complete_count=2, min_interrobot_distance_m=0.30,
        safety_radius_m=0.14, controller_crashed=False,
    )
    outcome, _ = classify_task_outcome("VALID", signals)
    assert outcome == "SAFE_DEGRADATION"


def test_cli_reads_json_stdin_and_writes_combined_verdict_to_stdout():
    import json
    import subprocess
    import sys
    from pathlib import Path

    payload = {
        "seeds_match_frozen_config": True,
        "realtime_factor_ok": True,
        "bag_metadata_ok": True,
        "analyzer_ok": True,
        "queue_drained": True,
        "complete_count": 2,
        "expected_complete_count": 2,
        "min_interrobot_distance_m": 0.20,
        "safety_radius_m": 0.14,
        "controller_crashed": False,
    }
    result = subprocess.run(
        [sys.executable, str(Path(__file__).resolve().parent / "matrix_verdict.py")],
        input=json.dumps(payload), capture_output=True, text=True,
    )
    assert result.returncode == 0
    output = json.loads(result.stdout)
    assert output["data_validity"] == "VALID"
    assert output["task_outcome"] == "SUCCESS"


def test_task_outcome_missing_distance_data_does_not_crash_success_path():
    """If min_interrobot_distance_m could not be computed (e.g. a
    position field was unreadable) but everything else indicates
    success, the classifier must not raise -- it simply cannot apply the
    collision check, and falls through to the complete_count check."""
    signals = TaskOutcomeSignals(
        complete_count=2, expected_complete_count=2, min_interrobot_distance_m=None,
        safety_radius_m=0.14, controller_crashed=False,
    )
    outcome, _ = classify_task_outcome("VALID", signals)
    assert outcome == "SUCCESS"

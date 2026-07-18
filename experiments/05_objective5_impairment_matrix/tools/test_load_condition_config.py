from pathlib import Path

import pytest

from load_condition_config import (
    ConditionNotExecutableError,
    TrialIndexError,
    UnknownConditionError,
    load_conditions,
    resolve_trial_params,
)

REAL_CSV = Path(__file__).resolve().parent.parent / "objective5_impairment_matrix_conditions.csv"


def test_real_frozen_csv_parses_and_has_all_seven_conditions():
    conditions = load_conditions(REAL_CSV)
    assert set(conditions.keys()) == {"A", "B", "C", "D", "E", "F", "G"}


def test_condition_b_resolves_deterministic_delay_no_seed_needed():
    params = resolve_trial_params(REAL_CSV, "B", trial_index=1)
    assert params.delay_s == 0.20
    assert params.jitter_s == 0.0
    assert params.drop_probability == 0.0
    assert params.seed_epuck1 == 0
    assert params.seed_epuck2 == 0


def test_condition_d_resolves_matched_base_seed_scheme_per_trial():
    p1 = resolve_trial_params(REAL_CSV, "D", trial_index=1)
    p5 = resolve_trial_params(REAL_CSV, "D", trial_index=5)
    assert p1.seed_epuck1 == 4001
    assert p1.seed_epuck2 == 4002
    assert p5.seed_epuck1 == 4005
    assert p5.seed_epuck2 == 4006
    assert p1.jitter_s == 0.30
    assert p1.delay_s == 0.15


def test_condition_e_and_g_share_matched_base_seeds():
    e = resolve_trial_params(REAL_CSV, "E", trial_index=3)
    g = resolve_trial_params(REAL_CSV, "G", trial_index=3)
    assert e.seed_epuck1 == g.seed_epuck1
    assert e.seed_epuck2 == g.seed_epuck2


def test_condition_f_resolves_with_real_outage_params_now_that_the_extension_is_implemented():
    params = resolve_trial_params(REAL_CSV, "F", trial_index=1)
    assert params.delay_s == 0.0
    assert params.jitter_s == 0.0
    assert params.drop_probability == 0.0
    assert params.outage_period_s == 15.0
    assert params.outage_duration_s == 0.7
    assert params.outage_phase_s == 10.0
    assert params.seed_epuck1 == 4001
    assert params.seed_epuck2 == 4002


def test_condition_not_executable_error_still_exists_for_a_hypothetical_unfrozen_condition(tmp_path):
    """Regression guard for the rejection path itself: a CSV row with a
    literal NOT_IMPLEMENTED marker must still be rejected, even though no
    current row in the real frozen CSV exercises this anymore (F was the
    only one, and it's now implemented)."""
    import csv as csv_module

    stub_path = tmp_path / "stub_conditions.csv"
    header = list(load_conditions(REAL_CSV)["A"].keys())
    row_a = dict(load_conditions(REAL_CSV)["A"])
    row_a["delay_s"] = "NOT_IMPLEMENTED"
    with open(stub_path, "w", newline="", encoding="utf-8") as fh:
        writer = csv_module.DictWriter(fh, fieldnames=header)
        writer.writeheader()
        writer.writerow(row_a)
    with pytest.raises(ConditionNotExecutableError):
        resolve_trial_params(stub_path, "A", trial_index=1)


def test_unknown_condition_id_rejected():
    with pytest.raises(UnknownConditionError):
        resolve_trial_params(REAL_CSV, "Z", trial_index=1)


def test_trial_index_out_of_range_rejected():
    with pytest.raises(TrialIndexError):
        resolve_trial_params(REAL_CSV, "D", trial_index=6)
    with pytest.raises(TrialIndexError):
        resolve_trial_params(REAL_CSV, "D", trial_index=0)


def test_condition_a_resolves_all_zero_and_deterministic():
    params = resolve_trial_params(REAL_CSV, "A", trial_index=2)
    assert params.delay_s == params.jitter_s == params.drop_probability == 0.0
    assert params.seed_epuck1 == 0
    assert params.seed_epuck2 == 0


def test_outage_fields_are_zero_for_non_f_conditions():
    for cond in ("A", "B", "C", "D", "E", "G"):
        params = resolve_trial_params(REAL_CSV, cond, trial_index=1)
        assert params.outage_period_s == 0.0
        assert params.outage_duration_s == 0.0


def test_cli_prints_shell_sourceable_key_value_lines():
    import subprocess
    import sys

    result = subprocess.run(
        [sys.executable, str(Path(__file__).resolve().parent / "load_condition_config.py"),
         "--csv", str(REAL_CSV), "--condition-id", "D", "--trial-index", "2"],
        capture_output=True, text=True,
    )
    assert result.returncode == 0
    lines = dict(line.split("=", 1) for line in result.stdout.strip().splitlines())
    assert lines["DELAY_S"] == "0.15"
    assert lines["SEED_EPUCK1"] == "4002"
    assert lines["SEED_EPUCK2"] == "4003"


def test_cli_exits_nonzero_with_stderr_marker_on_bad_condition():
    import subprocess
    import sys

    result = subprocess.run(
        [sys.executable, str(Path(__file__).resolve().parent / "load_condition_config.py"),
         "--csv", str(REAL_CSV), "--condition-id", "Z", "--trial-index", "1"],
        capture_output=True, text=True,
    )
    assert result.returncode == 1
    assert "CONDITION_CONFIG_ERROR" in result.stderr

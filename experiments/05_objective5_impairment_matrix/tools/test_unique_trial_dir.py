import pytest

from unique_trial_dir import TrialDirectoryExistsError, require_unique_trial_dir, trial_dir_name


def test_trial_dir_name_is_deterministic_and_zero_padded():
    assert trial_dir_name("B", 1) == "objective5_impairment_matrix_v1_condition_B_trial01_attempt01"
    assert trial_dir_name("D", 5, attempt=2) == "objective5_impairment_matrix_v1_condition_D_trial05_attempt02"


def test_require_unique_trial_dir_returns_path_when_absent(tmp_path):
    result = require_unique_trial_dir(tmp_path, "B", 1)
    assert not result.exists()
    assert result.name == "objective5_impairment_matrix_v1_condition_B_trial01_attempt01"


def test_require_unique_trial_dir_refuses_to_overwrite_existing(tmp_path):
    existing = tmp_path / trial_dir_name("C", 3)
    existing.mkdir()
    with pytest.raises(TrialDirectoryExistsError):
        require_unique_trial_dir(tmp_path, "C", 3)


def test_require_unique_trial_dir_allows_a_new_attempt_number_after_a_failed_one(tmp_path):
    (tmp_path / trial_dir_name("C", 3, attempt=1)).mkdir()
    result = require_unique_trial_dir(tmp_path, "C", 3, attempt=2)
    assert result.name.endswith("attempt02")

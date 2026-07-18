#!/usr/bin/env python3
"""Trial-directory uniqueness check, shared by the parameterized
orchestrator -- refuses to reuse or overwrite an existing bag directory,
matching the discipline already established for
physical_single_device_zero_impairment_baseline_v1's *_short_window
attempts (never overwritten, always renamed/preserved instead)."""
from __future__ import annotations

from pathlib import Path


class TrialDirectoryExistsError(FileExistsError):
    pass


def trial_dir_name(condition_id: str, trial_index: int, attempt: int = 1) -> str:
    return f"objective5_impairment_matrix_v1_condition_{condition_id}_trial{trial_index:02d}_attempt{attempt:02d}"


def require_unique_trial_dir(root: Path, condition_id: str, trial_index: int, attempt: int = 1) -> Path:
    """Returns the full path to use, raising TrialDirectoryExistsError
    (never silently overwriting) if it already exists."""
    path = root / trial_dir_name(condition_id, trial_index, attempt)
    if path.exists():
        raise TrialDirectoryExistsError(
            f"{path} already exists -- refusing to overwrite; use a higher --attempt"
        )
    return path

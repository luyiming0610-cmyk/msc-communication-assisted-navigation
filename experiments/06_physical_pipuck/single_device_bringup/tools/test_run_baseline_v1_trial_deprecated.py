"""Tests that the old, defective orchestrator (run_baseline_v1_trial.sh) is
hard-disabled: it must exit immediately with a nonzero code, print the
DEPRECATED_SHORT_WINDOW_ORCHESTRATOR marker and a pointer to the v2
script, and must never create a bag directory, a diag directory, or spawn
any recorder/sampler/rosbag process -- regardless of what arguments it is
called with, including no arguments at all (the deprecation guard must run
before the old script's own argument-count check)."""
import shutil
import subprocess
from pathlib import Path

SCRIPT = Path(__file__).resolve().parent / "run_baseline_v1_trial.sh"


def _run(args, cwd):
    return subprocess.run(
        ["bash", str(SCRIPT), *args],
        cwd=cwd, capture_output=True, text=True, timeout=15,
    )


def test_deprecated_script_exits_nonzero_with_marker(tmp_path):
    result = _run(["trial99_should_never_be_created"], tmp_path)
    assert result.returncode != 0
    assert result.returncode == 3
    assert "DEPRECATED_SHORT_WINDOW_ORCHESTRATOR" in result.stderr
    assert "run_baseline_v1_trial_v2.sh" in result.stderr
    assert "a7f2a7e" in result.stderr


def test_deprecated_script_creates_no_bag_or_diag_directory(tmp_path):
    marker_name = "trial99_should_never_be_created"
    _run([marker_name], tmp_path)
    # The old script builds its native paths under a hardcoded
    # /home/eamon/epuck_comm_bags root -- confirm no directory containing
    # this test's unique marker name was created anywhere under that root.
    root = Path("/home/eamon/epuck_comm_bags")
    if root.exists():
        matches = list(root.glob(f"*{marker_name}*"))
        assert matches == [], f"deprecated script created directories despite guard: {matches}"


def test_deprecated_script_exits_before_any_argument_validation():
    """Even with zero arguments (which the old script's own usage check
    would normally reject with a different message/exit code), the
    deprecation guard must fire first -- proving it runs unconditionally
    at the very top, not after argument parsing."""
    result = subprocess.run(
        ["bash", str(SCRIPT)],
        capture_output=True, text=True, timeout=15,
    )
    assert result.returncode == 3
    assert "DEPRECATED_SHORT_WINDOW_ORCHESTRATOR" in result.stderr


def test_deprecated_script_spawns_no_child_processes(tmp_path):
    """Confirms the guard exits before sourcing ROS or starting any
    background job: stderr must contain ONLY the two deprecation lines,
    nothing from a ROS setup.bash source or a spawned recorder/sampler/
    rosbag process (those would each add their own distinct output)."""
    result = _run(["trialX"], tmp_path)
    assert result.returncode == 3
    stderr_lines = [line for line in result.stderr.splitlines() if line.strip()]
    assert len(stderr_lines) == 2
    assert stderr_lines[0].startswith("DEPRECATED_SHORT_WINDOW_ORCHESTRATOR")
    assert stderr_lines[1].startswith("Use run_baseline_v1_trial_v2.sh")
    assert result.stdout == ""

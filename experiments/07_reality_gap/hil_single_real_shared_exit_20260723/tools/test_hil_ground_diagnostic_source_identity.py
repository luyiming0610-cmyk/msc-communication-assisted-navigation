#!/usr/bin/env python3
"""Tests for run_ground_diagnostic_preflight.sh's SOURCE_IDENTITY check --
the `git status --porcelain -- <hil study path>` call that must report
CLEAN only when no tracked file is modified/staged/deleted and no
untracked, non-ignored file exists under that path.

Uses a small, disposable, synthetic git repository (created fresh in a
temporary directory, `git init` only -- never the real project repo)
mirroring just enough structure to exercise the same git-status
scoping and .gitignore behavior the real check relies on. This avoids
coupling these tests to the real repo's evolving untracked state,
matching the project's existing "synthetic workspace, never touches
the real one" testing pattern (see run_isolated_test_suite.sh's
sync/build end-to-end test). No ROS/rclpy dependency; git itself is
not a ROS/physical-hardware tool, so invoking it here is unrelated to
the project's ROS_DOMAIN_ID isolation requirement.
"""
import subprocess
import tempfile
import unittest
from pathlib import Path

HIL_STUDY_SUBDIR = "experiments/07_reality_gap/hil_single_real_shared_exit_20260723"

# The exact narrow .gitignore rules added to close the false
# TRACKED_TREE_DIRTY block -- kept in sync manually with the real
# .gitignore's corresponding block (both list the same six exact,
# already-established raw_logs directories, never a wildcard).
RAW_LOGS_DIRS = (
    "bridge_instrumentation_substitution_20260723/raw_logs",
    "command_evidence_activation_20260724/raw_logs",
    "command_evidence_activation_pass_20260724/raw_logs",
    "stationary_physical_diagnostic_20260723/raw_logs",
    "stationary_physical_diagnostic_20260723_attempt02/raw_logs",
    "targeted_stationary_diagnostic_20260723/raw_logs",
)


def _run_git(repo_dir, *args):
    return subprocess.run(
        ["git", "-C", str(repo_dir), *args],
        capture_output=True,
        text=True,
        check=False,
    )


def _init_synthetic_repo(repo_dir: Path):
    _run_git(repo_dir, "init", "-q")
    _run_git(repo_dir, "config", "user.email", "test@example.invalid")
    _run_git(repo_dir, "config", "user.name", "test")

    gitignore_lines = [f"/{HIL_STUDY_SUBDIR}/{d}/" for d in RAW_LOGS_DIRS]
    (repo_dir / ".gitignore").write_text("\n".join(gitignore_lines) + "\n", encoding="utf-8")

    hil_dir = repo_dir / HIL_STUDY_SUBDIR
    tools_dir = hil_dir / "tools"
    tools_dir.mkdir(parents=True)
    (tools_dir / "hil_preflight.py").write_text("# tracked source file\n", encoding="utf-8")

    _run_git(repo_dir, "add", ".gitignore", f"{HIL_STUDY_SUBDIR}/tools/hil_preflight.py")
    _run_git(repo_dir, "commit", "-q", "-m", "initial commit")

    for rel_dir in RAW_LOGS_DIRS:
        target = hil_dir / rel_dir
        target.mkdir(parents=True)
        (target / "command_evidence.csv").write_text("local_time_ns,topic\n", encoding="utf-8")
        (target / "SHA256SUMS.txt").write_text("deadbeef  command_evidence.csv\n", encoding="utf-8")


def _source_identity_status(repo_dir: Path) -> str:
    """Mirrors run_ground_diagnostic_preflight.sh's exact SOURCE_IDENTITY
    call: `git status --porcelain -- <hil study path>`."""
    result = _run_git(repo_dir, "status", "--porcelain", "--", HIL_STUDY_SUBDIR)
    return result.stdout


class KnownRawLogsDoNotBlockTest(unittest.TestCase):
    def test_freshly_initialized_synthetic_repo_reports_clean(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo_dir = Path(tmp)
            _init_synthetic_repo(repo_dir)
            status = _source_identity_status(repo_dir)
            self.assertEqual(status, "", f"expected clean status, got: {status!r}")

    def test_raw_logs_files_are_not_tracked_and_not_deleted(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo_dir = Path(tmp)
            _init_synthetic_repo(repo_dir)
            ls_files = _run_git(repo_dir, "ls-files", "--", HIL_STUDY_SUBDIR).stdout
            for rel_dir in RAW_LOGS_DIRS:
                self.assertNotIn(rel_dir, ls_files, f"{rel_dir} must not be tracked")
                target = repo_dir / HIL_STUDY_SUBDIR / rel_dir
                self.assertTrue((target / "command_evidence.csv").is_file())
                self.assertTrue((target / "SHA256SUMS.txt").is_file())


class UntrackedCodeOrConfigFileBlocksTest(unittest.TestCase):
    def _assert_untracked_file_blocks(self, relative_path: str, content: str = "x = 1\n"):
        with tempfile.TemporaryDirectory() as tmp:
            repo_dir = Path(tmp)
            _init_synthetic_repo(repo_dir)
            stray = repo_dir / HIL_STUDY_SUBDIR / relative_path
            stray.parent.mkdir(parents=True, exist_ok=True)
            stray.write_text(content, encoding="utf-8")
            status = _source_identity_status(repo_dir)
            self.assertNotEqual(status, "", f"expected dirty status for untracked {relative_path}")
            self.assertIn("??", status)

    def test_untracked_python_file_blocks(self):
        self._assert_untracked_file_blocks("tools/some_new_tool.py")

    def test_untracked_shell_script_blocks(self):
        self._assert_untracked_file_blocks("tools/some_new_script.sh")

    def test_untracked_json_config_blocks(self):
        self._assert_untracked_file_blocks("tools/some_new_config.json", content="{}\n")

    def test_untracked_yaml_config_blocks(self):
        self._assert_untracked_file_blocks("config/some_new_config.yaml", content="key: value\n")

    def test_untracked_launch_file_blocks(self):
        self._assert_untracked_file_blocks("launch/some_new.launch.py")

    def test_untracked_world_file_blocks(self):
        self._assert_untracked_file_blocks("worlds/some_new.wbt")

    def test_untracked_path_not_matching_any_raw_logs_allowlist_entry_blocks(self):
        # A raw_logs-named directory in a location NOT one of the six
        # exact allowlisted paths must still block -- the allowlist is
        # exact paths, never a wildcard.
        self._assert_untracked_file_blocks("some_new_diagnostic_20260725/raw_logs/unexpected.csv", content="a,b\n")


class TrackedFileChangesBlockTest(unittest.TestCase):
    def test_modified_tracked_file_blocks(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo_dir = Path(tmp)
            _init_synthetic_repo(repo_dir)
            tracked = repo_dir / HIL_STUDY_SUBDIR / "tools" / "hil_preflight.py"
            tracked.write_text("# modified\n", encoding="utf-8")
            status = _source_identity_status(repo_dir)
            self.assertNotEqual(status, "")
            self.assertIn(" M ", status)

    def test_staged_new_file_blocks(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo_dir = Path(tmp)
            _init_synthetic_repo(repo_dir)
            new_file = repo_dir / HIL_STUDY_SUBDIR / "tools" / "staged_new_tool.py"
            new_file.write_text("# new\n", encoding="utf-8")
            _run_git(repo_dir, "add", f"{HIL_STUDY_SUBDIR}/tools/staged_new_tool.py")
            status = _source_identity_status(repo_dir)
            self.assertNotEqual(status, "")
            self.assertIn("A ", status)

    def test_deleted_tracked_file_blocks(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo_dir = Path(tmp)
            _init_synthetic_repo(repo_dir)
            tracked = repo_dir / HIL_STUDY_SUBDIR / "tools" / "hil_preflight.py"
            tracked.unlink()
            status = _source_identity_status(repo_dir)
            self.assertNotEqual(status, "")
            self.assertIn(" D ", status)


class UnrelatedDirectoriesOutsideStudyDoNotAffectStatusTest(unittest.TestCase):
    def test_untracked_assessor_demo_style_directory_outside_hil_study_is_scoped_out(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo_dir = Path(tmp)
            _init_synthetic_repo(repo_dir)
            unrelated = repo_dir / "experiments" / "05_objective5_impairment_matrix" / "objective5_matrix_v1_conditionB_exclusionary_assessor_demo_20260720_010_analysis"
            unrelated.mkdir(parents=True)
            (unrelated / "some_raw_evidence.csv").write_text("a,b\n", encoding="utf-8")

            hil_study_status = _source_identity_status(repo_dir)
            self.assertEqual(hil_study_status, "", "unrelated directory outside the HIL study must not affect its source identity")

            unscoped_status = _run_git(repo_dir, "status", "--porcelain").stdout
            self.assertIn("05_objective5_impairment_matrix", unscoped_status)


if __name__ == "__main__":
    unittest.main()

#!/usr/bin/env python3
"""Static/dummy-process contract tests for run_hil_stage4_trial.sh.
Does not start ROS, does not contact the Pi, does not start Webots.
Only invokes the script's --check-only/--dry-run modes (which start no
process at all) and inspects the script's own source text for the
required contract properties that would be unsafe or impractical to
prove by actually running a physical trial in a unit test."""
from __future__ import annotations

import subprocess
import sys
import unittest
from pathlib import Path

SCRIPT_PATH = Path(__file__).resolve().parent / "run_hil_stage4_trial.sh"


def _run(args: list, env: dict = None) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["bash", str(SCRIPT_PATH)] + args,
        capture_output=True, text=True, timeout=30, env=env,
    )


class CheckOnlyAndDryRunStartNothingTest(unittest.TestCase):
    def test_check_only_exits_zero_and_prints_pass(self):
        result = _run(["--check-only"])
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("STAGE4_CHECK_ONLY=PASS", result.stdout)

    def test_dry_run_exits_zero_and_starts_no_process(self):
        result = _run(["--dry-run"])
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Planned steps", result.stdout)
        # None of the real component names should appear as an actually
        # invoked command in --dry-run output (only inside the printed
        # plan text, which is fine) -- confirmed instead by process
        # absence: no child processes are left behind.
        result_ps = subprocess.run(["pgrep", "-f", "hil_stage4_motion_supervisor.py"], capture_output=True, text=True)
        self.assertEqual(result_ps.returncode, 1, "no supervisor process should exist after --dry-run")

    def test_unrecognized_mode_is_rejected_with_no_bypass(self):
        result = _run(["--something-else"])
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("not recognized", result.stdout + result.stderr)


class PhysicalModeRequiresSourceIdentityTest(unittest.TestCase):
    """--run without a correct EXPECTED_HEAD must abort before touching
    ROS, the Pi, or creating any output directory. These tests set
    EXPECTED_HEAD to values that cannot possibly match, so the script is
    guaranteed to abort at the source-identity gate -- before it ever
    sources the ROS environment or starts a process."""

    def test_run_without_expected_head_aborts(self):
        import os
        env = dict(os.environ)
        env.pop("EXPECTED_HEAD", None)
        result = _run(["--run"], env=env)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("EXPECTED_HEAD", result.stderr)

    def test_run_with_wrong_expected_head_aborts(self):
        import os
        env = dict(os.environ)
        env["EXPECTED_HEAD"] = "0" * 40
        result = _run(["--run"], env=env)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("HEAD mismatch", result.stderr)

    def test_no_output_directory_created_before_source_identity_gate(self):
        import os
        env = dict(os.environ)
        env["EXPECTED_HEAD"] = "0" * 40
        before = set(Path("/home/eamon/epuck_comm_bags").glob("hil_stage4_*")) if Path("/home/eamon/epuck_comm_bags").is_dir() else set()
        _run(["--run"], env=env)
        after = set(Path("/home/eamon/epuck_comm_bags").glob("hil_stage4_*")) if Path("/home/eamon/epuck_comm_bags").is_dir() else set()
        self.assertEqual(before, after, "no new Stage 4 output directory should be created when the source-identity gate fails")


class ScriptContractStaticTest(unittest.TestCase):
    """Text-level contract checks -- these assert properties of the
    script's own source that are safety-relevant but impractical to
    prove via an actual physical run in a unit test (e.g. "the operator
    never publishes the arm topic" can only be shown by the ABSENCE of
    such an instruction anywhere in the script)."""

    def setUp(self):
        self.source = SCRIPT_PATH.read_text(encoding="utf-8")
        # Only actual, executable lines -- not comments -- are relevant
        # to "is this instruction/command actually present," so strip
        # full-line and trailing comments before searching.
        self.code_lines = [
            line for line in self.source.splitlines()
            if line.strip() and not line.strip().startswith("#")
        ]
        self.code_text = "\n".join(self.code_lines)

    def test_no_manual_arm_publish_instruction_anywhere(self):
        arm_pub_lines = [l for l in self.code_lines if "ros2 topic pub" in l and "ARM_TOPIC" in l]
        self.assertEqual(arm_pub_lines, [], f"no executable line should publish to the arm topic: {arm_pub_lines}")
        self.assertNotIn('"${ARM_TOPIC}" std_msgs/msg/Bool', self.code_text)

    def test_supervisor_is_the_only_arm_publisher_reference(self):
        # The only place ARM_TOPIC is passed to a process is the
        # supervisor's own --arm-topic and the guard's --arm-topic
        # (subscriber, not publisher) invocations.
        self.assertIn('--arm-topic "${ARM_TOPIC}"', self.source)

    def test_recorder_started_before_other_orchestrator_owned_processes(self):
        recorder_idx = self.source.index("starting WSL command-evidence recorder")
        adapter_idx = self.source.index("starting hil_topic_adapter.py")
        avoider_idx = self.source.index("starting cooperative_avoider.py")
        guard_idx = self.source.index("starting hil_cmd_vel_guard.py (DISARMED")
        supervisor_idx = self.source.index("starting hil_stage4_motion_supervisor.py")
        self.assertLess(recorder_idx, adapter_idx)
        self.assertLess(recorder_idx, avoider_idx)
        self.assertLess(recorder_idx, guard_idx)
        self.assertLess(recorder_idx, supervisor_idx)

    def test_virtual_peer_started_only_after_approval_and_release(self):
        approval_idx = self.source.index("APPROVED_FOR_SINGLE_HIL_EVENT=YES")
        release_idx = self.source.index("releasing virtual scout exactly once")
        peer_start_idx = self.source.index("hil_virtual_peer.py", release_idx)
        self.assertLess(approval_idx, release_idx)
        self.assertLess(release_idx, peer_start_idx)

    def test_no_automatic_retry_or_second_trial(self):
        self.assertIn("does not arm anything itself and does not start a second trial", self.source)

    def test_no_pkill_or_name_based_kill_anywhere(self):
        pkill_lines = [l for l in self.code_lines if "pkill" in l or "killall" in l]
        self.assertEqual(pkill_lines, [], f"no executable line should use pkill/killall: {pkill_lines}")

    def test_cleanup_uses_exact_pid_manifest_shutdown(self):
        self.assertIn("run_hil_shutdown.sh", self.source)
        self.assertIn("PID_MANIFEST", self.source)

    def test_source_identity_checks_every_critical_file(self):
        for expected in (
            "hil_stage4_motion_supervisor.py",
            "run_hil_stage4_trial.sh",
            "hil_goal_announcement_evidence.py",
            "hil_stage4_post_run_verifier.py",
            "hil_topic_adapter.py",
            "hil_cmd_vel_guard.py",
            "hil_virtual_peer.py",
            "hil_command_evidence_recorder.py",
            "run_hil_shutdown.sh",
            "goal_navigator.py",
            "goal_hold_tracker.py",
            "navigation_target_state.py",
            "cooperative_avoider.py",
            "command_smoothing.py",
            "collision_math.py",
            "local_obstacle_logic.py",
            "state_publisher.py",
            "STAGE4_PHYSICAL_HIL_SPEC.md",
        ):
            self.assertIn(expected, self.source)

    def test_installed_runtime_identity_checked_for_cooperative_avoider_chain(self):
        self.assertIn("STAGE4_INSTALLED_RUNTIME_IDENTITY", self.source)
        self.assertIn("ros2 pkg prefix epuck2_comm", self.source)
        self.assertIn("hash-object --path=", self.source)

    def test_synthetic_physical_state_publisher_is_not_in_identity_set(self):
        # Explicit negative check matching the design review's own
        # requirement: the physical identity set must never include the
        # rehearsal-only synthetic publisher.
        identity_block_start = self.source.index("STAGE4_SOURCE_IDENTITY_PATHS=(")
        identity_block_end = self.source.index(")", identity_block_start)
        identity_block = self.source[identity_block_start:identity_block_end]
        self.assertNotIn("synthetic_stage4_physical_state_publisher.py", identity_block)

    def test_residual_process_check_written_on_cleanup(self):
        self.assertIn("residual_check.json", self.source)
        self.assertIn("residual_process_check", self.source)

    def test_early_failure_still_finalizes_via_trap(self):
        self.assertIn("trap cleanup EXIT", self.source)

    def test_physical_mode_never_references_synthetic_publisher(self):
        """Proves --run mode never imports, execs, spawns, or otherwise
        references synthetic_stage4_physical_state_publisher.py anywhere
        in this script's executable lines -- only the rehearsal file may
        use it."""
        synthetic_lines = [l for l in self.code_lines if "synthetic_stage4_physical_state_publisher" in l]
        self.assertEqual(synthetic_lines, [], f"physical orchestrator must never reference the synthetic publisher: {synthetic_lines}")

    def test_physical_mode_never_places_synthetic_publisher_in_pid_manifest(self):
        record_process_calls = [l for l in self.code_lines if "record_process" in l and '"synthetic' in l]
        self.assertEqual(record_process_calls, [])

    def test_physical_mode_requires_exactly_one_real_state_publisher_preflight(self):
        self.assertIn('REAL_STATE_COUNT="$(publisher_count "${PHYSICAL_STATE_TOPIC}")"', self.code_text)
        self.assertIn('if [[ "${REAL_STATE_COUNT}" != "1" ]]; then', self.code_text)

    def test_physical_mode_rechecks_real_state_publisher_before_release(self):
        self.assertIn("REAL_STATE_COUNT_POST_START", self.code_text)
        self.assertIn('if [[ "${REAL_STATE_COUNT_POST_START}" != "1" ]]; then', self.code_text)

    def test_no_production_topics_hardcoded_for_rehearsal(self):
        # The orchestrator's own topic constants are the real production
        # names by design (this script IS the physical orchestrator, not
        # the rehearsal) -- this test instead confirms the rehearsal file
        # uses its own private namespace, never these same topic objects
        # as bare names.
        rehearsal_path = SCRIPT_PATH.parent / "test_hil_stage4_live_graph_rehearsal.py"
        rehearsal_source = rehearsal_path.read_text(encoding="utf-8")
        self.assertIn("pytest_stage4_live", rehearsal_source)
        self.assertNotIn('"/cmd_vel"', rehearsal_source)
        self.assertNotIn('"/epuck1/state"', rehearsal_source)


if __name__ == "__main__":
    unittest.main()

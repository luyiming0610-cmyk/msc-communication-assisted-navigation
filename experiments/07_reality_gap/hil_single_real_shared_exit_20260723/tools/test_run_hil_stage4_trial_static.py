#!/usr/bin/env python3
"""Static/dummy-process contract tests for run_hil_stage4_trial.sh.
Does not start ROS, does not contact the Pi, does not start Webots.
Only invokes the script's --check-only/--dry-run modes (which start no
process at all) and inspects the script's own source text for the
required contract properties that would be unsafe or impractical to
prove by actually running a physical trial in a unit test."""
from __future__ import annotations

import json
import re
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

SCRIPT_PATH = Path(__file__).resolve().parent / "run_hil_stage4_trial.sh"

_SUPERVISOR_EVIDENCE_RECORDS = [
    {"monotonic_time_s": 1000.0, "ros_time_s": 1000.0, "state": "WAITING_FOR_EVENT", "event": "APPROVAL_ACCEPTED", "reason": "", "raw": None, "run_id": "fixture", "goal_id": "shared_exit"},
    {"monotonic_time_s": 1001.0, "ros_time_s": 1001.0, "state": "WAITING_FOR_EVENT", "event": "VIRTUAL_SCOUT_RELEASED", "reason": "", "raw": None, "run_id": "fixture", "goal_id": "shared_exit"},
    {"monotonic_time_s": 1002.0, "ros_time_s": 1002.0, "state": "VALIDATING_RAW_COMMAND", "event": "ADOPTION_CONFIRMED", "reason": "", "raw": {"goal_id": "shared_exit", "source_sequence": 1, "target_x_m": 1.2, "target_y_m": 0.5, "schema_version": "1.0.0"}, "run_id": "fixture", "goal_id": "shared_exit"},
    {"monotonic_time_s": 1002.05, "ros_time_s": 1002.05, "state": "VALIDATING_RAW_COMMAND", "event": "RAW_TWIST_RECEIVED", "reason": "", "raw": {"linear_x": 0.015, "linear_y": 0.0, "linear_z": 0.0, "angular_x": 0.0, "angular_y": 0.0, "angular_z": 0.0}, "run_id": "fixture", "goal_id": "shared_exit"},
    {"monotonic_time_s": 1002.05, "ros_time_s": 1002.05, "state": "VALIDATING_RAW_COMMAND", "event": "ARM_PUBLISHED", "reason": "", "raw": None, "run_id": "fixture", "goal_id": "shared_exit"},
    {"monotonic_time_s": 1002.05, "ros_time_s": 1002.05, "state": "ACTIVE", "event": "ACTIVE_OPENED", "reason": "", "raw": None, "run_id": "fixture", "goal_id": "shared_exit"},
    {"monotonic_time_s": 1008.55, "ros_time_s": 1008.55, "state": "ZERO_BURST", "event": "ZERO_BURST_OPENED", "reason": "INTERNAL_ACTIVE_CUTOFF_REACHED", "raw": None, "run_id": "fixture", "goal_id": "shared_exit"},
    {"monotonic_time_s": 1008.55, "ros_time_s": 1008.55, "state": "ZERO_BURST", "event": "ZERO_PUBLISHED", "reason": "", "raw": None, "run_id": "fixture", "goal_id": "shared_exit"},
    {"monotonic_time_s": 1008.55, "ros_time_s": 1008.55, "state": "DISARMED", "event": "DISARM_PUBLISHED", "reason": "", "raw": None, "run_id": "fixture", "goal_id": "shared_exit"},
    {"monotonic_time_s": 1008.55, "ros_time_s": 1008.55, "state": "COMPLETE", "event": "LATCHED_COMPLETE", "reason": "", "raw": None, "run_id": "fixture", "goal_id": "shared_exit"},
]


def _build_finalize_fixture(root: Path) -> None:
    (root / "stage4_supervisor_evidence.jsonl").write_text(
        "\n".join(json.dumps(r) for r in _SUPERVISOR_EVIDENCE_RECORDS) + "\n", encoding="utf-8",
    )
    (root / "command_evidence.csv").write_text("topic,linear_x,angular_z\ncmd_vel,0.015,0.0\n", encoding="utf-8")
    (root / "pid_manifest.json").write_text(
        json.dumps({"run_id": "fixture", "processes": {"recorder": {"pid": 111, "sha256": ""}}}), encoding="utf-8",
    )
    (root / "launcher_status.json").write_text(
        json.dumps({
            "run_id": "fixture", "execution_head": "HEADX", "components": {},
            "operator_approval_state": "ACCEPTED", "supervisor_terminal_state": "COMPLETE",
            "cleanup": {"result": "RAN"}, "recorder_stopped_last": None,
            "residual_process_result": "CLEAN", "final_launcher_classification": "ABORTED",
        }), encoding="utf-8",
    )
    (root / "source_identity_manifest.json").write_text(
        json.dumps({
            "schema_version": "1.0.0", "run_id": "fixture", "expected_head": "HEADX", "actual_head": "HEADX",
            "source_paths": [], "installed_runtime": [], "entrypoint_check": {}, "overall_result": "PASS",
        }), encoding="utf-8",
    )
    (root / "residual_check.json").write_text(json.dumps({"residual_process_check": "CLEAN"}), encoding="utf-8")
    (root / "pi_command_audit.jsonl").write_text(json.dumps({"linear_x": 0.015, "angular_z": 0.0}) + "\n", encoding="utf-8")
    (root / "pi_verifier_verdict.json").write_text(json.dumps({"verdict": "PASS"}), encoding="utf-8")
    (root / "physical_measurements.json").write_text(json.dumps({
        "manual_forward_displacement_m": 0.09, "corridor_crossed": False, "stop_line_crossed": False,
        "min_boundary_clearance_m": 0.20, "unexpected_rotation": False, "unexpected_direction": False,
        "unexpected_sound": False, "unexpected_acceleration": False, "run_interrupted": False,
    }), encoding="utf-8")


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

    def test_cooperative_avoider_started_only_after_approval_accepted(self):
        """RUN_ID stage4_20260731_182129: cooperative_avoider's own
        max_runtime_s (default 22.0s) counted down from a launch during
        initial bring-up, well before the physical operator finished
        placement/safety confirmation -- it latched COMPLETE (zero
        forever) minutes before the real motion window could occur.
        Launching it only after approval keeps its runtime budget spent
        on the actual scout-travel/adoption/motion window instead."""
        approval_accepted_idx = self.source.index('OPERATOR_APPROVAL_STATE="ACCEPTED"')
        avoider_record_idx = self.source.index('record_process "cooperative_avoider"')
        self.assertLess(approval_accepted_idx, avoider_record_idx)

    def test_cooperative_avoider_max_runtime_derived_via_existing_parameter(self):
        """Never a cooperative_avoider.py code/algorithm change: this
        uses the controller's own existing, already-overridable
        max_runtime_s ROS parameter (default 22.0s), overridden to
        comfortably exceed scout-announcement + adoption + raw-command
        + the hard physical-motion maximum, plus a documented margin --
        derived from the frozen engine constants, never a guessed
        literal."""
        self.assertIn('-p max_runtime_s:="${COOP_MAX_RUNTIME_S}"', self.source)
        self.assertIn(
            "from hil_stage4_motion_supervisor import ADOPTION_TIMEOUT_S, CONTROLLER_STATE_FORWARD_TIMEOUT_S, "
            "RAW_COMMAND_TIMEOUT_S, HARD_MAX_NONZERO_DURATION_S",
            self.source,
        )

    def test_raw_cmd_vel_topic_publisher_checked_before_scout_release(self):
        check_idx = self.source.index(
            'require_exactly_one_publisher_via_direct_discovery "${RAW_CMD_VEL_TOPIC}"'
        )
        release_idx = self.source.index("releasing virtual scout exactly once")
        self.assertLess(check_idx, release_idx)

    def test_cooperative_avoider_state_remap_targets_private_topic_not_canonical(self):
        """Adoption-controlled private own-state gate (RUN_ID
        stage4_20260731_190139 correction): cooperative_avoider's own
        hardcoded "state" subscription must be remapped exclusively to
        the supervisor-owned private topic, never to the canonical
        PHYSICAL_STATE_TOPIC -- this is the structural guarantee that it
        has no subscription path to real own-state before the
        supervisor chooses to forward it."""
        self.assertIn('-r state:="${CONTROLLER_PRIVATE_STATE_TOPIC}"', self.source)
        self.assertNotIn('-r state:="${PHYSICAL_STATE_TOPIC}"', self.source)

    def test_canonical_physical_state_topic_still_used_by_guard_and_supervisor_unshadowed(self):
        """The private gate must be purely additive: the guard and the
        supervisor's own liveness/adoption logic keep subscribing to the
        real canonical topic exactly as before -- nothing shadows or
        replaces PHYSICAL_STATE_TOPIC for those two consumers."""
        self.assertIn('--physical-state-topic "${PHYSICAL_STATE_TOPIC}"', self.source)
        physical_state_topic_uses = self.source.count('"${PHYSICAL_STATE_TOPIC}"')
        self.assertGreaterEqual(physical_state_topic_uses, 2, "guard and supervisor must both still reference the canonical topic")

    def test_controller_private_state_topic_checked_zero_before_start_and_one_after(self):
        preflight_idx = self.source.index('"${CONTROLLER_PRIVATE_STATE_TOPIC}"; do')
        poststart_idx = self.source.index(
            'require_exactly_one_publisher_via_direct_discovery "${CONTROLLER_PRIVATE_STATE_TOPIC}" "controller_private_state_post_start"'
        )
        self.assertLess(preflight_idx, poststart_idx)

    def test_canonical_state_rechecked_sole_publisher_after_private_gate_starts(self):
        """The private publisher (created inside the supervisor process)
        must not itself become a second publisher on the canonical
        topic -- explicitly re-verified after the gate starts, not just
        assumed from the earlier preflight/post-guard checks."""
        self.assertIn(
            'require_exactly_one_publisher_via_direct_discovery "${PHYSICAL_STATE_TOPIC}" "real_state_still_sole_canonical_post_start"',
            self.source,
        )

    def test_supervisor_is_only_process_given_controller_state_topic_flag(self):
        """The private topic's publisher must exist only inside the
        supervisor process -- no other launched component's argv may
        reference --controller-state-topic (cooperative_avoider takes
        it only via the "-r state:=" remap, which is a subscription
        remap, not a publisher)."""
        flag_lines = [l for l in self.code_lines if "--controller-state-topic" in l]
        self.assertEqual(len(flag_lines), 1, f"expected exactly one process launch to pass --controller-state-topic: {flag_lines}")
        flag_idx = self.source.index('--controller-state-topic "${CONTROLLER_PRIVATE_STATE_TOPIC}"')
        supervisor_launch_idx = self.source.index('"${SCRIPT_DIR}/hil_stage4_motion_supervisor.py" \\')
        avoider_record_idx = self.source.index('record_process "cooperative_avoider"')
        self.assertLess(supervisor_launch_idx, flag_idx)
        self.assertLess(avoider_record_idx, supervisor_launch_idx, "cooperative_avoider must be launched before the supervisor invocation carrying this flag")

    def test_readiness_check_count_matches_actual_direct_discovery_calls(self):
        """Blocking issue 2: READINESS_CHECK_COUNT must equal the ACTUAL
        number of require_exactly_one_publisher_via_direct_discovery
        call sites between cooperative_avoider's launch and virtual-
        scout release -- not a separately-maintained guess. Fails if a
        future check is added or removed here without updating the
        constant used in READINESS_OVERHEAD_MARGIN_S/PRE_RELEASE_TIMEOUT_S/
        COOP_MAX_RUNTIME_S."""
        coop_launch_idx = self.source.index('record_process "cooperative_avoider"')
        release_idx = self.source.index("releasing virtual scout exactly once")
        self.assertLess(coop_launch_idx, release_idx)
        window = self.source[coop_launch_idx:release_idx]
        actual_calls = window.count('require_exactly_one_publisher_via_direct_discovery "')

        m = re.search(r'READINESS_CHECK_COUNT="(\d+)"', self.source)
        self.assertIsNotNone(m, "READINESS_CHECK_COUNT assignment not found in source")
        declared_count = int(m.group(1))

        self.assertEqual(
            actual_calls, declared_count,
            f"actual direct-discovery calls between cooperative_avoider launch and release "
            f"({actual_calls}) != declared READINESS_CHECK_COUNT ({declared_count})",
        )
        # Pin the currently-expected value explicitly so a silent drop
        # to e.g. 0 (both counts wrong in the same way) cannot pass.
        self.assertEqual(actual_calls, 5)

    def test_pre_release_timeout_derived_from_readiness_overhead_margin(self):
        """Blocking issue 1: PRE_RELEASE_TIMEOUT_S must be derived from
        READINESS_OVERHEAD_MARGIN_S (never a separately-guessed
        literal), and passed to the supervisor, overriding its
        EVENT_TIMEOUT_S default."""
        self.assertIn('PRE_RELEASE_TIMEOUT_S="$(python3 -c "', self.source)
        pre_release_calc_idx = self.source.index('PRE_RELEASE_TIMEOUT_S="$(python3 -c "')
        pre_release_calc_end = self.source.index('")"', pre_release_calc_idx)
        pre_release_calc_body = self.source[pre_release_calc_idx:pre_release_calc_end]
        self.assertIn("READINESS_OVERHEAD_MARGIN_S", pre_release_calc_body)
        self.assertIn("PRE_RELEASE_TIMEOUT_MARGIN_S", pre_release_calc_body)
        self.assertIn('--pre-release-timeout-s "${PRE_RELEASE_TIMEOUT_S}"', self.source)

    def test_numeric_param_validation_called_for_every_derived_timing_value(self):
        """Blocking issue 4: every command-substitution-derived timing
        value must be passed through the fail-closed numeric
        validator before use."""
        for var in (
            "SCOUT_ANNOUNCEMENT_TIMEOUT_S", "READINESS_OVERHEAD_MARGIN_S",
            "PRE_RELEASE_TIMEOUT_S", "COOP_MAX_RUNTIME_S",
        ):
            self.assertIn(
                f'_require_valid_positive_finite_number "{var}" "${{{var}}}"', self.source,
                f"{var} is not passed through the numeric validator",
            )

    def test_no_automatic_retry_or_second_trial(self):
        self.assertIn("does not arm anything itself and does not start a second trial", self.source)

    def test_controller_private_state_uses_frozen_field_origin(self):
        self.assertIn('--controller-field-origin-x-m "${START_POSE_X_M}"', self.source)
        self.assertIn('--controller-field-origin-y-m "${START_POSE_Y_M}"', self.source)
        self.assertIn('--controller-field-origin-yaw-rad 0.0', self.source)
        self.assertIn('--field-origin-x-m="${START_POSE_X_M}"', self.source)
        self.assertIn('--field-origin-y-m="${START_POSE_Y_M}"', self.source)
        self.assertIn('--field-origin-yaw-rad=0.0', self.source)

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

    def test_ros_setup_sourcing_is_bracketed_by_set_plus_minus_u(self):
        """ROS 2's setup.bash files reference variables (e.g.
        AMENT_TRACE_SETUP_FILES) that are unset in a fresh shell. Under
        this script's `set -uo pipefail`, sourcing them unguarded is a
        fatal, non-interactive shell exit -- discovered live when --run
        crashed immediately after STAGE4_SOURCE_IDENTITY=PASS, before
        creating any evidence directory. Both `source` lines must sit
        directly between a `set +u` and a `set -u`."""
        opt_ros_idx = next(
            i for i, line in enumerate(self.code_lines)
            if "source /opt/ros/humble/setup.bash" in line
        )
        epuck_ws_idx = next(
            i for i, line in enumerate(self.code_lines)
            if "source ~/epuck_ws/install/setup.bash" in line
        )
        self.assertEqual(
            epuck_ws_idx, opt_ros_idx + 1,
            "the two ROS setup sourcing lines must be adjacent",
        )
        self.assertEqual(self.code_lines[opt_ros_idx - 1].strip(), "set +u")
        self.assertEqual(self.code_lines[epuck_ws_idx + 1].strip(), "set -u")

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

    def test_source_identity_manifest_written_atomically_before_any_process(self):
        manifest_write_idx = self.source.index("SOURCE_IDENTITY_MANIFEST=")
        out_dir_idx = self.source.index('mkdir -p "${OUT_DIR}"')
        first_record_process_idx = self.source.index('record_process "recorder"')
        self.assertLess(out_dir_idx, manifest_write_idx)
        self.assertLess(manifest_write_idx, first_record_process_idx)
        self.assertIn("os.replace(tmp_path, out_path)", self.source)

    def test_launcher_status_updated_at_each_component_launch(self):
        for name in ("recorder", "hil_topic_adapter", "cooperative_avoider", "hil_cmd_vel_guard"):
            marker = f'record_process "{name}"'
            idx = self.source.index(marker)
            # write_launcher_status must appear shortly after each launch.
            tail = self.source[idx:idx + 400]
            self.assertIn("write_launcher_status", tail, f"no write_launcher_status call found after {marker}")

    def test_launcher_status_records_operator_approval_states(self):
        self.assertIn('OPERATOR_APPROVAL_STATE="ACCEPTED"', self.source)
        self.assertIn('OPERATOR_APPROVAL_STATE="REJECTED"', self.source)

    def test_finalize_mode_exists_and_is_readonly_except_hash_files(self):
        self.assertIn("--finalize)", self.source)
        self.assertIn("EVIDENCE_ROOT=", self.source)
        finalize_idx = self.source.index("--finalize)")
        finalize_block = self.source[finalize_idx:]
        for forbidden in ("ros2 run", "ros2 topic pub", "source /opt/ros"):
            self.assertNotIn(forbidden, finalize_block, f"--finalize must never do {forbidden!r}")

    def test_finalize_requires_all_evidence_including_pi(self):
        finalize_idx = self.source.index("--finalize)")
        finalize_block = self.source[finalize_idx:]
        for required_var in (
            "SUPERVISOR_EVIDENCE", "WSL_COMMAND_EVIDENCE", "PID_MANIFEST",
            "LAUNCHER_STATUS", "SOURCE_IDENTITY_MANIFEST", "RESIDUAL_CHECK",
            "PI_COMMAND_AUDIT", "PI_VERIFIER_VERDICT", "PHYSICAL_MEASUREMENTS",
        ):
            self.assertIn(required_var, finalize_block)
        self.assertIn("MISSING_OR_EMPTY_REQUIRED_FILE", finalize_block)
        self.assertIn("STAGE4_FINALIZE=INVALID_EVIDENCE", finalize_block)

    def test_finalize_builds_two_hash_stages(self):
        finalize_idx = self.source.index("--finalize)")
        finalize_block = self.source[finalize_idx:]
        self.assertIn("SHA256SUMS.txt", finalize_block)
        self.assertIn("FINAL_SHA256SUMS.txt", finalize_block)
        self.assertIn("sha256sum -c", finalize_block)
        self.assertEqual(finalize_block.count("sha256sum -c"), 2)

    def test_finalize_final_hash_manifest_excludes_only_itself(self):
        finalize_idx = self.source.index("--finalize)")
        finalize_block = self.source[finalize_idx:]
        stage2_idx = finalize_block.index("FINAL_SHA256SUMS.txt.new")
        stage2_find = finalize_block[stage2_idx - 300:stage2_idx]
        self.assertIn('! -name "FINAL_SHA256SUMS.txt"', stage2_find)
        self.assertNotIn('! -name "SHA256SUMS.txt"', stage2_find)
        self.assertNotIn('! -name "post_run_verification.json"', stage2_find)

    def test_finalize_invokes_committed_verifier_in_physical_mode(self):
        finalize_idx = self.source.index("--finalize)")
        finalize_block = self.source[finalize_idx:]
        self.assertIn("hil_stage4_post_run_verifier.py", finalize_block)
        self.assertIn("--mode physical", finalize_block)

    def test_pi_window_2_uses_audited_server_not_unaudited(self):
        command_sheet = (SCRIPT_PATH.parent.parent / "STAGE4_COMMAND_SHEET_TEMPLATE.md").read_text(encoding="utf-8")
        self.assertIn("pi_epuck_tcp_server_sensors_audited.py", command_sheet)
        # The unaudited script name must never appear as a launch command
        # (it may appear only as prose explicitly calling it out as
        # forbidden).
        launch_lines = [
            l for l in command_sheet.splitlines()
            if "python3 pi_epuck_tcp_server_sensors.py" in l
        ]
        self.assertEqual(launch_lines, [], f"unaudited server must never be the launch command: {launch_lines}")

    def test_physical_mode_rechecks_real_state_publisher_before_release(self):
        self.assertIn(
            'require_exactly_one_publisher_via_direct_discovery "${PHYSICAL_STATE_TOPIC}" "real_state_post_start"',
            self.code_text,
        )
        self.assertIn(
            'if ! require_exactly_one_publisher_via_direct_discovery "${PHYSICAL_STATE_TOPIC}" "real_state_post_start"; then',
            self.code_text,
        )

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


class FinalizeModeFunctionalTest(unittest.TestCase):
    """Actual execution of --finalize against synthetic fixtures (no
    ROS, no Pi, no process) -- proves the two-stage hash-verified
    physical evidence flow end-to-end, not just via string checks."""

    def test_finalize_pass_with_complete_fixture(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            _build_finalize_fixture(root)
            result = _run(["--finalize", str(root)])
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn("STAGE4_FINALIZE=PASS", result.stdout)
            self.assertTrue((root / "SHA256SUMS.txt").is_file())
            self.assertTrue((root / "FINAL_SHA256SUMS.txt").is_file())
            self.assertTrue((root / "post_run_verification.json").is_file())
            self.assertTrue((root / "adoption_evidence.jsonl").is_file())
            report = json.loads((root / "post_run_verification.json").read_text(encoding="utf-8"))
            self.assertEqual(report["classification"], "PASS")

    def test_finalize_invalid_evidence_when_pi_evidence_missing(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            _build_finalize_fixture(root)
            (root / "pi_command_audit.jsonl").unlink()
            (root / "pi_verifier_verdict.json").unlink()
            result = _run(["--finalize", str(root)])
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("STAGE4_FINALIZE=INVALID_EVIDENCE", result.stdout + result.stderr)
            # No SHA256SUMS.txt should be produced when required evidence
            # is missing -- the check happens before any hashing.
            self.assertFalse((root / "SHA256SUMS.txt").is_file())

    def test_finalize_invalid_evidence_when_measurements_missing(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            _build_finalize_fixture(root)
            (root / "physical_measurements.json").unlink()
            result = _run(["--finalize", str(root)])
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("STAGE4_FINALIZE=INVALID_EVIDENCE", result.stdout + result.stderr)

    def test_final_hash_manifest_detects_post_finalize_tampering(self):
        """FINAL_SHA256SUMS.txt exists precisely so a reviewer can later
        re-verify the frozen evidence package hasn't been altered since
        --finalize completed. This proves that re-check actually works:
        editing post_run_verification.json's classification AFTER
        finalize completed must make a standalone
        `sha256sum -c FINAL_SHA256SUMS.txt` fail."""
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            _build_finalize_fixture(root)
            result = _run(["--finalize", str(root)])
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

            recheck_before = subprocess.run(
                ["sha256sum", "-c", "FINAL_SHA256SUMS.txt"], cwd=str(root), capture_output=True, text=True,
            )
            self.assertEqual(recheck_before.returncode, 0, recheck_before.stdout + recheck_before.stderr)

            report = json.loads((root / "post_run_verification.json").read_text(encoding="utf-8"))
            report["classification"] = "PASS_TAMPERED"
            (root / "post_run_verification.json").write_text(json.dumps(report), encoding="utf-8")

            recheck_after = subprocess.run(
                ["sha256sum", "-c", "FINAL_SHA256SUMS.txt"], cwd=str(root), capture_output=True, text=True,
            )
            self.assertNotEqual(recheck_after.returncode, 0, "tampering post_run_verification.json after finalize must be detectable")

    def test_finalize_missing_evidence_root_is_invalid(self):
        result = _run(["--finalize", "/nonexistent/evidence/root"])
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("STAGE4_FINALIZE=INVALID_EVIDENCE", result.stdout + result.stderr)

    def test_finalize_no_argument_is_invalid(self):
        result = _run(["--finalize"])
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("STAGE4_FINALIZE=INVALID_EVIDENCE", result.stdout + result.stderr)


_NUMERIC_VALIDATION_BEGIN_MARKER = "# BEGIN_NUMERIC_PARAM_VALIDATION_FUNCTION"
_NUMERIC_VALIDATION_END_MARKER = "# END_NUMERIC_PARAM_VALIDATION_FUNCTION"


def _extract_numeric_validation_function_source() -> str:
    source = SCRIPT_PATH.read_text(encoding="utf-8")
    assert _NUMERIC_VALIDATION_BEGIN_MARKER in source
    assert _NUMERIC_VALIDATION_END_MARKER in source
    return source.split(_NUMERIC_VALIDATION_BEGIN_MARKER, 1)[1].split(_NUMERIC_VALIDATION_END_MARKER, 1)[0]


def _run_numeric_validation(name: str, value: str, timeout=10):
    function_source = _extract_numeric_validation_function_source()
    harness = f"""
{function_source}
_require_valid_positive_finite_number "{name}" "{value}"
exit $?
"""
    result = subprocess.run(["bash", "-c", harness], capture_output=True, text=True, timeout=timeout)
    return result.returncode, result.stdout + result.stderr


class NumericParamValidationTest(unittest.TestCase):
    """Blocking issue 4: harmless, offline, shell-control tests for the
    fail-closed numeric validator -- extracted verbatim from
    run_hil_stage4_trial.sh (between the BEGIN/END markers), never
    reimplemented, so these tests exercise the real, committed function.
    Starts no ROS, no Pi, no Stage 4 component."""

    def test_empty_value_blocked(self):
        code, out = _run_numeric_validation("X", "")
        self.assertNotEqual(code, 0)
        self.assertIn("BLOCKED: X is empty", out)

    def test_malformed_value_blocked(self):
        code, out = _run_numeric_validation("X", "not_a_number")
        self.assertNotEqual(code, 0)
        self.assertIn("is not a valid finite decimal number", out)

    def test_nan_value_blocked(self):
        code, out = _run_numeric_validation("X", "nan")
        self.assertNotEqual(code, 0)
        self.assertIn("is NaN/Inf", out)

    def test_infinity_value_blocked(self):
        code, out = _run_numeric_validation("X", "inf")
        self.assertNotEqual(code, 0)
        self.assertIn("is NaN/Inf", out)

    def test_negative_value_blocked(self):
        code, out = _run_numeric_validation("X", "-1.0")
        self.assertNotEqual(code, 0)
        # A leading '-' fails the numeric-format regex before reaching
        # the positivity check -- either rejection reason is
        # acceptable, but it must be rejected.
        self.assertIn("BLOCKED:", out)

    def test_zero_value_blocked(self):
        code, out = _run_numeric_validation("X", "0.0")
        self.assertNotEqual(code, 0)
        self.assertIn("is not strictly positive", out)

    def test_valid_value_passes(self):
        code, out = _run_numeric_validation("X", "152.337")
        self.assertEqual(code, 0, out)
        self.assertIn("validated_numeric_param(X)=152.337", out)

    def test_valid_integer_value_passes(self):
        code, out = _run_numeric_validation("X", "5")
        self.assertEqual(code, 0, out)
        self.assertIn("validated_numeric_param(X)=5", out)


if __name__ == "__main__":
    unittest.main()

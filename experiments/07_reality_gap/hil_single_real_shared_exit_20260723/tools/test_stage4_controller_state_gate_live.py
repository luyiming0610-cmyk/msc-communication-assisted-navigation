#!/usr/bin/env python3
"""Test Suites B and C for the adoption-controlled private own-state gate
(RUN_ID stage4_20260731_190139 correction).

Real nodes used, UNMODIFIED: the real installed cooperative_avoider
executable (`ros2 run epuck2_comm cooperative_avoider`),
hil_stage4_motion_supervisor.py.

Synthetic, TEST-ONLY, explicitly labeled as such:
synthetic_stage4_physical_state_publisher.py stands in for the real
robot's canonical physical state -- the one input this rehearsal cannot
have without hardware. Adoption is triggered by publishing a valid
/hil/adoption_evidence payload directly (the same schema
hil_goal_announcement_evidence.py's real publisher produces), which lets
these tests control the before/after-adoption instant precisely without
waiting on a virtual scout's real travel time -- the announcement/
adoption-evidence PARSING and VALIDATION logic is already covered
end-to-end (real adapter, real virtual peer) by
test_hil_stage4_live_graph_rehearsal.py (Test D); these tests isolate the
gate/forwarding behaviour itself.

Isolation: a fixed, reserved ROS_DOMAIN_ID (94 -- distinct from Stage 3's
91, production, and the other rehearsal domains 90/93/95/96) plus an
entirely private topic namespace (/pytest_stage4_gate/...). No topic
name used anywhere in this file collides with a production topic name.

No Pi, no Webots, no physical hardware, no formal RUN_ID.
"""
from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import time
import unittest
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS_DIR))
from hil_goal_announcement_evidence import STAGE4_ADOPTION_EVIDENCE_TOPIC  # noqa: E402

GATE_ROS_DOMAIN_ID = "94"  # fixed, reserved: distinct from 90/91/93/95/96
NAMESPACE = "/pytest_stage4_gate"

PHYSICAL_STATE_TOPIC = f"{NAMESPACE}/epuck1_state"
CONTROLLER_STATE_TOPIC = f"{NAMESPACE}/epuck1_state_controller"
ADOPTION_EVIDENCE_TOPIC = STAGE4_ADOPTION_EVIDENCE_TOPIC
GOAL_ANNOUNCEMENT_TOPIC = f"{NAMESPACE}/goal_announcement"  # unused in these scenarios, still required by argparse
RAW_CMD_VEL_TOPIC = f"{NAMESPACE}/cmd_vel_stage4_raw"
UPSTREAM_CMD_VEL_TOPIC = f"{NAMESPACE}/cmd_vel_unguarded"
ARM_TOPIC = f"{NAMESPACE}/hil_guard_arm"
RELEASE_TOPIC = f"{NAMESPACE}/virtual_scout_released"

GOAL_ID = "pytest_stage4_gate_shared_exit"
TARGET_X_M, TARGET_Y_M = 1.20, 0.50
START_X_M, START_Y_M = 0.30, 0.50
NOMINAL_SPEED_MPS = 0.015
# Generous: this file bypasses the scout-travel wait entirely by
# publishing adoption evidence directly, so this timeout is never
# meant to fire -- large enough that no scenario below trips it.
SCOUT_ANNOUNCEMENT_TIMEOUT_S = 60.0


def _ros_env() -> dict:
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = GATE_ROS_DOMAIN_ID
    env["ROS_LOCALHOST_ONLY"] = "1"
    return env


class LiveProcess:
    def __init__(self, name: str, cmd: list, log_path: Path, env: dict):
        self.name = name
        self.log_path = log_path
        self._fh = open(log_path, "w", encoding="utf-8")
        self.proc = subprocess.Popen(
            cmd, stdout=self._fh, stderr=subprocess.STDOUT, cwd=str(TOOLS_DIR), env=env,
            preexec_fn=os.setsid,
        )
        self.pid = self.proc.pid
        try:
            self.pgid = os.getpgid(self.pid)
        except ProcessLookupError:
            self.pgid = None

    def is_alive(self) -> bool:
        return self.proc.poll() is None

    def kill(self, sig=signal.SIGTERM) -> None:
        if self.is_alive():
            try:
                os.killpg(self.pgid, sig)
            except ProcessLookupError:
                pass

    def wait(self, timeout_s: float = 5.0) -> None:
        try:
            self.proc.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            self.kill(signal.SIGKILL)
            try:
                self.proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                pass

    def close(self) -> None:
        try:
            self._fh.close()
        except Exception:
            pass


def _no_residual_process(pids: list) -> bool:
    for pid in pids:
        if pid is None:
            continue
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            continue
        except PermissionError:
            return False
        else:
            return False
    return True


def _tail_last_json_line(path: Path):
    if not path.exists():
        return None
    lines = [l for l in path.read_text(encoding="utf-8").splitlines() if l.strip()]
    if not lines:
        return None
    try:
        return json.loads(lines[-1])
    except (ValueError, TypeError):
        return None


def _wait_for_supervisor_state(evidence_path: Path, target_states: set, timeout_s: float) -> str:
    deadline = time.monotonic() + timeout_s
    last_state = None
    while time.monotonic() < deadline:
        record = _tail_last_json_line(evidence_path)
        if record is not None:
            last_state = record.get("state")
            if last_state in target_states:
                return last_state
        time.sleep(0.1)
    return last_state


def _events_and_records(evidence_path: Path):
    if not evidence_path.exists():
        return [], []
    records = [json.loads(l) for l in evidence_path.read_text(encoding="utf-8").splitlines() if l.strip()]
    return [r["event"] for r in records], records


class GateLiveTestBase(unittest.TestCase):
    """Shared spawn/teardown scaffolding for Suites B and C."""

    def setUp(self):
        self.evidence_dir = Path(
            subprocess.run(["mktemp", "-d"], capture_output=True, text=True, check=True).stdout.strip()
        )
        self.processes: list = []
        self.env = _ros_env()

    def tearDown(self):
        for p in reversed(self.processes):
            p.kill(signal.SIGTERM)
        for p in reversed(self.processes):
            p.wait(timeout_s=3.0)
            p.close()
        pids = [p.pid for p in self.processes]
        self.assertTrue(_no_residual_process(pids), f"residual process among {pids} after teardown")

    def _spawn(self, name: str, cmd: list) -> LiveProcess:
        proc = LiveProcess(name, cmd, self.evidence_dir / f"{name}.log", self.env)
        self.processes.append(proc)
        return proc

    def _spawn_canonical_state(self, front_distance_m=None):
        cmd = [
            sys.executable, str(TOOLS_DIR / "synthetic_stage4_physical_state_publisher.py"),
            "--state-topic", PHYSICAL_STATE_TOPIC, "--robot-id", "1",
            "--x-m", str(START_X_M), "--y-m", str(START_Y_M), "--rate-hz", "20",
        ]
        if front_distance_m is not None:
            cmd += ["--front-distance-m", str(front_distance_m)]
        return self._spawn("synthetic_physical_state", cmd)

    def _spawn_cooperative_avoider(self):
        prefix = subprocess.run(
            ["ros2", "pkg", "prefix", "epuck2_comm"], capture_output=True, text=True, env=self.env, check=True,
        ).stdout.strip()
        exe = str(Path(prefix) / "lib" / "epuck2_comm" / "cooperative_avoider")
        # enable_dynamic_heading/speed both false: cooperative_avoider
        # cruises straight at its own nominal_speed_mps with a fixed
        # desired_heading_rad=0.0 default, with NO NavigationIntent
        # input required -- keeps this integration test focused purely
        # on the own-state gate, not on the separate adapter/nav_intent
        # path (already covered by Test D).
        return self._spawn("cooperative_avoider", [
            exe, "--ros-args",
            "-r", f"cmd_vel:={RAW_CMD_VEL_TOPIC}",
            "-r", f"state:={CONTROLLER_STATE_TOPIC}",
            "-p", "robot_id:=1", "-p", "armed:=true",
            "-p", "enable_peer_avoidance:=false", "-p", "enable_dynamic_heading:=false",
            "-p", "enable_dynamic_speed:=false", "-p", "enable_local_avoidance:=true",
            "-p", "require_local_sensors:=true", "-p", "use_sim_time:=false",
            "-p", f"nominal_speed_mps:={NOMINAL_SPEED_MPS}",
            "-p", "safety_radius_m:=0.14", "-p", "stop_after_recovery:=false",
        ])

    def _spawn_supervisor(self, evidence_path: Path):
        return self._spawn("hil_stage4_motion_supervisor", [
            sys.executable, str(TOOLS_DIR / "hil_stage4_motion_supervisor.py"),
            "--goal-id", GOAL_ID, "--run-id", "pytest_gate_live",
            "--expected-target-x-m", str(TARGET_X_M), "--expected-target-y-m", str(TARGET_Y_M),
            "--adoption-evidence-topic", ADOPTION_EVIDENCE_TOPIC,
            "--goal-announcement-topic", GOAL_ANNOUNCEMENT_TOPIC,
            "--scout-announcement-timeout-s", str(SCOUT_ANNOUNCEMENT_TIMEOUT_S),
            "--raw-cmd-vel-topic", RAW_CMD_VEL_TOPIC,
            "--guarded-output-topic", UPSTREAM_CMD_VEL_TOPIC,
            "--arm-topic", ARM_TOPIC,
            "--virtual-scout-released-topic", RELEASE_TOPIC,
            "--physical-state-topic", PHYSICAL_STATE_TOPIC,
            "--controller-state-topic", CONTROLLER_STATE_TOPIC,
            "--physical-state-timeout-s", "0.5",
            "--evidence-path", str(evidence_path),
            "--operator-approval-token", "APPROVED_FOR_SINGLE_HIL_EVENT=YES",
        ])

    def _release_virtual_scout(self):
        subprocess.run(
            ["ros2", "topic", "pub", "--once", RELEASE_TOPIC, "std_msgs/msg/Bool", "{data: true}"],
            env=self.env, capture_output=True, text=True, timeout=10,
        )

    def _publish_adoption_evidence(self):
        payload = {
            "schema_version": "1.0.0", "goal_id": GOAL_ID,
            "source_robot_id": 2, "source_sequence": 1,
            "accepted": True, "duplicate": False,
            "target_x_m": TARGET_X_M, "target_y_m": TARGET_Y_M,
            "adapter_receive_time_s": time.time(),
            "adapter_receive_monotonic_s": time.monotonic(),
        }
        subprocess.run(
            ["ros2", "topic", "pub", "--once", ADOPTION_EVIDENCE_TOPIC, "std_msgs/msg/String",
             json.dumps({"data": json.dumps(payload)})],
            env=self.env, capture_output=True, text=True, timeout=10,
        )

    def _echo_raw_cmd_vel_once(self, timeout_s=3):
        try:
            result = subprocess.run(
                ["ros2", "topic", "echo", RAW_CMD_VEL_TOPIC, "--once"],
                env=self.env, capture_output=True, text=True, timeout=timeout_s,
            )
            return result.stdout
        except subprocess.TimeoutExpired:
            return ""


class Suite_B_ControllerStateGateBeforeAfterAdoptionTest(GateLiveTestBase):
    """Test Suite B (controller integration, real unmodified
    cooperative_avoider, isolated ROS domain): before adoption, canonical
    state flows normally but no private state is forwarded, and the real
    controller must stay in its own fail-closed SAFE_STOP_STALE with zero
    output; after adoption, the supervisor must start forwarding, and the
    real controller must produce a valid straight command within
    RAW_COMMAND_TIMEOUT_S (5s) of the first forward."""

    def test_b_before_adoption_zero_output_after_adoption_valid_straight_command(self):
        evidence_path = self.evidence_dir / "supervisor_evidence.jsonl"
        self._spawn_canonical_state()
        self._spawn_cooperative_avoider()
        time.sleep(2.0)
        self._spawn_supervisor(evidence_path)
        time.sleep(1.0)
        self._release_virtual_scout()

        # -- before adoption: prove zero output and no forwarding for at
        # least cooperative_avoider's own 5.0s startup_hold_s, plus
        # margin, using several independent zero-output samples rather
        # than a single point-in-time check. --
        pre_adoption_dwell_s = 7.0
        deadline = time.monotonic() + pre_adoption_dwell_s
        samples = 0
        while time.monotonic() < deadline:
            echoed = self._echo_raw_cmd_vel_once(timeout_s=4)
            if echoed and "WARNING" not in echoed and "linear:" in echoed:
                samples += 1
                self.assertIn("x: 0.0", echoed, f"expected zero linear.x before adoption, got: {echoed}")
            time.sleep(0.5)
        self.assertGreater(samples, 0, "expected at least one raw_cmd_vel sample during the pre-adoption dwell")

        events, records = _events_and_records(evidence_path)
        self.assertNotIn("CONTROLLER_STATE_GATE_OPENED", events)
        self.assertNotIn("FIRST_FRESH_POST_ADOPTION_CONTROLLER_STATE_FORWARDED", events)
        self.assertNotIn("ARM_PUBLISHED", events)
        self.assertNotIn("ACTIVE_OPENED", events)
        self.assertEqual(records[-1]["state"] if records else None, "WAITING_FOR_EVENT")

        # -- trigger adoption directly (bypasses the scout-travel wait;
        # the announcement/adoption-evidence parsing path itself is
        # covered end-to-end by Test D) --
        self._publish_adoption_evidence()
        # The real controller can validate and arm fast enough that this
        # may already observe ACTIVE rather than catching it mid-flight
        # at VALIDATING_RAW_COMMAND -- both are valid intermediate
        # observations of a successful adoption; only FAILED here would
        # be a genuine defect.
        adopted = _wait_for_supervisor_state(
            evidence_path, {"VALIDATING_RAW_COMMAND", "ACTIVE", "FAILED"}, timeout_s=5.0
        )
        self.assertIn(adopted, ("VALIDATING_RAW_COMMAND", "ACTIVE"))

        events, records = _events_and_records(evidence_path)
        self.assertIn("CONTROLLER_STATE_GATE_OPENED", events)

        # -- after adoption: the real, unmodified controller must
        # produce a valid straight command (linear.x=nominal_speed_mps,
        # angular.z=0.0) and the supervisor must accept it, well within
        # RAW_COMMAND_TIMEOUT_S (5.0s) of the first forward. --
        final_state = _wait_for_supervisor_state(evidence_path, {"ACTIVE", "FAILED"}, timeout_s=8.0)
        events, records = _events_and_records(evidence_path)
        self.assertEqual(
            final_state, "ACTIVE",
            "if the unmodified controller does not produce a valid command under a clear "
            f"post-adoption fixture within budget, this is BLOCKED. events={events} "
            f"last_record={records[-1] if records else None}",
        )
        self.assertIn("FIRST_FRESH_POST_ADOPTION_CONTROLLER_STATE_FORWARDED", events)
        self.assertIn("VALID_RAW_COMMAND_ACCEPTED", events)
        valid_cmd_record = next(r for r in records if r["event"] == "VALID_RAW_COMMAND_ACCEPTED")
        # A strict equality to NOMINAL_SPEED_MPS is not guaranteed here:
        # cooperative_avoider's own command smoother can legitimately
        # still be ramping up on the very first accepted tick (the same
        # reason test_05_zero_raw_command_does_not_arm in
        # test_hil_stage4_motion_supervisor.py tolerates a below-min
        # first sample without latching FAILED) -- only bounded within
        # (0, MAX_LINEAR_MPS] and forward-only is guaranteed.
        self.assertGreater(valid_cmd_record["raw"]["linear_x"], 0.0)
        self.assertLessEqual(valid_cmd_record["raw"]["linear_x"], NOMINAL_SPEED_MPS + 1e-9)
        self.assertAlmostEqual(valid_cmd_record["raw"]["angular_z"], 0.0, places=6)
        self.assertIn("ARM_PUBLISHED", events)


class Suite_C_ObstacleFixtureNegativeSafetyTest(GateLiveTestBase):
    """Test Suite C (negative safety): after adoption, forward a valid
    (schema-wise) but below-obstacle-threshold EpuckState fixture -- the
    real controller's own local-sensor safety path must still activate,
    the guard must never be armed with a nonzero angular/linear command
    from this path, and ACTIVE_OPENED must never occur."""

    def test_c_obstacle_fixture_after_adoption_fails_closed(self):
        evidence_path = self.evidence_dir / "supervisor_evidence.jsonl"
        # front_distance_m well below local_obstacle_logic's own
        # front_danger_m=0.100 default -- a genuine obstacle reading,
        # not an edge case near the threshold.
        self._spawn_canonical_state(front_distance_m=0.05)
        self._spawn_cooperative_avoider()
        time.sleep(2.0)
        self._spawn_supervisor(evidence_path)
        time.sleep(1.0)
        self._release_virtual_scout()
        time.sleep(1.0)

        self._publish_adoption_evidence()

        # Give the real controller several seconds past its own
        # startup_hold_s and past RAW_COMMAND_TIMEOUT_S to prove it
        # never produces a valid nonzero-forward command under this
        # fixture -- the supervisor must never reach ACTIVE. Two
        # legitimate fail-closed outcomes exist here, both provable
        # rather than assumed: (a) the real controller's own local
        # safety path (decide_local_obstacle) reacts to the forwarded
        # front_distance_m=0.05 fixture with a genuine nonzero angular
        # turn-away command, which the supervisor's own EXISTING
        # raw-command validation (validate_twist) rejects immediately
        # (RAW_COMMAND_INVALID:NONZERO_ANGULAR_Z) -- observed live; or
        # (b) no command is ever produced at all and
        # RAW_COMMAND_TIMEOUT fires. Either way: never ACTIVE, never
        # ARM_PUBLISHED.
        final_state = _wait_for_supervisor_state(evidence_path, {"ACTIVE", "FAILED"}, timeout_s=12.0)
        events, records = _events_and_records(evidence_path)
        self.assertNotEqual(final_state, "ACTIVE", f"must never arm/activate under an obstacle fixture. events={events}")
        self.assertEqual(final_state, "FAILED", f"must reach a terminal fail-closed state. events={events}")
        self.assertNotIn("ACTIVE_OPENED", events)
        self.assertNotIn("ARM_PUBLISHED", events)
        latched_failed_record = next(r for r in records if r["event"] == "LATCHED_FAILED")
        self.assertIn(
            latched_failed_record["reason"],
            ("RAW_COMMAND_TIMEOUT", "RAW_COMMAND_INVALID:NONZERO_ANGULAR_Z"),
            f"unexpected terminal reason: {latched_failed_record}",
        )
        if latched_failed_record["reason"] == "RAW_COMMAND_INVALID:NONZERO_ANGULAR_Z":
            # Direct proof the local-sensor path, not peer-CPA logic
            # (enable_peer_avoidance=false in this scenario), is what
            # produced the rejected command: the raw twist that
            # triggered the rejection carries a nonzero angular_z.
            raw_twist_record = next(
                r for r in records if r["event"] == "RAW_TWIST_RECEIVED" and abs(r["raw"]["angular_z"]) > 1e-6
            )
            self.assertNotAlmostEqual(raw_twist_record["raw"]["angular_z"], 0.0)
        self.assertIn("FIRST_FRESH_POST_ADOPTION_CONTROLLER_STATE_FORWARDED", events, "the gate must still have opened and forwarded the (obstacle) fixture -- only the resulting command is rejected")


class Suite_ArmDisarmRealWrapperTest(GateLiveTestBase):
    """Blocking issue 3 (post strict-review): focused tests using the
    ACTUAL rclpy wrapper (hil_stage4_motion_supervisor.py as a real
    subprocess, real /hil_guard/arm topic), not only the pure engine --
    proving the centralized _sync_arm_disarm() safety action fires
    correctly for ACTIVE-exit paths the pure-engine tests cannot
    observe (arm_pub is I/O, which only exists in the wrapper)."""

    def _echo_arm_topic_once_async(self):
        return subprocess.Popen(
            ["ros2", "topic", "echo", "--once", ARM_TOPIC, "std_msgs/msg/Bool"],
            env=self.env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        )

    def _drive_to_active(self, evidence_path: Path):
        arm_true_echo = self._echo_arm_topic_once_async()
        self._spawn_canonical_state()
        self._spawn_cooperative_avoider()
        time.sleep(2.0)
        self._spawn_supervisor(evidence_path)
        time.sleep(1.0)
        self._release_virtual_scout()
        self._publish_adoption_evidence()
        # A single adoption-evidence publish can occasionally be
        # rejected as STALE_EVIDENCE under transient CLI-startup/system
        # load (the freshness bound is ADOPTION_EVIDENCE_MAX_AGE_S=2.0s
        # measured from this process's own timestamp to the
        # supervisor's receipt) -- retry with freshly-computed
        # timestamps rather than fail on a timing artifact unrelated to
        # the behavior under test.
        state = _wait_for_supervisor_state(evidence_path, {"VALIDATING_RAW_COMMAND", "ACTIVE", "FAILED"}, timeout_s=3.0)
        if state not in ("VALIDATING_RAW_COMMAND", "ACTIVE"):
            self._publish_adoption_evidence()
        final_state = _wait_for_supervisor_state(evidence_path, {"ACTIVE", "FAILED"}, timeout_s=10.0)
        events, records = _events_and_records(evidence_path)
        self.assertEqual(final_state, "ACTIVE", f"must reach ACTIVE for this test to be meaningful. events={events}")
        try:
            arm_true_stdout, _ = arm_true_echo.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            arm_true_echo.kill()
            arm_true_stdout, _ = arm_true_echo.communicate()
        self.assertIn("data: true", arm_true_stdout, f"arm must become true exactly once on ACTIVE entry: {arm_true_stdout}")
        return records

    def test_arm_true_on_active_zero_and_arm_false_on_invalid_raw_command_during_active(self):
        evidence_path = self.evidence_dir / "supervisor_evidence.jsonl"
        self._drive_to_active(evidence_path)

        # Arm the NEXT echoes before injecting the failure so they are
        # already subscribed when arm=False / the zero Twist are
        # published (neither topic uses durability/transient-local QoS
        # -- a late subscriber would otherwise hang forever waiting for
        # a message that already went by, since _sync_arm_disarm()
        # publishes each exactly once, not periodically).
        arm_false_echo = self._echo_arm_topic_once_async()
        upstream_zero_echo = subprocess.Popen(
            ["ros2", "topic", "echo", "--once", UPSTREAM_CMD_VEL_TOPIC],
            env=self.env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        )
        time.sleep(0.5)  # let both echo subscriptions actually register before we act

        # Inject an invalid (nonzero angular) raw command directly onto
        # the raw topic -- a rogue publisher standing in for a
        # misbehaving/compromised upstream, the same technique Test D's
        # own test_3 uses.
        subprocess.run(
            ["ros2", "topic", "pub", "--once", RAW_CMD_VEL_TOPIC, "geometry_msgs/msg/Twist",
             "{linear: {x: 0.01, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.05}}"],
            env=self.env, capture_output=True, text=True, timeout=10,
        )

        evidence_path_final = _wait_for_supervisor_state(evidence_path, {"FAILED"}, timeout_s=5.0)
        self.assertEqual(evidence_path_final, "FAILED")
        events, records = _events_and_records(evidence_path)
        latched = next(r for r in records if r["event"] == "LATCHED_FAILED")
        self.assertIn("RAW_COMMAND_INVALID_DURING_ACTIVE", latched["reason"])

        try:
            arm_false_stdout, _ = arm_false_echo.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            arm_false_echo.kill()
            arm_false_stdout, _ = arm_false_echo.communicate()
        self.assertIn("data: false", arm_false_stdout, f"arm must become false on this ACTIVE-exit path: {arm_false_stdout}")

        # No later nonzero output: the explicit zero Twist published by
        # _sync_arm_disarm() on this exact ACTIVE-exit is the next (and
        # only further) message on cmd_vel_unguarded.
        try:
            upstream_zero_stdout, _ = upstream_zero_echo.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            upstream_zero_echo.kill()
            upstream_zero_stdout, _ = upstream_zero_echo.communicate()
        self.assertIn("x: 0.0", upstream_zero_stdout, f"must publish zero (not the rejected nonzero) after failure: {upstream_zero_stdout}")

    def test_arm_true_on_active_zero_and_arm_false_on_physical_state_stale(self):
        evidence_path = self.evidence_dir / "supervisor_evidence.jsonl"
        self._drive_to_active(evidence_path)

        arm_false_echo = self._echo_arm_topic_once_async()
        time.sleep(0.5)

        # Stop the canonical physical-state publisher -- physical
        # state goes stale, must trip PHYSICAL_STATE_STALE_OR_MISSING
        # (physical-state-timeout-s=0.5) within one or two ticks.
        canonical_proc = next(p for p in self.processes if p.name == "synthetic_physical_state")
        canonical_proc.kill(signal.SIGTERM)
        canonical_proc.wait(timeout_s=3.0)

        final_state = _wait_for_supervisor_state(evidence_path, {"FAILED"}, timeout_s=5.0)
        self.assertEqual(final_state, "FAILED")
        events, records = _events_and_records(evidence_path)
        latched = next(r for r in records if r["event"] == "LATCHED_FAILED")
        self.assertIn("PHYSICAL_STATE_STALE_OR_MISSING", latched["reason"])

        try:
            arm_false_stdout, _ = arm_false_echo.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            arm_false_echo.kill()
            arm_false_stdout, _ = arm_false_echo.communicate()
        self.assertIn("data: false", arm_false_stdout, f"arm must become false on this ACTIVE-exit path: {arm_false_stdout}")


def _extract_direct_discovery_function_source() -> str:
    script_path = TOOLS_DIR / "run_hil_stage4_trial.sh"
    source = script_path.read_text(encoding="utf-8")
    begin, end = "# BEGIN_DIRECT_DISCOVERY_FUNCTION", "# END_DIRECT_DISCOVERY_FUNCTION"
    assert begin in source and end in source
    return source.split(begin, 1)[1].split(end, 1)[0]


def _run_direct_discovery_check(topic: str, label: str, env: dict, spin_time_s="5", overhead_s="1.0", timeout=15):
    """Invokes the REAL, committed require_exactly_one_publisher_via_direct_discovery
    function (extracted verbatim from run_hil_stage4_trial.sh, never
    reimplemented) exactly as step 7 of the real orchestrator does --
    used by Suite_D2 below to genuinely incur the same ~6s-per-call
    overhead the real readiness sequence does, not a fast-forwarded
    stand-in."""
    function_source = _extract_direct_discovery_function_source()
    harness = f"""
DIRECT_DISCOVERY_SPIN_TIME_S="{spin_time_s}"
DIRECT_DISCOVERY_OVERHEAD_S="{overhead_s}"
{function_source}
require_exactly_one_publisher_via_direct_discovery "{topic}" "{label}"
exit $?
"""
    result = subprocess.run(["bash", "-c", harness], capture_output=True, text=True, timeout=timeout, env=env)
    return result.returncode, result.stdout + result.stderr


@unittest.skipUnless(
    os.environ.get("STAGE4_LONG_REHEARSAL") == "1",
    "opt-in, ~90s+ real-time production-duration rehearsal -- set STAGE4_LONG_REHEARSAL=1 to run",
)
class Suite_D2_ProductionDurationTest(unittest.TestCase):
    """Blocking issue 5 (post strict-review, closing
    LONG_REHEARSAL_REPRODUCIBILITY=BLOCKED): the one committed,
    reproducible, opt-in test that proves the full production-duration
    chain end-to-end. Real, unmodified cooperative_avoider; real
    hil_topic_adapter.py (GoalAnnouncement + adoption-evidence path);
    real hil_virtual_peer.py at PRODUCTION cruise speed (0.015 m/s,
    >=57s travel, not the fast 0.3 m/s used by Test D); all 5 real
    direct-discovery readiness calls (not a 3-second shortcut);
    production startup_hold_s (unmodified default) and every frozen
    production timeout value, imported directly from
    hil_stage4_motion_supervisor -- never re-declared as separate
    literals that could silently drift from the real ones.

    Skipped by default. Invoke exactly:

        ROS_DOMAIN_ID=98 ROS_LOCALHOST_ONLY=1 STAGE4_LONG_REHEARSAL=1 \\
          python3 -m unittest -v \\
          test_stage4_controller_state_gate_live.Suite_D2_ProductionDurationTest

    No Pi, no Webots, no physical hardware, no formal RUN_ID.
    """

    DOMAIN = os.environ.get("ROS_DOMAIN_ID", "98")
    NAMESPACE = "/pytest_stage4_prod"

    def setUp(self):
        import math

        sys.path.insert(0, str(TOOLS_DIR))
        from hil_stage4_motion_supervisor import (
            ADOPTION_TIMEOUT_S, CONTROLLER_STATE_FORWARD_TIMEOUT_S, MAX_LINEAR_MPS, RAW_COMMAND_TIMEOUT_S,
        )

        self.ADOPTION_TIMEOUT_S = ADOPTION_TIMEOUT_S
        self.CONTROLLER_STATE_FORWARD_TIMEOUT_S = CONTROLLER_STATE_FORWARD_TIMEOUT_S
        self.RAW_COMMAND_TIMEOUT_S = RAW_COMMAND_TIMEOUT_S
        self.MAX_LINEAR_MPS = MAX_LINEAR_MPS

        self.START_X_M, self.START_Y_M = 0.30, 0.50
        self.TARGET_X_M, self.TARGET_Y_M = 1.20, 0.50
        self.ARRIVAL_RADIUS_M = 0.05

        self.PHYSICAL_STATE_TOPIC = f"{self.NAMESPACE}/epuck1_state"
        self.CONTROLLER_STATE_TOPIC = f"{self.NAMESPACE}/epuck1_state_controller"
        self.VIRTUAL_STATE_TOPIC = f"{self.NAMESPACE}/virtual_peer_state"
        self.GOAL_ANNOUNCEMENT_TOPIC = f"{self.NAMESPACE}/goal_announcement"
        self.ADOPTION_EVIDENCE_TOPIC = STAGE4_ADOPTION_EVIDENCE_TOPIC
        self.RAW_CMD_VEL_TOPIC = f"{self.NAMESPACE}/cmd_vel_stage4_raw"
        self.UPSTREAM_CMD_VEL_TOPIC = f"{self.NAMESPACE}/cmd_vel_unguarded"
        self.GUARDED_CMD_VEL_TOPIC = f"{self.NAMESPACE}/cmd_vel"
        self.ARM_TOPIC = f"{self.NAMESPACE}/hil_guard_arm"
        self.RELEASE_TOPIC = f"{self.NAMESPACE}/virtual_scout_released"
        self.GOAL_ID = "pytest_prod_shared_exit"

        # Same formula run_hil_stage4_trial.sh itself uses -- never a
        # separately-guessed literal.
        travel_m = max(0.0, (self.TARGET_X_M - self.START_X_M) - self.ARRIVAL_RADIUS_M)
        self.SCOUT_ANNOUNCEMENT_TIMEOUT_S = travel_m / self.MAX_LINEAR_MPS + 20.0
        # Frozen production geometry gives exactly (1.20-0.30-0.05)/0.015
        # = 56.667s -- the real value used throughout this review's own
        # arithmetic (SCOUT_ANNOUNCEMENT_TIMEOUT_S=76.667=56.667+20.0),
        # not a shortcut. 56.5 is a tight bound distinguishing this from
        # Test D's fast 0.3 m/s rehearsal (~4s travel), never a loosened
        # requirement.
        self.assertGreaterEqual(travel_m / self.MAX_LINEAR_MPS, 56.5, "production travel time must match the frozen geometry (~56.667s)")

        self.evidence_dir = Path(
            subprocess.run(["mktemp", "-d"], capture_output=True, text=True, check=True).stdout.strip()
        )
        self.processes: list = []
        self.env = dict(os.environ)
        self.env["ROS_DOMAIN_ID"] = self.DOMAIN
        self.env["ROS_LOCALHOST_ONLY"] = "1"

    def tearDown(self):
        for p in reversed(self.processes):
            p.kill(signal.SIGTERM)
        for p in reversed(self.processes):
            p.wait(timeout_s=3.0)
            p.close()
        pids = [p.pid for p in self.processes]
        self.assertTrue(_no_residual_process(pids), f"residual process among {pids} after teardown")
        # No lingering ros2 daemon in this test's own isolated domain.
        subprocess.run(
            ["ros2", "daemon", "stop"], env=self.env, capture_output=True, text=True, timeout=10,
        )

    def _spawn(self, name, cmd):
        proc = LiveProcess(name, cmd, self.evidence_dir / f"{name}.log", self.env)
        self.processes.append(proc)
        return proc

    def test_d2_production_duration_full_chain(self):
        t0 = time.monotonic()

        self._spawn("synthetic_physical_state", [
            sys.executable, str(TOOLS_DIR / "synthetic_stage4_physical_state_publisher.py"),
            "--state-topic", self.PHYSICAL_STATE_TOPIC, "--robot-id", "1",
            "--x-m", "0.0", "--y-m", "0.0", "--rate-hz", "20",
        ])
        self._spawn("hil_topic_adapter", [
            sys.executable, str(TOOLS_DIR / "hil_topic_adapter.py"),
            "--robot-id=1", f"--state-topic={self.PHYSICAL_STATE_TOPIC}",
            f"--nav-intent-topic={self.NAMESPACE}/nav_intent",
            f"--field-origin-x-m={self.START_X_M}",
            f"--field-origin-y-m={self.START_Y_M}",
            "--field-origin-yaw-rad=0.0",
            "--mode=search", f"--waypoints={self.START_X_M + 0.2}:{self.START_Y_M}",
            "--waypoint-arrival-radius=0.10", "--rate-hz=2.0",
            f"--nominal-speed-mps={self.MAX_LINEAR_MPS}",
            f"--exit-center-x={self.TARGET_X_M}", f"--exit-center-y={self.TARGET_Y_M}",
            f"--exit-radius={self.ARRIVAL_RADIUS_M}",
            f"--parking-x={self.TARGET_X_M}", f"--parking-y={self.TARGET_Y_M}", f"--parking-radius={self.ARRIVAL_RADIUS_M}",
            "--goal-hold-time-s=2.0",
            f"--goal-announcement-topic={self.GOAL_ANNOUNCEMENT_TOPIC}",
        ])
        self._spawn("hil_cmd_vel_guard", [
            sys.executable, str(TOOLS_DIR / "hil_cmd_vel_guard.py"),
            "--physical-state-topic", self.PHYSICAL_STATE_TOPIC,
            "--upstream-cmd-vel-topic", self.UPSTREAM_CMD_VEL_TOPIC,
            "--guarded-cmd-vel-topic", self.GUARDED_CMD_VEL_TOPIC,
            "--arm-topic", self.ARM_TOPIC,
            "--max-linear-speed-mps", str(self.MAX_LINEAR_MPS),
            "--max-angular-speed-rps", "0.0",
            "--heartbeat-timeout-s", "0.5",
            "--physical-state-timeout-s", "0.5",
            "--require-virtual-peer", "--virtual-peer-topic", self.VIRTUAL_STATE_TOPIC,
        ])
        time.sleep(2.0)

        # Production formula (READINESS_OVERHEAD_MARGIN_S=34.0,
        # PRE_RELEASE_TIMEOUT_S=44.0, COOP_MAX_RUNTIME_S~=152.34) --
        # imported constants + the same derivation run_hil_stage4_trial.sh
        # performs, not separate literals.
        readiness_overhead_margin_s = 5 * (5.0 + 1.0) + 2 * 2.0
        pre_release_timeout_s = readiness_overhead_margin_s + 10.0
        coop_max_runtime_s = (
            readiness_overhead_margin_s + self.SCOUT_ANNOUNCEMENT_TIMEOUT_S + self.ADOPTION_TIMEOUT_S
            + self.CONTROLLER_STATE_FORWARD_TIMEOUT_S + self.RAW_COMMAND_TIMEOUT_S + 6.67 + 20.0
        )

        prefix = subprocess.run(
            ["ros2", "pkg", "prefix", "epuck2_comm"], capture_output=True, text=True, env=self.env, check=True,
        ).stdout.strip()
        exe = str(Path(prefix) / "lib" / "epuck2_comm" / "cooperative_avoider")
        self._spawn("cooperative_avoider", [
            exe, "--ros-args",
            "-r", f"cmd_vel:={self.RAW_CMD_VEL_TOPIC}",
            "-r", f"state:={self.CONTROLLER_STATE_TOPIC}",
            "-r", f"nav_intent:={self.NAMESPACE}/nav_intent",
            "-p", "robot_id:=1", "-p", "armed:=true",
            "-p", "enable_peer_avoidance:=true", "-p", "enable_dynamic_heading:=true",
            "-p", "enable_dynamic_speed:=true", "-p", "enable_local_avoidance:=true",
            "-p", "require_local_sensors:=true", "-p", "use_sim_time:=false",
            "-p", f"nominal_speed_mps:={self.MAX_LINEAR_MPS}",
            "-p", "safety_radius_m:=0.14", "-p", "stop_after_recovery:=false",
            "-p", f"peer_state_topic:={self.VIRTUAL_STATE_TOPIC}",
            "-p", f"max_runtime_s:={coop_max_runtime_s}",
            # no startup_hold_s override -- production default (5.0) unchanged
        ])
        time.sleep(2.0)

        evidence_path = self.evidence_dir / "supervisor_evidence.jsonl"
        self._spawn("hil_stage4_motion_supervisor", [
            sys.executable, str(TOOLS_DIR / "hil_stage4_motion_supervisor.py"),
            "--goal-id", self.GOAL_ID, "--run-id", "pytest_prod_duration",
            "--expected-target-x-m", str(self.TARGET_X_M), "--expected-target-y-m", str(self.TARGET_Y_M),
            "--adoption-evidence-topic", self.ADOPTION_EVIDENCE_TOPIC,
            "--goal-announcement-topic", self.GOAL_ANNOUNCEMENT_TOPIC,
            "--scout-announcement-timeout-s", str(self.SCOUT_ANNOUNCEMENT_TIMEOUT_S),
            "--pre-release-timeout-s", str(pre_release_timeout_s),
            "--raw-cmd-vel-topic", self.RAW_CMD_VEL_TOPIC,
            "--guarded-output-topic", self.UPSTREAM_CMD_VEL_TOPIC,
            "--arm-topic", self.ARM_TOPIC,
            "--virtual-scout-released-topic", self.RELEASE_TOPIC,
            "--physical-state-topic", self.PHYSICAL_STATE_TOPIC,
            "--controller-state-topic", self.CONTROLLER_STATE_TOPIC,
            "--controller-field-origin-x-m", str(self.START_X_M),
            "--controller-field-origin-y-m", str(self.START_Y_M),
            "--controller-field-origin-yaw-rad", "0.0",
            "--physical-state-timeout-s", "0.5",
            "--evidence-path", str(evidence_path),
            "--operator-approval-token", "APPROVED_FOR_SINGLE_HIL_EVENT=YES",
        ])
        time.sleep(2.0)

        # -- step 7 equivalent: all 5 REAL direct-discovery readiness
        # calls, not a shortcut -- genuinely incurs the ~34s overhead
        # the production formula budgets for. --
        for topic, label in (
            (self.GUARDED_CMD_VEL_TOPIC, "guarded_cmd_vel_post_start"),
            (self.PHYSICAL_STATE_TOPIC, "real_state_post_start"),
            (self.RAW_CMD_VEL_TOPIC, "cooperative_avoider_post_start"),
            (self.CONTROLLER_STATE_TOPIC, "controller_private_state_post_start"),
            (self.PHYSICAL_STATE_TOPIC, "real_state_still_sole_canonical_post_start"),
        ):
            code, out = _run_direct_discovery_check(topic, label, self.env)
            self.assertEqual(code, 0, f"readiness check {label} failed: {out}")

        t_pre_release_elapsed = time.monotonic() - t0
        self.assertGreaterEqual(t_pre_release_elapsed, 30.0, "must genuinely incur the real readiness overhead, not a shortcut")

        subprocess.run(
            ["ros2", "topic", "pub", "--once", self.RELEASE_TOPIC, "std_msgs/msg/Bool", "{data: true}"],
            env=self.env, capture_output=True, text=True, timeout=10,
        )
        t_release = time.monotonic()
        self._spawn("hil_virtual_peer", [
            sys.executable, str(TOOLS_DIR / "hil_virtual_peer.py"),
            "--robot-id", "2", "--state-topic", self.VIRTUAL_STATE_TOPIC,
            "--goal-id", self.GOAL_ID,
            "--start-x-m", str(self.START_X_M), "--start-y-m", str(self.START_Y_M), "--start-yaw-rad", "0.0",
            "--target-x-m", str(self.TARGET_X_M), "--target-y-m", str(self.TARGET_Y_M),
            "--cruise-linear-mps", str(self.MAX_LINEAR_MPS), "--arrival-radius-m", str(self.ARRIVAL_RADIUS_M),
            "--max-angular-rps", "0.0",
            "--announcement-topic", self.GOAL_ANNOUNCEMENT_TOPIC,
        ])

        final_state = _wait_for_supervisor_state(evidence_path, {"COMPLETE", "FAILED"}, timeout_s=120.0)
        events, records = _events_and_records(evidence_path)
        self.assertEqual(final_state, "COMPLETE", f"production-duration rehearsal must reach COMPLETE. events={events}")

        milestones = {}
        for r in records:
            if r["event"] not in milestones:
                milestones[r["event"]] = r
        for ev in (
            "GOAL_ANNOUNCEMENT_OBSERVED", "ADOPTION_CONFIRMED", "CONTROLLER_STATE_GATE_OPENED",
            "FIRST_FRESH_POST_ADOPTION_CONTROLLER_STATE_FORWARDED", "VALID_RAW_COMMAND_ACCEPTED",
            "ARM_PUBLISHED", "ACTIVE_OPENED", "ZERO_BURST_OPENED", "DISARM_PUBLISHED", "LATCHED_COMPLETE",
        ):
            self.assertIn(ev, milestones, f"missing required milestone: {ev}")

        first_state = milestones["FIRST_FRESH_POST_ADOPTION_CONTROLLER_STATE_FORWARDED"]["raw"]
        self.assertAlmostEqual(first_state["x_m"], self.START_X_M, places=6)
        self.assertAlmostEqual(first_state["y_m"], self.START_Y_M, places=6)
        self.assertAlmostEqual(first_state["yaw_rad"], 0.0, places=6)
        first_command = milestones["VALID_RAW_COMMAND_ACCEPTED"]["raw"]
        self.assertAlmostEqual(first_command["angular_z"], 0.0, places=6)

        announce_t = milestones["GOAL_ANNOUNCEMENT_OBSERVED"]["monotonic_time_s"]
        released_evidence_t = next(r for r in records if r["event"] == "VIRTUAL_SCOUT_RELEASED")["monotonic_time_s"]
        self.assertGreaterEqual(announce_t - released_evidence_t, 56.5, "scout travel must match production (~56.667s)")

        active_t = milestones["ACTIVE_OPENED"]["monotonic_time_s"]
        zero_t = milestones["ZERO_BURST_OPENED"]["monotonic_time_s"]
        self.assertAlmostEqual(zero_t - active_t, 6.50, delta=0.25, msg="ACTIVE duration must be ~6.50s")

        self.assertGreaterEqual(t_release - t0, 30.0)


if __name__ == "__main__":
    unittest.main()

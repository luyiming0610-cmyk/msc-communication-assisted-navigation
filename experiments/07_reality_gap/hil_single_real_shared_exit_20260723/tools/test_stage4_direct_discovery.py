"""Focused regression tests for
require_exactly_one_publisher_via_direct_discovery, extracted verbatim
from run_hil_stage4_trial.sh between the
BEGIN/END_DIRECT_DISCOVERY_FUNCTION markers so the tests exercise the
real, committed function rather than a reimplementation.

Background: the prior fix (require_exactly_one_publisher_with_retry,
removed) retried the daemon-based publisher_count() up to 5 times.
Live evidence (hil_cmd_vel_guard.log across two separate aborted
attempts, RUN_ID stage4_20260731_151052 and stage4_20260731_164028)
proved the guard was continuously healthy and self-publishing the
whole time the check read zero -- the failure was in the query, not
the guard. `ros2 topic info --help` confirms --spin-time only applies
when --no-daemon is also given, meaning the default query goes through
a persistent background daemon whose cached graph view can lag a
just-started publisher. This was reproduced offline: a fresh
daemon-based query issued immediately after starting an isolated dummy
publisher (domain 96) returned an empty read, catching up on the very
next query. `--no-daemon --spin-time` bypasses that daemon entirely
and performs its own bounded, fresh discovery -- this replaces the
retry loop with exactly one such query per gate.
"""
import subprocess
import unittest
from pathlib import Path

SCRIPT_PATH = Path(__file__).resolve().parent / "run_hil_stage4_trial.sh"
BEGIN_MARKER = "# BEGIN_DIRECT_DISCOVERY_FUNCTION"
END_MARKER = "# END_DIRECT_DISCOVERY_FUNCTION"

# An isolated domain not used by the sanctioned suite (90), the
# forbidden Stage 3 domains (0, 89, 77), or the real Stage 3 run (91).
TEST_ROS_DOMAIN_ID = "96"


def _extract_function_source():
    source = SCRIPT_PATH.read_text(encoding="utf-8")
    assert BEGIN_MARKER in source, "BEGIN_DIRECT_DISCOVERY_FUNCTION marker missing"
    assert END_MARKER in source, "END_DIRECT_DISCOVERY_FUNCTION marker missing"
    return source.split(BEGIN_MARKER, 1)[1].split(END_MARKER, 1)[0]


def _run_harness(preamble, topic, label, extra_env=None, timeout=30):
    function_source = _extract_function_source()
    harness = f"""
DIRECT_DISCOVERY_SPIN_TIME_S="5"
{preamble}
{function_source}
require_exactly_one_publisher_via_direct_discovery "{topic}" "{label}"
exit $?
"""
    env = dict(extra_env or {})
    result = subprocess.run(
        ["bash", "-c", harness],
        capture_output=True, text=True, timeout=timeout,
        env=env if env else None,
    )
    return result.returncode, result.stdout


class StaticContractTest(unittest.TestCase):
    def setUp(self):
        self.source = SCRIPT_PATH.read_text(encoding="utf-8")

    def test_uses_no_daemon_and_spin_time(self):
        self.assertIn("--no-daemon", self.source)
        self.assertIn('--spin-time "${DIRECT_DISCOVERY_SPIN_TIME_S}"', self.source)
        self.assertIn('DIRECT_DISCOVERY_SPIN_TIME_S="5"', self.source)

    def test_does_not_use_or_echo_zero_fallback_in_new_function(self):
        body = _extract_function_source()
        self.assertNotIn("|| echo 0", body)

    def test_no_retry_loop_in_new_function(self):
        body = _extract_function_source()
        self.assertNotIn("for attempt", body)
        self.assertNotIn("sleep 0.5", body)

    def test_absolute_topic_name_normalization_present(self):
        body = _extract_function_source()
        self.assertIn('if [[ "${abs_topic}" != /* ]]; then', body)


class QueryErrorAndParseErrorTest(unittest.TestCase):
    """Uses a stubbed `ros2` (first in PATH) to test the function's own
    error-handling logic in isolation -- no real ROS graph needed for
    these two paths."""

    def _run_with_stub_ros2(self, stub_body, topic="/dummy", label="test"):
        preamble = f"""
STUB_DIR="$(mktemp -d)"
cat > "${{STUB_DIR}}/ros2" <<'STUBEOF'
#!/usr/bin/env bash
{stub_body}
STUBEOF
chmod +x "${{STUB_DIR}}/ros2"
export PATH="${{STUB_DIR}}:${{PATH}}"
"""
        code, out = _run_harness(preamble, topic, label)
        return code, out

    def test_command_failure_is_query_error(self):
        code, out = self._run_with_stub_ros2("exit 1")
        self.assertNotEqual(code, 0, out)
        self.assertIn("reason=QUERY_ERROR", out)

    def test_missing_publisher_count_line_is_parse_error(self):
        code, out = self._run_with_stub_ros2("echo 'Type: std_msgs/msg/String'")
        self.assertNotEqual(code, 0, out)
        self.assertIn("reason=PARSE_ERROR", out)

    def test_non_numeric_count_is_parse_error(self):
        code, out = self._run_with_stub_ros2("echo 'Publisher count: many'")
        self.assertNotEqual(code, 0, out)
        self.assertIn("reason=PARSE_ERROR", out)


class IsolatedRosGraphTest(unittest.TestCase):
    """Real ROS 2 graph, no hardware -- isolated domain, harmless dummy
    publishers only."""

    def setUp(self):
        self.publisher_procs = []

    def tearDown(self):
        for proc in self.publisher_procs:
            proc.terminate()
        for proc in self.publisher_procs:
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=5)

    def _start_dummy_publisher(self, topic):
        proc = subprocess.Popen(
            [
                "bash", "-c",
                f"source /opt/ros/humble/setup.bash && "
                f"exec ros2 topic pub {topic} std_msgs/msg/String "
                f"'{{data: hello}}' -r 10",
            ],
            env={
                **__import__("os").environ,
                "ROS_DOMAIN_ID": TEST_ROS_DOMAIN_ID,
                "ROS_LOCALHOST_ONLY": "1",
            },
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        self.publisher_procs.append(proc)
        return proc

    def _start_dummy_subscriber(self, topic):
        # Makes the topic "known" to the graph (type registered) with
        # zero publishers -- the realistic zero-publisher case in
        # production, where a subscriber (e.g. the recorder) is always
        # already present before the guard/state_publisher starts.
        proc = subprocess.Popen(
            [
                "bash", "-c",
                f"source /opt/ros/humble/setup.bash && "
                f"exec ros2 topic echo {topic} std_msgs/msg/String",
            ],
            env={
                **__import__("os").environ,
                "ROS_DOMAIN_ID": TEST_ROS_DOMAIN_ID,
                "ROS_LOCALHOST_ONLY": "1",
            },
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        self.publisher_procs.append(proc)
        return proc

    def _harness_preamble(self):
        return (
            "source /opt/ros/humble/setup.bash\n"
            f"export ROS_DOMAIN_ID={TEST_ROS_DOMAIN_ID}\n"
            "export ROS_LOCALHOST_ONLY=1\n"
        )

    def test_one_publisher_passes(self):
        topic = f"/diag_direct_discovery_one_{__import__('os').getpid()}"
        self._start_dummy_publisher(topic)
        import time
        time.sleep(2)  # let it actually start publishing
        code, out = _run_harness(self._harness_preamble(), topic, "one_pub", timeout=20)
        self.assertEqual(code, 0, out)
        self.assertIn("decision=PASS", out)

    def test_zero_publisher_fails(self):
        topic = f"/diag_direct_discovery_zero_{__import__('os').getpid()}"
        self._start_dummy_subscriber(topic)
        import time
        time.sleep(2)  # let the subscriber register so the topic is "known"
        code, out = _run_harness(self._harness_preamble(), topic, "zero_pub", timeout=20)
        self.assertNotEqual(code, 0, out)
        self.assertIn("reason=zero_publishers_after_bounded_discovery", out)

    def test_two_publishers_fails_immediately(self):
        topic = f"/diag_direct_discovery_two_{__import__('os').getpid()}"
        self._start_dummy_publisher(topic)
        self._start_dummy_publisher(topic)
        import time
        time.sleep(2)
        code, out = _run_harness(self._harness_preamble(), topic, "two_pub", timeout=20)
        self.assertNotEqual(code, 0, out)
        self.assertIn("reason=publisher_count_exceeds_one", out)


if __name__ == "__main__":
    unittest.main()

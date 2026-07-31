"""Focused regression tests for require_exactly_one_publisher_with_retry,
extracted verbatim from run_hil_stage4_trial.sh between the
BEGIN/END_READINESS_RETRY_FUNCTION markers so the tests exercise the
real, committed function rather than a reimplementation.

Background: publisher_count() is a single one-shot `ros2 topic info`
CLI call with no retry -- discovered live (RUN_ID
stage4_20260731_151052) that it can report a transient zero for a
topic whose real publisher is alive and healthy the whole time
(hil_cmd_vel_guard.log showed continuous self-publishing through and
after the moment of the spurious zero read). This bounded,
discovery-aware retry requires exactly one publisher on two
CONSECUTIVE reads before PASS, fails immediately on any count > 1
(never retried -- that is a genuine safety violation), and fails
closed if two consecutive 1-reads are never observed within 5
attempts.
"""
import os
import subprocess
import unittest
from pathlib import Path

SCRIPT_PATH = Path(__file__).resolve().parent / "run_hil_stage4_trial.sh"
BEGIN_MARKER = "# BEGIN_READINESS_RETRY_FUNCTION"
END_MARKER = "# END_READINESS_RETRY_FUNCTION"


def _extract_function_source():
    source = SCRIPT_PATH.read_text(encoding="utf-8")
    assert BEGIN_MARKER in source, "BEGIN_READINESS_RETRY_FUNCTION marker missing"
    assert END_MARKER in source, "END_READINESS_RETRY_FUNCTION marker missing"
    body = source.split(BEGIN_MARKER, 1)[1].split(END_MARKER, 1)[0]
    return body


def _run_with_mock_sequence(counts):
    """Sources the real, extracted function with a mock publisher_count
    that returns each value in `counts` in order (repeating the last
    value if exhausted), then invokes it and returns (exit_code, stdout).

    The mock's own call index is persisted in a counter file rather
    than a plain bash variable: `count="$(publisher_count ...)"` in the
    real function runs the mock inside a command-substitution subshell,
    so any in-memory index update would be lost the instant that
    subshell exits.
    """
    function_source = _extract_function_source()
    queue = " ".join(str(c) for c in counts)
    counter_path = f"/tmp/test_readiness_retry_counter_{os.getpid()}"
    Path(counter_path).write_text("0")
    harness = f"""
set -uo pipefail
_MOCK_COUNTS=({queue})
_MOCK_COUNTER_FILE="{counter_path}"
publisher_count() {{
    local idx
    idx="$(cat "${{_MOCK_COUNTER_FILE}}")"
    if [[ $idx -ge ${{#_MOCK_COUNTS[@]}} ]]; then
        idx=$((${{#_MOCK_COUNTS[@]}} - 1))
    fi
    echo "${{_MOCK_COUNTS[$idx]}}"
    echo $((idx + 1)) > "${{_MOCK_COUNTER_FILE}}"
}}

{function_source}

require_exactly_one_publisher_with_retry "dummy_topic" "test_label"
exit $?
"""
    try:
        result = subprocess.run(
            ["bash", "-c", harness],
            capture_output=True, text=True, timeout=30,
        )
    finally:
        Path(counter_path).unlink(missing_ok=True)
    return result.returncode, result.stdout


class ReadinessRetryTest(unittest.TestCase):
    def test_two_consecutive_ones_passes(self):
        code, out = _run_with_mock_sequence([1, 1])
        self.assertEqual(code, 0, out)
        self.assertIn("decision=PASS", out)

    def test_zero_then_two_ones_passes(self):
        code, out = _run_with_mock_sequence([0, 1, 1])
        self.assertEqual(code, 0, out)
        self.assertIn("decision=PASS", out)

    def test_persistent_zero_fails(self):
        code, out = _run_with_mock_sequence([0, 0, 0, 0, 0])
        self.assertNotEqual(code, 0, out)
        self.assertIn("decision=FAIL", out)
        self.assertIn("attempt=5", out)

    def test_count_above_one_fails_immediately(self):
        code, out = _run_with_mock_sequence([2, 1, 1, 1, 1])
        self.assertNotEqual(code, 0, out)
        self.assertIn("decision=FAIL", out)
        self.assertIn("reason=publisher_count_exceeds_one", out)
        # Immediate: must not have attempted a second read.
        self.assertNotIn("attempt=2", out)

    def test_unstable_alternating_sequence_fails(self):
        code, out = _run_with_mock_sequence([1, 0, 1, 0, 1])
        self.assertNotEqual(code, 0, out)
        self.assertIn("decision=FAIL", out)
        self.assertIn("attempt=5", out)

    def test_every_attempt_and_count_is_logged(self):
        code, out = _run_with_mock_sequence([0, 0, 1, 1])
        self.assertEqual(code, 0, out)
        for attempt, count in [(1, 0), (2, 0), (3, 1), (4, 1)]:
            self.assertIn(f"attempt={attempt} topic=dummy_topic count={count}", out)


if __name__ == "__main__":
    unittest.main()

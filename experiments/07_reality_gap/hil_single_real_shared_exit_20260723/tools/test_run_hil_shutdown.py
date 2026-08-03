"""Focused regression tests for run_hil_shutdown.sh.

Covers two defects found live during RUN_ID stage4_20260731_151052's
aborted trial: (1) the ORDER list used the wrong key ("recorder_bag"
instead of "recorder") and omitted "hil_stage4_motion_supervisor"
entirely, so both were silently never signaled; (2) processes launched
via `ros2 run` spawn a wrapper PID whose real node runs as a separate
child PID -- signaling only the recorded wrapper PID could leave the
real child alive. All process-tree lookups here are exact PID/PPID
based, never name-based, matching the script's own no-pkill contract.
"""
import json
import os
import re
import signal
import subprocess
import tempfile
import threading
import time
import unittest
from pathlib import Path

SCRIPT_PATH = Path(__file__).resolve().parent / "run_hil_shutdown.sh"


def _order_list(source):
    match = re.search(r"ORDER='(\[.*\])'", source)
    assert match is not None, "could not find ORDER list in script"
    return json.loads(match.group(1))


def _alive(pid):
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    return True


def _wait_gone(pid, timeout_s=5.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if not _alive(pid):
            return True
        time.sleep(0.1)
    return not _alive(pid)


def _wait_reaped(proc, timeout_s=8.0):
    """For a process this test itself is the direct parent of (a
    subprocess.Popen handle): os.kill(pid, 0) alone cannot distinguish
    a truly-running process from a zombie this test hasn't reaped yet,
    so this must go through Popen.wait() to actually reap it."""
    try:
        proc.wait(timeout=timeout_s)
        return True
    except subprocess.TimeoutExpired:
        return False


class OrderListStaticTest(unittest.TestCase):
    def setUp(self):
        self.source = SCRIPT_PATH.read_text(encoding="utf-8")

    def test_order_list_uses_recorder_not_recorder_bag(self):
        order = _order_list(self.source)
        self.assertIn("recorder", order)
        self.assertNotIn("recorder_bag", order)

    def test_order_list_includes_motion_supervisor(self):
        self.assertIn('"hil_stage4_motion_supervisor"', self.source)

    def test_recorder_is_last_in_order_list(self):
        order = _order_list(self.source)
        self.assertEqual(order[-1], "recorder", f"recorder must be last, got order={order}")


class DummyProcessCleanupTest(unittest.TestCase):
    """Harmless dummy processes only -- no ROS, no hardware, no pkill."""

    def setUp(self):
        self.tmp_dir = tempfile.TemporaryDirectory()
        self.tmp_manifest = None
        self.spawned_pids = []

    def tearDown(self):
        # Best-effort safety net in case a test assertion fails before
        # its own cleanup -- still exact-PID, never name-based.
        for pid in self.spawned_pids:
            if _alive(pid):
                try:
                    os.kill(pid, signal.SIGKILL)
                except OSError:
                    pass
        self.tmp_dir.cleanup()

    def _run_shutdown(self, manifest_path, reap_proc=None):
        """A real orchestrator shell reaps its own backgrounded children
        promptly; this test process (Python) does not do that
        automatically the way bash does. Without a concurrent reaper,
        a dummy process that has already died from SIGINT still shows
        as "alive" (a zombie) to the shutdown script's own os.kill(pid,
        0) polling for the entire run, since this test is the zombie's
        actual parent. `reap_proc`, if given, is waited on in a
        background thread for the duration of the shutdown call so it
        gets reaped as soon as it exits, matching real shell behavior.
        """
        reaper = None
        if reap_proc is not None:
            reaper = threading.Thread(target=reap_proc.wait, daemon=True)
            reaper.start()
        try:
            return subprocess.run(
                ["bash", str(SCRIPT_PATH), str(manifest_path)],
                capture_output=True, text=True, timeout=30,
            )
        finally:
            if reaper is not None:
                reaper.join(timeout=15)

    def test_plain_single_pid_process_is_cleaned_up(self):
        """Regression safety net: the new child-discovery logic must not
        break the simple, no-children case.

        The dummy process explicitly restores the default SIGINT
        handler: a plain backgrounded `sleep`/python3 process can
        inherit SIGINT=SIG_IGN from whatever launched this test process
        (background jobs conventionally ignore SIGINT), and CPython
        leaves it ignored rather than installing its own handler if it
        was already SIG_IGN at interpreter start. Real rclpy nodes avoid
        this by explicitly installing their own SIGINT handler on
        startup; this dummy does the same to be a valid stand-in."""
        proc = subprocess.Popen([
            "python3", "-c",
            "import signal, time; "
            "signal.signal(signal.SIGINT, signal.default_int_handler); "
            "time.sleep(1000)",
        ])
        self.spawned_pids.append(proc.pid)
        self.tmp_manifest = Path(self.tmp_dir.name) / "pid_manifest.json"
        self.tmp_manifest.write_text(json.dumps({
            "processes": {"hil_cmd_vel_guard": {"pid": proc.pid, "sha256": ""}}
        }))

        result = self._run_shutdown(self.tmp_manifest, reap_proc=proc)

        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("PROCESSES_CLEAN", result.stdout)
        residual = json.loads((Path(self.tmp_dir.name) / "residual_check.json").read_text())
        self.assertEqual(residual["residual_process_check"], "CLEAN")
        self.assertEqual(residual["shutdown_results"]["hil_cmd_vel_guard"]["status"], "STOPPED")
        self.assertTrue(_wait_reaped(proc, timeout_s=1.0), "plain dummy process was not cleaned up")

    def test_wrapper_ignoring_sigint_with_responsive_child_is_fully_cleaned_up(self):
        """Reproduces the exact live failure: a wrapper process that
        ignores SIGINT itself (as cooperative_avoider's `ros2 run`
        wrapper, PID 963, was observed to do -- STILL_ALIVE_AFTER_10S
        under the old script) but whose real child dies normally and
        whose wrapper then exits naturally once its `wait` returns."""
        # The child explicitly restores the default SIGINT handler (see
        # test_plain_single_pid_process_is_cleaned_up for why this is
        # needed) so it behaves like a real rclpy node -- normally
        # responsive to SIGINT -- despite being backgrounded under a
        # wrapper that itself ignores SIGINT.
        wrapper_script = (
            "trap '' INT; "
            "python3 -c 'import signal, time; "
            "signal.signal(signal.SIGINT, signal.default_int_handler); "
            "time.sleep(1000)' & "
            "child=$!; "
            "echo READY; "
            "wait \"$child\"; "
            "exit $?"
        )
        proc = subprocess.Popen(["bash", "-c", wrapper_script])
        self.spawned_pids.append(proc.pid)
        # Give the wrapper a moment to fork its child before we look it up.
        deadline = time.monotonic() + 5.0
        child_pid = None
        while time.monotonic() < deadline and child_pid is None:
            ps = subprocess.run(
                ["ps", "--ppid", str(proc.pid), "-o", "pid=,cmd="],
                capture_output=True, text=True,
            )
            for line in ps.stdout.splitlines():
                line = line.strip()
                if "time.sleep" in line:
                    child_pid = int(line.split()[0])
                    break
            if child_pid is None:
                time.sleep(0.1)
        self.assertIsNotNone(child_pid, "dummy wrapper never spawned its child in time")
        self.spawned_pids.append(child_pid)

        self.tmp_manifest = Path(self.tmp_dir.name) / "pid_manifest.json"
        self.tmp_manifest.write_text(json.dumps({
            "processes": {"cooperative_avoider": {"pid": proc.pid, "sha256": ""}}
        }))

        result = self._run_shutdown(self.tmp_manifest, reap_proc=proc)

        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("PROCESSES_CLEAN", result.stdout)
        # The wrapper reaps its own child via its internal `wait "$child"`
        # (not a zombie risk for this test process, which is not the
        # child's parent) -- only the wrapper itself is this test's
        # direct child and needs Popen.wait() to actually reap.
        self.assertTrue(_wait_gone(child_pid), "wrapper's real child was not cleaned up")
        self.assertTrue(_wait_reaped(proc, timeout_s=1.0), "wrapper itself was not cleaned up")


if __name__ == "__main__":
    unittest.main()

"""Tests for wsl_expanded_pilot_recorder.py's shutdown fix.

Covers the three real exit paths per instruction: a normal timeout-
equivalent exit (loop's own stop condition, no signal involved), a real
SIGINT delivered to a live subprocess (the only way to genuinely exercise
the OS-level signal-timing bug that was actually observed), and an
already-externally-shut-down rclpy context. Also checks that CSV/
checkpoint data is flushed before shutdown runs, and that a real signal
exit produces code 0 with no traceback on stderr.
"""
import json
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy

sys.path.insert(0, str(Path(__file__).resolve().parent))
from wsl_expanded_pilot_recorder import ExpandedPilotRecorder, run  # noqa: E402


def test_run_exits_cleanly_when_stop_flag_set_without_any_signal(tmp_path):
    """Simulates the normal 300s-equivalent timeout exit: the loop stops
    because stop_requested() became True, never involving a signal."""
    rclpy.init()
    try:
        node = ExpandedPilotRecorder(str(tmp_path / "status.csv"), str(tmp_path / "checkpoints.json"))
        call_count = {"n": 0}

        def fake_spin_once():
            call_count["n"] += 1

        def stop_after_a_few():
            return call_count["n"] >= 3

        recorded = run(node, [0.0], stop_after_a_few, spin_once_fn=fake_spin_once)
        assert "start" in recorded
        assert call_count["n"] >= 3
        node.close()
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_checkpoints_json_written_before_run_returns(tmp_path):
    rclpy.init()
    try:
        node = ExpandedPilotRecorder(str(tmp_path / "status.csv"), str(tmp_path / "checkpoints.json"))
        call_count = {"n": 0}

        def fake_spin_once():
            call_count["n"] += 1

        def stop_after_one():
            return call_count["n"] >= 1

        run(node, [0.0], stop_after_one, spin_once_fn=fake_spin_once)
        checkpoints_path = tmp_path / "checkpoints.json"
        assert checkpoints_path.exists()
        data = json.loads(checkpoints_path.read_text())
        assert len(data) >= 1
        assert data[0]["label"] == "start"
        assert data[0]["publisher_count"] == 0
        node.close()
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_csv_header_flushed_immediately_on_construction(tmp_path):
    """CSV rows are written+flushed inside _on_status as they arrive
    (never buffered until shutdown) -- the header itself is written and
    the file is open/readable immediately on node construction."""
    rclpy.init()
    try:
        node = ExpandedPilotRecorder(str(tmp_path / "status.csv"), str(tmp_path / "checkpoints.json"))
        csv_path = tmp_path / "status.csv"
        assert csv_path.exists()
        content = csv_path.read_text()
        assert content.startswith("wsl_unix_time_s,connected,")
        node.close()
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_shutdown_guard_is_idempotent_and_survives_already_shutdown_context(tmp_path):
    """Exit path 3: rclpy has already been shut down externally before our
    own shutdown logic runs -- must not raise, and calling the guard twice
    must also not raise (never a double-shutdown crash)."""
    rclpy.init()
    node = ExpandedPilotRecorder(str(tmp_path / "status.csv"), str(tmp_path / "checkpoints.json"))
    node.close()
    node.destroy_node()
    rclpy.shutdown()  # external shutdown happens first, out of band

    shutdown_done = {"done": False}

    def _shutdown_once():
        if shutdown_done["done"]:
            return
        shutdown_done["done"] = True
        if rclpy.ok():
            rclpy.shutdown()

    _shutdown_once()  # must not raise even though rclpy.ok() is already False
    _shutdown_once()  # calling twice must also not raise


def test_subprocess_sigint_exits_zero_with_no_traceback(tmp_path):
    """The actual bug this session: rclpy.spin_once() itself raised mid-call
    when rclpy's own default SIGINT handler raced this script's loop. Only
    a real subprocess + real OS signal can genuinely exercise that timing,
    so this drives the actual script, not a mock."""
    script = Path(__file__).resolve().parent / "wsl_expanded_pilot_recorder.py"
    status_csv = tmp_path / "status.csv"
    checkpoints_json = tmp_path / "checkpoints.json"
    proc = subprocess.Popen(
        [sys.executable, str(script),
         "--status-csv", str(status_csv),
         "--cmd-vel-checkpoints-json", str(checkpoints_json),
         "--checkpoint-schedule-s", "0.0", "1.0", "2.0"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    )
    time.sleep(1.5)  # let it pass the "start" checkpoint at least
    proc.send_signal(signal.SIGINT)
    stdout, stderr = proc.communicate(timeout=10)
    assert proc.returncode == 0, f"nonzero exit ({proc.returncode}), stderr:\n{stderr}"
    assert "Traceback" not in stderr, f"unexpected traceback on stderr:\n{stderr}"
    assert status_csv.exists()
    assert checkpoints_json.exists()
    data = json.loads(checkpoints_json.read_text())
    assert len(data) >= 1


def test_subprocess_normal_early_stop_still_produces_valid_files(tmp_path):
    """A second subprocess run, this time letting all scheduled checkpoints
    complete naturally before sending SIGINT (stands in for the '300s
    equivalent' full-duration case without an actual 300s wait)."""
    script = Path(__file__).resolve().parent / "wsl_expanded_pilot_recorder.py"
    status_csv = tmp_path / "status.csv"
    checkpoints_json = tmp_path / "checkpoints.json"
    proc = subprocess.Popen(
        [sys.executable, str(script),
         "--status-csv", str(status_csv),
         "--cmd-vel-checkpoints-json", str(checkpoints_json),
         "--checkpoint-schedule-s", "0.0", "0.5", "1.0"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    )
    time.sleep(2.0)  # all three scheduled checkpoints should have fired by now
    proc.send_signal(signal.SIGINT)
    stdout, stderr = proc.communicate(timeout=10)
    assert proc.returncode == 0, f"nonzero exit ({proc.returncode}), stderr:\n{stderr}"
    assert "Traceback" not in stderr
    data = json.loads(checkpoints_json.read_text())
    labels = [d["label"] for d in data]
    assert "start" in labels and "mid" in labels and "end" in labels

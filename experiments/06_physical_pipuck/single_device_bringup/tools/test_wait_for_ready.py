"""Tests for wait_for_ready.py's READY-barrier primitives.

Covers, with real subprocesses/PIDs where the behavior depends on process
liveness (not fakeable with a plain function call), and synthetic files
where it doesn't:
  - status/system recorder delayed start (csv becomes ready only after
    real rows land, not immediately),
  - rosbag delayed topic subscription (bag becomes ready only after all
    expected topics appear in the log, not as soon as "Recording..." does),
  - a process dying before satisfying its READY condition must abort
    (ProcessDiedBeforeReady), distinct from a plain timeout,
  - a torn/incomplete final CSV line (write still in progress on disk)
    must not be miscounted as a valid ready row -- proves shutdown-flush
    completeness is actually checked, not assumed.
"""
import subprocess
import sys
import time

import pytest

from wait_for_ready import csv_ready, bag_ready, poll_until_ready, ProcessDiedBeforeReady


CSV_WRITER_DELAYED_STUB = """
import csv, sys, time
path = sys.argv[1]
delay_s = float(sys.argv[2])
with open(path, "w", newline="", encoding="utf-8") as fh:
    fh.write("wsl_unix_time_s,value\\n")
    fh.flush()
time.sleep(delay_s)
with open(path, "a", newline="", encoding="utf-8") as fh:
    for i in range(4):
        fh.write(f"{time.time():.3f},{i}\\n")
        fh.flush()
        time.sleep(0.05)
time.sleep(5)
"""

CSV_WRITER_DIES_IMMEDIATELY_STUB = """
import sys
path = sys.argv[1]
with open(path, "w", newline="", encoding="utf-8") as fh:
    fh.write("wsl_unix_time_s,value\\n")
    fh.flush()
sys.exit(0)
"""

BAG_LOG_DELAYED_STUB = """
import sys, time
path = sys.argv[1]
delay_s = float(sys.argv[2])
with open(path, "w", encoding="utf-8") as fh:
    fh.write("[INFO] Listening for topics...\\n")
    fh.write("[INFO] Recording...\\n")
    fh.flush()
time.sleep(delay_s)
with open(path, "a", encoding="utf-8") as fh:
    for t in ("/odom", "/scan", "/epuck1/state"):
        fh.write(f"[INFO] Subscribed to topic '{t}'\\n")
        fh.flush()
        time.sleep(0.05)
time.sleep(5)
"""


def _spawn(stub_code, *args):
    return subprocess.Popen([sys.executable, "-c", stub_code, *args])


def test_csv_ready_becomes_true_only_after_delayed_rows_land(tmp_path):
    csv_path = tmp_path / "status.csv"
    proc = _spawn(CSV_WRITER_DELAYED_STUB, str(csv_path), "1.0")
    try:
        # Immediately after spawn, header may exist but rows don't yet --
        # not ready.
        time.sleep(0.1)
        ready_early, _ = csv_ready(str(csv_path), "wsl_unix_time_s", min_rows=2)
        assert ready_early is False

        start = time.monotonic()
        ready, reason = poll_until_ready(
            lambda: csv_ready(str(csv_path), "wsl_unix_time_s", min_rows=2),
            proc.pid, timeout_s=5.0, poll_interval_s=0.1,
        )
        elapsed = time.monotonic() - start
        assert ready is True
        assert elapsed >= 0.9, "became ready suspiciously fast -- delayed rows should have been waited for"
    finally:
        proc.kill()
        proc.wait(timeout=5)


def test_bag_ready_becomes_true_only_after_delayed_subscriptions(tmp_path):
    log_path = tmp_path / "bag_record.log"
    proc = _spawn(BAG_LOG_DELAYED_STUB, str(log_path), "1.0")
    try:
        time.sleep(0.1)
        ready_early, reason_early = bag_ready(str(log_path), ["/odom", "/scan", "/epuck1/state"])
        assert ready_early is False
        assert "missing subscriptions" in reason_early or "not yet recording" in reason_early

        start = time.monotonic()
        ready, reason = poll_until_ready(
            lambda: bag_ready(str(log_path), ["/odom", "/scan", "/epuck1/state"]),
            proc.pid, timeout_s=5.0, poll_interval_s=0.1,
        )
        elapsed = time.monotonic() - start
        assert ready is True
        assert elapsed >= 0.9
    finally:
        proc.kill()
        proc.wait(timeout=5)


def test_process_dying_before_ready_raises_distinct_exception(tmp_path):
    """The core safety requirement: a process that exits before its READY
    condition is met must cause an abort (ProcessDiedBeforeReady), not be
    silently treated as an ordinary timeout, and must be detected promptly
    -- not only after the full timeout window elapses."""
    csv_path = tmp_path / "status.csv"
    proc = _spawn(CSV_WRITER_DIES_IMMEDIATELY_STUB, str(csv_path))
    proc.wait(timeout=5)  # ensure it has actually exited before polling starts
    assert proc.returncode == 0

    start = time.monotonic()
    with pytest.raises(ProcessDiedBeforeReady):
        poll_until_ready(
            lambda: csv_ready(str(csv_path), "wsl_unix_time_s", min_rows=2),
            proc.pid, timeout_s=10.0, poll_interval_s=0.1,
        )
    elapsed = time.monotonic() - start
    assert elapsed < 2.0, "process death should be detected quickly, not only at the full 10s timeout"


def test_torn_incomplete_final_line_not_counted_as_valid_row(tmp_path):
    """Simulates polling a CSV file mid-write: the last line has no
    trailing newline and is missing its second field entirely (write was
    interrupted between the two fh.write() calls a real recorder might
    make). Must not be counted as a valid ready row."""
    csv_path = tmp_path / "status.csv"
    csv_path.write_text(
        "wsl_unix_time_s,value\n"
        "1000.000,1\n"
        "1000.100,2\n"
        "1000.20",  # torn: no comma, no value, no newline
        encoding="utf-8",
    )
    ready, reason = csv_ready(str(csv_path), "wsl_unix_time_s", min_rows=3)
    assert ready is False
    assert "only 2" in reason  # the torn line must not count as a 3rd row

    ready2, _ = csv_ready(str(csv_path), "wsl_unix_time_s", min_rows=2)
    assert ready2 is True  # the 2 genuinely complete rows are still valid


def test_csv_ready_rejects_non_monotonic_timestamps(tmp_path):
    csv_path = tmp_path / "status.csv"
    csv_path.write_text(
        "wsl_unix_time_s,value\n1000.500,1\n1000.100,2\n",
        encoding="utf-8",
    )
    ready, reason = csv_ready(str(csv_path), "wsl_unix_time_s", min_rows=2)
    assert ready is False
    assert "monotonic" in reason


def test_bag_ready_rejects_log_containing_error_or_warn(tmp_path):
    log_path = tmp_path / "bag_record.log"
    log_path.write_text(
        "[INFO] Recording...\n"
        "[INFO] Subscribed to topic '/odom'\n"
        "[WARN] some dropped message warning\n",
        encoding="utf-8",
    )
    ready, reason = bag_ready(str(log_path), ["/odom"])
    assert ready is False
    assert "warn" in reason.lower()

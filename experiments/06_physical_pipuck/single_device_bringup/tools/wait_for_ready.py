#!/usr/bin/env python3
"""READY-barrier primitives for the fixed baseline_v1 orchestrator.

Two things must be proven READY before the orchestrator records a T0 and
starts the 315s formal timer:

  1. status recorder + WSL system sampler: process alive, CSV header
     flushed, >=2 valid data rows present, timestamps monotonic.
  2. rosbag: process alive, log shows "Recording...", all expected topics
     have a "Subscribed to topic" line, no error/warn line present.

Both single-shot checks (`csv_ready`, `bag_ready`) are pure functions over
already-read file content -- no polling, no process interaction -- so they
are directly unit-testable with synthetic strings. `poll_until_ready`
wraps a check function in a bounded polling loop that ALSO monitors the
target process's liveness, so a process that dies before satisfying the
READY condition is detected and reported distinctly from a plain timeout
(exit code 2 vs 1) -- the caller must abort, not proceed past a barrier
whose owning process is already dead.
"""
from __future__ import annotations

import argparse
import csv
import os
import re
import sys
import time


def pid_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    return True


def csv_ready(csv_path: str, time_column: str, min_rows: int = 2) -> tuple:
    """Reads the CSV directly from disk each call (no caching) so a caller
    polling this function always sees the current on-disk state. A
    partially-written final line (write interrupted mid-row, e.g. between
    two separate fh.write() calls in the recorder, or with no trailing
    newline yet) leaves csv.DictReader unable to fill every expected
    column -- any row missing a value for ANY header column (None from
    DictReader's own short-row handling) is treated as incomplete and
    skipped, not counted as a valid ready row, even if the time column
    itself happens to already be present."""
    if not os.path.exists(csv_path):
        return False, "file does not exist yet"
    try:
        with open(csv_path, encoding="utf-8", newline="") as fh:
            reader = csv.DictReader(fh)
            if reader.fieldnames is None:
                return False, "header not present"
            times = []
            for row in reader:
                if any(v is None for v in row.values()):
                    continue  # torn/incomplete row -- missing one or more columns
                raw = row.get(time_column)
                if raw in (None, ""):
                    continue
                try:
                    times.append(float(raw))
                except ValueError:
                    continue
    except (OSError, csv.Error) as exc:
        return False, f"read error: {exc}"

    if len(times) < min_rows:
        return False, f"only {len(times)} valid data row(s), need >= {min_rows}"
    if any(times[i] > times[i + 1] for i in range(len(times) - 1)):
        return False, "timestamps not monotonic"
    return True, f"ready: {len(times)} row(s)"


_SUBSCRIBED_RE = re.compile(r"Subscribed to topic '([^']+)'")
_ERROR_RE = re.compile(r"\berror\b", re.IGNORECASE)
_WARN_RE = re.compile(r"\bwarn(ing)?\b", re.IGNORECASE)


def bag_ready(log_path: str, expected_topics: list) -> tuple:
    if not os.path.exists(log_path):
        return False, "log file does not exist yet"
    try:
        with open(log_path, encoding="utf-8", errors="replace") as fh:
            text = fh.read()
    except OSError as exc:
        return False, f"read error: {exc}"

    if "Recording..." not in text:
        return False, "not yet recording"
    if _ERROR_RE.search(text):
        return False, "log contains an error line"
    if _WARN_RE.search(text):
        return False, "log contains a warn/warning line"

    subscribed = set(_SUBSCRIBED_RE.findall(text))
    missing = [t for t in expected_topics if t not in subscribed]
    if missing:
        return False, f"missing subscriptions: {missing}"
    return True, f"ready: {len(subscribed)} topic(s) subscribed"


class ProcessDiedBeforeReady(Exception):
    pass


def poll_until_ready(check_fn, pid: int, timeout_s: float, poll_interval_s: float = 0.2) -> tuple:
    """Returns (True, reason) once check_fn() reports ready. Raises
    ProcessDiedBeforeReady if the watched pid dies first (even if the
    on-disk state technically looks ready at that exact instant -- a dead
    owning process is never an acceptable READY state). Returns
    (False, reason) on a plain timeout with the process still alive."""
    deadline = time.monotonic() + timeout_s
    last_reason = "never checked"
    while time.monotonic() < deadline:
        if not pid_alive(pid):
            raise ProcessDiedBeforeReady(f"pid {pid} died before READY (last check: {last_reason})")
        ready, last_reason = check_fn()
        if ready:
            return True, last_reason
        time.sleep(poll_interval_s)
    if not pid_alive(pid):
        raise ProcessDiedBeforeReady(f"pid {pid} died before READY (last check: {last_reason})")
    return False, f"timeout after {timeout_s}s (last check: {last_reason})"


def main():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="mode", required=True)

    p_csv = sub.add_parser("csv")
    p_csv.add_argument("--path", required=True)
    p_csv.add_argument("--time-column", required=True)
    p_csv.add_argument("--min-rows", type=int, default=2)
    p_csv.add_argument("--pid", type=int, required=True)
    p_csv.add_argument("--timeout", type=float, default=15.0)

    p_bag = sub.add_parser("bag")
    p_bag.add_argument("--log-path", required=True)
    p_bag.add_argument("--topics", required=True, help="comma-separated expected topic names")
    p_bag.add_argument("--pid", type=int, required=True)
    p_bag.add_argument("--timeout", type=float, default=15.0)

    args = parser.parse_args()

    if args.mode == "csv":
        check_fn = lambda: csv_ready(args.path, args.time_column, args.min_rows)
    else:
        topics = [t for t in args.topics.split(",") if t]
        check_fn = lambda: bag_ready(args.log_path, topics)

    try:
        ready, reason = poll_until_ready(check_fn, args.pid, args.timeout)
    except ProcessDiedBeforeReady as exc:
        print(f"PROCESS_DIED_BEFORE_READY: {exc}", file=sys.stderr)
        sys.exit(2)

    if ready:
        print(f"READY: {reason}")
        sys.exit(0)
    else:
        print(f"TIMEOUT: {reason}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()

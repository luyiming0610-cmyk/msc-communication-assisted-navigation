#!/usr/bin/env python3
"""Analyzer for physical_single_device_transport_diagnostic_pilot01.

DIAGNOSTIC_PHYSICAL only. Does not validate the current EpuckState
protocol or avoidance behaviour -- only the base Pi-TCP-WSL transport
link. Reads (never modifies): the rosbag, wsl_transport_status.csv,
wsl_transport_totals.json, wsl_system_metrics.csv, pi_system_metrics.csv
(if present), bag_info.txt, qos_*.txt, bag_record_warnings.txt.

Key measurement-integrity rules enforced here (per instruction):
- RTT and last_state_age_s are single-WSL-clock-domain values (computed
  entirely from WSL's own time.monotonic()/time.time() calls in
  wsl_epuck_tcp_bridge.py) and are reported as VALID distributions.
- wall_clock_delta_ms is a Pi-vs-WSL clock-offset diagnostic ONLY. No
  cross-domain one-way latency is computed or reported; that field is
  never treated as latency.
- The currently-running BASE bridge does not expose a paired source
  sequence number to any WSL topic -- true sequence-gap/duplicate/
  out-of-order/aligned-window PDR are reported NOT_MEASURABLE with this
  reason, never approximated via actual-count/theoretical-rate.
- Any metric that cannot be reliably computed from what was actually
  recorded is NOT_MEASURABLE or NOT_VALID, never silently defaulted to a
  PASS-shaped number.
"""
import argparse
import csv
import json
import math
import statistics
from pathlib import Path


EXPECTED_SAMPLE_INTERVAL_S = 1.0
MAX_ACCEPTABLE_SAMPLE_GAP_S = 3.0  # any single gap beyond this = sampler stall
MIN_ACCEPTABLE_SAMPLE_COUNT_FRACTION = 0.90  # vs. expected total_s / interval_s


def _percentile(sorted_values, fraction):
    if not sorted_values:
        return None
    if len(sorted_values) == 1:
        return sorted_values[0]
    index = fraction * (len(sorted_values) - 1)
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return sorted_values[int(index)]
    weight = index - lower
    return sorted_values[lower] * (1 - weight) + sorted_values[upper] * weight


def _dist_stats(values):
    values = [v for v in values if v is not None]
    if not values:
        return {"sample_count": 0, "mean": None, "median": None, "p95": None, "p99": None, "max": None}
    sorted_values = sorted(values)
    return {
        "sample_count": len(values),
        "mean": statistics.fmean(values),
        "median": _percentile(sorted_values, 0.50),
        "p95": _percentile(sorted_values, 0.95),
        "p99": _percentile(sorted_values, 0.99),
        "max": sorted_values[-1],
    }


def _read_transport_status_csv(path: Path):
    rows = []
    with path.open(encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            rows.append(row)
    return rows


def _timestamp_integrity(path: Path, time_col: str, label: str):
    """Monotonicity/duplicate/gap check -- never assumed, always measured."""
    times = []
    if not path.exists():
        return {"label": label, "path": str(path), "valid_rows": 0, "exists": False}
    with path.open(encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            raw = row.get(time_col)
            if raw in (None, ""):
                continue
            try:
                times.append(float(raw))
            except ValueError:
                pass
    if not times:
        return {"label": label, "path": str(path), "valid_rows": 0, "exists": True}
    monotonic_non_decreasing = all(times[i] <= times[i + 1] for i in range(len(times) - 1))
    duplicate_count = sum(1 for i in range(len(times) - 1) if times[i] == times[i + 1])
    gaps = [times[i + 1] - times[i] for i in range(len(times) - 1)]
    return {
        "label": label,
        "path": str(path),
        "valid_rows": len(times),
        "monotonic_non_decreasing": monotonic_non_decreasing,
        "duplicate_timestamp_count": duplicate_count,
        "first_time_unix_s": times[0],
        "last_time_unix_s": times[-1],
        "span_s": times[-1] - times[0],
        "max_gap_s": max(gaps) if gaps else 0.0,
        "min_gap_s": min(gaps) if gaps else 0.0,
        "mean_gap_s": (sum(gaps) / len(gaps)) if gaps else 0.0,
    }


def _in_window(t, window_start, window_end):
    return window_start <= t <= window_end


def _read_bag_topic(bag_dir: Path, topic: str):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    type_names = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in type_names:
        return []
    msg_type = get_message(type_names[topic])
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    rows = []
    while reader.has_next():
        _, raw, ts = reader.read_next()
        message = deserialize_message(raw, msg_type)
        rows.append((ts, message))
    rows.sort(key=lambda r: r[0])
    return rows


def _system_csv_stats(path: Path, columns, time_col=None, window=None):
    if not path.exists():
        return {col: {"sample_count": 0, "mean": None, "median": None, "p95": None, "p99": None, "max": None} for col in columns}
    data = {col: [] for col in columns}
    with path.open(encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            if window is not None and time_col is not None:
                raw_t = row.get(time_col)
                if raw_t in (None, ""):
                    continue
                try:
                    t = float(raw_t)
                except ValueError:
                    continue
                if not _in_window(t, *window):
                    continue
            for col in columns:
                raw = row.get(col, "")
                if raw not in ("", None):
                    try:
                        data[col].append(float(raw))
                    except ValueError:
                        pass
    return {col: _dist_stats(vals) for col, vals in data.items()}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--native-bag-dir", type=Path, required=True)
    parser.add_argument("--native-diag-dir", type=Path, required=True)
    parser.add_argument("--pi-metrics-csv", type=Path, required=False)
    parser.add_argument("--main-window-start", type=float, required=True,
                         help="Unix seconds; the start of a real, measured overlap window across Pi/WSL/bag timestamps, NOT a fixed offset assumption.")
    parser.add_argument("--main-window-end", type=float, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    main_window = (args.main_window_start, args.main_window_end)

    reasons_fail = []
    not_measurable = []
    not_valid = []

    timestamp_integrity = [
        _timestamp_integrity(args.native_diag_dir / "wsl_transport_status.csv", "wsl_unix_time_s", "wsl_transport_status"),
        _timestamp_integrity(args.native_diag_dir / "wsl_system_metrics.csv", "unix_time_s", "wsl_system_metrics"),
    ]
    if args.pi_metrics_csv:
        timestamp_integrity.append(_timestamp_integrity(args.pi_metrics_csv, "unix_time_s", "pi_system_metrics"))
    for integrity in timestamp_integrity:
        if integrity.get("valid_rows", 0) == 0:
            continue
        if not integrity["monotonic_non_decreasing"]:
            reasons_fail.append(f"{integrity['label']}: timestamps are not monotonically non-decreasing")
        if integrity["duplicate_timestamp_count"] > 0:
            reasons_fail.append(f"{integrity['label']}: {integrity['duplicate_timestamp_count']} duplicate timestamp(s)")
        if integrity["max_gap_s"] > MAX_ACCEPTABLE_SAMPLE_GAP_S:
            reasons_fail.append(f"{integrity['label']}: max sample gap {integrity['max_gap_s']:.2f}s exceeds {MAX_ACCEPTABLE_SAMPLE_GAP_S}s")

    transport_rows = _read_transport_status_csv(args.native_diag_dir / "wsl_transport_status.csv")
    if not transport_rows:
        reasons_fail.append("wsl_transport_status.csv is empty -- no bridge status was ever recorded")

    times = [float(r["wsl_unix_time_s"]) for r in transport_rows] if transport_rows else []
    run_start = min(times) if times else None
    run_end = max(times) if times else None

    main_rows = [
        r for r, t in zip(transport_rows, times)
        if _in_window(t, *main_window)
    ]
    if len(main_rows) == 0:
        reasons_fail.append("0 wsl_transport_status rows fall inside the specified main window -- window bounds do not overlap the recorded data")

    def _phase_metrics(rows):
        connected_flags = [r["connected"] == "True" for r in rows]
        rtts = [float(r["last_rtt_ms"]) for r in rows if r.get("last_rtt_ms") not in ("", "None", None)]
        state_ages = [float(r["last_state_age_s"]) for r in rows if r.get("last_state_age_s") not in ("", "None", None)]
        rx_counts = [int(r["rx_count"]) for r in rows if r.get("rx_count") not in ("", "None", None)]
        crc_errors = [int(r["crc_errors"]) for r in rows if r.get("crc_errors") not in ("", "None", None)]
        scan_counts = sum(int(r["scan_count_this_window"]) for r in rows if r.get("scan_count_this_window"))
        odom_counts = sum(int(r["odom_count_this_window"]) for r in rows if r.get("odom_count_this_window"))
        nonzero_cmd_vel = sum(int(r["nonzero_cmd_vel_count_this_window"]) for r in rows if r.get("nonzero_cmd_vel_count_this_window"))

        connected_fraction = (sum(connected_flags) / len(connected_flags)) if connected_flags else None
        reconnects = 0
        for i in range(1, len(connected_flags)):
            if connected_flags[i] and not connected_flags[i - 1]:
                reconnects += 1

        return {
            "sample_count": len(rows),
            "connected_fraction": connected_fraction,
            "reconnect_count": reconnects,
            "rtt_ms": _dist_stats(rtts),
            "state_age_s": _dist_stats(state_ages),
            "rx_count_delta": (rx_counts[-1] - rx_counts[0]) if len(rx_counts) >= 2 else None,
            "crc_errors_delta": (crc_errors[-1] - crc_errors[0]) if len(crc_errors) >= 2 else None,
            "scan_msgs_via_wsl_topic": scan_counts,
            "odom_msgs_via_wsl_topic": odom_counts,
            "nonzero_cmd_vel_observed": nonzero_cmd_vel,
        }

    full_run_metrics = _phase_metrics(transport_rows)
    main_window_metrics = _phase_metrics(main_rows)

    if main_window_metrics["nonzero_cmd_vel_observed"]:
        reasons_fail.append(f"nonzero /cmd_vel observed on the WSL ROS graph during the main window ({main_window_metrics['nonzero_cmd_vel_observed']} messages)")

    # rosbag-based measurements (authoritative message counts/rates; NOT
    # sequence-based, so explicitly not used to compute PDR). Bag recording
    # timestamps are rosbag2's own WSL-process wall-clock (this bag was
    # recorded by `ros2 bag record` running in WSL, same machine/clock as
    # wsl_unix_time_s -- unlike the Objective 5 sim case, there is no
    # separate sim-clock domain here), so filtering by main_window's Unix
    # seconds is a same-domain comparison.
    def _bag_stats_for_rows(rows):
        duration_s = (rows[-1][0] - rows[0][0]) / 1.0e9 if len(rows) > 1 else 0.0
        return {
            "message_count": len(rows),
            "duration_s": duration_s,
            "avg_rate_hz": (len(rows) / duration_s) if duration_s > 0 else None,
        }

    bag_topics_full = {}
    bag_topics_main = {}
    cmd_vel_nonzero_in_bag_full = 0
    cmd_vel_nonzero_in_bag_main = 0
    try:
        for topic in ("/scan", "/odom", "/epuck_bridge/status", "/cmd_vel"):
            all_rows = _read_bag_topic(args.native_bag_dir, topic)
            main_window_rows = [(ts, msg) for ts, msg in all_rows if _in_window(ts / 1.0e9, *main_window)]
            bag_topics_full[topic] = _bag_stats_for_rows(all_rows)
            bag_topics_main[topic] = _bag_stats_for_rows(main_window_rows)
            if topic == "/cmd_vel":
                for _, msg in all_rows:
                    if abs(msg.linear.x) > 1e-9 or abs(msg.angular.z) > 1e-9:
                        cmd_vel_nonzero_in_bag_full += 1
                for _, msg in main_window_rows:
                    if abs(msg.linear.x) > 1e-9 or abs(msg.angular.z) > 1e-9:
                        cmd_vel_nonzero_in_bag_main += 1
    except RuntimeError as exc:
        not_measurable.append(f"rosbag reading failed: {exc}")
    bag_topics = bag_topics_full
    cmd_vel_nonzero_in_bag = cmd_vel_nonzero_in_bag_full

    if cmd_vel_nonzero_in_bag_full:
        reasons_fail.append(f"bag contains {cmd_vel_nonzero_in_bag_full} nonzero /cmd_vel message(s) over the full run")
    if cmd_vel_nonzero_in_bag_main:
        reasons_fail.append(f"bag contains {cmd_vel_nonzero_in_bag_main} nonzero /cmd_vel message(s) within the main window")

    # PDR / sequence gap / duplicate / out-of-order: NOT_MEASURABLE.
    not_measurable.append(
        "sequence_gap_count/duplicate_count/out_of_order_count/aligned_window_pdr: "
        "the currently-running BASE wsl_epuck_tcp_bridge.py does not expose the "
        "Pi's internal per-state 'seq' field on any WSL ROS topic or in "
        "/epuck_bridge/status (it is used only for internal de-duplication in "
        "_publish_latest_state, then discarded) -- there is no paired source "
        "sequence number available to compute true PDR without modifying the "
        "bridge, which was explicitly out of scope for this pilot."
    )

    # One-way / cross-domain latency: NOT_VALID.
    not_valid.append(
        "one_way_latency_ms: no cross-device clock synchronization procedure "
        "(NTP/chrony) has been run or verified between the Pi and the laptop/WSL "
        "-- wall_clock_delta_ms exists only as a clock-offset diagnostic and is "
        "never used here to compute a one-way latency. RTT (last_rtt_ms) and "
        "last_state_age_s ARE valid, because both are computed entirely within "
        "WSL's own clock (time.monotonic()/time.time() in "
        "wsl_epuck_tcp_bridge.py) and never subtract a Pi-side timestamp from a "
        "WSL-side one."
    )

    bag_record_warnings_path = args.native_diag_dir / "bag_record_warnings.txt"
    bag_warning_lines = bag_record_warnings_path.read_text(encoding="utf-8").splitlines() if bag_record_warnings_path.exists() else []
    if bag_warning_lines:
        reasons_fail.append(f"bag_record.log contains {len(bag_warning_lines)} drop/warn/error line(s)")

    if full_run_metrics["reconnect_count"] > 0:
        reasons_fail.append(f"{full_run_metrics['reconnect_count']} unexpected reconnect(s) observed over the full run")
    if main_window_metrics["crc_errors_delta"] not in (0, None):
        reasons_fail.append(f"crc_errors increased by {main_window_metrics['crc_errors_delta']} during the main window")
    if main_window_metrics["crc_errors_delta"] is None:
        not_measurable.append("crc_errors_delta (main window): insufficient status samples in the main window")

    # Sampler health: derived from each file's OWN measured span (never a
    # fixed 300s assumption) -- a sampler that stalled produces a span with
    # a real gap (already checked above via max_gap_s) or a valid_rows
    # count far below span_s/expected_interval.
    def _sampler_row_count_ok(integrity, label):
        if integrity.get("valid_rows", 0) == 0:
            return False
        expected = integrity["span_s"] / EXPECTED_SAMPLE_INTERVAL_S
        if expected > 0 and integrity["valid_rows"] < expected * MIN_ACCEPTABLE_SAMPLE_COUNT_FRACTION:
            reasons_fail.append(
                f"{label}: {integrity['valid_rows']} samples over a {integrity['span_s']:.1f}s span "
                f"is far below the ~{expected:.0f} expected at {EXPECTED_SAMPLE_INTERVAL_S}s interval"
            )
            return False
        return True

    wsl_sampler_ok = _sampler_row_count_ok(timestamp_integrity[1], "wsl_system_sampler")
    pi_sampler_ok = (
        _sampler_row_count_ok(timestamp_integrity[2], "pi_system_sampler")
        if len(timestamp_integrity) > 2 else False
    )
    if not args.pi_metrics_csv or not args.pi_metrics_csv.exists():
        not_measurable.append("Pi-side system/Wi-Fi metrics: pi_system_metrics.csv not supplied to the analyzer")

    wsl_sys_stats_full = _system_csv_stats(args.native_diag_dir / "wsl_system_metrics.csv", ["cpu_percent", "mem_used_mb"])
    wsl_sys_stats_main = _system_csv_stats(args.native_diag_dir / "wsl_system_metrics.csv", ["cpu_percent", "mem_used_mb"], "unix_time_s", main_window)
    if args.pi_metrics_csv and args.pi_metrics_csv.exists():
        pi_columns = ["cpu_percent", "mem_used_mb", "wifi_link_quality", "wifi_signal_dbm"]
        pi_sys_stats_full = _system_csv_stats(args.pi_metrics_csv, pi_columns)
        pi_sys_stats_main = _system_csv_stats(args.pi_metrics_csv, pi_columns, "unix_time_s", main_window)
    else:
        pi_sys_stats_full = None
        pi_sys_stats_main = None

    # Naming per instruction: this pilot is DIAGNOSTIC_PHYSICAL and must
    # never be called a formal communication-performance PASS. Given the
    # base bridge structurally cannot expose paired sequence numbers,
    # PDR/sequence-gap/duplicate/out-of-order are ALWAYS NOT_MEASURABLE
    # here regardless of run quality -- that structural gap is why a
    # clean run is labeled PASS_WITH_LIMITATION rather than an
    # unqualified PASS. TRANSPORT_STABILITY_PASS is reserved for a run
    # with no NOT_MEASURABLE/NOT_VALID items at all (not achievable with
    # the current base bridge, kept here for a future bridge revision).
    if reasons_fail:
        verdict = "FAIL"
    elif not_measurable or not_valid:
        verdict = "PASS_WITH_LIMITATION"
    else:
        verdict = "TRANSPORT_STABILITY_PASS"

    result = {
        "verdict": verdict,
        "verdict_meaning": (
            "PASS_WITH_LIMITATION: base Pi-TCP-WSL transport link met all "
            "measurable stability criteria, but this is NOT a formal "
            "communication-performance PASS -- PDR/sequence integrity are "
            "structurally NOT_MEASURABLE with the current base bridge, and "
            "no cross-device clock sync means no one-way latency is valid."
            if verdict == "PASS_WITH_LIMITATION" else
            "TRANSPORT_STABILITY_PASS: all measurable criteria passed with no "
            "NOT_MEASURABLE/NOT_VALID items."
            if verdict == "TRANSPORT_STABILITY_PASS" else
            "FAIL: see fail_reasons."
        ),
        "experiment_kind": "DIAGNOSTIC_PHYSICAL",
        "scope_note": (
            "This pilot validates ONLY the base Pi-TCP-WSL transport link. It "
            "does NOT validate the current EpuckState protocol (no converter "
            "exists yet) and does NOT validate avoidance behaviour (no "
            "controller was run)."
        ),
        "wsl_sampler_healthy": wsl_sampler_ok,
        "pi_sampler_healthy": pi_sampler_ok,
        "fail_reasons": reasons_fail,
        "not_measurable": not_measurable,
        "not_valid": not_valid,
        "timestamp_integrity": timestamp_integrity,
        "run_window": {
            "run_start_unix_s": run_start, "run_end_unix_s": run_end,
            "main_window_start_unix_s": args.main_window_start,
            "main_window_end_unix_s": args.main_window_end,
            "main_window_span_s": args.main_window_end - args.main_window_start,
            "note": "main_window bounds were computed from the REAL measured overlap of Pi/WSL/bag timestamps, not a fixed offset assumption -- see the overlap-window calculation in the session record.",
        },
        "full_run": full_run_metrics,
        "main_window": main_window_metrics,
        "bag_topics_full_run": bag_topics_full,
        "bag_topics_main_window": bag_topics_main,
        "cmd_vel_nonzero_in_bag_full_run": cmd_vel_nonzero_in_bag_full,
        "cmd_vel_nonzero_in_bag_main_window": cmd_vel_nonzero_in_bag_main,
        "bag_record_warning_lines": bag_warning_lines[:20],
        "wsl_system_metrics_full_run": wsl_sys_stats_full,
        "wsl_system_metrics_main_window": wsl_sys_stats_main,
        "pi_system_metrics_full_run": pi_sys_stats_full,
        "pi_system_metrics_main_window": pi_sys_stats_main,
    }

    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "verdict.json").write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Analyzer for
physical_expanded_bridge_epuckstate_integration_pilot01_attempt01.

DIAGNOSTIC_PHYSICAL only. Computes three EXPLICITLY SEPARATE sequence/
delivery tiers, never conflated:

  A. Pi -> WSL application-level state transport, from the expanded
     bridge's own /epuck_bridge/status fields (state_seq_first/last,
     state_unique_received, state_missing, state_out_of_order). Reported
     as APPLICATION_STATE_SEQUENCE_DELIVERY_RATIO (never "PDR", "IP packet
     loss", or "TCP packet loss"). NOTE: the bridge code
     (update_sequence_stats() in wsl_epuck_tcp_bridge_sensors.py, read
     directly) does NOT separately track "duplicate" from "out_of_order"
     -- any received sequence number <= the last-seen one increments
     out_of_order_count, whether it was a true duplicate or a genuine
     reorder. This analyzer does not fabricate a separate duplicate count
     that the underlying tool does not actually produce.

  B. state_publisher's own EpuckState.sequence, as CAPTURED BY THE BAG.
     This measures whether the bag missed any state_publisher-generated
     message; it does NOT measure Pi-to-WSL transport loss (tier A does).

  C. Raw sensor topics (/odom, /scan, /tof, /ps0-7): message counts, rates,
     max inter-arrival gap, total stall time. No source-side sequence
     exists for these, so no PDR is claimed for them at all.

RTT/state_age are read directly from the bridge's own status; one-way
Pi-to-WSL latency is never computed (no clock-sync procedure has been
run/verified).
"""
import argparse
import csv
import json
import math
import statistics
from pathlib import Path


EXPECTED_SAMPLE_INTERVAL_S = 1.0
MAX_ACCEPTABLE_SAMPLE_GAP_S = 3.0


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
        return {"sample_count": 0, "mean": None, "median": None, "p95": None, "p99": None, "max": None, "min": None}
    sorted_values = sorted(values)
    return {
        "sample_count": len(values),
        "mean": statistics.fmean(values),
        "median": _percentile(sorted_values, 0.50),
        "p95": _percentile(sorted_values, 0.95),
        "p99": _percentile(sorted_values, 0.99),
        "max": sorted_values[-1],
        "min": sorted_values[0],
    }


def _timestamp_integrity(path: Path, time_col: str, label: str):
    if not path.exists():
        return {"label": label, "path": str(path), "valid_rows": 0, "exists": False}
    times = []
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
        "label": label, "path": str(path), "valid_rows": len(times),
        "monotonic_non_decreasing": monotonic_non_decreasing,
        "duplicate_timestamp_count": duplicate_count,
        "first_time_unix_s": times[0], "last_time_unix_s": times[-1],
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


def _tier_a_app_delivery(status_rows_in_window):
    """Reads the bridge's OWN already-computed cumulative counters directly
    from the LAST status row in the window (they are cumulative since
    connection start, not per-tick deltas) -- never recomputed here."""
    if not status_rows_in_window:
        return None
    last = status_rows_in_window[-1]
    def _int_or_none(v):
        try:
            return int(v)
        except (TypeError, ValueError):
            return None
    def _float_or_none(v):
        try:
            return float(v)
        except (TypeError, ValueError):
            return None
    return {
        "state_seq_first": _int_or_none(last.get("state_seq_first")),
        "state_seq_last": _int_or_none(last.get("state_seq_last")),
        "state_unique_received": _int_or_none(last.get("state_unique_received")),
        "state_missing": _int_or_none(last.get("state_missing")),
        "state_out_of_order_INCLUDES_ANY_DUPLICATE": _int_or_none(last.get("state_out_of_order")),
        "duplicate_count_SEPARATELY_TRACKED": None,
        "duplicate_note": "the bridge's own update_sequence_stats() does not distinguish a true duplicate from a generic out-of-order/backward arrival -- both increment the same counter. Not fabricated here.",
        "APPLICATION_STATE_SEQUENCE_DELIVERY_RATIO": _float_or_none(last.get("state_delivery_ratio")),
        "naming_note": "this is Pi application-level state-sequence receipt completeness at the WSL bridge. It is NOT IP packet loss, NOT TCP packet loss, and is NOT automatically equal to the EpuckState bag capture ratio (tier B) or any rosbag PDR.",
    }


def _tier_b_bag_capture(bag_epuckstate_rows):
    """EpuckState.sequence continuity as captured by the bag -- measures
    bag-capture completeness of state_publisher's OWN output stream, not
    Pi-to-WSL transport (that's tier A)."""
    if not bag_epuckstate_rows:
        return {"message_count": 0}
    seqs = [int(msg.sequence) for _, msg in bag_epuckstate_rows]
    first_seq, last_seq = seqs[0], seqs[-1]
    seen = {}
    duplicate_count = 0
    out_of_order_count = 0
    previous = None
    for s in seqs:
        seen[s] = seen.get(s, 0) + 1
        if previous is not None and s < previous:
            out_of_order_count += 1
        previous = s
    duplicate_count = sum(c - 1 for c in seen.values() if c > 1)
    unique = len(seen)
    span = (last_seq - first_seq) % (2**32)
    expected = span + 1
    capture_ratio = unique / expected if expected > 0 else None
    return {
        "message_count": len(seqs),
        "first_sequence": first_seq,
        "last_sequence": last_seq,
        "unique_sequence_count": unique,
        "expected_sequence_count": expected,
        "gap_count": max(0, expected - unique),
        "duplicate_count": duplicate_count,
        "out_of_order_count": out_of_order_count,
        "bag_capture_ratio": capture_ratio,
        "note": "measures whether the bag captured every state_publisher-generated EpuckState message; does NOT measure Pi-to-WSL bridge transport loss (see tier A)",
    }


def _tier_c_raw_sensor(bag_dir: Path, topic: str, window):
    all_rows = _read_bag_topic(bag_dir, topic)
    window_rows = [(ts, msg) for ts, msg in all_rows if _in_window(ts / 1.0e9, *window)]
    if len(window_rows) < 2:
        return {"message_count": len(window_rows), "avg_rate_hz": None, "max_gap_s": None, "total_stall_s": None,
                "note": "no source-side sequence exists for this topic; PDR is not claimed"}
    times_s = [ts / 1.0e9 for ts, _ in window_rows]
    duration = times_s[-1] - times_s[0]
    gaps = [b - a for a, b in zip(times_s, times_s[1:])]
    expected_gap = duration / (len(times_s) - 1) if len(times_s) > 1 else None
    stall_threshold = (expected_gap * 3) if expected_gap else 1.0
    total_stall_s = sum(g for g in gaps if g > stall_threshold)
    return {
        "message_count": len(window_rows),
        "duration_s": duration,
        "avg_rate_hz": (len(window_rows) / duration) if duration > 0 else None,
        "max_gap_s": max(gaps) if gaps else None,
        "total_stall_s": total_stall_s,
        "note": "no source-side sequence exists for this topic; PDR is not claimed",
    }


def _validity_flags_durations(bag_epuckstate_rows):
    """Time-weighted duration at each distinct validity_flags value, using
    consecutive bag-record timestamp deltas as the duration each value was
    held (last sample's own duration is not counted, has no successor)."""
    if len(bag_epuckstate_rows) < 2:
        return {}
    durations = {}
    for (ts_a, msg_a), (ts_b, _) in zip(bag_epuckstate_rows, bag_epuckstate_rows[1:]):
        flags = int(msg_a.validity_flags)
        dt = (ts_b - ts_a) / 1.0e9
        durations[flags] = durations.get(flags, 0.0) + dt
    return {str(k): v for k, v in sorted(durations.items())}


def _nan_inf_accounting(bag_epuckstate_rows):
    """+Inf in the six v4 zone fields and front/left/right_distance_m is
    protocol-allowed (documented 'no detection' convention). NaN anywhere,
    or +/-Inf in any OTHER field, is treated as an anomaly."""
    allowed_inf_fields = (
        "front_distance_m", "left_distance_m", "right_distance_m",
        "left_front_m", "left_mid_m", "left_rear_m",
        "right_front_m", "right_mid_m", "right_rear_m",
    )
    other_float_fields = ("x_m", "y_m", "yaw_rad", "linear_velocity_mps", "angular_velocity_rps")
    allowed_inf_count = 0
    nan_count = 0
    unexpected_inf_count = 0
    nan_locations = []
    for _, msg in bag_epuckstate_rows:
        for field in allowed_inf_fields:
            v = getattr(msg, field)
            if math.isnan(v):
                nan_count += 1
                nan_locations.append((int(msg.sequence), field))
            elif math.isinf(v):
                allowed_inf_count += 1
        for field in other_float_fields:
            v = getattr(msg, field)
            if math.isnan(v):
                nan_count += 1
                nan_locations.append((int(msg.sequence), field))
            elif math.isinf(v):
                unexpected_inf_count += 1
    return {
        "protocol_allowed_inf_count": allowed_inf_count,
        "nan_count": nan_count,
        "nan_sample_locations": nan_locations[:20],
        "unexpected_inf_count_in_pose_velocity_fields": unexpected_inf_count,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--native-bag-dir", type=Path, required=True)
    parser.add_argument("--native-diag-dir", type=Path, required=True)
    parser.add_argument("--pi-metrics-csv", type=Path, required=False)
    parser.add_argument("--main-window-start", type=float, required=True)
    parser.add_argument("--main-window-end", type=float, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    main_window = (args.main_window_start, args.main_window_end)

    reasons_fail = []
    not_measurable = []
    not_valid = []

    timestamp_integrity = [
        _timestamp_integrity(args.native_diag_dir / "wsl_expanded_status.csv", "wsl_unix_time_s", "wsl_expanded_status"),
        _timestamp_integrity(args.native_diag_dir / "wsl_system_metrics.csv", "unix_time_s", "wsl_system_metrics"),
    ]
    if args.pi_metrics_csv:
        timestamp_integrity.append(_timestamp_integrity(args.pi_metrics_csv, "unix_time_s", "pi_system_metrics"))
    for integrity in timestamp_integrity:
        if integrity.get("valid_rows", 0) == 0:
            continue
        if not integrity["monotonic_non_decreasing"]:
            reasons_fail.append(f"{integrity['label']}: timestamps not monotonic")
        if integrity["duplicate_timestamp_count"] > 0:
            reasons_fail.append(f"{integrity['label']}: {integrity['duplicate_timestamp_count']} duplicate timestamp(s)")
        if integrity["max_gap_s"] > MAX_ACCEPTABLE_SAMPLE_GAP_S:
            reasons_fail.append(f"{integrity['label']}: max sample gap {integrity['max_gap_s']:.2f}s exceeds {MAX_ACCEPTABLE_SAMPLE_GAP_S}s")

    status_rows = []
    status_path = args.native_diag_dir / "wsl_expanded_status.csv"
    if status_path.exists():
        with status_path.open(encoding="utf-8") as fh:
            status_rows = list(csv.DictReader(fh))
    if not status_rows:
        reasons_fail.append("wsl_expanded_status.csv is empty")

    status_main = [r for r in status_rows if _in_window(float(r["wsl_unix_time_s"]), *main_window)]
    if not status_main:
        reasons_fail.append("0 status rows fall inside the main window")

    connected_flags = [r["connected"] == "True" for r in status_main]
    connected_fraction = (sum(connected_flags) / len(connected_flags)) if connected_flags else None
    reconnects = sum(1 for i in range(1, len(connected_flags)) if connected_flags[i] and not connected_flags[i - 1])
    if reconnects > 0:
        reasons_fail.append(f"{reconnects} unexpected reconnect(s) in the main window")

    crc_vals = [int(r["crc_errors"]) for r in status_main if r.get("crc_errors") not in ("", "None", None)]
    crc_delta = (crc_vals[-1] - crc_vals[0]) if len(crc_vals) >= 2 else None
    if crc_delta not in (0, None):
        reasons_fail.append(f"crc_errors increased by {crc_delta} in the main window")

    rtts = [float(r["last_rtt_ms"]) for r in status_main if r.get("last_rtt_ms") not in ("", "None", None)]
    rtt_stats = _dist_stats(rtts)
    rtt_tail = {}
    if rtts:
        rtt_tail = {
            "count_gt_50ms": sum(1 for v in rtts if v > 50), "percent_gt_50ms": 100.0 * sum(1 for v in rtts if v > 50) / len(rtts),
            "count_gt_100ms": sum(1 for v in rtts if v > 100), "percent_gt_100ms": 100.0 * sum(1 for v in rtts if v > 100) / len(rtts),
            "count_gt_200ms": sum(1 for v in rtts if v > 200), "percent_gt_200ms": 100.0 * sum(1 for v in rtts if v > 200) / len(rtts),
        }
        longest_run = cur_run = 0
        for r in status_main:
            v = r.get("last_rtt_ms")
            v = float(v) if v not in ("", "None", None) else None
            if v is not None and v > 50:
                cur_run += 1
                longest_run = max(longest_run, cur_run)
            else:
                cur_run = 0
        rtt_tail["longest_consecutive_status_ticks_gt_50ms"] = longest_run

    state_ages = [float(r["last_state_age_s"]) for r in status_main if r.get("last_state_age_s") not in ("", "None", None)]
    state_age_stats = _dist_stats(state_ages)

    tier_a = _tier_a_app_delivery(status_main)
    if tier_a:
        if tier_a["state_missing"] not in (0, None):
            reasons_fail.append(f"tier A: state_missing={tier_a['state_missing']} (expected 0)")
        if tier_a["state_out_of_order_INCLUDES_ANY_DUPLICATE"] not in (0, None):
            reasons_fail.append(f"tier A: state_out_of_order={tier_a['state_out_of_order_INCLUDES_ANY_DUPLICATE']} (expected 0)")
        if tier_a["APPLICATION_STATE_SEQUENCE_DELIVERY_RATIO"] not in (1.0, None):
            reasons_fail.append(f"tier A: APPLICATION_STATE_SEQUENCE_DELIVERY_RATIO={tier_a['APPLICATION_STATE_SEQUENCE_DELIVERY_RATIO']} (expected 1.0)")

    # Bag-based tiers B and C, plus EpuckState field/protocol checks.
    bag_epuckstate_all = _read_bag_topic(args.native_bag_dir, "/epuck1/state")
    bag_epuckstate_main = [(ts, msg) for ts, msg in bag_epuckstate_all if _in_window(ts / 1.0e9, *main_window)]
    tier_b = _tier_b_bag_capture(bag_epuckstate_main)
    if bag_epuckstate_main:
        if tier_b.get("gap_count", 0) != 0:
            reasons_fail.append(f"tier B: EpuckState bag gap_count={tier_b['gap_count']} (expected 0)")
        if tier_b.get("duplicate_count", 0) != 0:
            reasons_fail.append(f"tier B: EpuckState bag duplicate_count={tier_b['duplicate_count']} (expected 0)")
        if tier_b.get("out_of_order_count", 0) != 0:
            reasons_fail.append(f"tier B: EpuckState bag out_of_order_count={tier_b['out_of_order_count']} (expected 0)")
        if tier_b.get("bag_capture_ratio") not in (1.0, None):
            reasons_fail.append(f"tier B: bag_capture_ratio={tier_b.get('bag_capture_ratio')} (expected 1.0)")
    else:
        not_measurable.append("tier B: no /epuck1/state messages in the main window")

    # version/source/robot_id/validity_flags/NaN checks (from the bag, main window).
    field_errors = []
    for ts, msg in bag_epuckstate_main:
        if int(msg.version) != 1:
            field_errors.append(f"version={msg.version} at seq={msg.sequence}")
        if int(msg.source) != 2:
            field_errors.append(f"source={msg.source} (expected 2=SOURCE_HARDWARE) at seq={msg.sequence}")
        if int(msg.robot_id) != 1:
            field_errors.append(f"robot_id={msg.robot_id} at seq={msg.sequence}")
    if field_errors:
        reasons_fail.append(f"{len(field_errors)} EpuckState field error(s), e.g. {field_errors[:5]}")

    validity_durations = _validity_flags_durations(bag_epuckstate_main)
    nan_inf = _nan_inf_accounting(bag_epuckstate_main)
    if nan_inf["nan_count"] > 0:
        reasons_fail.append(f"{nan_inf['nan_count']} NaN value(s) found in EpuckState fields")
    if nan_inf["unexpected_inf_count_in_pose_velocity_fields"] > 0:
        reasons_fail.append(f"{nan_inf['unexpected_inf_count_in_pose_velocity_fields']} unexpected +/-Inf in pose/velocity fields")

    # Tier C: raw sensor topics.
    tier_c = {}
    for topic in ("/odom", "/scan", "/tof", "/ps0", "/ps1", "/ps2", "/ps3", "/ps4", "/ps5", "/ps6", "/ps7"):
        tier_c[topic] = _tier_c_raw_sensor(args.native_bag_dir, topic, main_window)

    # cmd_vel: bag-based nonzero check + checkpoint publisher counts.
    cmd_vel_rows = _read_bag_topic(args.native_bag_dir, "/cmd_vel")
    cmd_vel_nonzero = sum(1 for _, msg in cmd_vel_rows if abs(msg.linear.x) > 1e-9 or abs(msg.angular.z) > 1e-9)
    if cmd_vel_nonzero:
        reasons_fail.append(f"{cmd_vel_nonzero} nonzero /cmd_vel message(s) in the bag")

    checkpoints_path = args.native_diag_dir / "cmd_vel_checkpoints.json"
    cmd_vel_checkpoints = json.loads(checkpoints_path.read_text(encoding="utf-8")) if checkpoints_path.exists() else []
    for cp in cmd_vel_checkpoints:
        if cp.get("publisher_count", -1) != 0:
            reasons_fail.append(f"cmd_vel checkpoint '{cp.get('label')}': publisher_count={cp.get('publisher_count')} (expected 0)")
    if len(cmd_vel_checkpoints) < 3:
        not_measurable.append(f"only {len(cmd_vel_checkpoints)}/3 cmd_vel checkpoints recorded (start/mid/end)")

    bag_record_warnings_path = args.native_diag_dir / "bag_record_warnings.txt"
    bag_warning_lines = bag_record_warnings_path.read_text(encoding="utf-8").splitlines() if bag_record_warnings_path.exists() else []
    if bag_warning_lines:
        reasons_fail.append(f"bag_record.log contains {len(bag_warning_lines)} drop/warn/error line(s)")

    not_valid.append(
        "one_way_latency_ms (Pi sensor acquisition -> WSL receipt/publish): NOT COMPUTED. "
        "No NTP/chrony clock-sync procedure has been run or verified between the Pi and "
        "WSL/laptop. RTT and last_state_age_s remain VALID (single WSL clock domain, see "
        "prior pilot's documented formula)."
    )
    not_measurable.append(
        "tier A duplicate_count: the expanded bridge's own update_sequence_stats() does not "
        "separately track duplicates from generic out-of-order arrivals -- both increment "
        "the same out_of_order counter. Not fabricated as a separate number here."
    )

    wsl_sys_stats_main = _system_csv_stats(args.native_diag_dir / "wsl_system_metrics.csv", ["cpu_percent", "mem_used_mb"], "unix_time_s", main_window)
    if args.pi_metrics_csv and args.pi_metrics_csv.exists():
        pi_columns = ["cpu_percent", "mem_used_mb", "wifi_link_quality", "wifi_signal_dbm"]
        pi_sys_stats_main = _system_csv_stats(args.pi_metrics_csv, pi_columns, "unix_time_s", main_window)
    else:
        pi_sys_stats_main = None
        not_measurable.append("Pi-side system/Wi-Fi metrics: pi_system_metrics.csv not supplied")

    if reasons_fail:
        verdict = "FAIL"
    elif not_measurable or not_valid:
        verdict = "PASS_WITH_LIMITATION"
    else:
        verdict = "TRANSPORT_STABILITY_PASS"

    result = {
        "verdict": verdict,
        "experiment_kind": "DIAGNOSTIC_PHYSICAL",
        "scope_note": "Expanded bridge + EpuckState stationary integration check. NOT a formal baseline. No ground motion, no controller.",
        "fail_reasons": reasons_fail,
        "not_measurable": not_measurable,
        "not_valid": not_valid,
        "timestamp_integrity": timestamp_integrity,
        "run_window": {
            "main_window_start_unix_s": args.main_window_start,
            "main_window_end_unix_s": args.main_window_end,
            "main_window_span_s": args.main_window_end - args.main_window_start,
        },
        "connected_fraction": connected_fraction,
        "reconnect_count": reconnects,
        "crc_errors_delta": crc_delta,
        "rtt_status_snapshot_ms": {**rtt_stats, "tail": rtt_tail, "provenance": "1Hz /epuck_bridge/status snapshot of most recently completed individual RTT, not a full transaction census"},
        "state_age_s": {**state_age_stats, "formula": "time.time() - self._last_state_time, both WSL time.time() calls, single clock domain, VALID"},
        "tier_a_pi_to_wsl_app_delivery": tier_a,
        "tier_b_state_publisher_to_bag_capture": tier_b,
        "tier_c_raw_sensor_topics": tier_c,
        "epuckstate_field_errors": field_errors[:20],
        "validity_flags_durations_s": validity_durations,
        "nan_inf_accounting": nan_inf,
        "cmd_vel_nonzero_in_bag": cmd_vel_nonzero,
        "cmd_vel_checkpoints": cmd_vel_checkpoints,
        "bag_record_warning_lines": bag_warning_lines[:20],
        "wsl_system_metrics_main_window": wsl_sys_stats_main,
        "pi_system_metrics_main_window": pi_sys_stats_main,
    }

    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "verdict.json").write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Final (post-batch) per-trial analysis: folds the batch-level Pi raw
metrics CSV in as the 4th source alongside bag/status_csv/system_csv,
recomputes the real 4-source overlap and centered 240.000s main window
(never reusing the earlier 3-source-only window from the
THREE_SOURCE_PROVISIONAL_PASS pass), slices the Pi CSV to that exact
window, re-runs the tier-A delta + tier-B/C/field/cmd_vel/NaN/CRC/
reconnect/warning analyzer against the (possibly slightly different)
4-source main window, and issues the trial's FINAL_PASS / FINAL_FAIL
verdict. No trial in this batch is FINAL_PASS until this script has run
for it with the real Pi CSV in hand.
"""
import argparse
import csv
import json
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from window_calc import evaluate_window
from compute_tier_a_delta import compute_delta, load_snapshot_json
from slice_pi_metrics import slice_pi_metrics

import yaml


def csv_first_last(path: Path, time_col: str):
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
        return None, None
    return times[0], times[-1]


def bag_start_end(bag_dir: Path):
    meta = yaml.safe_load((bag_dir / "metadata.yaml").read_text(encoding="utf-8"))
    info = meta["rosbag2_bagfile_information"]
    start_ns = info["starting_time"]["nanoseconds_since_epoch"]
    duration_ns = info["duration"]["nanoseconds"]
    return start_ns / 1e9, (start_ns + duration_ns) / 1e9


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trial", required=True)
    parser.add_argument("--bag-dir", type=Path, required=True)
    parser.add_argument("--diag-dir", type=Path, required=True)
    parser.add_argument("--analysis-dir", type=Path, required=True)
    parser.add_argument("--analyzer-script", type=Path, required=True)
    parser.add_argument("--pi-batch-csv", type=Path, required=True)
    parser.add_argument("--existing-manifest", type=Path, required=True,
                         help="the THREE_SOURCE_PROVISIONAL_PASS-stage runtime_manifest.json, carried forward and extended")
    parser.add_argument("--trial-log", type=Path, required=True)
    args = parser.parse_args()

    fail_reasons = []

    bag_start, bag_end = bag_start_end(args.bag_dir)
    status_first, status_last = csv_first_last(args.diag_dir / "wsl_expanded_status.csv", "wsl_unix_time_s")
    sys_first, sys_last = csv_first_last(args.diag_dir / "wsl_system_metrics.csv", "unix_time_s")
    pi_first, pi_last = csv_first_last(args.pi_batch_csv, "unix_time_s")

    sources = {
        "bag": (bag_start, bag_end),
        "status_csv": (status_first, status_last),
        "system_csv": (sys_first, sys_last),
        "pi_batch_csv": (pi_first, pi_last),
    }
    window = evaluate_window(sources)
    if window["verdict"] != "OK":
        fail_reasons.append(f"4-source window: SHORT_WINDOW, overlap={window['common_overlap_span_s']:.3f}s < {window['required_total_s']:.3f}s required")

    tier_a_delta = None
    if (args.diag_dir / "bridge_status_trial_start.json").exists():
        start_snapshot = load_snapshot_json(str(args.diag_dir / "bridge_status_trial_start.json"))
        end_snapshot = load_snapshot_json(str(args.diag_dir / "bridge_status_trial_end.json"))
        tier_a_delta = compute_delta(start_snapshot, end_snapshot)
        if tier_a_delta["trial_state_missing_delta"] != 0:
            fail_reasons.append(f"tier A delta: state_missing_delta={tier_a_delta['trial_state_missing_delta']} (expected 0)")
        if tier_a_delta["trial_state_out_of_order_delta"] != 0:
            fail_reasons.append(f"tier A delta: state_out_of_order_delta={tier_a_delta['trial_state_out_of_order_delta']} (expected 0)")
        if tier_a_delta["APPLICATION_STATE_SEQUENCE_DELIVERY_RATIO_delta"] not in (1.0, None):
            fail_reasons.append(f"tier A delta: ratio={tier_a_delta['APPLICATION_STATE_SEQUENCE_DELIVERY_RATIO_delta']} (expected 1.0)")
        if tier_a_delta["trial_crc_errors_delta"] != 0:
            fail_reasons.append(f"tier A delta: crc_errors_delta={tier_a_delta['trial_crc_errors_delta']} (expected 0)")

    analyzer_result = None
    pi_slice = None
    if window["verdict"] == "OK":
        args.analysis_dir.mkdir(parents=True, exist_ok=True)
        cmd = [
            sys.executable, str(args.analyzer_script),
            "--native-bag-dir", str(args.bag_dir),
            "--native-diag-dir", str(args.diag_dir),
            "--pi-metrics-csv", "__UNUSED__",  # placeholder, overwritten below with real slice path once known
            "--main-window-start", str(window["main_window_start_unix_s"]),
            "--main-window-end", str(window["main_window_end_unix_s"]),
            "--output-dir", str(args.analysis_dir),
        ]

        pi_window_csv = args.analysis_dir / "pi_system_metrics_window.csv"
        pi_slice = slice_pi_metrics(
            args.pi_batch_csv,
            window["main_window_start_unix_s"],
            window["main_window_end_unix_s"],
            pi_window_csv,
        )

        cmd[cmd.index("__UNUSED__")] = str(pi_window_csv)
        proc = subprocess.run(cmd, capture_output=True, text=True)
        if proc.returncode != 0:
            fail_reasons.append(f"analyzer subprocess failed: {proc.stderr[-500:]}")
        else:
            analyzer_result = json.loads((args.analysis_dir / "verdict.json").read_text(encoding="utf-8"))
            for reason in analyzer_result.get("fail_reasons", []):
                fail_reasons.append(f"analyzer: {reason}")

    trial_log_text = args.trial_log.read_text(encoding="utf-8", errors="replace") if args.trial_log.exists() else ""
    recorder_traceback = "Traceback" in trial_log_text
    if recorder_traceback:
        fail_reasons.append("recorder/orchestrator log contains a Traceback")

    verdict = "FINAL_FAIL" if fail_reasons else "FINAL_PASS"

    epuckstate_actual_hz = None
    if analyzer_result:
        tb = analyzer_result.get("tier_b_state_publisher_to_bag_capture", {})
        msg_count = tb.get("message_count")
        span = window["main_window_end_unix_s"] - window["main_window_start_unix_s"]
        if msg_count and span:
            epuckstate_actual_hz = msg_count / span

    result = {
        "trial": args.trial,
        "verdict": verdict,
        "scope": "physical_single_device_zero_impairment_baseline_v1 -- same driver/expanded-server/WSL-bridge continuous session, 5 SEPARATE measurement windows, not 5 independent cold-start sessions",
        "fail_reasons": fail_reasons,
        "window_audit_4source": window,
        "tier_a_delta": tier_a_delta,
        "pi_metrics_slice": pi_slice,
        "epuckstate_actual_hz_main_window": epuckstate_actual_hz,
        "recorder_traceback_observed": recorder_traceback,
        "analyzer_verdict": analyzer_result,
        "one_way_latency_note": "Pi-to-WSL one-way latency is NOT reported -- no NTP/chrony clock-sync procedure between Pi and WSL has been verified. RTT and state_age_s remain valid (single WSL clock domain).",
        "tier_a_semantics_note": "APPLICATION_STATE_SEQUENCE_DELIVERY_RATIO is Pi application-level state-sequence receipt completeness, NOT IP/TCP packet loss.",
    }
    (args.analysis_dir / "final_verdict.json").write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    existing_manifest = json.loads(args.existing_manifest.read_text(encoding="utf-8"))
    manifest = {
        **existing_manifest,
        "window_audit_4source": window,
        "tier_a_delta": tier_a_delta,
        "pi_metrics_slice_provenance": pi_slice,
        "final_verdict": verdict,
        "final_fail_reasons": fail_reasons,
    }
    (args.analysis_dir / "runtime_manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    print(json.dumps({"trial": args.trial, "verdict": verdict, "fail_reasons": fail_reasons,
                       "window_span_s": window["common_overlap_span_s"], "epuckstate_actual_hz": epuckstate_actual_hz}))
    sys.exit(0 if verdict == "FINAL_PASS" else 1)


if __name__ == "__main__":
    main()

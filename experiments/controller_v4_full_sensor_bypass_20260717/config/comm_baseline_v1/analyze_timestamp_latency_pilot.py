#!/usr/bin/env python3
"""objective5_timestamp_latency_validation_pilot01 acceptance checks.

Compares condition A (delay=0) against condition B (delay=0.25s, same
scenario, same seed) using sequence_counter's own LIVE latency
measurement (message.stamp vs the counter node's own get_clock().now()
at receipt -- both under use_sim_time=true, same clock domain, so this
does not have the bag-record-clock-vs-sim-time mismatch that
analyze_comm_performance.py's bag-based age computation has). Never
recomputes PDR/sequence stats -- reads them from analyze_comm_performance
and the relay CSV, which were already run against each condition's own
bag. Read-only; diagnostic-only (no cooperative_avoider was launched in
either condition, so this is explicitly not formal Objective5 evidence).
"""
import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyze_measurement_chain import bag_topic_stats, read_relay_csv  # noqa: E402

NAMESPACES = ("epuck1", "epuck2")
LATENCY_INCREMENT_TOLERANCE_S = 0.05
ZERO_DELAY_MAX_PLAUSIBLE_AGE_S = 1.0


def _load_condition(bag_dir: Path, diag_dir: Path):
    config = json.loads((diag_dir / "pilot_condition_config.json").read_text(encoding="utf-8"))
    per_robot = {}
    for ns in NAMESPACES:
        counter_path = diag_dir / f"{ns}_counter.json"
        counter = json.loads(counter_path.read_text(encoding="utf-8")) if counter_path.exists() else {}
        counter_state = counter.get("state", {})
        relay = read_relay_csv(diag_dir / f"{ns}_relay.csv")
        bag_raw = bag_topic_stats(bag_dir, f"/{ns}/state_raw")
        bag_relayed = bag_topic_stats(bag_dir, f"/{ns}/state")

        # Aligned-window PDR, same principle as
        # analyze_objective5_formal_baseline.py: the recorder's DDS
        # subscription needs a moment to match after ros2 bag record
        # starts, so a handful of early messages (published before the
        # subscription matched) are legitimately outside the bag's own
        # observed window and must not be counted as "dropped" -- a naive
        # bag_count/relay_forwarded_count ratio from sequence 0 would
        # otherwise report a false ~3% loss that has nothing to do with
        # delay/latency behaviour.
        bag_relayed_seqs = bag_relayed["sequences"]
        relay_seqs = relay["sequences_forwarded"]
        window_relay = (
            {s for s in relay_seqs if (bag_relayed["first_sequence"] or 0) <= s <= (bag_relayed["last_sequence"] or -1)}
            if bag_relayed_seqs else set()
        )
        aligned_pdr = (len(window_relay & bag_relayed_seqs) / len(window_relay)) if window_relay else None

        per_robot[ns] = {
            "counter_complete": counter.get("complete"),
            "counter_state": counter_state,
            "relay_forwarded_count": relay["forwarded"],
            "bag_raw_unique_sequence_count": bag_raw["unique_sequence_count"],
            "bag_relayed_unique_sequence_count": bag_relayed["unique_sequence_count"],
            "aligned_window_pdr": aligned_pdr,
        }
    return config, per_robot


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--condition-a-bag-dir", type=Path, required=True)
    parser.add_argument("--condition-a-diag-dir", type=Path, required=True)
    parser.add_argument("--condition-b-bag-dir", type=Path, required=True)
    parser.add_argument("--condition-b-diag-dir", type=Path, required=True)
    parser.add_argument("--output-path", type=Path, required=True)
    args = parser.parse_args()

    reasons_fail = []
    config_a, robots_a = _load_condition(args.condition_a_bag_dir, args.condition_a_diag_dir)
    config_b, robots_b = _load_condition(args.condition_b_bag_dir, args.condition_b_diag_dir)

    per_robot_report = {}
    for ns in NAMESPACES:
        a, b = robots_a[ns], robots_b[ns]
        ca, cb = a["counter_state"], b["counter_state"]

        if a["counter_complete"] is not True:
            reasons_fail.append(f"{ns}/condition_a: counter complete flag is not true")
        if b["counter_complete"] is not True:
            reasons_fail.append(f"{ns}/condition_b: counter complete flag is not true")

        for label, c in (("condition_a", ca), ("condition_b", cb)):
            if c.get("negative_age_sample_count", 0) != 0:
                reasons_fail.append(f"{ns}/{label}: {c['negative_age_sample_count']} negative-age samples (stamp after receipt)")
            if c.get("anomalous_age_sample_count", 0) != 0:
                reasons_fail.append(f"{ns}/{label}: {c['anomalous_age_sample_count']} anomalous-age samples (clock-domain-mismatch symptom)")
            if not c.get("valid_age_sample_count", 0):
                reasons_fail.append(f"{ns}/{label}: zero valid age samples -- cannot validate stamp/latency at all")

        mean_a = ca.get("mean_message_age_s")
        mean_b = cb.get("mean_message_age_s")
        if mean_a is None or mean_b is None:
            reasons_fail.append(f"{ns}: cannot compute observed latency increment, mean_message_age_s missing in one condition")
            observed_increment_s = None
            increment_error_s = None
        else:
            if mean_a > ZERO_DELAY_MAX_PLAUSIBLE_AGE_S:
                reasons_fail.append(f"{ns}/condition_a: mean_message_age_s={mean_a:.4f}s is not a small positive value at zero configured delay")
            observed_increment_s = mean_b - mean_a
            configured_increment_s = config_b["configured_delay_s"] - config_a["configured_delay_s"]
            increment_error_s = observed_increment_s - configured_increment_s
            if abs(increment_error_s) > LATENCY_INCREMENT_TOLERANCE_S:
                reasons_fail.append(
                    f"{ns}: observed latency increment {observed_increment_s:.4f}s vs configured "
                    f"{configured_increment_s:.4f}s differs by {increment_error_s:.4f}s "
                    f"(> {LATENCY_INCREMENT_TOLERANCE_S}s tolerance)"
                )

        # sequence consistency between the two live, from-sequence-0
        # observers (relay CSV and sequence_counter) -- both subscribe at
        # the same early point, so these should match closely. The bag is
        # compared separately via aligned-window PDR (below), NOT folded
        # into this same-start-point check, because the recorder's DDS
        # subscription genuinely needs a moment to match after `ros2 bag
        # record` starts and legitimately observes a later window than
        # relay/counter -- see aligned_window_pdr's docstring note.
        for label, robot in (("condition_a", a), ("condition_b", b)):
            counter_unique = ca.get("unique_sequence_count") if label == "condition_a" else cb.get("unique_sequence_count")
            spread = abs(counter_unique - robot["relay_forwarded_count"])
            if spread > 2:
                reasons_fail.append(
                    f"{ns}/{label}: relay_forwarded ({robot['relay_forwarded_count']}) vs "
                    f"counter_unique ({counter_unique}) disagree by {spread}"
                )

        for label, robot in (("condition_a", a), ("condition_b", b)):
            aligned_pdr = robot["aligned_window_pdr"]
            if aligned_pdr is None or aligned_pdr < 0.99:
                reasons_fail.append(f"{ns}/{label}: aligned-window PDR (bag vs relay-forwarded) = {aligned_pdr} (<0.99)")

        per_robot_report[ns] = {
            "condition_a": {**ca, "relay_forwarded_count": a["relay_forwarded_count"], "aligned_window_pdr": a["aligned_window_pdr"]},
            "condition_b": {**cb, "relay_forwarded_count": b["relay_forwarded_count"], "aligned_window_pdr": b["aligned_window_pdr"]},
            "observed_latency_increment_s": observed_increment_s,
            "increment_error_vs_configured_s": increment_error_s,
        }

    for label, config in (("condition_a", config_a), ("condition_b", config_b)):
        factor_pre = config.get("preload_realtime_factor", 0)
        factor_full = config.get("full_load_realtime_factor", 0)
        if not (0.8 <= factor_pre <= 1.2 and 0.8 <= factor_full <= 1.2):
            reasons_fail.append(f"{label}: realtime factor out of range (preload={factor_pre}, full_load={factor_full})")

    verdict = "PASS" if not reasons_fail else "FAIL"
    result = {
        "verdict": verdict,
        "fail_reasons": reasons_fail,
        "note": (
            "diagnostic-only pilot; no cooperative_avoider launched in either "
            "condition; latency figures here are the live sequence_counter "
            "measurement (message.stamp vs receipt, same clock domain), NOT "
            "the bag-based analyze_comm_performance figure (see that "
            "module's clock-domain-mismatch note). TIMEBASE_RESET is a "
            "cooperative_avoider-only log signal and does not apply here "
            "since no controller was launched -- N/A by construction, not "
            "checked as a pass criterion."
        ),
        "condition_a_config": config_a,
        "condition_b_config": config_b,
        "per_robot": per_robot_report,
    }
    args.output_path.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()

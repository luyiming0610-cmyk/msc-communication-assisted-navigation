#!/usr/bin/env python3
"""Real, tested analysis entry point for an Objective5 impairment-matrix
trial -- replaces the orchestrator's earlier hardcoded `ANALYZER_OK="true"`
placeholder with an actual analysis pass that can fail.

Unified latency formula (frozen, applies to every condition A-G alike):
    message_age_s = consumer_callback_ros_time - EpuckState.production_stamp
Both endpoints come from `sequence_counter.py`'s own live measurement
(`self.get_clock().now()` at message receipt, vs `msg.stamp` set by
`state_publisher.py`'s `self.get_clock().now()` at publish time) -- both
under `use_sim_time=true`, reading the one shared `/clock` topic. This is
NOT the bag-based age computed by `analyze_comm_performance.py`, which
subtracts a sim-time stamp from the rosbag recorder's own wall-clock
receive timestamp (`ros2 bag record` is not run with `--use-sim-time`) --
a genuine, structural cross-clock-domain quantity, confirmed to land at
~1.7e9 "seconds" (Unix-epoch-scale), correctly excluded by that module's
own `_MAX_PLAUSIBLE_AGE_S` guard. This analyzer never uses that bag-based
age; it reuses `analyze_comm_performance.analyze()` only for its
throughput/bandwidth/rate figures, which are single-clock-domain
quantities (bag-internal duration deltas) and are not affected by the
mismatch.

This module has two halves, deliberately separated so every non-trivial
decision is unit-testable without a real bag/relay/counter run:
- pure functions (`summarize_relay_csv`, `classify_latency_measurement_status`,
  `build_direction_report`, `overall_measurement_validity`, ...) that
  operate on already-loaded dicts/rows
- thin I/O functions (`load_relay_csv`, `load_counter_json`, `analyze_trial`)
  that read real files and call the pure functions

The orchestrator and the standalone `--analysis-only` replay CLI both call
the same `analyze_trial()` function -- there is exactly one analysis code
path, not two that could silently diverge.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

# Kept in sync with sequence_counter.py's own value deliberately.
_MAX_PLAUSIBLE_AGE_S = 30.0
# Measured publish period (design doc section 2.1), reused as the
# tolerance unit for "is this observed age plausible for its configured
# delay/jitter" checks below.
_PUBLISH_PERIOD_S = 0.1151

REQUIRED_TOP_LEVEL_FIELDS = (
    "condition_id",
    "measurement_validity",
    "directions",
    "queue_drain",
)
REQUIRED_DIRECTION_FIELDS = ("direction", "relay", "sequence", "latency")

# Every formal (and any future, non-legacy) trial must report all five of
# these as finite numbers -- p99_message_age_s was added to
# sequence_counter.py after some already-collected exclusionary pilots
# ran, so LEGACY replay of THAT already-collected data is the only case
# permitted to have a null p99 (see legacy_replay in build_latency_block
# / analyze_trial). No formal trial run under current code may ever be
# missing any of these.
LATENCY_STAT_FIELDS = (
    "mean_message_age_s",
    "median_message_age_s",
    "p95_message_age_s",
    "p99_message_age_s",
    "max_message_age_s",
)


def _is_finite_number(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def validate_latency_schema_strict(latency_block: dict) -> list:
    """Pure. The formal-trial completeness gate: sample_count must be > 0
    and every LATENCY_STAT_FIELDS entry must be a finite number (not
    None, NaN, or Inf). Returns a list of problem strings (empty means
    the block passes). Applied to every direction unless the analysis
    is explicitly run in --legacy-replay mode against already-collected
    exclusionary-pilot data that predates the p99 field."""
    problems = []
    if latency_block.get("sample_count", 0) <= 0:
        problems.append("sample_count must be > 0 for a formal trial (0 samples means every message this direction was lost or the counter never ran)")
    for field in LATENCY_STAT_FIELDS:
        value = latency_block.get(field)
        if not _is_finite_number(value):
            problems.append(f"{field} missing or not a finite number: {value!r}")
    return problems


def summarize_relay_csv(rows: list) -> dict:
    """Pure. `rows` are dict rows from a relay CSV (DictReader output),
    columns: received_seq, action, scheduled_delay_s, receive_time_s,
    release_time_s, source_stamp_s, actual_release_time_s, drop_reason."""
    received_count = len(rows)
    forwarded_count = sum(1 for r in rows if r.get("action") == "forwarded")
    dropped_count = received_count - forwarded_count
    independent_drop_count = sum(1 for r in rows if r.get("drop_reason") == "independent")
    outage_drop_count = sum(1 for r in rows if r.get("drop_reason") == "outage")
    return {
        "received_count": received_count,
        "forwarded_count": forwarded_count,
        "dropped_count": dropped_count,
        "independent_drop_count": independent_drop_count,
        "outage_drop_count": outage_drop_count,
    }


def build_sequence_block(counter_topic_summary: dict) -> dict:
    """Pure. `counter_topic_summary` is one topic's dict from a
    sequence_counter.py JSON output (already sim-time-domain, computed
    live at message receipt -- never derived from a rosbag)."""
    received = counter_topic_summary.get("received_count", 0)
    expected = counter_topic_summary.get("expected_count")
    capture_ratio = (received / expected) if expected else None
    return {
        "received_count": received,
        "unique_sequence_count": counter_topic_summary.get("unique_sequence_count"),
        "expected_count": expected,
        "missing_count": counter_topic_summary.get("sequence_gap_count", 0),
        "duplicate_count": counter_topic_summary.get("duplicate_count", 0),
        "out_of_order_count": counter_topic_summary.get("out_of_order_count", 0),
        "capture_ratio": capture_ratio,
    }


def classify_latency_measurement_status(
    sample_count: int,
    mean_age_s,
    configured_delay_s,
    configured_jitter_s,
    publish_period_s: float = _PUBLISH_PERIOD_S,
    tolerance_periods: float = 2.0,
) -> str:
    """Pure. Frozen unified classification, applied identically to every
    condition A-G:
    - NOT_APPLICABLE_NO_SAMPLES: no received messages had a valid age
      sample (e.g. every message on this direction was dropped).
    - RESOLUTION_LIMITED: configured impairment for this direction is
      zero delay and zero jitter, and the observed mean age is within a
      small multiple of one publish period -- a legitimate near-zero
      result at simulation-clock resolution, NOT a clock-domain
      mismatch and NOT to be silently reported as exactly "0 real
      seconds of network delay".
    - VALID_AT_SIM_CLOCK_RESOLUTION: observed mean age falls within the
      configured delay+jitter band (with a publish-period-scale
      tolerance) -- a genuine, trustworthy sim-clock-domain latency
      measurement.
    - METRIC_INVALID: observed mean age is implausible given the
      configured impairment for this direction (e.g. a B trial with
      configured_delay_s=0.20 but an observed mean age near 0, or near
      1.0) -- the measurement itself, not the underlying network
      behaviour, should not be trusted without diagnosis.
    """
    if sample_count == 0:
        return "NOT_APPLICABLE_NO_SAMPLES"
    delay = configured_delay_s or 0.0
    jitter = configured_jitter_s or 0.0
    tolerance = max(jitter, publish_period_s * tolerance_periods)
    if delay == 0.0 and jitter == 0.0:
        if mean_age_s is not None and mean_age_s <= tolerance:
            return "RESOLUTION_LIMITED"
        return "METRIC_INVALID"
    if mean_age_s is None:
        return "METRIC_INVALID"
    lower = max(0.0, delay - tolerance)
    upper = delay + jitter + tolerance
    if lower <= mean_age_s <= upper:
        return "VALID_AT_SIM_CLOCK_RESOLUTION"
    return "METRIC_INVALID"


def build_latency_block(
    counter_topic_summary: dict, configured_delay_s, configured_jitter_s, legacy_replay: bool = False
) -> dict:
    """Pure. Assembles the frozen latency block using ONLY
    sequence_counter.py's already-sim-time-domain aggregates -- never
    the bag-based age computation.

    `legacy_replay=True` is the ONLY case permitted to accept a null
    p99_message_age_s (already-collected exclusionary-pilot data whose
    sequence_counter.py binary predates the p99 field) -- it is marked
    explicitly (`legacy_missing_p99`, `measurement_limitation`) rather
    than silently accepted, and this data remains
    EXCLUSIONARY_DIAGNOSTIC only, never eligible for formal statistics.
    `legacy_replay=False` (the default, used for every trial run under
    current code) instead runs the strict completeness gate
    (validate_latency_schema_strict) and forces the status to
    METRIC_INVALID -- which propagates to an overall INVALID
    measurement_validity -- if sample_count is 0, or any of
    LATENCY_STAT_FIELDS is missing, None, NaN, or Inf."""
    sample_count = counter_topic_summary.get("valid_age_sample_count", 0)
    mean_age = counter_topic_summary.get("mean_message_age_s")
    status = classify_latency_measurement_status(
        sample_count, mean_age, configured_delay_s, configured_jitter_s
    )
    block = {
        "sample_count": sample_count,
        "mean_message_age_s": mean_age,
        "median_message_age_s": counter_topic_summary.get("median_message_age_s"),
        "p95_message_age_s": counter_topic_summary.get("p95_message_age_s"),
        "p99_message_age_s": counter_topic_summary.get("p99_message_age_s"),
        "max_message_age_s": counter_topic_summary.get("max_message_age_s"),
        "negative_age_sample_count": counter_topic_summary.get("negative_age_sample_count", 0),
        "anomalous_age_sample_count": counter_topic_summary.get("anomalous_age_sample_count", 0),
        "configured_delay_s": configured_delay_s,
        "configured_jitter_s": configured_jitter_s,
        "latency_measurement_status": status,
        "clock_domain_evidence": (
            "state_publisher, network_impairment_relay, and sequence_counter "
            "are all launched with use_sim_time=true and read the same "
            "/clock topic; message_age_s = consumer_callback_ros_time "
            "(sequence_counter's own get_clock().now() at message receipt) "
            "- EpuckState.stamp (state_publisher's get_clock().now() at "
            "publish time). Never the rosbag recorder's wall-clock time."
        ),
    }
    if legacy_replay:
        block["legacy_missing_p99"] = block["p99_message_age_s"] is None
        block["measurement_limitation"] = (
            [
                "sequence_counter.py counter version predates p99 field "
                "(counter version predates p99 field) -- archived data "
                "has no raw samples to retroactively compute p99 from"
            ]
            if block["legacy_missing_p99"]
            else []
        )
        block["schema_problems"] = []
    else:
        problems = validate_latency_schema_strict(block)
        block["schema_problems"] = problems
        if problems:
            block["latency_measurement_status"] = "METRIC_INVALID"
    return block


def build_direction_report(
    direction_label: str,
    relay_csv_rows: list,
    counter_topic_summary: dict,
    configured_delay_s,
    configured_jitter_s,
    legacy_replay: bool = False,
) -> dict:
    """Pure. One direction's (e.g. epuck1->epuck2) full report."""
    relay_block = summarize_relay_csv(relay_csv_rows)
    sequence_block = build_sequence_block(counter_topic_summary)
    latency_block = build_latency_block(
        counter_topic_summary, configured_delay_s, configured_jitter_s, legacy_replay=legacy_replay
    )
    consistency_ok = relay_block["forwarded_count"] == sequence_block["received_count"]
    return {
        "direction": direction_label,
        "relay": relay_block,
        "sequence": sequence_block,
        "latency": latency_block,
        "relay_forwarded_matches_consumer_received": consistency_ok,
    }


def overall_measurement_validity(direction_reports: list) -> str:
    """Pure. VALID only if every direction's latency status is not
    METRIC_INVALID and the relay/consumer message counts are
    consistent (a mismatch there means loss between the relay and the
    subscriber that neither the relay's own drop count nor the
    counter's sequence-gap count alone would explain)."""
    for d in direction_reports:
        if d["latency"]["latency_measurement_status"] == "METRIC_INVALID":
            return "INVALID"
        if not d["relay_forwarded_matches_consumer_received"]:
            return "INVALID"
    return "VALID"


def validate_output_schema(payload: dict) -> list:
    """Pure. Returns a list of missing/invalid required fields (empty
    list means the schema is complete). Used both by this module's own
    writer (defence in depth) and by the orchestrator's post-hoc check
    of the written JSON file."""
    problems = []
    for field in REQUIRED_TOP_LEVEL_FIELDS:
        if field not in payload:
            problems.append(f"missing top-level field: {field}")
    directions = payload.get("directions")
    if not isinstance(directions, list) or not directions:
        problems.append("directions must be a non-empty list")
    else:
        for i, d in enumerate(directions):
            for field in REQUIRED_DIRECTION_FIELDS:
                if field not in d:
                    problems.append(f"directions[{i}] missing field: {field}")
    return problems


# --- I/O (thin; all decisions live in the pure functions above) ---

def load_relay_csv(path) -> list:
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def load_counter_json(path) -> dict:
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def analyze_trial(
    native_diag_log_dir: str,
    native_bag_dir: str,
    frozen_params: dict,
    queue_drain: dict,
    realtime_factor: dict,
    task_outcome_inputs: dict,
    legacy_replay: bool = False,
) -> dict:
    """The single analysis entry point. Both the orchestrator's inline
    call and the standalone --analysis-only replay CLI call this exact
    function -- there is only one analysis code path.

    `legacy_replay=True` must ONLY be used for re-analyzing
    already-collected EXCLUSIONARY_DIAGNOSTIC data whose
    sequence_counter.py binary predates the p99 field -- it relaxes the
    strict latency-completeness gate and instead marks
    legacy_missing_p99/measurement_limitation on the affected
    direction(s). Every trial run under current code (formal or any
    future exclusionary pilot) must use the default `legacy_replay=False`
    and will be marked measurement_validity=INVALID if p99 (or any other
    required latency field) is missing, non-finite, or if sample_count
    is 0."""
    diag_dir = Path(native_diag_log_dir)
    relay1_rows = load_relay_csv(diag_dir / "epuck1_relay.csv")
    relay2_rows = load_relay_csv(diag_dir / "epuck2_relay.csv")
    counter1 = load_counter_json(diag_dir / "epuck1_counter.json")
    counter2 = load_counter_json(diag_dir / "epuck2_counter.json")

    delay_s = frozen_params.get("delay_s")
    jitter_s = frozen_params.get("jitter_s")

    # epuck1's relay forwards epuck1's OWN state to epuck2 -- consumed on
    # the "state" topic in the epuck2 namespace's own counter instance.
    direction_1_to_2 = build_direction_report(
        "epuck1_to_epuck2", relay1_rows, counter1.get("state", {}), delay_s, jitter_s,
        legacy_replay=legacy_replay,
    )
    direction_2_to_1 = build_direction_report(
        "epuck2_to_epuck1", relay2_rows, counter2.get("state", {}), delay_s, jitter_s,
        legacy_replay=legacy_replay,
    )
    directions = [direction_1_to_2, direction_2_to_1]

    throughput_by_direction = {}
    analyzer_error = None
    try:
        from epuck2_comm.analyze_comm_performance import analyze as bag_analyze

        bag_result = bag_analyze(Path(native_bag_dir))
        for topic, direction_report in (
            ("/epuck1/state", direction_1_to_2),
            ("/epuck2/state", direction_2_to_1),
        ):
            topic_result = bag_result.get("topics", {}).get(topic, {})
            sessions = topic_result.get("sessions", [])
            if sessions:
                s = sessions[0]
                direction_report["throughput"] = {
                    "mean_bandwidth_bytes_per_s": s.get("mean_bandwidth_bytes_per_s"),
                    "total_bytes": s.get("total_bytes"),
                    "duration_s": s.get("duration_s"),
                    "actual_rate_hz": s.get("actual_rate_hz"),
                }
            else:
                direction_report["throughput"] = {"note": "no session data in bag-based analysis"}
    except Exception as exc:  # noqa: BLE001 -- captured explicitly, not swallowed
        analyzer_error = f"bag-based throughput analysis failed: {exc}"
        for direction_report in directions:
            direction_report["throughput"] = {"note": analyzer_error}

    measurement_validity = overall_measurement_validity(directions)
    if analyzer_error is not None:
        measurement_validity = "INVALID"

    payload = {
        "condition_id": frozen_params.get("condition_id"),
        "trial_index": frozen_params.get("trial_index"),
        "measurement_validity": measurement_validity,
        "legacy_replay": legacy_replay,
        "directions": directions,
        "queue_drain": queue_drain,
        "realtime_factor": realtime_factor,
        "task_outcome_inputs": task_outcome_inputs,
        "errors": [analyzer_error] if analyzer_error else [],
    }
    return payload


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--native-diag-log-dir", required=True)
    parser.add_argument("--native-bag-dir", required=True)
    parser.add_argument("--frozen-params-json", required=True, help="path to frozen_params.json")
    parser.add_argument("--queue-drain-json", required=True, help="JSON string: queue drain facts")
    parser.add_argument("--realtime-factor-json", required=True, help="JSON string: realtime factor facts")
    parser.add_argument(
        "--legacy-replay", action="store_true",
        help=(
            "Relax the strict latency-completeness gate for re-analyzing "
            "already-collected EXCLUSIONARY_DIAGNOSTIC data whose "
            "sequence_counter.py binary predates the p99 field. Must "
            "NEVER be passed for a formal trial or any trial run under "
            "current code -- the orchestrator never passes this flag."
        ),
    )
    parser.add_argument("--task-outcome-inputs-json", required=True, help="JSON string: task outcome inputs")
    parser.add_argument("--output-path", required=True)
    args = parser.parse_args()

    try:
        with open(args.frozen_params_json, encoding="utf-8") as f:
            frozen_params = json.load(f)
        queue_drain = json.loads(args.queue_drain_json)
        realtime_factor = json.loads(args.realtime_factor_json)
        task_outcome_inputs = json.loads(args.task_outcome_inputs_json)

        payload = analyze_trial(
            args.native_diag_log_dir,
            args.native_bag_dir,
            frozen_params,
            queue_drain,
            realtime_factor,
            task_outcome_inputs,
            legacy_replay=args.legacy_replay,
        )
        problems = validate_output_schema(payload)
        if problems:
            payload["measurement_validity"] = "INVALID"
            payload["errors"] = payload.get("errors", []) + [f"schema: {p}" for p in problems]

        with open(args.output_path, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2)
        print(json.dumps(payload, indent=2))

        if payload["measurement_validity"] != "VALID":
            return 1
        return 0
    except Exception as exc:  # noqa: BLE001 -- top-level CLI guard: never a silent non-zero-less failure
        error_payload = {
            "measurement_validity": "INVALID",
            "directions": [],
            "queue_drain": {},
            "errors": [f"analyzer crashed: {exc}"],
        }
        try:
            with open(args.output_path, "w", encoding="utf-8") as f:
                json.dump(error_payload, f, indent=2)
        except Exception:  # noqa: BLE001 -- best-effort; the exit code is authoritative
            pass
        print(json.dumps(error_payload, indent=2), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Tests for hil_offline_stage3_post_run_verifier.py -- synthetic CSV/JSON
fixtures only, no ROS, no real Stage 3 run needed."""
from __future__ import annotations

import csv
import json
import tempfile
import unittest
from pathlib import Path

from hil_offline_stage3_evidence_recorder import (
    CSV_FIELDS, GATE_DECISION_EVENT_ROW_TOPIC, PHASE_EVENT_ROW_TOPIC,
)
from hil_offline_stage3_post_run_verifier import (
    EXPECTED_STAGE3_ROS_DOMAIN_ID,
    PHYSICAL_CLAIM_DISCLAIMER,
    RESULT_TYPE,
    TASK_OUTCOME_VALUES,
    evaluate_data_validity,
    evaluate_gate_decision_evidence_structure,
    evaluate_gate_forwarding_outcome,
    evaluate_stale_interval,
    evaluate_task_outcome,
    run_verifier,
)

TOPIC_CONTRACT = {
    "own_state_topic": "/hil_offline_stage3/epuck1/state",
    "virtual_peer_source_topic": "/hil_offline_stage3/virtual_peer/source_state",
    "virtual_peer_guard_input_topic": "/hil_offline_stage3/virtual_peer/guard_input_state",
    "goal_announcement_topic": "/hil_offline_stage3/goal_announcement",
    "nav_intent_topic": "/hil_offline_stage3/epuck1/nav_intent",
    "requested_cmd_vel_topic": "/hil_offline_stage3/cmd_vel_unguarded_test_only",
    "guarded_cmd_vel_topic": "/hil_offline_stage3/cmd_vel_guarded_test_only",
    "arm_topic": "/hil_offline_stage3/guard_arm_test_only",
    "bridge_status_topic": "/hil_offline_stage3/bridge_status_test_only",
    "phase_event_topic": "/hil_offline_stage3/phase_event_test_only",
    "gate_decision_topic": "/hil_offline_stage3/gate_decision_test_only",
}

SOURCE_TOPIC = TOPIC_CONTRACT["virtual_peer_source_topic"]
GATE_INPUT_TOPIC = TOPIC_CONTRACT["virtual_peer_guard_input_topic"]

ONE_SECOND_NS = 1_000_000_000


def _write_csv(path, rows):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=CSV_FIELDS, lineterminator="\n")
        w.writeheader()
        for r in rows:
            full = {k: r.get(k, "") for k in CSV_FIELDS}
            w.writerow(full)


def _write_summary(path, *, ros_domain_id=91, row_counts=None):
    row_counts = row_counts or {t: 5 for t in TOPIC_CONTRACT.values()}
    with open(path, "w", encoding="utf-8") as f:
        json.dump({
            "start_wall_time_ns": 1, "end_wall_time_ns": 2,
            "ros_domain_id": ros_domain_id,
            "topic_contract": TOPIC_CONTRACT,
            "row_count_by_topic": row_counts,
            "recorder_health_ok": True,
        }, f)


def _phase_row(ts_ns, phase):
    return {"local_time_ns": ts_ns, "local_monotonic_ns": ts_ns, "topic": PHASE_EVENT_ROW_TOPIC, "phase": phase}


def _gate_input_row(ts_ns):
    return {"local_time_ns": ts_ns, "local_monotonic_ns": ts_ns, "topic": GATE_INPUT_TOPIC, "sequence": ts_ns}


def _source_row(ts_ns):
    return {"local_time_ns": ts_ns, "local_monotonic_ns": ts_ns, "topic": SOURCE_TOPIC, "sequence": ts_ns}


def _gate_decision_row(
    ts_ns, *, epoch, gate_state, decision, sequence,
    first_after_reopen=False, dest_topic=None, event_type="GATE_DECISION",
):
    return {
        "local_time_ns": ts_ns, "local_monotonic_ns": ts_ns, "topic": GATE_DECISION_EVENT_ROW_TOPIC,
        "gate_decision_event_type": event_type,
        "gate_decision_gate_epoch": epoch,
        "gate_decision_gate_state": gate_state,
        "gate_decision_source_protocol_version": 1,
        "gate_decision_source_robot_id": 2,
        "gate_decision_source_sequence": sequence,
        "gate_decision_source_production_stamp_s": ts_ns / ONE_SECOND_NS,
        "gate_decision_decision": decision,
        "gate_decision_decision_timestamp_s": ts_ns / ONE_SECOND_NS,
        "gate_decision_first_source_after_reopen": first_after_reopen,
        "gate_decision_forwarded_destination_topic": dest_topic if decision == "FORWARDED" else "",
    }


def _valid_gate_decision_rows(close_ts, reopen_ts, gate_input_topic=GATE_INPUT_TOPIC):
    """One well-formed REJECTED_GATE_CLOSED event during the closed
    epoch (epoch 0) and one well-formed FORWARDED, first-after-reopen
    event in the reopened epoch (epoch 1) -- a textbook-correct,
    minimal gate-decision-event evidence set."""
    return [
        _gate_decision_row(close_ts + int(0.5 * ONE_SECOND_NS), epoch=0, gate_state="CLOSED",
                            decision="REJECTED_GATE_CLOSED", sequence=10),
        _gate_decision_row(reopen_ts + int(0.1 * ONE_SECOND_NS), epoch=1, gate_state="OPEN",
                            decision="FORWARDED", sequence=20, first_after_reopen=True,
                            dest_topic=gate_input_topic),
    ]


def _valid_stale_interval_rows(close_ts=10 * ONE_SECOND_NS, reopen_ts=13 * ONE_SECOND_NS, *, include_gate_decisions=True):
    """A textbook-correct interval: gate-input rows before closure and
    after reopening, source rows continuing throughout, stale-zero
    strictly after the peer timeout (default 1.0s) past closure and
    before reopen, recovery strictly after the first post-reopen
    gate-decision event, and (by default) a matching pair of well-formed
    gate-decision events for the closed/reopened epochs."""
    rows = [
        _gate_input_row(close_ts - ONE_SECOND_NS),
        _source_row(close_ts - ONE_SECOND_NS),
        _phase_row(close_ts, "PEER_GATE_CLOSED"),
        _source_row(close_ts + int(0.5 * ONE_SECOND_NS)),
        _source_row(close_ts + int(1.5 * ONE_SECOND_NS)),
        _phase_row(close_ts + int(1.6 * ONE_SECOND_NS), "STALE_ZERO_CONFIRMED"),
        _phase_row(reopen_ts, "PEER_GATE_REOPENED"),
        _source_row(reopen_ts + int(0.1 * ONE_SECOND_NS)),
        _gate_input_row(reopen_ts + int(0.1 * ONE_SECOND_NS)),
        _phase_row(reopen_ts + int(0.2 * ONE_SECOND_NS), "RECOVERY_CONFIRMED"),
    ]
    if include_gate_decisions:
        rows.extend(_valid_gate_decision_rows(close_ts, reopen_ts))
    # Rows must be in genuine chronological order by local_time_ns --
    # a real single-threaded recorder writes rows strictly in receipt
    # order, and evaluate_data_validity() fails closed on any
    # out-of-order timestamp, so fixtures must respect that too.
    rows.sort(key=lambda r: r["local_time_ns"])
    return rows


class StaleIntervalPureLogicTest(unittest.TestCase):
    """evaluate_stale_interval() now only proves boundary-event
    existence/ordering and source-continuation -- it deliberately no
    longer attempts any cross-topic forwarding/backlog inference (see
    GateForwardingContractTest below for that, proven from the gate's
    own decision events instead)."""

    def test_valid_interval_passes(self):
        result = evaluate_stale_interval(
            _valid_stale_interval_rows(), source_topic=SOURCE_TOPIC, gate_input_topic=GATE_INPUT_TOPIC,
        )
        self.assertTrue(result.ok, result.reasons)
        self.assertIsNotNone(result.close_ts_ns)
        self.assertIsNotNone(result.reopen_ts_ns)
        self.assertLess(result.close_ts_ns, result.reopen_ts_ns)

    def test_source_also_stopping_unexpectedly_fails(self):
        rows = [r for r in _valid_stale_interval_rows() if not (
            r.get("topic") == SOURCE_TOPIC and 10 * ONE_SECOND_NS < r.get("local_time_ns") < 13 * ONE_SECOND_NS
        )]
        result = evaluate_stale_interval(rows, source_topic=SOURCE_TOPIC, gate_input_topic=GATE_INPUT_TOPIC)
        self.assertFalse(result.ok)
        self.assertTrue(any("SOURCE_STATE_DID_NOT_CONTINUE_DURING_CLOSED_INTERVAL" in r for r in result.reasons))

    def test_missing_reopen_event_fails(self):
        rows = [r for r in _valid_stale_interval_rows() if r.get("phase") != "PEER_GATE_REOPENED"]
        result = evaluate_stale_interval(rows, source_topic=SOURCE_TOPIC, gate_input_topic=GATE_INPUT_TOPIC)
        self.assertFalse(result.ok)
        self.assertTrue(any("PEER_GATE_REOPENED_EVENT_COUNT_NOT_EXACTLY_ONE" in r for r in result.reasons))

    def test_duplicate_close_events_fail(self):
        rows = _valid_stale_interval_rows()
        rows.append(_phase_row(9 * ONE_SECOND_NS, "PEER_GATE_CLOSED"))
        result = evaluate_stale_interval(rows, source_topic=SOURCE_TOPIC, gate_input_topic=GATE_INPUT_TOPIC)
        self.assertFalse(result.ok)
        self.assertTrue(any("PEER_GATE_CLOSED_EVENT_COUNT_NOT_EXACTLY_ONE" in r for r in result.reasons))

    def test_duplicate_reopen_events_fail(self):
        rows = _valid_stale_interval_rows()
        rows.append(_phase_row(14 * ONE_SECOND_NS, "PEER_GATE_REOPENED"))
        result = evaluate_stale_interval(rows, source_topic=SOURCE_TOPIC, gate_input_topic=GATE_INPUT_TOPIC)
        self.assertFalse(result.ok)
        self.assertTrue(any("PEER_GATE_REOPENED_EVENT_COUNT_NOT_EXACTLY_ONE" in r for r in result.reasons))

    def test_close_after_reopen_fails(self):
        rows = [
            _phase_row(20 * ONE_SECOND_NS, "PEER_GATE_CLOSED"),
            _phase_row(10 * ONE_SECOND_NS, "PEER_GATE_REOPENED"),
        ]
        result = evaluate_stale_interval(rows, source_topic=SOURCE_TOPIC, gate_input_topic=GATE_INPUT_TOPIC)
        self.assertFalse(result.ok)
        self.assertTrue(any("GATE_CLOSE_DOES_NOT_STRICTLY_PRECEDE_REOPEN" in r for r in result.reasons))

    def test_no_gate_input_before_closure_fails(self):
        rows = [r for r in _valid_stale_interval_rows() if not (
            r.get("topic") == GATE_INPUT_TOPIC and r.get("local_time_ns", 0) <= 10 * ONE_SECOND_NS
        )]
        result = evaluate_stale_interval(rows, source_topic=SOURCE_TOPIC, gate_input_topic=GATE_INPUT_TOPIC)
        self.assertFalse(result.ok)
        self.assertTrue(any("NO_GATE_INPUT_ROW_BEFORE_CLOSURE" in r for r in result.reasons))


class GateDecisionEvidenceStructureTest(unittest.TestCase):
    """DATA_VALIDITY-level checks: is every gate-decision event
    well-formed? (Never whether the contract itself was honoured --
    that is evaluate_gate_forwarding_outcome's job, a TASK_OUTCOME
    concern, tested separately below.)"""

    def test_valid_events_pass(self):
        close_ts, reopen_ts = 10 * ONE_SECOND_NS, 13 * ONE_SECOND_NS
        rows = _valid_gate_decision_rows(close_ts, reopen_ts)
        result = evaluate_gate_decision_evidence_structure(rows, gate_input_topic=GATE_INPUT_TOPIC)
        self.assertTrue(result.ok, result.reasons)

    def test_no_events_at_all_fails(self):
        result = evaluate_gate_decision_evidence_structure([], gate_input_topic=GATE_INPUT_TOPIC)
        self.assertFalse(result.ok)
        self.assertIn("NO_GATE_DECISION_EVENTS_RECORDED", result.reasons)

    def test_unmatched_decision_value_fails_closed(self):
        rows = [_gate_decision_row(1 * ONE_SECOND_NS, epoch=0, gate_state="CLOSED",
                                    decision="MAYBE", sequence=1)]
        result = evaluate_gate_decision_evidence_structure(rows, gate_input_topic=GATE_INPUT_TOPIC)
        self.assertFalse(result.ok)
        self.assertTrue(any("UNMATCHED_GATE_DECISION_VALUE" in r for r in result.reasons))

    def test_malformed_event_type_fails_closed(self):
        rows = [_gate_decision_row(1 * ONE_SECOND_NS, epoch=0, gate_state="CLOSED",
                                    decision="REJECTED_GATE_CLOSED", sequence=1, event_type="SOMETHING_ELSE")]
        result = evaluate_gate_decision_evidence_structure(rows, gate_input_topic=GATE_INPUT_TOPIC)
        self.assertFalse(result.ok)
        self.assertTrue(any("MALFORMED_GATE_DECISION_EVENT_TYPE" in r for r in result.reasons))

    def test_forwarded_event_with_wrong_destination_topic_fails(self):
        rows = [_gate_decision_row(1 * ONE_SECOND_NS, epoch=1, gate_state="OPEN",
                                    decision="FORWARDED", sequence=1, first_after_reopen=True,
                                    dest_topic="/hil_offline_stage3/wrong_topic")]
        result = evaluate_gate_decision_evidence_structure(rows, gate_input_topic=GATE_INPUT_TOPIC)
        self.assertFalse(result.ok)
        self.assertTrue(any("FORWARDED_EVENT_DESTINATION_TOPIC_MISMATCH" in r for r in result.reasons))

    def test_rejected_event_with_nonempty_destination_topic_fails(self):
        row = _gate_decision_row(1 * ONE_SECOND_NS, epoch=0, gate_state="CLOSED",
                                  decision="REJECTED_GATE_CLOSED", sequence=1)
        row["gate_decision_forwarded_destination_topic"] = GATE_INPUT_TOPIC
        result = evaluate_gate_decision_evidence_structure([row], gate_input_topic=GATE_INPUT_TOPIC)
        self.assertFalse(result.ok)
        self.assertTrue(any("REJECTED_EVENT_HAS_NONEMPTY_FORWARDED_DESTINATION_TOPIC" in r for r in result.reasons))

    def test_missing_epoch_fails_closed(self):
        row = _gate_decision_row(1 * ONE_SECOND_NS, epoch=0, gate_state="CLOSED",
                                  decision="REJECTED_GATE_CLOSED", sequence=1)
        row["gate_decision_gate_epoch"] = ""
        result = evaluate_gate_decision_evidence_structure([row], gate_input_topic=GATE_INPUT_TOPIC)
        self.assertFalse(result.ok)
        self.assertTrue(any("GATE_DECISION_EVENT_MISSING_EPOCH" in r for r in result.reasons))

    def test_nonfinite_decision_timestamp_fails_closed(self):
        row = _gate_decision_row(1 * ONE_SECOND_NS, epoch=0, gate_state="CLOSED",
                                  decision="REJECTED_GATE_CLOSED", sequence=1)
        row["gate_decision_decision_timestamp_s"] = "not_a_number"
        result = evaluate_gate_decision_evidence_structure([row], gate_input_topic=GATE_INPUT_TOPIC)
        self.assertFalse(result.ok)
        self.assertTrue(any("MISSING_OR_NONFINITE_DECISION_TIMESTAMP" in r for r in result.reasons))


class GateForwardingContractTest(unittest.TestCase):
    """TASK_OUTCOME-level checks: proves the strict first-post-reopen
    forwarding contract using ONLY the gate's own GATE_DECISION_EVENT
    rows -- never by comparing this recorder's rows for gate_input_topic
    against its rows for the source topic (two independently-scheduled
    subscriber callbacks, no guaranteed relative ordering)."""

    def test_correct_contract_passes(self):
        close_ts, reopen_ts = 10 * ONE_SECOND_NS, 13 * ONE_SECOND_NS
        rows = _valid_gate_decision_rows(close_ts, reopen_ts)
        result = evaluate_gate_forwarding_outcome(rows)
        self.assertIsNone(result.outcome, result.reasons)

    def test_only_one_epoch_seen_fails_gate_forwarding(self):
        rows = [_gate_decision_row(1 * ONE_SECOND_NS, epoch=0, gate_state="OPEN",
                                    decision="FORWARDED", sequence=1, dest_topic=GATE_INPUT_TOPIC)]
        result = evaluate_gate_forwarding_outcome(rows)
        self.assertEqual(result.outcome, "GATE_FORWARDING_FAILURE")
        self.assertTrue(any("DO_NOT_SPAN_A_REOPEN" in r for r in result.reasons))

    def test_forwarded_while_closed_fails_gate_forwarding(self):
        close_ts, reopen_ts = 10 * ONE_SECOND_NS, 13 * ONE_SECOND_NS
        rows = _valid_gate_decision_rows(close_ts, reopen_ts)
        rows.append(_gate_decision_row(close_ts + int(0.6 * ONE_SECOND_NS), epoch=0, gate_state="CLOSED",
                                        decision="FORWARDED", sequence=11, dest_topic=GATE_INPUT_TOPIC))
        result = evaluate_gate_forwarding_outcome(rows)
        self.assertEqual(result.outcome, "GATE_FORWARDING_FAILURE")
        self.assertTrue(any("GATE_FORWARDED_WHILE_CLOSED" in r for r in result.reasons))

    def test_no_rejected_events_while_closed_fails(self):
        """The source must continue to be processed while closed -- if
        no REJECTED_GATE_CLOSED event was ever recorded during the
        closed epoch, that is itself a contract violation, never
        silently treated as 'nothing happened, so it's fine'."""
        close_ts, reopen_ts = 10 * ONE_SECOND_NS, 13 * ONE_SECOND_NS
        rows = [_gate_decision_row(reopen_ts + int(0.1 * ONE_SECOND_NS), epoch=1, gate_state="OPEN",
                                    decision="FORWARDED", sequence=20, first_after_reopen=True,
                                    dest_topic=GATE_INPUT_TOPIC)]
        # inject a closed-epoch marker with no actual events by forcing
        # epochs to span two values without any CLOSED-state row
        rows.append(_gate_decision_row(close_ts, epoch=0, gate_state="OPEN",
                                        decision="FORWARDED", sequence=5, dest_topic=GATE_INPUT_TOPIC))
        result = evaluate_gate_forwarding_outcome(rows)
        self.assertEqual(result.outcome, "GATE_FORWARDING_FAILURE")
        self.assertTrue(any("NO_GATE_DECISION_EVENTS_WHILE_CLOSED" in r for r in result.reasons))

    def test_first_post_reopen_not_marked_fails(self):
        close_ts, reopen_ts = 10 * ONE_SECOND_NS, 13 * ONE_SECOND_NS
        rows = [
            _gate_decision_row(close_ts + int(0.5 * ONE_SECOND_NS), epoch=0, gate_state="CLOSED",
                                decision="REJECTED_GATE_CLOSED", sequence=10),
            _gate_decision_row(reopen_ts + int(0.1 * ONE_SECOND_NS), epoch=1, gate_state="OPEN",
                                decision="FORWARDED", sequence=20, first_after_reopen=False,
                                dest_topic=GATE_INPUT_TOPIC),
        ]
        result = evaluate_gate_forwarding_outcome(rows)
        self.assertEqual(result.outcome, "GATE_FORWARDING_FAILURE")
        self.assertTrue(any("NOT_MARKED_FIRST_AFTER_REOPEN" in r for r in result.reasons))

    def test_first_post_reopen_rejected_instead_of_forwarded_fails(self):
        close_ts, reopen_ts = 10 * ONE_SECOND_NS, 13 * ONE_SECOND_NS
        rows = [
            _gate_decision_row(close_ts + int(0.5 * ONE_SECOND_NS), epoch=0, gate_state="CLOSED",
                                decision="REJECTED_GATE_CLOSED", sequence=10),
            _gate_decision_row(reopen_ts + int(0.1 * ONE_SECOND_NS), epoch=1, gate_state="OPEN",
                                decision="REJECTED_GATE_CLOSED", sequence=20, first_after_reopen=True),
        ]
        result = evaluate_gate_forwarding_outcome(rows)
        self.assertEqual(result.outcome, "GATE_FORWARDING_FAILURE")
        self.assertTrue(any("NOT_FORWARDED(decision=REJECTED_GATE_CLOSED)" in r for r in result.reasons))

    def test_multiple_events_marked_first_after_reopen_fails(self):
        close_ts, reopen_ts = 10 * ONE_SECOND_NS, 13 * ONE_SECOND_NS
        rows = [
            _gate_decision_row(close_ts + int(0.5 * ONE_SECOND_NS), epoch=0, gate_state="CLOSED",
                                decision="REJECTED_GATE_CLOSED", sequence=10),
            _gate_decision_row(reopen_ts + int(0.1 * ONE_SECOND_NS), epoch=1, gate_state="OPEN",
                                decision="FORWARDED", sequence=20, first_after_reopen=True,
                                dest_topic=GATE_INPUT_TOPIC),
            _gate_decision_row(reopen_ts + int(0.2 * ONE_SECOND_NS), epoch=1, gate_state="OPEN",
                                decision="FORWARDED", sequence=21, first_after_reopen=True,
                                dest_topic=GATE_INPUT_TOPIC),
        ]
        result = evaluate_gate_forwarding_outcome(rows)
        self.assertEqual(result.outcome, "GATE_FORWARDING_FAILURE")
        self.assertTrue(any("MULTIPLE_EVENTS_MARKED_FIRST_SOURCE_AFTER_REOPEN" in r for r in result.reasons))

    def test_cached_pre_reopen_sequence_forwarded_after_reopen_is_backlog_replay(self):
        """A forwarded post-reopen event whose source_sequence is <= a
        sequence already seen (as a decision event) while the gate was
        CLOSED is a replayed cached message -- proven via the message's
        OWN sequence number, never a local receipt timestamp."""
        close_ts, reopen_ts = 10 * ONE_SECOND_NS, 13 * ONE_SECOND_NS
        rows = [
            _gate_decision_row(close_ts + int(0.5 * ONE_SECOND_NS), epoch=0, gate_state="CLOSED",
                                decision="REJECTED_GATE_CLOSED", sequence=50),
            _gate_decision_row(reopen_ts + int(0.1 * ONE_SECOND_NS), epoch=1, gate_state="OPEN",
                                decision="FORWARDED", sequence=12, first_after_reopen=True,
                                dest_topic=GATE_INPUT_TOPIC),
        ]
        result = evaluate_gate_forwarding_outcome(rows)
        self.assertEqual(result.outcome, "BACKLOG_REPLAY_DETECTED")
        self.assertTrue(any("BACKLOG_REPLAY_DETECTED" in r for r in result.reasons))

    def test_backlog_replay_takes_priority_over_other_gate_forwarding_reasons(self):
        close_ts, reopen_ts = 10 * ONE_SECOND_NS, 13 * ONE_SECOND_NS
        rows = [
            _gate_decision_row(close_ts + int(0.5 * ONE_SECOND_NS), epoch=0, gate_state="CLOSED",
                                decision="REJECTED_GATE_CLOSED", sequence=50),
            # not marked first_after_reopen AND a backlog replay
            _gate_decision_row(reopen_ts + int(0.1 * ONE_SECOND_NS), epoch=1, gate_state="OPEN",
                                decision="FORWARDED", sequence=12, first_after_reopen=False,
                                dest_topic=GATE_INPUT_TOPIC),
        ]
        result = evaluate_gate_forwarding_outcome(rows)
        self.assertEqual(result.outcome, "BACKLOG_REPLAY_DETECTED")

    def test_no_events_for_final_reopened_epoch_fails(self):
        close_ts = 10 * ONE_SECOND_NS
        rows = [
            _gate_decision_row(close_ts + int(0.5 * ONE_SECOND_NS), epoch=0, gate_state="CLOSED",
                                decision="REJECTED_GATE_CLOSED", sequence=10),
            _gate_decision_row(close_ts + int(0.6 * ONE_SECOND_NS), epoch=1, gate_state="CLOSED",
                                decision="REJECTED_GATE_CLOSED", sequence=11),
        ]
        result = evaluate_gate_forwarding_outcome(rows)
        self.assertEqual(result.outcome, "GATE_FORWARDING_FAILURE")
        self.assertTrue(any("NO_GATE_DECISION_EVENTS_FOR_FINAL_REOPENED_EPOCH" in r for r in result.reasons))


class DataValidityTest(unittest.TestCase):
    def test_missing_files_are_invalid(self):
        result = evaluate_data_validity(
            csv_path="/nonexistent/evidence.csv", summary_json_path="/nonexistent/summary.json",
            residual_process_detected=False,
        )
        self.assertFalse(result.valid)
        self.assertTrue(any("MISSING_OR_EMPTY_CSV" in r for r in result.reasons))

    def test_valid_minimal_fixture_passes(self):
        with tempfile.TemporaryDirectory() as d:
            csv_path = str(Path(d) / "e.csv")
            json_path = str(Path(d) / "s.json")
            _write_csv(csv_path, [
                {"local_time_ns": ONE_SECOND_NS, "local_monotonic_ns": ONE_SECOND_NS,
                 "topic": TOPIC_CONTRACT["own_state_topic"], "validity_flags": 7, "sequence": 1},
            ] + _valid_stale_interval_rows())
            _write_summary(json_path)
            result = evaluate_data_validity(csv_path=csv_path, summary_json_path=json_path,
                                             residual_process_detected=False)
            self.assertTrue(result.valid, result.reasons)

    def test_forbidden_domain_id_fails(self):
        with tempfile.TemporaryDirectory() as d:
            csv_path = str(Path(d) / "e.csv")
            json_path = str(Path(d) / "s.json")
            _write_csv(csv_path, _valid_stale_interval_rows())
            _write_summary(json_path, ros_domain_id=77)
            result = evaluate_data_validity(csv_path=csv_path, summary_json_path=json_path,
                                             residual_process_detected=False)
            self.assertFalse(result.valid)
            self.assertTrue(any("ROS_DOMAIN_ID_FORBIDDEN" in r for r in result.reasons))

    def test_domain_91_accepted_alongside_valid_evidence(self):
        self.assertEqual(EXPECTED_STAGE3_ROS_DOMAIN_ID, 91)
        with tempfile.TemporaryDirectory() as d:
            csv_path = str(Path(d) / "e.csv")
            json_path = str(Path(d) / "s.json")
            _write_csv(csv_path, [
                {"local_time_ns": ONE_SECOND_NS, "local_monotonic_ns": ONE_SECOND_NS,
                 "topic": TOPIC_CONTRACT["own_state_topic"], "validity_flags": 7, "sequence": 1},
            ] + _valid_stale_interval_rows())
            _write_summary(json_path, ros_domain_id=91)
            result = evaluate_data_validity(csv_path=csv_path, summary_json_path=json_path,
                                             residual_process_detected=False)
            self.assertTrue(result.valid, result.reasons)

    def test_production_topic_in_contract_fails(self):
        with tempfile.TemporaryDirectory() as d:
            csv_path = str(Path(d) / "e.csv")
            json_path = str(Path(d) / "s.json")
            _write_csv(csv_path, _valid_stale_interval_rows())
            bad_contract = dict(TOPIC_CONTRACT)
            bad_contract["arm_topic"] = "/hil_guard/arm"
            with open(json_path, "w", encoding="utf-8") as f:
                json.dump({"start_wall_time_ns": 1, "end_wall_time_ns": 2, "ros_domain_id": 91,
                           "topic_contract": bad_contract,
                           "row_count_by_topic": {t: 5 for t in bad_contract.values()},
                           "recorder_health_ok": True}, f)
            result = evaluate_data_validity(csv_path=csv_path, summary_json_path=json_path,
                                             residual_process_detected=False)
            self.assertFalse(result.valid)
            self.assertTrue(any("PRODUCTION_TOPIC_USED" in r for r in result.reasons))

    def test_residual_process_detected_fails(self):
        with tempfile.TemporaryDirectory() as d:
            csv_path = str(Path(d) / "e.csv")
            json_path = str(Path(d) / "s.json")
            _write_csv(csv_path, _valid_stale_interval_rows())
            _write_summary(json_path)
            result = evaluate_data_validity(csv_path=csv_path, summary_json_path=json_path,
                                             residual_process_detected=True)
            self.assertFalse(result.valid)
            self.assertTrue(any("RESIDUAL_PROCESS_DETECTED" in r for r in result.reasons))

    def test_missing_gate_decision_events_fails_data_validity(self):
        """A run that never produced any gate-decision evidence at all
        is an evidence-quality (DATA_VALIDITY) defect -- not something
        the TASK_OUTCOME layer should have to guess about."""
        with tempfile.TemporaryDirectory() as d:
            csv_path = str(Path(d) / "e.csv")
            json_path = str(Path(d) / "s.json")
            _write_csv(csv_path, _valid_stale_interval_rows(include_gate_decisions=False))
            _write_summary(json_path)
            result = evaluate_data_validity(csv_path=csv_path, summary_json_path=json_path,
                                             residual_process_detected=False)
            self.assertFalse(result.valid)
            self.assertTrue(any("NO_GATE_DECISION_EVENTS_RECORDED" in r for r in result.reasons))

    def test_malformed_gate_decision_event_fails_data_validity_not_task_outcome(self):
        """A malformed gate-decision event (unmatched decision value) is
        an evidence-quality defect -- it must surface as DATA_VALIDITY,
        never be silently absorbed into a TASK_OUTCOME category."""
        with tempfile.TemporaryDirectory() as d:
            csv_path = str(Path(d) / "e.csv")
            json_path = str(Path(d) / "s.json")
            close_ts, reopen_ts = 10 * ONE_SECOND_NS, 13 * ONE_SECOND_NS
            rows = _valid_stale_interval_rows(close_ts, reopen_ts, include_gate_decisions=False)
            rows.append(_gate_decision_row(close_ts + int(0.5 * ONE_SECOND_NS), epoch=0, gate_state="CLOSED",
                                            decision="NOT_A_REAL_DECISION", sequence=1))
            _write_csv(csv_path, rows)
            _write_summary(json_path)
            result = evaluate_data_validity(csv_path=csv_path, summary_json_path=json_path,
                                             residual_process_detected=False)
            self.assertFalse(result.valid)
            self.assertTrue(any("UNMATCHED_GATE_DECISION_VALUE" in r for r in result.reasons))


class TaskOutcomeTaxonomyTest(unittest.TestCase):
    def test_taxonomy_contains_no_physical_wording(self):
        for value in TASK_OUTCOME_VALUES:
            self.assertNotIn("UNSAFE", value)
            self.assertNotIn("PHYSICAL", value)

    def test_taxonomy_contains_gate_forwarding_and_backlog_values(self):
        self.assertIn("GATE_FORWARDING_FAILURE", TASK_OUTCOME_VALUES)
        self.assertIn("BACKLOG_REPLAY_DETECTED", TASK_OUTCOME_VALUES)

    def test_result_type_and_disclaimer_present(self):
        self.assertEqual(RESULT_TYPE, "OFFLINE_SOFTWARE_CONTRACT_RESULT")
        self.assertIn("not evidence of physical collision", PHYSICAL_CLAIM_DISCLAIMER)


class TaskOutcomeTest(unittest.TestCase):
    def _base_rows(self):
        close_ts, reopen_ts = 10 * ONE_SECOND_NS, 13 * ONE_SECOND_NS
        rows = _valid_stale_interval_rows(close_ts, reopen_ts)
        rows.append({"local_time_ns": 1, "local_monotonic_ns": 1,
                      "topic": TOPIC_CONTRACT["goal_announcement_topic"], "goal_id": "shared_exit"})
        rows.append(_phase_row(2, "ANNOUNCEMENT_ADOPTED"))
        rows.append({"local_time_ns": 3, "local_monotonic_ns": 3, "topic": PHASE_EVENT_ROW_TOPIC,
                      "duplicate_sent": "True"})
        rows.append(_phase_row(15 * ONE_SECOND_NS, "COMPLETE"))
        rows.append({"local_time_ns": 4, "local_monotonic_ns": 4,
                      "topic": TOPIC_CONTRACT["guarded_cmd_vel_topic"], "linear_x": 0.01, "angular_z": 0.02})
        return rows, close_ts, reopen_ts

    def test_all_conditions_met_gives_success(self):
        rows, close_ts, reopen_ts = self._base_rows()
        result = evaluate_task_outcome(
            rows=rows, topic_contract=TOPIC_CONTRACT,
            test_only_angular_bound_rps=0.05, test_only_linear_bound_mps=0.02,
            close_ts_ns=close_ts, reopen_ts_ns=reopen_ts,
        )
        self.assertEqual(result.outcome, "SUCCESS", result.reasons)

    def test_missing_announcement_gives_adoption_failure(self):
        rows, close_ts, reopen_ts = self._base_rows()
        rows = [r for r in rows if r.get("topic") != TOPIC_CONTRACT["goal_announcement_topic"]]
        result = evaluate_task_outcome(rows=rows, topic_contract=TOPIC_CONTRACT,
                                        test_only_angular_bound_rps=0.05, test_only_linear_bound_mps=0.02,
                                        close_ts_ns=close_ts, reopen_ts_ns=reopen_ts)
        self.assertEqual(result.outcome, "ADOPTION_FAILURE")

    def test_missing_duplicate_sent_gives_duplicate_handling_failure(self):
        rows, close_ts, reopen_ts = self._base_rows()
        rows = [r for r in rows if _as_bool_test(r.get("duplicate_sent")) is not True]
        result = evaluate_task_outcome(rows=rows, topic_contract=TOPIC_CONTRACT,
                                        test_only_angular_bound_rps=0.05, test_only_linear_bound_mps=0.02,
                                        close_ts_ns=close_ts, reopen_ts_ns=reopen_ts)
        self.assertEqual(result.outcome, "DUPLICATE_HANDLING_FAILURE")

    def test_out_of_bound_guarded_command_gives_guard_bound_violation_not_unsafe_failure(self):
        rows, close_ts, reopen_ts = self._base_rows()
        rows.append({"local_time_ns": 5, "local_monotonic_ns": 5,
                      "topic": TOPIC_CONTRACT["guarded_cmd_vel_topic"], "linear_x": 5.0, "angular_z": 0.0})
        result = evaluate_task_outcome(rows=rows, topic_contract=TOPIC_CONTRACT,
                                        test_only_angular_bound_rps=0.05, test_only_linear_bound_mps=0.02,
                                        close_ts_ns=close_ts, reopen_ts_ns=reopen_ts)
        self.assertEqual(result.outcome, "GUARD_BOUND_VIOLATION")
        self.assertTrue(result.reasons)

    def test_stale_zero_before_timeout_gives_stale_zero_failure(self):
        rows, close_ts, reopen_ts = self._base_rows()
        # Replace the correctly-timed STALE_ZERO_CONFIRMED with one that
        # fires immediately after closure (before the 1.0s peer timeout
        # has elapsed).
        rows = [r for r in rows if r.get("phase") != "STALE_ZERO_CONFIRMED"]
        rows.append(_phase_row(close_ts + 1, "STALE_ZERO_CONFIRMED"))
        result = evaluate_task_outcome(rows=rows, topic_contract=TOPIC_CONTRACT,
                                        test_only_angular_bound_rps=0.05, test_only_linear_bound_mps=0.02,
                                        close_ts_ns=close_ts, reopen_ts_ns=reopen_ts)
        self.assertEqual(result.outcome, "STALE_ZERO_FAILURE")
        self.assertTrue(any("BEFORE_PEER_TIMEOUT_ELAPSED" in r for r in result.reasons))

    def test_recovery_before_fresh_peer_state_gives_recovery_failure(self):
        rows, close_ts, reopen_ts = self._base_rows()
        rows = [r for r in rows if r.get("phase") != "RECOVERY_CONFIRMED"]
        rows.append(_phase_row(reopen_ts - 1, "RECOVERY_CONFIRMED"))  # before reopen, cannot be after a fresh post-reopen state
        result = evaluate_task_outcome(rows=rows, topic_contract=TOPIC_CONTRACT,
                                        test_only_angular_bound_rps=0.05, test_only_linear_bound_mps=0.02,
                                        close_ts_ns=close_ts, reopen_ts_ns=reopen_ts)
        self.assertEqual(result.outcome, "RECOVERY_FAILURE")
        self.assertTrue(any("RECOVERY_CONFIRMED_NOT_AFTER_REOPEN_PHASE_EVENT" in r for r in result.reasons))

    def test_missing_recovery_phase_entirely_gives_recovery_failure(self):
        """Durable regression: no RECOVERY_CONFIRMED phase event anywhere
        in the evidence at all (not merely mistimed) must still fail
        closed as RECOVERY_FAILURE, never silently treated as SUCCESS."""
        rows, close_ts, reopen_ts = self._base_rows()
        rows = [r for r in rows if r.get("phase") != "RECOVERY_CONFIRMED"]
        result = evaluate_task_outcome(rows=rows, topic_contract=TOPIC_CONTRACT,
                                        test_only_angular_bound_rps=0.05, test_only_linear_bound_mps=0.02,
                                        close_ts_ns=close_ts, reopen_ts_ns=reopen_ts)
        self.assertEqual(result.outcome, "RECOVERY_FAILURE")
        self.assertIn("RECOVERY_NOT_CONFIRMED", result.reasons)

    def test_recovery_after_reopen_passes_even_with_inverted_cross_topic_gate_decision_timestamp(self):
        """Durable regression for the exact race this correction fixes:
        a real production run can have its recorder receive and
        timestamp the first-post-reopen GATE_DECISION_EVENT row AFTER
        (later local_time_ns than) the RECOVERY_CONFIRMED phase-event
        row, purely because they arrive via two independently-scheduled
        recorder subscriptions -- this must NOT be treated as a recovery
        failure. Only same-topic (phase-event-vs-phase-event) ordering
        may be compared; the gate-decision event's existence (not its
        cross-topic receipt order) is what recovery correctness depends
        on here."""
        rows, close_ts, reopen_ts = self._base_rows()
        # Strip the default gate-decision rows and re-add them with the
        # first-post-reopen FORWARDED event timestamped intentionally
        # AFTER the RECOVERY_CONFIRMED phase event (inverted delivery
        # order vs. the true production sequencing).
        rows = [r for r in rows if r.get("topic") != GATE_DECISION_EVENT_ROW_TOPIC]
        recovery_ts = next(int(r["local_time_ns"]) for r in rows if r.get("phase") == "RECOVERY_CONFIRMED")
        rows.append(_gate_decision_row(close_ts + int(0.5 * ONE_SECOND_NS), epoch=0, gate_state="CLOSED",
                                        decision="REJECTED_GATE_CLOSED", sequence=10))
        rows.append(_gate_decision_row(recovery_ts + 1, epoch=1, gate_state="OPEN",
                                        decision="FORWARDED", sequence=20, first_after_reopen=True,
                                        dest_topic=GATE_INPUT_TOPIC))
        result = evaluate_task_outcome(rows=rows, topic_contract=TOPIC_CONTRACT,
                                        test_only_angular_bound_rps=0.05, test_only_linear_bound_mps=0.02,
                                        close_ts_ns=close_ts, reopen_ts_ns=reopen_ts)
        self.assertEqual(result.outcome, "SUCCESS", result.reasons)

    def test_gate_forwarded_while_closed_gives_gate_forwarding_failure(self):
        rows, close_ts, reopen_ts = self._base_rows()
        rows.append(_gate_decision_row(close_ts + int(0.6 * ONE_SECOND_NS), epoch=0, gate_state="CLOSED",
                                        decision="FORWARDED", sequence=11, dest_topic=GATE_INPUT_TOPIC))
        result = evaluate_task_outcome(rows=rows, topic_contract=TOPIC_CONTRACT,
                                        test_only_angular_bound_rps=0.05, test_only_linear_bound_mps=0.02,
                                        close_ts_ns=close_ts, reopen_ts_ns=reopen_ts)
        self.assertEqual(result.outcome, "GATE_FORWARDING_FAILURE")

    def test_backlog_replay_gives_backlog_replay_detected_outcome(self):
        rows, close_ts, reopen_ts = self._base_rows()
        rows = [r for r in rows if r.get("topic") != GATE_DECISION_EVENT_ROW_TOPIC]
        rows.append(_gate_decision_row(close_ts + int(0.5 * ONE_SECOND_NS), epoch=0, gate_state="CLOSED",
                                        decision="REJECTED_GATE_CLOSED", sequence=50))
        rows.append(_gate_decision_row(reopen_ts + int(0.1 * ONE_SECOND_NS), epoch=1, gate_state="OPEN",
                                        decision="FORWARDED", sequence=12, first_after_reopen=True,
                                        dest_topic=GATE_INPUT_TOPIC))
        result = evaluate_task_outcome(rows=rows, topic_contract=TOPIC_CONTRACT,
                                        test_only_angular_bound_rps=0.05, test_only_linear_bound_mps=0.02,
                                        close_ts_ns=close_ts, reopen_ts_ns=reopen_ts)
        self.assertEqual(result.outcome, "BACKLOG_REPLAY_DETECTED")

    def test_backlog_replay_and_duplicate_handling_failure_prefers_duplicate(self):
        """ADOPTION_FAILURE/DUPLICATE_HANDLING_FAILURE remain the two
        highest-priority outcomes -- a backlog replay does not mask a
        duplicate-handling defect, it is only ever more specific than
        the generic GATE_FORWARDING_FAILURE/GUARD_BOUND_VIOLATION/etc."""
        rows, close_ts, reopen_ts = self._base_rows()
        rows = [r for r in rows if _as_bool_test(r.get("duplicate_sent")) is not True]
        rows = [r for r in rows if r.get("topic") != GATE_DECISION_EVENT_ROW_TOPIC]
        rows.append(_gate_decision_row(close_ts + int(0.5 * ONE_SECOND_NS), epoch=0, gate_state="CLOSED",
                                        decision="REJECTED_GATE_CLOSED", sequence=50))
        rows.append(_gate_decision_row(reopen_ts + int(0.1 * ONE_SECOND_NS), epoch=1, gate_state="OPEN",
                                        decision="FORWARDED", sequence=12, first_after_reopen=True,
                                        dest_topic=GATE_INPUT_TOPIC))
        result = evaluate_task_outcome(rows=rows, topic_contract=TOPIC_CONTRACT,
                                        test_only_angular_bound_rps=0.05, test_only_linear_bound_mps=0.02,
                                        close_ts_ns=close_ts, reopen_ts_ns=reopen_ts)
        self.assertEqual(result.outcome, "DUPLICATE_HANDLING_FAILURE")


def _as_bool_test(value):
    if value in (None, "", "None"):
        return None
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("true", "1")


class ValidFailurePreservationTest(unittest.TestCase):
    def test_valid_data_with_failed_task_outcome_is_reported_not_hidden(self):
        with tempfile.TemporaryDirectory() as d:
            csv_path = str(Path(d) / "e.csv")
            json_path = str(Path(d) / "s.json")
            close_ts, reopen_ts = 10 * ONE_SECOND_NS, 13 * ONE_SECOND_NS
            rows = [
                {"local_time_ns": ONE_SECOND_NS, "local_monotonic_ns": ONE_SECOND_NS,
                 "topic": TOPIC_CONTRACT["own_state_topic"], "validity_flags": 7, "sequence": 1},
                {"local_time_ns": 2 * ONE_SECOND_NS, "local_monotonic_ns": 2 * ONE_SECOND_NS,
                 "topic": TOPIC_CONTRACT["goal_announcement_topic"], "goal_id": "shared_exit"},
                _phase_row(3 * ONE_SECOND_NS, "ANNOUNCEMENT_ADOPTED"),
                {"local_time_ns": 4 * ONE_SECOND_NS, "local_monotonic_ns": 4 * ONE_SECOND_NS,
                 "topic": PHASE_EVENT_ROW_TOPIC, "duplicate_sent": "True"},
            ] + _valid_stale_interval_rows(close_ts, reopen_ts) + [
                _phase_row(15 * ONE_SECOND_NS, "COMPLETE"),
                {"local_time_ns": 16 * ONE_SECOND_NS, "local_monotonic_ns": 16 * ONE_SECOND_NS,
                 "topic": TOPIC_CONTRACT["guarded_cmd_vel_topic"], "linear_x": 999.0, "angular_z": 0.0},
            ]
            rows.sort(key=lambda r: r["local_time_ns"])
            _write_csv(csv_path, rows)
            _write_summary(json_path)
            result = run_verifier(
                csv_path=csv_path, summary_json_path=json_path, residual_process_detected=False,
                test_only_angular_bound_rps=0.05, test_only_linear_bound_mps=0.02,
            )
            self.assertEqual(result["DATA_VALIDITY"], "VALID", result["data_validity_reasons"])
            self.assertEqual(result["TASK_OUTCOME"], "GUARD_BOUND_VIOLATION")
            self.assertTrue(result["task_outcome_reasons"])
            self.assertEqual(result["result_type"], "OFFLINE_SOFTWARE_CONTRACT_RESULT")
            self.assertIn("not evidence of physical collision", result["physical_claim"])

    def test_invalid_data_forces_task_outcome_not_evaluable(self):
        result = run_verifier(
            csv_path="/nonexistent.csv", summary_json_path="/nonexistent.json",
            residual_process_detected=False, test_only_angular_bound_rps=0.05, test_only_linear_bound_mps=0.02,
        )
        self.assertEqual(result["DATA_VALIDITY"], "INVALID")
        self.assertEqual(result["TASK_OUTCOME"], "NOT_EVALUABLE")

    def test_backlog_replay_on_valid_data_is_reported_not_hidden(self):
        with tempfile.TemporaryDirectory() as d:
            csv_path = str(Path(d) / "e.csv")
            json_path = str(Path(d) / "s.json")
            close_ts, reopen_ts = 10 * ONE_SECOND_NS, 13 * ONE_SECOND_NS
            rows = [
                {"local_time_ns": ONE_SECOND_NS, "local_monotonic_ns": ONE_SECOND_NS,
                 "topic": TOPIC_CONTRACT["own_state_topic"], "validity_flags": 7, "sequence": 1},
                {"local_time_ns": 2 * ONE_SECOND_NS, "local_monotonic_ns": 2 * ONE_SECOND_NS,
                 "topic": TOPIC_CONTRACT["goal_announcement_topic"], "goal_id": "shared_exit"},
                _phase_row(3 * ONE_SECOND_NS, "ANNOUNCEMENT_ADOPTED"),
                {"local_time_ns": 4 * ONE_SECOND_NS, "local_monotonic_ns": 4 * ONE_SECOND_NS,
                 "topic": PHASE_EVENT_ROW_TOPIC, "duplicate_sent": "True"},
            ] + _valid_stale_interval_rows(close_ts, reopen_ts, include_gate_decisions=False) + [
                _phase_row(15 * ONE_SECOND_NS, "COMPLETE"),
                {"local_time_ns": 5 * ONE_SECOND_NS, "local_monotonic_ns": 5 * ONE_SECOND_NS,
                 "topic": TOPIC_CONTRACT["guarded_cmd_vel_topic"], "linear_x": 0.0, "angular_z": 0.0},
                _gate_decision_row(close_ts + int(0.5 * ONE_SECOND_NS), epoch=0, gate_state="CLOSED",
                                    decision="REJECTED_GATE_CLOSED", sequence=50),
                _gate_decision_row(reopen_ts + int(0.1 * ONE_SECOND_NS), epoch=1, gate_state="OPEN",
                                    decision="FORWARDED", sequence=12, first_after_reopen=True,
                                    dest_topic=GATE_INPUT_TOPIC),
            ]
            rows.sort(key=lambda r: r["local_time_ns"])
            _write_csv(csv_path, rows)
            _write_summary(json_path)
            result = run_verifier(
                csv_path=csv_path, summary_json_path=json_path, residual_process_detected=False,
                test_only_angular_bound_rps=0.05, test_only_linear_bound_mps=0.02,
            )
            self.assertEqual(result["DATA_VALIDITY"], "VALID", result["data_validity_reasons"])
            self.assertEqual(result["TASK_OUTCOME"], "BACKLOG_REPLAY_DETECTED")
            self.assertTrue(result["task_outcome_reasons"])


if __name__ == "__main__":
    unittest.main()

#!/usr/bin/env python3
"""Per-session runtime confirmations for the first ground diagnostic.

Some safety confirmations -- operator physically present, Wi-Fi checked
in the test area -- are true only for THIS session and must be
re-confirmed every time a session starts. Unlike measured geometry or
fixed venue facts, they must never be committed into the tracked
ground_diagnostic_params.json as a permanent fact (a future session
with nobody present must not silently inherit an old "true"). This
module manages a small, timestamped, gitignored session-state file
instead. No ROS/rclpy dependency -- pure logic plus a thin CLI wrapper,
the same pattern as hil_preflight.py.
"""
from __future__ import annotations

import json
import sys
from dataclasses import dataclass, field
from datetime import datetime, timezone

REQUIRED_SESSION_FIELDS = ("operator_present_confirmed", "wifi_checked_in_test_area")
DEFAULT_MAX_AGE_S = 4 * 3600.0
TIMESTAMP_KEY = "session_started_utc"


def fresh_session_state() -> dict:
    """The only way a session file is created or reset: current
    timestamp, every confirmation false. There is no code path that
    reuses an old timestamp with new confirmations, or an old
    confirmation alongside a new timestamp -- init always resets both
    together, so a session can never be silently extended or replayed.
    """
    state = {
        "_comment": (
            "Per-session runtime confirmations for the first ground "
            "diagnostic. Gitignored -- never committed. Re-run "
            "'hil_ground_diagnostic_session.py init' at the start of "
            "every session; never hand-edit or reuse an old copy."
        ),
        TIMESTAMP_KEY: datetime.now(timezone.utc).isoformat(),
    }
    for name in REQUIRED_SESSION_FIELDS:
        state[name] = False
    return state


def load_session_state(path: str) -> dict | None:
    try:
        with open(path, "r", encoding="utf-8") as handle:
            return json.load(handle)
    except FileNotFoundError:
        return None


def save_session_state(path: str, state: dict) -> None:
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(state, handle, indent=2)
        handle.write("\n")


@dataclass(frozen=True)
class SessionCheckResult:
    ok: bool
    reason: str = ""
    age_s: float | None = None
    false_fields: tuple[str, ...] = field(default_factory=tuple)


def check_session_state_ready(
    session: dict | None,
    max_age_s: float = DEFAULT_MAX_AGE_S,
    now: datetime | None = None,
) -> SessionCheckResult:
    """A session is ready only if: the file exists, its timestamp is
    present and parseable, that timestamp is not in the future, it is
    not older than max_age_s (a stale file from an earlier session must
    never be silently treated as still valid), and every required field
    is literally True (not merely truthy) -- there is no default that
    counts as confirmed.
    """
    if session is None:
        return SessionCheckResult(ok=False, reason="SESSION_FILE_MISSING")

    raw_ts = session.get(TIMESTAMP_KEY)
    if not raw_ts:
        return SessionCheckResult(ok=False, reason="MISSING_TIMESTAMP")
    try:
        ts = datetime.fromisoformat(raw_ts)
    except (TypeError, ValueError):
        return SessionCheckResult(ok=False, reason="UNPARSEABLE_TIMESTAMP")
    if ts.tzinfo is None:
        ts = ts.replace(tzinfo=timezone.utc)

    now = now or datetime.now(timezone.utc)
    age_s = (now - ts).total_seconds()
    if age_s < 0:
        return SessionCheckResult(ok=False, reason="TIMESTAMP_IN_FUTURE", age_s=age_s)
    if age_s > max_age_s:
        return SessionCheckResult(ok=False, reason="SESSION_STALE", age_s=age_s)

    false_fields = tuple(name for name in REQUIRED_SESSION_FIELDS if session.get(name) is not True)
    if false_fields:
        return SessionCheckResult(ok=False, reason="CONFIRMATIONS_NOT_TRUE", age_s=age_s, false_fields=false_fields)

    return SessionCheckResult(ok=True, age_s=age_s)


def main(argv=None) -> int:
    import argparse

    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)

    init_p = sub.add_parser("init", help="create/reset a fresh session file (all confirmations false)")
    init_p.add_argument("--path", required=True)

    set_p = sub.add_parser("set", help="flip one confirmation to true in an existing, non-stale session file")
    set_p.add_argument("--path", required=True)
    set_p.add_argument("--field", required=True, choices=REQUIRED_SESSION_FIELDS)

    check_p = sub.add_parser("check", help="report whether the session file is ready")
    check_p.add_argument("--path", required=True)
    check_p.add_argument("--max-age-s", type=float, default=DEFAULT_MAX_AGE_S)

    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    if args.command == "init":
        save_session_state(args.path, fresh_session_state())
        print(f"SESSION_STATE_INITIALIZED path={args.path}")
        return 0

    if args.command == "set":
        session = load_session_state(args.path)
        if session is None:
            print("SESSION_SET_BLOCKED=SESSION_FILE_MISSING (run 'init' first)")
            return 1
        precheck = check_session_state_ready(session, max_age_s=DEFAULT_MAX_AGE_S)
        if precheck.reason == "SESSION_STALE":
            print(
                f"SESSION_SET_BLOCKED=SESSION_STALE age_s={precheck.age_s} "
                "(re-run 'init' -- a stale session must never be silently extended)"
            )
            return 1
        if precheck.reason in ("MISSING_TIMESTAMP", "UNPARSEABLE_TIMESTAMP", "TIMESTAMP_IN_FUTURE"):
            print(f"SESSION_SET_BLOCKED={precheck.reason} (re-run 'init')")
            return 1
        session[args.field] = True
        save_session_state(args.path, session)
        print(f"SESSION_FIELD_SET field={args.field} value=true")
        return 0

    if args.command == "check":
        session = load_session_state(args.path)
        result = check_session_state_ready(session, max_age_s=args.max_age_s)
        if result.ok:
            print("SESSION_STATE_CHECK=PASS")
            return 0
        print(f"SESSION_STATE_CHECK=BLOCKED reason={result.reason} false_fields={list(result.false_fields)}")
        return 1

    return 1


if __name__ == "__main__":
    raise SystemExit(main())

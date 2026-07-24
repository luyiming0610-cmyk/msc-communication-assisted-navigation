#!/usr/bin/env python3
"""Offline + read-only-physical preflight checks for the HIL trial.

Pure-logic checks live at module level with no ROS/network dependency
so they are unit-testable. The `main()` CLI wraps them for use from
`run_hil_shared_exit_trial.sh` in --check-only mode. This module never
publishes to cmd_vel, never starts Webots/driver/bridge processes, and
never connects to the physical robot -- connectivity checks that DO
touch the network live in `hil_physical_connectivity_check` below,
guarded separately, and are still strictly read-only (no writes, no
nonzero commands).
"""
from __future__ import annotations

import json
import sys
from dataclasses import dataclass, field

UNCONFIRMED = "UNCONFIRMED_PHYSICAL_MEASUREMENT"


@dataclass(frozen=True)
class ParamCheckResult:
    ok: bool
    unconfirmed_paths: tuple[str, ...] = field(default_factory=tuple)
    missing_paths: tuple[str, ...] = field(default_factory=tuple)


def _resolve(params: dict, dotted_path: str):
    node = params
    for part in dotted_path.split("."):
        if not isinstance(node, dict) or part not in node:
            return None, False
        node = node[part]
    return node, True


def check_required_params_confirmed(params: dict) -> ParamCheckResult:
    """Checks every path in params['required_before_ground_motion'] is
    present and not literally UNCONFIRMED_PHYSICAL_MEASUREMENT.

    This function alone is what stands between the launcher and ever
    allowing ground motion with a fabricated or guessed physical value:
    it treats "missing" and "still UNCONFIRMED" identically -- both
    block -- there is no silent default.
    """
    required = params.get("required_before_ground_motion", [])
    unconfirmed: list[str] = []
    missing: list[str] = []
    for dotted_path in required:
        value, found = _resolve(params, dotted_path)
        if not found:
            missing.append(dotted_path)
        elif value == UNCONFIRMED:
            unconfirmed.append(dotted_path)
    ok = not unconfirmed and not missing
    return ParamCheckResult(ok=ok, unconfirmed_paths=tuple(unconfirmed), missing_paths=tuple(missing))


def check_required_fields_ready(params: dict) -> ParamCheckResult:
    """Like check_required_params_confirmed(), but also aware of
    boolean confirmation fields (added for the first ground diagnostic,
    which mixes numeric UNCONFIRMED_PHYSICAL_MEASUREMENT geometry
    fields with boolean "has this been checked/recorded" confirmation
    fields in the same required_before_ground_motion list). A path is
    NOT ready if it is missing, still the literal UNCONFIRMED string, or
    a boolean that is still `False` -- there is no default that counts
    as ready. check_required_params_confirmed() itself is unchanged and
    still used as-is wherever only numeric fields are involved.
    """
    required = params.get("required_before_ground_motion", [])
    unconfirmed: list[str] = []
    missing: list[str] = []
    for dotted_path in required:
        value, found = _resolve(params, dotted_path)
        if not found:
            missing.append(dotted_path)
        elif value == UNCONFIRMED:
            unconfirmed.append(dotted_path)
        elif isinstance(value, bool) and value is False:
            unconfirmed.append(dotted_path)
    ok = not unconfirmed and not missing
    return ParamCheckResult(ok=ok, unconfirmed_paths=tuple(unconfirmed), missing_paths=tuple(missing))


@dataclass(frozen=True)
class NamespaceCheckResult:
    ok: bool
    conflicts: tuple[str, ...] = field(default_factory=tuple)


def check_namespace_conflicts(
    physical_namespace: str,
    virtual_namespace: str,
    reserved_namespaces: tuple[str, ...] = ("epuck1", "epuck2", "epuck3"),
) -> NamespaceCheckResult:
    """Rejects any namespace collision between the HIL physical robot,
    the HIL virtual peer, and the reserved N2/N3 formal-batch robot
    namespaces (epuck1/epuck2/epuck3) -- a collision would let a stray
    HIL node publish onto a frozen formal-result topic, or vice versa.
    """
    conflicts = []
    if physical_namespace == virtual_namespace:
        conflicts.append(f"PHYSICAL_EQUALS_VIRTUAL({physical_namespace})")
    for reserved in reserved_namespaces:
        if physical_namespace == reserved:
            conflicts.append(f"PHYSICAL_COLLIDES_WITH_RESERVED({reserved})")
        if virtual_namespace == reserved:
            conflicts.append(f"VIRTUAL_COLLIDES_WITH_RESERVED({reserved})")
    return NamespaceCheckResult(ok=not conflicts, conflicts=tuple(conflicts))


@dataclass(frozen=True)
class ProtocolVersionCheckResult:
    ok: bool
    reason: str = ""


def check_protocol_version(declared_version: int, expected_version: int) -> ProtocolVersionCheckResult:
    if declared_version != expected_version:
        return ProtocolVersionCheckResult(
            ok=False,
            reason=f"PROTOCOL_VERSION_MISMATCH(declared={declared_version},expected={expected_version})",
        )
    return ProtocolVersionCheckResult(ok=True)


def load_frozen_params(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def run_offline_checks(params_path: str) -> dict:
    """Runs every check that requires no network/ROS/process access.
    Returns a plain dict so the launcher shell script can consume it
    as JSON without importing this module directly.
    """
    params = load_frozen_params(params_path)
    param_result = check_required_params_confirmed(params)
    ns_result = check_namespace_conflicts(
        physical_namespace="epuck5809",
        virtual_namespace="epuck_virtual_peer",
    )
    proto_result = check_protocol_version(
        declared_version=params.get("frozen_protocol", {}).get("epuck_state_protocol_version", -1),
        expected_version=1,
    )

    all_ok = param_result.ok and ns_result.ok and proto_result.ok
    if all_ok:
        status = "OFFLINE_CHECKS_PASS"
    elif param_result.unconfirmed_paths or param_result.missing_paths:
        status = "BLOCKED_AWAITING_LAB_MEASUREMENT"
    else:
        status = "OFFLINE_CHECKS_FAIL"

    return {
        "status": status,
        "param_check_ok": param_result.ok,
        "unconfirmed_paths": list(param_result.unconfirmed_paths),
        "missing_paths": list(param_result.missing_paths),
        "namespace_check_ok": ns_result.ok,
        "namespace_conflicts": list(ns_result.conflicts),
        "protocol_version_check_ok": proto_result.ok,
        "protocol_version_reason": proto_result.reason,
    }


def main(argv=None) -> int:
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--frozen-params", required=True)
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    result = run_offline_checks(args.frozen_params)
    print(json.dumps(result, indent=2))
    return 0 if result["status"] == "OFFLINE_CHECKS_PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Pure decision logic for sync_and_build_epuck2_comm.sh -- hardened
2026-07-23 after review found the original version defaulted to an
executing, `rsync --delete` sync with no path validation, no
deletion-safety check, no backup, and no confirmation gate.

Every function here is plain data in, plain data out -- no
subprocess, no filesystem writes, no rsync invocation -- so the exact
decisions the shell script makes (abort or proceed, and why) can be
unit tested against synthetic inputs without ever touching a real
directory, package, or workspace.
"""
from __future__ import annotations

import json
import os
from dataclasses import dataclass, field

REQUIRED_CONFIRM_TOKEN = "CONFIRM_SYNC_EXECUTE"

# Deliberately obscure, never referenced by any runbook or normal
# command -- exists ONLY so the synthetic end-to-end test
# (test_sync_and_build_epuck2_comm_e2e.sh) can exercise this script's
# full logic (path validation through to a real rsync execute) against
# temporary synthetic directories without ever touching the real
# workspace. Any real invocation of this script leaves this variable
# unset and gets the real whitelist below.
_TEST_OVERRIDE_ENV_VAR = "SYNC_EPUCK2_COMM_TEST_EXPECTED_DESTINATIONS_JSON"


def _load_expected_destinations() -> dict:
    override = os.environ.get(_TEST_OVERRIDE_ENV_VAR)
    if override:
        return json.loads(override)
    # The ONLY destinations this script is ever allowed to sync into,
    # for a real (non-test) invocation. Anything else -- a typo'd
    # path, a symlink resolving somewhere unexpected, an unrelated
    # workspace package -- must abort, not proceed with a best-effort
    # guess.
    return {
        "epuck2_comm": "/home/eamon/epuck_ws/src/epuck2_comm",
        "epuck2_comm_interfaces": "/home/eamon/epuck_ws/src/epuck2_comm_interfaces",
    }


EXPECTED_DESTINATIONS = _load_expected_destinations()


@dataclass(frozen=True)
class PathValidationResult:
    ok: bool
    reason: str = ""


def validate_absolute_path(path: str) -> PathValidationResult:
    """Rejects empty, relative, or `..`-containing paths outright --
    a sync target must be an unambiguous, fully-resolved absolute path,
    never something that depends on the caller's current directory."""
    if not path:
        return PathValidationResult(ok=False, reason="EMPTY_PATH")
    if not os.path.isabs(path):
        return PathValidationResult(ok=False, reason=f"NOT_ABSOLUTE({path})")
    if ".." in path.split(os.sep):
        return PathValidationResult(ok=False, reason=f"CONTAINS_DOTDOT({path})")
    return PathValidationResult(ok=True)


def validate_destination_is_expected(
    package_name: str, resolved_destination: str, expected: dict = EXPECTED_DESTINATIONS
) -> PathValidationResult:
    """Aborts unless the resolved destination is EXACTLY the one
    intended path for this package -- not "looks like it", not "same
    basename", not "under the same parent" -- byte-for-byte equal
    after path resolution, so a symlink, a typo, or an unrelated
    package can never be silently synced into."""
    expected_path = expected.get(package_name)
    if expected_path is None:
        return PathValidationResult(ok=False, reason=f"UNKNOWN_PACKAGE({package_name})")
    if resolved_destination != expected_path:
        return PathValidationResult(
            ok=False,
            reason=(
                f"DESTINATION_MISMATCH(package={package_name}, "
                f"resolved={resolved_destination}, expected={expected_path})"
            ),
        )
    return PathValidationResult(ok=True)


@dataclass(frozen=True)
class ItemizedChange:
    action: str  # "delete" or "update"
    relative_path: str


def parse_itemized_changes(rsync_dry_run_output: str) -> tuple:
    """Parses `rsync -n --itemize-changes` output into a tuple of
    ItemizedChange. Deletion lines look like `*deleting   some/path`;
    every other non-empty line is treated as an update/create (the
    itemize flag characters themselves are not otherwise interpreted --
    this script only needs to distinguish "would be deleted" from
    "would be changed/created" to decide whether the deletion-safety
    check applies)."""
    changes = []
    for line in rsync_dry_run_output.splitlines():
        line = line.rstrip()
        if not line:
            continue
        if line.startswith("*deleting"):
            rel = line[len("*deleting"):].strip()
            changes.append(ItemizedChange(action="delete", relative_path=rel))
        elif line[0] in ">c<h.*" and len(line) > 12:
            rel = line[11:].strip()
            if rel:
                changes.append(ItemizedChange(action="update", relative_path=rel))
    return tuple(changes)


@dataclass(frozen=True)
class DeletionSafetyResult:
    ok: bool
    unexpected_deletions: tuple = field(default_factory=tuple)


def find_unexpected_deletions(
    changes: tuple, tracked_files: set
) -> DeletionSafetyResult:
    """A deletion is "expected" only if the exact same relative path is
    git-tracked in the source tree -- i.e., rsync --delete is about to
    remove a destination file specifically because the source (the
    real, intended package content) no longer has it. Any deletion
    target NOT in the source's own tracked-file list is an unexpected/
    untracked destination file (a stray local edit, a scratch file, a
    __pycache__ artifact that slipped past .gitignore, etc.) -- this
    function flags every one of those; the caller must abort rather
    than silently destroy them."""
    unexpected = tuple(
        change.relative_path
        for change in changes
        if change.action == "delete" and change.relative_path not in tracked_files
    )
    return DeletionSafetyResult(ok=not unexpected, unexpected_deletions=unexpected)


def check_confirm_token(provided: str, expected: str = REQUIRED_CONFIRM_TOKEN) -> bool:
    """Plain equality -- deliberately not hashed/obscured. The point of
    this token is not secrecy, it is forcing a deliberate, separate,
    typed-out confirmation distinct from just passing --execute, so an
    --execute flag copy-pasted from an earlier command/history entry
    can never silently re-trigger a destructive sync."""
    return provided == expected


@dataclass(frozen=True)
class ExecuteGateResult:
    ok: bool
    reason: str = ""


def check_execute_gate(*, execute_requested: bool, confirm_token: str) -> ExecuteGateResult:
    """The single gate standing between --check-only (always safe) and
    an actual destructive sync: both --execute AND a correct,
    separately-supplied confirmation token are required. Neither alone
    is sufficient."""
    if not execute_requested:
        return ExecuteGateResult(ok=False, reason="CHECK_ONLY_MODE_NO_EXECUTE_REQUESTED")
    if not check_confirm_token(confirm_token):
        return ExecuteGateResult(ok=False, reason="MISSING_OR_INCORRECT_CONFIRM_TOKEN")
    return ExecuteGateResult(ok=True)


if __name__ == "__main__":
    import argparse
    import sys

    parser = argparse.ArgumentParser(
        description="Parse an rsync -n --itemize-changes report and check it for "
        "unexpected/untracked deletions, given a tracked-files list (one path per line)."
    )
    parser.add_argument("--rsync-dry-run-output", required=True, help="path to a saved rsync -n report")
    parser.add_argument("--tracked-files", required=True, help="path to a newline-separated tracked-file list")
    args = parser.parse_args()

    with open(args.rsync_dry_run_output, encoding="utf-8") as fh:
        report = fh.read()
    with open(args.tracked_files, encoding="utf-8") as fh:
        tracked = {line.strip() for line in fh if line.strip()}

    changes = parse_itemized_changes(report)
    result = find_unexpected_deletions(changes, tracked)
    if result.ok:
        print("DELETION_SAFETY_CHECK_PASS")
        sys.exit(0)
    print("DELETION_SAFETY_CHECK_FAIL")
    for path in result.unexpected_deletions:
        print(f"UNEXPECTED_DELETION: {path}")
    sys.exit(1)

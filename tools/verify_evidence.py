#!/usr/bin/env python3
"""Read-only verification of the dissertation evidence package."""

from __future__ import annotations

import argparse
import csv
import hashlib
import re
import sys
from pathlib import Path


LFS_HEADER = b"version https://git-lfs.github.com/spec/v1\n"
LFS_OID_RE = re.compile(rb"^oid sha256:([0-9a-f]{64})$", re.MULTILINE)
LFS_SIZE_RE = re.compile(rb"^size ([0-9]+)$", re.MULTILINE)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_lfs_pointer(path: Path) -> tuple[str, int] | None:
    with path.open("rb") as stream:
        data = stream.read(512)
    if not data.startswith(LFS_HEADER):
        return None
    oid = LFS_OID_RE.search(data)
    size = LFS_SIZE_RE.search(data)
    if oid is None or size is None:
        raise ValueError("malformed Git LFS pointer")
    return oid.group(1).decode("ascii"), int(size.group(1))


def verify_bags(repo: Path, inventory: Path, lfs_mode: str) -> tuple[int, int]:
    checked = 0
    failures = 0
    pointer_count = 0
    with inventory.open(newline="", encoding="utf-8-sig") as stream:
        rows = list(csv.DictReader(stream))

    for row in rows:
        checked += 1
        path = repo / row["relative_path"]
        expected_size = int(row["size_bytes"])
        expected_hash = row["sha256"].lower()
        if not path.is_file():
            print(f"FAIL missing: {row['relative_path']}")
            failures += 1
            continue

        try:
            pointer = read_lfs_pointer(path)
        except ValueError as exc:
            print(f"FAIL {row['relative_path']}: {exc}")
            failures += 1
            continue

        if pointer is not None:
            pointer_count += 1
            oid, declared_size = pointer
            if lfs_mode == "materialized":
                print(f"FAIL LFS object not materialised: {row['relative_path']}")
                failures += 1
            elif oid != expected_hash or declared_size != expected_size:
                print(f"FAIL LFS pointer mismatch: {row['relative_path']}")
                failures += 1
            continue

        actual_size = path.stat().st_size
        if actual_size != expected_size:
            print(
                f"FAIL size {row['relative_path']}: "
                f"expected {expected_size}, found {actual_size}"
            )
            failures += 1
            continue
        actual_hash = sha256_file(path)
        if actual_hash != expected_hash:
            print(f"FAIL SHA-256: {row['relative_path']}")
            failures += 1

    print(
        f"Bag inventory: {checked - failures}/{checked} entries verified "
        f"({pointer_count} Git LFS pointers inspected)."
    )
    return checked, failures


def verify_sha256_manifest(evidence_dir: Path, manifest: Path) -> tuple[int, int]:
    checked = 0
    failures = 0
    for line_number, raw_line in enumerate(
        manifest.read_text(encoding="utf-8").splitlines(), start=1
    ):
        line = raw_line.strip()
        if not line:
            continue
        parts = line.split(maxsplit=1)
        if len(parts) != 2 or not re.fullmatch(r"[0-9a-fA-F]{64}", parts[0]):
            print(f"FAIL malformed manifest line {line_number}: {manifest}")
            failures += 1
            continue
        expected_hash, relative_name = parts
        relative_name = relative_name.lstrip("* ")
        if relative_name.startswith("./"):
            relative_name = relative_name[2:]
        path = evidence_dir / relative_name
        checked += 1
        if not path.is_file():
            print(f"FAIL missing Stage 4 file: {relative_name}")
            failures += 1
        elif sha256_file(path) != expected_hash.lower():
            print(f"FAIL Stage 4 SHA-256: {relative_name}")
            failures += 1
    print(f"Stage 4 manifest: {checked - failures}/{checked} files verified.")
    return checked, failures


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--root",
        type=Path,
        default=Path(__file__).resolve().parents[1],
        help="repository root (default: inferred from this script)",
    )
    parser.add_argument(
        "--lfs-mode",
        choices=("materialized", "pointer"),
        default="materialized",
        help="require downloaded bag data or verify pointer identity only",
    )
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument("--bags-only", action="store_true")
    selection.add_argument("--stage4-only", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    repo = args.root.resolve()
    total_checked = 0
    total_failures = 0

    if not args.stage4_only:
        checked, failures = verify_bags(
            repo,
            repo / "docs" / "RAW_ROSBAG_INVENTORY.csv",
            args.lfs_mode,
        )
        total_checked += checked
        total_failures += failures

    if not args.bags_only:
        evidence_dir = (
            repo
            / "experiments"
            / "07_reality_gap"
            / "hil_single_real_shared_exit_20260723"
            / "formal_evidence"
            / "stage4_20260803_144220"
        )
        checked, failures = verify_sha256_manifest(
            evidence_dir, evidence_dir / "FINAL_SHA256SUMS.txt"
        )
        total_checked += checked
        total_failures += failures

    if total_failures:
        print(f"Evidence verification FAILED: {total_failures} issue(s).")
        return 2
    print(f"Evidence verification PASSED: {total_checked} entries checked.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

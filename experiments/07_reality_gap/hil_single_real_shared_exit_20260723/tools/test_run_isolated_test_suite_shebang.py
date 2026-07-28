#!/usr/bin/env python3
"""Regression guard for the run_isolated_test_suite.sh BOM defect found
2026-07-28: a PowerShell text-replace step used to renumber that
script's step labels silently prepended a UTF-8 byte-order-mark before
the shebang. `bash run_isolated_test_suite.sh` tolerated it in every
run this session, but a direct `./run_isolated_test_suite.sh`
invocation relies on the kernel finding the literal bytes `#!` at byte
offset 0 -- a leading BOM breaks that. This test reads the file as raw
bytes (never decoded as text, so a BOM cannot be silently absorbed by
a codec) and would have caught the defect.
"""
import unittest
from pathlib import Path

SCRIPT_PATH = Path(__file__).parent / "run_isolated_test_suite.sh"


class RunIsolatedTestSuiteShebangTest(unittest.TestCase):
    def test_file_starts_with_the_literal_shebang_bytes_no_bom(self):
        with open(SCRIPT_PATH, "rb") as fh:
            first_two_bytes = fh.read(2)
        self.assertEqual(
            first_two_bytes,
            b"#!",
            f"expected literal shebang at byte offset 0, got {first_two_bytes!r} (possible BOM)",
        )


if __name__ == "__main__":
    unittest.main()

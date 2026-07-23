#!/usr/bin/env python3
"""Unit tests for sync_epuck2_comm_logic.py's pure decision functions.
Every test uses synthetic strings/dicts/sets only -- no real directory,
no rsync invocation, no git repo, and never anything under
~/epuck_ws. This is exactly what item 2 of the 2026-07-23 hardening
request required: test the sync logic against synthetic inputs only.
"""
import unittest

from sync_epuck2_comm_logic import (
    EXPECTED_DESTINATIONS,
    REQUIRED_CONFIRM_TOKEN,
    check_confirm_token,
    check_execute_gate,
    find_unexpected_deletions,
    parse_itemized_changes,
    validate_absolute_path,
    validate_destination_is_expected,
)


class ValidateAbsolutePathTest(unittest.TestCase):
    def test_valid_absolute_path_passes(self):
        result = validate_absolute_path("/home/eamon/epuck_ws/src/epuck2_comm")
        self.assertTrue(result.ok)

    def test_empty_path_fails(self):
        result = validate_absolute_path("")
        self.assertFalse(result.ok)
        self.assertEqual(result.reason, "EMPTY_PATH")

    def test_relative_path_fails(self):
        result = validate_absolute_path("epuck_ws/src/epuck2_comm")
        self.assertFalse(result.ok)
        self.assertIn("NOT_ABSOLUTE", result.reason)

    def test_dotdot_traversal_fails(self):
        result = validate_absolute_path("/home/eamon/epuck_ws/src/../../../etc")
        self.assertFalse(result.ok)
        self.assertIn("CONTAINS_DOTDOT", result.reason)


class ValidateDestinationIsExpectedTest(unittest.TestCase):
    def test_exact_expected_path_passes(self):
        result = validate_destination_is_expected(
            "epuck2_comm", "/home/eamon/epuck_ws/src/epuck2_comm"
        )
        self.assertTrue(result.ok)

    def test_unknown_package_name_fails(self):
        result = validate_destination_is_expected(
            "some_other_package", "/home/eamon/epuck_ws/src/some_other_package"
        )
        self.assertFalse(result.ok)
        self.assertIn("UNKNOWN_PACKAGE", result.reason)

    def test_wrong_destination_for_known_package_fails(self):
        # Same package name, but a resolved path that does NOT exactly
        # match -- e.g. a trailing slash difference, a symlink
        # resolving elsewhere, or a typo'd sibling directory.
        result = validate_destination_is_expected(
            "epuck2_comm", "/home/eamon/epuck_ws/src/epuck2_comm_old_backup"
        )
        self.assertFalse(result.ok)
        self.assertIn("DESTINATION_MISMATCH", result.reason)

    def test_using_a_custom_expected_map_is_honored(self):
        custom = {"my_pkg": "/tmp/synthetic_dest/my_pkg"}
        result = validate_destination_is_expected(
            "my_pkg", "/tmp/synthetic_dest/my_pkg", expected=custom
        )
        self.assertTrue(result.ok)

    def test_real_expected_destinations_are_absolute(self):
        for path in EXPECTED_DESTINATIONS.values():
            self.assertTrue(validate_absolute_path(path).ok)


class ParseItemizedChangesTest(unittest.TestCase):
    def test_deletion_lines_are_parsed(self):
        output = "\n".join(
            [
                "*deleting   some_stray_file.py",
                "*deleting   __pycache__/old.cpython-310.pyc",
            ]
        )
        changes = parse_itemized_changes(output)
        deletions = [c.relative_path for c in changes if c.action == "delete"]
        self.assertEqual(
            deletions, ["some_stray_file.py", "__pycache__/old.cpython-310.pyc"]
        )

    def test_update_lines_are_not_classified_as_deletions(self):
        output = ">f+++++++++ test_new_file.py"
        changes = parse_itemized_changes(output)
        self.assertTrue(all(c.action != "delete" for c in changes))

    def test_empty_output_yields_no_changes(self):
        self.assertEqual(parse_itemized_changes(""), ())

    def test_blank_lines_are_ignored(self):
        output = "\n\n*deleting   x.py\n\n"
        changes = parse_itemized_changes(output)
        self.assertEqual(len(changes), 1)


class FindUnexpectedDeletionsTest(unittest.TestCase):
    def test_deletion_of_a_tracked_file_is_expected_ok(self):
        # Source genuinely no longer has this file -- rsync --delete
        # removing it from the destination is the CORRECT, intended
        # behavior, not a hazard.
        from sync_epuck2_comm_logic import ItemizedChange

        changes = (ItemizedChange(action="delete", relative_path="old_test.py"),)
        tracked = {"old_test.py", "cooperative_avoider.py"}
        result = find_unexpected_deletions(changes, tracked)
        self.assertTrue(result.ok)
        self.assertEqual(result.unexpected_deletions, ())

    def test_deletion_of_an_untracked_file_is_flagged(self):
        from sync_epuck2_comm_logic import ItemizedChange

        changes = (
            ItemizedChange(action="delete", relative_path="my_local_scratch_notes.py"),
        )
        tracked = {"cooperative_avoider.py"}
        result = find_unexpected_deletions(changes, tracked)
        self.assertFalse(result.ok)
        self.assertIn("my_local_scratch_notes.py", result.unexpected_deletions)

    def test_mixed_expected_and_unexpected_deletions_flags_only_unexpected(self):
        from sync_epuck2_comm_logic import ItemizedChange

        changes = (
            ItemizedChange(action="delete", relative_path="tracked_removed.py"),
            ItemizedChange(action="delete", relative_path="untracked_local.py"),
        )
        tracked = {"tracked_removed.py"}
        result = find_unexpected_deletions(changes, tracked)
        self.assertFalse(result.ok)
        self.assertEqual(result.unexpected_deletions, ("untracked_local.py",))

    def test_non_deletion_changes_never_trigger_the_safety_check(self):
        from sync_epuck2_comm_logic import ItemizedChange

        changes = (ItemizedChange(action="update", relative_path="anything.py"),)
        result = find_unexpected_deletions(changes, tracked_files=set())
        self.assertTrue(result.ok)


class CheckConfirmTokenTest(unittest.TestCase):
    def test_correct_token_passes(self):
        self.assertTrue(check_confirm_token(REQUIRED_CONFIRM_TOKEN))

    def test_wrong_token_fails(self):
        self.assertFalse(check_confirm_token("wrong"))

    def test_empty_token_fails(self):
        self.assertFalse(check_confirm_token(""))

    def test_token_is_case_sensitive(self):
        self.assertFalse(check_confirm_token(REQUIRED_CONFIRM_TOKEN.lower()))


class CheckExecuteGateTest(unittest.TestCase):
    def test_no_execute_requested_is_check_only_and_blocked(self):
        result = check_execute_gate(execute_requested=False, confirm_token=REQUIRED_CONFIRM_TOKEN)
        self.assertFalse(result.ok)
        self.assertEqual(result.reason, "CHECK_ONLY_MODE_NO_EXECUTE_REQUESTED")

    def test_execute_requested_without_correct_token_is_blocked(self):
        result = check_execute_gate(execute_requested=True, confirm_token="")
        self.assertFalse(result.ok)
        self.assertEqual(result.reason, "MISSING_OR_INCORRECT_CONFIRM_TOKEN")

    def test_execute_requested_with_wrong_token_is_blocked(self):
        result = check_execute_gate(execute_requested=True, confirm_token="not_the_token")
        self.assertFalse(result.ok)

    def test_execute_requested_with_correct_token_passes(self):
        result = check_execute_gate(execute_requested=True, confirm_token=REQUIRED_CONFIRM_TOKEN)
        self.assertTrue(result.ok)

    def test_execute_flag_alone_is_never_sufficient(self):
        # The exact hazard this gate exists to prevent: --execute
        # copy-pasted from shell history without the separate token.
        result = check_execute_gate(execute_requested=True, confirm_token="")
        self.assertFalse(result.ok)


if __name__ == "__main__":
    unittest.main()

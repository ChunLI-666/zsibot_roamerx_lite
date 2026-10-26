#!/usr/bin/env python3
"""Validate budget arguments before the container starts ROS components."""
from pathlib import Path
import subprocess
import sys
import unittest


BINARY = Path(sys.argv.pop(1)).resolve()


class BudgetArguments(unittest.TestCase):
    def run_cli(self, *arguments):
        return subprocess.run([str(BINARY), *arguments], capture_output=True,
                              text=True, timeout=5)

    def test_auto_and_explicit_budgets_parse_without_starting_ros(self):
        for arguments in (('--help',), ('--executor-threads', '0', '--help'),
                          ('--executor-threads', '1', '--help'),
                          ('--executor-threads=2', '--help'),
                          ('--use_multi_threaded_executor', '--executor-threads', '4', '--help'),
                          ('--executor-threads', '2', '--help', '--ros-args',
                           '-r', '__node:=budget_arguments')):
            with self.subTest(arguments=arguments):
                result = self.run_cli(*arguments)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn('worker count per isolated component', result.stdout)

    def test_invalid_budget_never_falls_back_to_hardware_concurrency(self):
        for value in ('-1', '2.5', '', 'NaN', '257', '999999999999999999999'):
            with self.subTest(value=value):
                result = self.run_cli('--executor-threads', value)
                self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
                self.assertIn('requires 0 (auto)', result.stderr)

    def test_missing_or_unknown_option_is_an_error(self):
        for arguments in (('--executor-threads',), ('--executor-thread', '2')):
            with self.subTest(arguments=arguments):
                result = self.run_cli(*arguments)
                self.assertEqual(result.returncode, 2, result.stdout + result.stderr)


if __name__ == '__main__':
    unittest.main()

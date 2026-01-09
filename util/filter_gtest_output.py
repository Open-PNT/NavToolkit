#!/usr/bin/env python3

'''
Google Test output minifier.

Usage: filter_gtest_output.py <test_binary> [args...]

     <test_binary> -- An executable that uses googletest's RUN_ALL_TESTS()
     [args...] -- Arguments to pass through to the test binary, including
                  things like --gtest_filter.

This program runs your test binary in a virtual TTY and displays a subset of
its output, only showing you the output from tests that failed or that wrote
something to standard out.
'''

import re
import sys
from subprocess import Popen, PIPE, STDOUT


def gtest_leader(line):
    esc = "\x1b"
    m = re.search(r'\A(?:%s.*?m)*\[([=-]{10}| +\w+ +)\] ' % esc, line)
    if m is not None:
        return m.group(1).strip()


class GtestOutputStateMachine:
    section = 0
    last_suite_start = None
    last_suite_noise = False
    last_test_start = None
    last_test_noise = False
    last_print_was_blank = False
    noisy_successes = 0

    def _suppress_consecutive_blanks(self, line):
        if line or not self.last_print_was_blank:
            print(line)
        self.last_print_was_blank = not line

    def on_unknown_line(self, line):
        if line:
            if self.last_suite_start is not None:
                self._suppress_consecutive_blanks(self.last_suite_start)
                self.last_suite_start = None
            if self.last_test_start is not None:
                self._suppress_consecutive_blanks(self.last_test_start)
                self.last_test_start = None
            self.last_suite_noise = self.last_test_noise = True
        self._suppress_consecutive_blanks(line)

    def on_section_boundary(self, line):
        self.section += 1
        self._suppress_consecutive_blanks(line)

    def on_suite_boundary(self, line):
        self.last_suite_start = line
        if self.last_suite_noise:
            self._suppress_consecutive_blanks(line)
        self.last_suite_noise = False

    def on_test_run(self, line):
        self.last_test_start = line
        self.last_test_noise = False

    def on_test_ok(self, line):
        if self.last_test_noise:
            self._suppress_consecutive_blanks(line)
            self.noisy_successes += 1

    def on_test_failed(self, line):
        self.last_test_noise = True
        self._suppress_consecutive_blanks(line)

    on_test_skipped = on_test_failed

    def on_receive_line(self, line):
        dispatcher = {
            '=' * 10: self.on_section_boundary,
            None: self._suppress_consecutive_blanks,
        }
        if self.section == 1:
            dispatcher.update(
                {
                    '-' * 10: self.on_suite_boundary,
                    'RUN': self.on_test_run,
                    'OK': self.on_test_ok,
                    'FAILED': self.on_test_failed,
                    'SKIPPED': self.on_test_skipped,
                    None: self.on_unknown_line,
                }
            )
        return dispatcher.get(gtest_leader(line), dispatcher[None])(line)


def call_popen(argv):
    try:
        return Popen(['unbuffer'] + argv, stderr=STDOUT, stdout=PIPE)
    except FileNotFoundError:
        print(
            "To enable spdlog color-coded output, install `unbuffer`, which"
            " is usually included in your OS's `expect` package."
        )
        return Popen(argv, stderr=STDOUT, stdout=PIPE)


def filter_gtest_output(*test_argv):
    test_argv = list(test_argv)
    if not any(it.startswith('--gtest_color=') for it in test_argv):
        test_argv.append('--gtest_color=yes')

    sm = GtestOutputStateMachine()
    try:
        with call_popen(test_argv) as proc:
            for line in proc.stdout:
                sm.on_receive_line(line.decode().strip())
            return proc.wait()
    finally:
        if sm.noisy_successes:
            print('Tests with log noise: %r' % sm.noisy_successes)


if __name__ == '__main__':
    if not sys.argv[1:]:
        sys.stderr.write(__doc__)
        sys.exit(1)
    sys.exit(filter_gtest_output(*sys.argv[1:]))

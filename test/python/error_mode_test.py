#!/usr/bin/env python3

from navtk import (
    set_global_error_mode,
    get_global_error_mode,
    ErrorMode,
    ErrorModeLock,
    log_or_throw,
)
from contextlib import contextmanager
import unittest
import os
from threading import Thread, RLock
from util.redirect import redirected_outputs

os.environ['TERM'] = 'dumb'  # prevent spdlog from trying to be colorful


DEFAULT_EXCEPTION = RuntimeError


class CustomException(Exception):
    pass


def custom_factory(message):
    return CustomException(message)


def some_mode_other_than(mode):
    return ErrorMode.DIE if mode == ErrorMode.OFF else ErrorMode.OFF


def set_error_mode_when_unlocked(mode, lock):
    with lock:
        set_global_error_mode(mode)


class ErrorModeTests(unittest.TestCase):
    def setUp(self):
        self.initial_error_mode = get_global_error_mode()

    def tearDown(self):
        set_global_error_mode(self.initial_error_mode)

    @contextmanager
    def _log_or_throw_harness(self, expect, text, exc_type=DEFAULT_EXCEPTION):
        log_texts = []
        with redirected_outputs() as redir:
            if expect in (ErrorMode.LOG, ErrorMode.OFF):
                yield
            else:
                try:
                    yield
                except exc_type as exc:
                    log_texts.append(''.join(exc.args))
            log_texts.append(str(redir))
        for log_text in log_texts:
            if expect != ErrorMode.OFF and text:
                self.assertIn(text, log_text)
            else:
                self.assertEqual("", log_text)

    def _check_log_or_throw_overloads(self, mode):
        lvl_text = "custom" if mode == ErrorMode.DIE else "info"
        with ErrorModeLock(mode):
            with self._log_or_throw_harness(mode, "a 1 b 2"):
                log_or_throw("a {} b {}", 1, 2)

            with self._log_or_throw_harness(mode, lvl_text):
                log_or_throw(2, "custom level")

            with self._log_or_throw_harness(mode, "a 1 b 2", CustomException):
                log_or_throw(CustomException, "a {} b {}", 1, 2)

            with self._log_or_throw_harness(mode, lvl_text, CustomException):
                log_or_throw(CustomException, 2, "custom type and level")

            with self._log_or_throw_harness(mode, "a 1 b 2", CustomException):
                log_or_throw(custom_factory, "a {} b {}", 1, 2)

            with self._log_or_throw_harness(mode, lvl_text, CustomException):
                log_or_throw(custom_factory, 2, "custom type and level")

        with ErrorModeLock(some_mode_other_than(mode)):
            with self._log_or_throw_harness(mode, "a 1 b 2"):
                log_or_throw(mode, "a {} b {}", 1, 2)

            with self._log_or_throw_harness(mode, lvl_text):
                log_or_throw(2, mode, "custom level")

            with self._log_or_throw_harness(mode, "a 1 b 2", CustomException):
                log_or_throw(CustomException, mode, "a {} b {}", 1, 2)

            with self._log_or_throw_harness(mode, lvl_text, CustomException):
                log_or_throw(CustomException, 2, mode, "custom type and level")

            with self._log_or_throw_harness(mode, "a 1 b 2", CustomException):
                log_or_throw(custom_factory, mode, "a {} b {}", 1, 2)

            with self._log_or_throw_harness(mode, lvl_text, CustomException):
                log_or_throw(custom_factory, 2, mode, "custom type and level")

    def test_log_or_throw_with_error_mode_off(self):
        self._check_log_or_throw_overloads(ErrorMode.OFF)

    def test_log_or_throw_with_error_mode_log(self):
        self._check_log_or_throw_overloads(ErrorMode.LOG)

    def test_log_or_throw_with_error_mode_die(self):
        self._check_log_or_throw_overloads(ErrorMode.DIE)

    def test_error_mode_lock_context_manager_threading(self):
        set_global_error_mode(ErrorMode.OFF)
        lock = RLock()
        lock.acquire()
        bg = Thread(
            target=set_error_mode_when_unlocked, args=(ErrorMode.LOG, lock)
        )
        bg.start()
        with ErrorModeLock(ErrorMode.DIE):
            lock.release()
            # this join will fail because the background thread is waiting for
            # the ErrorModeLock to release.
            bg.join(0.1)
            self.assertTrue(
                bg.is_alive(), "Background thread should not have completed."
            )
            self.assertEqual(
                ErrorMode.DIE,
                get_global_error_mode(),
                "ErrorMode set by ErrorModeLock not preserved",
            )
        bg.join(0.1)
        self.assertFalse(
            bg.is_alive(), "Background thread should've completed."
        )
        self.assertEqual(
            ErrorMode.LOG,
            get_global_error_mode(),
            "Background thread did not set error mode.",
        )

    def test_error_mode_lock_context_manager_reenter(self):
        expected_restore = ErrorMode.LOG
        set_global_error_mode(expected_restore)
        target = ErrorModeLock(ErrorMode.DIE)
        for run in (1, 2):
            with target:
                self.assertEqual(
                    ErrorMode.DIE,
                    get_global_error_mode(),
                    "ErrorMode not set by ErrorModeLock, run=%d" % run,
                )
            self.assertEqual(
                expected_restore,
                get_global_error_mode(),
                "ErrorMode not restored by ErrorModeLock, run=%d" % run,
            )
            expected_restore = ErrorMode.OFF
            set_global_error_mode(expected_restore)

    def test_error_mode_lock_context_manager_nest(self):
        with ErrorModeLock(ErrorMode.DIE):
            self.assertEqual(ErrorMode.DIE, get_global_error_mode())
            with ErrorModeLock(ErrorMode.LOG):
                self.assertEqual(ErrorMode.LOG, get_global_error_mode())
            self.assertEqual(ErrorMode.DIE, get_global_error_mode())


if __name__ == '__main__':
    unittest.main()

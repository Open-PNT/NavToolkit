#!/usr/bin/env python3
"""
Runs a series of intentionally-invalid test programs to check whether
various address sanitizer features are supported on your platform/compiler.
"""

import argparse
from glob import glob
from os import environ, makedirs
from os.path import dirname, abspath, join, relpath, basename
import re
import subprocess
import sys

from buildkit.system import outcome, env
from buildkit.console import ViewFrame

SOURCE_DIR = join(dirname(abspath(__file__)), 'asan_tests')

TEST_PROGRAM_EXTENSION = '.cxx'

DEFAULT_COMPILER = (
    [environ[it] for it in ['CXX', 'CPP', 'CC'] if it in environ] or ['g++']
)[0]


def asan_source_path(name):
    '''
    Given a name, return the absolute path to an asan test with that name.
    '''
    return join(SOURCE_DIR, name + TEST_PROGRAM_EXTENSION)


def asan_binary_path(name):
    '''
    Given a name, return the absolute path to the binary that will be
    compiled for the asan test with that name.
    '''
    return join(SOURCE_DIR, 'bin', name)


def infer_flags(exe_wrapper):
    '''
    Given an exe wrapper argv, check whether compiling for the wrapper's
    emulated platform is likely to require additional libraries. Return
    a list of flags to be injected to a compiler's argv.
    '''
    flags = []
    if exe_wrapper and '-L' in exe_wrapper:
        last_was_dashl = False
        for arg in exe_wrapper:
            if last_was_dashl:
                flags.extend(['-L', join(arg, 'lib')])
            last_was_dashl = (
                arg[0] == '-' and arg[-1] == 'L' and not arg.startswith('--')
            )
    return flags


def infer_env(name):
    '''
    Given the name of an asan test, return a dict of environment variables
    that should be set while running that test.
    '''
    out = {'ASAN_OPTIONS': ''}
    with open(asan_source_path(name)) as source_code:
        for num, line in enumerate(source_code):
            match = re.search(r'^[\s/*]*(\w+)=(\S+)', line)
            if match:
                out[match.group(1)] = match.group(2)
            if num > 5:
                break
    return out


def list_sanitizer_tests():
    '''
    Return a list of all the available test program names
    '''
    return [
        basename(it)[: -len(TEST_PROGRAM_EXTENSION)]
        for it in glob(join(SOURCE_DIR, "*" + TEST_PROGRAM_EXTENSION))
    ]


class AsanTester:
    '''
    A utility that compiles test programs to verify various address
    sanitizer features work on your platform.
    '''

    silence = False

    @classmethod
    def from_args(cls, args):
        '''
        Use an argparse output object to construct an instance of AsanTester
        '''
        return cls(args.compiler, args.exe_wrapper)

    def __init__(self, compiler=DEFAULT_COMPILER, exe_wrapper=()):
        self.compiler = compiler
        self.exe_wrapper = list(exe_wrapper)

    def _fail(self, outcome_context, message):
        if not self.silence:
            outcome_context.fail(message)

    def compile_asan_program(self, name):
        '''
        Compile an asan tester program
        '''
        source = asan_source_path(name)
        binary = asan_binary_path(name)
        makedirs(dirname(binary), exist_ok=True)
        argv = (
            [self.compiler]
            + infer_flags(self.exe_wrapper)
            + [
                '-std=c++20',
                '-g',
                '-fsanitize=address',
                relpath(source, SOURCE_DIR),
                '-o',
                relpath(binary, SOURCE_DIR),
            ]
        )
        with outcome(argv, cwd=SOURCE_DIR) as run:
            run.check_returncode()
            if run.result.stdout:
                self._fail(run, f'Compiling `{source}` produced warnings.')
                return False
        return True

    def run_asan_program(self, name):
        '''
        Invoke an asan tester program and verify that its output matches
        the error message we expect.
        '''
        binary = asan_binary_path(name)
        argv = self.exe_wrapper + [relpath(binary, SOURCE_DIR)]
        inferences = infer_env(name)
        with outcome(
            argv,
            cwd=SOURCE_DIR,
            env=env(inferences),
            show_output_on_error=not self.silence,
        ) as run:
            if not run.result.returncode:
                return self._fail(run, f"`{binary}` did not throw error")
            esc = re.escape(inferences.get('looking_for', name))
            pat = re.compile(r'Sanitizer:(?!.*not supported)[\w -]+%s' % esc)
            if not pat.findall(run.result.stdout.decode('ascii', 'ignore')):
                self._fail(run, f"`{binary}`'s error message was not '{name}'")
                return False
        return True

    def check_sanitizer(self, name, **kw):
        '''
        Test a specific asan feature
        '''
        with ViewFrame(
            action=name,
            catch=subprocess.CalledProcessError,
            hide_inner=kw.get('hide_inner', True),
        ):
            return self.compile_asan_program(name) and self.run_asan_program(
                name
            )
        return False

    def test_asan(self, hide_inner=True):
        '''
        Run the full suite of asan tests and return true if they're all OK
        '''
        asan_ok = True
        action = f'Test address sanitizer on `{basename(self.compiler)}`'
        with ViewFrame(action=action) as frame:
            for test in list_sanitizer_tests():
                if not self.check_sanitizer(test, hide_inner=hide_inner):
                    frame.finish_status = 'fail'
                    asan_ok = False
        return asan_ok


def add_asan_tester_arguments(parser):
    '''
    Add arguments to the given parser for configuring an AsanTester instance.
    '''
    parser.add_argument(
        '--compiler',
        nargs='?',
        default=DEFAULT_COMPILER,
        help='Path to a C++ compiler to test against',
    )
    parser.add_argument(
        '--exe_wrapper',
        nargs=argparse.REMAINDER,
        default=[],
        help='Command line prefix used to run compiled tests',
    )


def main():
    '''Parse command line arguments and run an AsanTester instance'''
    parser = argparse.ArgumentParser(description=__doc__)
    add_asan_tester_arguments(parser)
    args = parser.parse_args()
    sys.exit(0 if AsanTester.from_args(args).test_asan() else 1)


if __name__ == '__main__':
    main()

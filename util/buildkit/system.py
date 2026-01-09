"""
Routines for interacting with the operating system, including tools for
querying information about the environment, and utility functions for
calling subprocesses within a ViewFrame for pretty UX.
"""

from contextlib import contextmanager
from getpass import getuser
from os import environ
from os.path import isfile, join
from pwd import getpwnam
from shlex import quote
import subprocess

from .console import ViewFrame, UI

# Keyword arguments known to be accepted by subprocess.run
SUBPROC_RUN_KW = {
    'stdin',
    'input',
    'stdout',
    'stderr',
    'capture_output',
    'shell',
    'cwd',
    'timeout',
    'check',
    'encoding',
    'errors',
    'text',
    'env',
    'universal_newlines',
}


def identify_user():
    '''
    Identify the id and username of the user who invoked this process.
    Return the actual login user, not root, when run with sudo.
    '''

    username = getuser()
    return getpwnam(username).pw_uid, username


def merge(*dicts, into=None):
    '''
    Combine multiple dicts by repeatedly calling "update"
    '''

    into = {} if into is None else into
    for source in dicts:
        into.update(source)
    return into


def env(*dicts, **kw):
    '''
    Generate a dict based on the current environment variables, but
    overridden by the keys/values supplied.
    '''
    return merge(environ, kw, *dicts)


def filter_keys(mapping, keys):
    '''
    Return a dict of mapping.items() containing only the keys given in the
    keys argument.
    '''
    return {k: v for k, v in mapping.items() if k in keys}


def argv_to_string(argv, **kw):
    '''
    Given an argv list (as one would pass to exec or popen), return a
    shell-friendly string that's been quoted and escaped so the user
    can copy and paste it if necessary.
    '''
    if isinstance(argv, str):
        return argv
    argv = list(argv)
    if not argv[0].startswith('/') and isfile(
        join(kw.get('cwd', '.'), argv[0])
    ):
        argv[0] = f'./{argv[0]}'
    return " ".join(quote(it) for it in argv)


def _proc_context(argv, kwargs):
    hide_inner = kwargs.pop('hide_inner', True)
    action = kwargs.pop('action', '+ ' + argv_to_string(argv, **kwargs))
    return ViewFrame(action=action, hide_inner=hide_inner, **kwargs)


class UnexpectedOutcomeError(subprocess.CalledProcessError):
    '''
    Thrown by the outcome context manager when the results of a called
    process do not match the expectations set by the `with outcome` block.
    '''

    def __init__(self, message, outcome_wrapper):
        # pylint: disable=super-init-not-called
        self.message = message
        self.cmd = outcome_wrapper.cmd
        self.returncode = outcome_wrapper.result.returncode
        self.stdout = outcome_wrapper.result.stdout
        self.stderr = outcome_wrapper.result.stderr

    def __str__(self):
        return self.message


class OutcomeWrapper:
    '''
    Context provided in a `with outcome` block, used for validating the
    results of a subprocess call.

    .cmd will contain the original command that was run
    .result will contain a run result (from subprocess.run), providing:
      .result.returncode - the exit status code
      .result.stdout - text the called program wrote to standard out
      .result.stderr - text the called program wrote to standard error
    '''

    def __init__(self, cmd, result):
        self.cmd = cmd
        self.result = result

    def fail(self, message):
        '''
        Indicate that the command result does not conform to your
        expectations, writing an error to the user.

        This function impacts control flow -- it exits the current ViewFrame
        throwing an exception that subclasses subprocess.CalledProcessError.
        '''
        raise UnexpectedOutcomeError(message, self)

    def check_returncode(self):
        '''
        Verify the program returned a zero exit code, and call `fail` with
        a predefined error message if not.
        '''
        if self.result.returncode:
            self.fail(
                'Got exit status %d from `%s`'
                % (self.result.returncode, argv_to_string(self.cmd))
            )


@contextmanager
def outcome(*a, **kw):
    '''
    Context manager that runs the given program in a pretty way, and lets
    the caller examine the output to detect failure conditions before
    reporting to the user that the program completed.

    Keyword arguments are forwarded to both subprocess.run and ViewFrame,
    so you can use any keyword argument supported by either of those.

    By default, the action showed to the user as the title of the ViewFrame
    is "+ " followed by the argv passed in as the first positional argument
    (mimicking the behavior of bash's `set -x`). You can override this by
    setting the `action` keyword argument.
    '''
    with _proc_context(a[0], kw):
        run_kw = merge(
            {'stderr': subprocess.STDOUT, 'stdout': subprocess.PIPE},
            filter_keys(kw, SUBPROC_RUN_KW),
        )
        run_result = subprocess.run(*a, **run_kw)
        if kw.get('show_output_on_error', True):
            for stream in [run_result.stdout, run_result.stderr]:
                if stream is not None:
                    UI.write(run_result.stdout)
        yield OutcomeWrapper(a[0], run_result)


def call(*a, **kw):
    '''
    Run a subprocess with the given argv, returning its console output.

    Can be used as a drop-in replacement for subprocess.check_output or
    subprocess.check_call. Keyword arguments are forwarded to a
    `with outcome` block.
    '''
    with outcome(*a, **kw) as wrapper:
        wrapper.check_returncode()
        out = wrapper.result.stdout
        if out is not None:
            out = out.decode('u8', 'ignore').rstrip('\n')
        return out

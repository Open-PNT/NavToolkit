from tempfile import TemporaryFile
from contextlib import contextmanager
from os import fdopen, dup, dup2
from sys import stdout, stderr
from locale import getpreferredencoding

try:
    from ctypes import cdll
    from ctypes.util import find_library

    try:
        LIBC = cdll.msvcrt
    except (OSError, AttributeError):
        LIBC = cdll.LoadLibrary(find_library('c'))
except ImportError:
    LIBC = None


def flush(*streams):
    global LIBC
    # High-level flush first -- clear out Python's buffers
    for stream in streams:
        stream.flush()
    if LIBC is not None:
        try:
            LIBC.fflush(None)  # Flush the low-level (C/C++) buffer
        except BaseException:
            LIBC = None  # If fflush fails for any reason, don't keep trying


class FileAsString:
    def __init__(self, fd):
        self.fd = fd
        self.final = None

    def bake(self):
        self.final = bytes(self)
        self.fd = None

    def _cast_like(self, other):
        if isinstance(other, str):
            return str(self)
        return bytes(self)

    def __bytes__(self):
        if self.final is not None:
            return self.final
        here = self.fd.tell()
        self.fd.seek(0)
        try:
            return self.fd.read()
        finally:
            self.fd.seek(here)

    def __str__(self):
        return bytes(self).decode(getpreferredencoding())

    def __eq__(self, other):
        return self._cast_like(other) == other

    def __repr__(self):
        return repr(bytes(self))

    def __contains__(self, other):
        return other in self._cast_like(other)

    def __len__(self):
        return len(self.final) if self.final is not None else self.fd.tell()

    def __bool__(self):
        return bool(self.final or self.fd.tell())


@contextmanager
def redirected_outputs():
    """
    Context manager that points both stdout and stderr at a temporary file
    so it may be intercepted and monitored.

    The functionality is fundamentally similar to the built-in
    contextlib.redirect_stdout, except that function doesn't work at the
    file-descriptor level, and thus does not impact spdlog (or other things
    defined in C++)

    See also https://stackoverflow.com/a/22434262
    """

    out_fd = stdout.fileno()
    err_fd = stderr.fileno()

    # First, open our temporary file, and copies of the file descriptors
    with TemporaryFile() as dest, fdopen(dup(out_fd), 'wb') as out_bak, fdopen(
        dup(err_fd), 'wb'
    ) as err_bak:
        flush(stdout, stderr)
        fas = FileAsString(dest)
        try:
            # overwrite the file descriptors with the `dest`
            dup2(dest.fileno(), out_fd)
            dup2(dest.fileno(), err_fd)
            yield fas
        finally:
            # overwrite file descriptors with the copies we made in the
            # with: block
            flush(stdout, stderr)
            dup2(out_bak.fileno(), out_fd)
            dup2(err_bak.fileno(), err_fd)
            fas.bake()

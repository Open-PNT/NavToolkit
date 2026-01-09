#!/usr/bin/env python3

import sys
from mypy.stubgen import main
from os import path

if __name__ == '__main__':
    exit_code = main()

    # Exit code is nonzero meaning that mypy reported an issue, so forward the
    # error code.
    if exit_code != 0 and exit_code is not None:
        sys.exit(exit_code)

    # Exit code is zero meaning mypy reported success, but sometimes it does
    # this erroneously so we'll check the output. See
    # https://github.com/python/mypy/issues/16762 for more information.

    # Grab the build directory from the input arguments.
    for arg in sys.argv:
        if arg.startswith('-o'):
            build_directory = arg.removeprefix('-o')

    # Verify that at least one of the expected output files exists. Generally
    # it seems to be all or nothing.
    sentinel_file = path.join(build_directory, 'navtk', '__init__.pyi')
    if path.exists(sentinel_file):
        sys.exit(0)
    else:
        print(f'Could not find {sentinel_file}')
        sys.exit(1)

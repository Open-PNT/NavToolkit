#!/usr/bin/env python3

import sys

from mkdoc import mkdoc, PREFIX_BLACKLIST, CursorKind

PREFIX_BLACKLIST.append(CursorKind.NAMESPACE)


def main():
    '''
    Forward system arguments to mkdoc, translating the return into an exit
    code.
    '''
    exit_code = 0 if mkdoc(sys.argv[1:-1], sys.argv[-1]) else 1
    sys.exit(exit_code)


if __name__ == '__main__':
    main()

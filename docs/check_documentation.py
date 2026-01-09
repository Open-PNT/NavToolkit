#!/usr/bin/env python3

from subprocess import check_output, STDOUT
from re import split
from os.path import isfile

if not isfile('docs/Doxyfile'):
    raise Exception('Cannot find the Doxygen config file, Doxyfile')

doxygen_output = check_output(
    'doxygen', stderr=STDOUT, shell=True, cwd='docs'
).decode()


def get_line_by_number(path, line):
    with open(path) as fd:
        for no, text in enumerate(fd, start=1):
            if no == line:
                return text
    return ""


warnings_detected = [
    w for w in split(r'\n(?=.*warning:)', doxygen_output) if w
]

num_warnings = len(warnings_detected)

if num_warnings > 0:
    raise Exception(
        'Found {} warnings: \n'.format(num_warnings)
        + '\n'.join(warnings_detected)
    )
else:
    print('No warnings were detected. Documentation check passed.')

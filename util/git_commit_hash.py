#!/usr/bin/env python3
"""
In the directory that this script is executed from (not the directory in which
the script is stored), run git commands and print out the commit hash of the
current commit.

If the working tree is unclean or the index has uncommitted changes, print
the string 'dirty'.

If git is not found, print the string 'unknown'.
"""

import subprocess

from os.path import abspath, dirname, exists, join


def retrieve_hash():

    # We may not be able retrieve commit hash, for example if run from an
    # unzipped source package
    if not exists(join(dirname(abspath(__file__)), "..", ".git")):
        return 'unknown'

    try:
        git_status_proc = subprocess.run(
            ['git', 'status', '--porcelain'],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            universal_newlines=True,
        )
    except OSError:
        return 'unknown'

    if git_status_proc.returncode != 0:
        return 'unknown'

    # A clean working tree would have empty stdout from git status
    if git_status_proc.stdout.strip() != '':
        return 'dirty'

    git_revparse_proc = subprocess.run(
        ['git', 'rev-parse', '--verify', 'HEAD'],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        universal_newlines=True,
    )

    return git_revparse_proc.stdout.strip()


if __name__ == '__main__':
    print(retrieve_hash())

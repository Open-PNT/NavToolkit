#!/usr/bin/env python3

r'''
The documentation generation pipeline for this project is a bit complicated.

 /─────────\  ┌───────┐  ┌───────┐  ┌──────┐    ┌──────┐
 │Developer│  │Doxygen│  │Breathe│  │Exhale│    │Sphinx│
 \────┬────/  └───┬───┘  └───┬───┘  └──┬───┘    └──┬───┘
      │ C++ Code  │          │         │           │
      │──────────>│          │         │           │
      │           │          │         │           │
      │           │   XML    │         │           │
      │           │─────────>│         │           │
      │           │          │         │           │
      │           │          │  reST   │           │
      │           │          │────────>│           │
      │           │          │         │           │
      │           │          │         │ API (reST)│
      │           │          │         │ ──────────>
      │           │          │         │           │
      │           │ Tutorials (reST)   │           │
      │───────────────────────────────────────────>│
      │           │          │         │           │
      │           │       HTML         │           │
      │<───────────────────────────────────────────│
 /────┴────\  ┌───┴───┐  ┌───┴───┐  ┌──┴───┐    ┌──┴───┐
 │Developer│  │Doxygen│  │Breathe│  │Exhale│    │Sphinx│
 \─────────/  └───────┘  └───────┘  └──────┘    └──────┘

The goal is to create documentation that can be hosted on a website,
resembling "Read the Docs" (https://docs.readthedocs.io).

Read the Docs uses the Sphinx Python module to generate documentation.
Sphinx (https://www.sphinx-doc.org) is designed to generate docs for Python.

NavToolkit is written in C++, which Sphinx does not understand. A popular
C++ documentation generator is Doxygen (https://doxygen.nl), which extracts
API documentation from C++ files and presents it in a variety of formats,
including HTML and XML.

While we could just use Doxygen HTML output, we want to use Sphinx because we
want to write our documentation in reStructuredText (.rst), and Doxygen does
not support this.  Plus "Sphinx looks nicer".

Sphinx cannot extract API docs from C++.  Doxygen can.  Sphinx does not
understand Doxygen output.  This is where we start using Sphinx extensions.

The Breathe extension (https://breathe.readthedocs.io) understands Doxygen
XML output and can rewrite it into reStructuredText for Sphinx to consume.
However, Breathe requires you to add directives to print this information
for individual classes, functions, etc. Breathe provides directives like
`.. doxygenindex::` which prints everything but is hard to navigate.

The Exhale extension (https://exhale.readthedocs.io) rebuilds the Doxygen
Class Hierarchy, File Hierarchy, and Page Hierarchy using Breathe directives
for Sphinx to consume.  Then we do not need to use any Breathe directives to
display API documentation, and can instead include `api-exhale/library_root`
to show these automatically-generated hierarchies.

The MathJax extension uses the MathJax javascript library
(https://www.mathjax.org/) to render LaTeX equations.  For more info, see:
https://www.sphinx-doc.org/en/master/usage/extensions/math.html

With MathJax, the embedded search engine, and themes, we are careful to avoid
anything which loads assets from 3rd-party domains.  This is both a good
security practice, and required for the docs to render correctly when viewed on
a local machine (`file://` protocol in the browser) as opposed to being hosted
on a server.
'''

import sys
import subprocess
import json
from glob import glob
import os
from os.path import abspath, join, dirname

HERE = dirname(abspath(__file__))


def fgrep(text, path):
    with open(path, encoding='u8') as fd:
        return any(text in line for line in fd)


def build_documentation(build_directory, display_sphinx_warnings=False):
    '''
    First generates an API by running Doxygen then runs Sphinx, which uses
    breath/exhale extensions to convert the Doxygen-generated API into a
    ReadTheDocs theme alongside the tutorial docs.
    '''
    docs_dir = join(build_directory, 'docs')
    print('Clearing any previously generated documentation files...')
    sphinx_warnings_file_name = join(
        build_directory, 'sphinx_doc_warnings.txt'
    )
    subprocess.check_call(
        [
            'rm',
            '-rf',
            docs_dir,
            'api-exhale',
            'doxygen_output',
            sphinx_warnings_file_name,
        ],
        cwd=HERE,
    )
    print('Running Doxygen...')
    projectinfo = json.loads(
        subprocess.check_output(
            [
                "meson",
                "introspect",
                "--projectinfo",
                join(dirname(HERE), 'meson.build'),
            ]
        )
    )
    __version__ = projectinfo['version']
    env = os.environ.copy()
    env["NAVTK_DOXYGEN_PROJECT_NUMBER"] = __version__
    subprocess.check_call("doxygen", cwd=HERE, env=env)
    print('Running Sphinx...')
    if display_sphinx_warnings:
        sphinx_build = subprocess.run(
            ['sphinx-build', '-j', 'auto', '.', docs_dir], cwd=HERE
        )
    else:
        with open(sphinx_warnings_file_name, 'w') as warnings_file:
            sphinx_build = subprocess.Popen(
                ('sphinx-build', '-j', 'auto', '.', docs_dir),
                cwd=HERE,
                stderr=warnings_file,
            )
            sphinx_build.wait()
    if sphinx_build.returncode:
        print(
            'Sphinx exited with code %s' % sphinx_build.returncode,
            file=sys.stderr,
        )
    likely_bad = [
        path
        for path in glob(
            join(docs_dir, 'api-exhale', '**', '*'), recursive=True
        )
        if fgrep('Unable', path)
    ]

    if sphinx_build.returncode or likely_bad:
        print(
            'WARNING: Documentation build was not successful for all',
            'documented functions.',
            file=sys.stderr,
        )
        print(
            '    Check that your Sphinx, breathe, and exhale versions match',
            'those listed in docker/requirements.txt.',
            file=sys.stderr,
        )
        print(
            '    For more information,',
            'see {}'.format(sphinx_warnings_file_name),
            '(or the warnings above if show_sphinx_warnings=true in meson',
            'configuration).',
            file=sys.stderr,
        )
        if likely_bad:
            print(
                'The following files likely contain errors:', file=sys.stderr
            )
            for path in likely_bad:
                print('    ' + path, file=sys.stderr)
        sys.exit(sphinx_build.returncode or 1)


def main():
    '''
    Parses system arguments and passes them to helper functions which set up
    the documentation dependencies and build the documentation.
    '''
    arguments = sys.argv[1:]
    if len(arguments) < 2:
        print(
            "Warning: build_documentation.py needs two arguments. Unable ",
            "to build the documentation.",
        )
    build_documentation(arguments[1], '--show-warnings' in arguments)


if __name__ == '__main__':
    main()

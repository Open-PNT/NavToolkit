# NavToolkit

## Merge Request Process

Please see the [Development Process](https://git.aspn.us/pntos/pntos/-/wikis/Developing/Our-Development-Process)
page on the wiki for more information about our review and merge request process. Other pages on
that wiki also provide guidance on developing in NavToolkit.

## Style Guide

Please see the [C++ Style Guide](https://git.aspn.us/pntos/pntos-style-guides/-/wikis/home)
page for more information about C++ style rules.

## Some Helpful Tools

### Ubuntu 24.04

You may also want to install the following additional tools to take full advantage of the techniques
we describe in this file.

```shell
sudo apt-get install docker.io clang-tools
```

Note that by default on Ubuntu you will need to follow Docker's [Post-installation steps for
Linux](https://docs.docker.com/engine/install/linux-postinstall/).

### macOS Sonoma v14.4

While clang-tools should be included from the installation instructions in README.md for most
operating systems, you may want Docker on your machine. On macOS, you will need to install Docker
from its [website](https://www.docker.com) by downloading and running the installer. To ensure
there is enough memory to run the commands of this ReadMe, you will want to increase the amount
of memory allocated. Adjust the memory slider from Docker->Preferences->Resources (we recommend
20 GB).

## Formatting

To contribute changes to NavToolkit, your code should pass a `clang-format` check. In order to
ensure consistency and because different major versions of `clang-format` sometimes produce
different results, we limit the versions of `clang-format` which can be used to format the code (see
the meson output to see if your version is acceptable). If you do not have one of the whitelisted
versions, then formatting should be performed using docker. See the "Building and formatting with
the Docker container" section for instructions on how to use our docker container to format your
code.

Python code contributed to NavToolkit should conform to:

- PEP8, which can be verified by `ninja -C build flake8` if `flake8` has been installed or via
  `docker/docker_interface.py flake8` otherwise.
- `black`, which can be called via the `format_py` target in meson on bare metal or the `format`
  command in docker via `docker_interface.py`.

## Contributing to Generated Documentation

If making changes to the docs, you can enable a faster dev cycle by only regenerating the
non-API docs incrementally via

```shell
env DOC_PREVIEW=1 ninja -C build docs
```

To check the warnings generated while building documentation, either build the docs and then
inspect `build/sphinx_doc_warnings.txt`, or run `meson setup build` with the flag
`-Dshow_sphinx_warnings=true` before building docs (to print warnings to the terminal).

When making changes to functionality exposed via the Python bindings, ensure that you see a message
containing `Python docstrings included: YES` when running `meson setup build`. If you instead see
`Python docstrings included: NO`, then you'll need to resolve the failure using the error message
that follows. Alternately, the python bindings will build if following the instructions in
"Building and formatting with the Docker Container".

## Building and formatting with the Docker container

You may want to build the codebase using docker to troubleshoot your environment setup instead or
to avoid modifying your build environment. It is also necessary to format using the docker
container to ensure cross-platform consistency.

In order to build the container and setup the build directory using meson, run:

```shell
docker/docker_interface.py setup
```

This script assumes that Python3 is installed.

After the docker container has been built run:

```shell
docker/docker_interface.py build
```

to build NavToolkit or run:

```shell
docker/docker_interface.py format
```

to format the source code.

Of course, it is possible to build and run the containers directly. The docker containers assume the
build context is `docker/` rather than the project root directory. All commands will be run inside
the container as an account with the user-name/group-name `docker`, so user directories like SSH
keys or ccache cache should be mapped to `/home/docker`.

## Running code sanitizers outside of Docker

The docker container runs with address sanitizers enabled, which is not true of a normal build.
It is possible to enable address sanitizers by passing the appropriate options to meson during
build folder configuration:

```shell
meson setup build -Db_sanitize=address,undefined
```

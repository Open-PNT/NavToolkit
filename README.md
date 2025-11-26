# NavToolkit

## Installing Python Dependencies

Some platforms (for example, Python3.12 on macOS or Ubuntu 23.04 and later) require the user to
install Python packages into a virtual environment, prohibiting users from installing Python packages
system-wide. If you fall into this category or if you want to use a virtual Python environment,
please proceed to [Setting up a Virtual Python
Environment](#setting-up-a-virtual-python-environment). Otherwise, proceed to [Installing Python
Dependencies Outside of a Virtual Python
Environment](#installing-python-dependencies-outside-of-a-virtual-python-environment).

### Setting up a Virtual Python Environment

**Warning:** macOS users should install Python via homebrew (see [macOS Sonoma
v14.4](#macos-sonoma-v144) for information on how to do so) before going through this section.
Otherwise, the virtual environment will be tied to the Apple Python rather than the homebrew Python.

To set up a virtual environment that lives in this project directory, run:

```shell
python3 -m venv .venv --system-site-packages
```

**Warning:** sometimes the above command requires additional dependencies to run. For example, some
platforms require the `python3-venv` package to execute the above.

To start using the virtual environment in a given shell, run:

```shell
source .venv/bin/activate
```

**Warning:** the above command is valid for shells like `bash` or `zsh`. To use it, for example,
with `fish` you would run `source .venv/bin/activate.fish` instead.

**Note:** if you want to setup the virtual environment as a global environment that is always loaded
for every shell instance, create the virtual environment in a persistent location (like somewhere
inside your home directory, for example). Then add a command to your shell's setup script (e.g.
`~/.bashrc` on bash).

You should be ready to install dependencies. Please proceed to [Installing
Dependencies](#installing-dependencies).

### Installing Python Dependencies Outside of a Virtual Python Environment

The `pip` commands in this file assume they are being installed to a virtual Python environment and
do not contain a `--user` option. It is recommended that you modify any `pip install` command you
run from this file to contain a `--user` option before executing it. Some platforms require it and
on others it is considered good practice.

By default the programs installed by `pip3` are put into `~/.local/bin`, so you'll want to add that location to your path
depending on your shell configuration.  If `meson setup build` reports that the project requires a
newer version, or that meson is not installed, run this command to adjust your path:

```shell
export PATH=~/.local/bin:$PATH
```


## Installing Dependencies

The following subsections detail how to configure various supported environments for use with
NavToolkit. Use the section that most closely resembles your environment, then move to [Building
with meson](#building).

**Note:** many commands assume they are being run from the project's root directory.

### Ubuntu 22.04 and Ubuntu 24.04

The following commands should do all the setup necessary to build the C++ library, Python extension
module, and documentation as well as run the examples on Ubuntu:

```shell
sudo apt-get update
sudo apt-get install ninja-build libopenblas-dev python3-numpy python3-matplotlib python3-clang clang doxygen texlive-base texlive-font-utils python3-pip git pkg-config
pip3 install -r <(grep -vf <(pip3 freeze | cut -d= -f1) docker/requirements.txt)
```

The following dependencies are not required but some targets will be disabled if they are not
installed.

```shell
sudo apt install libgdal-dev liblcm-dev
pip3 install lcm==1.4.4
```

### macOS Sonoma v14.4

First, install Xcode from the App Store, sign the license agreement (should be prompted by opening
Xcode or by command-line `xcode-select --install`), and install [Homebrew](https://brew.sh/) (other
package managers may work, but are not supported).

Before proceeding, ensure you have properly set up a Python virtual environment as described in the
[Setting up a Virtual Python Environment](#setting-up-a-virtual-python-environment) section above.

The following commands should do all the preliminary setup on macOS for the C++ library,
Python extension module, documentation and examples:

```shell
brew install python ninja llvm pkg-config doxygen gcovr glib mactex numpy cmake python-matplotlib
pip3 install -r <(grep -vf <(pip3 freeze | cut -d= -f1) docker/requirements.txt)
pip3 install clang
```

The following dependency is not required but some targets will be disabled if it is not installed.

```shell
brew install gdal
```

**Note:** The PATH variable should be adjusted such that the Python installed by Homebrew is found
before the standard `/usr/bin/python`.  If it is not done automatically, it will need to be done
manually.

### Fedora 43

The following commands do most of the setup necessary to build the C++ library, Python extension
module, and documentation as well as run the examples:

```shell
sudo dnf install -y \
    ccache \
    clang \
    cmake3 \
    @development-tools \
    doxygen \
    flexiblas-devel \
    flexiblas-openblas-serial \
    gdal-devel \
    glib2-devel \
    libasan    \
    libubsan \
    ninja-build \
    python3-devel \
    python3-matplotlib \
    python3-numpy \
    python3-pip \
    texlive
flexiblas default openblas-serial
# Install pip packages, excluding any that were already installed by dnf/yum. This command should be
# run from the navtk project root directory.
pip3 install -r <(grep -vf <(pip3 freeze | cut -d= -f1) docker/requirements.txt)
```

## Building

Some of the subprojects are authenticated git repositories which meson attempts to clone via SSH. If
you have an SSH key set up on [your ASPN GitLab account](https://git.aspn.us/-/profile/keys) and
access to the PNTOS project, then it is possible for the meson instructions to succeed as-is. If you
do not have access to the PNTOS project on ASPN Gitlab but you do have SSH access to one of the
mirrors then you can modify all of the `.wrap` files in the `subprojects/` directory that have
`[wrap-git]` at the top, modifying the `url =` line to point to the repository that you do have
access to. If you do not have access to any of the mirrors, then you can build by copying the
subprojects from another source into the `subprojects/` directory.

To build, you'll first need to configure the build system:

```shell
meson setup build
```

If successful, you should have a configured build system and be ready to build NavToolkit:

```shell
ninja -C build
```

If this command is not successful, please see the [troubleshooting section](#troubleshooting) for more help.

This will build the NavToolkit shared libraries, import libraries, python extension module,
and tests. To run the tests:

```shell
ninja -C build test
```

**Note:** This project is known to build on Linux and macOS with a modern C++ compiler.
While it may work on other platforms, we do not currently support them.

## Running the Examples from Meson/ninja

The Straight Flight example is a simulated scenario of an aircraft flying North. A reference
solution and measurement updates are simulated by adding white noise to a simple truth trajectory.

Compile and run examples using ninja:

```shell
ninja -C build run_straight_flight_example
ninja -C build run_aliased_example
ninja -C build run_bias_example_with_update
ninja -C build run_gps_ins_tightly_coupled_ground_example
ninja -C build run_gps_ins_loosely_coupled_flight_example
```

If the python bindings were built, then you can also run the Python examples using ninja:

```shell
ninja -C build run_straight_flight_example_py
ninja -C build run_virtual_state_block_example_py
ninja -C build run_scalar_fogm_example_py
ninja -C build run_particle_filter_example_py
ninja -C build run_bias_example_with_update_py
ninja -C build run_gps_ins_loosely_coupled_flight_example_py # Only if LCM was installed
```

Depending on your environment, these examples may not run with the default Matplotlib backend.
Using the tkinter backend appears to resolve these issues.  To use it, make sure the tkinter package
is installed, and then set the `MPLBACKEND` environment variable to `TkAgg`.  For example,

```shell
MPLBACKEND=TkAgg ninja -C build run_straight_flight_example_py
```

## Using the Python module outside of Meson/ninja

The meson run targets set `PYTHONPATH` to the location of the non-installed NavToolkit and ASPN-C++
Python modules. When running outside of meson and ninja, it is possible to also use the
non-installed modules by setting `PYTHONPATH` or using `sys.path.append` but these approaches can be
brittle and are not recommended. Instead, please see `README_PYTHON.md` for instructions on how to
install the Python module.

## Generating Documentation

Once all of the dependencies have been installed run:

```shell
ninja -C build docs
```

The docs will be available at `build/docs/index.html`.

## Troubleshooting

### Crashes during Building

Compiling this project can be resource-intensive.  We recommend having 2.5GB of RAM available per
CPU thread compiling this project.  If your system is freezing or crashing while building, you can
reduce the number of CPU threads used by telling ninja how many jobs to run in parallel, for
example `ninja -C build -j8` will limit ninja to 8 simultaneous build tasks.  Linking in particular
can take significant memory.  To lower just the number of simultaneous linker operations, update
your existing configuration in the subdirectory `build` with
`meson configure build -Dbackend_max_links=2` to only link two items in parallel.

### Clean Builds

Often times, it is beneficial to have an entirely "clean build".

Some common cases where it is necessary to build from scratch are:

- You're getting an error about a subproject directory having no `meson.build`
- You've switched branches or otherwise changed the current commit and now you're getting
  subproject-related errors when configuring a build directory or compiling
- You've encountered any other error which might indicate a version mismatch
- You want to be 100% sure the commit you're building against is pulling the correct sources

The steps below (or any functional equivalent) should work to delete your build directory and reset the
subprojects directory.

**Warning**: some of the commands below will remove files that aren't checked into git and should be run
with caution and only if the user understands what the commands are intended to do.

**Note**: these commands assume they are being run from the root navtk project directory.

To reset the `subprojects/` directory:

```shell
# First, do a dry run to verify this only removes files you want removed
git clean -xdffn subprojects
# This command actually performs the operation
git clean -xdff subprojects
```


To start over with a fresh build directory called `build/`, simply remove the old one:

```
rm -rf build/
```

and run `meson` again.

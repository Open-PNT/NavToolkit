# NavToolkit

This document contains guidance on how to install the NavToolkit Python module.

**Note:** Some operating systems require or encourage the use of a Python virtual environment. For
more information, please see `README.md`.

## Installing the NavToolkit Python module

### Option 1: installing a prebuilt wheel

Installing from [PyPi](https://pypi.org/project/navtk/) can be the most convenient way to install
NavToolkit's Python module because it skips having to install build dependencies or build source
code. On the other hand, the wheels only cover the most common platforms and versions of Python. So
if your environment is not supported then you will need to build from source.

Installing the NavToolkit Python module can be as simple as:

```shell
pip3 install  navtk
```

If this approach does not work for you, then you may want to consider building from source by
following the instructions in the next section.

### Option 2: building from source

To build and install the NavToolkit Python module from source, run:

```shell
pip3 install -v .
```

## Testing the Python module

To verify that the Python module has been correctly built and installed, you could try running one
of the examples:

```shell
examples/straight_flight_example.py
```

Or, you could try creating a NavToolkit object in a Python shell:

```shell
python3
>>> from navtk.filtering import StandardFusionEngine
>>> s = StandardFusionEngine()
>>> s.get_time()
0.000000000s
>>> quit()
```

## Uninstalling

To uninstall the NavToolkit Python module, run:

```shell
pip3 uninstall navtk
```

## Other notes

The above instructions will actually install two Python modules: one for NavToolkit (`navtk`) and
one for its dependency ASPN-C++ (`aspn23_xtensor`). Both can be uninstalled with `pip3 uninstall
navtk`. For more information, please see
[firehose#41](https://git.aspn.us/pntos/firehose/-/issues/41).

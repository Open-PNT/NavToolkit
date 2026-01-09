# NavToolkit

This document contains guidance on how to install the NavToolkit Python module. 

**Note:** Some operating systems require or encourage the use of a Python virtual environment. For
more information, please see `README.md`.

## Installing the NavToolkit Python module

### Option 1: installing a prebuilt wheel

The [wheelhouse package registry](https://git.aspn.us/pntos/wheelhouse/-/packages) contains python
packages in the form of wheels for various projects, including NavToolkit. This can be the most
convenient way to install NavToolkit's Python module because it skips having to install build
dependencies or build source code. On the other hand, the wheels only cover the most common
platforms and versions of Python. So if your environment is not supported then you will need to
build from source.

In order to enable pip to install packages from this package registry, you will need a Personal
Access Token (PAT) with `read_api` privileges. You can create one from [your account page on
git.aspn.us](https://git.aspn.us/-/user_settings/personal_access_tokens).

Once you have created a token, use it to set an environment variable, replacing  `<TOKEN_VALUE>`
with the value of your token:

```shell
export WHEELHOUSE_URL=https://:<TOKEN_VALUE>@git.aspn.us/api/v4/projects/94/packages/pypi/simple
```

Now installing the NavToolkit Python module can be as simple as:

```shell
pip3 install --index-url $WHEELHOUSE_URL navtk
```

For alternatives to including a PAT in a `--index-url` argument, consider using
[keyring](https://pip.pypa.io/en/stable/topics/authentication/#using-keyring-s-python-module) or
[git credential helper](https://git-scm.com/doc/credential-helpers).

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

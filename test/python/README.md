# Python Unit Tests

## Running The Python Unit Tests

All Python unit tests stored in this folder are executed with the test target in ninja.
The following assume meson was configured to build NavToolkit in a folder named `build`.

To run all unit tests in from the root folder:

```
ninja -C build test
```

To run an individual test outside of ninja, see the root `README.md` and follow the instructions
under the section "Running the Examples", substituting the name of the Python test file for the
example file.

## Adding Unit Tests

Only files ending with "`_test.py`" will be discovered by meson.

Tests should include both the `#!/usr/bin/env python3` shebang and the following:

```python
if __name__ == '__main__':
    unittest.main()
```

After running `ninja test`, review `meson-logs/testlog.txt` to confirm tests were discovered and run.

For more information on Python unit tests, see the
[official documentation](https://docs.python.org/3/library/unittest.html).

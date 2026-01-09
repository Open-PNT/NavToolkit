#!/usr/bin/env python3

import unittest
import os
from os.path import join, basename, expanduser
import tempfile

from navtk import ErrorMode
from navtk.utils import open_data_file


TEMP_DIR = tempfile.mkdtemp()
HOME_DIR = expanduser('~')


class UtilsTests(unittest.TestCase):
    def test_open_data_file_with_env_path(self):
        sample_file = join(TEMP_DIR, "heigoland.txt")
        with open(sample_file, 'w') as fd:
            fd.write("expected text")
        os.environ['NAVTK_HEIGOLAND_PATH'] = sample_file
        with open_data_file("heigoland", "doesnt matter.txt") as fd:
            self.assertEqual("expected text", fd.read())

    def test_open_data_file_with_home_relative_path(self):
        (fd, sample_file) = tempfile.mkstemp(dir=HOME_DIR)
        try:
            try:
                os.write(fd, "expected text".encode('u8'))
            finally:
                os.close(fd)
            os.environ['NAVTK_HOME_BASED_PATH'] = join(
                "$HOME", basename(sample_file)
            )
            with open_data_file("home-based", "whatever") as fd:
                self.assertEqual("expected text", fd.read())
        finally:
            os.unlink(sample_file)

    def test_open_data_file_that_doesnt_exist(self):
        self.assertIsNone(
            open_data_file(
                ErrorMode.OFF,
                "improbably long human-readable label",
                "improbably long filename.txt",
            )
        )


if __name__ == '__main__':
    unittest.main()

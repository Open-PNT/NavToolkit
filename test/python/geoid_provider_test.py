#!/usr/bin/env python3

import unittest

from navtk.navutils import hae_to_msl


class UtilsTests(unittest.TestCase):
    def test_geoid_provider(self):
        hae_to_msl(100, 0.69, -1.84)


if __name__ == '__main__':
    unittest.main()

import unittest
from unittest.mock import MagicMock

from moon.main import min_dist


class MoonMainTest(unittest.TestCase):

    def test_min_dist(self):
        self.assertAlmostEqual(min_dist([1000, 2000, 3000]), 1.0)
        self.assertAlmostEqual(min_dist([]), 0.0)
        self.assertAlmostEqual(min_dist([0]), 10.0)

# vim: expandtab sw=4 ts=4


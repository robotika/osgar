import unittest
from unittest.mock import MagicMock

from osgar.platforms.tello import TelloDrone


class TelloDroneTest(unittest.TestCase):

    def test_usage(self):
        r = TelloDrone({}, MagicMock())

# vim: expandtab sw=4 ts=4

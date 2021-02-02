import unittest
import datetime
from unittest.mock import MagicMock

from osgar.drivers.tello import TelloDrone


class TelloDroneTest(unittest.TestCase):

    def test_usage(self):
        r = TelloDrone({}, MagicMock())

# vim: expandtab sw=4 ts=4

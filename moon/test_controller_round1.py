import unittest
from unittest.mock import MagicMock
from datetime import timedelta

from osgar.bus import Bus

from moon.controller_round1 import SpaceRoboticsChallengeRound1


class SpaceRoboticsChallengeRound1Test(unittest.TestCase):
        
    def test_usage(self):
        config = {}
        bus = Bus(MagicMock(write=MagicMock(return_value=timedelta())))
        r1 = SpaceRoboticsChallengeRound1(config, bus=bus.handle('app'))
        r1.start()
        r1.request_stop()
        r1.join()

# vim: expandtab sw=4 ts=4
 

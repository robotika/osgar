import unittest
from unittest.mock import MagicMock
from datetime import timedelta
import numpy as np
from osgar.bus import Bus

from moon.controller_excavator_round2 import SpaceRoboticsChallengeExcavatorRound2

from osgar.bus import BusShutdownException

class SpaceRoboticsChallengeExcavatorRound2Test(unittest.TestCase):

    def test_usage(self):
        config = {}
        bus = Bus(MagicMock(write=MagicMock(return_value=timedelta())))
        r1 = SpaceRoboticsChallengeExcavatorRound2(config, bus=bus.handle('app'))
        r1.start()
        r1.request_stop()
        r1.join()


    def test_find_next_volatile(self):
        # from seed 1500
        config = {'debug': False}
        bus = Bus(MagicMock(write=MagicMock(return_value=timedelta())))
        r2e = SpaceRoboticsChallengeExcavatorRound2(config, bus=bus.handle('app'))

        r2e.xyz = [10.394101, -8.620875, 0.786391]
        r2e.yaw = 2.767378

        r2e.vol_list = [[49.6034306594, -45.9196365544], [-40.7186943541, 25.742583001], [52.5927389894, 31.5772572951], [-35.4015899876, 30.9052755259], [28.626252316, -36.5104539519], [50.846455282, 10.1442217972], [49.3141912004, -41.6719396868], [-33.8047498291, -45.7775507432], [-32.6856318871, 30.7811787209], [41.5629764612, 43.1950851664], [-6.56325228377, 46.9468649025], [-5.62467497268, -4.92013629455], [14.3408662367, 17.4889646202], [54.657022764, 54.5170630599], [15.4820006992, 1.24266919623], [1.23307211219, 52.6044006624], [56.128567115, -6.59073868759], [-1.08706993599, 51.8179897589], [-39.812812773, -32.6237207556], [-39.8509161911, -54.5155864259], [15.8911609315, -3.84581195752], [51.9294757594, 14.3942159102], [-40.8507536421, -45.2608876114], [-27.542088652, 57.0], [51.8354318585, 34.5281584002], [57.2500220512, 45.3890817188], [-37.3400408782, 47.8302730305], [-14.2076440605, -29.3234846144], [-56.6504633815, 46.898890127]]

        v = r2e.get_next_volatile()
        np.testing.assert_array_equal([-5.62467497268, -4.92013629455], v)
# vim: expandtab sw=4 ts=4

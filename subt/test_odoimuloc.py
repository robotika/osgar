
import unittest
import math

from unittest.mock import MagicMock
from datetime import timedelta

import numpy as np

from osgar.bus import Bus
from osgar.lib import quaternion
from osgar.lib.unittest import TestCase

from subt.odoimuloc import Localization


class LocalizationTest(TestCase):

    def test_odom(self):
        origin = np.asarray([10.0, 0.0, 0.0])
        inc = np.asarray([100, 0.0, 0.0])
        bus = Bus(MagicMock(write=MagicMock(return_value=timedelta())))
        loc = Localization({}, bus.handle('loc'))
        loc.xyz = origin
        tester = bus.handle('tester')
        tester.register('orientation', 'odom')
        bus.connect('loc.pose3d', 'tester.pose3d')
        bus.connect('tester.orientation', 'loc.orientation')
        bus.connect('tester.odom', 'loc.odom')
        loc.start()
        tester.publish('orientation', quaternion.identity())
        tester.publish('odom', 0*inc)
        tester.publish('odom', 1*inc)
        tester.publish('odom', 2*inc)
        dt, channel, pose3d = tester.listen()
        self.assertEqual(channel, 'pose3d')
        self.assertPose3dEqual(pose3d, [list(origin), quaternion.identity()])
        dt, channel, pose3d = tester.listen()
        self.assertPose3dEqual(pose3d, [list(origin+inc/1000), quaternion.identity()])
        dt, channel, pose3d = tester.listen()
        self.assertPose3dEqual(pose3d, [list(origin+2*inc/1000), quaternion.identity()])
        loc.request_stop()
        loc.join()

    def test_orientation(self):
        origin = np.asarray([10.0, 0.0, 0.0])
        inc = np.asarray([100, 0.0, 0.0])
        inc_ori = np.asarray([0.0, 100, 0.0])
        orientation = quaternion.from_axis_angle([0, 0, 1], math.radians(90))
        bus = Bus(MagicMock(write=MagicMock(return_value=timedelta())))
        loc = Localization({}, bus.handle('loc'))
        loc.xyz = origin
        tester = bus.handle('tester')
        tester.register('orientation', 'odom')
        bus.connect('loc.pose3d', 'tester.pose3d')
        bus.connect('tester.orientation', 'loc.orientation')
        bus.connect('tester.odom', 'loc.odom')
        loc.start()
#        tester.publish('origin', ['name'] + list(origin) + quaternion.identity())
        tester.publish('orientation', orientation)
        tester.publish('odom', 0*inc)
        tester.publish('odom', 1*inc)
        tester.publish('odom', 2*inc)
        dt, channel, pose3d = tester.listen()
        self.assertEqual(channel, 'pose3d')
        self.assertPose3dEqual(pose3d, [list(origin), orientation])
        dt, channel, pose3d = tester.listen()
        self.assertPose3dEqual(pose3d, [list(origin+inc_ori/1000), orientation])
        dt, channel, pose3d = tester.listen()
        self.assertPose3dEqual(pose3d, [list(origin+2*inc_ori/1000), orientation])
        loc.request_stop()
        loc.join()


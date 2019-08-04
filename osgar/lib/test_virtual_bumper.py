import unittest
import math
from datetime import timedelta

from osgar.lib.virtual_bumper import VirtualBumper


class VirtualBumperTest(unittest.TestCase):    
    def test_usage(self):
        vb = VirtualBumper(timedelta(seconds=3))
        t0 = timedelta(seconds=12)
        t_step = timedelta(seconds=1)
        pose = [1.0, 2.0, math.pi]
        vb.update_pose(t0, pose)
        vb.update_desired_speed(0.5, 0.0)
        self.assertFalse(vb.collision())
        vb.update_pose(t0 + t_step, pose)
        vb.update_pose(t0 + 2*t_step, pose)
        vb.update_pose(t0 + 3*t_step, pose)
        self.assertTrue(vb.collision())

        vb.update_desired_speed(0.0, 0.0)
        self.assertFalse(vb.collision())

    def test_motion(self):
        vb = VirtualBumper(timedelta(seconds=2))
        t0 = timedelta(seconds=0)
        t_step = timedelta(seconds=1)
        pose = [0.0, 0.0, 0.0]
        vb.update_pose(t0, pose)
        vb.update_desired_speed(0.5, 0.0)
        self.assertFalse(vb.collision())

        pose = [0.1, 0.0, 0.0]
        vb.update_pose(t0 + t_step, pose)
        pose = [0.2, 0.0, 0.0]
        vb.update_pose(t0 + 2*t_step, pose)
        self.assertFalse(vb.collision())

    def test_real_data(self):
        # taken from Eduro
        poses = [
            (7.213, -1.344, -1.55439023182615),
            (7.213, -1.344, -1.5563100940033436),
            (7.213, -1.344, -1.556833692778942),
            (7.213, -1.344, -1.555611962302546),
            (7.213, -1.344, -1.556833692778942),
            (7.213, -1.344, -1.556833692778942),
            (7.213, -1.344, -1.55439023182615),
            (7.213, -1.344, -1.5538666330505517),
            (7.213, -1.344, -1.5580554232553379),
            (7.213, -1.344, -1.55439023182615),
            (7.213, -1.344, -1.556833692778942),
            (7.213, -1.344, -1.556833692778942)]
        vb = VirtualBumper(timedelta(seconds=2))
        t0 = timedelta(seconds=0)
        t_step = timedelta(seconds=0.5)
        vb.update_desired_speed(0.5, 0.0)
        for i, pose in enumerate(poses):
            vb.update_pose(t0 + i*t_step, pose)
        self.assertTrue(vb.collision())

# vim: expandtab sw=4 ts=4

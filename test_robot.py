import unittest

from robot import Robot


class RobotTest(unittest.TestCase):

    def test_usage(self):
        empty_config = {'stream_id':2, 'drivers': []}
        robot = Robot(config=empty_config, logger=None)
        robot.start()
        robot.update()
        robot.finish()


# vim: expandtab sw=4 ts=4

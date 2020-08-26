import unittest
from unittest.mock import MagicMock
import math

from moon.vehicles.rover import Rover, compute_steering_angle


JOINT_NAME = [b'bl_arm_joint', b'bl_steering_arm_joint', b'bl_wheel_joint', 
        b'br_arm_joint', b'br_steering_arm_joint', b'br_wheel_joint', b'fl_arm_joint',
        b'fl_steering_arm_joint', b'fl_wheel_joint', b'fr_arm_joint', b'fr_steering_arm_joint',
        b'fr_wheel_joint', b'sensor_joint']


class RoverTest(unittest.TestCase):

    def test_rover_pid(self):
        rover = Rover(config={}, bus=MagicMock())

        rover.joint_name = JOINT_NAME  # in reality received as the first joint status
        rover.on_joint_effort([0]*len(JOINT_NAME))

    def test_get_steering_and_effort(self):
        rover = Rover(config={}, bus=MagicMock())
        self.assertEqual(rover.get_steering_and_effort(), ([0.0, 0.0, 0.0, 0.0], [0, 0, 0, 0]))

        rover.drive_speed = 500.0  # sigh, corresponds to 0.5m/s now
        self.assertEqual(rover.get_steering_and_effort(), ([0.0, 0.0, 0.0, 0.0], [40, 40, 40, 40]))

        rover.drive_speed = -500.0
        self.assertEqual(rover.get_steering_and_effort(), ([0.0, 0.0, 0.0, 0.0], [-40, -40, -40, -40]))

    def test_compute_steering_angle(self):
        self.assertEqual(compute_steering_angle((1, -1), 2), math.radians(45))

        # triangle 3, 4, 5
        self.assertAlmostEqual(compute_steering_angle((3, 0), 4), math.acos(4/5))


# vim: expandtab sw=4 ts=4


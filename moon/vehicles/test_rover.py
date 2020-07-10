import unittest
from unittest.mock import MagicMock

from moon.vehicles.rover import Rover


JOINT_NAME = [b'bl_arm_joint', b'bl_steering_arm_joint', b'bl_wheel_joint', 
        b'br_arm_joint', b'br_steering_arm_joint', b'br_wheel_joint', b'fl_arm_joint',
        b'fl_steering_arm_joint', b'fl_wheel_joint', b'fr_arm_joint', b'fr_steering_arm_joint',
        b'fr_wheel_joint', b'sensor_joint']

class RoverTest(unittest.TestCase):

    def test_rover_pid(self):
        rover = Rover(config={}, bus=MagicMock())

        rover.joint_name = JOINT_NAME  # in reality received as the first joint status
        rover.on_joint_effort([0]*len(JOINT_NAME))

# vim: expandtab sw=4 ts=4


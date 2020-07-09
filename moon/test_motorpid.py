import unittest

from moon.motorpid import MotorPID


class MotorPIDTest(unittest.TestCase):

    def test_pid(self):
        pid = MotorPID(p=1, i=0, d=0)
        pid.set_desired_speed(0.5)
        effort = pid.update(current_speed=0)
        self.assertEqual(effort, 0.5)


# vim: expandtab sw=4 ts=4


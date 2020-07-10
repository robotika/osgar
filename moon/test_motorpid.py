import unittest

from moon.motorpid import MotorPID


class MotorPIDTest(unittest.TestCase):

    def test_pid(self):
        pid = MotorPID(p=1, i=0, d=0)
        pid.set_desired_speed(0.5)
        pid.update(current_speed=0)
        effort = pid.get_effort()
        self.assertEqual(effort, 0.5)


# vim: expandtab sw=4 ts=4


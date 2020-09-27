import unittest

from subt.name_decoder import parse_robot_name


class NameDecoderTest(unittest.TestCase):
    def test_parse_robot_name(self):
        steps = parse_robot_name('A10F100L')
        self.assertEqual(steps, [('wait', 10), ('left', 100), ('home', 200)])

# vim: expandtab sw=4 ts=4


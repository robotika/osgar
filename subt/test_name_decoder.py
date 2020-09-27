import unittest

from subt.name_decoder import parse_robot_name, split_multi


class NameDecoderTest(unittest.TestCase):
    def test_parse_robot_name(self):
        steps = parse_robot_name('A10F100L')
        self.assertEqual(steps, [('wait', 10), ('left', 100), ('home', 200)])

    def test_both_sides(self):
        steps = parse_robot_name('A10F100LF200R')
        self.assertEqual(steps, [('wait', 10), ('left', 100), ('home', 200), ('right', 200), ('home', 400)])

    def test_variable_side(self):
        steps = parse_robot_name('A10F100L200R')
        self.assertEqual(steps, [('wait', 10), ('left', 100), ('right', 200), ('home', 600)])

    def test_split_multi(self):
        self.assertEqual(split_multi('123L456R78', ['L', 'R']), ['123L', '456R', '78'])

# vim: expandtab sw=4 ts=4


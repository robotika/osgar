import unittest

from subt.name_decoder import parse_robot_name, split_multi


class NameDecoderTest(unittest.TestCase):
    def test_parse_robot_name(self):
        steps = parse_robot_name('A10W100L')
        self.assertEqual(steps, [('wait', 10), ('enter-left', None), ('left', 100), ('home', 200)])

    def test_both_sides(self):
        steps = parse_robot_name('A10W100LH200R')
        self.assertEqual(steps, [('wait', 10), ('enter-left', None), ('left', 100), ('home', 200), ('enter-right', None), ('right', 200), ('home', 400)])

    def test_variable_side(self):
        steps = parse_robot_name('A10W100L200R')
        self.assertEqual(steps, [('wait', 10), ('enter-left', None), ('left', 100), ('right', 200), ('home', 600)])

    def test_no_wait(self):
        steps = parse_robot_name('A100L')
        self.assertEqual(steps, [('enter-left', None), ('left', 100), ('home', 200)])

    def test_split_multi(self):
        self.assertEqual(split_multi('123L456R78', ['L', 'R']), ['123L', '456R', '78'])

    def test_old_code(self):
        with self.assertRaises(ValueError):
            parse_robot_name('A10F100L')

    def test_double_home(self):
        steps = parse_robot_name('A100LH')
        self.assertEqual(steps, [('enter-left', None), ('left', 100), ('home', 200)])

    def test_home_side(self):
        # it turned out that "enter" is ambiguous as there is offset navigating the robot
        # closer to the wall needed for the next left/right/center command.
        # In order to simplify processing the type has to be integrated here.
        steps = parse_robot_name('A100L')
        self.assertEqual(steps, [('enter-left', None), ('left', 100), ('home', 200)])

        steps = parse_robot_name('A100C')
        self.assertEqual(steps, [('enter-center', None), ('center', 100), ('home', 200)])

# vim: expandtab sw=4 ts=4


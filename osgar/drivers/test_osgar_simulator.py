import unittest
from osgar.drivers.osgar_simulator import *
from unittest.mock import MagicMock

class TestOsgarSimulator(unittest.TestCase):
    def test_current_robot_arr(self):
        bus = MagicMock()
        s = OsgarSimulator(bus=bus, config={})
        s.draw_map()


if __name__ == '__main__':
    unittest.main()

import unittest
from localization import SimpleOdometry

class LocalizationTest(unittest.TestCase):

    def test_from_dict(self):
        loc = SimpleOdometry.from_dict({"pose":[1, 2, 3]})
        self.assertEqual(loc.pose(), (1, 2, 3))
        self.assertEqual(loc.global_map, [])

        loc = SimpleOdometry.from_dict({"cones":[[2, 3]]})
        self.assertEqual(loc.pose(), (0, 0, 0))
        self.assertEqual(loc.global_map, [[2, 3]])


if __name__ == "__main__":
    unittest.main()

# vim: expandtab sw=4 ts=4


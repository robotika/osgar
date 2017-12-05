import os
import unittest
from .config import *


def test_data(filename):
    return os.path.join(os.path.dirname(__file__), 'test_data', filename)


class ConfigTest(unittest.TestCase):

    def test_load(self):
        conf = Config.load(test_data('playground12x5.json'))
        self.assertEqual(conf.version, 1)
        self.assertEqual(len(conf.data['localization']['cones']), 4)

    def test_empty_config(self):
        conf = Config()
        self.assertIsNone(conf.data.get('localization'))

if __name__ == "__main__":
    unittest.main() 

# vim: expandtab sw=4 ts=4 


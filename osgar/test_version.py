import unittest
import osgar

class VersionTest(unittest.TestCase):
    def test_version_present(self):
        self.assertTrue(hasattr(osgar, '__version__'))
        self.assertIsInstance(osgar.__version__, str)
        self.assertTrue(len(osgar.__version__) > 0)

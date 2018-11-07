import unittest
from .metalog import *


def test_data(filename):
    return os.path.join(os.path.dirname(__file__), 'test_data', filename)


class MetaLogTest(unittest.TestCase):

    def test_missing_remission(self):
        ml = MetaLog(test_data('meta_170516_192926.log'))
        filename = ml.getLog('laser')
        self.assertIsNotNone(filename)  # logs/laser_170516_192931.log
        filename = ml.getLog('remission')
        self.assertIsNone(filename)
        filename = ml.getLog('camera')
        self.assertIsNotNone(filename)  # logs/camera_170516_192931.log

    def test_getlog_twice(self):
        ml = MetaLog(test_data('meta_170516_192926.log'))
        filename = ml.getLog('laser')
        self.assertIsNotNone(filename)  # logs/laser_170516_192931.log
        filename = ml.getLog('laser')
        self.assertIsNone(filename)

    def test_now(self):
        ml = MetaLog(test_data('meta_160628_174658.log'))
        self.assertEqual(ml.now(),
                         datetime.datetime(2016, 6, 28, 17, 46, 58, 603000))
        self.assertIsNone(ml.now())


if __name__ == "__main__":
    unittest.main() 

# vim: expandtab sw=4 ts=4 


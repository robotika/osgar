import os
import unittest
from osgar.lib.config import load, merge_dict, MergeConflictError, get_class_by_name
from osgar.drivers.logsocket import LogTCP

def test_data(filename, test_dir='test_data'):
    return os.path.join(os.path.dirname(__file__), test_dir, filename)


class MyTestRobot:
    def get_name(self):
        return 'Karel'


class ConfigTest(unittest.TestCase):

    def test_load_config_files(self):
        conf_dir = '../../config'
        filename = test_data('ro2018-spider-gps-imu.json', conf_dir)
        conf = load(filename)
        self.assertEqual(conf['version'], 2)

    def test_multiple_config_files(self):
        conf_dir = '../../config'
        filename1 = test_data('ro2018-spider-gps-imu.json', conf_dir)
        filename2 = test_data('ro2018-czu-waypoints.json', conf_dir)
        conf = load(filename1, filename2)
        self.assertIn('maxspeed',conf['robot']['modules']['app']['init'])
        self.assertIn('out',conf['robot']['modules']['app'])

    def test_merge_dict(self):
        self.assertEqual(merge_dict({'A':1}, {'B':2}), {'A':1, 'B':2})

        dict1 = {'root':{'A':1}}
        dict2 = {'root':{'B':2}}
        merge = {'root':{'A':1, 'B':2}}
        self.assertEqual(merge_dict(dict1, dict2), merge)

        self.assertEqual(merge_dict({'A':1}, {'A':1}), {'A':1})

        with self.assertRaises(MergeConflictError) as e:
            merge_dict({'A':1}, {'A':2})

        with self.assertRaises(MergeConflictError) as e:
            merge_dict({'A':{'B':1}}, {'A':2})

    def test_get_class_by_name(self):
        robot = get_class_by_name('osgar.lib.test_config:MyTestRobot')()
        self.assertEqual(robot.get_name(), 'Karel')

        node = get_class_by_name('tcp')
        self.assertEqual(node, LogTCP)

        node = get_class_by_name('udp')
        self.assertNotEqual(node, LogTCP)

# vim: expandtab sw=4 ts=4


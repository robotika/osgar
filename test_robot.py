import unittest
from unittest.mock import patch, MagicMock
import time

from robot import Robot


class RobotTest(unittest.TestCase):

    def test_dummy_usage(self):
        empty_config = {'modules': {}, 'links':[]}
        robot = Robot(config=empty_config, logger=None)
        robot.start()
        robot.update()
        robot.finish()

    def test_config(self):
        with patch('drivers.logserial.serial.Serial') as mock:
            instance = mock.return_value
            instance.read = MagicMock(return_value=b'$GNGGA,182433.10,5007.71882,N,01422.50467,E,1,05,6.09,305.1,M,44.3,M,,*41')

            config = {
                    'modules': {
                        'gps': {
                            'driver': 'gps',
                            'out':['position'],
                            'init':{}
                        },
                        'serial_gps': {
                            'driver': 'serial',
                            'out':['raw'],
                            'init': {'port': 'COM51', 'speed': 4800}
                        }
                    },
                    'links': [('serial_gps.raw', 'gps.raw')]
            }
            logger = MagicMock()
            robot = Robot(config=config, logger=logger)
            self.assertEqual(len(robot.modules), 2)
            self.assertEqual(sum([sum([len(q) for q in module.bus.out.values()])
                                  for module in robot.modules.values()]), 1)
            robot.start()
            time.sleep(0.1)
            robot.update()
            robot.finish()

    def test_spider_config(self):
        # first example with loop spider <-> serial
        import json
        import os

        with open(os.path.dirname(__file__) + '/config/test-spider.json') as f:
            config = json.loads(f.read())

        with patch('drivers.logserial.serial.Serial') as mock:
            logger = MagicMock()
            robot = Robot(config=config['robot'], logger=logger)

# vim: expandtab sw=4 ts=4

import unittest
from unittest.mock import patch, MagicMock
import time
import json
import os

from osgar.record import Recorder


class RecorderTest(unittest.TestCase):

    def test_dummy_usage(self):
        empty_config = {'modules': {}, 'links':[]}
        recorder = Recorder(config=empty_config, logger=None)
        recorder.start()
        recorder.update()
        recorder.finish()

    def test_config(self):
        with patch('osgar.drivers.logserial.serial.Serial') as mock:
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
            recorder = Recorder(config=config, logger=logger)
            self.assertEqual(len(recorder.modules), 2)
            self.assertEqual(sum([sum([len(q) for q in module.bus.out.values()])
                                  for module in recorder.modules.values()]), 1)
            recorder.start()
            time.sleep(0.1)
            recorder.update()
            recorder.finish()

    def test_spider_config(self):
        # first example with loop spider <-> serial
        with open(os.path.dirname(__file__) + '/../config/test-spider.json') as f:
            config = json.loads(f.read())

        with patch('osgar.drivers.logserial.serial.Serial') as mock:
            logger = MagicMock()
            recorder = Recorder(config=config['robot'], logger=logger)


    def test_all_supported_config_files(self):
        supported = ['test-spider.json', 'test-gps-imu.json',
                'test-spider-gps-imu.json', 'test-windows-gps.json']

        with patch('osgar.drivers.logserial.serial.Serial') as mock:
            logger = MagicMock()
            for filename in supported:
                with open(os.path.join(os.path.dirname(__file__), '..', 'config',
                                   filename)) as f:
                    config = json.loads(f.read())
                recorder = Recorder(config=config['robot'], logger=logger)

    def test_application(self):
        # plug-in external application
        with open(os.path.dirname(__file__) + '/../config/ro2018-spider-gps-imu.json') as f:
            config = json.loads(f.read())

        class DummyApp:
            def __init__(self, config, bus):
                self.bus = bus  # needed for linkage

        with patch('osgar.drivers.logserial.serial.Serial') as mock:
            logger = MagicMock()
            recorder = Recorder(config=config['robot'], logger=logger, application=DummyApp)

# vim: expandtab sw=4 ts=4

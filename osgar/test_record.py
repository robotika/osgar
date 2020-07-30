import unittest
from unittest.mock import patch, MagicMock
from datetime import timedelta
import json
import os
import threading
import signal
import subprocess
import tempfile
import sys

from osgar.record import Recorder


class Sleeper:

    def __init__(self, cfg, bus):
        self.e = threading.Event()

    def start(self):
        self.t = threading.Thread(target=self.e.wait, args=(5,))
        self.t.start()

    def join(self, timeout=None):
        self.t.join(timeout)

    def request_stop(self):
        self.e.set()


class RecorderTest(unittest.TestCase):

    def test_dummy_usage(self):
        empty_config = {'modules': {}, 'links':[]}
        with Recorder(config=empty_config, logger=MagicMock()) as recorder:
            pass

    def test_missing_init(self):
        # init section for modules is now optional
        mini_config = {'modules': {
            "dummy": {
                "driver": "osgar.test_record:Sleeper"
            },
        }, 'links':[]}
        with Recorder(config=mini_config, logger=MagicMock()) as recorder:
            pass

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
            logger = MagicMock(write = MagicMock(return_value=timedelta(seconds=135)))
            with Recorder(config=config, logger=logger) as recorder:
                self.assertEqual(len(recorder.modules), 2)
                self.assertEqual(sum([sum([len(q) for q in module.bus.out.values()])
                                      for module in recorder.modules.values()]), 1)

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

    @unittest.skipIf(os.name != "posix", "requires posix shell")
    def test_sigint_shell(self):
        config = {
            'version': 2,
            'robot': {
                'modules': {
                    "app": {
                        "driver": "osgar.test_record:Sleeper",
                        "init": {}
                    },
                }, 'links':[]
            }
        }
        with tempfile.NamedTemporaryFile() as cfg:
            cfg.write(json.dumps(config).encode('ascii'))
            cfg.flush()
            env = os.environ.copy()
            env['OSGAR_LOGS'] = '.'
            with subprocess.Popen(
                    f"echo starting; {sys.executable} -m osgar.record {cfg.name}; echo should not get here",
                    shell=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    start_new_session=True,
                    env=env,
            ) as proc:
                grp_id = os.getpgid(proc.pid)
                self.assertEqual(proc.stdout.readline().strip(), b"starting")
                log_line = proc.stderr.readline().strip().split()
                log_filename = log_line[-1]
                self.assertTrue(log_filename.endswith(b".log"), log_line)
                self.assertIn(b"SIGINT handler installed", proc.stderr.readline())
                os.killpg(grp_id, signal.SIGINT)
                stdout, stderr = proc.communicate()
                self.assertIn(b"committing suicide by SIGINT", stderr)
                self.assertEqual(len(stdout), 0, stdout)
                self.assertEqual(len(stderr.splitlines()), 1, stderr)
            os.unlink(log_filename)


# vim: expandtab sw=4 ts=4

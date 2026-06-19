import unittest
import tempfile
import os
import sys
from ast import literal_eval

from osgar.logger import LogWriter, LogReader
from osgar.replay import replay
from osgar.lib.config import config_load


class DummyDriver:
    def __init__(self, config, bus):
        self.bus = bus
        self.config = config
        self.speed = config.get('speed', 0.5)

    def start(self):
        pass

    def join(self, timeout=None):
        pass

    def request_stop(self):
        pass


class ReplayTest(unittest.TestCase):
    def setUp(self):
        self.temp_files = []
        self.module_instances = []

    def tearDown(self):
        for m in self.module_instances:
            if hasattr(m, 'bus') and hasattr(m.bus, 'reader') and m.bus.reader is not None:
                m.bus.reader.close()
        for f in self.temp_files:
            if os.path.exists(f):
                try:
                    os.remove(f)
                except OSError:
                    pass

    def create_mock_log(self, config):
        fd, path = tempfile.mkstemp(suffix='.log')
        os.close(fd)
        self.temp_files.append(path)
        
        with LogWriter(filename=path, note="mock-args") as log:
            log.write(0, bytes(str(config), 'ascii'))
            log.register('dummy.out')
        return path

    def create_mock_config_file(self, config):
        fd, path = tempfile.mkstemp(suffix='.json')
        os.close(fd)
        self.temp_files.append(path)
        import json
        with open(path, 'w') as f:
            json.dump(config, f)
        return path

    def test_replay_default_config(self):
        config = {
            'version': 2,
            'robot': {
                'modules': {
                    'dummy': {
                        'driver': 'osgar.test_replay:DummyDriver',
                        'init': {
                            'speed': 1.2
                        }
                    }
                },
                'links': []
            }
        }
        log_path = self.create_mock_log(config)
        
        class Args:
            logfile = log_path
            config = None
            params = None
            module = 'dummy'
            debug = False
            ignore = None
            force = False
            duration = None
            output = None

        module_instance = replay(Args())
        self.module_instances.append(module_instance)
        self.assertEqual(module_instance.speed, 1.2)

    def test_replay_with_params_override(self):
        config = {
            'version': 2,
            'robot': {
                'modules': {
                    'dummy': {
                        'driver': 'osgar.test_replay:DummyDriver',
                        'init': {
                            'speed': 1.2
                        }
                    }
                },
                'links': []
            }
        }
        log_path = self.create_mock_log(config)
        
        class Args:
            logfile = log_path
            config = None
            params = ['dummy.speed=0.9']
            module = 'dummy'
            debug = False
            ignore = None
            force = False
            duration = None
            output = None

        module_instance = replay(Args())
        self.module_instances.append(module_instance)
        self.assertEqual(module_instance.speed, 0.9)

    def test_replay_with_config_and_params(self):
        config_in_log = {
            'version': 2,
            'robot': {
                'modules': {
                    'dummy': {
                        'driver': 'osgar.test_replay:DummyDriver',
                        'init': {
                            'speed': 1.2
                        }
                    }
                },
                'links': []
            }
        }
        log_path = self.create_mock_log(config_in_log)

        alt_config = {
            'version': 2,
            'robot': {
                'modules': {
                    'dummy': {
                        'driver': 'osgar.test_replay:DummyDriver',
                        'init': {
                            'speed': 2.5
                        }
                    }
                },
                'links': []
            }
        }
        config_path = self.create_mock_config_file(alt_config)
        
        class Args:
            logfile = log_path
            config = [config_path]
            params = ['dummy.speed=4.0']
            module = 'dummy'
            debug = False
            ignore = None
            force = False
            duration = None
            output = None

        module_instance = replay(Args())
        self.module_instances.append(module_instance)
        self.assertEqual(module_instance.speed, 4.0)

    def test_replay_output_writes_modified_config(self):
        config = {
            'version': 2,
            'robot': {
                'modules': {
                    'dummy': {
                        'driver': 'osgar.test_replay:DummyDriver',
                        'init': {
                            'speed': 1.2
                        }
                    }
                },
                'links': []
            }
        }
        log_path = self.create_mock_log(config)
        
        fd, output_path = tempfile.mkstemp(suffix='.log')
        os.close(fd)
        self.temp_files.append(output_path)

        class Args:
            logfile = log_path
            config = None
            params = ['dummy.speed=0.1']
            module = 'dummy'
            debug = False
            ignore = None
            force = True
            duration = None
            output = output_path

        module_instance = replay(Args())
        self.module_instances.append(module_instance)
        self.assertEqual(module_instance.speed, 0.1)

        # Close the writer so the output log is flushed to disk
        module_instance.bus.writer.close()

        # Read stream 0 from output_path and verify that speed is 0.1
        with LogReader(output_path, only_stream_id=0) as log:
            next(log)  # skip args note
            written_config_str = next(log)[-1]
            written_config = literal_eval(written_config_str.decode('ascii'))
            
            speed = written_config['robot']['modules']['dummy']['init']['speed']
            self.assertEqual(speed, 0.1)


# vim: expandtab sw=4 ts=4

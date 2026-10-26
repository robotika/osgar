import unittest
from unittest.mock import patch, MagicMock
from datetime import datetime, timedelta, timezone
import json
import os
import threading
import signal
import subprocess
import tempfile
import sys
import socket
import time
import asyncio

from osgar.record import Recorder, record
from osgar.node import Node
from osgar.logger import LogWriter, LogReader
from osgar.lib.serialize import deserialize, serialize
from osgar.drivers.logsocket import LogSocket, LogTCPBase
from osgar.replay import replay
from types import SimpleNamespace
from osgar.asyncnode import AsyncNode
from osgar.shutdown import ShutdownTester


class StopRequester(Node):
    def run(self):
        self.bus.request_stop()


class FinishedApp(Node):
    def run(self):
        pass


class FinalCommandNode(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('raw')

    def on_shutdown(self, data):
        if data['phase'] == 'prepare':
            self.publish('raw', b'STOP')
        super().on_shutdown(data)


class ShutdownTransport(LogSocket):
    """Exercise the socket transport loops against a simulated device."""
    def __init__(self, config, bus):
        connection, self.device = socket.socketpair()
        super().__init__(connection, {'timeout': 0.01}, bus)

    _send = LogTCPBase._send


class AcknowledgedStopNode(FinalCommandNode):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        self.phases = []
        self.response = b''

    def on_shutdown(self, data):
        self.phases.append(data['phase'])
        if data['phase'] == 'prepare':
            self.publish('raw', b'STOP')
        else:
            super().on_shutdown(data)

    def on_raw(self, data):
        self.response += data
        if self.phases == ['prepare'] and self.response == b'ACK':
            self.bus.shutdown_ready()


class AsyncShutdownTransport(AsyncNode):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        self.sent = []
        self.cleaned = False

    async def on_raw(self, data):
        await asyncio.sleep(0.01)
        self.sent.append(data)

    async def main(self):
        try:
            await self.listen_loop()
        finally:
            await asyncio.sleep(0.01)
            self.cleaned = True


class StalledAsyncShutdown(AsyncNode):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        self.cleaned = False

    async def on_shutdown(self, data):
        if data['phase'] == 'prepare':
            await asyncio.Event().wait()
        else:
            super().on_shutdown(data)

    async def main(self):
        try:
            await self.listen_loop()
        finally:
            await asyncio.sleep(0.01)
            self.cleaned = True


class Sleeper:

    def __init__(self, cfg, bus):
        # dedicated part for test of environment variables
        if cfg.get('assert_env'):
            assert 'env' in cfg, cfg
        self.e = threading.Event()

    def start(self):
        self.t = threading.Thread(target=self.e.wait, args=(5,))
        self.t.start()

    def join(self, timeout=None):
        self.t.join(timeout)

    def request_stop(self):
        self.e.set()


class RecorderTest(unittest.TestCase):

    def test_replay_stop_request_preserves_lifecycle_messages(self):
        config = {'version': 2, 'robot': {'modules': {
            'dummy': {'driver': 'osgar.shutdown:ShutdownTester', 'shutdown_protocol': 'messages'},
        }, 'links': [['timer.tick', 'dummy.tick']]}}
        with tempfile.TemporaryDirectory() as directory:
            filename = os.path.join(directory, 'shutdown.log')
            with LogWriter(filename=filename, note='shutdown-replay-test') as logger:
                logger.write(0, str(config).encode('ascii'), dt=timedelta())
                tick = logger.register('timer.tick')
                lifecycle = logger.register('_system.shutdown_dummy')
                ready = logger.register('dummy._shutdown_ready')
                logger.write(tick, serialize(3.1), dt=timedelta(seconds=3.1))
                logger.write(lifecycle, serialize({'phase': 'prepare'}), dt=timedelta(seconds=3.2))
                logger.write(ready, serialize(True), dt=timedelta(seconds=3.3))
                logger.write(lifecycle, serialize({'phase': 'close'}), dt=timedelta(seconds=3.4))
            for force in (False, True):
                with self.subTest(force=force):
                    phases = []
                    def on_shutdown(node, data):
                        phases.append(data['phase'])
                        Node.on_shutdown(node, data)
                    args = SimpleNamespace(logfile=filename, config=None, params=None, module='dummy',
                                           debug=False, ignore=None, force=force, duration=None, output=None)
                    node = replay(args)
                    try:
                        with patch.object(ShutdownTester, 'on_shutdown', on_shutdown):
                            with self.assertRaises(SystemExit):
                                node.run()
                        self.assertEqual(phases, ['prepare', 'close'])
                    finally:
                        node.bus.reader.close()

    def test_async_transport_drains_and_finishes_cleanup(self):
        for protocol in ('legacy', 'messages'):
            with self.subTest(protocol=protocol), tempfile.TemporaryDirectory() as directory:
                config = {'modules': {
                    'transport': {'driver': 'osgar.test_record:AsyncShutdownTransport',
                                  'shutdown_group': 'transport', 'shutdown_protocol': protocol},
                    'controller': {'driver': 'osgar.test_record:FinalCommandNode', 'shutdown_protocol': 'messages'},
                }, 'links': [['controller.raw', 'transport.raw']]}
                with LogWriter(filename=os.path.join(directory, 'async.log')) as logger:
                    with Recorder(config, logger) as recorder:
                        pass
                    transport = recorder.modules['transport']
                    self.assertEqual(transport.sent, [b'STOP'])
                    self.assertTrue(transport.cleaned)
                    self.assertFalse(transport.is_alive())

    def test_async_timeout_cancels_blocked_handler(self):
        config = {'shutdown_timeout': 0.05, 'modules': {
            'controller': {'driver': 'osgar.test_record:StalledAsyncShutdown', 'shutdown_protocol': 'messages'},
        }, 'links': []}
        with tempfile.TemporaryDirectory() as directory:
            with LogWriter(filename=os.path.join(directory, 'timeout.log')) as logger:
                with self.assertLogs('osgar.record', level='ERROR') as logs:
                    with Recorder(config, logger) as recorder:
                        pass
                node = recorder.modules['controller']
                self.assertIn('STOPPING', '\n'.join(logs.output))
                self.assertTrue(node.cleaned)
                self.assertFalse(node.is_alive())

    def test_device_acknowledgement_and_replay(self):
        config = {'modules': {
            'tcp': {'driver': 'osgar.test_record:ShutdownTransport', 'shutdown_group': 'transport'},
            'controller': {'driver': 'osgar.test_record:AcknowledgedStopNode', 'shutdown_protocol': 'messages'},
        }, 'links': [['controller.raw', 'tcp.raw'], ['tcp.raw', 'controller.raw']]}
        with tempfile.TemporaryDirectory() as directory:
            filename = os.path.join(directory, 'ack.log')
            with LogWriter(filename=filename, note='ack-test') as logger:
                logger.write(0, str({'version': 2, 'robot': config}).encode('ascii'))
                with Recorder(config, logger) as recorder:
                    transport = recorder.modules['tcp']
                    observations = []
                    def device():
                        transport.device.settimeout(1)
                        command = transport.device.recv(4)
                        observations.append((command, transport.bus.is_alive(),
                                             recorder.bus.handle('controller').shutdown_ready_event.is_set()))
                        transport.device.sendall(b'ACK')
                    responder = threading.Thread(target=device, daemon=True)
                    responder.start()
                responder.join(1)
                self.assertEqual(observations, [(b'STOP', True, False)])
                self.assertEqual(recorder.modules['controller'].phases, ['prepare', 'close'])
                transport.socket.close()
                transport.device.close()
            for force in (False, True):
                args = SimpleNamespace(logfile=filename, config=None, params=None, module='controller',
                                       debug=False, ignore=None, force=force, duration=None, output=None)
                node = replay(args)
                try:
                    with self.assertRaises(SystemExit):
                        node.run()
                    self.assertEqual(node.phases, ['prepare', 'close'])
                    self.assertEqual(node.response, b'ACK')
                finally:
                    node.bus.reader.close()

    def test_missing_acknowledgement_times_out(self):
        config = {'shutdown_timeout': 0.1, 'modules': {
            'tcp': {'driver': 'osgar.test_record:ShutdownTransport', 'shutdown_group': 'transport'},
            'controller': {'driver': 'osgar.test_record:AcknowledgedStopNode', 'shutdown_protocol': 'messages'},
        }, 'links': [['controller.raw', 'tcp.raw'], ['tcp.raw', 'controller.raw']]}
        with tempfile.TemporaryDirectory() as directory:
            with LogWriter(filename=os.path.join(directory, 'timeout.log')) as logger:
                with self.assertLogs('osgar.record', level='ERROR') as logs:
                    with Recorder(config, logger) as recorder:
                        pass
                self.assertIn('STOPPING', '\n'.join(logs.output))
                self.assertFalse(recorder.bus.handle('controller').shutdown_ready_event.is_set())
                recorder.modules['controller'].join(1)
                transport = recorder.modules['tcp']
                self.assertFalse(transport.output_thread.is_alive())
                transport.socket.close()
                transport.device.close()

    def test_node_requests_global_stop(self):
        for with_app in (False, True):
            with self.subTest(with_app=with_app), tempfile.TemporaryDirectory() as directory:
                modules = {'requester': {'driver': 'osgar.test_record:StopRequester'}}
                if with_app:
                    modules['app'] = {'driver': 'osgar.test_record:Sleeper'}
                config = {'version': 2, 'robot': {'modules': modules, 'links': []}}
                filename = os.path.join(directory, 'config.json')
                with open(filename, 'w') as stream:
                    json.dump(config, stream)
                result = subprocess.run(
                    [sys.executable, '-m', 'osgar.record', filename, '--duration', '10',
                     '--log', os.path.join(directory, 'run.log')],
                    capture_output=True, timeout=3,
                )
                self.assertEqual(result.returncode, 0, result.stderr.decode())
                self.assertNotIn(b'still running', result.stderr)

    def test_stop_calls_every_module_once(self):
        recorder = Recorder({'modules': {}, 'links': []}, MagicMock())
        recorder.modules = {'one': MagicMock(), 'two': MagicMock()}
        recorder.request_stop()
        recorder.request_stop()
        self.assertTrue(recorder.bus.stop_requested.is_set())
        for module in recorder.modules.values():
            module.request_stop.assert_not_called()
        recorder.__exit__(None, None, None)
        for module in recorder.modules.values():
            module.request_stop.assert_called_once_with()

    def test_final_command_precedes_transport_shutdown(self):
        config = {'modules': {
            # Deliberately put the transport first: configuration order must not matter.
            'tcp': {'driver': 'osgar.test_record:ShutdownTransport', 'shutdown_group': 'transport'},
            'controller': {'driver': 'osgar.test_record:FinalCommandNode', 'shutdown_protocol': 'messages'},
        }, 'links': [['controller.raw', 'tcp.raw']]}
        with tempfile.TemporaryDirectory() as directory:
            filename = os.path.join(directory, 'shutdown.log')
            start_time = datetime.now(timezone.utc) - timedelta(seconds=100)
            with LogWriter(filename=filename, start_time=start_time) as logger:
                with Recorder(config, logger) as recorder:
                    transport = recorder.modules['tcp']
                    lifecycle_stream = recorder.bus.handle('_system').stream_id['shutdown_controller']
                    recorder.request_stop()
                try:
                    transport.device.settimeout(1)
                    self.assertEqual(transport.device.recv(4), b'STOP')
                    self.assertFalse(transport.output_thread.is_alive())
                    self.assertFalse(transport.input_thread.is_alive())
                    self.assertEqual(recorder.state, 'CLOSED')
                finally:
                    transport.socket.close()
                    transport.device.close()
            with LogReader(filename) as reader:
                commands = [deserialize(data) for _, stream, data in reader if stream == 2]
            self.assertEqual(commands, [b'STOP'])
            with LogReader(filename) as reader:
                lifecycle = [(timestamp, deserialize(data)) for timestamp, stream, data in reader
                             if stream == lifecycle_stream]
            self.assertEqual([data['phase'] for _, data in lifecycle], ['prepare', 'close'])
            self.assertEqual(lifecycle[0][1]['deadline'], lifecycle[1][1]['deadline'])
            for timestamp, data in lifecycle:
                self.assertGreaterEqual(data['deadline'], 100)
                self.assertAlmostEqual(data['deadline'] - timestamp.total_seconds(), data['timeout'], delta=0.1)

    def test_stalled_stop_does_not_block_other_modules(self):
        release = threading.Event()
        entered = threading.Event()
        finished = threading.Event()
        recorder = Recorder({'modules': {}, 'links': [], 'shutdown_timeout': 0.05}, MagicMock())
        stalled = MagicMock()
        def wait_forever():
            entered.set()
            release.wait()
        stalled.request_stop.side_effect = wait_forever
        stalled.join.side_effect = finished.set
        transport = MagicMock()
        recorder.modules = {'stalled': stalled, 'tcp': transport}
        recorder.shutdown_groups = {'tcp': 'transport'}
        start = time.monotonic()
        try:
            with self.assertLogs('osgar.record', level='ERROR') as logs:
                recorder.__exit__(None, None, None)
            self.assertTrue(entered.is_set())
            self.assertLess(time.monotonic() - start, 1)
            self.assertIn('Shutdown timeout in STOPPING: stalled', '\n'.join(logs.output))
            transport.request_stop.assert_called_once_with()
            self.assertEqual(recorder.state, 'CLOSED')
        finally:
            release.set()
            self.assertTrue(finished.wait(1))

    def test_shutdown_configuration_validation(self):
        for timeout in (0, -1, float('inf'), float('nan'), '5'):
            with self.subTest(timeout=timeout), self.assertRaises(ValueError):
                Recorder({'modules': {}, 'shutdown_timeout': timeout}, MagicMock())
        with self.assertRaises(ValueError):
            Recorder({'modules': {'bad': {'shutdown_group': 'unknown'}}}, MagicMock())
        with self.assertRaises(ValueError):
            Recorder({'modules': {'bad': {'shutdown_protocol': 'unknown'}}}, MagicMock())
        with self.assertRaises(ValueError):
            Recorder({'modules': {'_system': {}}}, MagicMock())

    def test_failed_controller_cleanup_still_stops_transport(self):
        recorder = Recorder({'modules': {}, 'links': []}, MagicMock())
        controller, transport = MagicMock(), MagicMock()
        controller.request_stop.side_effect = RuntimeError('cleanup failed')
        recorder.modules = {'controller': controller, 'tcp': transport}
        recorder.shutdown_groups = {'tcp': 'transport'}
        with self.assertLogs('osgar.record', level='ERROR') as logs:
            recorder.__exit__(None, None, None)
        self.assertIn('Shutdown failed in STOPPING: controller: cleanup failed', '\n'.join(logs.output))
        transport.request_stop.assert_called_once_with()
        self.assertEqual(recorder.state, 'CLOSED')

    def test_transport_drain_timeout(self):
        release, finished = threading.Event(), threading.Event()
        recorder = Recorder({'modules': {}, 'links': [], 'shutdown_timeout': 0.05}, MagicMock())
        transport = MagicMock()
        def stalled_join():
            release.wait()
            finished.set()
        transport.join.side_effect = stalled_join
        recorder.modules = {'tcp': transport}
        recorder.shutdown_groups = {'tcp': 'transport'}
        try:
            with self.assertLogs('osgar.record', level='ERROR') as logs:
                recorder.__exit__(None, None, None)
            self.assertIn('Shutdown timeout in DRAINING: tcp', '\n'.join(logs.output))
            self.assertEqual(recorder.state, 'CLOSED')
        finally:
            release.set()
            self.assertTrue(finished.wait(1))

    def test_app_completion_and_duration(self):
        cases = [({'app': {'driver': 'osgar.test_record:FinishedApp'}}, 10), ({}, 0.01)]
        for modules, duration in cases:
            with self.subTest(modules=modules), tempfile.TemporaryDirectory() as directory:
                config = {'version': 2, 'robot': {'modules': modules, 'links': []}}
                record(config, 'test-', log_filename=os.path.join(directory, 'run.log'), duration_sec=duration)

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
        supported = ['test-spider.json', 'test-gps-imu.json', 'test-windows-gps.json']

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
                self.assertIn(b'Shutdown phase: STOPPING', stderr)
                self.assertIn(b'Shutdown phase: DRAINING', stderr)
            os.unlink(log_filename)

    def test_config_env(self):
        config = {
            'version': 2,
            'robot': {
                'env': {
                    'OSGAR_LOG_PREFIX': None
                },
                'modules': {
                    "app": {
                        "driver": "osgar.test_record:Sleeper",
                        "init": {
                            'assert_env': True
                        }
                    },
                }, 'links':[]
            }
        }
        record(config, 'log-env-', duration_sec=0.1)

    def test_config_env_assert(self):
        config = {
            'version': 2,
            'robot': {
                'env': {
                    'OSGAR_LOGS_PREFIX': None
                },
                'modules': {
                    "app": {
                        "driver": "osgar.test_record:Sleeper",
                        "init": {
                            'assert_env': True,
                            'env': 'This will collide -> intentionally assert'
                        }
                    },
                }, 'links':[]
            }
        }
        with self.assertRaises(AssertionError) as err:
            record(config, 'log-env2-', duration_sec=0.1)
        self.assertEqual(str(err.exception),
                         "{'assert_env': True, 'env': 'This will collide -> intentionally assert'}")


# vim: expandtab sw=4 ts=4

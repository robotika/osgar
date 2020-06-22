import datetime
import tempfile
import unittest
import os
import shutil
import pathlib
import time

from unittest.mock import MagicMock

from threading import Thread

import osgar.logger

from osgar.zmqrouter import _Router, _Bus, record


class Noop:
    def __init__(self, config, bus):
        self.bus = bus
        self.bus.register("output")

    def start(self):
        pass

    def join(self, timeout=None):
        pass


class Publisher:
    def __init__(self, config, bus):
        self.bus = bus
        self.output_name = config["output"].split(":")[0] # drop any possible suffix
        self.bus.register(config["output"])

    def start(self):
        self.thread = Thread(target=self.run)
        self.thread.start()

    def join(self, timeout=None):
        self.thread.join(timeout)

    def run(self):
        count = 10
        for i in range(count):
            dt = self.bus.publish(self.output_name, i)
            time.sleep(0.01)
            #print("  published", i, dt)


class NoQuit:
    def __init__(self, config, bus):
        self.bus = bus
        self.bus.register()

    def start(self):
        self.thread = Thread(target=self.run)
        self.thread.start()

    def join(self, timeout=None):
        self.thread.join(timeout)

    def run(self):
        while True:
            time.sleep(1)


class Test(unittest.TestCase):
    def setUp(self):
        self.tempdir = pathlib.Path(tempfile.mkdtemp(dir=pathlib.Path.cwd()))
        os.environ['OSGAR_LOGS'] = str(self.tempdir)

    def tearDown(self):
        shutil.rmtree(self.tempdir)

    def test_threads(self):
        main()

    def test_noop(self):
        config = {
            'version': 2,
            'robot': {
                'modules': {
                    "noop": {
                        "driver": "osgar.test_zmqrouter:Noop",
                        "init": {}
                    },
                }, 'links':[]
            }
        }
        record(config, log_filename='noop.log')

    def test_publisher_single(self):
        config = { 'version': 2, 'robot': { 'modules': {}, 'links': [] } }
        config['robot']['modules']['publisher'] = {
            "driver": "osgar.test_zmqrouter:Publisher",
            "init": { "output": "count"}
        }
        record(config, log_filename='publisher.log')
        with osgar.logger.LogReader(self.tempdir/"publisher.log", only_stream_id=1) as log:
            last_dt = datetime.timedelta()
            count = 0
            for dt, channel, data in log:
                self.assertGreater(dt, last_dt)
                self.assertEqual(int.from_bytes(data, 'little'), count)
                last_dt = dt
                count += 1

    def test_publisher_multi(self):
        config = { 'version': 2, 'robot': { 'modules': {}, 'links': [] } }
        for i in range(3):
            config['robot']['modules'][f'publisher{i}'] = {
                "driver": "osgar.test_zmqrouter:Publisher",
                "init": { "output": f"count{i}" }
            }
        record(config, log_filename='publisher.log')

    def test_compress(self):
        config = { 'version': 2, 'robot': { 'modules': {}, 'links': [] } }
        config['robot']['modules']['publisher'] = {
            "driver": "osgar.test_zmqrouter:Publisher",
            "init": { "output": "count:gz"}
        }
        record(config, log_filename='compressed-publisher.log')

    def test_null(self):
        config = { 'version': 2, 'robot': { 'modules': {}, 'links': [] } }
        config['robot']['modules']['publisher'] = {
            "driver": "osgar.test_zmqrouter:Publisher",
            "init": { "output": "count:null"}
        }
        record(config, log_filename='null-publisher.log')

    def test_fail_to_register(self):
        config = { 'version': 2, 'robot': { 'modules': {}, 'links': [] } }
        config['robot']['modules']['publisher'] = {
            "driver": "osgar.test_zmqrouter:Nonexisting",
            "init": { "output": "count:null"}
        }
        record(config, log_filename='null-publisher.log')

    def test_fail_to_quit(self):
        config = { 'version': 2, 'robot': { 'modules': {}, 'links': [] } }
        config['robot']['modules']['publisher'] = {
            "driver": "osgar.test_zmqrouter:Publisher",
            "init": { "output": "count:null"}
        }
        config['robot']['modules']['noquit'] = {
            "driver": "osgar.test_zmqrouter:NoQuit",
        }
        record(config, log_filename='noquit.log')


def main():
    nodes = ["listener0", "listener1", "publisher"]
    links = [
        ["publisher.count", "listener0.count"],
        ["publisher.count", "listener1.count"],
    ]

    logger = MagicMock()
    logger.start_time = datetime.datetime.now(datetime.timezone.utc)
    with _Router(logger) as router:
        Thread(target=node_listener, args=("listener0",)).start()
        Thread(target=node_listener, args=("listener1",)).start()
        Thread(target=node_publisher, args=("publisher", "count", 10)).start()

        router.register_nodes(nodes)
        for link_from, link_to in links:
            router.connect(link_from, link_to)
        router.run()
    #print(logger.mock_calls)


def node_listener(name):
    bus = _Bus(name)
    bus.register()
    expected = 0
    while True:
        dt, channel, data = bus.listen()
        assert data == expected
        expected += 1
        #print(f"  {name}", dt, channel, data)


def node_publisher(name, channel, count):
    bus = _Bus(name)
    bus.register(channel)
    for i in range(count):
        dt = bus.publish(channel, i)
        #print("  published", i, dt)
    bus.request_stop()


if __name__ == "__main__":
    import logging, sys
    logging.basicConfig(
        stream=sys.stderr,
        level=logging.DEBUG,
        format='%(asctime)s %(name)-12s %(levelname)-8s %(message)s',
        datefmt='%Y-%m-%d %H:%M',
    )
    main()

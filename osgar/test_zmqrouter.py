import datetime
import tempfile
import unittest
import os
import shutil
import pathlib

from unittest.mock import MagicMock

from threading import Thread

import osgar.logger

from osgar.zmqrouter import Router, Bus, record


class Noop:
    def __init__(self, config, bus):
        self.bus = bus

    def start(self):
        self.bus.register()

    def join(self, timeout=None):
        pass


class Publisher:
    def __init__(self, config, bus):
        self.bus = bus

    def start(self):
        self.bus.register("count")
        self.thread = Thread(target=self.run)
        self.thread.start()

    def join(self, timeout=None):
        self.thread.join(timeout)

    def run(self):
        count = 10
        for i in range(count):
            dt = self.bus.publish("count", i)
            #print("  published", i, dt)


class Test(unittest.TestCase):
    def setUp(self):
        self.tempdir = pathlib.Path(tempfile.mkdtemp(dir=pathlib.Path.cwd()))
        os.environ['OSGAR_LOGS'] = str(self.tempdir)

    def tearDown(self):
        shutil.rmtree(self.tempdir)

    def test_threads(self):
        main()

    def test_record_noop(self):
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

    def test_record_publisher(self):
        config = {
            'version': 2,
            'robot': {
                'modules': {
                    "publisher": {
                        "driver": "osgar.test_zmqrouter:Publisher",
                        "init": {}
                    },
                }, 'links':[]
            }
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


def main():
    nodes = ["listener0", "listener1", "publisher"]
    links = [
        ["publisher.count", "listener0.count"],
        ["publisher.count", "listener1.count"],
    ]

    logger = MagicMock()
    logger.start_time = datetime.datetime.now(datetime.timezone.utc)
    with Router(logger) as router:
        Thread(target=node_listener, args=("listener0",)).start()
        Thread(target=node_listener, args=("listener1",)).start()
        Thread(target=node_publisher, args=("publisher", "count", 10)).start()

        router.register_nodes(nodes)
        for link_from, link_to in links:
            router.connect(link_from, link_to)
        router.run()
    #print(logger.mock_calls)


def node_listener(name):
    bus = Bus(name)
    bus.register()
    expected = 0
    while True:
        dt, channel, data = bus.listen()
        assert data == expected
        expected += 1
        #print(f"  {name}", dt, channel, data)


def node_publisher(name, channel, count):
    bus = Bus(name)
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

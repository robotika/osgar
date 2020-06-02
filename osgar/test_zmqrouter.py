import unittest

from threading import Thread

from osgar.zmqrouter import Router, Bus, record


class Test(unittest.TestCase):
    def test_threads(self):
        main()

    def test_record(self):
        with Router() as router:
            pass


def main():
    nodes = ["listener0", "listener1", "publisher"]
    links = [
        ["publisher.count", "listener0.count"],
        ["publisher.count", "listener1.count"],
    ]

    with Router() as router:
        Thread(target=node_listener, args=("listener0",)).start()
        Thread(target=node_listener, args=("listener1",)).start()
        Thread(target=node_publisher, args=("publisher", "count", 10)).start()

        router.connect(len(nodes), links)
        router.run()


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
        print("  published", i, dt)
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

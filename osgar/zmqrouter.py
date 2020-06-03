import ast
import collections
import datetime
import inspect
import logging
import signal
import struct
import subprocess
import sys


from pprint import pprint

import zmq

import osgar.lib.serialize
import osgar.lib.config

g_logger = logging.getLogger(__name__)


def main():
    signal.signal(signal.SIGINT,signal.SIG_IGN)
    module_config = ast.literal_eval(sys.argv[1])
    klass = osgar.lib.config.get_class_by_name(module_config['driver'])
    bus = Bus(module_config['name'])
    instance = klass(config=module_config['init'], bus=bus)
    instance.start()
    g_logger.info(f"{module_config['name']} running")
    instance.join()
    bus.request_stop()


def record(config, log_prefix, log_filename=None, duration_sec=None):
    logger = LogWriter()
    with Router(logger) as router:
        modules = {}
        for module_name, module_config in config['robot']['modules'].items():
            module_config['name'] = module_name
            modules[module_name] = subprocess.Popen([sys.executable, __file__, str(module_config)])

        router.register_nodes(modules.keys())
        links =  config['robot']['links']
        for link_from, link_to in links:
            router.connect(link_from, link_to)
        router.run()

        for module in modules.values():
            module.wait() # TODO terminate rogue modules


class Router:
    def __init__(self, logger):
        self.logger = logger
        self.start_time = datetime.datetime.now(datetime.timezone.utc)
        self._context = zmq.Context()
        self.socket = self._context.socket(zmq.ROUTER)
        self.nodes = dict()
        self.subscriptions = dict()
        self.stream_id = dict()
        self.listening = set()
        self.stopping = datetime.timedelta()

    def __enter__(self):
        self.socket.bind("tcp://127.0.0.1:8881")
        signal.signal(signal.SIGINT, self.sigint)
        self.logger.start(self.start_time)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.socket.close()
        self._context.term()
        self.logger.stop()

    def sigint(self, signum, frame):
        # TODO send message to myself telling me to stop?
        # use Event? what is ok to do from a signal handler?
        self.request_stop(b'ctrl-c')

    def register_nodes(self, nodes):
        # wait for all nodes to "register" their outputs
        nodes = [bytes(node_name, "ascii") for node_name in nodes]
        for _ in range(len(nodes)):
            packet = self.socket.recv_multipart()  # TODO handle timeout
            sender, _, action, *outputs = packet
            assert action == b"register", (sender, action, outputs)
            assert sender not in self.nodes
            assert sender in nodes, (sender, nodes)
            print(sender, outputs)
            # receiving queue
            self.nodes[sender] = collections.deque()
            for o in outputs:
                link_from = sender + b"." + o
                idx = self.logger.register(link_from.decode('ascii'))
                self.stream_id[link_from] = idx

    def connect(self, link_from, link_to):
        link_from = bytes(link_from, "ascii")
        assert link_from in self.stream_id, self.stream_id
        receiver, channel = bytes(link_to, "ascii").split(b".")
        self.subscriptions.setdefault(link_from, []).append((receiver, channel))

    def run(self):
        # answer all connected nodes that it is ok to run now (connections are set up)
        for node_name in self.nodes:
            self.socket.send_multipart([node_name, b"", b"start"])

        for packet in self.receive():
            sender, action, *args = packet
            getattr(self, action.decode('ascii'))(sender, *args)

    def receive(self, hz=10):
        socket = self.socket
        poller = zmq.Poller()
        poller.register(socket, zmq.POLLIN)
        while not self.stopping or self._now() - self.stopping < datetime.timedelta(seconds=1):
            obj = dict(poller.poll(1000//hz))
            if socket in obj and obj[socket] == zmq.POLLIN:
                packet = socket.recv_multipart()
                yield (packet[0], *packet[2:]) # drop frame separator from REQ socket
            elif self.stopping: # timeout && we are stopping
                return

    def listen(self, sender):
        queue = self.nodes[sender]
        if len(queue) == 0:
            self.listening.add(sender)
        else:
            packet = queue.popleft()
            assert packet[0] == sender, (packet[0], sender)
            self.socket.send_multipart(packet)

    def publish(self, sender, channel, data):
        dt = self._now()
        dt_uint32 = self._pack_timestamp(dt)
        self.socket.send_multipart([sender, b"", b"publish", dt_uint32])
        link_from = sender + b"." + channel
        for node_name, input_channel in self.subscriptions.get(link_from, []):
            self.send(node_name, b'listen', dt_uint32, input_channel, data)
        stream_id = self.stream_id[link_from]
        self.logger.write(stream_id, data, dt)

    def request_stop(self, sender):
        if self.stopping:
            return
        g_logger.info(f"{sender.decode('ascii')} requested stop")
        self.stopping = self._now()
        for node_name in self.nodes:
            self.send(node_name, b"quit")

    def is_alive(self, sender):
        dt_uint32 = self._pack_timestamp(self._now())
        self.socket.send_multipart([sender, b"", b"quit" if self.stopping else b"is_alive", dt_uint32])

    def send(self, node_name, *args):
        packet = [node_name, b"", *args]
        if node_name in self.listening:
            self.listening.remove(node_name)
            self.socket.send_multipart(packet)
        else:
            self.nodes[node_name].append(packet)

    def _now(self):
        return datetime.datetime.now(datetime.timezone.utc) - self.start_time

    def _pack_timestamp(self, dt):
        assert dt <= datetime.timedelta(microseconds=(1<<32)-1)
        dt_int = (dt.seconds * 1000000 + dt.microseconds)
        dt_uint32 = struct.pack("I", dt_int)
        return dt_uint32

class Bus:
    def __init__(self, name):
        self.name = name
        context = zmq.Context.instance()
        self.sock = context.socket(zmq.REQ)
        self.sock.setsockopt_string(zmq.IDENTITY, name)
        self.sock.connect("tcp://127.0.0.1:8881")

    def register(self, *outputs):
        bytes_outputs = list(o.encode('ascii') for o in outputs)
        self.sock.send_multipart([b"register", *bytes_outputs])
        resp = self.sock.recv()
        assert resp == b'start'

    def listen(self):
        self.sock.send(b"listen")
        resp = self.sock.recv_multipart()
        if resp[0] == b"quit":
            raise self._quit()
        assert resp[0] == b"listen"
        microseconds = struct.unpack('I', resp[1])[0]
        dt = datetime.timedelta(microseconds=microseconds)
        channel = resp[2].decode('ascii')
        data = osgar.lib.serialize.deserialize(resp[3])
        return dt, channel, data

    def publish(self, channel, data):
        raw = osgar.lib.serialize.serialize(data)
        self.sock.send_multipart([b"publish", channel.encode('ascii'), raw])
        resp, timestamp = self.sock.recv_multipart()
        assert resp == b"publish"
        microseconds = struct.unpack('I', timestamp)[0]
        dt = datetime.timedelta(microseconds=microseconds)
        return dt

    def request_stop(self):
        self.sock.send_multipart([b"request_stop"])

    def is_alive(self):
        self.sock.send_multipart([b"is_alive"])
        resp, timestamp = self.sock.recv_multipart()
        assert resp in [b"is_alive", b"quit"], resp
        if resp == b"quit":
            self._quit()
            return False
        return True

    def _quit(self):
        g_logger.info(f"{self.name} received quit")
        return SystemExit()


if __name__ == "__main__":
    logging.basicConfig(
        stream=sys.stderr,
        level=logging.DEBUG,
        format='%(asctime)s %(name)-12s %(levelname)-8s %(message)s',
        datefmt='%Y-%m-%d %H:%M',
    )
    main()


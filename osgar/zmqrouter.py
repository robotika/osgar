import collections
import datetime
import logging
import signal
import subprocess
import sys
import threading


import zmq

import osgar.lib.serialize
import osgar.lib.config
import osgar.logger

ENDPOINT = "tcp://127.0.0.1:8882"

g_logger = logging.getLogger(__name__)


def child(name, module_config):
    # todo how to inherit logging setup from parent process?
    signal.signal(signal.SIGINT,signal.SIG_IGN)
    klass = osgar.lib.config.get_class_by_name(module_config['driver'])
    bus = Bus(name)
    instance = klass(config=module_config['init'], bus=bus)
    instance.start()
    g_logger.info(f"{name} running")
    instance.join()
    bus.request_stop()


def record(config, log_prefix=None, log_filename=None, duration_sec=None):
    g_logger.info("recording...")
    with osgar.logger.LogWriter(prefix=log_prefix, filename=log_filename, note=str(sys.argv)) as log:
        log.write(0, bytes(str(config), 'ascii'))
        g_logger.info(log.filename)
        with Router(log) as router:
            modules = {}
            for module_name, module_config in config['robot']['modules'].items():
                program = f"import {__name__}; {__name__}.child('{module_name}', {module_config})"
                modules[module_name] = subprocess.Popen([sys.executable, "-c", program])

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
        self.start_time = self.logger.start_time
        self._context = zmq.Context()
        self.socket = self._context.socket(zmq.ROUTER)
        self.nodes = dict()
        self.delays = dict()
        self.subscriptions = dict()
        self.stream_id = dict()
        self.listening = set()
        self.stopping = datetime.timedelta()
        self.no_output = set()
        self.compressed_output = set()

    def __enter__(self):
        self.socket.bind(ENDPOINT)
        signal.signal(signal.SIGINT, self.sigint)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.socket.close()
        self._context.term()

    def sigint(self, signum, frame):
        # TODO se pipe-to-self trick to tell receive loop it is time to quit
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
            # receiving queue
            self.nodes[sender] = collections.deque()
            self.delays[sender] = datetime.timedelta()
            for name_and_type in outputs:
                o, *suffix = name_and_type.split(b':')
                suffix = suffix[0] if suffix else b''
                link_from = sender + b"." + o
                idx = self.logger.register(str(link_from, 'ascii'))
                self.stream_id[link_from] = idx
                if suffix == b'null':
                    self.no_output.add(idx)
                elif suffix == b'gz':
                    self.compressed_output.add(idx)

    def connect(self, link_from, link_to):
        link_from = bytes(link_from, "ascii")
        link_to = bytes(link_to, "ascii")
        assert link_from in self.stream_id, (link_from, self.stream_id.keys())
        receiver, channel = link_to.split(b".")
        self.subscriptions.setdefault(link_from, []).append((receiver, channel))
        g_logger.info(f"connect {link_from} -> {link_to}")

    def run(self):
        # answer all connected nodes that it is ok to run now (connections are set up)
        for node_name in self.nodes:
            self.socket.send_multipart([node_name, b"", b"start"])

        for packet in self.receive():
            sender, action, *args = packet
            getattr(self, str(action, 'ascii'))(sender, *args)

        for sender, delay in self.delays.items():
            g_logger.info(f"{str(sender, 'ascii')}: {delay}")

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
            first_dt = queue[0][0]
            last_dt = queue[-1][0]
            packet = queue.popleft()[1]
            assert packet[0] == sender, (packet[0], sender)
            self.socket.send_multipart(packet)
            delay = last_dt - first_dt
            if self.delays[sender] < delay:
                self.delays[sender] = delay

    def publish(self, sender, channel, data):
        dt = self._now()
        dt_uint32 = osgar.logger.format_timedelta(dt)
        self.socket.send_multipart([sender, b"", b"publish", dt_uint32])
        link_from = sender + b"." + channel
        for node_name, input_channel in self.subscriptions.get(link_from, []):
            self.send(node_name, b'listen', dt, input_channel, data)
        stream_id = self.stream_id[link_from]
        if stream_id in self.no_output:
            data = osgar.lib.serialize.serialize(None)
        elif stream_id in self.compressed_output:
            data = osgar.lib.serialize.compress(data)
        self.logger.write(stream_id, data, dt)

    def request_stop(self, sender):
        if self.stopping:
            return
        g_logger.info(f"{str(sender, 'ascii')} requested stop")
        self.stopping = self._now()
        for node_name in self.nodes:
            self.send(node_name, b"quit", self.stopping)

    def is_alive(self, sender):
        dt_uint32 = osgar.logger.format_timedelta(self._now())
        self.socket.send_multipart([sender, b"", b"quit" if self.stopping else b"is_alive", dt_uint32])

    def send(self, node_name, action, dt, *args):
        dt_uint32 = osgar.logger.format_timedelta(dt)
        packet = [node_name, b"", action, dt_uint32, *args]
        if node_name in self.listening:
            self.listening.remove(node_name)
            self.socket.send_multipart(packet)
        else:
            self.nodes[node_name].append((dt, packet))

    def _now(self):
        return datetime.datetime.now(datetime.timezone.utc) - self.start_time


class Bus:
    def __init__(self, name):
        self.lock = threading.RLock()
        self.name = name
        context = zmq.Context()
        self.sock = context.socket(zmq.REQ)
        self.sock.setsockopt_string(zmq.IDENTITY, name)
        self.sock.connect(ENDPOINT)
        self.parse_listen_dt = osgar.logger.timedelta_parser()
        self.parse_publish_dt = osgar.logger.timedelta_parser()

    def register(self, *outputs):
        bytes_outputs = list(bytes(o, 'ascii') for o in outputs)
        with self.lock:
            self.sock.send_multipart([b"register", *bytes_outputs])
            resp = self.sock.recv()
        assert resp == b'start'

    def listen(self):
        with self.lock:
            self.sock.send(b"listen")
            action, dt, *args = self.sock.recv_multipart()
        if action == b"quit":
            raise self._quit()
        assert action == b"listen"
        dt = self.parse_listen_dt(dt)
        channel = str(args[0], 'ascii')
        data = osgar.lib.serialize.deserialize(args[1])
        return dt, channel, data

    def publish(self, channel, data):
        raw = osgar.lib.serialize.serialize(data)
        with self.lock:
            self.sock.send_multipart([b"publish", bytes(channel, 'ascii'), raw])
            resp, timestamp = self.sock.recv_multipart()
        assert resp == b"publish"
        dt = self.parse_publish_dt(timestamp)
        return dt

    def request_stop(self):
        with self.lock:
            self.sock.send_multipart([b"request_stop"])

    def is_alive(self):
        with self.lock:
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

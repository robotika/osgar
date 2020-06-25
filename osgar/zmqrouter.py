import collections
import datetime
import logging
import signal
import subprocess
import sys
import threading
import time


import zmq

import osgar.lib.serialize
import osgar.lib.config
import osgar.logger

ENDPOINT = "tcp://127.0.0.1:8882"
ASSERT_QUEUE_DELAY = datetime.timedelta(seconds=.1)
STOPPING_TIMEOUT = datetime.timedelta(seconds=2)

g_logger = logging.getLogger(sys.modules[__name__].__spec__.name)


def child(name, module_config, log_level):
    # todo how to inherit logging setup from parent process?
    # a proper way is probably using a config file
    import logging, sys
    #print(logging.getLevelName(log_level))
    logging.basicConfig(
        stream=sys.stdout,
        level=log_level,
        format='%(asctime)s %(name)-16s %(levelname)-8s %(message)s',
    )
    # todo end
    signal.signal(signal.SIGINT,signal.SIG_IGN)
    klass = osgar.lib.config.get_class_by_name(module_config['driver'])
    bus = _Bus(name)
    instance = klass(config=module_config.get('init', {}), bus=bus)
    instance.start()
    g_logger.info(f"{name} running")
    instance.join()
    g_logger.info(f"{name} finished")
    bus.request_stop()
    g_logger.debug(f"{name} stop confirmed")


def record(config, log_prefix=None, log_filename=None, duration_sec=None):
    g_logger.info("recording...")
    with osgar.logger.LogWriter(prefix=log_prefix, filename=log_filename, note=str(sys.argv)) as log:
        log.write(0, bytes(str(config), 'ascii'))
        g_logger.info(log.filename)
        with _Router(log) as router:
            modules = {}
            for module_name, module_config in config['robot']['modules'].items():
                program = f"import {__name__}; {__name__}.child('{module_name}', {module_config}, {logging.root.level})"
                modules[module_name] = subprocess.Popen([sys.executable, "-c", program])

            try:
                router.register_nodes(modules.keys(), timeout=datetime.timedelta(seconds=1))
                links =  config['robot']['links']
                for link_from, link_to in links:
                    router.connect(link_from, link_to)
                router.run()
            except Exception as e:
                g_logger.error(str(e))
                router.request_stop(b"exception")

            for module in modules.values():
                try:
                    # always stop within 1s
                    timeout = 1 - (router.now() - router.stopping).total_seconds()
                    if timeout < 0:
                        timeout = 0
                    module.wait(timeout)
                except subprocess.TimeoutExpired:
                    module.kill()
                    module.wait()


class _Router:
    def __init__(self, logger):
        self.logger = logger
        self.start_time = self.logger.start_time
        self._context = zmq.Context()
        self.socket = self._context.socket(zmq.ROUTER)
        self.nodes = dict()
        self.delays = dict()
        self.last_listen = dict()
        self.stopped = set()
        self.subscriptions = dict()
        self.stream_id = dict()
        self.listening = set()
        self.stopping = datetime.timedelta()
        self.no_output = set()
        self.compressed_output = set()
        self._saved_sigint = None

    def __enter__(self):
        self.socket.bind(ENDPOINT)
        self._saved_sigint = signal.signal(signal.SIGINT, self.sigint)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        signal.signal(signal.SIGINT, self._saved_sigint)
        self.socket.close()
        self._context.term()

    def sigint(self, signum, frame):
        # TODO se pipe-to-self trick to tell receive loop it is time to quit
        g_logger.error("ctrl-c")
        self.request_stop(b'ctrl-c')

    def register_nodes(self, nodes, timeout=datetime.timedelta.max):
        # wait for all nodes to "register" their outputs
        nodes = set(bytes(node_name, "ascii") for node_name in nodes)
        for packet in self.receive(timeout=timeout):
            sender, action, *args = packet
            assert action == b"register", (sender, action, args)
            assert sender not in self.nodes             # we have not registered the node yet
            assert sender in nodes, (sender, nodes)     # it is one of the nodes we expect
            self.nodes[sender] = collections.deque()    # receiving queue
            self.delays[sender] = datetime.timedelta()
            for name_and_type in args:
                o, *suffix = name_and_type.split(b':')
                suffix = suffix[0] if suffix else b''
                link_from = sender + b"." + o
                idx = self.logger.register(str(link_from, 'ascii'))
                self.stream_id[link_from] = idx
                if suffix == b'null':
                    self.no_output.add(idx)
                elif suffix == b'gz':
                    self.compressed_output.add(idx)
            if self.nodes.keys() == nodes:
                return
        raise RuntimeError("unexpected stop")

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
            if delay > ASSERT_QUEUE_DELAY:
                g_logger.error(f"{str(sender, 'ascii')} max delay: {delay}")
            else:
                g_logger.info(f"{str(sender, 'ascii')} max delay: {delay}")

    def receive(self, *, hz=10, timeout=datetime.timedelta.max):
        socket = self.socket
        poller = zmq.Poller()
        poller.register(socket, zmq.POLLIN)
        start_time = self.now()
        while True:
            now = self.now()
            if now - start_time > timeout:
                raise RuntimeError("timeout")
            if self.stopping and now - self.stopping > STOPPING_TIMEOUT:
                g_logger.error(f'failed to stop within timeout of {STOPPING_TIMEOUT}, exiting anyway')
                for name, q in self.nodes.items():
                    if len(q) > 0:
                        g_logger.error(f"{name} queue: {len(q)}")
                g_logger.error(f"still running: {self.nodes.keys() - self.stopped}")
                return
            if self.stopping and self.nodes.keys() == self.stopped:
                g_logger.info('all done, stopping')
                return
            obj = dict(poller.poll(1000//hz))
            if socket in obj and obj[socket] == zmq.POLLIN:
                packet = socket.recv_multipart()
                yield (packet[0], *packet[2:]) # drop frame separator from REQ socket

    def listen(self, sender):
        queue = self.nodes[sender]
        if len(queue) == 0:
            self.listening.add(sender)
        else:
            packet = queue.popleft()[1]
            assert packet[0] == sender, (packet[0], sender)
            self.socket.send_multipart(packet)
            self.last_listen[sender] = self.now()

    def publish(self, sender, channel, data):
        dt = self.now()
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

        delay = dt - self.last_listen.get(sender, dt)
        if self.delays.get(sender, datetime.timedelta()) < delay:
            self.delays[sender] = delay
        if delay > ASSERT_QUEUE_DELAY:
            self.report_error(name=sender, delay=delay.total_seconds(), channel=channel)

    def request_stop(self, sender):
        # confirm
        dt_uint32 = osgar.logger.format_timedelta(self.now())
        packet = [sender, b"", b"quit", dt_uint32]
        self.socket.send_multipart(packet)
        assert sender not in self.stopped
        self.stopped.add(sender)
        if self.stopping:
            return
        g_logger.info(f"{str(sender, 'ascii')} requested stop")
        self.stopping = self.now()
        for node_name in self.nodes:
            if node_name != sender:
                self.send(node_name, b"quit", self.stopping)

    def is_alive(self, sender):
        dt_uint32 = osgar.logger.format_timedelta(self.now())
        self.socket.send_multipart([sender, b"", b"quit" if self.stopping else b"is_alive", dt_uint32])

    def send(self, node_name, action, dt, *args):
        dt_uint32 = osgar.logger.format_timedelta(dt)
        packet = [node_name, b"", action, dt_uint32, *args]
        if node_name in self.listening:
            self.listening.remove(node_name)
            self.socket.send_multipart(packet)
            self.last_listen[node_name] = dt
        else:
            self.nodes[node_name].append((dt, packet))

    def now(self):
        return datetime.datetime.now(datetime.timezone.utc) - self.start_time

    def report_error(self, **err):
        self.logger.write(0, bytes(str(err), encoding='ascii'))
        g_logger.error(str(err))


class _Bus:
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
            action, dt, *args = self.sock.recv_multipart()

    def is_alive(self):
        with self.lock:
            self.sock.send_multipart([b"is_alive"])
            resp, timestamp = self.sock.recv_multipart()
        assert resp in [b"is_alive", b"quit"], resp
        if resp == b"quit":
            self._quit()
            return False
        return True

    def sleep(self, duration):
        time.sleep(duration)

    def _quit(self):
        g_logger.info(f"{self.name} received quit")
        return SystemExit()

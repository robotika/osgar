import ast
import collections
import datetime
import inspect
import logging
import struct
import subprocess
import sys


from pprint import pprint
from threading import Thread

import zmq

import osgar.lib.serialize
import osgar.lib.config


def main():
    module_config = ast.literal_eval(sys.argv[1])
    pprint(module_config)
    klass = osgar.lib.config.get_class_by_name(module_config['driver'])
    bus = Bus(module_config['name'])
    instance = klass(config=module_config['init'], bus=bus)
    instance.start()
    print(module_config['name'], "running")
    instance.join()


def record(config, log_prefix, log_filename=None, duration_sec=None):
    router = Router()
    modules = {}
    for module_name, module_config in config['robot']['modules'].items():
        module_config['name'] = module_name
        modules[module_name] = subprocess.Popen([sys.executable, __file__, str(module_config)])

    router.connect(len(modules), config['robot']['links'])
    pprint(router.subscriptions)
    router.run()

    for module in modules.values():
        module.join()


class Router:
    def __init__(self):
        self.start_time = datetime.datetime.now(datetime.timezone.utc)
        context = zmq.Context.instance()
        self.s = context.socket(zmq.ROUTER)
        self.s.bind("tcp://127.0.0.1:8881")
        self.nodes = dict()
        self.subscriptions = dict()
        self.listening = set()
        self.stopping = False

    def connect(self, num, links):
        # wait for all nodes to "register" their outputs
        for _ in range(num):
            packet = self.s.recv_multipart() # TODO handle timeout
            sender, _, action, *args = packet
            assert action == b"register", (sender, action, args)
            assert sender not in self.nodes
            self.nodes[sender] = collections.deque()
        
        for sender, receiver in links:
            sender = sender.encode("ascii")
            receiver = receiver.encode("ascii")
            receiver, input = receiver.split(b'.')
            self.subscriptions.setdefault(sender, []).append((receiver, input))

        # answer all connected nodes that it is ok to run now (connections are set up)
        for node_name in self.nodes:
            self.s.send_multipart([node_name, b"", b"register"])

    def run(self):
        for packet in self.receive():
            sender, action, *args = packet
            getattr(self, action.decode('ascii'))(sender, *args)
        for node_name in self.nodes:
            self.send(node_name, b"quit")

    def receive(self, hz=10):
        socket = self.s
        poller = zmq.Poller()
        poller.register(socket, zmq.POLLIN)
        try:
            while True:
                obj = dict(poller.poll(1000//hz))
                if socket in obj and obj[socket] == zmq.POLLIN:
                    packet = socket.recv_multipart()
                    yield (packet[0], *packet[2:]) # drop frame separator from REQ socket
                elif self.stopping: # timeout && we are stopping
                    return
        except KeyboardInterrupt:
            pass

    def listen(self, sender):
        queue = self.nodes[sender]
        if len(queue) == 0:
            self.listening.add(sender)
        else:
            packet = queue.popleft()
            assert packet[0] == sender, (packet[0], sender)
            self.s.send_multipart(packet)

    def publish(self, sender, channel, data):
        dt = datetime.datetime.now(datetime.timezone.utc) - self.start_time
        assert dt <= datetime.timedelta(microseconds=(1<<32)-1)
        dt_int = (dt.seconds * 1000000 + dt.microseconds)
        dt_uint32 = struct.pack("I", dt_int)
        self.s.send_multipart([sender, b"", b"publish", dt_uint32])
        for node_name, input_channel in self.subscriptions.get(sender+b"."+channel, []):
            self.send(node_name, b'listen', dt_uint32, input_channel, data)
    
    def request_stop(self, sender):
        print(sender.decode('ascii'), "requested stop")
        self.stopping = True

    def send(self, node_name, *args):
        packet = [node_name, b"", *args]
        if node_name in self.listening:
            self.listening.remove(node_name)
            self.s.send_multipart(packet)
        else:
            self.nodes[node_name].append(packet)


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
        assert resp == b'register'

    def listen(self):
        self.sock.send(b"listen")
        resp = self.sock.recv_multipart()
        if resp[0] == b"quit":
            print(f"{self.name} received quit")
            raise SystemExit()
        assert resp[0] == b"listen"
        microseconds = struct.unpack('I', resp[1])[0]
        dt = datetime.timedelta(microseconds=microseconds)
        channel = resp[2].decode('ascii')
        data = osgar.lib.serialize.deserialize(resp[3])
        return dt, channel, data

    def publish(self, channel, data):
        raw = osgar.lib.serialize.serialize(data)
        self.sock.send_multipart([b"publish", channel.encode('ascii'), raw])
        resp = self.sock.recv_multipart()
        assert resp[0] == b"publish"
        microseconds = struct.unpack('I', resp[1])[0]
        dt = datetime.timedelta(microseconds=microseconds)
        return dt

    def request_stop(self):
        self.sock.send_multipart([b"request_stop"])

    def is_alive(self):
        return True


if __name__ == "__main__":
    main()


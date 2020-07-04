"""
  ZeroMQ Thread Pool for handling multiple ROS services
"""

from threading import Thread

import zmq

from osgar.bus import BusShutdownException


class ZMQPool:
    def __init__(self, config, bus):
        bus.register('raw:gz' if config.get('save_data', False) else 'raw:null', 'response', 'timeout')
        mode = config['mode']
        self.endpoint = config['endpoint']
        self.timeout = config.get('timeout', 1)  # default recv timeout 1s

        # support only mode='REQ'
        self.thread = Thread(target=self.run_reqrep)

        self.thread.name = bus.name
        self.bus = bus

    def start(self):
        self.thread.start()

    def join(self, timeout=None):
        self.thread.join(timeout=timeout)
    
    class ReqRepWorker(Thread):
        def __init__(self, context, bus, endpoint):
            Thread.__init__ (self)
            self.context = context
            self.bus = bus

            self.rossocket = context.socket(zmq.REQ)
            self.rossocket.RCVTIMEO = int(2000)
            self.rossocket.connect(endpoint)
            self.stop_requested = False
            
        def stop(self):
            self.stop_requested = True
            
        def run(self):
            worker = self.context.socket(zmq.REQ)
            worker.setsockopt(zmq.LINGER, 0)
            worker.RCVTIMEO = 1000 
            worker.connect('inproc://reqrepbackend')

            def run_loop():
                while not self.stop_requested:
                    try:
                        worker.send(b"ready")
                        while True:
                            try:
                                data = worker.recv()
                                break
                            except zmq.Again:
                                if self.stop_requested:
                                    return

                        ident, msg = data.decode('ascii').split('|')
                        self.rossocket.send_string(msg)
                        rsp = self.rossocket.recv().decode('ascii')
                        self.bus.publish('response', [ident, rsp])
                    except Exception as e:
                        print("logzeromq thread: %s" % str(e))

            run_loop()
            worker.close()
            self.rossocket.close()
            
    def run_reqrep(self):
        context = zmq.Context()
        backend = context.socket(zmq.ROUTER)
        backend.bind('inproc://reqrepbackend')

        workers = []
        for i in range(5):
            worker = self.ReqRepWorker(context, self.bus, self.endpoint)
            worker.start()
            workers.append(worker)

        try:
            while True:
                dt, __, data = self.bus.listen()
                # data is [<ident>, <ROS request payload>]
                ident, payload = data
                address, empty, ready = backend.recv_multipart()                
                backend.send_multipart([address, b'', (ident + '|' + payload).encode('ascii')])

        except BusShutdownException:
            for w in workers:
                w.stop()
                w.join()

        backend.close()
        context.term()

    def request_stop(self):
        self.bus.shutdown()

# vim: expandtab sw=4 ts=4

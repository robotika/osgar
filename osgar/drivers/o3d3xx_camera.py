"""
    Driver for O3D3xx camera, https://www.ifm.com/de/en/product/O3D305
    Implemented via https://github.com/ifm/o3d3xx-python
"""

from threading import Thread
import o3d3xx

class O3DCamera:
    def __init__(self, config, bus):
        self.input_thread = Thread(target=self.run_input, daemon=True)
        self.bus = bus
        self.address = config.get("address", "192.168.0.69")
        bus.register('raw')


    def start(self):
        self.input_thread.start()

    def join(self, timeout=None):
        self.input_thread.join(timeout=timeout)

    def run_input(self):
        while self.bus.is_alive():
            # Capture frame-by-frame

            #self.bus.publish('raw', data.tobytes())


    def request_stop(self):
        self.bus.shutdown()

# vim: expandtab sw=4 ts=4

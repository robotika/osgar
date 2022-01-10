"""
    Driver for O3D3xx camera, https://www.ifm.com/de/en/product/O3D305
    Implemented via https://github.com/ifm/o3d3xx-python
"""

from threading import Thread
import logging
import numpy as np

g_logger = logging.getLogger(__name__)
try:
    import o3d3xx
except:
    g_logger.error("o3d3xx is not installed!")

class O3DCamera:
    def __init__(self, config, bus):
        self.input_thread = Thread(target=self.run_input, daemon=True)
        self.bus = bus
        self.address = config.get("address", "192.168.0.69")
        self.bus.register('depth', 'amplitude_image')
        self.cam = o3d3xx.FormatClient(self.address, 50010, o3d3xx.PCICFormat.blobs("amplitude_image", "distance_image"))


    def start(self):
        self.input_thread.start()

    def join(self, timeout=None):
        self.input_thread.join(timeout=timeout)

    def run_input(self):
        while self.bus.is_alive():
            # Capture frame-by-frame
            data = self.cam.readNextFrame()
            assert "distance_image" in data
            assert "amplitude_image" in data
            depth = np.frombuffer(data['distance_image'],dtype='uint16').reshape(132,176)
            amplitude_image = np.frombuffer(data['amplitude_image'],dtype='uint16').reshape(132,176)
            self.bus.publish("depth", depth)
            self.bus.publish("amplitude_image", amplitude_image)
            # for name in data:
            #     print(name)
            # print("---------------")

    def request_stop(self):
        self.bus.shutdown()

# vim: expandtab sw=4 ts=4

"""
  Publish jpg images captured using libuvc (pupil-labs fork).
  Only cameras directly supporting jpg output are supported.
"""

from threading import Thread
import sys

import uvc

from osgar.bus import BusShutdownException


class UsbCam:
    def __init__(self, config, bus):
        self.input_thread = Thread(target=self.run_input, daemon=True)
        self.bus = bus

        bus = config['bus']
        port = config['port']
        # to find proper bus and port number run
        # python -c "import uvc; from pprint import pprint; pprint(uvc.device_list())"

        self.sleep = config.get('sleep', 0.1)
        self.cap = None
        
        dev_list = uvc.device_list()
        for dev in dev_list:
            if dev['bus_number'] == bus and dev['bus_ports'][0] == port:
                self.cap = uvc.Capture(dev["uid"])
                self.cap.frame_mode = self.cap.avaible_modes[0]
                self.cap.bandwidth_factor = 1.2
                # when incomplete pictures are being received, increase bandwidth_factor
                # when "no space left on device" error, decrease bandwidth_factor
                # for more info see https://docs.pupil-labs.com/#jpeg-size-estimation-and-custom-video-backends
                return

        error = "camera not found on bus %i and port %i" % (bus, port)
        self.bus.report_error(error)
        print(error, file=sys.stderr)

    def start(self):
        self.input_thread.start()

    def join(self, timeout=None):
        self.input_thread.join(timeout=timeout)

    def run_input(self):
        if self.cap is None:
            return
        while self.bus.is_alive():
            frame = self.cap.get_frame_robust()
            self.bus.publish('jpg', bytes(frame.jpeg_buffer))
            self.bus.sleep(self.sleep)
        self.cap.close()

    def request_stop(self):
        self.bus.shutdown()

# vim: expandtab sw=4 ts=4

"""
  Log video stream provided by OpenCV camera
"""

import cv2
from threading import Thread

from osgar.logger import LogWriter
from osgar.bus import BusShutdownException


class LogOpenCVCamera:
    def __init__(self, config, bus):
        self.input_thread = Thread(target=self.run_input, daemon=True)
        self.bus = bus

        port = config.get('port', 0)
        self.cap = bus.logged(cv2.VideoCapture, port)
        self.sleep = config.get('sleep')

    def start(self):
        self.input_thread.start()

    def join(self, timeout=None):
        self.input_thread.join(timeout=timeout)

    def run_input(self):
        while self.bus.is_alive():
            # Capture frame-by-frame
            ret, frame = self.cap.read()
            print(ret)
            if ret:
                retval, data = cv2.imencode('*.jpeg', frame)
                print(len(data))
                if len(data) > 0:
                    self.bus.publish('raw', data.tobytes())
                if self.sleep is not None:
                    self.bus.sleep(self.sleep)
        self.cap.release()

    def request_stop(self):
        self.bus.shutdown()

# vim: expandtab sw=4 ts=4

import sys
sys.path.append("/home/jakub/git/apriltag/build/")  # TODO integrate to system
import apriltag
import cv2
import numpy as np
from threading import Thread

from osgar.bus import BusShutdownException
from osgar.node import Node


class Apriltag(Node):

    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("tag")

        self.thread = Thread(target=self.run)
        #self.thread.name = bus.name

    def start(self):
        self.thread.start()

    def join(self, timeout=None):
        self.thread.join(timeout=timeout)

    def run(self):
        print("HI")
        detector = apriltag.apriltag('tag16h5', threads=1)
        try:
            print("bus?")
            while True:
                print("chanel??")
                dt, channel, data = self.listen()
                print(channel)
                if channel == "image":
                    img = np.frombuffer(data, dtype=np.uint8)
                    gray = cv2.imdecode(img, cv2.IMREAD_GRAYSCALE)
                    print(gray)
                    found = detector.detect(gray)
                    found = [tag for tag in found if tag['margin'] > 30 and tag['hamming'] == 0]
                    if len(found) > 0:
                        print(found)
        except BusShutdownException:
            print("exc")
            pass

    def request_stop(self):
        self.bus.shutdown()

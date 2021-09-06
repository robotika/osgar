"""
  Jetson artf detector wrapper
"""
import os.path
from io import StringIO

import cv2
import numpy as np
from subt.tf_detector import CvDetector

from osgar.node import Node

def result2report(result):
    return 1


class ArtifactDetectorJetson(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("localized_artf", "dropped", "debug_image", "debug_result")
        self.time = None
        self.width = None
        self.detector = CvDetector().subt_detector

    def wait_for_data(self):
        while True:
            self.time, channel, data = self.listen()
            if channel == "image":
                return self.time, data

    def run(self):
        dropped = 0
        while True:
            now = self.publish("dropped", dropped)
            dropped = -1
            timestamp = now
            while timestamp <= now:
                timestamp, img_data = self.wait_for_data()
                dropped += 1
                img = cv2.imdecode(np.frombuffer(img_data, dtype=np.uint8), cv2.IMREAD_COLOR)
                self.detect(img)
                timestamp, channel = self.wait_for_data()

    def detect(self, img):
        if self.width is None:
            self.width = img.shape[1]
        assert self.width == img.shape[1], (self.width, img.shape[1])

        result = self.detector(img)
        if result:
            print(result)
            report = result2report(result)
            if report is not None:
                self.publish('localized_artf', report)
                self.publish('debug_image', img)

        return result
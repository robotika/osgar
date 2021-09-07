"""
  Jetson artf detector wrapper
"""
import os.path
from io import StringIO

import cv2
import numpy as np
from subt.tf_detector import CvDetector

from osgar.node import Node
from subt.artf_node import result2report
from osgar.bus import BusShutdownException


class ArtifactDetectorJetson(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("localized_artf", "dropped", "debug_image", "debug_result")
        self.time = None
        self.width = None
        self.height = None
        self.camera_pose = config.get("camera_pose", ([0, 0, 0], [0, 0, 0, 1]))
        self.fx = config.get("fx", 149.01)
        self.detector = CvDetector().subt_detector

    def wait_for_data(self):
        while True:
            self.time, channel, data = self.listen()
            if channel == "image":
                return self.time, data

    def run(self):
        try:
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
        except BusShutdownException:
            pass

    def detect(self, img):
        if self.width is None:
            self.height, self.width = img.shape[:2]
        assert self.width == img.shape[1], (self.width, img.shape[1])
        assert self.height == img.shape[0], (self.height, img.shape[0])

        result = self.detector(img)
        if result:
            print(result)
            for res in result:
                dist = 2  # There is no source of the artf dist in this moment so just put some number
                report = result2report(res, (dist, self.width, self.height), self.fx, ([0, 0, 0], [0, 0, 0, 1]),
                                            self.camera_pose, 10)  # TODO real camera_pose and robot_pose
                if report is not None:
                    print(report)
                    self.publish('localized_artf', report)
                    self.publish('debug_image', img)

        return result
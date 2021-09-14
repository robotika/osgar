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
        self.camera_pose = config.get("camera_pose", ([0, 0, 0], [0, 0, 0, 1]))
        self.fx = config.get("fx", 149.01)
        self.detector = CvDetector(1).subt_detector
        self.last_pose3d = None

    def wait_for_data(self):
        while True:
            self.time, channel, data = self.listen()
            if channel == "pose3d":
                self.last_pose3d = data
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
                if self.last_pose3d:
                    self.detect(img_data)
                timestamp, img_data = self.wait_for_data()
        except BusShutdownException:
            pass

    def detect(self, img_data):
        img = cv2.imdecode(np.frombuffer(img_data, dtype=np.uint8), cv2.IMREAD_COLOR)
        result = self.detector(img)
        if result:
            self.publish('debug_result', result)
            for res in result:
                artf_name, score, (x0, y0, x1, y1) = res
                xc = int(round((x0+x1)/2))
                yc = int(round((y0+y1)/2))
                res_for_report = [(artf_name, [(xc, yc, score)])]  # convert to mdnet compatible format
                fake_depth = np.ones(img.shape[:2]) * 1  # There is no source of the artf dist in this moment so just put some number
                report = result2report(res_for_report, fake_depth, self.fx, self.last_pose3d, self.camera_pose, 10)
                if report is not None:
                    self.publish('localized_artf', report)
                    self.publish('debug_image', img_data)

        return result

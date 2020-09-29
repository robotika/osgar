"""
  OSGAR ArtifactDetectorDNN wrapper for DNN detector
"""
import os.path
from io import StringIO

import cv2
import numpy as np
from subt.tf_detector import CvDetector
try:
    import torch
    import subt.artf_model
    from subt.artf_detector import Detector
except ImportError:
    print('\nWarning: missing torch!\n')

from osgar.node import Node
from osgar.bus import BusShutdownException
from subt.artifacts import (RESCUE_RANDY, BACKPACK, PHONE, HELMET, ROPE)


NAME2IGN = {
    'survivor': RESCUE_RANDY,
    'backpack': BACKPACK,
    'phone': PHONE,
    'helmet': HELMET,
    'rope': ROPE
}


def result2report(result, depth):
    width = depth.shape[1]
    x_arr = [x for x, y, certainty in result[0][1]]  # ignore multiple objects
    dist = [depth[y][x] for x, y, certainty in result[0][1]]  # ignore multiple objects
    x_min, x_max = min(x_arr), max(x_arr)
    deg_100th = int(round(100 * 69.4 * (width/2 - (x_min + x_max)/2)/width))
    if 0xFFFF in dist:
        return None  # out of range
    return [NAME2IGN[result[0][0]], deg_100th, int(np.median(dist))]


class ArtifactDetectorDNN(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("artf", "dropped", "debug_artf", "stdout")
        self.time = None
        self.width = None  # not sure if we will need it
        self.depth = None  # more precise artifact position from depth image
        self.cv_detector = CvDetector().subt_detector
        self.detector = self.create_detector()

    def create_detector(self):
        model = os.path.join(os.path.dirname(__file__), '../../../mdnet0.64.64.13.4.relu.pth')
        confidence_thresholds = {
            'survivor': 0.95,
            'backpack': 0.977,
            'phone': 0.97,
            'helmet': 0.963,
            'rope': 0.99
        }
        max_gap = 16
        min_group_size = 2

        use_cuda = torch.cuda.is_available()
        device = torch.device("cuda" if use_cuda else "cpu")
        print('Using:', device)
        model, categories = subt.artf_model.load_model(model, device)
        return Detector(model, confidence_thresholds, categories, device,
                        max_gap, min_group_size)

    def wait_for_image(self):
        channel = ""
        while channel != "image":
            self.time, channel, data = self.listen()
            setattr(self, channel, data)
        return self.time

    def stdout(self, *args, **kwargs):
        # maybe refactor to Node?
        output = StringIO()
        print(*args, file=output, **kwargs)
        contents = output.getvalue().strip()
        output.close()
        self.publish('stdout', contents)
        print(self.time, contents)

    def run(self):
        try:
            dropped = 0
            while True:
                now = self.publish("dropped", dropped)
                dropped = -1
                timestamp = now
                while timestamp <= now:
                    timestamp = self.wait_for_image()
                    dropped += 1
                self.detect(self.image)
        except BusShutdownException:
            pass

    def detect(self, image):
        img = cv2.imdecode(np.fromstring(image, dtype=np.uint8), 1)
        if self.width is None:
            self.stdout('Image resolution', img.shape)
            self.width = img.shape[1]
        assert self.width == img.shape[1], (self.width, img.shape[1])

        result = self.detector(img)
        result_cv = self.cv_detector(img)
        if len(result) > 0 and len(result_cv) > 0:
            if result[0][0] == result_cv[0][0]: # check artefacts names
                self.stdout(result, result_cv)
                report = result2report(result, self.depth)
                if report is not None:
                    self.publish('artf', report)
                    self.publish('debug_artf', image)  # JPEG

# vim: expandtab sw=4 ts=4

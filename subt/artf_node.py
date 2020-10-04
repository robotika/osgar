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


def check_results(result, result_cv):
    ret = []
    if len(result) == 0 or len(result_cv) == 0:
        return ret
    for r_cv in result_cv:
        name_cv, score_cv, bbox = r_cv
        bbox_x1, bbox_y1, bbox_x2, bbox_y2 = bbox
        for r in result:
            name, points = r
            if name != name_cv:
                continue
            x = np.array([p[0] for p in points])
            y = np.array([p[1] for p in points])
            x_in_bbox = (x > bbox_x1) & (x < bbox_x2)  # arr of boolean values
            y_in_bbox = (y > bbox_y1) & (y < bbox_y2)  # arr of boolean values
            if np.any(x_in_bbox & y_in_bbox):  # at least one point is in the bbox
                ret.append(r)
                result.remove(r)
    return ret


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
        bus.register("artf", "dropped", "debug_artf", "debug_depth:gz", "stdout")
        self.time = None
        self.width = None  # not sure if we will need it
        self.depth = None  # more precise artifact position from depth image
        self.cv_detector = CvDetector().subt_detector
        self.detector = self.create_detector()

    def create_detector(self):
        model = os.path.join(os.path.dirname(__file__), '../../../mdnet1.64.64.13.4.relu.pth')
        confidence_thresholds = {
            'survivor': 0.92,
            'backpack': 0.9988,
            'phone': 0.87,
            'helmet': 0.5,
            'rope': 0.83
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
        if result or result_cv:
            self.stdout(result, result_cv)  # publish the results independent to detection validity
            checked_result = check_results(result, result_cv)
            if checked_result:
                report = result2report(checked_result, self.depth)
                if report is not None:
                    self.publish('artf', report)
                    self.publish('debug_artf', image)  # JPEG
                    self.publish('debug_depth', self.depth)

# vim: expandtab sw=4 ts=4

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


def check_results(result_mdnet, result_cv):
    result = result_mdnet.copy()
    ret = []
    if len(result) == 0 or len(result_cv) == 0:
        return ret
    for r_cv in result_cv:
        name_cv, score_cv, bbox = r_cv
        bbox_x1, bbox_y1, bbox_x2, bbox_y2 = bbox
        ret_points = []
        for r in result.copy():
            name, points = r
            if name != name_cv:
                continue
            x = np.array([p[0] for p in points])
            y = np.array([p[1] for p in points])
            x_in_bbox = np.logical_and((x > bbox_x1), (x < bbox_x2))
            y_in_bbox = np.logical_and((y > bbox_y1), (y < bbox_y2))
            if np.any(np.logical_and(x_in_bbox, y_in_bbox)):  # at least one point is in the bbox
                ret_points.extend(points)
                result.remove(r)
        if ret_points:
            ret.append((name_cv, ret_points))
    return ret


def result2report(result, depth, fx):
    # return relative XYZ distances to camera
    width = depth.shape[1]
    height = depth.shape[0]
    x_arr = [x for x, y, certainty in result[0][1]]  # ignore multiple objects
    y_arr = [y for x, y, certainty in result[0][1]]  # ignore multiple objects
    dist = [depth[y][x] for x, y, certainty in result[0][1]]  # ignore multiple objects
    if 0xFFFF in dist:
        return None  # out of range
    x_min, x_max = min(x_arr), max(x_arr)
    y_min, y_max = min(y_arr), max(y_arr)
    scale = np.median(dist)
    rel_x = int(scale)  # relative X-coordinate in front
    rel_y = int(scale * (width/2 - (x_min + x_max)/2)/fx)  # Y-coordinate is to the left
    rel_z = int(scale * (height/2 - (y_min + y_max)/2)/fx)  # Z-up
    return [NAME2IGN[result[0][0]], [rel_x, rel_y, rel_z]]


def result2list(result):
    """
    convert MDNET result to msgpack friendly format
    """
    ret = []
    for artifact, points in result:
        ret.append([artifact, [list(p) for p in points]])
    return ret


def cv_result2list(result):
    """
    convert OpenCV DNN detector result to msgpack friendly format
    """
    ret = []
    for artifact, certainty, bbox in result:
        ret.append([artifact, certainty, bbox.tolist()])
    return ret


class ArtifactDetectorDNN(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("artf", "dropped", "debug_artf", "debug_depth:gz", "stdout",
                     "debug_result", "debug_cv_result")
        self.time = None
        self.width = None  # not sure if we will need it
        self.depth = None  # more precise artifact position from depth image
        self.cv_detector = CvDetector().subt_detector
        self.detector = self.create_detector()
        self.fx = config.get('fx', 554.25469)  # use drone X4 for testing (TODO update all configs!)

    def create_detector(self):
        model = os.path.join(os.path.dirname(__file__), '../../../mdnet1.64.64.13.4.relu.pth')
        confidence_thresholds = {
            'survivor': 0.8,
            'backpack': 0.95,
            'phone': 0.77,
            'helmet': 0.9,
            'rope': 0.8
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
            # publish the results independent to detection validity
            self.stdout(result, result_cv)  # for debugging in case of crash
            self.publish('debug_result', str(result))
            self.publish('debug_cv_result', str(result_cv))
            checked_result = check_results(result, result_cv)
            if checked_result:
                report = result2report(checked_result, self.depth, self.fx)
                if report is not None:
                    self.publish('artf', report)
                    self.publish('debug_artf', image)  # JPEG
                    self.publish('debug_depth', self.depth)


if __name__ == "__main__":
    # run "replay" without calling detections - only XYZ offset check
    import argparse
    from datetime import timedelta

    from numpy import array, int32  # workaround for str() serialized numpy array

    from osgar.lib.serialize import deserialize
    from osgar.logger import LogReader, lookup_stream_id, lookup_stream_names
    from ast import literal_eval

    parser = argparse.ArgumentParser(description='Test 3D reports')
    parser.add_argument('logfile', help='OSGAR logfile')
    parser.add_argument('--time-limit-sec', '-t', help='cut time in seconds', type=float)
    parser.add_argument('--verbose', '-v', help="verbose mode", action='store_true')
    args = parser.parse_args()

    names = lookup_stream_names(args.logfile)
    assert 'detector.artf' in names, names  # XYZ relative offsets
    assert 'detector.debug_depth' in names, names
    assert 'detector.debug_result' in names, names
    assert 'detector.debug_cv_result' in names, names

    artf_stream_id = names.index('detector.artf') + 1
    depth_stream_id = names.index('detector.debug_depth') + 1
    result_id = names.index('detector.debug_result') + 1
    cv_result_id = names.index('detector.debug_cv_result') + 1

    # read config file from log
    with LogReader(args.logfile, only_stream_id=0) as log:
        print("original args:", next(log)[-1])  # old arguments
        config_str = next(log)[-1]
        config = literal_eval(config_str.decode('ascii'))

    assert 'detector' in config['robot']['modules']
    fx = config['robot']['modules']['detector']['init']['fx']

    depth = None
    last_artf = None  # reported before debug_depth
    last_result = None
    last_cv_result = None
    with LogReader(args.logfile,
                   only_stream_id=[artf_stream_id, depth_stream_id, result_id, cv_result_id]) as logreader:
        for time, stream, msg_data in logreader:
            if args.time_limit_sec is not None and time.total_seconds() > args.time_limit_sec:
                break
            data = deserialize(msg_data)

            if stream == depth_stream_id:
                depth = data
                # debug_depth is stored ONLY when both detector detect something and it is fused
                assert last_result is not None
                assert last_cv_result is not None

                checked_result = check_results(last_result, last_cv_result)
                assert checked_result  # the debug depth is stored, so there should valid report
                report = result2report(checked_result, depth, fx)
                if args.verbose:
                    print(report)
                assert last_artf == report, (last_artf, report)

            elif stream in [result_id, cv_result_id]:
                if args.verbose:
                    print(time, data)
#                arr = literal_eval(data)
                arr = eval(data)  # workaround for str() serialized numpy array

                if stream == result_id:
                    last_result = arr
                elif stream == cv_result_id:
                    last_cv_result = arr
                else:
                    assert False, stream

            elif stream == artf_stream_id:
                if args.verbose:
                    print(time, 'Original report:', data)
                last_artf = data
                assert last_artf is not None, time
            else:
                assert False, stream  # unexpected stream

# vim: expandtab sw=4 ts=4

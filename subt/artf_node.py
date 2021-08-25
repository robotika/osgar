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
from osgar.lib.depth import decompress as decompress_depth
from osgar.lib.quaternion import rotate_vector, rotation_matrix
from subt.artifacts import (RESCUE_RANDY, BACKPACK, PHONE, HELMET, ROPE, EXTINGUISHER, DRILL, VENT, CUBE)


NAME2IGN = {
    'survivor': RESCUE_RANDY,
    'backpack': BACKPACK,
    'phone': PHONE,
    'helmet': HELMET,
    'rope': ROPE,
    'fire_extinguisher': EXTINGUISHER,
    'drill': DRILL,
    'vent': VENT,
    'cube' : CUBE
}


def check_borders(result, borders):
    for ii, (name, points, r_cv) in enumerate(result.copy()):
        points.sort(key=lambda item: item[2], reverse=True)
        x = points[1][2]  # mdnet score
        y = r_cv[1]  # cv_detector score
        a1, b1, a2, b2 = borders[name]  # coefficients of lines equations - borders
        if y < min(a1 * x + b1, a2 * x + b2):  # the value is below the borders
            result.pop(ii)

        return result


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
            ret.append((name_cv, ret_points, r_cv))
    return ret


def as_matrix(translation, rotation):
    m = np.eye(4)
    m[:3,:3] = rotation_matrix(rotation)
    m[:3,3] = translation
    return m


def transform(transformation, xyz):
    shift, rotation = transformation
    rotated = rotate_vector(xyz, rotation)
    return [sum(v) for v in zip(rotated, shift)]


def result2report(result, depth, fx, robot_pose, camera_pose, max_depth):
    """return relative XYZ distances to camera"""
    if depth is None:
        return None  # ignore detected artifacts for missing depth data
                     # typically some glitch on start
    width = depth.shape[1]
    height = depth.shape[0]
    x_arr = [x for x, y, certainty in result[0][1]]  # ignore multiple objects
    y_arr = [y for x, y, certainty in result[0][1]]  # ignore multiple objects
    dist = [depth[y][x] for x, y, certainty in result[0][1]]  # ignore multiple objects
    if any(d == 0 or d > max_depth for d in dist):
        return None  # out of range
    x_min, x_max = min(x_arr), max(x_arr)
    y_min, y_max = min(y_arr), max(y_arr)
    scale = np.median(dist)
    # Coordinate of the artifact relative to the camera.
    camera_rel = [scale,  # relative X-coordinate in front
                  scale * (width/2 - (x_min + x_max)/2)/fx,  # Y-coordinate is to the left
                  scale * (height/2 - (y_min + y_max)/2)/fx]  # Z-up
    # Coordinate of the artifact relative to the robot.
    robot_rel = transform(camera_pose, camera_rel)
    # Global coordinate of the artifact.
    world_xyz = transform(robot_pose, robot_rel)
    return [NAME2IGN[result[0][0]], world_xyz]


def get_border_lines(border_points):
    ret = {}
    for name, points in border_points.items():
        A, B, C = points
        a1 = (B[1] - A[1]) / (B[0] - A[0])  # slope of the first line
        a2 = (C[1] - B[1]) / (C[0] - B[0])  # slope of the second line
        b1 = B[1] - a1 * B[0]  # the first intercept
        b2 = B[1] - a2 * B[0]  # the second intercept
        ret[name] = [a1, b1, a2, b2]

    return ret


def create_detector(confidence_thresholds):
    model = os.path.join(os.path.dirname(__file__), '../../../mdnet6.128.128.13.4.elu.pth')
    max_gap = 16
    min_group_size = 2

    use_cuda = torch.cuda.is_available()
    device = torch.device("cuda" if use_cuda else "cpu")
    print('Using:', device)
    model, categories = subt.artf_model.load_model(model, device)
    return Detector(model, confidence_thresholds, categories, device,
                    max_gap, min_group_size)

class ArtifactDetectorDNN(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("localized_artf", "dropped", "debug_rgbd", "stdout",
                     "debug_result", "debug_cv_result", "debug_camera")
        confidence_thresholds = {  # used for mdnet
            'survivor': 0.5,
            'backpack': 0.7,
            'phone': 0.5,
            'helmet': 0.5,
            'rope': 0.5,
            'fire_extinguisher': 0.5,
            'drill': 0.5,
            'vent': 0.5,
            'cube': 0.5
        }
        # Confidence borders points
        # There are tree border points for each artifact, point coordinates: x - mdnet, y - cv_detector
        confidence_borders = {
            'survivor': [[0.5, 1],[0.85, 0.7],[1, 0.1]],
            'backpack': [[0.7, 1],[0.8, 0.8],[0.95, 0.2]],
            'phone': [[0.5, 1],[0.7, 0.7],[1, 0.1]],
            'helmet': [[0.5, 1],[0.82, 0.9],[95, 0.2]],
            'rope': [[0.5, 0.7],[0.85, 0.65],[1, 0.1]],
            'fire_extinguisher': [[0.5, 1],[0.9, 0.975],[1, 0.7]],
            'drill': [[0.5, 1],[0.9, 0.75],[1, 0.5]],
            'vent': [[0.5, 0.2],[0.8, 0.18],[1, 0.1]],
            'cube': [[0.5, 0.75],[0.85, 0.72],[1, 0.1]]
        }
        self.border_lines = get_border_lines(confidence_borders)
        self.time = None
        self.width = None  # not sure if we will need it
        self.depth = None  # more precise artifact position from depth image
        self.cv_detector = CvDetector().subt_detector
        self.detector = create_detector(confidence_thresholds)
        self.fx = config.get('fx', 554.25469)  # use drone X4 for testing (TODO update all configs!)
        self.max_depth = config.get('max_depth', 10.0)
        self.triangulation_baseline_min = config.get('triangulation_baseline_min', 0.03)
        self.triangulation_baseline_max = config.get('triangulation_baseline_max', 0.20)
        self.batch_size = config.get('batch_size', 1)  # how many images process in one step
        self.prev_camera = {}

    def wait_for_data(self):
        channel = ""
        while channel != "rgbd" and not channel.startswith("camera"):
            self.time, channel, data = self.listen()
            setattr(self, channel, data)
        return self.time, channel

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
            self.stdout(cv2.dnn.getAvailableTargets(cv2.dnn.DNN_BACKEND_CUDA))
            dropped = 0
            while True:
                now = self.publish("dropped", dropped)
                dropped = -1
                timestamp = now
                while timestamp <= now:
                    timestamp, channel = self.wait_for_data()
                    dropped += 1
                for count in range(self.batch_size):
                    if channel == 'rgbd':
                        self.detect_from_rgbd(self.rgbd)
                    else:
                        self.detect_from_img(channel, getattr(self, channel))
                    if count + 1 < self.batch_size:
                        # process also immediately following images
                        timestamp, channel = self.wait_for_data()
        except BusShutdownException:
            pass

    def detect(self, img):
        if self.width is None:
            self.stdout('Image resolution', img.shape)
            self.width = img.shape[1]
        assert self.width == img.shape[1], (self.width, img.shape[1])

        result = self.detector(img)
        result_cv = self.cv_detector(img)
        checked_result = None
        if result or result_cv:
            # publish the results independent to detection validity
            self.publish('debug_result', result)
            self.publish('debug_cv_result', result_cv)
            checked_result = check_results(result, result_cv)
            if checked_result:
                checked_result = check_borders(checked_result, self.border_lines)
        return checked_result

    def detect_from_rgbd(self, rgbd):
        robot_pose, camera_pose, image_data, depth_data = rgbd
        img = cv2.imdecode(np.fromstring(image_data, dtype=np.uint8), cv2.IMREAD_COLOR)
        depth = decompress_depth(depth_data)
        checked_result = self.detect(img)
        if checked_result:
            report = result2report(checked_result, depth, self.fx,
                    robot_pose, camera_pose, self.max_depth)
            if report is not None:
                self.publish('localized_artf', report)
                self.publish('debug_rgbd', rgbd)

    def detect_from_img(self, camera_name, data):
        curr_robot_pose, curr_camera_pose, curr_img_data = data
        curr_img = cv2.imdecode(np.fromstring(curr_img_data, dtype=np.uint8), cv2.IMREAD_COLOR)
        curr_img_gray = cv2.cvtColor(curr_img, cv2.COLOR_BGR2GRAY)
        # If we saw an artifact in the previous image, we need to estimate its
        # location.
        prev_detection = self.prev_camera.get(camera_name)
        if prev_detection is not None:
            prev_robot_pose, prev_camera_pose, artf_name, prev_uvs, prev_img_data, prev_img_gray, checked_result = prev_detection
            curr_uvs, status, err = cv2.calcOpticalFlowPyrLK(prev_img_gray, curr_img_gray, prev_uvs.astype(np.float32), None)
            num_tracked = np.sum(status)
            if num_tracked > 0:
                status = status[:,0].astype(np.bool)
                curr_uvs = curr_uvs[status]
                prev_uvs = prev_uvs[status]
                assert(curr_uvs.shape == prev_uvs.shape)

                prev_to_global = as_matrix(*prev_robot_pose) @ as_matrix(*prev_camera_pose)
                curr_to_local = np.linalg.inv(as_matrix(*curr_robot_pose) @ as_matrix(*curr_camera_pose))
                TO_OPTICAL = np.array([[ 0, -1,  0, 0],
                                       [ 0,  0, -1, 0],
                                       [ 1,  0,  0, 0],
                                       [ 0,  0,  0, 1]], dtype=np.float)
                FROM_OPTICAL = TO_OPTICAL.T  # inverse
                # projection_matrix = camera_matrix @ camera_pose
                # https://stackoverflow.com/questions/16101747/how-can-i-get-the-camera-projection-matrix-out-of-calibratecamera-return-value
                # https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
                # We calculate everything in the previous coordinate frame of the camera.
                cx = (curr_img.shape[1] - 1) / 2
                cy = (curr_img.shape[0] - 1) / 2
                fx = fy = self.fx
                camera_matrix = np.array([[fx,  0, cx],
                                          [ 0, fy, cy],
                                          [ 0,  0,  1]])
                prev_projection_matrix = camera_matrix @ np.eye(3, 4)
                to_curr_camera = curr_to_local @ prev_to_global
                curr_projection_matrix = camera_matrix @ (TO_OPTICAL @ to_curr_camera @ FROM_OPTICAL)[:3,:]
                traveled_dist = np.linalg.norm(to_curr_camera[:3,3])
                if traveled_dist < self.triangulation_baseline_min:
                    # Let's keep the previous detection for triangulation from a more distant point.
                    return
                elif traveled_dist <= self.triangulation_baseline_max:
                    points3d = cv2.triangulatePoints(prev_projection_matrix.astype(np.float64), curr_projection_matrix.astype(np.float64), prev_uvs.T.astype(np.float64), curr_uvs.T.astype(np.float64)).T
                    points3d = cv2.convertPointsFromHomogeneous(points3d)
                    points3d = points3d[:,0,:]  # Getting rid of the unnecessary extra dimension in the middle.
                    fake_depth = np.zeros(curr_img.shape[:2])
                    fake_depth[prev_uvs[:,1], prev_uvs[:,0]] = points3d[:,2]  # Depth is the last dimension in an optical coordinate frame.
                    report = result2report(checked_result, fake_depth, self.fx,
                            prev_robot_pose, prev_camera_pose, self.max_depth)
                    if report is not None:
                        self.publish('localized_artf', report)
                        self.publish('debug_camera', [camera_name, [prev_robot_pose, prev_camera_pose, prev_img_data], [curr_robot_pose, curr_camera_pose, curr_img_data]])
                # else: The robot got too far from the detection point.

                del self.prev_camera[camera_name]

        # Detect artifacts in the current image.
        checked_result = self.detect(curr_img)
        if checked_result:
            # TODO: Consider remembering all blobs and not just the first one.
            artf_name = checked_result[0][0]
            uvs = np.asarray([point[:2] for point in checked_result[0][1]])
            self.prev_camera[camera_name] = curr_robot_pose, curr_camera_pose, artf_name, uvs, curr_img_data, curr_img_gray, checked_result


if __name__ == "__main__":
    # run "replay" without calling detections - only XYZ offset check
    import argparse
    from datetime import timedelta

    from osgar.lib.serialize import deserialize
    from osgar.logger import LogReader, lookup_stream_id, lookup_stream_names
    from ast import literal_eval

    parser = argparse.ArgumentParser(description='Test 3D reports')
    parser.add_argument('logfile', help='OSGAR logfile')
    parser.add_argument('--time-limit-sec', '-t', help='cut time in seconds', type=float)
    parser.add_argument('--verbose', '-v', help="verbose mode", action='store_true')
    parser.add_argument('--module-name', '-m', help='name of the detector module in the log', default='detector')
    args = parser.parse_args()

    names = lookup_stream_names(args.logfile)
    assert 'detector.localized_artf' in names, names  # XYZ world coordinates
    assert 'detector.debug_rgbd' in names, names
    assert 'detector.debug_result' in names, names
    assert 'detector.debug_cv_result' in names, names

    artf_stream_id = names.index('detector.localized_artf') + 1
    rgbd_stream_id = names.index('detector.debug_rgbd') + 1
    result_id = names.index('detector.debug_result') + 1
    cv_result_id = names.index('detector.debug_cv_result') + 1

    # read config file from log
    with LogReader(args.logfile, only_stream_id=0) as log:
        print("original args:", next(log)[-1])  # old arguments
        config_str = next(log)[-1]
        config = literal_eval(config_str.decode('ascii'))

    assert 'detector' in config['robot']['modules']
    fx = config['robot']['modules'][args.module_name]['init']['fx']
    max_depth = config['robot']['modules'][args.module_name]['init'].get('max_depth', 10.0)

    last_artf = None  # reported before debug_rgbd
    last_result = None
    last_cv_result = None
    with LogReader(args.logfile,
                   only_stream_id=[artf_stream_id, rgbd_stream_id, result_id, cv_result_id]) as logreader:
        for time, stream, msg_data in logreader:
            if args.time_limit_sec is not None and time.total_seconds() > args.time_limit_sec:
                break
            data = deserialize(msg_data)

            if stream == rgbd_stream_id:
                robot_pose, camera_pose, __rgb, depth = data
                # debug_rgbd is stored ONLY when both detectors detect something and it is fused
                assert last_result is not None
                assert last_cv_result is not None

                checked_result = check_results(last_result, last_cv_result)
                assert checked_result  # the debug rgbd is stored, so there should be a valid report
                report = result2report(checked_result, decompress_depth(depth),
                        fx, robot_pose, camera_pose, max_depth)
                if args.verbose:
                    print(report)
                assert last_artf == report, (last_artf, report)

            elif stream in [result_id, cv_result_id]:
                if args.verbose:
                    print(time, data)
                if stream == result_id:
                    last_result = data
                elif stream == cv_result_id:
                    last_cv_result = data
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

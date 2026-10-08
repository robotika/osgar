"""
  AprilTag processing Node
"""

import math

import av
import cv2
import numpy as np

from osgar.node import Node


class AprilTag(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        apriltag_type = config.get('type', '25h9')
        self.dict_apriltag = {
            '16h5': cv2.aruco.DICT_APRILTAG_16h5,
            '25h9': cv2.aruco.DICT_APRILTAG_25h9,
        }[apriltag_type]
        bus.register('apriltags', 'targets')
        self.codec = av.CodecContext.create('hevc', 'r')  # h265

    def detect_april_tags(self, image):
        dictionary = cv2.aruco.getPredefinedDictionary(self.dict_apriltag)
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(image)
        if markerCorners is None or markerIds is None:
            return [[], []]
        assert len(markerCorners) == len(markerIds), (markerCorners, markerIds)
        return [[int(x[0]) for x in markerIds],
                [[[int(a), int(b)] for a, b in x[0]] for x in markerCorners]]

    def corners_to_dist(self, corners):
        center_x = sum([x for x, _ in corners])/4.0
        center_y = sum([y for _, y in corners])/4.0
        size = sum([math.hypot(x - center_x, y - center_y) for x, y in corners])/4.0
        return 1.3 * 35.0/size  # GG factor

    def corners_to_angle(self, corners):
        center_x = sum([x for x, _ in corners])/4.0
        width = 1920
        return math.radians(69/2)*(width/2 - center_x)/(width/2)

    def process_and_publish(self, img):
        tags = self.detect_april_tags(img)
        if len(tags[0]) > 0:
            print(self.time, tags, [self.corners_to_dist(c) for c in tags[1]])
        self.publish('apriltags', tags)
        targets = [[self.corners_to_dist(c), self.corners_to_angle(c)] for c in tags[1]]
        self.publish('targets', targets)

    def on_jpeg(self, data):
        nparr = np.frombuffer(data, np.uint8)
        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        self.process_and_publish(img)

    def on_video(self, data):
        try:
            packets = self.codec.parse(data)
            for packet in packets:
                try:
                    frames = self.codec.decode(packet)
                    for frame in frames:
                        img = frame.to_ndarray(format='bgr24')
                        if img is not None:
                            self.process_and_publish(img)
                except av.error.FFmpegError:
                    # Ignore decoding errors from incomplete packets/keyframes at startup
                    pass
        except av.error.FFmpegError:
            # Ignore parsing errors from incomplete streams
            pass

# vim: expandtab sw=4 ts=4

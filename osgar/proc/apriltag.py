"""
  Follow AprilTag
"""

import math

import av
import cv2

from osgar.node import Node
from osgar.bus import BusShutdownException
from osgar.exceptions import EmergencyStopException


class AprilTag(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('apriltags', 'targets')
        self.codec = av.CodecContext.create('hevc', 'r')  # h265

    def detect_april_tags(self, image):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_25h9)
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

    def on_video(self, data):
        try:
            packets = self.codec.parse(data)
            for packet in packets:
                try:
                    frames = self.codec.decode(packet)
                    for frame in frames:
                        img = frame.to_ndarray(format='bgr24')
                        if img is not None:
                            tags = self.detect_april_tags(img)
                            if len(tags[0]) > 0:
                                print(self.time, tags, [self.corners_to_dist(c) for c in tags[1]])
                            self.publish('apriltags', tags)
                            targets = [[self.corners_to_dist(c), self.corners_to_angle(c)] for c in tags[1]]
                            self.publish('targets', targets)
                except av.error.FFmpegError:
                    # Ignore decoding errors from incomplete packets/keyframes at startup
                    pass
        except av.error.FFmpegError:
            # Ignore parsing errors from incomplete streams
            pass


class FollowAprilTag(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_steering')

        # Configuration parameters
        self.max_speed = config.get('max_speed', 0.5)
        self.target_distance = config.get('target_distance', 0.5)
        self.raise_exception_on_stop = config.get('terminate_on_stop', True)

    def send_speed_cmd(self, speed, steering_angle):
        return self.bus.publish(
            'desired_steering',
            [round(speed * 1000), round(math.degrees(steering_angle) * 100)]
        )

    def on_emergency_stop(self, data):
        if data:
            self.send_speed_cmd(0, 0)  # STOP!

        if self.raise_exception_on_stop and data:
            raise EmergencyStopException()

    def on_bumpers_front(self, data):
        if data:
            self.send_speed_cmd(0, 0)

    def on_bumpers_rear(self, data):
        if data:
            self.send_speed_cmd(0, 0)

    def on_targets(self, data):
        if len(data) == 0:
            self.send_speed_cmd(0, 0)
        else:
            dist, angle = min(data, key=lambda x: x[0])
            if dist < self.target_distance:
                self.send_speed_cmd(0, 0)
            else:
                self.send_speed_cmd(self.max_speed, angle)
# vim: expandtab sw=4 ts=4

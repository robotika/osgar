"""
  Loop detection.
"""

import math

from osgar.lib.quaternion import angle_between

class LoopDetector:
    def __init__(self, orientation_similarity_threshold=math.radians(20),
                       pose_distance_threshold=0.5,
                       granularity=0.3,
                       max_loop_length=100.0,
                       min_loop_length=5.0):
        self.orientation_similarity_threshold = orientation_similarity_threshold
        self.pose_distance_threshold = pose_distance_threshold
        self.granularity = granularity
        self.max_loop_length = max_loop_length
        self.min_loop_length = min_loop_length

        self.trajectory = []


    def add(self, xyz, orientation):
        if not self.trajectory:
            self.trajectory.append([xyz, orientation, 0])
            return

        prev_xyz = self.trajectory[-1][0]
        d = math.sqrt(sum((a - b)**2 for (a, b) in zip(xyz, prev_xyz)))
        if d < self.granularity:
            return
        self.trajectory[-1][2] = d
        self.trajectory.append([xyz, orientation, 0])


    def add_all(self, poses):
        for pose in poses:
            self.add(*pose)


    def loop(self):
        if not self.trajectory:
            return None

        loop_candidate = []
        loop_candidate_length = 0.0
        is_loop = False

        curr_xyz, curr_orientation, _ = self.trajectory[-1]

        for past_xyz, past_orientation, d in reversed(self.trajectory):
            loop_candidate.append((past_xyz, past_orientation))

            loop_candidate_length += d
            if loop_candidate_length < self.min_loop_length:
                continue

            if loop_candidate_length > self.max_loop_length:
                break

            dist = math.sqrt(sum((a-b)**2 for (a, b) in zip(curr_xyz, past_xyz)))
            if dist > self.pose_distance_threshold:
                continue

            angle = angle_between(curr_orientation, past_orientation)
            if angle > self.orientation_similarity_threshold:
                continue

            is_loop = True
            break


        return loop_candidate[::-1] if is_loop else None

if __name__ == '__main__':
    import argparse
    import cv2
    import os
    import sys

    from osgar.lib.serialize import deserialize
    from osgar.logger import LogReader, lookup_stream_id

    parser = argparse.ArgumentParser()
    parser.add_argument('--logfile', help='Path to Osgar log.')
    parser.add_argument('--pose3d-stream',
                        help='Stream id or name with pose and orientation data.')
    args = parser.parse_args()
    assert (args.logfile)
    assert (args.pose3d_stream)

    pose3d_stream_id = lookup_stream_id(args.logfile, args.pose3d_stream)
    loop_detector = LoopDetector()
    with LogReader(args.logfile,
                   only_stream_id=pose3d_stream_id) as logreader:
        for time, stream, data in logreader:
            pose, orientation = deserialize(data)
            loop_detector.add(pose, orientation)
            loop = loop_detector.loop()
            if loop:
                print('xxxx', loop)
            #else:
            #    print('---- ', pose)


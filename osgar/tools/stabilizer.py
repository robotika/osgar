#!/usr/bin/python
"""
  Experiment with video stabilization
"""
from datetime import timedelta

try:
    import cv2
except ImportError:
    print('\nERROR: Please install OpenCV\n    pip install opencv-python\n')

try:
    import numpy as np
except ImportError:
    print('\nERROR: Please install numpy\n    pip install numpy\n')

from osgar.logger import LogReader, lookup_stream_id
from osgar.lib.serialize import deserialize


def stabilize_video(logfile, stream, outfile=None):
    only_stream = lookup_stream_id(logfile, stream)
    # https://stackoverflow.com/questions/49971484/opencv-orb-descriptor-typeerror-incorrect-type-of-self-must-be-feature2d-or
    # orb = cv2.ORB()
    orb = cv2.ORB_create()
    with LogReader(logfile, only_stream_id=only_stream) as log:
        prev = None
        for timestamp, stream_id, data in log:
            buf = deserialize(data)
            img = cv2.imdecode(np.fromstring(buf, dtype=np.uint8), 1)
            if prev is not None:
                # find the keypoints and descriptors
                img1, img2 = prev, img
                kp1, des1 = orb.detectAndCompute(img1, None)
                kp2, des2 = orb.detectAndCompute(img2, None)

                # create BFMatcher object
                bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

                # Match descriptors.
                matches = bf.match(des1, des2)

                # Sort them in the order of their distance.
                matches = sorted(matches, key=lambda x: x.distance)
                print(len(matches))
                for m in matches[:3]:
#                    print(m.distance, m.imgIdx, m.queryIdx, m.trainIdx)
                    assert m.imgIdx == 0, m.imgIdx
#                    print(kp1[m.queryIdx].pt, kp2[m.trainIdx].pt)
                    print([a - b for a, b in zip(kp1[m.queryIdx].pt, kp2[m.trainIdx].pt)])

                # Draw first 10 matches.
                img3 = img1.copy()
                img3 = cv2.drawMatches(img1, kp1, img2, kp2, matches[:10], flags=2, outImg=img3)

                cv2.imshow('image', img3)
                c = cv2.waitKey(100)
                if c >= 0:
                    break
            prev = img
    cv2.destroyAllWindows()


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Stabilize video in logfile')
    parser.add_argument('logfile', help='recorded log file')
    parser.add_argument('--stream', help='stream ID or name', default='camera.raw')
    args = parser.parse_args()

    stabilize_video(args.logfile, args.stream)


if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4 


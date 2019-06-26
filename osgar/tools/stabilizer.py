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


def stabilize_video(logfile, stream, outfile=None, fps=None):
    only_stream = lookup_stream_id(logfile, stream)
    # https://stackoverflow.com/questions/49971484/opencv-orb-descriptor-typeerror-incorrect-type-of-self-must-be-feature2d-or
    # orb = cv2.ORB()
    orb = cv2.ORB_create()
    offset = 0
    stable = np.zeros((480, 1280, 3), np.uint8)
    writer = None
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
                arr = []
                for m in matches[:10]:
#                    print(m.distance, m.imgIdx, m.queryIdx, m.trainIdx)
                    assert m.imgIdx == 0, m.imgIdx
#                    print(kp1[m.queryIdx].pt, kp2[m.trainIdx].pt)
                    diff = [a - b for a, b in zip(kp1[m.queryIdx].pt, kp2[m.trainIdx].pt)]
#                    print(diff)
                    arr.append(int(diff[0]))
                median = sorted(arr)[len(arr)//2]
                offset += median
                print(offset, median, arr)

                # Draw first 10 matches.
                img3 = img1.copy()
                img3 = cv2.drawMatches(img1, kp1, img2, kp2, matches[:10], flags=2, outImg=img3)
                print(img3.shape)

                # small_image.copyTo(big_image(cv::Rect(x, y, small_image.cols, small_image.rows)));
                # https://answers.opencv.org/question/37568/how-to-insert-a-small-size-image-on-to-a-big-image/
                if offset < 0:
                    offset = 0
                if offset >= 640:
                    offset = 640
                stable[0:480, offset:offset+640, ] = img2

                # TODO hystesis, move the boundary
                if outfile is not None and writer is None:
                    height, width = stable.shape[:2]
                    writer = cv2.VideoWriter(outfile,
                                             cv2.VideoWriter_fourcc('F', 'M', 'P', '4'),
                                             fps,
                                             (width, height))

                if writer:
                    writer.write(stable)

                cv2.imshow('image', stable)
                c = cv2.waitKey(10)
                if c >= 0:
                    break
            prev = img
    if writer:
        writer.release()
    cv2.destroyAllWindows()


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Stabilize video in logfile')
    parser.add_argument('logfile', help='recorded log file')
    parser.add_argument('--stream', help='stream ID or name', default='camera.raw')
    parser.add_argument('--out', help='output filename (.avi)')
    parser.add_argument('--fps', help='frames per second', type=int, default=10)
    args = parser.parse_args()

    stabilize_video(args.logfile, args.stream, outfile=args.out, fps=args.fps)


if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4 


#!/usr/bin/python
"""
  Convert logfile to AVI video
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


def create_video(logfile, stream, outfile, add_time=False,
                 start_time_sec=0, end_time_sec=None, fps=25):
    assert outfile.endswith(".avi"), outFilename
    only_stream = lookup_stream_id(logfile, stream)
    with LogReader(logfile, only_stream_id=only_stream) as log:
        writer = None
        for timestamp, stream_id, data in log:
            buf = deserialize(data)
            img = cv2.imdecode(np.fromstring(buf, dtype=np.uint8), 1)
            if writer is None:
                height, width = img.shape[:2]
                writer = cv2.VideoWriter(outfile,
                                         cv2.VideoWriter_fourcc('F', 'M', 'P', '4'),
                                         fps,
                                         (width, height))
            if add_time:
                if (width, height) == (640, 480):
                    x, y = 350, 50
                    thickness = 3
                    size = 3.0
                else:
                    x, y = 800, 100
                    thickness = 5
                    size = 5.0
                # clip microseconds to miliseconds
                s = str(timestamp-timedelta(seconds=start_time_sec))[:-3]
                cv2.putText(img, s, (x, y), cv2.FONT_HERSHEY_PLAIN, 
                            size, (255, 255, 255), thickness=thickness)

            if timestamp.total_seconds() > start_time_sec:
                writer.write(img)
            if end_time_sec is not None and timestamp.total_seconds() > end_time_sec:
                break
        writer.release()


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Convert logfile to AVI video')
    parser.add_argument('logfile', help='recorded log file')
    parser.add_argument('--stream', help='stream ID or name', default='camera.raw')
    parser.add_argument('--out', '-o', help='output AVI file', default='out.avi')
    parser.add_argument('--display-time', '-t', help='add timestamp info', action='store_true')
    parser.add_argument('--start-time-sec', '-s', help='start video at later time (sec)',
                        type=float, default=0.0)
    parser.add_argument('--end-time-sec', '-e', help='cut video at given time (sec)',
                        type=float, default=None)
    parser.add_argument('--fps', help='frames per second', type=int, default=25)
    args = parser.parse_args()

    create_video(args.logfile, args.stream, args.out, add_time=args.display_time,
                 start_time_sec=args.start_time_sec, end_time_sec=args.end_time_sec, fps=args.fps)


if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4 


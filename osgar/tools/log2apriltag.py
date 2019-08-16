import apriltag
import cv2
import numpy

from osgar.logger import LogIndexedReader, LogReader, lookup_stream_id, lookup_stream_names
from osgar.lib.serialize import deserialize

def main():
    import argparse
    import sys
    import io
    from pprint import pprint

    parser = argparse.ArgumentParser(description='')
    parser.add_argument('log', help='filepaths', nargs='+')
    parser.add_argument('--threads', help='how many threads to use', type=int, default=1)
    parser.add_argument('--gui', help='show found tags', default=False, action='store_true')
    parser.add_argument('--margin', help='apriltag decision margin threshold', default=30, type=int)

    args = parser.parse_args()
    detector = apriltag.apriltag('tag16h5', threads=args.threads)

    for filepath in args.log:
        print(filepath)
        streams = lookup_stream_names(filepath)
        processing = []
        for i, stream in enumerate(streams):
            if (stream.startswith("camera") and stream.endswith("raw")) or stream.endswith('.jpg') or stream.endswith('.image'):
                print("processing stream {} => {}".format(i, stream))
                processing.append(i+1)
        if len(processing) == 0:
            print("no jpeg found in streams:")
            pprint(streams)
            continue

        with LogReader(filepath, only_stream_id=processing) as log:
            for dt, channel, data in log:
                try:
                    jpeg = deserialize(data)
                except Exception as e:
                    print(e)
                    continue
                np_jpeg = numpy.frombuffer(jpeg, dtype='u1')
                gray = cv2.imdecode(np_jpeg, cv2.IMREAD_GRAYSCALE)
                found = detector.detect(gray)
                found = [tag for tag in found if tag['margin'] > args.margin and tag['hamming'] == 0]
                if len(found) > 0:
                    ids = list(tag['id'] for tag in found)
                    print(dt, end=' ')
                    if args.gui:
                        img = cv2.imdecode(np_jpeg, cv2.IMREAD_COLOR)
                    for tag in found:
                        rect = tag['lb-rb-rt-lt'].astype('float32')
                        area = cv2.contourArea(rect)
                        print('{:2d}: {{margin: {:3d}, area: {:4d}}}'.format(tag['id'], int(tag['margin']), int(area)), end=' ')
                        center = tuple(tag['center'].astype(int))
                        poly = rect.astype('int32')
                        if args.gui:
                            cv2.circle(img, center, 3, (0, 0 ,255), -1)
                            cv2.polylines(img, [poly], True, (0, 255, 255), 3)
                    print()
                    if args.gui:
                        cv2.imshow('image', img)
                        key = cv2.waitKey(0)
                        if key == 27:
                            return

if __name__ == "__main__":
    main()


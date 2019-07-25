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

    args = parser.parse_args()
    detector = apriltag.apriltag('tag16h5', threads=args.threads)

    for filepath in args.log:
        print(filepath)
        streams = lookup_stream_names(filepath)
        processing = []
        for i, stream in enumerate(streams):
            if stream == "camera.raw" or stream.endswith('.jpg') or stream.endswith('.image'):
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
                if len(found) > 0:
                    ids = list(tag['id'] for tag in found if tag['hamming'] == 0)
                    if (len(ids) > 0):
                        print(dt, end=' ')
                        pprint(ids)

if __name__ == "__main__":
    main()


"""
   Laser Feature extractor
"""


def extract_features(scan):
    pass


if __name__ == "__main__":
    import argparse
    from osgar.logger import LogReader, lookup_stream_id
    from osgar.lib.serialize import deserialize

    parser = argparse.ArgumentParser(description='Extract features in laser scan')
    parser.add_argument('filename', help='input log file')
    args = parser.parse_args()

    filename = args.filename
    only_stream = lookup_stream_id(filename, 'lidar.scan')
    index = 0
    with LogReader(filename) as log:
        for ind, row in enumerate(log.read_gen(only_stream)):
            if ind < index:
                continue
            timestamp, stream_id, data = row
            data = deserialize(data)
            print(extract_features(data))
            break
    

# vim: expandtab sw=4 ts=4


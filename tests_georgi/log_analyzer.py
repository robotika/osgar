from datetime import timedelta
from osgar.logger import LogReader, lookup_stream_id, lookup_stream_names
from osgar.lib.serialize import deserialize

def read_logfile(logfile, stream):
    names = lookup_stream_names(logfile)
    print(f"Available streams: {names}")
    only_stream = lookup_stream_id(logfile, stream)
    with LogReader(logfile, only_stream_id=only_stream) as log:
        for timestamp, stream_id, data in log:
            if timedelta(seconds=3) <= timestamp < timedelta(seconds=4):
                depth = deserialize(data)
                print(timestamp)
                print(depth[400//2][640//2])
                print()

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='Extract data from logfile')
    parser.add_argument('--logfile', help='recorded log file')
    parser.add_argument('--stream', help='stream ID or name', default='oak.depth')
    args = parser.parse_args()
    read_logfile(args.logfile, args.stream)

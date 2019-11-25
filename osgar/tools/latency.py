import argparse
import datetime

from pprint import pprint

from osgar.logger import LogReader, lookup_stream_names
from osgar.lib.serialize import deserialize

def main():
    parser = argparse.ArgumentParser(description='Output Eduro latency in ms')
    parser.add_argument('logfile', help='filename of stored file')
    args = parser.parse_args()

    streams = lookup_stream_names(args.logfile)
    #pprint(streams)
    can_in = streams.index('can.can')+1
    can_out = streams.index('can.raw')+1
    #pprint(f"in: {can_in}, out: {can_out}")

    with LogReader(args.logfile, only_stream_id=(can_in, can_out)) as log:
        sync = datetime.timedelta()
        for timestamp, stream_id, data in log:
            data = deserialize(data)
            if isinstance(data, bytes):
                data = [(data[0] << 3) | (data[1] >> 5), data[2:], 0]
            if stream_id == can_in:
                if data[0] == 0x80:
                    sync = timestamp
            else:
                if data[0] == 0x201:
                    print(f'{(timestamp - sync)/datetime.timedelta(microseconds=1000)}')


if __name__ == "__main__":
    main()

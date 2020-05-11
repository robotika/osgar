"""
  Multiple Logfile Merger
"""
from datetime import timedelta
from osgar.logger import LogReader, LogWriter, lookup_stream_id, lookup_stream_names


def merge_logfiles(logfile, outfile):
    print('Out:', outfile)
    assert len(logfile) == 2, len(logfile)  # experiment with 2 "to get started"
    only_stream0 = lookup_stream_id(logfile[0], 'rosmsg.image')
    only_stream1 = lookup_stream_id(logfile[1], 'rosmsg.image')
    with LogReader(logfile[0], only_stream_id=only_stream0) as log0, \
         LogReader(logfile[1], only_stream_id=only_stream1) as log1:
        print(log0.start_time, log1.start_time, log0.start_time - log1.start_time)
        with LogWriter(filename=outfile, start_time=log0.start_time) as out:  # TODO min log0/log1
            timestamp = timedelta()
            out.write(stream_id=0, data=b"{'names': ['log0.image', 'log1.image']}", dt=timestamp)
            timestamp0, stream_id0, data0 = next(log0)
            timestamp1, stream_id1, data1 = next(log1)
            while True:
                if timestamp0 <= timestamp1:
                    timestamp, stream_id, data = timestamp0, 1, data0
                    timestamp0, stream_id0, data0 = next(log0)
                else:
                    timestamp, stream_id, data = timestamp1, 2, data1
                    timestamp1, stream_id1, data1 = next(log1)
                out.write(stream_id=stream_id, data=data, dt=timestamp)


def main():
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logfile', help='input log file', nargs='+')
    parser.add_argument('--out', '-o', help='output logfile', required=True)
    args = parser.parse_args()

    merge_logfiles(args.logfile, args.out)


if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4 


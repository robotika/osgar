"""
  Multiple Logfile Merger
"""
from osgar.logger import LogReader, LogWriter, lookup_stream_id, lookup_stream_names


def merge_logfiles(logfile, outfile):
    print('Out:', outfile)
    assert len(logfile) == 2, len(logfile)  # experiment with 2 "to get started"
    only_stream0 = lookup_stream_id(logfile[0], 'rosmsg.image')
    only_stream1 = lookup_stream_id(logfile[1], 'rosmsg.image')
    with LogReader(logfile[0], only_stream_id=only_stream0) as log0, \
         LogReader(logfile[1], only_stream_id=only_stream1) as log1:
        print(log0.start_time, log1.start_time)
        assert False
        with LogWriter(filename=outfile, start_time=log.start_time) as out:
            for timestamp, stream_id, data in log:
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


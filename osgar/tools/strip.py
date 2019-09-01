"""
  Remove some streams from logfile
"""
from datetime import timedelta

from osgar.logger import LogReader, LogWriter, lookup_stream_id, lookup_stream_names


def strip_logfile(logfile, only_stream, outfile):
    if outfile is None:
        outfile = logfile[:-4] + '-strip' + logfile[-4:]
    print('Out:', outfile)
    with LogReader(logfile, only_stream_id=only_stream) as log:
        with LogWriter(filename=outfile, start_time=log.start_time) as out:
            for timestamp, stream_id, data in log:
                out.write(stream_id=stream_id, data=data, dt=timestamp)


def main():
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logfile', help='input log file')
    parser.add_argument('--keep', help='stream ID or names to keep', nargs='+')
    parser.add_argument('--remove', help='stream ID or names to be removed', nargs='+')
    parser.add_argument('--out', '-o', help='output logfile')
    args = parser.parse_args()

    if args.keep is None and args.remove is None:
        only_stream = None
    else:
        only_stream = [0, ]
        if args.keep is not None:
            names = args.keep
        elif args.remove is not None:
            names = set(lookup_stream_names(args.logfile)) - set(args.remove)
        for name in names:
            only_stream.append(lookup_stream_id(args.logfile, name))

    strip_logfile(args.logfile, only_stream, args.out)


if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4 


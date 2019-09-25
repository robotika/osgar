"""
  ASCII Art map from robot positions
  (used for quick overview of terminal application)
"""
from osgar.logger import LogReader, lookup_stream_id
from osgar.lib.serialize import deserialize


def create_ascii_art_map(logfile, stream, step):
    only_stream = lookup_stream_id(logfile, stream)
    art = set()
    with LogReader(logfile, only_stream_id=only_stream) as log:
        for timestamp, stream_id, data in log:
            x, y, __ = deserialize(data)
            i, j = (int((x/1000.0 + step/2)/step), int((y/1000.0 + step/2)/step))
            art.add((i, j))

    arr_i = [i for i, __ in art]
    arr_j = [j for __, j in art]
    min_i, max_i = min(arr_i), max(arr_i)
    min_j, max_j = min(arr_j), max(arr_j)
    for j in range(max_j, min_j - 1,  -1):
        s = ''
        for i in range(min_i, max_i + 1):
            s += 'X' if (i, j) in art else ' '
        print(s)


def main():
    import argparse

    parser = argparse.ArgumentParser(description='ASCII Art map from robot positions')
    parser.add_argument('logfile', help='recorded log file')
    parser.add_argument('--stream', help='stream ID or name', default='app.pose2d')
    parser.add_argument('--step', help='square side size in meters', type=float, default=1.0)
    args = parser.parse_args()

    create_ascii_art_map(args.logfile, args.stream, step=args.step)


if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4 


"""
  Extract position with topics statistic from AWS ROS_INFO output
"""
import sys
import os
from itertools import chain


def aws2info(filename, outname):
    KEY = 'Python3: stdout '
    with open(filename) as f, open(outname, 'w') as out:
        for line in f:
            if KEY in line and line.strip().endswith(']'):
                out.write(line[line.index(KEY)+len(KEY):])


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('filename', help='AWS ROS recorded log file')
    args = parser.parse_args()

    assert 'rosout.log' in args.filename, args.filename
    outname = args.filename.replace('rosout.log', 'info.txt')
    aws2info(args.filename, outname=outname)

# vim: expandtab sw=4 ts=4

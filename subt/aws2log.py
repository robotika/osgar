"""
  Convert AWS ROS_INFO dump into OSGAR logfile
"""
import sys


def aws2log(filename):
    with open(filename) as f:
        for line in f:
            if 'Python3' in line and 'Dump' in line:
                print(line)
                break

        for line in f:
            if 'Python3' in line and 'Size' in line:
                print(line)
                break

        with open('out.log', 'wb') as out:
            i = 0
            for line in f:
                if 'Python3' in line:
                    assert 'stdout' in line, line
                    s = line.split('Python3: stdout ')[1]
                    offset = int(s.split()[0])
                    data = eval(s[s.index(' '):])
                    out.write(data)
                    assert i * 100 == offset, (i, s, prev)
                    prev = line
                    i += 1



if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('filename', help='AWS ROS recorded log file')
    args = parser.parse_args()

    aws2log(args.filename)

# vim: expandtab sw=4 ts=4

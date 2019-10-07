"""
  Convert AWS ROS_INFO dump into OSGAR logfile
"""
import sys
import os.path


def aws2log(filename, outname, recover=False):
    with open(filename) as f:
        if not recover:
            # search for the beginning
            for line in f:
                if 'Python3' in line and 'Dump' in line:
                    print(line)
                    break

            for line in f:
                if 'Python3' in line and 'Size' in line:
                    print(line)
                    break

        with open(outname, 'wb') as out:
            prev = None
            i = 0
            for line in f:
                if 'Python3' in line:
                    assert 'stdout' in line, line
                    s = line.split('Python3: stdout ')[1]
                    offset = int(s.split()[0])
                    data = eval(s[s.index(' '):])
                    out.write(data)
                    if prev is None and recover:
                        i = offset//100
                    assert i * 100 == offset, (i, s, prev)
                    prev = line
                    i += 1



if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('filename', help='AWS ROS recorded log file')
    parser.add_argument('--recover', help='Skip initial part and recover from missing global header', action='store_true')
    args = parser.parse_args()

    outname = os.path.join(os.path.dirname(args.filename), 'osgar.log')
    aws2log(args.filename, outname=outname, recover=args.recover)

# vim: expandtab sw=4 ts=4

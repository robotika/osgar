#!/usr/bin/env python
"""
  simple tool ZIP output logfile into single ZIP archive
    ./zipit.py <metalog filename>
"""

import sys
import os
from zipfile import ZipFile

PREFIX = 'osgar_'


def zipit(metalog_filename, output_filename):
    dir_name = os.path.dirname(metalog_filename)
    with ZipFile(output_filename, 'w') as myzip:
        myzip.write(metalog_filename, os.path.basename(metalog_filename))
        camera_file = None
        for line in open(metalog_filename):
            if line.startswith('['):
                continue  # initial arguments
            assert ': ' in line, line
            assert len(line.split(':')) == 2, line.split(':')
            name = line.split(':')[1].strip()
            assert name.startswith('logs/')
            filename = name[5:]
            print(filename)
            myzip.write(os.path.join(dir_name, filename), filename)
            if filename.startswith('gps_'):
                filename = filename.replace('.log', '.nmea')
                filename = filename.replace('gps_', 'gps')
                print(filename)
                myzip.write(os.path.join(dir_name, filename), filename)
            elif filename.startswith('camera_'):
                camera_file = os.path.join(dir_name, filename)
        
        if camera_file is not None:
            for line in open(camera_file):
                if not line.startswith('('):
                    continue
                name = eval(line)[0]
                assert name.startswith('logs/')
                filename = name[5:]
                print(filename)
                myzip.write(os.path.join(dir_name, filename), filename)


if __name__ == "__main__": 
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)

    for filename in sys.argv[1:]:
        assert 'meta_' in filename
        out_name = os.path.basename(filename).replace('meta_', PREFIX)
        out_name = out_name.replace('.log', '.zip')
        zipit(filename, out_name)

# vim: expandtab sw=4 ts=4 


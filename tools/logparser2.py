#!/usr/bin/python
"""
  Log parser - 2nd generation (timestamps based)
  usage:
       ./logparser2.py <metalog>
"""
import ast
import os
import sys
from zipfile import ZipFile

def sensor_gen(path, *selected):
    """
      Yield timestamp ordered data from all selected sensors.
      By default return data from all sensors.
    """
    selected = set(selected)
    with Log(path) as log:
        data = []
        for sensor in log.streams:
            if sensor != 'can' and (len(selected) == 0 or sensor in selected):
                data.extend(_stream(log, sensor))

        data.sort()
        for a in data:
            yield a


def _stream(log, id):
    if id == 'can': # binary log, not implemented
        return
    if id == 'timestamps':
        with log.open(log.streams[id]) as stream:
            for line in stream:
                line = ast.literal_eval(line)
                assert len(line) == 3
                yield float(line[0]), 'can', line[1:]
        return

    with log.open(log.streams[id]) as stream:
        for i, line in enumerate(stream):
            if i % 2 == 1:
                data = ast.literal_eval(line)
                yield timestamp, id, data
            else:
                vals = line.split()
                if len(vals) != 2: # only at the end of file
                    return
                timestamp = float(vals[1])


def Log(path):
    "Factory function returning either ZipLog or Dirlog based on argument."
    if path.endswith('.zip'):
        return ZipLog(path)
    else:
        filename = os.path.basename(path)
        assert filename.startswith('meta_')
        return DirLog(path)


class DirLog:
    def __init__(self, metapath):
        self.dirpath = os.path.dirname(metapath)
        self.metaname = os.path.basename(metapath)

    def __enter__(self):
        self.streams = dict(_meta(self))
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        pass

    def open(self, filename):
        assert '/' not in filename
        return open(self.dirpath + '/' + filename)

    def read(self, filename):
        with self.open(filename) as f:
            return f.read()


class ZipLog:
    def __init__(self, filepath):
        self.filepath = filepath

    def __enter__(self):
        self.zip = ZipFile(self.filepath)
        meta_filename = [name for name in self.zip.namelist() if name.startswith('meta_')]
        assert len(meta_filename) == 1
        self.metaname = meta_filename[0]
        self.streams = dict(_meta(self))
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.zip.close()

    def open(self, filename):
        assert '/' not in filename
        return self.zip.open(filename)

    def read(self, filename):
        assert '/' not in filename
        return self.zip.read(filename)


def _meta(log):
    with log.open(log.metaname) as stream:
        for line in filter(lambda a: ':' in a, stream):  # skip unknown lines:
            sensor, filepath = line.strip().split(':')
            yield sensor, os.path.basename(filepath)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    for timestamp, sensor, data in sensor_gen(sys.argv[1]):
        print(timestamp, sensor, data[:10])

# vim: expandtab sw=4 ts=4 

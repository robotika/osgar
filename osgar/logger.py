#
#  Multi-stream logging with timestamps
#
# This is a simple logger for storing several independent data streams. It was
# developed during Naio competition (see https://github.com/robotika/naio) as
# an alternative to multiple log files. The concept is similar to ADTF (Automotive
# Data and Time-Triggered Framework) DAT files.
#
#   The file is using a hand made serialization of Python timedelta structures
# for timestamps.
# Each data block has header of form:
#   <timestamp>, <streamID>, <size of data>, <data>
# where "timestamp" is 32bit unsigned integer in microseconds, "streamID" is
# 16bit unsigned integer defining stream (0 is reserved for info, input and
# output have different ID), and "size" is 16bit unsigned integer defining
# the size of followed data.
#
#   The choice of microseconds was compromise between necessary number of bytes
# for timestamp and available resolution. 32bit corresponds to 4294 seconds, i.e.
# more than an hour of recording. The file contains absolute time and date in
# the overall file header allowing splitting the file and getting absolute time
# from deltas if necessary.
#
#   The stream ID is currently just integer without detailed description. There
# is planned extra info stored in "info channel/stream" ID = 0.
#
#   Finally limiting the block size to 65kB can be overcome by pre-reserved value
# 0 and 0xFFFF. The big block could be then split into several smaller once. This
# part is not defined yet.
#

import datetime
import struct
import os
import logging
import time
from threading import RLock
from ast import literal_eval


INFO_STREAM_ID = 0
ENV_OSGAR_LOGS = 'OSGAR_LOGS'

TIMESTAMP_OVERFLOW_STEP = (1 << 32)  # in microseconds resolution
TIMESTAMP_MASK = TIMESTAMP_OVERFLOW_STEP - 1

class LogWriter:
    def __init__(self, prefix='naio', note=''):
        self.lock = RLock()
        self.start_time = datetime.datetime.utcnow()
        self.filename = prefix + self.start_time.strftime("%y%m%d_%H%M%S.log")
        if ENV_OSGAR_LOGS in os.environ:
            os.makedirs(os.environ[ENV_OSGAR_LOGS], exist_ok=True)
            self.filename = os.path.join(os.environ[ENV_OSGAR_LOGS], self.filename)
        else:
            logging.warning('Environment variable %s is not set - using working directory' % ENV_OSGAR_LOGS)
        self.f = open(self.filename, 'wb')
        self.f.write(b'Pyr\x00')

        t = self.start_time
        self.f.write(struct.pack('HBBBBBI', t.year, t.month, t.day,
                t.hour, t.minute, t.second, t.microsecond))
        self.f.flush()
        if len(note) > 0:
            self.write(stream_id=INFO_STREAM_ID, data=bytes(note, encoding='utf-8'))
        self.names = []

    def register(self, name):
        with self.lock:
            assert name not in self.names, (name, self.names)
            self.names.append(name)
            self.write(stream_id=INFO_STREAM_ID, data=bytes(str({'names': self.names}), encoding='ascii'))
            return len(self.names)

    def write(self, stream_id, data):
        with self.lock:
            dt = datetime.datetime.utcnow() - self.start_time
            bytes_data = data
            assert dt.days == 0, dt  # multiple days not supported yet
            time_frac = (dt.seconds * 1000000 + dt.microseconds) & TIMESTAMP_MASK
            index = 0
            while index + 0xFFFF <= len(bytes_data):
                self.f.write(struct.pack('IHH', time_frac,
                             stream_id, 0xFFFF))
                self.f.write(bytes_data[index:index + 0xFFFF])
                index += 0xFFFF
            self.f.write(struct.pack('IHH', time_frac,
                         stream_id, len(bytes_data) - index))
            self.f.write(bytes_data[index:])
            self.f.flush()
        return dt

    def close(self):
        self.f.close()
        self.f = None


    # context manager functions
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


class LogReader:
    def __init__(self, filename, follow=False, only_stream_id=None):
        self.filename = filename
        self.follow = follow
        self.f = open(self.filename, 'rb')
        data = self._read(4)
        assert data == b'Pyr\x00', data

        data = self._read(12)
        self.start_time = datetime.datetime(*struct.unpack('HBBBBBI', data))
        self.us_offset = 0  # increase after overflow
        self.prev_microseconds = 0
        self.gen = self._read_gen(only_stream_id=only_stream_id)

    def _read(self, size):
        buf = self.f.read(size)
        if self.follow:
            while len(buf) < size:
                time.sleep(0.1)
                buf += self.f.read(size - len(buf))
        return buf

    def _read_gen(self, only_stream_id=None):
        "packed generator - yields (time, stream, data)"
        if only_stream_id is None:
            multiple_streams = set()
        else:
            try:
                multiple_streams = set(only_stream_id)
            except TypeError:
                multiple_streams = set([only_stream_id])

        while True:
            header = self._read(8)
            if len(header) < 8:
                break
            microseconds, stream_id, size = struct.unpack('IHH', header)
            if self.prev_microseconds > microseconds:
                self.us_offset += TIMESTAMP_OVERFLOW_STEP
            self.prev_microseconds = microseconds
            microseconds += self.us_offset
            dt = datetime.timedelta(microseconds=microseconds)
            data = self._read(size)
            assert len(data) == size, (len(data), size)
            while size == 0xFFFF:
                header = self._read(8)
                if len(header) < 8:
                    break
                ref_microseconds, ref_stream_id, size = struct.unpack('IHH', header)
                assert microseconds & TIMESTAMP_MASK == ref_microseconds, (microseconds & TIMESTAMP_MASK, ref_microseconds)
                assert stream_id == ref_stream_id, (stream_id, ref_stream_id)
                part = self._read(size)
                assert len(part) == size, (len(part), size)
                data += part

            if len(multiple_streams) == 0 or stream_id in multiple_streams:
                yield dt, stream_id, data

    def close(self):
        self.f.close()
        self.f = None


    # context manager functions
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def __next__(self):
        return next(self.gen)

    def __iter__(self):
        return self


class LogAsserter(LogReader):
    def __init__(self, filename, only_stream_id=None):
        LogReader.__init__(self, filename, only_stream_id=only_stream_id)
        self.assert_stream_id = None

    def write(self, stream_id, data):
        if self.assert_stream_id is not None:
            pass


def lookup_stream_names(filename):
    names = []
    with LogReader(filename) as log:
        for __, channel, line in log:
            # optimization - all names are defined BEFORE other data on other channels
            if channel != 0:
                break
            if b'Errno' in line:
                continue
            d = literal_eval(line.decode('ascii'))
            if 'names' in d:
                names = d['names']
    return names


def lookup_stream_id(filename, stream_name):
    if stream_name is None:
        return None
    try:
        return int(stream_name)
    except ValueError:
        pass
    names = lookup_stream_names(filename)
    return names.index(stream_name) + 1


if __name__ == "__main__":
    import argparse
    import sys
    from osgar.lib.serialize import deserialize

    parser = argparse.ArgumentParser(description='Extract data from log')
    parser.add_argument('logfile', help='filename of stored file')
    parser.add_argument('--stream', help='stream ID or name', default=None)
    parser.add_argument('--list-names', '-l', help='list stream names', action='store_true')
    parser.add_argument('--times', help='display timestamps', action='store_true')
    parser.add_argument('--raw', help='skip data deserialization',
                        action='store_true')
    args = parser.parse_args()

    only_stream = lookup_stream_id(args.logfile, args.stream)

    if args.list_names:
        print(lookup_stream_names(args.logfile))
        sys.exit()

    with LogReader(args.logfile, only_stream_id=only_stream) as log:
        for timestamp, stream_id, data in log:
            if not args.raw and stream_id != 0:
                data = deserialize(data)
            if args.times:
                print(timestamp, stream_id, data)
            else:
                sys.stdout.buffer.write(data)

# vim: expandtab sw=4 ts=4

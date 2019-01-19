import unittest
import os
import time
from threading import Timer
from datetime import datetime, timedelta
from unittest.mock import patch, MagicMock

import numpy as np

import osgar.logger  # needed for patching the osgar.logger.datetime.datetime
from osgar.logger import (LogWriter, LogReader, LogAsserter, INFO_STREAM_ID,
                          lookup_stream_id)


class TimeStandsStill:
    # inspired by
    # https://marcusahnve.org/blog/2017/mocking-datetime-in-python/
    def __init__(self, datetime):
        self.datetime = datetime

    def utcnow(self):
        return self.datetime


def delayed_copy(src, dst, skip_size):
    with open(src, 'rb') as f:
        buf = f.read(skip_size)
        assert len(buf) == skip_size, (len(buf), skip_size)
        with open(dst, 'ab+') as f2:
            buf = f.read()
            f2.write(buf)
            f2.flush()


class LoggerTest(unittest.TestCase):

    def setUp(self):
        ref = os.environ.get('OSGAR_LOGS')
        if ref is not None:
            del os.environ['OSGAR_LOGS']

    def test_writer_prefix(self):
        log = LogWriter(prefix='tmp')
        self.assertTrue(log.filename.startswith('tmp'))
        log.close()
        os.remove(log.filename)

    def test_context_manager(self):
        with LogWriter(prefix='tmpp', note='1st test') as log:
            self.assertTrue(log.filename.startswith('tmpp'))
            filename = log.filename
            start_time = log.start_time
            t1 = log.write(10, b'\x01\x02\x02\x04')
            time.sleep(0.01)
            t2 = log.write(10, b'\x05\x06\x07\x08')
            self.assertLess(t1, t2)
        
        with LogReader(filename) as log:
            self.assertEqual(start_time, log.start_time)

            __, stream_id, data = next(log)
            self.assertEqual(INFO_STREAM_ID, stream_id)

            t, stream_id, data = next(log)
            self.assertEqual(stream_id, 10)
            self.assertEqual(data, b'\x01\x02\x02\x04')

            t, stream_id, data = next(log)
            self.assertTrue(t.microseconds > 100)
            
            with self.assertRaises(StopIteration):
                __ = next(log)

        with LogReader(filename, only_stream_id=10) as log:
            for t, stream_id, data in log:
                self.assertEqual(stream_id, 10)
                self.assertEqual(data, b'\x01\x02\x02\x04')
                break

        os.remove(log.filename)

    def test_read_two_streams(self):
        with LogWriter(prefix='tmp2', note='test_read_two_streams') as log:
            filename = log.filename
            t1 = log.write(1, b'\x01\x02\x02\x04')
            time.sleep(0.001)
            t2 = log.write(3, b'\x05\x06')
            time.sleep(0.001)
            t3 = log.write(2, b'\x07\x08')

        with LogReader(filename, only_stream_id=[1, 2]) as log:
            arr = []
            for t, stream_id, data in log:
                self.assertIn(stream_id, [1, 2])
                arr.append((t, stream_id))
            self.assertEqual(arr, [(t1, 1), (t3, 2)])

        os.remove(log.filename)

    def test_register(self):
        with LogWriter(prefix='tmp2', note='test_register') as log:
            filename = log.filename
            self.assertEqual(log.register('raw'), 1)

            with self.assertRaises(AssertionError):
                log.register('raw')  # duplicity name

            self.assertEqual(log.register('gps.position'), 2)

        with LogReader(filename, only_stream_id=INFO_STREAM_ID) as log:
            arr = []
            for __, __, data in log:
                if b'names' in data:
                    arr.append(data)
            self.assertEqual(len(arr), 2, arr)
            self.assertEqual(arr[0], b"{'names': ['raw']}")
            self.assertEqual(arr[1], b"{'names': ['raw', 'gps.position']}")

        os.remove(filename)

    def test_log_asserter(self):
        with LogWriter(prefix='tmp3', note='test_log_asserter') as log:
            filename = log.filename
            t1 = log.write(1, b'\x01\x02')
            time.sleep(0.001)
            t2 = log.write(2, b'\x05')
            time.sleep(0.001)
            t3 = log.write(1, b'\x07\x08')

        with LogAsserter(filename, only_stream_id=[1, 2]) as log:
            log.assert_stream_id = 2
            arr = []
            for t, stream_id, data in log:
                self.assertIn(stream_id, [1,2])
                if stream_id == 2:
                    log.write(2, b'\x05')
                arr.append((t, stream_id))
            self.assertEqual(arr, [(t1, 1), (t2, 2), (t3, 1)])

        os.remove(log.filename)

    def test_lookup_stream_id(self):
        self.assertEqual(lookup_stream_id('dummy.log', None), None)
        self.assertEqual(lookup_stream_id('dummy.log', '3'), 3)
        # TODO name lookup

    def test_large_block(self):
        data = bytes([x for x in range(100)]*1000)
        self.assertEqual(len(data), 100000)
        with LogWriter(prefix='tmp4', note='test_large_block') as log:
            filename = log.filename
            t1 = log.write(1, data)
            t2 = log.write(1, data[:0xFFFF])
            t3 = log.write(1, b'')
            t4 = log.write(1, b'ABC')
            t5 = log.write(1, data+data)  # multiple split

        with LogReader(filename, only_stream_id=1) as log:
            arr = []
            for __, __, data in log:
                arr.append(data)
            self.assertEqual([len(x) for x in arr],
                             [100000, 65535, 0, 3, 200000])

        os.remove(log.filename)

    def test_environ(self):
        with LogWriter(prefix='tmp5', note='test_filename_before') as log:
            self.assertTrue(log.filename.startswith('tmp5'))
        os.remove(log.filename)

        os.environ['OSGAR_LOGS'] = 'tmp_dir'
        with LogWriter(prefix='tmp6', note='test_filename_after') as log:
            self.assertTrue(log.filename.startswith('tmp_dir'))
        os.remove(log.filename)

        del os.environ['OSGAR_LOGS']
        with LogWriter(prefix='tmp7', note='test_filename_after2') as log:
            self.assertTrue(log.filename.startswith('tmp7'))
        os.remove(log.filename)

    def test_time_overflow(self):
        with LogWriter(prefix='tmp8', note='test_time_overflow') as log:
            log.start_time = datetime.utcnow() - timedelta(hours=1, minutes=30)
            t1 = log.write(1, b'\x01\x02')
            self.assertGreater(t1, timedelta(hours=1))
            filename = log.filename
        with LogReader(filename, only_stream_id=1) as log:
            dt, channel, data = next(log)
            self.assertGreater(dt, timedelta(minutes=10))
        os.remove(filename)


    def test_time_overflow2(self):
        with patch('osgar.logger.datetime.datetime'):
            osgar.logger.datetime.datetime = TimeStandsStill(datetime(2020, 1, 21))
            with osgar.logger.LogWriter(prefix='tmp9', note='test_time_overflow') as log:
                filename = log.filename
                t1 = log.write(1, b'\x01')
                self.assertEqual(t1, timedelta(0))
                osgar.logger.datetime.datetime = TimeStandsStill(datetime(2020, 1, 21, 1))
                t2 = log.write(1, b'\x02')
                self.assertEqual(t2, timedelta(hours=1))
                osgar.logger.datetime.datetime = TimeStandsStill(datetime(2020, 1, 21, 2))
                t3 = log.write(1, b'\x03')
                self.assertEqual(t3, timedelta(hours=2))
                osgar.logger.datetime.datetime = TimeStandsStill(datetime(2020, 1, 21, 4))
                # TODO this write should rise exception as the time gap is too big to track!
                t4 = log.write(1, b'\x04')
                self.assertEqual(t4, timedelta(hours=4))
        with LogReader(filename, only_stream_id=1) as log:
            dt, channel, data = next(log)
            self.assertEqual(dt, timedelta(hours=0))
            dt, channel, data = next(log)
            self.assertEqual(dt, timedelta(hours=1))
            dt, channel, data = next(log)
            self.assertEqual(dt, timedelta(hours=2))
#            dt, channel, data = next(log)
#            self.assertEqual(dt, timedelta(hours=4))
        os.remove(filename)

    def test_large_blocks_with_time_overflow(self):
        with patch('osgar.logger.datetime.datetime'):
            osgar.logger.datetime.datetime = TimeStandsStill(datetime(2019, 1, 1))
            with osgar.logger.LogWriter(prefix='tmpA', note='test_time_overflow with large blocks') as log:
                filename = log.filename
                t1 = log.write(1, b'\x01'*100000)
                self.assertEqual(t1, timedelta(0))
                osgar.logger.datetime.datetime = TimeStandsStill(datetime(2019, 1, 1, 1))
                t2 = log.write(1, b'\x02'*100000)
                self.assertEqual(t2, timedelta(hours=1))
                osgar.logger.datetime.datetime = TimeStandsStill(datetime(2019, 1, 1, 2))
                t3 = log.write(1, b'\x03'*100000)
                self.assertEqual(t3, timedelta(hours=2))
        with LogReader(filename, only_stream_id=1) as log:
            dt, channel, data = next(log)
            self.assertEqual(dt, timedelta(hours=0))
            dt, channel, data = next(log)
            self.assertEqual(dt, timedelta(hours=1))
            dt, channel, data = next(log)
            self.assertEqual(dt, timedelta(hours=2))
        os.remove(filename)

    def test_no_eof(self):
        # similar to tail -f (--follow)
        with LogWriter(prefix='tmpB', note='test_EOF') as log:
            filename = log.filename
            t1 = log.write(1, b'\x01'*100)
            time.sleep(0.001)
            t2 = log.write(1, b'\x02'*100)
            time.sleep(0.001)
            t3 = log.write(1, b'\x03'*100000)

        with LogReader(filename, only_stream_id=1) as log:
            dt, channel, data = next(log)
            self.assertEqual(data, b'\x01'*100)
            dt, channel, data = next(log)
            self.assertEqual(data, b'\x02'*100)

        partial = filename + '.part'
        with open(filename, 'rb') as f_in:
            with open(partial, 'wb') as f_out:
                f_out.write(f_in.read(100))
        
        with LogReader(partial, only_stream_id=1) as log:
            with self.assertRaises(AssertionError):
                dt, channel, data = next(log)

        proc = Timer(0.1, delayed_copy, [filename, partial, 100])
        proc.start()
        with LogReader(partial, follow=True, only_stream_id=1) as log:
            dt, channel, data = next(log)
            self.assertEqual(data, b'\x01'*100)
        proc.join()

        partial = filename + '.part'
        with open(filename, 'rb') as f_in:
            with open(partial, 'wb') as f_out:
                f_out.write(f_in.read(100000))

        with LogReader(partial, only_stream_id=1) as log:
            dt, channel, data = next(log)
            self.assertEqual(data, b'\x01'*100)
            dt, channel, data = next(log)
            self.assertEqual(data, b'\x02'*100)
            with self.assertRaises(AssertionError):
                dt, channel, data = next(log)

        os.remove(partial)
        os.remove(filename)

    def test_read_gen_seek(self):
        with LogWriter(prefix='tmpC', note='test_read_gen_seek') as log:
            filename = log.filename
            t1 = log.write(1, b'\x01')
            time.sleep(0.001)
            t2 = log.write(1, b'\x02')
            time.sleep(0.001)
            t3 = log.write(1, b'\x03')

        with LogReader(filename, only_stream_id=1) as log:
            dt, channel, data = next(log)
            self.assertEqual(data, b'\x01')
            dt, channel, data = next(log)
            self.assertEqual(data, b'\x02')
        os.remove(filename)

# vim: expandtab sw=4 ts=4

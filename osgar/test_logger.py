import unittest
import os
import tempfile
import time
import logging

from threading import Timer
from datetime import datetime, timedelta, timezone
from unittest.mock import patch
from contextlib import ExitStack
from pathlib import Path

import osgar.logger  # needed for patching the osgar.logger.datetime.datetime
from osgar.logger import (LogWriter, LogReader, LogAsserter, INFO_STREAM_ID,
                          lookup_stream_id, LogIndexedReader)



class TimeStandsStill:
    # inspired by
    # https://marcusahnve.org/blog/2017/mocking-datetime-in-python/
    def __init__(self, datetime):
        self.datetime = datetime.replace(tzinfo=timezone.utc)

    def now(self, tzinfo):
        assert tzinfo == timezone.utc
        return self.datetime


def delayed_copy(src, dst, skip_size):
    with open(src, 'rb') as f:
        buf = f.read(skip_size)
        assert len(buf) == skip_size, (len(buf), skip_size)
        with open(dst, 'ab+') as f2:
            buf = f.read()
            f2.write(buf)
            f2.flush()


class LoggerStreamingTest(unittest.TestCase):

    def setUp(self):
        ref = os.environ.get('OSGAR_LOGS')
        if ref is None:
            os.environ['OSGAR_LOGS'] = "."

    def test_timedelta_sequence(self):
        parse_timedelta = osgar.logger.timedelta_parser()
        for i in range(20000):
            t_ref = timedelta(seconds=i)
            binary = osgar.logger.format_timedelta(t_ref)
            t_parsed = parse_timedelta(binary)
            self.assertEqual(t_ref, t_parsed)

    def test_timedelta_cases(self):
        t_ref = timedelta(hours=1, minutes=11, seconds=35)
        last_dt = t_ref - timedelta(seconds=1)
        binary = osgar.logger.format_timedelta(t_ref)
        parse_timedelta = osgar.logger.timedelta_parser(last_dt)
        t_parsed = parse_timedelta(binary)
        self.assertEqual(t_ref, t_parsed)

    def test_writer_prefix(self):
        with LogWriter(prefix='tmp') as log:
            self.assertTrue(Path(log.filename).name.startswith('tmp'))
        os.remove(log.filename)

    def test_context_manager(self):
        with LogWriter(prefix='tmpp', note='1st test') as log:
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
            self.assertTrue(Path(log.filename).name.startswith('tmp5'))
        os.remove(log.filename)

        os.environ['OSGAR_LOGS'] = 'tmp_dir'
        with LogWriter(prefix='tmp6', note='test_filename_after') as log:
            self.assertEqual(Path(log.filename).parent.name, 'tmp_dir')
        os.remove(log.filename)

        del os.environ['OSGAR_LOGS']
        with self.assertLogs(level=logging.WARNING):
            with LogWriter(prefix='tmp7', note='test_filename_after2') as log:
                self.assertTrue(Path(log.filename).name.startswith('tmp7'))
        os.remove(log.filename)

    def test_time_overflow(self):
        with LogWriter(prefix='tmp8', note='test_time_overflow') as log:
            log.start_time = datetime.now(timezone.utc) - timedelta(hours=1, minutes=30)
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
            with self.assertLogs(osgar.logger.__name__, logging.ERROR):
                with self.assertRaises(StopIteration):
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

    def test_large_invalid(self):
        header = osgar.logger.format_header(datetime(2019, 1, 1, 1))
        packet = osgar.logger.format_packet(1, b"\x00"*(2**16), timedelta())
        invalid = b"\xFF"*len(packet[-2]) # overwrite header of the last subpacket
        self.assertNotEqual(packet[-2], invalid)
        packet[-2] = invalid
        logdata = b"".join(header + packet)

        with tempfile.TemporaryDirectory(dir=".") as d:
            filename = Path(".") / d / "log"
            with open(filename, "wb") as f:
                f.write(logdata[:-1])
            with LogReader(filename) as l:
                with self.assertRaises(StopIteration),\
                     self.assertLogs(logger=osgar.logger.__name__, level=logging.ERROR):
                    next(l)

    def test_incomplete(self):
        header = osgar.logger.format_header(datetime(2019, 1, 1, 1))
        packet = osgar.logger.format_packet(1, b"\x00"*10, timedelta())
        logdata = b"".join(header + packet)

        with tempfile.TemporaryDirectory(dir=".") as d:
            filename = Path(".") / d / "log"
            with open(filename, "wb") as f:
                f.write(logdata[:-1])
            with LogReader(filename) as l:
                with self.assertLogs(osgar.logger.__name__, logging.ERROR):
                    with self.assertRaises(StopIteration):
                        next(l)


class LoggerIndexedTest(unittest.TestCase):

    def setUp(self):
        ref = os.environ.get('OSGAR_LOGS')
        if ref is None:
            os.environ['OSGAR_LOGS'] = "."

    def test_indexed_reader(self):
        note = 'test_indexed_reader'
        sample = [b'\x01', b'\x02', b'\x03']
        times = []
        with ExitStack() as at_exit:

            with LogWriter(prefix='tmpIndexed', note=note) as log:
                at_exit.callback(os.remove, log.filename)
                for a in sample:
                    t = log.write(1, a)
                    times.append(t)
                    time.sleep(0.001)

            with LogIndexedReader(log.filename) as log:
                dt, channel, data = log[0]
                assert data.decode('utf-8') == note, data
                for i in range(len(sample)):
                    dt, channel, data = log[i+1]
                    assert data == sample[i], data
                    assert dt == times[i], (dt, times[i])
                assert len(log) == len(sample) + 1, len(log)

    def test_indexed_large_block(self):
        data = bytes([x for x in range(100)]*1000)
        self.assertEqual(len(data), 100000)
        with ExitStack() as at_exit:
            with LogWriter(prefix='tmpIndexedLarge', note='test_large_block') as log:
                at_exit.callback(os.remove, log.filename)
                t1 = log.write(1, data)
                t2 = log.write(1, data[:0xFFFF])
                t3 = log.write(1, b'')
                t4 = log.write(1, b'ABC')
                t5 = log.write(1, data+data)  # multiple split

            with LogIndexedReader(log.filename) as log:
                self.assertEqual(len(log[1][2]), 100000)
                self.assertEqual(len(log[2][2]), 65535)
                self.assertEqual(len(log[3][2]), 0)
                self.assertEqual(len(log[4][2]), 3)
                self.assertEqual(len(log[5][2]), 200000)

    def test_time_overflow(self):
        with ExitStack() as at_exit:
            with patch('osgar.logger.datetime.datetime'):
                osgar.logger.datetime.datetime = TimeStandsStill(datetime(2020, 1, 21))
                with osgar.logger.LogWriter(prefix='tmp9', note='test_time_overflow') as log:
                    at_exit.callback(os.remove, log.filename)
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
            with LogIndexedReader(log.filename) as log:
                dt, channel, data = log[1]
                self.assertEqual(dt, timedelta(hours=0))
                dt, channel, data = log[2]
                self.assertEqual(dt, timedelta(hours=1))
                dt, channel, data = log[3]
                self.assertEqual(dt, timedelta(hours=2))
    #            dt, channel, data = next(log)
    #            self.assertEqual(dt, timedelta(hours=4))

    def test_large_blocks_with_time_overflow(self):
        with ExitStack() as at_exit:
            with patch('osgar.logger.datetime.datetime'):
                osgar.logger.datetime.datetime = TimeStandsStill(datetime(2019, 1, 1))
                with osgar.logger.LogWriter(prefix='tmpA', note='test_time_overflow with large blocks') as log:
                    at_exit.callback(os.remove, log.filename)
                    t1 = log.write(1, b'\x01'*100000)
                    self.assertEqual(t1, timedelta(0))
                    osgar.logger.datetime.datetime = TimeStandsStill(datetime(2019, 1, 1, 1))
                    t2 = log.write(1, b'\x02'*100000)
                    self.assertEqual(t2, timedelta(hours=1))
                    osgar.logger.datetime.datetime = TimeStandsStill(datetime(2019, 1, 1, 2))
                    t3 = log.write(1, b'\x03'*100000)
                    self.assertEqual(t3, timedelta(hours=2))
            with LogIndexedReader(log.filename) as log:
                dt, channel, data = log[1]
                self.assertEqual(dt, timedelta(hours=0))
                dt, channel, data = log[2]
                self.assertEqual(dt, timedelta(hours=1))
                dt, channel, data = log[3]
                self.assertEqual(dt, timedelta(hours=2))

    def test_no_eof(self):
        with patch('osgar.logger.datetime.datetime'):
            osgar.logger.datetime.datetime = TimeStandsStill(datetime(2019, 1, 1))
            with LogWriter(prefix='tmpEof', note='test_EOF') as log:
                filename = log.filename
                osgar.logger.datetime.datetime = TimeStandsStill(datetime(2019, 1, 1, 1))
                t1 = log.write(1, b'\x01'*100)
                osgar.logger.datetime.datetime = TimeStandsStill(datetime(2019, 1, 1, 2))
                t2 = log.write(1, b'\x02'*100)
                osgar.logger.datetime.datetime = TimeStandsStill(datetime(2019, 1, 1, 3))
                t3 = log.write(1, b'\x03'*100000)

        partial = filename + '.part'
        with open(filename, 'rb') as f_in, open(partial, 'wb') as f_out:
            f_out.write(f_in.read(100))
            f_out.flush()

            with LogIndexedReader(partial) as log:
                self.assertEqual(len(log), 1)
                dt, channel, data = log[0]
                with self.assertRaises(IndexError):
                    log[1]

                f_out.write(f_in.read(100))
                f_out.flush()
                log.grow()
                self.assertEqual(len(log), 2)
                dt, channel, data = log[1]
                self.assertEqual(data, b'\x01'*100)
                self.assertEqual(dt, timedelta(hours=1))
                with self.assertRaises(IndexError):
                    log[2]

                f_out.write(f_in.read())
                f_out.flush()

                log.grow()
                dt, channel, data = log[2]
                self.assertEqual(dt, timedelta(hours=2))
                self.assertEqual(data, b'\x02'*100)
                dt, channel, data = log[3]
                self.assertEqual(dt, timedelta(hours=3))
                self.assertEqual(len(log), 4)
                self.assertEqual(data, b'\x03'*100000)

        os.remove(partial)
        os.remove(filename)

    def test_large_blocks_with_growing_file(self):
        block_size = 100000 # fits into 2 packets, so 16 bytes overhead
        with ExitStack() as at_exit:
            with patch('osgar.logger.datetime.datetime'):
                osgar.logger.datetime.datetime = TimeStandsStill(datetime(2019, 1, 1))
                with osgar.logger.LogWriter(prefix='tmpA', note='') as log:
                    at_exit.callback(os.remove, log.filename)
                    t1 = log.write(1, b'\x01'*block_size)
                    self.assertEqual(t1, timedelta(0))
                    osgar.logger.datetime.datetime = TimeStandsStill(datetime(2019, 1, 1, 1))
                    t2 = log.write(1, b'\x02'*block_size)
                    self.assertEqual(t2, timedelta(hours=1))
                    osgar.logger.datetime.datetime = TimeStandsStill(datetime(2019, 1, 1, 2))
                    t3 = log.write(1, b'\x03'*block_size)
                    self.assertEqual(t3, timedelta(hours=2))

            partial = log.filename + '.part'
            with open(log.filename, 'rb') as f_in, open(partial, 'wb') as f_out:
                at_exit.callback(os.remove, partial)
                # log file starts with 16 byte header
                f_out.write(f_in.read(16))
                f_out.write(f_in.read(100))
                f_out.flush()

                with LogIndexedReader(partial) as log:
                    with self.assertRaises(IndexError):
                        log[0]
                    # add whole block plus 2 headers minus 100 already there
                    f_out.write(f_in.read(block_size-100+16))
                    f_out.flush()
                    log.grow()
                    self.assertEqual(len(log), 1)

                    # add a partial header only
                    f_out.write(f_in.read(4))
                    f_out.flush()
                    log.grow()
                    self.assertEqual(len(log), 1)

                    # add another block
                    f_out.write(f_in.read(block_size+16))
                    f_out.flush()
                    log.grow()

                    dt, channel, data = log[0]
                    self.assertEqual(dt, timedelta(hours=0))
                    dt, channel, data = log[1]
                    self.assertEqual(dt, timedelta(hours=1))

                    f_out.write(f_in.read())
                    f_out.flush()
                    log.grow()

                    dt, channel, data = log[2]
                    self.assertEqual(dt, timedelta(hours=2))

    def test_large_invalid(self):
        header = osgar.logger.format_header(datetime(2019, 1, 1, 1))
        packet = osgar.logger.format_packet(1, b"\x00"*(2**16), timedelta())
        invalid = b"\xFF"*len(packet[-2])
        self.assertNotEqual(packet[-2], invalid)
        packet[-2] = invalid
        logdata = b"".join(header + packet)

        with tempfile.TemporaryDirectory(dir=".") as d:
            filename = Path(".") / d / "log"
            with open(filename, "wb") as f:
                f.write(logdata[:-1])
            with self.assertLogs(logger=osgar.logger.__name__, level=logging.ERROR) as log:
                with LogIndexedReader(filename) as l:
                    with self.assertRaises(IndexError):
                        l[0]

    def test_incomplete(self):
        header = osgar.logger.format_header(datetime(2019, 1, 1, 1))
        packet = osgar.logger.format_packet(1, b"\x00"*10, timedelta())
        logdata = b"".join(header + packet)

        with tempfile.TemporaryDirectory(dir=".") as d:
            filename = Path(".") / d / "log"
            with open(filename, "wb") as f:
                f.write(logdata[:-1])
            with LogIndexedReader(filename) as l:
                with self.assertRaises(IndexError):
                    l[0]


# vim: expandtab sw=4 ts=4

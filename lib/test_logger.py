import unittest
import os
import time

from lib.logger import *


class LoggerTest(unittest.TestCase):
    
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

            __, stream_id, data = next(log.read_gen())
            self.assertEqual(INFO_STREM_ID, stream_id)

            t, stream_id, data = next(log.read_gen())
            self.assertEqual(stream_id, 10)
            self.assertEqual(data, b'\x01\x02\x02\x04')

            t, stream_id, data = next(log.read_gen())
            self.assertTrue(t.microseconds > 100)
            
            with self.assertRaises(StopIteration):
                __ = next(log.read_gen())

        with LogReader(filename) as log:
            for t, stream_id, data in log.read_gen(only_stream_id=10):
                self.assertEqual(stream_id, 10)
                self.assertEqual(data, b'\x01\x02\x02\x04')
                break

        os.remove(log.filename)

# vim: expandtab sw=4 ts=4

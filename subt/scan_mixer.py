"""
  Convert depth image to "lidar" scan
"""
import logging
from datetime import timedelta
import numpy as np

from osgar.node import Node

g_logger = logging.getLogger(__name__)


class ScanMixer(Node):

    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("scan")
        self.scan = None  # original lidar scan
        self.rs_scan = None  # RealSense RGBD converted scan from ROS
        self.rs_scan_time = timedelta(0)
        self.verbose = False

    def update(self):
        channel = super().update()
        assert channel in ["scan", "rs_scan"], channel

        if channel == 'rs_scan':
            self.rs_scan_time = self.time
        elif channel == 'scan':
            #assert len(self.scan) in [271, ], len(self.scan)  # Eduro=271
            if self.rs_scan is not None:
                max_delay = timedelta(milliseconds=500)
                rs_delay = self.time - self.rs_scan_time
                if rs_delay > max_delay:
                    g_logger.debug(f"rs_scan older than {max_delay}: {rs_delay}")
                    self.publish('scan', self.scan)
                else:
                    indexes = np.linspace(0, len(self.rs_scan), num=len(self.scan), endpoint=False, dtype=int)
                    rs_scan = np.asarray(self.rs_scan)
                    scan = np.asarray(self.scan)
                    # replace infinity with long distance
                    infinity = 30000
                    rs_scan[rs_scan == 0] = infinity
                    scan[scan == 0] = infinity
                    # resample
                    rs_scan = rs_scan[indexes]
                    scan = np.minimum(rs_scan, scan)
                    # replace long distance with infinity
                    scan[scan == infinity] = 0
                    self.publish('scan', scan.tolist())
            else:
                self.publish('scan', self.scan)
        else:
            assert False, channel  # unsupported channel

        return channel


if __name__ == "__main__":
    import argparse
    from unittest.mock import MagicMock
    from osgar import logger
    from osgar import bus
    from osgar.lib.serialize import deserialize

    parser = argparse.ArgumentParser(description='')
    parser.add_argument('logfile', help='filename of stored file')
    rs_scan = "rosmsg.scan"
    scan = "lidar.scan"
    stream_names = [rs_scan, scan]

    bus = bus.Bus(MagicMock())
    tester = bus.handle('tester')
    tester.register("scan", "rs_scan")
    mixer = ScanMixer(config={}, bus=bus.handle('mixer'))
    bus.connect("tester.scan", "mixer.scan")
    bus.connect("tester.rs_scan", "mixer.rs_scan")
    bus.connect("mixer.scan", "tester.mixed")
    mixer.start()

    args = parser.parse_args()
    stream_ids = [logger.lookup_stream_id(args.logfile, stream) for stream in stream_names]
    id2name = { stream_id: name for stream_id, name in zip(stream_ids, ["rs_scan", "scan"]) }
    with logger.LogReader(args.logfile, only_stream_id=stream_ids) as log:
        for timestamp, stream_id, bytes_data in log:
            data = deserialize(bytes_data)
            tester.publish(id2name[stream_id], data)
            if id2name[stream_id] == "scan":
                t, s, d = tester.listen()

    mixer.request_stop()
    mixer.join()

# vim: expandtab sw=4 ts=4

"""
  SICK Tim571 LIDAR Driver
"""

from threading import Thread

from osgar.bus import BusShutdownException


STX = b'\x02'
ETX = b'\x03'


class SICKLidar(Thread):
    def __init__(self, config, bus):
        Thread.__init__(self)
        self.setDaemon(True)

        self.bus = bus
        self.buf = b''
        self.sleep = config.get('sleep')

    @staticmethod
    def parse_raw_data(raw_data):
        """Parse scan data for TiM571 and TiM551 SICK LIDARs"""
        data = raw_data.split()
        assert len(data) in [26, 854, 846, 583, 1663], len(data)
        assert data[1] == b'LMDscandata', data[:2]
        timestamp = int(data[9], 16)  # TODO verify
        freq = int(data[16], 16)
        if len(data) == 26:  # empty scan
            return [], None
        assert freq == 1500, freq  # TiM 15Hz
        assert data[20] == b'DIST1', data[20]
        resolution = int(data[24], 16)
        assert resolution in [3333, 10000], resolution
        scan_size = int(data[25], 16)
        assert scan_size in [811, 271], scan_size
        scan_start = 26
        scan_end = scan_start + scan_size
        dist = [int(x, 16) for x in data[scan_start:scan_end]]

        remission = None
        # should have have scan remission?
        rssi_index = scan_end + 1
        if rssi_index < len(data) and data[rssi_index] == b'RSSI1':
            assert int(data[rssi_index + 2], 16) == 0, data[rssi_index + 2]
            angular_step = int(data[rssi_index + 4], 16)
            assert angular_step == resolution, (angular_step, resolution)

            amount = int(data[rssi_index + 5], 16)
            assert amount == scan_size, (amount, scan_size)
            rssi_index += 6
            remission = [int(x,16) for x in data[rssi_index:rssi_index + amount]]

        return dist, remission

    def process_packet(self, packet):
        if packet.startswith(STX) and packet.endswith(ETX):
            return self.parse_raw_data(packet)
        return None

    def split_buffer(self, data):
        i = data.find(ETX)
        if i >= 0:
            return data[i + 1:], data[:i + 1]
        return data, b''

    def process_gen(self, data):
        self.buf, packet = self.split_buffer(self.buf + data)
        while len(packet) > 0:
            ret = self.process_packet(packet)
            if ret is not None:
                yield ret
            # now process only existing (remaining) buffer
            self.buf, packet = self.split_buffer(self.buf)  

    def run(self):
        try:
            self.bus.publish('raw', STX + b'sRN LMDscandata' + ETX)
            while True:
                packet = self.bus.listen()
                dt, __, data = packet
                for out in self.process_gen(data):
                    assert out is not None
                    scan, remission = out
                    if len(scan) > 0:
                        self.bus.publish('scan', scan)
#                    print(dt, [x for x in zip(scan, remission)])
#                    print()
                    if self.sleep is not None:
                        self.bus.sleep(self.sleep)
                    self.bus.publish('raw', STX + b'sRN LMDscandata' + ETX)
        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()


# vim: expandtab sw=4 ts=4

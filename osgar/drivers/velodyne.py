"""
  Parse Velodyne VLP-16 data
"""
import struct
import math

from osgar.node import Node

LASER_ANGLES = [-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15]
NUM_LASERS = 16


def parse_packet(data, offset_step=100):
    assert len(data) == 1206, len(data)
    assert offset_step % 100 == 0, offset_step  # must be divisible by 100
    timestamp, factory = struct.unpack_from("<IH", data, offset=1200)
    assert factory == 0x2237, hex(factory)  # 0x22=VLP-16, 0x37=Strongest Return
#    time = timestamp/1000000.0
#    if self.time is not None:
#        lost_packets = int(round((time - self.time)/EXPECTED_PACKET_TIME)) - 1
#    else:
#        lost_packets = 0
#    self.time = time
#    if lost_packets > 0 and (self.last_blocked is None or self.time > self.last_blocked + EXPECTED_SCAN_DURATION):
#        self.last_blocked = self.time + EXPECTED_SCAN_DURATION
#        self.scan_index += 1
#        print("DROPPED index", self.scan_index)
#    if self.last_blocked is not None and self.time < self.last_blocked:
#        return  # to catch up-to-date packets again ...

    ret = []
    for offset in range(0, 1200, offset_step):  # 100 bytes per one reading
        flag, azi = struct.unpack_from("<HH", data, offset)
        assert flag == 0xEEFF, hex(flag)
        azimuth = azi/100.0
        # H-distance (2mm step), B-reflectivity (0
        arr = struct.unpack_from('<' + "HB"*32, data, offset + 4)
        dist = []
        for i in range(NUM_LASERS):
            dist.append(arr[i*2] * 0.002)

        # so now we have azimuth and NUM_LASERS distance readings
        for d, beta_deg in zip(dist, LASER_ANGLES):
            beta = math.radians(beta_deg)
            x = d * math.cos(azimuth) * math.cos(beta)
            y = d * math.sin(azimuth) * math.cos(beta)
            z = d * math.sin(beta)
            ret.append([x, y, z])
    return ret


class Velodyne(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        self.offset_step = config.get('offset_step', 200)  # skip every second packet (we need 1deg resolution input 0.4)
        assert self.offset_step % 100 == 0, self.offset_step  # must be divisible by 100

    def update(self):
        channel = super().update()
        assert channel == 'raw', channel
        self.publish('xyz', parse_packet(self.raw, offset_step=self.offset_step))

# vim: expandtab sw=4 ts=4

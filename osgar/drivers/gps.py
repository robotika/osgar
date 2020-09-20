"""
  GPS Driver
"""

from threading import Thread
import struct

from osgar.bus import BusShutdownException


INVALID_COORDINATES = [None, None]
BIN_PREAMBULE = bytes([0xB5, 0x62])


def checksum(s):
    sum = 0
    for ch in s:
        sum ^= ch
    return b"%02X" % (sum)


def str2ms(s):
    'convert DDMM.MMMMMM string to arc milliseconds(int)'
    if s == b'':  # unknown position
        return None
    dm, frac = (b'0000' + s).split(b'.')
    try:
        return round((int(dm[:-2]) * 60 + float(dm[-2:] + b'.' + frac)) * 60000)
    except Exception as e:
        print(e)
        return None


def parse_line(line):
    assert line.startswith(b'$GNGGA') or line.startswith(b'$GPGGA'), line
    if checksum(line[1:-3]) != line[-2:]:
        print('Checksum error!', line, checksum(line[1:-3]))
        return [None, None]
    s = line.split(b',')
    coord = [str2ms(s[4]), str2ms(s[2])]
    return coord


def parse_bin(data):
    assert data.startswith(BIN_PREAMBULE), data
    c, i, size = struct.unpack_from('<BBH', data, 2)
    assert len(data) == size + 8, (len(data), size + 8)
    assert c == 1, c  # class = 1 ... NAVigation messages
    assert i in [3, 0x30, 6, 7, 0x34, 0x35, 1, 2, 0x13, 0x14, 4, 0x11, 0x12, 0x20, 0x23, 0x24, 0x21,
                0x26, 0x22, 0x9, 0x3B, 0x3C, 0x39, 0x61, ], hex(i)  # ID
#    print(hex(i))

    # TODO verify checksum!
    payload = data[6:-2]

    # 31.18.20 UBX-NAV-STATUS (0x01 0x03)
    # 31.18.20.1 Receiver Navigation Status
    # Receiver Navigation Status
    if i == 0x03:
        fix = payload[4]
        #assert fix in [2, 3], fix  # 2D, 3D
        if fix not in [2, 3]:
            print("GPS no fix!")

    # 31.18.21 UBX-NAV-SVINFO (0x01 0x30)
    # 31.18.21.1 Space Vehicle Information
    # Information about satellites used or visible
    if i == 0x30:
        return None

    # 31.18.15 UBX-NAV-RELPOSNED (0x01 0x3C)
    # 31.18.15.1 Relative Positioning Information in NED frame
    if i == 0x3C:
        assert len(payload) == 40, len(payload)
        assert payload[0] == 0, payload[0]  # version
        iTOW, rel_pos_north_cm, rel_pos_east_cm = struct.unpack_from('<Iii', payload, 4)
        accN, accE = struct.unpack_from('<II', payload, 24)
        return {'rel_position': [rel_pos_east_cm, rel_pos_north_cm]}

    # 31.18.30 UBX-NAV-VELNED (0x01 0x12)
    # 31.18.30.1 Velocity Solution in NED
    if i == 0x12:
        assert len(payload) == 36, len(payload)
        iTOW, velN, velE, velD = struct.unpack_from('<Iiii', payload, 0)
        gSpeed, heading, sAcc, cAcc = struct.unpack_from('<IiII', payload, 20)
        #print(gSpeed, heading/1e5, sAcc, cAcc/1e5)
        return None  # do not integrate for now

    # 31.18.14 UBX-NAV-PVT (0x01 0x07)
    # 31.18.14.1 Navigation Position Velocity Time Solution
    if i == 0x07:
        assert len(payload) == 92, len(payload)
        gSpeed, heading, sAcc, cAcc = struct.unpack_from('<IiII', payload, 60)
        #print('xx', gSpeed, heading/1e5, sAcc, cAcc/1e5)
        return None  # do not integrate for now


def split_buffer(data):
    # in dGPS there is a block of binary data so stronger selection is required
    start_nmea = max(data.find(b'$GNGGA'), data.find(b'$GPGGA'))
    start_bin = data.find(BIN_PREAMBULE)
    if start_nmea < 0 or (0 <= start_bin < start_nmea):
        if start_bin < 0 or start_bin + 8 >= len(data):
            return data, b''
        else:
            # extract binary data: preambule, class, ID, len, payload, checksum
            c, i, size = struct.unpack_from('<BBH', data, start_bin + 2)
            end = start_bin + 6 + size + 2
            if end >= len(data):
                return data, b''
            return data[end:], data[start_bin:end]

    start = start_nmea
    end = data[start:-2].find(b'*')
    if end < 0:
        return data, b''
    return data[start+end+3:], data[start:start+end+3]


class GPS(Thread):
    def __init__(self, config, bus):
        bus.register('position')
        Thread.__init__(self)
        self.setDaemon(True)

        self.bus = bus
        self.buf = b''

    def process_packet(self, line):
        if line.startswith(b'$GNGGA') or line.startswith(b'$GPGGA'):
            coords = parse_line(line)
            return {'position': coords}
        elif line.startswith(BIN_PREAMBULE):
            return parse_bin(line)
        return None

    def process_gen(self, data):
        self.buf, packet = split_buffer(self.buf + data)
        while len(packet) > 0:
            ret = self.process_packet(packet)
            if ret is not None:
                for k, v in ret.items():
                    yield k, v
            self.buf, packet = split_buffer(self.buf)  # i.e. process only existing buffer now

    def run(self):
        try:
            while True:
                packet = self.bus.listen()  # there should be some timeout and in case of failure send None
                dt, __, data = packet
                for name, out in self.process_gen(data):
                    assert out is not None
                    self.bus.publish(name, out)
        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()


def print_output(packet):
    print(packet)

# vim: expandtab sw=4 ts=4

"""
  Parse 3DM-GX5-25 data
"""
# www.microstrain.com/sites/default/files/3dm-gx5-25_dcp_manual_8500-0065.pdf
import struct
import math
import datetime

from osgar.node import Node


def verify_checksum(packet):
    # 16-bit Fletcher Checksum Algorithm
    ch1, ch2 = 0, 0
    for b in packet[:-2]:
        ch1 += b
        ch2 += ch1
    ret = ((ch1 << 8) & 0xFF00) + (ch2 & 0xFF)
    return packet[-2] == (ch1 & 0xFF) and packet[-1] == (ch2 & 0xFF)


def get_packet(buf):
    try:
        i = buf.index(b'\x75\x65')
    except ValueError:
        return None, buf
    try:
        packet_size = buf[i + 3] + 4 + 2  # + header size + checksum
        if i + packet_size > len(buf):
            return None, buf
    except IndexError:
        return None, buf
    packet = buf[i:i + packet_size]
    if not verify_checksum(packet):
        print('LORD Checksum error:', packet.hex())
        packet = None
    return packet, buf[i + packet_size:]


def parse_packet(packet, verbose=False):
    assert packet.startswith(b'\x75\x65'), packet.hex()
    assert len(packet) > 3, len(packet)
    packet_size = packet[3] + 4 + 2 # + header size + checksum
    assert len(packet) == packet_size, (len(packet), packet_size)
    desc = packet[2]

    # IMU Data Set (0x80)
    assert desc in [0x80, 0x81, 0x82], hex(desc)
    i = 4
    acc, gyro, euler, quat, gps = None, None, None, None, None
    lat, lon, sat_num, gps_time = None, None, None, None
    while i < packet_size - 2:
        field_length = packet[i]
        cmd = packet[i + 1]

        if desc == 0x80 and cmd == 0x04:
            # Scaled Accelerometer Vector
            assert field_length == 14, field_length
            acc = struct.unpack_from('>fff', packet, i + 2)
            if verbose:
                print('acc', acc)
        elif desc == 0x80 and cmd == 0x05:
            # Scaled Gyro Vector (0x80, 0x05)
            assert field_length == 14, field_length
            gyro = struct.unpack_from('>fff', packet, i + 2)
            if verbose:
                print('gyro', gyro)
        elif desc == 0x80 and cmd == 0x06:
            # Scaled Magnetometer Vector (0x80, 0x06)
            assert field_length == 14, field_length
            mag = struct.unpack_from('>fff', packet, i + 2)
            if verbose:
                print('mag', mag)
        elif desc == 0x80 and cmd == 0x0C:
            # CF Euler Angles (0x80, 0x0C)
            # This value is produced by the Complementary Filter fusion algorithm.
            pass

        # GPS related messages
        elif desc == 0x81 and cmd == 0x03:
            # LLH Position (0x81, 0x03)
            assert field_length == 44, field_length
            row = struct.unpack_from('>ddddffH', packet, i + 2)
            lat, lon, height_elipsoid, height_msl, hori, vert, flags = row
            if verbose:
                print('GPS', (lat, lon))
        elif desc == 0x81 and cmd == 0x08:
            # UTC Time (0x81, 0x08)
            assert field_length == 15, field_length
            msg = struct.unpack_from('>HBBBBBIH', packet, i + 2)
            year, month, day, hour, minute, second, millisecond, flags = msg
            gps_time = datetime.datetime(year, month, day, hour, minute, second, millisecond)
            if verbose:
                print('UTCTime', year, month, day, hour, minute, second, millisecond, flags)
        elif desc == 0x81 and cmd == 0x0b:
            # GNSS Fix Information (0x81, 0x0B)
            assert field_length == 8, field_length
            msg = struct.unpack_from('>BBHH', packet, i + 2)
            fix_type, sat_num, fix_flags, valid = msg
            if verbose:
                print('GPSFix', msg)

        elif desc == 0x82 and cmd == 0x03:
            # Orientation, Quaternion (0x82, 0x03)
            q0, q1, q2, q3, valid = struct.unpack_from('>ffffH', packet, i + 2)
            if valid == 0x1:
                quat = q0, q1, q2, q3
            if verbose:
                print('quaterion', q0, q1, q2, q3, valid)
        elif desc == 0x82 and cmd == 0x05:
            # Orientation, Euler Angles (0x82, 0x05)
            roll, pitch, yaw, valid = struct.unpack_from('>fffH', packet, i + 2)
            if valid == 0x1:
#                euler = yaw, pitch, roll
#                euler = math.pi - yaw, pitch, roll  # hacked, guessed
                euler = math.pi - yaw, pitch, math.pi - roll  # hacked2, guessed for K2
            if verbose:
                print('euler', roll, pitch, yaw, valid)
        elif desc == 0x82 and cmd == 0x0A:
            # Attitude Uncertainty, Euler Angles (0x82, 0x0A)
            roll, pitch, yaw, valid = struct.unpack_from('>fffH', packet, i + 2)
            if verbose:
                print('euler-uncertain', roll, pitch, yaw, valid)
        elif desc == 0x82 and cmd == 0x0D:
            # Linear Acceleration (0x82, 0x0D)
            # Filter-Compensated Linear Acceleration Data (gravity vector removed)
            x, y, z, valid = struct.unpack_from('>fffH', packet, i + 2)
            if verbose:
                print('lin', x, y, z, valid)
        elif desc == 0x82 and cmd == 0x0E:
            # Compensated Angular Rate (0x82, 0x0E)
            x, y, z, valid = struct.unpack_from('>fffH', packet, i + 2)
            if verbose:
                print('ang', x, y, z, valid)
        elif desc == 0x82 and cmd == 0x14:
            # Heading Update Source information expressed in the sensor frame.
            heading, uncertainty, source, valid = struct.unpack_from('>ffHH', packet, i + 2)
            if verbose:
                print('heading', heading, uncertainty, source, valid)
        else:
            assert False, (hex(desc), hex(cmd))

        i += field_length

    if lat is not None:
        gps = (lat, lon), sat_num, gps_time

    return acc, gyro, euler, quat, gps


class LordIMU(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('orientation', 'rotation', 'gps_position')
        self._buf = b''
        self.raw = None  # not automatically defined yet
        self.verbose = False

    def on_raw(self, data):
        self.raw = data
        self._buf += self.raw
        while True:
            packet, self._buf = get_packet(self._buf)
            if packet is None:
                break
            acc, gyro, euler, quat, gps = parse_packet(packet, self.verbose)
            if euler is not None:
                yaw, pitch, roll = euler
                # This is a three component vector containing the Roll, Pitch and Yaw angles 
                # in radians.
                self.publish('rotation', 
                        [int(round(math.degrees(x) * 100)) for x in [yaw, pitch, roll]])
            if quat is not None:
                q0, q1, q2, q3 = quat
                self.publish('orientation', [q3, q0, q1, q2])
            if gps is not None:
                # TODO check for GPS fix
                lat, lon = gps[0]
                x = int(lon * 3600000)  # deg to milisec
                y = int(lat * 3600000)
                self.publish('gps_position', [x, y])

# vim: expandtab sw=4 ts=4

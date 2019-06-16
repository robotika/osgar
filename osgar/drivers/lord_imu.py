"""
  Parse 3DM-GX5-25 data
"""
# www.microstrain.com/sites/default/files/3dm-gx5-25_dcp_manual_8500-0065.pdf
import struct
import math

from osgar.node import Node


def verify_checksum(packet):
    # 16-bit Fletcher Checksum Algorithm
    ch1, ch2 = 0, 0
    for b in packet[:-2]:
        ch1 += b
        ch2 += ch1
    ret = ((ch1 << 8) & 0xFF00) + (ch2 & 0xFF)
    assert packet[-2] == (ch1 & 0xFF), (hex(packet[-2]), hex(ch1 & 0xFF))
    assert packet[-1] == (ch2 & 0xFF), (hex(packet[-1]), hex(ch2 & 0xFF))
    return ret


def get_packet(buf):
    try:
        i = buf.index(b'\x75\x65')
    except ValueError:
        return None, buf
    packet_size = buf[i + 3] + 4 + 2  # + header size + checksum
    if i + packet_size > len(buf):
        return None, buf
    packet = buf[i:i + packet_size]
    verify_checksum(packet)
    return packet, buf[i + packet_size:]


def parse_packet(packet, verbose=False):
    assert packet.startswith(b'\x75\x65'), packet.hex()
    assert len(packet) > 3, len(packet)
    packet_size = packet[3] + 4 + 2 # + header size + checksum
    assert len(packet) == packet_size, (len(packet), packet_size)
    desc = packet[2]

    # IMU Data Set (0x80)
    assert desc in [0x80, 0x82], hex(desc)
    i = 4
    acc, gyro, euler, quat = None, None, None, None
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
        elif desc == 0x82 and cmd == 0x03:
            # Orientation, Quaternion (0x82, 0x03)
            q0, q1, q2, q3, valid = struct.unpack_from('>ffffH', packet, i + 2)
            if valid == 0x1:
                quat = q0, q1, q2, q3
            if verbose:
                print('quaterion', q0, q1, q2, q3, valid)
        elif desc == 0x82 and cmd == 0x0A:
            # Attitude Uncertainty, Euler Angles (0x82, 0x0A)
            roll, pitch, yaw, valid = struct.unpack_from('>fffH', packet, i + 2)
            if valid == 0x1:
                euler = yaw, pitch, roll
            if verbose:
                print('euler', roll, pitch, yaw, valid)
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

    return acc, gyro, euler, quat


class LordIMU(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        self._buf = b''
        self.raw = None  # not automatically defined yet
        self.verbose = False

    def update(self):
        channel = super().update()
        assert channel == 'raw', channel
        self._buf += self.raw 
        while True:
            packet, self._buf = get_packet(self._buf)
            if packet is None:
                break
            acc, gyro, euler, quat = parse_packet(packet, self.verbose)
            if euler is not None:
                yaw, pitch, roll = euler
                # This is a three component vector containing the Roll, Pitch and Yaw angles 
                # in radians.
                self.publish('rotation', 
                        [int(round(math.degrees(x) * 100)) for x in [yaw, pitch, roll]])
            if quat is not None:
                self.publish('orientation', list(quat))
        
# vim: expandtab sw=4 ts=4

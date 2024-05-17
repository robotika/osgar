"""
  Handle data from VanJee Lidar WLR-719C (4 lines)
"""
import struct
import math

from osgar.node import Node
from osgar.bus import BusShutdownException


class VanJeeLidar(Node):
    def __init__(self, config, bus):
        bus.register('raw', 'xyz', 'scan', 'scan10', 'scanup')
        super().__init__(config, bus)
        self.last_frame = None  # not defined
        self.points = []
        self.debug_arr = []
        self.verbose = False

    def on_raw(self, data):
        assert len(data) in [34, 1384], len(data)
        if len(data) != 1384:
            return
        assert data[:2] == b'\xFF\xAA', data[:2].hex()
        length = struct.unpack_from('>H', data, 2)[0]
        assert length == 1380, length
        frame = struct.unpack_from('>H', data, 4)[0]
        if self.last_frame is not None:
            assert (self.last_frame + 1) & 0xFFFF == frame, (self.last_frame, frame)
        self.last_frame = frame
        assert data[6:8] == bytes.fromhex('0000'), data[6:10].hex()  # reserved
        # reserved [8:10]  - variable
        assert data[11:14] == bytes.fromhex('02 190C'), data[11:14].hex()
        # reserved [14:16]
        assert data[16:18] == bytes.fromhex('01 04'), data[16:18].hex()
        bank, motor_speed = struct.unpack_from('>BH', data, 18)
        assert 1 <= bank <= 16, bank
#        assert 38000 <= motor_speed <= 39300, motor_speed  # disabled as it can be easily "of of range"
        if bank == 1:
            self.points = []  # reset last scan

        assert data[-2:] == b'\xEE\xEE', data[-2:].hex()
        self.points.extend(struct.unpack_from('>' + 'BH' * 450, data, 21))
        if bank == 16:
            if len(self.points) == 2*7200:
                scan = self.points[5::8]  # -10, -5, 0, 0.3
                self.publish('scan', scan)
                scan10 = self.points[1::8]  # -10, -5, 0, 0.3
                self.publish('scan10', scan10)
                scanup = self.points[7::8]  # -10, -5, 0, 0.3
                self.publish('scanup', scanup)
                if self.verbose:
                    self.debug_arr.append((self.time.total_seconds(), self.points[:]))
            else:
                print(self.time, f'Incomplete scan - only {len(self.points)//2} points out of 7200')  # intensity, dist in mm

    def run(self):
        magic_packet = b'\xFF\xAA\x00\x1E\x00\x00\x00\x00\x00\x00\x01\x01\x19\x0C\x00\x00\x00\x00\x00\x00\x00\x00\x05\x03\x00\x00\x00\x00\x00\x00\x00\x0D\xEE\xEE'
        self.publish('raw', magic_packet)
        try:
            while True:
                self.update()
        except BusShutdownException:
            pass

    def draw(self):
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Slider

        fig, ax = plt.subplots()

        i = 0
        line10, = ax.plot(self.debug_arr[i][1][1::8], 'o', linewidth=2, label='-10 deg')
        line5, = ax.plot(self.debug_arr[i][1][3::8], 'o', linewidth=2, label='-5 deg')
        line0, = ax.plot(self.debug_arr[i][1][5::8], 'o', linewidth=2, label='0 deg')
        line03, = ax.plot(self.debug_arr[i][1][7::8], 'o', linewidth=2, label='0.3 deg')

        # adjust the main plot to make room for the sliders
        fig.subplots_adjust(bottom=0.25)
        plt.xlabel('angle index')
        plt.ylabel('distance (mm)')

        axfreq = fig.add_axes([0.1, 0.1, 0.8, 0.03])
        freq_slider = Slider(
            ax=axfreq,
            label='Frame',
            valmin=0,
            valmax=len(self.debug_arr)-1,
            valinit=i,
            valstep=1,
        )

        def update(val):
            i = int(freq_slider.val)
            line10.set_ydata(self.debug_arr[i][1][1::8])
            line5.set_ydata(self.debug_arr[i][1][3::8])
            line0.set_ydata(self.debug_arr[i][1][5::8])
            line03.set_ydata(self.debug_arr[i][1][7::8])

        freq_slider.on_changed(update)
        fig.legend()
        plt.show()

#######################################################
# External utilities based on 3rd party code snippets
import socket


#from utils_719c import checksum
def checksum(data):
    result = 0
    for b in data[2:-4]:
        result ^= b
    return result


def handle_response(data):
    (frame_header, frame_length, frame_number, timestanmp, check_type, frame_type,
     device_type, reserved_a, reserved_b, command, subcommand, param1, param2,
     ip0, ip1, ip2, ip3, port, mac0, mac1, mac2, mac3, mac4, mac5,
     check_code, end_of_frame) = struct.unpack(
        '>HHHIBBHIIBBBBBBBBHBBBBBBHH', data)
    assert (frame_header == 0xFFAA)
    assert (end_of_frame == 0xEEEE)
    assert (check_code == checksum(data))
    assert (frame_length == 0x26)
    assert (frame_type == 0x02)
    assert (device_type == 0x190C)
    assert (command == 0x05)
    assert (subcommand == 0x01)
    assert (param1 == 0)
    assert (param2 == 0)
    print(f'address: {ip0}.{ip1}.{ip2}.{ip3}:{port} ' +
          f'[{mac0}:{mac1}:{mac2}:{mac3}:{mac4}:{mac5}]')


def get_ip_719c(broadcast_address, own_port, lidar_port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    # Without this bind, the broadcast does not work.
    sock.bind((broadcast_address, own_port))

    own_ip = bytes([int(x) for x in broadcast_address.split('.')])
    cmd = bytearray(
        b'\xFF\xAA' +
        b'\x00\x20' +
        b'\x00\x00' +
        b'\x00\x00\x00\x00' +
        b'\x01' +
        b'\x01' +
        b'\x19\x0c' +
        b'\x00\x00\x00\x00\x00\x00\x00\x00' +
        b'\x05' +
        b'\x01' +
        b'\x00' +
        b'\x00' +
        own_ip +
        struct.pack('>H', own_port) +
        b'\x00\x00' +
        b'\xEE\xEE')
    cmd[-3] = checksum(cmd)

    assert (len(own_ip) == 4), len(own_ip)
    assert (len(cmd) == 36), len(cmd)

    sock.sendto(cmd, (broadcast_address, lidar_port))
    # We need to bind to '' to be able to receive a broadcast to 255.255.255.255.
    # However, the bind() above prevents us from doing it with the existing
    # socket. So let's quickly (!) create a new one.
    sock.close()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.bind(('', own_port))
    sock.settimeout(5)
    try:
        data = sock.recv(1024)
        handle_response(data)
    except TimeoutError:
        print('No device found.')


def address_parts(addr):
    return [int(a) for a in addr.split('.')]


def set_ip_719c(lidar_new_address, lidar_new_mask, lidar_new_gateway, lidar_old_address, lidar_port):
    assert (lidar_new_address is not None)
    assert (lidar_new_gateway is not None)
    assert (lidar_new_address != lidar_new_gateway)

    cmd = bytearray(
        b'\xFF\xAA' +
        b'\x00\x2E' +
        b'\x00\x00' +
        b'\x00\x00\x00\x00' +
        b'\x01' +
        b'\x01' +
        b'\x19\x0c' +
        b'\x00\x00\x00\x00\x00\x00\x00\x00' +
        b'\x02\x14\x00\x00' +
        b'\x00\x00\x00\x00' +
        b'\x00\x00\x00\x00' +
        b'\x00\x00\x00\x00' +
        b'\x00\x00\x00\x00\x00\x00\x00\x00' +
        b'\x00\x00' +
        b'\xEE\xEE'
    )

    cmd[26:30] = address_parts(lidar_new_address)
    cmd[30:34] = address_parts(lidar_new_mask)
    cmd[34:38] = address_parts(lidar_new_gateway)
    cmd[-3] = checksum(cmd)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.sendto(cmd, (lidar_old_address, lidar_port))


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(help='sub-command help', dest='command')
    subparsers.required = True
    parser_get = subparsers.add_parser('get', help='get lidar IP address and port')
    parser_get.add_argument('--broadcast-address', default='192.168.0.255')
    parser_get.add_argument('--own-port', default=6061, type=int)
    parser_get.add_argument('--lidar-port', default=6060, type=int)

    parser_set = subparsers.add_parser('set', help='set lidar new IP address')
    parser_set.add_argument('--lidar-old-address', default='192.168.0.2')
    parser_set.add_argument('--lidar-port', default=6060, type=int)
    parser_set.add_argument('--lidar-new-address', required=True)
    parser_set.add_argument('--lidar-new-mask', default='255.255.255.0')
    parser_set.add_argument('--lidar-new-gateway', required=True)
    args = parser.parse_args()

    if args.command == 'get':
        get_ip_719c(args.broadcast_address, args.own_port, args.lidar_port)
    elif args.command == 'set':
        set_ip_719c(args.lidar_new_address, args.lidar_new_mask, args.lidar_new_gateway,
                    args.lidar_old_address, args.lidar_port)
    else:
        assert 0, f'Unknown command "{args.command}"'

# vim: expandtab sw=4 ts=4

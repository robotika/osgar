"""
  Wrapper for CAN-serial communication and control of CAN bridge
"""

import serial
import struct
from threading import Thread
from collections import OrderedDict

from osgar.bus import BusShutdownException


CAN_BRIDGE_READY = b'\xfe\x10'  # CAN bridge is ready to accept configuration commands
CAN_BRIDGE_SYNC = b'\xFF'*10    # CAN bridge synchronization bytes
CAN_BRIDGE_START = b'\xfe\x31'  # start bridge

CAN_SPEED = OrderedDict([
                ('10k', b'\xfe\x50'),
                ('20k', b'\xfe\x51'),
                ('50k', b'\xfe\x52'),
                ('125k', b'\xfe\x53'),
                ('250k', b'\xfe\x54'),
                ('500k', b'\xfe\x55'),
                ('800k', b'\xfe\x56'),
                ('1M', b'\xfe\x57')
            ])

def CAN_packet(msg_id, data):
    header = [(msg_id>>3) & 0xff, (msg_id<<5) & 0xe0 | (len(data) & 0xf)]
    return bytes(header + data)


def print_packet(data, dbc = {}):
    assert len(data) >= 2, len(data)

    header = data[:2]
    rtr = (header[1] >> 4) & 0x1  # Remote transmission request
    size = (header[1]) & 0x0f
    if rtr:
        return [hex(x) for x in header]
    else:
        assert len(data) == 2 + size, (len(data), 2 + size)
        msg_id = ((data[0]) << 3) | (((data[1]) >> 5) & 0x1f)
        if msg_id in dbc:
            return hex(msg_id), [hex(x) for x in struct.unpack(dbc[msg_id], data[2:])]
        else:
            return hex(msg_id), [hex(x) for x in data[2:]]


class CANSerial(Thread):
    def __init__(self, config, bus):
        Thread.__init__(self)
        self.setDaemon(True)

        self.bus = bus
        self.buf = b''

        speed = config.get('speed', '1M')  # default 1Mbit
        if speed not in CAN_SPEED:
            raise ValueError('unsupported speed: {}\nUse:{}'.format(speed, list(CAN_SPEED.keys())))
        self.can_speed_cmd = CAN_SPEED[speed]
        self.is_canopen = config.get('canopen', False)
        self.can_bridge_initialized = False

    @staticmethod
    def split_buffer(data):
        # skip 0xFF prefix bytes (CAN bridge control bytes)
        data = data.lstrip(b'\xff')

        if len(data) >= 2:
            # see https://en.wikipedia.org/wiki/CAN_bus
            header = data[:2]
            rtr = (header[1] >> 4) & 0x1  # Remote transmission request
            size = (header[1]) & 0x0f
            if rtr:
                return data[2:], header
            elif len(data) >= 2 + size:
                return data[2+size:], data[:2+size]
        return data, b''  # no complete packet available yet

    def process_packet(self, packet):
        if packet == CAN_BRIDGE_READY:
            self.bus.publish('raw', CAN_BRIDGE_SYNC)
            self.bus.publish('raw', self.can_speed_cmd)
            self.bus.publish('raw', CAN_BRIDGE_START)
            self.can_bridge_initialized = True
            if self.is_canopen:
                self.bus.publish('raw', CAN_packet(0, [1, 0]))  # operational
            return None
        return packet

    def process_gen(self, data):
        self.buf, packet = self.split_buffer(self.buf + data)
        while len(packet) > 0:
            ret = self.process_packet(packet)
            if ret is not None:
                yield ret
            self.buf, packet = self.split_buffer(self.buf)  # i.e. process only existing buffer now

    def run(self):
        try:
            while True:
                dt, channel, data = self.bus.listen()
                if channel == 'raw':
                    if len(data) > 0:
                        for packet in self.process_gen(data):
                            self.bus.publish('can', packet)
                elif channel == 'can':
                    if self.can_bridge_initialized:
                        # at the moment is can serial just forwarding raw packets
                        self.bus.publish('raw', data)
                    else:
                        print('CAN bridge not initialized yet!')
                else:
                    assert False, channel  # unsupported input channel
        except BusShutdownException:
            pass

    def request_stop(self):
        if self.is_canopen:
            self.bus.publish('raw', CAN_packet(0, [128, 0]))  # pre-operational
        self.bus.shutdown()


if __name__ == "__main__":
    import argparse
    from osgar.logger import LogReader, lookup_stream_id
    from osgar.lib.serialize import deserialize

    parser = argparse.ArgumentParser(description='Parse CAN stream messages')
    parser.add_argument('logfile', help='filename of stored file')
    parser.add_argument('--stream', help='stream ID or name', default='can.can')
    args = parser.parse_args()

    dbc = {
            0x200: 'H',     # status
            0x201: 'HHHH',  # wheels
            0x202: 'HHHH',  # drive status
            0x203: 'HHHH',  # zero steering
            0x204: 'HHBBH'  # user input
        }

    stream_id = lookup_stream_id(args.logfile, args.stream)
    with LogReader(args.logfile) as log:
        for timestamp, stream_id, data in log.read_gen(stream_id):
            print(timestamp, print_packet(deserialize(data), dbc))

# vim: expandtab sw=4 ts=4

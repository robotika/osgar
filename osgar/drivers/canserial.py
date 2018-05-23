"""
  Wrapper for CAN-serial communication and control of CAN bridge
"""

import serial
import struct
from threading import Thread

from osgar.logger import LogWriter, LogReader
from osgar.bus import BusShutdownException


CAN_BRIDGE_READY = b'\xfe\x10'  # CAN bridge is ready to accept configuration commands
CAN_BRIDGE_SYNC = b'\xFF'*10    # CAN bridge synchronization bytes
CAN_SPEED_1MB = b'\xfe\x57'     # configure CAN bridge to communicate on 1Mb CAN network
CAN_BRIDGE_START = b'\xfe\x31'  # start bridge


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
            self.bus.publish('raw', CAN_SPEED_1MB)
            self.bus.publish('raw', CAN_BRIDGE_START)
            self.can_bridge_initialized = True
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
        self.bus.shutdown()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='Parse CAN stream messages')
    parser.add_argument('logfile', help='filename of stored file')
    parser.add_argument('--stream', help='stream ID', type=int, required=True)
    parser.add_argument('--times', help='display timestamps', action='store_true')
    args = parser.parse_args()

    dbc = {
            0x200: 'H',     # status
            0x201: 'HHHH',  # wheels
            0x202: 'HHHH',  # drive status
            0x203: 'HHHH',  # zero steering
            0x204: 'HHBBH'  # user input
        }

    with LogReader(args.logfile) as log:
        buf = b''
        for timestamp, stream_id, data in log.read_gen(args.stream):
            if args.times:
                buf, packet = CANSerial.split_buffer(buf + data)
                while len(packet) > 0:
                    print(timestamp, stream_id, print_packet(packet, dbc))
                    buf, packet = CANSerial.split_buffer(buf)
            else:
                sys.stdout.buffer.write(data)


# vim: expandtab sw=4 ts=4
